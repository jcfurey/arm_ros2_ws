// An IMU system that measures motion from the sensor's pose, for Bullet Featherstone.
//
// gz-sim's Imu system takes each IMU's angular velocity and linear
// acceleration from components the Physics system fills from the engine's
// link frame data. gz-physics 9's Bullet Featherstone plugin reports zero
// velocity for every non-base link (it reads btMultibodyLink's
// m_absFrameTotVelocity, which Bullet fills only in its global-velocity mode;
// fixed upstream after gz-physics 9 in gazebosim/gz-physics#1112) and never
// computes link accelerations. An IMU on the arm therefore reads only noise
// and gravity's direction.
//
// This system takes the Imu system's place for the IMUs of the model it is
// loaded in. Each step, after physics, it takes the sensor's world pose from
// the link poses, which the engine reports correctly, and derives angular
// velocity from the rotation over the step and acceleration from the change
// in the sensor point's velocity. gz-sensors' ImuSensor then adds gravity and
// the SDF noise and publishes, as under the Imu system. The sensor's
// WorldPose, AngularVelocity and LinearAcceleration components are removed,
// since Physics would query the engine for each of them every step.
//
// gz_ros2_control holds each joint's velocity command for a controller period,
// so the simulated arm's velocity changes in steps at the controller rate,
// which a real arm's servos and inertia smooth out. Acceleration is the
// velocity change over that period, which removes the steps, and both signals
// then pass a first-order low-pass like a MEMS sensor's output filter.
// Orientation, when enabled, is relative to the sensor's starting orientation.
//
// Parameters:
//   <control_period>   seconds over which acceleration is averaged: the arm's
//                      controller period (default 0.01, ros2_controllers.yaml)
//   <accel_cutoff_hz>  accelerometer low-pass (default 31: the BMI055 at the
//                      D435i's 62.5 Hz accelerometer rate)
//   <gyro_cutoff_hz>   gyroscope low-pass (default 64)
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/ImuSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace arm_bringup {

class PoseImu : public gz::sim::System,
                public gz::sim::ISystemConfigurePriority,
                public gz::sim::ISystemConfigure,
                public gz::sim::ISystemPreUpdate,
                public gz::sim::ISystemUpdate {
public:
  // After Physics moves the links. gz-sim's Physics implements
  // ConfigurePriority but does not register it with its plugin, so it runs at
  // the default priority rather than kPhysicsPriority.
  PriorityType ConfigurePriority() override { return gz::sim::System::kDefaultPriority + 1; }

  void Configure(const gz::sim::Entity & model, const std::shared_ptr<const sdf::Element> & sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    model_ = model;
    control_period_ = sdf->Get<double>("control_period", 0.01).first;
    accel_cutoff_ = sdf->Get<double>("accel_cutoff_hz", 31.0).first;
    gyro_cutoff_ = sdf->Get<double>("gyro_cutoff_hz", 64.0).first;
  }

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & ecm) override
  {
    // The model's entities predate this system's first step.
    std::vector<gz::sim::Entity> added;
    auto collect = [&](const gz::sim::Entity & entity, const gz::sim::components::Imu *) -> bool
    {
      added.push_back(entity);
      return true;
    };
    if (!initialized_) {
      ecm.Each<gz::sim::components::Imu>(collect);
      initialized_ = true;
    } else if (ecm.HasNewEntities()) {
      ecm.EachNew<gz::sim::components::Imu>(collect);
    }
    for (const auto entity : added) {
      if (gz::sim::topLevelModel(entity, ecm) == model_) {
        AddSensor(entity, ecm);
      }
    }
    if (ecm.HasEntitiesMarkedForRemoval()) {
      ecm.EachRemoved<gz::sim::components::Imu>(
        [&](const gz::sim::Entity & entity, const gz::sim::components::Imu *) -> bool
        {
          imus_.erase(entity);
          return true;
        });
    }
  }

  void Update(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (info.paused || dt <= 0.0) {
      return;
    }
    const auto window = static_cast<size_t>(std::max(1.0, std::round(control_period_ / dt)));
    const double accel_alpha = dt / (dt + 1.0 / (2.0 * M_PI * accel_cutoff_));
    const double gyro_alpha = dt / (dt + 1.0 / (2.0 * M_PI * gyro_cutoff_));
    for (auto & [entity, imu] : imus_) {
      const auto pose = gz::sim::worldPose(entity, ecm);
      auto & motion = imu.motion;
      // A first step, a reset (time went back), a teleport, or a non-finite pose restarts the history.
      if (!motion.pose || info.simTime <= motion.time || !((pose.Pos() - motion.pose->Pos()).Length() <= 1.0)) {
        if (motion.pose && info.simTime <= motion.time) {
          imu.sensor->SetNextDataUpdateTime(info.simTime);
        }
        motion = Motion{pose, {}, {}, {}, info.simTime};
        continue;
      }
      // World-frame angular velocity over the last step, from the relative
      // rotation. atan2 rather than AxisAngle's acos(w): rounding puts w of a
      // tiny rotation just above 1, where acos is NaN.
      auto delta = pose.Rot() * motion.pose->Rot().Inverse();
      if (delta.W() < 0.0) {
        delta = gz::math::Quaterniond(-delta.W(), -delta.X(), -delta.Y(), -delta.Z());
      }
      const gz::math::Vector3d half_axis(delta.X(), delta.Y(), delta.Z());
      const double sine = half_axis.Length();
      const double scale = sine > 1e-12 ? 2.0 * std::atan2(sine, delta.W()) / sine : 2.0 / delta.W();
      motion.omega += gyro_alpha * (half_axis * (scale / dt) - motion.omega);
      // The sensor point's velocity over the last step, kept for one control period.
      motion.velocities.push_back((pose.Pos() - motion.pose->Pos()) / dt);
      if (motion.velocities.size() > window + 1) {
        motion.velocities.pop_front();
      }
      const size_t span = motion.velocities.size() - 1;
      if (span > 0) {
        const auto accel = (motion.velocities.back() - motion.velocities.front()) / (span * dt);
        motion.accel += accel_alpha * (accel - motion.accel);
      }
      motion.pose = pose;
      motion.time = info.simTime;
      if (imu.sensor->NextDataUpdateTime() > info.simTime) {
        continue;
      }
      if (imu.sensor->HasConnections()) {
        imu.sensor->SetWorldPose(pose);
        imu.sensor->SetAngularVelocity(pose.Rot().RotateVectorReverse(motion.omega));
        imu.sensor->SetLinearAcceleration(pose.Rot().RotateVectorReverse(motion.accel));
        imu.sensor->Update(info.simTime, false);
      } else if (imu.sensor->UpdateRate() > 0.0) {
        // Unwatched: look for subscribers again a period later, not every step.
        imu.sensor->SetNextDataUpdateTime(info.simTime + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / imu.sensor->UpdateRate())));
      }
    }
  }

private:
  struct Motion {
    std::optional<gz::math::Pose3d> pose;
    std::deque<gz::math::Vector3d> velocities;
    gz::math::Vector3d omega, accel;
    std::chrono::steady_clock::duration time{};
  };

  struct Tracked {
    std::unique_ptr<gz::sensors::ImuSensor> sensor;
    Motion motion;
  };

  // As gz-sim's Imu system creates its sensors.
  void AddSensor(const gz::sim::Entity entity, gz::sim::EntityComponentManager & ecm)
  {
    sdf::Sensor data = ecm.Component<gz::sim::components::Imu>(entity)->Data();
    data.SetName(gz::sim::removeParentScope(gz::sim::scopedName(entity, ecm, "::", false), "::"));
    if (data.Topic().empty()) {
      data.SetTopic(gz::sim::scopedName(entity, ecm) + "/imu");
    }
    auto sensor = factory_.CreateSensor<gz::sensors::ImuSensor>(data);
    if (!sensor) {
      gzerr << "Failed to create IMU [" << data.Name() << "]" << std::endl;
      return;
    }
    if (auto * parent = ecm.Component<gz::sim::components::ParentEntity>(entity)) {
      if (auto * name = ecm.Component<gz::sim::components::Name>(parent->Data())) {
        sensor->SetParent(name->Data());
      }
    }
    const auto world = ecm.EntityByComponents(gz::sim::components::World());
    if (auto * gravity = ecm.Component<gz::sim::components::Gravity>(world)) {
      sensor->SetGravity(gravity->Data());
    }
    sensor->SetOrientationReference(gz::sim::worldPose(entity, ecm).Rot());
    if (data.ImuSensor()) {
      sensor->SetOrientationEnabled(data.ImuSensor()->OrientationEnabled());
    }
    ecm.CreateComponent(entity, gz::sim::components::SensorTopic(sensor->Topic()));
    ecm.RemoveComponent<gz::sim::components::WorldPose>(entity);
    ecm.RemoveComponent<gz::sim::components::AngularVelocity>(entity);
    ecm.RemoveComponent<gz::sim::components::LinearAcceleration>(entity);
    imus_[entity].sensor = std::move(sensor);
  }

  gz::sim::Entity model_ = gz::sim::kNullEntity;
  double control_period_ = 0.01, accel_cutoff_ = 31.0, gyro_cutoff_ = 64.0;
  bool initialized_ = false;
  gz::sensors::SensorFactory factory_;
  std::unordered_map<gz::sim::Entity, Tracked> imus_;
};

}  // namespace arm_bringup

GZ_ADD_PLUGIN(arm_bringup::PoseImu, gz::sim::System,
              arm_bringup::PoseImu::ISystemConfigurePriority,
              arm_bringup::PoseImu::ISystemConfigure,
              arm_bringup::PoseImu::ISystemPreUpdate,
              arm_bringup::PoseImu::ISystemUpdate)
GZ_ADD_PLUGIN_ALIAS(arm_bringup::PoseImu, "arm_bringup::PoseImu")
