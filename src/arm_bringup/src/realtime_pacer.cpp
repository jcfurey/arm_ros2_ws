// Real-time pacing with catch-up for gz-sim worlds.
//
// gz-sim's own throttle sleeps toward the next step deadline and, when a step
// wakes late, re-anchors the schedule at the late wake-up. Every late wake-up
// therefore loses sim time for good. Under container load, 1 ms steps wake
// past gz-sim's 200 us spin window often enough to hold the real-time factor
// near 0.8 while the simulation thread is mostly idle.
//
// Load this system with <real_time_factor>0</real_time_factor> in the world's
// <physics> (gz-sim then steps without sleeping) and it paces each step
// against an absolute wall-clock schedule instead. A late wake-up is repaid by
// the following steps, which run without sleeping until the schedule is met,
// so the average rate holds the target. Lag beyond <max_lag> (default
// 0.25 s) is forgiven rather than repaid in a burst. It must outlast the
// stalls that load causes: on a 4-vCPU host the workcell steps at only ~1.09x
// real time flat out, a 50 ms allowance was exceeded often enough to hold the
// factor at 0.96, and 0.25 s holds 1.00. Physics, step size and every
// sim-time output are unchanged; only when steps run in wall-clock time
// differs.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>

#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

namespace arm_bringup {

class RealtimePacer : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    factor_ = std::max(1e-3, sdf->Get<double>("real_time_factor", 1.0).first);
    max_lag_ = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(std::max(0.0, sdf->Get<double>("max_lag", 0.25).first)));
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager &) override
  {
    if (info.paused) {
      anchored_ = false;
      return;
    }
    const auto now = Clock::now();
    const bool rewound = info.simTime < last_sim_;
    last_sim_ = info.simTime;
    if (!anchored_ || rewound) {
      // First step, after a pause, or after a world reset rewound sim time.
      anchor_wall_ = now;
      anchor_sim_ = info.simTime;
      anchored_ = true;
      return;
    }
    const auto due = anchor_wall_ + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(info.simTime - anchor_sim_) / factor_);
    if (now > due + max_lag_) {
      // The simulation cannot keep up; forgive the excess instead of
      // fast-forwarding through it once load drops.
      anchor_wall_ += now - (due + max_lag_);
    } else if (due > now) {
      std::this_thread::sleep_until(due);
    }
  }

private:
  using Clock = std::chrono::steady_clock;
  double factor_ = 1.0;
  Clock::duration max_lag_{};
  bool anchored_ = false;
  Clock::time_point anchor_wall_{};
  std::chrono::steady_clock::duration anchor_sim_{};
  std::chrono::steady_clock::duration last_sim_{};
};

}  // namespace arm_bringup

GZ_ADD_PLUGIN(arm_bringup::RealtimePacer, gz::sim::System,
              arm_bringup::RealtimePacer::ISystemConfigure,
              arm_bringup::RealtimePacer::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(arm_bringup::RealtimePacer, "arm_bringup::RealtimePacer")
