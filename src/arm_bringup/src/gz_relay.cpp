// Relay Gazebo's clock and movable-part poses to ROS at bounded rates.
//
// Clock: gz-sim publishes its clock every physics step (1 kHz here), and
// ros_gz_bridge forwards every message. Each use_sim_time node then wakes for
// each one; across a dozen processes that fan-out costs more CPU than the
// clock is worth and delays the simulation thread's wake-ups. gz-transport's
// subscriber-side throttle keeps the newest sample at `clock_rate` Hz instead.
// controller_manager is the exception: it times each update by its node clock
// rather than the simulation time gz_ros2_control hands it, so at 200 Hz its
// trajectory samples and joint-state stamps fell up to several milliseconds
// off the 10 ms updates. It gets every step on /controller_manager/clock,
// which the robot's gz_ros2_control remaps /clock to.
//
// Parts: SceneBroadcaster already publishes every dynamic entity's pose, with
// names, on /world/<name>/dynamic_pose/info. Reading it here avoids a
// PosePublisher system per part, and gz-sim runs each system's PostUpdate on
// its own thread joined by barriers every step, so systems are not free.
// ros_gz_bridge would drop the names when converting Pose_V. Each listed
// model's world pose is published as /parts/<model>/pose.
//
// Grasp welds: requests on the ROS topic /grasp_weld (std_msgs/String) are
// forwarded to the arm_bringup::GraspWeld world system's Gazebo topic.
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {

gz::transport::SubscribeOptions throttled(double rate)
{
  gz::transport::SubscribeOptions options;
  if (rate > 0.0) {
    options.SetMsgsPerSec(static_cast<uint64_t>(rate));
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gz_relay");
  const auto world = node->declare_parameter<std::string>("world", "default");
  const auto clock_rate = node->declare_parameter<double>("clock_rate", 200.0);
  const auto pose_rate = node->declare_parameter<double>("pose_rate", 30.0);
  const auto parts = node->declare_parameter<std::vector<std::string>>("parts", std::vector<std::string>{});

  gz::transport::Node gz_node, gz_control_clock_node;
  auto relay_to = [&node](const std::string & topic) {
      auto publisher = node->create_publisher<rosgraph_msgs::msg::Clock>(topic, rclcpp::ClockQoS());
      return std::function<void(const gz::msgs::Clock &)>([publisher](const gz::msgs::Clock & clock) {
          rosgraph_msgs::msg::Clock message;
          message.clock.sec = static_cast<int32_t>(clock.sim().sec());
          message.clock.nanosec = static_cast<uint32_t>(clock.sim().nsec());
          publisher->publish(message);
        });
    };
  const std::string clock_topic = "/world/" + world + "/clock";
  if (!gz_node.Subscribe(clock_topic, relay_to("/clock"), throttled(clock_rate)) ||
      !gz_control_clock_node.Subscribe(clock_topic, relay_to("/controller_manager/clock"))) {
    RCLCPP_FATAL(node->get_logger(), "Cannot subscribe to Gazebo topic %s", clock_topic.c_str());
    rclcpp::shutdown();
    return 1;
  }

  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> part_pubs;
  for (const auto & part : parts) {
    if (part.empty()) {
      continue;  // a launch passes [""] for "no parts": ROS cannot type an empty list
    }
    part_pubs[part] = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/parts/" + part + "/pose", rclcpp::SensorDataQoS());
  }
  std::function<void(const gz::msgs::Pose_V &)> relay_poses = [&part_pubs](const gz::msgs::Pose_V & poses) {
      for (const auto & pose : poses.pose()) {
        // Model poses are relative to the world; links are relative to their
        // model and share names across models, so only model names match.
        const auto found = part_pubs.find(pose.name());
        if (found == part_pubs.end()) {
          continue;
        }
        geometry_msgs::msg::PoseStamped message;
        message.header.stamp.sec = static_cast<int32_t>(poses.header().stamp().sec());
        message.header.stamp.nanosec = static_cast<uint32_t>(poses.header().stamp().nsec());
        message.header.frame_id = "world";
        message.pose.position.x = pose.position().x();
        message.pose.position.y = pose.position().y();
        message.pose.position.z = pose.position().z();
        message.pose.orientation.x = pose.orientation().x();
        message.pose.orientation.y = pose.orientation().y();
        message.pose.orientation.z = pose.orientation().z();
        message.pose.orientation.w = pose.orientation().w();
        found->second->publish(message);
      }
    };
  const std::string pose_topic = "/world/" + world + "/dynamic_pose/info";
  if (!part_pubs.empty() && !gz_node.Subscribe(pose_topic, relay_poses, throttled(pose_rate))) {
    RCLCPP_FATAL(node->get_logger(), "Cannot subscribe to Gazebo topic %s", pose_topic.c_str());
    rclcpp::shutdown();
    return 1;
  }
  auto weld_pub = gz_node.Advertise<gz::msgs::StringMsg>("/grasp_weld");
  auto weld_sub = node->create_subscription<std_msgs::msg::String>(
    "/grasp_weld", rclcpp::QoS(10).reliable(), [&weld_pub](const std_msgs::msg::String & request) {
      gz::msgs::StringMsg message;
      message.set_data(request.data);
      weld_pub.Publish(message);
    });
  RCLCPP_INFO(node->get_logger(), "Relaying %s at up to %.0f Hz and %zu part poses at up to %.0f Hz",
              clock_topic.c_str(), clock_rate, part_pubs.size(), pose_rate);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
