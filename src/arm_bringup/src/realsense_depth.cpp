// Turn Gazebo's ideal depth into what a RealSense D4xx and realsense-ros publish.
//
// Gazebo renders exact depth in float metres. A D4xx computes depth from
// stereo disparity, so its error grows with the square of the distance:
//
//   RMS error = z^2 * subpixel / (focal_px * baseline)
//
// (Intel, "Tuning depth cameras for best performance", depth RMS error). This
// node adds that noise, rounds to the driver's 1 mm depth unit, marks pixels
// outside [min_depth, max_depth] invalid (0), and publishes 16UC1 millimetres
// as realsense-ros does on depth/image_rect_raw. The focal length comes from
// the image width and horizontal field of view.
//
// It reads Gazebo's depth topic itself rather than through ros_gz_bridge: a
// 640x360 float image bridged and then received again by this node cost ~5%
// of real time with both cameras streaming. The topic is subscribed only
// while the output has subscribers, so an unwatched camera is never rendered.
// The ideal depth stays available through the bridge as ground truth.
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// Draw standard normals from a fixed table with a xorshift index: a frame has
// ~230k pixels, and std::normal_distribution would cost several ms per frame.
class NormalTable {
public:
  explicit NormalTable(uint64_t seed) : table_(1 << 20), state_(seed | 1)
  {
    std::mt19937_64 engine(seed);
    std::normal_distribution<float> normal(0.0f, 1.0f);
    for (auto & value : table_) {
      value = normal(engine);
    }
  }

  float next()
  {
    state_ ^= state_ << 13;
    state_ ^= state_ >> 7;
    state_ ^= state_ << 17;
    return table_[state_ & (table_.size() - 1)];
  }

private:
  std::vector<float> table_;
  uint64_t state_;
};

}  // namespace

class RealsenseDepth : public rclcpp::Node {
public:
  RealsenseDepth()
  : Node("realsense_depth"),
    gz_topic_(declare_parameter<std::string>("gz_topic", "")),
    baseline_(declare_parameter<double>("baseline", 0.050)),
    subpixel_(declare_parameter<double>("subpixel", 0.08)),
    hfov_(declare_parameter<double>("horizontal_fov", 1.5184)),
    min_depth_(declare_parameter<double>("min_depth", 0.17)),
    max_depth_(declare_parameter<double>("max_depth", 10.0)),
    noise_(declare_parameter<bool>("noise", true)),
    normals_(static_cast<uint64_t>(declare_parameter<int>("seed", 1)))
  {
    if (gz_topic_.empty()) {
      throw std::invalid_argument("realsense_depth needs the Gazebo depth image topic (gz_topic)");
    }
    // Reliable: a best-effort image spans several UDP fragments and is lost
    // whole when one is (see docker/cyclonedds.xml).
    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("output", "depth/image_rect_raw"), rclcpp::QoS(5));
    // Follow the output's subscribers, so the camera renders only when watched.
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { follow_subscribers(); });
  }

private:
  void follow_subscribers()
  {
    const bool watched = publisher_->get_subscription_count() + publisher_->get_intra_process_subscription_count() > 0;
    if (watched && !subscribed_) {
      subscribed_ = gz_node_.Subscribe(gz_topic_, &RealsenseDepth::convert, this);
      if (!subscribed_) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot subscribe to %s", gz_topic_.c_str());
      }
    } else if (!watched && subscribed_) {
      gz_node_.Unsubscribe(gz_topic_);
      subscribed_ = false;
    }
  }

  // Runs on a gz-transport thread.
  void convert(const gz::msgs::Image & input)
  {
    if (input.pixel_format_type() != gz::msgs::PixelFormatType::R_FLOAT32) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Expected R_FLOAT32 depth from %s", gz_topic_.c_str());
      return;
    }
    const uint32_t width = input.width(), height = input.height();
    const size_t step = input.step() ? input.step() : width * sizeof(float);
    if (input.data().size() < step * height) {
      return;
    }
    auto output = std::make_unique<sensor_msgs::msg::Image>();
    output->header.stamp.sec = static_cast<int32_t>(input.header().stamp().sec());
    output->header.stamp.nanosec = static_cast<uint32_t>(input.header().stamp().nsec());
    for (const auto & entry : input.header().data()) {
      if (entry.key() == "frame_id" && entry.value_size() > 0) {
        output->header.frame_id = entry.value(0);
      }
    }
    output->height = height;
    output->width = width;
    output->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    output->is_bigendian = false;
    output->step = width * sizeof(uint16_t);
    output->data.resize(static_cast<size_t>(output->step) * height);
    // z^2 * subpixel / (f * B), with f in pixels from the image width and field of view.
    const double focal = 0.5 * width / std::tan(0.5 * hfov_);
    const float scale = noise_ ? static_cast<float>(subpixel_ / (focal * baseline_)) : 0.0f;
    const auto * in_data = reinterpret_cast<const uint8_t *>(input.data().data());
    std::lock_guard<std::mutex> lock(normals_mutex_);
    for (uint32_t row = 0; row < height; ++row) {
      const auto * in = in_data + row * step;
      auto * out = reinterpret_cast<uint16_t *>(output->data.data() + static_cast<size_t>(row) * output->step);
      for (uint32_t column = 0; column < width; ++column) {
        float z;
        std::memcpy(&z, in + column * sizeof(float), sizeof(float));
        if (!std::isfinite(z) || z < min_depth_ || z > max_depth_) {
          out[column] = 0;  // the driver's "no depth"
          continue;
        }
        const float noisy = z + scale * z * z * normals_.next();
        out[column] = static_cast<uint16_t>(std::lround(std::clamp(noisy * 1000.0f, 1.0f, 65535.0f)));
      }
    }
    publisher_->publish(std::move(output));
  }

  std::string gz_topic_;
  double baseline_, subpixel_, hfov_, min_depth_, max_depth_;
  bool noise_;
  NormalTable normals_;
  std::mutex normals_mutex_;
  gz::transport::Node gz_node_;
  bool subscribed_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealsenseDepth>());
  rclcpp::shutdown();
  return 0;
}
