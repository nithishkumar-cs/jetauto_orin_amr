#include <amr_interfaces/msg/detection2_d_array.hpp>
#include <amr_interfaces/msg/node_health.hpp>
#include <amr_interfaces/msg/obstacle.hpp>
#include <amr_interfaces/msg/obstacle_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace perception_geometry
{
namespace
{

double seconds_since(const rclcpp::Time& now, const rclcpp::Time& then)
{
  if (then.nanoseconds() == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return (now - then).seconds();
}

}  // namespace

class DepthObstacleProjectorNode final : public rclcpp::Node
{
public:
  explicit DepthObstacleProjectorNode(const rclcpp::NodeOptions& options)
  : Node("depth_obstacle_projector_node", options),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.15);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 8.0);
    sample_fraction_ = declare_parameter<double>("sample_fraction", 0.30);
    stale_depth_timeout_s_ = declare_parameter<double>("stale_depth_timeout_s", 0.25);
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.03);

    const auto detections_topic = declare_parameter<std::string>("detections_topic", "/perception/detections_2d");
    const auto depth_topic = declare_parameter<std::string>("depth_topic", "/sensors/depth/image_rect");
    const auto camera_info_topic = declare_parameter<std::string>("camera_info_topic", "/sensors/depth/camera_info");
    const auto obstacles_topic = declare_parameter<std::string>("obstacles_topic", "/perception/obstacles_3d");

    obstacle_pub_ = create_publisher<amr_interfaces::msg::ObstacleArray>(obstacles_topic, 10);
    health_pub_ = create_publisher<amr_interfaces::msg::NodeHealth>("/diagnostics/geometry/health", 10);

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&DepthObstacleProjectorNode::depth_callback, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&DepthObstacleProjectorNode::info_callback, this, std::placeholders::_1));
    detections_sub_ = create_subscription<amr_interfaces::msg::Detection2DArray>(
      detections_topic,
      10,
      std::bind(&DepthObstacleProjectorNode::detections_callback, this, std::placeholders::_1));
  }

private:
  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_depth_ = msg;
    latest_depth_receive_time_ = now();
  }

  void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_info_ = msg;
  }

  void detections_callback(const amr_interfaces::msg::Detection2DArray::ConstSharedPtr msg)
  {
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    rclcpp::Time depth_receive_time;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      depth = latest_depth_;
      info = latest_info_;
      depth_receive_time = latest_depth_receive_time_;
    }

    if (!depth || !info) {
      publish_health(amr_interfaces::msg::NodeHealth::WARN, "waiting for depth image and camera info");
      return;
    }

    if (seconds_since(now(), depth_receive_time) > stale_depth_timeout_s_) {
      publish_health(amr_interfaces::msg::NodeHealth::STALE, "depth image is stale");
      return;
    }

    amr_interfaces::msg::ObstacleArray obstacles;
    obstacles.header.stamp = msg->header.stamp;
    obstacles.header.frame_id = target_frame_.empty() ? depth->header.frame_id : target_frame_;

    for (const auto& detection : msg->detections) {
      const auto depth_m = median_depth_for_detection(*depth, detection);
      if (!std::isfinite(depth_m)) {
        continue;
      }

      const double fx = info->k[0];
      const double fy = info->k[4];
      const double cx = info->k[2];
      const double cy = info->k[5];
      if (fx <= 0.0 || fy <= 0.0) {
        continue;
      }

      geometry_msgs::msg::PointStamped point_in;
      point_in.header = depth->header;
      point_in.point.z = depth_m;
      point_in.point.x = ((static_cast<double>(detection.center_x) - cx) / fx) * depth_m;
      point_in.point.y = ((static_cast<double>(detection.center_y) - cy) / fy) * depth_m;

      geometry_msgs::msg::PointStamped point_out = point_in;
      if (!target_frame_.empty() && target_frame_ != point_in.header.frame_id) {
        try {
          point_out = tf_buffer_->transform(point_in, target_frame_, tf2::durationFromSec(tf_timeout_s_));
        } catch (const std::exception& ex) {
          publish_health(amr_interfaces::msg::NodeHealth::WARN, std::string("TF transform failed: ") + ex.what());
          continue;
        }
      }

      amr_interfaces::msg::Obstacle obstacle;
      obstacle.header = obstacles.header;
      obstacle.track_id = 0;
      obstacle.class_id = detection.class_id;
      obstacle.confidence = detection.score;
      obstacle.position = point_out.point;
      obstacle.size.x = static_cast<double>(detection.size_x) / fx * depth_m;
      obstacle.size.y = static_cast<double>(detection.size_y) / fy * depth_m;
      obstacle.size.z = 0.5;
      obstacle.dynamic = false;
      obstacle.source = msg->source_model;
      obstacles.obstacles.push_back(std::move(obstacle));
    }

    obstacle_pub_->publish(std::move(obstacles));
    publish_health(amr_interfaces::msg::NodeHealth::OK, "depth projection active");
  }

  float median_depth_for_detection(
    const sensor_msgs::msg::Image& depth,
    const amr_interfaces::msg::Detection2D& detection) const
  {
    const int width = static_cast<int>(depth.width);
    const int height = static_cast<int>(depth.height);
    if (width <= 0 || height <= 0) {
      return std::numeric_limits<float>::quiet_NaN();
    }

    const int sample_w = std::max(3, static_cast<int>(std::round(detection.size_x * sample_fraction_)));
    const int sample_h = std::max(3, static_cast<int>(std::round(detection.size_y * sample_fraction_)));
    const int cx = static_cast<int>(std::round(detection.center_x));
    const int cy = static_cast<int>(std::round(detection.center_y));
    const int x0 = std::clamp(cx - sample_w / 2, 0, width - 1);
    const int x1 = std::clamp(cx + sample_w / 2, 0, width - 1);
    const int y0 = std::clamp(cy - sample_h / 2, 0, height - 1);
    const int y1 = std::clamp(cy + sample_h / 2, 0, height - 1);

    samples_.clear();
    samples_.reserve(static_cast<std::size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));

    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const auto value = read_depth(depth, x, y);
        if (std::isfinite(value) && value >= min_depth_m_ && value <= max_depth_m_) {
          samples_.push_back(value);
        }
      }
    }

    if (samples_.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }

    const auto mid = samples_.begin() + static_cast<std::ptrdiff_t>(samples_.size() / 2);
    std::nth_element(samples_.begin(), mid, samples_.end());
    return *mid;
  }

  float read_depth(const sensor_msgs::msg::Image& depth, int x, int y) const
  {
    const std::size_t offset = static_cast<std::size_t>(y) * depth.step;

    if (depth.encoding == "16UC1") {
      const auto* row = reinterpret_cast<const uint16_t*>(depth.data.data() + offset);
      const uint16_t raw = row[x];
      if (raw == 0) {
        return std::numeric_limits<float>::quiet_NaN();
      }
      return static_cast<float>(static_cast<double>(raw) * depth_scale_);
    }

    if (depth.encoding == "32FC1") {
      const auto* row = reinterpret_cast<const float*>(depth.data.data() + offset);
      return row[x];
    }

    return std::numeric_limits<float>::quiet_NaN();
  }

  void publish_health(uint8_t status, const std::string& message)
  {
    amr_interfaces::msg::NodeHealth health;
    health.header.stamp = now();
    health.node_name = get_name();
    health.status = status;
    health.message = message;
    health_pub_->publish(std::move(health));
  }

  std::string target_frame_;
  double depth_scale_{0.001};
  double min_depth_m_{0.15};
  double max_depth_m_{8.0};
  double sample_fraction_{0.30};
  double stale_depth_timeout_s_{0.25};
  double tf_timeout_s_{0.03};

  mutable std::vector<float> samples_;
  std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;
  rclcpp::Time latest_depth_receive_time_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<amr_interfaces::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<amr_interfaces::msg::ObstacleArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<amr_interfaces::msg::NodeHealth>::SharedPtr health_pub_;
};

}  // namespace perception_geometry

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_geometry::DepthObstacleProjectorNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
