#include <amr_interfaces/msg/node_health.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace tf_and_calibration
{

class CalibrationValidatorNode final : public rclcpp::Node
{
public:
  explicit CalibrationValidatorNode(const rclcpp::NodeOptions& options)
  : Node("calibration_validator_node", options),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "rgb_camera_optical_frame");
    min_focal_px_ = declare_parameter<double>("min_focal_px", 100.0);
    max_focal_px_ = declare_parameter<double>("max_focal_px", 3000.0);
    stale_camera_info_s_ = declare_parameter<double>("stale_camera_info_s", 1.0);

    const auto camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic",
      "/sensors/rgb/camera_info");

    health_pub_ = create_publisher<amr_interfaces::msg::NodeHealth>("/diagnostics/calibration/health", 10);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&CalibrationValidatorNode::info_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CalibrationValidatorNode::validate, this));
  }

private:
  void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    latest_info_ = msg;
    latest_info_time_ = now();
  }

  void validate()
  {
    if (!latest_info_) {
      publish_health(amr_interfaces::msg::NodeHealth::WARN, "waiting for camera info");
      return;
    }

    if ((now() - latest_info_time_).seconds() > stale_camera_info_s_) {
      publish_health(amr_interfaces::msg::NodeHealth::STALE, "camera info is stale");
      return;
    }

    const double fx = latest_info_->k[0];
    const double fy = latest_info_->k[4];
    if (fx < min_focal_px_ || fy < min_focal_px_ || fx > max_focal_px_ || fy > max_focal_px_) {
      publish_health(amr_interfaces::msg::NodeHealth::ERROR, "camera intrinsics out of expected range");
      return;
    }

    try {
      (void)tf_buffer_->lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);
    } catch (const std::exception& ex) {
      publish_health(amr_interfaces::msg::NodeHealth::WARN, std::string("missing camera TF: ") + ex.what());
      return;
    }

    publish_health(amr_interfaces::msg::NodeHealth::OK, "camera calibration and TF validated");
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

  std::string base_frame_;
  std::string camera_frame_;
  double min_focal_px_{100.0};
  double max_focal_px_{3000.0};
  double stale_camera_info_s_{1.0};

  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;
  rclcpp::Time latest_info_time_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<amr_interfaces::msg::NodeHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tf_and_calibration

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_and_calibration::CalibrationValidatorNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
