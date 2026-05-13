#include <amr_interfaces/msg/node_health.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace sensor_drivers
{

class UsbCameraNode final : public rclcpp::Node
{
public:
  explicit UsbCameraNode(const rclcpp::NodeOptions& options)
  : Node("rgb_camera_node", options)
  {
    device_ = declare_parameter<std::string>("device", "/dev/video0");
    frame_id_ = declare_parameter<std::string>("frame_id", "rgb_camera_optical_frame");
    output_encoding_ = declare_parameter<std::string>("output_encoding", "bgr8");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 720);
    fps_ = declare_parameter<double>("fps", 30.0);
    use_mjpeg_ = declare_parameter<bool>("use_mjpeg", true);
    reconnect_period_s_ = declare_parameter<double>("reconnect_period_s", 1.0);
    info_.distortion_model = declare_parameter<std::string>("distortion_model", "plumb_bob");

    const auto d = declare_parameter<std::vector<double>>("d", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
    const auto k = declare_parameter<std::vector<double>>(
      "k",
      std::vector<double>{
        static_cast<double>(width_), 0.0, static_cast<double>(width_) * 0.5,
        0.0, static_cast<double>(width_), static_cast<double>(height_) * 0.5,
        0.0, 0.0, 1.0});
    const auto r = declare_parameter<std::vector<double>>(
      "r",
      std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    const auto p = declare_parameter<std::vector<double>>(
      "p",
      std::vector<double>{
        static_cast<double>(width_), 0.0, static_cast<double>(width_) * 0.5, 0.0,
        0.0, static_cast<double>(width_), static_cast<double>(height_) * 0.5, 0.0,
        0.0, 0.0, 1.0, 0.0});

    info_.width = static_cast<uint32_t>(std::max(0, width_));
    info_.height = static_cast<uint32_t>(std::max(0, height_));
    info_.d = d;
    copy_fixed(k, info_.k);
    copy_fixed(r, info_.r);
    copy_fixed(p, info_.p);

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS());
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS());
    health_pub_ = create_publisher<amr_interfaces::msg::NodeHealth>("health", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, fps_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UsbCameraNode::capture_once, this));

    open_camera();
  }

private:
  template <std::size_t N>
  static void copy_fixed(const std::vector<double>& src, std::array<double, N>& dst)
  {
    for (std::size_t i = 0; i < N; ++i) {
      dst[i] = i < src.size() ? src[i] : 0.0;
    }
  }

  void open_camera()
  {
    capture_.release();

    if (!capture_.open(device_, cv::CAP_V4L2)) {
      publish_health(amr_interfaces::msg::NodeHealth::ERROR, "camera open failed");
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to open %s", device_.c_str());
      last_open_attempt_ = now();
      return;
    }

    capture_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
    capture_.set(cv::CAP_PROP_FPS, fps_);

    if (use_mjpeg_) {
      capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    }

    publish_health(amr_interfaces::msg::NodeHealth::OK, "camera opened");
    RCLCPP_INFO(
      get_logger(),
      "Opened %s at requested %dx%d %.1f FPS",
      device_.c_str(),
      width_,
      height_,
      fps_);
  }

  void capture_once()
  {
    if (!capture_.isOpened()) {
      const auto elapsed = (now() - last_open_attempt_).seconds();
      if (elapsed >= reconnect_period_s_) {
        open_camera();
      }
      return;
    }

    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      publish_health(amr_interfaces::msg::NodeHealth::ERROR, "camera read failed");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read frame from %s", device_.c_str());
      capture_.release();
      last_open_attempt_ = now();
      return;
    }

    if (frame.channels() != 3) {
      cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    }

    if (output_encoding_ == "rgb8") {
      cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    } else if (output_encoding_ != "bgr8") {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Unsupported output_encoding=%s; publishing bgr8",
        output_encoding_.c_str());
      output_encoding_ = "bgr8";
    }

    if (!frame.isContinuous()) {
      frame = frame.clone();
    }

    const auto stamp = now();

    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = frame_id_;
    image.height = static_cast<uint32_t>(frame.rows);
    image.width = static_cast<uint32_t>(frame.cols);
    image.encoding = output_encoding_;
    image.is_bigendian = false;
    image.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.cols * frame.elemSize());
    image.data.assign(frame.datastart, frame.dataend);
    image_pub_->publish(std::move(image));

    info_.header.stamp = stamp;
    info_.header.frame_id = frame_id_;
    info_.width = static_cast<uint32_t>(frame.cols);
    info_.height = static_cast<uint32_t>(frame.rows);
    info_pub_->publish(info_);

    publish_health(amr_interfaces::msg::NodeHealth::OK, "camera streaming");
  }

  void publish_health(uint8_t status, const std::string& message)
  {
    amr_interfaces::msg::NodeHealth health;
    health.header.stamp = now();
    health.node_name = get_name();
    health.status = status;
    health.message = message;
    health.frequency_hz = static_cast<float>(fps_);
    health.age.sec = 0;
    health.age.nanosec = 0;
    health_pub_->publish(std::move(health));
  }

  std::string device_;
  std::string frame_id_;
  std::string output_encoding_;
  int width_{0};
  int height_{0};
  double fps_{30.0};
  bool use_mjpeg_{true};
  double reconnect_period_s_{1.0};

  cv::VideoCapture capture_;
  rclcpp::Time last_open_attempt_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::CameraInfo info_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<amr_interfaces::msg::NodeHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sensor_drivers

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_drivers::UsbCameraNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
