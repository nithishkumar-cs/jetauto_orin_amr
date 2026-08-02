#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "perception_detection_depth_projection/detection_depth_projection_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace perception_detection_depth_projection
{
namespace
{

using namespace std::chrono_literals;

constexpr char kCameraFrame[] = "camera_rgb_optical_frame";
constexpr char kDetectionsInputTopic[] = "/test/projection/detections_2d";
constexpr char kDepthInputTopic[] = "/test/projection/depth";
constexpr char kCameraInfoInputTopic[] = "/test/projection/camera_info";
constexpr char kDetectionsOutputTopic[] = "/test/projection/detections_3d";

enum class PublishOrder {
  DETECTIONS_DEPTH_CAMERA_INFO,
  CAMERA_INFO_DETECTIONS_DEPTH,
  DEPTH_CAMERA_INFO_DETECTIONS,
};

struct InputMessages
{
  vision_msgs::msg::Detection2DArray detections;
  sensor_msgs::msg::Image depth;
  sensor_msgs::msg::CameraInfo camera_info;
};

class DetectionDepthProjectionComponentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("detections_in_topic", kDetectionsInputTopic),
       rclcpp::Parameter("depth_in_topic", kDepthInputTopic),
       rclcpp::Parameter("camera_info_in_topic", kCameraInfoInputTopic),
       rclcpp::Parameter("detections_out_topic", kDetectionsOutputTopic),
       rclcpp::Parameter("sync_queue_size", 10), rclcpp::Parameter("sync_max_interval_ms", 50)});
    projection_node_ = std::make_shared<DetectionDepthProjectionNode>(options);
    io_node_ = std::make_shared<rclcpp::Node>("detection_depth_projection_test_io");

    detections_publisher_ = io_node_->create_publisher<vision_msgs::msg::Detection2DArray>(
      kDetectionsInputTopic, rclcpp::SensorDataQoS());
    depth_publisher_ = io_node_->create_publisher<sensor_msgs::msg::Image>(
      kDepthInputTopic, rclcpp::SensorDataQoS());
    camera_info_publisher_ = io_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      kCameraInfoInputTopic, rclcpp::SensorDataQoS());
    output_subscription_ = io_node_->create_subscription<vision_msgs::msg::Detection3DArray>(
      kDetectionsOutputTopic, 10, [this](vision_msgs::msg::Detection3DArray::SharedPtr message) {
        std::scoped_lock lock(outputs_mutex_);
        outputs_.push_back(*message);
      });

    executor_.add_node(projection_node_);
    executor_.add_node(io_node_);
    ASSERT_TRUE(wait_for_connections());
  }

  void TearDown() override
  {
    executor_.remove_node(io_node_);
    executor_.remove_node(projection_node_);
    io_node_.reset();
    projection_node_.reset();
  }

  bool wait_for_connections()
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (
        detections_publisher_->get_subscription_count() > 0U &&
        depth_publisher_->get_subscription_count() > 0U &&
        camera_info_publisher_->get_subscription_count() > 0U &&
        projection_node_->count_subscribers(kDetectionsOutputTopic) > 0U) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  bool wait_for_output_count(std::size_t expected_count)
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (output_count() >= expected_count) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  std::size_t output_count()
  {
    std::scoped_lock lock(outputs_mutex_);
    return outputs_.size();
  }

  vision_msgs::msg::Detection3DArray output_at(std::size_t index)
  {
    std::scoped_lock lock(outputs_mutex_);
    return outputs_.at(index);
  }

  InputMessages make_inputs(const rclcpp::Time & stamp, const std::string & encoding = "16UC1")
  {
    InputMessages inputs;
    inputs.detections.header.stamp = stamp;
    inputs.detections.header.frame_id = kCameraFrame;
    auto & detection = inputs.detections.detections.emplace_back();
    detection.id = "detection-1";
    detection.bbox.center.position.x = 5.0;
    detection.bbox.center.position.y = 5.0;
    detection.bbox.size_x = 4.0;
    detection.bbox.size_y = 2.0;
    auto & primary_result = detection.results.emplace_back();
    primary_result.hypothesis.class_id = "person";
    primary_result.hypothesis.score = 0.9;
    auto & alternate_result = detection.results.emplace_back();
    alternate_result.hypothesis.class_id = "mannequin";
    alternate_result.hypothesis.score = 0.1;

    inputs.depth.header.stamp = stamp;
    inputs.depth.header.frame_id = kCameraFrame;
    inputs.depth.width = 10U;
    inputs.depth.height = 10U;
    inputs.depth.encoding = encoding;
    inputs.depth.is_bigendian = false;
    if (encoding == "16UC1") {
      inputs.depth.step = inputs.depth.width * sizeof(std::uint16_t);
      inputs.depth.data.resize(static_cast<std::size_t>(inputs.depth.step) * inputs.depth.height);
      for (std::size_t offset = 0U; offset < inputs.depth.data.size(); offset += 2U) {
        inputs.depth.data[offset] = 0xD0U;
        inputs.depth.data[offset + 1U] = 0x07U;
      }
    } else if (encoding == "32FC1") {
      inputs.depth.step = inputs.depth.width * sizeof(float);
      inputs.depth.data.resize(static_cast<std::size_t>(inputs.depth.step) * inputs.depth.height);
      for (std::size_t offset = 0U; offset < inputs.depth.data.size(); offset += 4U) {
        inputs.depth.data[offset] = 0x00U;
        inputs.depth.data[offset + 1U] = 0x00U;
        inputs.depth.data[offset + 2U] = 0x00U;
        inputs.depth.data[offset + 3U] = 0x40U;
      }
    } else {
      inputs.depth.step = inputs.depth.width;
      inputs.depth.data.resize(static_cast<std::size_t>(inputs.depth.step) * inputs.depth.height);
    }

    inputs.camera_info.header.stamp = stamp;
    inputs.camera_info.header.frame_id = kCameraFrame;
    inputs.camera_info.width = inputs.depth.width;
    inputs.camera_info.height = inputs.depth.height;
    inputs.camera_info.k[0] = 100.0;
    inputs.camera_info.k[2] = 5.0;
    inputs.camera_info.k[4] = 100.0;
    inputs.camera_info.k[5] = 5.0;
    return inputs;
  }

  void publish_inputs(
    const InputMessages & inputs, PublishOrder order = PublishOrder::DETECTIONS_DEPTH_CAMERA_INFO)
  {
    switch (order) {
      case PublishOrder::DETECTIONS_DEPTH_CAMERA_INFO:
        detections_publisher_->publish(inputs.detections);
        depth_publisher_->publish(inputs.depth);
        camera_info_publisher_->publish(inputs.camera_info);
        return;
      case PublishOrder::CAMERA_INFO_DETECTIONS_DEPTH:
        camera_info_publisher_->publish(inputs.camera_info);
        detections_publisher_->publish(inputs.detections);
        depth_publisher_->publish(inputs.depth);
        return;
      case PublishOrder::DEPTH_CAMERA_INFO_DETECTIONS:
        depth_publisher_->publish(inputs.depth);
        camera_info_publisher_->publish(inputs.camera_info);
        detections_publisher_->publish(inputs.detections);
        return;
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<DetectionDepthProjectionNode> projection_node_;
  rclcpp::Node::SharedPtr io_node_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr output_subscription_;
  std::mutex outputs_mutex_;
  std::vector<vision_msgs::msg::Detection3DArray> outputs_;
};

TEST_F(DetectionDepthProjectionComponentTest, PublishesCompleteDetectionOnOverriddenTopics)
{
  const auto input_stamp = io_node_->now();
  publish_inputs(make_inputs(input_stamp));
  ASSERT_TRUE(wait_for_output_count(1U));
  const auto output = output_at(0U);

  EXPECT_EQ(
    projection_node_->get_parameter("detections_in_topic").as_string(), kDetectionsInputTopic);
  EXPECT_EQ(projection_node_->get_parameter("depth_in_topic").as_string(), kDepthInputTopic);
  EXPECT_EQ(
    projection_node_->get_parameter("camera_info_in_topic").as_string(), kCameraInfoInputTopic);
  EXPECT_EQ(
    projection_node_->get_parameter("detections_out_topic").as_string(), kDetectionsOutputTopic);
  EXPECT_EQ(output.header.frame_id, kCameraFrame);
  EXPECT_EQ(rclcpp::Time(output.header.stamp), input_stamp);
  ASSERT_EQ(output.detections.size(), 1U);

  const auto & detection = output.detections.front();
  EXPECT_EQ(detection.id, "detection-1");
  ASSERT_EQ(detection.results.size(), 2U);
  EXPECT_EQ(detection.results[0].hypothesis.class_id, "person");
  EXPECT_DOUBLE_EQ(detection.results[0].hypothesis.score, 0.9);
  EXPECT_EQ(detection.results[1].hypothesis.class_id, "mannequin");
  EXPECT_DOUBLE_EQ(detection.results[1].hypothesis.score, 0.1);
  for (const auto & result : detection.results) {
    EXPECT_DOUBLE_EQ(result.pose.pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(result.pose.pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(result.pose.pose.position.z, 2.0);
    EXPECT_DOUBLE_EQ(result.pose.pose.orientation.w, 1.0);
  }
  EXPECT_DOUBLE_EQ(detection.bbox.center.position.x, 0.0);
  EXPECT_DOUBLE_EQ(detection.bbox.center.position.y, 0.0);
  EXPECT_DOUBLE_EQ(detection.bbox.center.position.z, 2.0);
  EXPECT_DOUBLE_EQ(detection.bbox.center.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(detection.bbox.size.x, 0.08);
  EXPECT_DOUBLE_EQ(detection.bbox.size.y, 0.04);
  EXPECT_DOUBLE_EQ(detection.bbox.size.z, 0.0);
}

TEST_F(DetectionDepthProjectionComponentTest, AcceptsFloatMetreEncoding)
{
  publish_inputs(make_inputs(io_node_->now(), "32FC1"));
  ASSERT_TRUE(wait_for_output_count(1U));

  EXPECT_DOUBLE_EQ(output_at(0U).detections.front().bbox.center.position.z, 2.0);
}

TEST_F(DetectionDepthProjectionComponentTest, PublishesBoundingBoxWithoutHypotheses)
{
  auto inputs = make_inputs(io_node_->now());
  inputs.detections.detections.front().results.clear();
  publish_inputs(inputs);
  ASSERT_TRUE(wait_for_output_count(1U));

  const auto & detection = output_at(0U).detections.front();
  EXPECT_TRUE(detection.results.empty());
  EXPECT_DOUBLE_EQ(detection.bbox.center.position.x, 0.0);
  EXPECT_DOUBLE_EQ(detection.bbox.center.position.y, 0.0);
  EXPECT_DOUBLE_EQ(detection.bbox.center.position.z, 2.0);
}

TEST_F(DetectionDepthProjectionComponentTest, RejectsUnsupportedEncoding)
{
  publish_inputs(make_inputs(io_node_->now(), "8UC1"));
  spin_for(300ms);

  EXPECT_EQ(output_count(), 0U);
}

TEST_F(DetectionDepthProjectionComponentTest, RejectsMismatchedCameraFrame)
{
  auto inputs = make_inputs(io_node_->now());
  inputs.camera_info.header.frame_id = "other_camera_frame";
  publish_inputs(inputs);
  spin_for(300ms);

  EXPECT_EQ(output_count(), 0U);
}

TEST_F(DetectionDepthProjectionComponentTest, RejectsMismatchedImageSize)
{
  auto inputs = make_inputs(io_node_->now());
  inputs.camera_info.width = 9U;
  publish_inputs(inputs);
  spin_for(300ms);

  EXPECT_EQ(output_count(), 0U);
}

TEST_F(DetectionDepthProjectionComponentTest, SynchronizesRepeatedSkewedOutOfOrderInputs)
{
  constexpr std::size_t frame_count = 25U;
  const auto first_stamp = io_node_->now();
  const auto depth_skew = rclcpp::Duration::from_seconds(0.010);
  const auto camera_info_skew = rclcpp::Duration::from_seconds(0.020);
  const auto frame_period = rclcpp::Duration::from_seconds(0.100);

  for (std::size_t index = 0U; index < frame_count; ++index) {
    const auto detection_stamp = first_stamp + frame_period * static_cast<double>(index);
    auto inputs = make_inputs(detection_stamp);
    inputs.depth.header.stamp = detection_stamp + depth_skew;
    inputs.camera_info.header.stamp = detection_stamp + camera_info_skew;
    const auto order = static_cast<PublishOrder>(index % 3U);

    publish_inputs(inputs, order);
    spin_for(20ms);
    if (index > 0U) {
      ASSERT_TRUE(wait_for_output_count(index)) << "Frame index " << index - 1U;
    }
  }

  const auto flush_stamp = first_stamp + frame_period * static_cast<double>(frame_count);
  publish_inputs(make_inputs(flush_stamp));
  ASSERT_TRUE(wait_for_output_count(frame_count));
  for (std::size_t index = 0U; index < frame_count; ++index) {
    const auto expected_stamp = first_stamp + frame_period * static_cast<double>(index);
    EXPECT_EQ(rclcpp::Time(output_at(index).header.stamp), expected_stamp);
  }
}

TEST_F(DetectionDepthProjectionComponentTest, RejectsInputsBeyondMaximumSyncInterval)
{
  const auto first_stamp = io_node_->now();
  auto outside_interval = make_inputs(first_stamp);
  outside_interval.depth.header.stamp = first_stamp + rclcpp::Duration::from_seconds(0.075);
  outside_interval.camera_info.header.stamp = first_stamp + rclcpp::Duration::from_seconds(0.010);
  publish_inputs(outside_interval, PublishOrder::CAMERA_INFO_DETECTIONS_DEPTH);

  const auto valid_stamp = first_stamp + rclcpp::Duration::from_seconds(1.0);
  publish_inputs(make_inputs(valid_stamp));
  ASSERT_TRUE(wait_for_output_count(1U));
  spin_for(100ms);

  EXPECT_EQ(output_count(), 1U);
  EXPECT_EQ(rclcpp::Time(output_at(0U).header.stamp), valid_stamp);
}

}  // namespace
}  // namespace perception_detection_depth_projection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
