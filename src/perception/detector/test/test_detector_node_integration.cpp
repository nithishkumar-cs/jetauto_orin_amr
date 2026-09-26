#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "perception_detector/detector_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace perception_detector
{
namespace
{

using namespace std::chrono_literals;

constexpr char kImageInputTopic[] = "/test/integration/detector/image_in";
constexpr char kDetectionsOutputTopic[] = "/test/integration/detector/detections_out";
constexpr char kDefaultEnginePath[] = "/opt/jetauto_orin_amr/models/detector/yolo26n_fp16.engine";
constexpr char kDefaultLabelsPath[] = "/opt/jetauto_orin_amr/models/detector/coco80.labels";

std::string artifact_path(const char * environment_variable, const char * default_path)
{
  const auto * configured = std::getenv(environment_variable);
  return configured != nullptr && configured[0] != '\0' ? configured : default_path;
}

sensor_msgs::msg::Image make_test_image(const rclcpp::Node & node)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = node.now();
  image.header.frame_id = "camera_rgb_optical_frame";
  image.width = 640U;
  image.height = 480U;
  image.encoding = "rgb8";
  image.is_bigendian = false;
  image.step = image.width * 3U;
  image.data.resize(static_cast<std::size_t>(image.step) * image.height);

  for (std::size_t row = 0U; row < image.height; ++row) {
    for (std::size_t column = 0U; column < image.width; ++column) {
      const auto offset = row * image.step + column * 3U;
      image.data[offset] = static_cast<std::uint8_t>(column % 256U);
      image.data[offset + 1U] = static_cast<std::uint8_t>(row % 256U);
      image.data[offset + 2U] = static_cast<std::uint8_t>((row + column) % 256U);
    }
  }
  return image;
}

bool valid_detection(
  const vision_msgs::msg::Detection2D & detection, std::size_t image_width,
  std::size_t image_height)
{
  const auto center_x = detection.bbox.center.position.x;
  const auto center_y = detection.bbox.center.position.y;
  const auto size_x = detection.bbox.size_x;
  const auto size_y = detection.bbox.size_y;
  const auto left = center_x - size_x / 2.0;
  const auto right = center_x + size_x / 2.0;
  const auto top = center_y - size_y / 2.0;
  const auto bottom = center_y + size_y / 2.0;
  return std::isfinite(center_x) && std::isfinite(center_y) && std::isfinite(size_x) &&
         std::isfinite(size_y) && size_x > 0.0 && size_y > 0.0 && left >= 0.0 && top >= 0.0 &&
         right <= static_cast<double>(image_width) && bottom <= static_cast<double>(image_height);
}

TEST(DetectorNodeIntegrationTest, RunsInstalledYoloEngineThroughRosTopics)
{
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count <= 0) {
    GTEST_SKIP() << "CUDA device is unavailable";
  }

  const auto engine_path =
    artifact_path("PERCEPTION_DETECTOR_TEST_ENGINE_PATH", kDefaultEnginePath);
  const auto labels_path =
    artifact_path("PERCEPTION_DETECTOR_TEST_LABELS_PATH", kDefaultLabelsPath);
  if (!std::filesystem::is_regular_file(engine_path)) {
    GTEST_SKIP() << "TensorRT test engine is unavailable: " << engine_path;
  }
  if (!std::filesystem::is_regular_file(labels_path)) {
    GTEST_SKIP() << "detector test labels are unavailable: " << labels_path;
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("image_in_topic", kImageInputTopic),
     rclcpp::Parameter("detections_out_topic", kDetectionsOutputTopic),
     rclcpp::Parameter("backend", "yolo_tensorrt"),
     rclcpp::Parameter("model_architecture", "yolo26"),
     rclcpp::Parameter("preprocessing_backend", "cuda"),
     rclcpp::Parameter("engine_path", engine_path), rclcpp::Parameter("labels_path", labels_path),
     rclcpp::Parameter("output_format", "end_to_end_xyxy")});
  auto detector_node = std::make_shared<DetectorNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("detector_integration_test_io");
  auto image_publisher =
    io_node->create_publisher<sensor_msgs::msg::Image>(kImageInputTopic, rclcpp::SensorDataQoS());
  std::optional<vision_msgs::msg::Detection2DArray> output;
  auto output_subscription = io_node->create_subscription<vision_msgs::msg::Detection2DArray>(
    kDetectionsOutputTopic, 10,
    [&output](vision_msgs::msg::Detection2DArray::SharedPtr message) { output = *message; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(detector_node);
  executor.add_node(io_node);
  const auto connection_deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < connection_deadline &&
         (image_publisher->get_subscription_count() == 0U ||
          io_node->count_publishers(kDetectionsOutputTopic) == 0U)) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_GT(image_publisher->get_subscription_count(), 0U);
  ASSERT_GT(io_node->count_publishers(kDetectionsOutputTopic), 0U);

  const auto image = make_test_image(*io_node);
  for (std::size_t attempt = 0U; attempt < 3U && !output; ++attempt) {
    image_publisher->publish(image);
    const auto output_deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < output_deadline && !output) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->header, image.header);
  EXPECT_LE(output->detections.size(), 100U);
  for (const auto & detection : output->detections) {
    EXPECT_EQ(detection.header, image.header);
    EXPECT_TRUE(valid_detection(detection, image.width, image.height));
    ASSERT_EQ(detection.results.size(), 1U);
    EXPECT_FALSE(detection.results.front().hypothesis.class_id.empty());
    EXPECT_TRUE(std::isfinite(detection.results.front().hypothesis.score));
    EXPECT_GE(detection.results.front().hypothesis.score, 0.0);
    EXPECT_LE(detection.results.front().hypothesis.score, 1.0);
  }

  executor.remove_node(io_node);
  executor.remove_node(detector_node);
}

}  // namespace
}  // namespace perception_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
