#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "perception_detector/detector_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace perception_detector
{
namespace
{

using namespace std::chrono_literals;

constexpr char kImageInputTopic[] = "/test/detector/image_in";
constexpr char kDetectionsOutputTopic[] = "/test/detector/detections_out";

class FakeDetectorBackend final : public DetectorBackend
{
public:
  std::string name() const override
  {
    return "fake_test_backend";
  }

  std::vector<Detection> detect(const ImageView & image) override
  {
    std::scoped_lock lock(mutex_);
    ++call_count_;
    last_encoding_ = image.encoding();
    last_width_ = image.width();
    if (throw_on_detect_) {
      throw std::runtime_error("synthetic inference failure");
    }
    return detections_;
  }

  void set_detections(std::vector<Detection> detections)
  {
    std::scoped_lock lock(mutex_);
    detections_ = std::move(detections);
  }

  void set_throw_on_detect(bool value)
  {
    std::scoped_lock lock(mutex_);
    throw_on_detect_ = value;
  }

  std::size_t call_count() const
  {
    std::scoped_lock lock(mutex_);
    return call_count_;
  }

  PixelEncoding last_encoding() const
  {
    std::scoped_lock lock(mutex_);
    return last_encoding_;
  }

  std::size_t last_width() const
  {
    std::scoped_lock lock(mutex_);
    return last_width_;
  }

private:
  mutable std::mutex mutex_;
  std::vector<Detection> detections_;
  std::size_t call_count_{0U};
  PixelEncoding last_encoding_{PixelEncoding::RGB8};
  std::size_t last_width_{0U};
  bool throw_on_detect_{false};
};

class DetectorNodeComponentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("image_in_topic", kImageInputTopic),
       rclcpp::Parameter("detections_out_topic", kDetectionsOutputTopic)});
    auto backend = std::make_unique<FakeDetectorBackend>();
    backend_ = backend.get();
    detector_node_ = std::make_shared<DetectorNode>(std::move(backend), options);
    io_node_ = std::make_shared<rclcpp::Node>("detector_component_test_io");

    image_publisher_ = io_node_->create_publisher<sensor_msgs::msg::Image>(
      kImageInputTopic, rclcpp::SensorDataQoS());
    output_subscription_ = io_node_->create_subscription<vision_msgs::msg::Detection2DArray>(
      kDetectionsOutputTopic, 10, [this](vision_msgs::msg::Detection2DArray::SharedPtr message) {
        std::scoped_lock lock(outputs_mutex_);
        outputs_.push_back(*message);
      });

    executor_.add_node(detector_node_);
    executor_.add_node(io_node_);
    ASSERT_TRUE(wait_for_connections());
  }

  void TearDown() override
  {
    executor_.remove_node(io_node_);
    executor_.remove_node(detector_node_);
    io_node_.reset();
    detector_node_.reset();
  }

  sensor_msgs::msg::Image make_image(const std::string & encoding = "rgb8")
  {
    sensor_msgs::msg::Image image;
    image.header.stamp = io_node_->now();
    image.header.frame_id = "camera_rgb_optical_frame";
    image.width = 8U;
    image.height = 6U;
    image.encoding = encoding;
    image.step = image.width * 3U;
    image.data.resize(static_cast<std::size_t>(image.step) * image.height, 127U);
    return image;
  }

  bool wait_for_connections()
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (
        image_publisher_->get_subscription_count() > 0U &&
        detector_node_->count_subscribers(kDetectionsOutputTopic) > 0U) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  bool wait_for_output_count(std::size_t count)
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (output_count() >= count) {
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

  std::size_t output_count() const
  {
    std::scoped_lock lock(outputs_mutex_);
    return outputs_.size();
  }

  vision_msgs::msg::Detection2DArray output_at(std::size_t index) const
  {
    std::scoped_lock lock(outputs_mutex_);
    return outputs_.at(index);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<DetectorNode> detector_node_;
  rclcpp::Node::SharedPtr io_node_;
  FakeDetectorBackend * backend_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr output_subscription_;
  mutable std::mutex outputs_mutex_;
  std::vector<vision_msgs::msg::Detection2DArray> outputs_;
};

TEST_F(DetectorNodeComponentTest, PublishesNormalizedDetectionAndPreservesImageHeader)
{
  backend_->set_detections({Detection{4.0, 3.0, 4.0, 2.0, 0.9, "person"}});
  const auto image = make_image();
  image_publisher_->publish(image);

  ASSERT_TRUE(wait_for_output_count(1U));
  const auto output = output_at(0U);
  EXPECT_EQ(detector_node_->get_parameter("image_in_topic").as_string(), kImageInputTopic);
  EXPECT_EQ(
    detector_node_->get_parameter("detections_out_topic").as_string(), kDetectionsOutputTopic);
  EXPECT_EQ(detector_node_->get_parameter("image_queue_size").as_int(), 1);
  EXPECT_EQ(output.header, image.header);
  ASSERT_EQ(output.detections.size(), 1U);
  const auto & detection = output.detections.front();
  EXPECT_EQ(detection.header, image.header);
  EXPECT_TRUE(detection.id.empty());
  EXPECT_DOUBLE_EQ(detection.bbox.center.position.x, 4.0);
  EXPECT_DOUBLE_EQ(detection.bbox.center.position.y, 3.0);
  EXPECT_DOUBLE_EQ(detection.bbox.size_x, 4.0);
  EXPECT_DOUBLE_EQ(detection.bbox.size_y, 2.0);
  ASSERT_EQ(detection.results.size(), 1U);
  EXPECT_EQ(detection.results.front().hypothesis.class_id, "person");
  EXPECT_DOUBLE_EQ(detection.results.front().hypothesis.score, 0.9);
}

TEST_F(DetectorNodeComponentTest, PublishesAnEmptyArrayWhenTheBackendFindsNothing)
{
  image_publisher_->publish(make_image());

  ASSERT_TRUE(wait_for_output_count(1U));
  EXPECT_TRUE(output_at(0U).detections.empty());
  EXPECT_EQ(backend_->call_count(), 1U);
}

TEST_F(DetectorNodeComponentTest, AcceptsBgrImagesAndPassesAViewToTheBackend)
{
  image_publisher_->publish(make_image("bgr8"));

  ASSERT_TRUE(wait_for_output_count(1U));
  EXPECT_EQ(backend_->last_encoding(), PixelEncoding::BGR8);
  EXPECT_EQ(backend_->last_width(), 8U);
}

TEST_F(DetectorNodeComponentTest, RejectsUnsupportedAndTruncatedImagesBeforeInference)
{
  image_publisher_->publish(make_image("mono8"));
  auto truncated = make_image();
  truncated.data.pop_back();
  image_publisher_->publish(truncated);
  spin_for(300ms);

  EXPECT_EQ(backend_->call_count(), 0U);
  EXPECT_EQ(output_count(), 0U);
}

TEST_F(DetectorNodeComponentTest, PublishesEveryResultReturnedByTheBackend)
{
  backend_->set_detections(
    {Detection{2.0, 3.0, 2.0, 2.0, 0.9, "person"}, Detection{6.0, 3.0, 2.0, 2.0, 0.8, "cart"}});
  image_publisher_->publish(make_image());

  ASSERT_TRUE(wait_for_output_count(1U));
  const auto output = output_at(0U);
  ASSERT_EQ(output.detections.size(), 2U);
  EXPECT_EQ(output.detections.front().results.front().hypothesis.class_id, "person");
  EXPECT_EQ(output.detections.back().results.front().hypothesis.class_id, "cart");
}

TEST_F(DetectorNodeComponentTest, DoesNotPublishAFalseResultWhenInferenceThrows)
{
  backend_->set_throw_on_detect(true);
  image_publisher_->publish(make_image());
  spin_for(300ms);

  EXPECT_EQ(backend_->call_count(), 1U);
  EXPECT_EQ(output_count(), 0U);
}

TEST(DetectorNodeProductionConstructionTest, RejectsAnUnregisteredBackend)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("backend", "not_registered")});

  EXPECT_THROW(std::make_shared<DetectorNode>(options), std::invalid_argument);
}

TEST(DetectorNodeProductionConstructionTest, RejectsAnUnknownPreprocessingBackend)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("backend", "yolo_tensorrt"),
     rclcpp::Parameter("preprocessing_backend", "not_registered")});

  EXPECT_THROW(std::make_shared<DetectorNode>(options), std::invalid_argument);
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
