#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "amr_interfaces/msg/tracked_obstacle_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "perception_tracking/tracking_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace perception_tracking
{
namespace
{

using namespace std::chrono_literals;

constexpr char kObservationsInputTopic[] = "/test/tracking/observations_in";
constexpr char kTracksOutputTopic[] = "/test/tracking/tracks_out";
constexpr char kSensorFrame[] = "test_sensor_frame";
constexpr char kTrackingFrame[] = "odom";
constexpr char kOutputFrame[] = "base_link";
constexpr double kUnknownVariance = 1234.0;

class TrackingNodeComponentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("observations_in_topic", kObservationsInputTopic),
       rclcpp::Parameter("tracks_out_topic", kTracksOutputTopic),
       rclcpp::Parameter("source", "camera"), rclcpp::Parameter("tracking_frame", kTrackingFrame),
       rclcpp::Parameter("output_frame", kOutputFrame),
       rclcpp::Parameter("transform_timeout_ms", 50),
       rclcpp::Parameter("unobserved_state_variance", kUnknownVariance),
       rclcpp::Parameter("observations_queue_size", 5), rclcpp::Parameter("min_confirmed_hits", 1),
       rclcpp::Parameter("max_unobserved_duration_s", 0.5)});
    tracking_node_ = std::make_shared<TrackingNode>(options);
    io_node_ = std::make_shared<rclcpp::Node>("tracking_component_test_io");
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(io_node_);

    observations_publisher_ = io_node_->create_publisher<vision_msgs::msg::Detection3DArray>(
      kObservationsInputTopic, rclcpp::QoS(10));
    tracks_subscription_ = io_node_->create_subscription<amr_interfaces::msg::TrackedObstacleArray>(
      kTracksOutputTopic, rclcpp::QoS(10),
      [this](amr_interfaces::msg::TrackedObstacleArray::SharedPtr message) {
        std::scoped_lock lock(outputs_mutex_);
        outputs_.push_back(*message);
      });

    executor_.add_node(tracking_node_);
    executor_.add_node(io_node_);
    publish_static_transforms();
    ASSERT_TRUE(wait_for_connections());
    spin_for(100ms);
  }

  void TearDown() override
  {
    executor_.remove_node(io_node_);
    executor_.remove_node(tracking_node_);
    static_broadcaster_.reset();
    io_node_.reset();
    tracking_node_.reset();
  }

  void publish_static_transforms()
  {
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = io_node_->now();
    odom_to_base.header.frame_id = kTrackingFrame;
    odom_to_base.child_frame_id = kOutputFrame;
    odom_to_base.transform.translation.x = 1.0;
    odom_to_base.transform.rotation.w = 1.0;

    geometry_msgs::msg::TransformStamped base_to_sensor;
    base_to_sensor.header.stamp = odom_to_base.header.stamp;
    base_to_sensor.header.frame_id = kOutputFrame;
    base_to_sensor.child_frame_id = kSensorFrame;
    base_to_sensor.transform.translation.x = 0.5;
    base_to_sensor.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform({odom_to_base, base_to_sensor});
  }

  vision_msgs::msg::Detection3DArray make_observations(bool classified = true)
  {
    vision_msgs::msg::Detection3DArray observations;
    observations.header.stamp = io_node_->now();
    observations.header.frame_id = kSensorFrame;
    auto & detection = observations.detections.emplace_back();
    detection.header = observations.header;
    detection.bbox.center.position.x = 2.0;
    detection.bbox.center.position.y = 1.0;
    detection.bbox.center.position.z = 0.4;
    detection.bbox.center.orientation.w = 1.0;
    detection.bbox.size.x = 0.6;
    detection.bbox.size.y = 0.5;
    detection.bbox.size.z = 1.7;
    if (classified) {
      auto & result = detection.results.emplace_back();
      result.hypothesis.class_id = "person";
      result.hypothesis.score = 0.9;
    }
    return observations;
  }

  bool wait_for_connections()
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (
        observations_publisher_->get_subscription_count() > 0U &&
        io_node_->count_publishers(kTracksOutputTopic) > 0U) {
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

  amr_interfaces::msg::TrackedObstacleArray output_at(std::size_t index) const
  {
    std::scoped_lock lock(outputs_mutex_);
    return outputs_.at(index);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<TrackingNode> tracking_node_;
  rclcpp::Node::SharedPtr io_node_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr observations_publisher_;
  rclcpp::Subscription<amr_interfaces::msg::TrackedObstacleArray>::SharedPtr tracks_subscription_;
  mutable std::mutex outputs_mutex_;
  std::vector<amr_interfaces::msg::TrackedObstacleArray> outputs_;
};

TEST_F(TrackingNodeComponentTest, TransformsObservationAndPublishesTrackedContract)
{
  const auto observations = make_observations();
  observations_publisher_->publish(observations);

  ASSERT_TRUE(wait_for_output_count(1U));
  const auto output = output_at(0U);
  EXPECT_EQ(
    tracking_node_->get_parameter("observations_in_topic").as_string(), kObservationsInputTopic);
  EXPECT_EQ(tracking_node_->get_parameter("tracks_out_topic").as_string(), kTracksOutputTopic);
  EXPECT_EQ(output.header.stamp, observations.header.stamp);
  EXPECT_EQ(output.header.frame_id, kOutputFrame);
  ASSERT_EQ(output.obstacles.size(), 1U);
  const auto & track = output.obstacles.front();
  EXPECT_EQ(track.track_id, 1U);
  EXPECT_EQ(track.source, "camera");
  EXPECT_EQ(track.classification.class_id, "person");
  EXPECT_DOUBLE_EQ(track.classification.score, 0.9);
  EXPECT_NEAR(track.pose.pose.position.x, 2.5, 1e-9);
  EXPECT_NEAR(track.pose.pose.position.y, 1.0, 1e-9);
  EXPECT_NEAR(track.pose.pose.position.z, 0.4, 1e-9);
  EXPECT_DOUBLE_EQ(track.pose.pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(track.size.x, 0.6);
  EXPECT_DOUBLE_EQ(track.size.y, 0.5);
  EXPECT_DOUBLE_EQ(track.size.z, 1.7);
  EXPECT_NEAR(track.velocity.twist.linear.x, 0.0, 1e-12);
  EXPECT_FALSE(track.is_dynamic);
  EXPECT_GT(track.pose.covariance[0], 0.0);
  EXPECT_DOUBLE_EQ(track.pose.covariance[14], kUnknownVariance);
  EXPECT_DOUBLE_EQ(track.velocity.covariance[35], kUnknownVariance);
}

TEST_F(TrackingNodeComponentTest, PreservesUnclassifiedLidarStyleObservation)
{
  observations_publisher_->publish(make_observations(false));

  ASSERT_TRUE(wait_for_output_count(1U));
  const auto output = output_at(0U);
  ASSERT_EQ(output.obstacles.size(), 1U);
  EXPECT_TRUE(output.obstacles.front().classification.class_id.empty());
  EXPECT_DOUBLE_EQ(output.obstacles.front().classification.score, 0.0);
}

TEST_F(TrackingNodeComponentTest, PublishesPredictionThenRemovesExpiredTrack)
{
  auto observations = make_observations();
  observations_publisher_->publish(observations);
  ASSERT_TRUE(wait_for_output_count(1U));
  const auto track_id = output_at(0U).obstacles.front().track_id;

  vision_msgs::msg::Detection3DArray empty;
  empty.header.frame_id = kSensorFrame;
  empty.header.stamp = static_cast<builtin_interfaces::msg::Time>(
    rclcpp::Time(observations.header.stamp) + rclcpp::Duration::from_seconds(0.2));
  observations_publisher_->publish(empty);
  ASSERT_TRUE(wait_for_output_count(2U));
  ASSERT_EQ(output_at(1U).obstacles.size(), 1U);
  EXPECT_EQ(output_at(1U).obstacles.front().track_id, track_id);

  empty.header.stamp = static_cast<builtin_interfaces::msg::Time>(
    rclcpp::Time(observations.header.stamp) + rclcpp::Duration::from_seconds(0.8));
  observations_publisher_->publish(empty);
  ASSERT_TRUE(wait_for_output_count(3U));
  EXPECT_TRUE(output_at(2U).obstacles.empty());
}

TEST_F(TrackingNodeComponentTest, RejectsMissingTransformWithoutPublishing)
{
  auto observations = make_observations();
  observations.header.frame_id = "missing_sensor_frame";
  observations.detections.front().header = observations.header;
  observations_publisher_->publish(observations);
  spin_for(250ms);

  EXPECT_EQ(output_count(), 0U);
}

TEST_F(TrackingNodeComponentTest, SkipsInvalidDetectionAndPublishesEmptyArray)
{
  auto observations = make_observations();
  observations.detections.front().bbox.size.x = -1.0;
  observations_publisher_->publish(observations);

  ASSERT_TRUE(wait_for_output_count(1U));
  EXPECT_TRUE(output_at(0U).obstacles.empty());
}

TEST(TrackingNodeConstructionTest, RejectsInvalidRequiredParameters)
{
  rclcpp::NodeOptions source_options;
  source_options.parameter_overrides({rclcpp::Parameter("source", "")});
  EXPECT_THROW(std::make_shared<TrackingNode>(source_options), std::invalid_argument);

  rclcpp::NodeOptions hits_options;
  hits_options.parameter_overrides(
    {rclcpp::Parameter("source", "camera"), rclcpp::Parameter("min_confirmed_hits", 0)});
  EXPECT_THROW(std::make_shared<TrackingNode>(hits_options), std::invalid_argument);

  rclcpp::NodeOptions timeout_options;
  timeout_options.parameter_overrides(
    {rclcpp::Parameter("source", "camera"), rclcpp::Parameter("transform_timeout_ms", -1)});
  EXPECT_THROW(std::make_shared<TrackingNode>(timeout_options), std::invalid_argument);
}

}  // namespace
}  // namespace perception_tracking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
