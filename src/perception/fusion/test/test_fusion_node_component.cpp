#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "amr_interfaces/msg/tracked_obstacle_array.hpp"
#include "perception_fusion/fusion_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception_fusion
{
namespace
{

using namespace std::chrono_literals;

constexpr char kFusionFrame[] = "base_link";
constexpr char kCameraInputTopic[] = "/test/fusion/camera_tracks";
constexpr char kLidarInputTopic[] = "/test/fusion/lidar_tracks";
constexpr char kOutputTopic[] = "/test/fusion/output";
constexpr double kUnknownVariance = 4321.0;

class FusionNodeComponentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("camera_tracks_in_topic", kCameraInputTopic),
       rclcpp::Parameter("lidar_tracks_in_topic", kLidarInputTopic),
       rclcpp::Parameter("fused_obstacles_out_topic", kOutputTopic),
       rclcpp::Parameter("fusion_frame", kFusionFrame), rclcpp::Parameter("sync_queue_size", 10),
       rclcpp::Parameter("sync_max_interval_ms", 50),
       rclcpp::Parameter("cross_sensor_association_distance_m", 0.75),
       rclcpp::Parameter("unobserved_state_variance", kUnknownVariance)});
    fusion_node_ = std::make_shared<FusionNode>(options);
    io_node_ = std::make_shared<rclcpp::Node>("fusion_component_test_io");
    camera_publisher_ = io_node_->create_publisher<amr_interfaces::msg::TrackedObstacleArray>(
      kCameraInputTopic, rclcpp::QoS(10));
    lidar_publisher_ = io_node_->create_publisher<amr_interfaces::msg::TrackedObstacleArray>(
      kLidarInputTopic, rclcpp::QoS(10));
    output_subscription_ = io_node_->create_subscription<amr_interfaces::msg::TrackedObstacleArray>(
      kOutputTopic, rclcpp::QoS(10),
      [this](amr_interfaces::msg::TrackedObstacleArray::SharedPtr message) {
        std::scoped_lock lock(outputs_mutex_);
        outputs_.push_back(*message);
      });

    executor_.add_node(fusion_node_);
    executor_.add_node(io_node_);
    ASSERT_TRUE(wait_for_connections());
  }

  void TearDown() override
  {
    executor_.remove_node(io_node_);
    executor_.remove_node(fusion_node_);
    io_node_.reset();
    fusion_node_.reset();
  }

  bool wait_for_connections()
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (
        camera_publisher_->get_subscription_count() > 0U &&
        lidar_publisher_->get_subscription_count() > 0U &&
        fusion_node_->count_subscribers(kOutputTopic) > 0U) {
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

  amr_interfaces::msg::TrackedObstacleArray make_tracks(
    const rclcpp::Time & stamp, const std::string & source, double x_m,
    bool include_obstacle = true)
  {
    amr_interfaces::msg::TrackedObstacleArray tracks;
    tracks.header.stamp = static_cast<builtin_interfaces::msg::Time>(stamp);
    tracks.header.frame_id = kFusionFrame;
    if (!include_obstacle) {
      return tracks;
    }

    auto & obstacle = tracks.obstacles.emplace_back();
    obstacle.track_id = source == "camera" ? 10U : 20U;
    obstacle.pose.pose.position.x = x_m;
    obstacle.pose.pose.position.y = 1.0;
    obstacle.pose.pose.position.z = source == "camera" ? 0.8 : 0.0;
    obstacle.pose.pose.orientation.w = 1.0;
    const auto position_variance = source == "camera" ? 0.04 : 0.01;
    obstacle.pose.covariance[0] = position_variance;
    obstacle.pose.covariance[7] = position_variance;
    obstacle.size.x = source == "camera" ? 0.6 : 0.8;
    obstacle.size.y = source == "camera" ? 0.4 : 0.7;
    obstacle.size.z = source == "camera" ? 1.7 : 0.0;
    obstacle.velocity.twist.linear.x = source == "camera" ? 0.4 : 0.0;
    obstacle.velocity.covariance[0] = 0.25;
    obstacle.velocity.covariance[7] = 0.25;
    obstacle.is_dynamic = source == "camera";
    obstacle.source = source;
    if (source == "camera") {
      obstacle.classification.class_id = "person";
      obstacle.classification.score = 0.9;
    }
    return tracks;
  }

  void publish_pair(
    const amr_interfaces::msg::TrackedObstacleArray & camera,
    const amr_interfaces::msg::TrackedObstacleArray & lidar)
  {
    camera_publisher_->publish(camera);
    lidar_publisher_->publish(lidar);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<FusionNode> fusion_node_;
  rclcpp::Node::SharedPtr io_node_;
  rclcpp::Publisher<amr_interfaces::msg::TrackedObstacleArray>::SharedPtr camera_publisher_;
  rclcpp::Publisher<amr_interfaces::msg::TrackedObstacleArray>::SharedPtr lidar_publisher_;
  rclcpp::Subscription<amr_interfaces::msg::TrackedObstacleArray>::SharedPtr output_subscription_;
  mutable std::mutex outputs_mutex_;
  std::vector<amr_interfaces::msg::TrackedObstacleArray> outputs_;
};

TEST_F(FusionNodeComponentTest, SynchronizesAndPublishesCovarianceWeightedFusedContract)
{
  const auto stamp = io_node_->now();
  publish_pair(make_tracks(stamp, "camera", 1.0), make_tracks(stamp, "lidar", 1.2));

  ASSERT_TRUE(wait_for_output_count(1U));
  const auto output = output_at(0U);
  EXPECT_EQ(output.header.stamp, static_cast<builtin_interfaces::msg::Time>(stamp));
  EXPECT_EQ(output.header.frame_id, kFusionFrame);
  ASSERT_EQ(output.obstacles.size(), 1U);
  const auto & obstacle = output.obstacles.front();
  EXPECT_EQ(obstacle.track_id, 1U);
  EXPECT_NEAR(obstacle.pose.pose.position.x, 1.16, 1e-12);
  EXPECT_NEAR(obstacle.pose.covariance[0], 0.008, 1e-12);
  EXPECT_EQ(obstacle.classification.class_id, "person");
  EXPECT_DOUBLE_EQ(obstacle.classification.score, 0.9);
  EXPECT_DOUBLE_EQ(obstacle.size.x, 0.8);
  EXPECT_DOUBLE_EQ(obstacle.size.y, 0.7);
  EXPECT_DOUBLE_EQ(obstacle.size.z, 1.7);
  EXPECT_TRUE(obstacle.is_dynamic);
  EXPECT_EQ(obstacle.source, "camera+lidar");
  EXPECT_DOUBLE_EQ(obstacle.pose.covariance[14], kUnknownVariance);
  EXPECT_DOUBLE_EQ(obstacle.velocity.covariance[35], kUnknownVariance);
  EXPECT_EQ(fusion_node_->get_parameter("camera_tracks_in_topic").as_string(), kCameraInputTopic);
}

TEST_F(FusionNodeComponentTest, SynchronizesRepeatedSkewedOutOfOrderTrackArrays)
{
  constexpr std::size_t frame_count = 25U;
  const auto first_stamp = io_node_->now();
  const auto lidar_skew = rclcpp::Duration::from_seconds(0.02);
  const auto frame_period = rclcpp::Duration::from_seconds(0.1);

  for (std::size_t index = 0U; index < frame_count; ++index) {
    const auto camera_stamp = first_stamp + frame_period * static_cast<double>(index);
    const auto camera = make_tracks(camera_stamp, "camera", 1.0);
    const auto lidar = make_tracks(camera_stamp + lidar_skew, "lidar", 1.1);
    if (index % 2U == 0U) {
      camera_publisher_->publish(camera);
      lidar_publisher_->publish(lidar);
    } else {
      lidar_publisher_->publish(lidar);
      camera_publisher_->publish(camera);
    }
    spin_for(20ms);
    if (index > 0U) {
      ASSERT_TRUE(wait_for_output_count(index)) << "Frame index " << index - 1U;
    }
  }

  const auto flush_stamp = first_stamp + frame_period * static_cast<double>(frame_count);
  publish_pair(make_tracks(flush_stamp, "camera", 1.0), make_tracks(flush_stamp, "lidar", 1.1));
  ASSERT_TRUE(wait_for_output_count(frame_count));
  for (std::size_t index = 0U; index < frame_count; ++index) {
    const auto expected_stamp =
      first_stamp + frame_period * static_cast<double>(index) + lidar_skew;
    EXPECT_EQ(
      output_at(index).header.stamp, static_cast<builtin_interfaces::msg::Time>(expected_stamp));
  }
}

TEST_F(FusionNodeComponentTest, PublishesUnmatchedCameraAndLidarTracksSeparately)
{
  const auto stamp = io_node_->now();
  publish_pair(make_tracks(stamp, "camera", 0.0), make_tracks(stamp, "lidar", 3.0));

  ASSERT_TRUE(wait_for_output_count(1U));
  const auto output = output_at(0U);
  ASSERT_EQ(output.obstacles.size(), 2U);
  EXPECT_NE(output.obstacles[0].track_id, output.obstacles[1].track_id);
  EXPECT_EQ(output.obstacles[0].source, "camera");
  EXPECT_EQ(output.obstacles[1].source, "lidar");
}

TEST_F(FusionNodeComponentTest, PreservesFusedIdAcrossSingleAndDualSourceCycles)
{
  const auto first_stamp = io_node_->now();
  publish_pair(
    make_tracks(first_stamp, "camera", 1.0, false), make_tracks(first_stamp, "lidar", 1.0));
  ASSERT_TRUE(wait_for_output_count(1U));
  ASSERT_EQ(output_at(0U).obstacles.size(), 1U);
  const auto fused_id = output_at(0U).obstacles.front().track_id;

  const auto second_stamp = first_stamp + rclcpp::Duration::from_seconds(0.1);
  publish_pair(make_tracks(second_stamp, "camera", 1.1), make_tracks(second_stamp, "lidar", 1.0));
  ASSERT_TRUE(wait_for_output_count(2U));
  ASSERT_EQ(output_at(1U).obstacles.size(), 1U);
  EXPECT_EQ(output_at(1U).obstacles.front().track_id, fused_id);

  const auto third_stamp = second_stamp + rclcpp::Duration::from_seconds(0.1);
  publish_pair(
    make_tracks(third_stamp, "camera", 1.2), make_tracks(third_stamp, "lidar", 1.0, false));
  ASSERT_TRUE(wait_for_output_count(3U));
  ASSERT_EQ(output_at(2U).obstacles.size(), 1U);
  EXPECT_EQ(output_at(2U).obstacles.front().track_id, fused_id);
}

TEST_F(FusionNodeComponentTest, RejectsMismatchedFrames)
{
  const auto stamp = io_node_->now();
  auto camera = make_tracks(stamp, "camera", 1.0);
  camera.header.frame_id = "odom";
  publish_pair(camera, make_tracks(stamp, "lidar", 1.0));
  spin_for(250ms);

  EXPECT_EQ(output_count(), 0U);
}

TEST_F(FusionNodeComponentTest, SkipsInvalidInputTrackAndKeepsValidSource)
{
  const auto stamp = io_node_->now();
  auto camera = make_tracks(stamp, "camera", 1.0);
  camera.obstacles.front().size.x = -1.0;
  publish_pair(camera, make_tracks(stamp, "lidar", 1.0));

  ASSERT_TRUE(wait_for_output_count(1U));
  const auto output = output_at(0U);
  ASSERT_EQ(output.obstacles.size(), 1U);
  EXPECT_EQ(output.obstacles.front().source, "lidar");
}

TEST_F(FusionNodeComponentTest, EnforcesSyncBoundAndRecoversWhenMatchingSampleArrives)
{
  const auto camera_stamp = io_node_->now();
  const auto lidar_stamp = camera_stamp + rclcpp::Duration::from_seconds(0.2);
  camera_publisher_->publish(make_tracks(camera_stamp, "camera", 1.0));
  lidar_publisher_->publish(make_tracks(lidar_stamp, "lidar", 1.0));
  spin_for(250ms);
  EXPECT_EQ(output_count(), 0U);

  camera_publisher_->publish(make_tracks(lidar_stamp, "camera", 1.0));
  ASSERT_TRUE(wait_for_output_count(1U));
  EXPECT_EQ(output_at(0U).header.stamp, static_cast<builtin_interfaces::msg::Time>(lidar_stamp));
}

TEST(FusionNodeConstructionTest, RejectsInvalidParameters)
{
  rclcpp::NodeOptions frame_options;
  frame_options.parameter_overrides({rclcpp::Parameter("fusion_frame", "")});
  EXPECT_THROW(std::make_shared<FusionNode>(frame_options), std::invalid_argument);

  rclcpp::NodeOptions queue_options;
  queue_options.parameter_overrides({rclcpp::Parameter("sync_queue_size", 0)});
  EXPECT_THROW(std::make_shared<FusionNode>(queue_options), std::invalid_argument);

  rclcpp::NodeOptions association_options;
  association_options.parameter_overrides(
    {rclcpp::Parameter("cross_sensor_association_distance_m", 0.0)});
  EXPECT_THROW(std::make_shared<FusionNode>(association_options), std::invalid_argument);
}

}  // namespace
}  // namespace perception_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
