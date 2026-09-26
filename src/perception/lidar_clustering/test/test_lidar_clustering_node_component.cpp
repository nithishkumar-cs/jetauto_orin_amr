#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "perception_lidar_clustering/lidar_clustering_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace perception_lidar_clustering
{
namespace
{

using namespace std::chrono_literals;

constexpr char kScanInputTopic[] = "/test/lidar_clustering/scan_in";
constexpr char kClustersOutputTopic[] = "/test/lidar_clustering/clusters_out";

class LidarClusteringNodeComponentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("scan_in_topic", kScanInputTopic),
       rclcpp::Parameter("clusters_out_topic", kClustersOutputTopic),
       rclcpp::Parameter("scan_queue_size", 2),
       rclcpp::Parameter("max_adjacent_point_distance_m", 0.25),
       rclcpp::Parameter("min_cluster_points", 2)});
    clustering_node_ = std::make_shared<LidarClusteringNode>(options);
    io_node_ = std::make_shared<rclcpp::Node>("lidar_clustering_component_test_io");

    scan_publisher_ = io_node_->create_publisher<sensor_msgs::msg::LaserScan>(
      kScanInputTopic, rclcpp::SensorDataQoS());
    output_subscription_ = io_node_->create_subscription<vision_msgs::msg::Detection3DArray>(
      kClustersOutputTopic, 10, [this](vision_msgs::msg::Detection3DArray::SharedPtr message) {
        std::scoped_lock lock(outputs_mutex_);
        outputs_.push_back(*message);
      });

    executor_.add_node(clustering_node_);
    executor_.add_node(io_node_);
    ASSERT_TRUE(wait_for_connections());
  }

  void TearDown() override
  {
    executor_.remove_node(io_node_);
    executor_.remove_node(clustering_node_);
    io_node_.reset();
    clustering_node_.reset();
  }

  sensor_msgs::msg::LaserScan make_scan(std::vector<float> ranges)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = io_node_->now();
    scan.header.frame_id = "laser_frame";
    scan.angle_min = 0.0F;
    scan.angle_increment = 0.1F;
    scan.angle_max = scan.angle_min + static_cast<float>(ranges.size() - 1U) * scan.angle_increment;
    scan.range_min = 0.1F;
    scan.range_max = 10.0F;
    scan.ranges = std::move(ranges);
    return scan;
  }

  bool wait_for_connections()
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (
        scan_publisher_->get_subscription_count() > 0U &&
        io_node_->count_publishers(kClustersOutputTopic) > 0U) {
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

  vision_msgs::msg::Detection3DArray output_at(std::size_t index) const
  {
    std::scoped_lock lock(outputs_mutex_);
    return outputs_.at(index);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<LidarClusteringNode> clustering_node_;
  rclcpp::Node::SharedPtr io_node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr output_subscription_;
  mutable std::mutex outputs_mutex_;
  std::vector<vision_msgs::msg::Detection3DArray> outputs_;
};

TEST_F(LidarClusteringNodeComponentTest, PublishesClustersWithInputHeaderAndNoClassification)
{
  const auto scan = make_scan({1.0F, 1.0F, std::numeric_limits<float>::infinity(), 2.0F, 2.0F});
  scan_publisher_->publish(scan);

  ASSERT_TRUE(wait_for_output_count(1U));
  const auto output = output_at(0U);
  EXPECT_EQ(clustering_node_->get_parameter("scan_in_topic").as_string(), kScanInputTopic);
  EXPECT_EQ(
    clustering_node_->get_parameter("clusters_out_topic").as_string(), kClustersOutputTopic);
  EXPECT_EQ(clustering_node_->get_parameter("scan_queue_size").as_int(), 2);
  EXPECT_EQ(output.header, scan.header);
  ASSERT_EQ(output.detections.size(), 2U);
  for (const auto & detection : output.detections) {
    EXPECT_EQ(detection.header, scan.header);
    EXPECT_TRUE(detection.results.empty());
    EXPECT_DOUBLE_EQ(detection.bbox.center.position.z, 0.0);
    EXPECT_DOUBLE_EQ(detection.bbox.center.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(detection.bbox.size.z, 0.0);
    EXPECT_GT(detection.bbox.size.y, 0.0);
  }
}

TEST_F(LidarClusteringNodeComponentTest, PublishesEmptyArrayForValidScanWithoutReturns)
{
  const auto no_return = std::numeric_limits<float>::infinity();
  scan_publisher_->publish(make_scan({no_return, no_return, no_return}));

  ASSERT_TRUE(wait_for_output_count(1U));
  EXPECT_TRUE(output_at(0U).detections.empty());
}

TEST_F(LidarClusteringNodeComponentTest, RejectsMalformedScanWithoutPublishing)
{
  auto scan = make_scan({1.0F, 1.0F, 1.0F});
  scan.angle_increment = 0.0F;
  scan_publisher_->publish(scan);
  spin_for(300ms);

  EXPECT_EQ(output_count(), 0U);
}

TEST_F(LidarClusteringNodeComponentTest, RejectsScanWithoutCoordinateFrame)
{
  auto scan = make_scan({1.0F, 1.0F, 1.0F});
  scan.header.frame_id.clear();
  scan_publisher_->publish(scan);
  spin_for(300ms);

  EXPECT_EQ(output_count(), 0U);
}

TEST(LidarClusteringNodeConstructionTest, RejectsInvalidParameters)
{
  rclcpp::NodeOptions points_options;
  points_options.parameter_overrides({rclcpp::Parameter("min_cluster_points", 0)});
  EXPECT_THROW(std::make_shared<LidarClusteringNode>(points_options), std::invalid_argument);

  rclcpp::NodeOptions gap_options;
  gap_options.parameter_overrides({rclcpp::Parameter("max_adjacent_point_distance_m", 0.0)});
  EXPECT_THROW(std::make_shared<LidarClusteringNode>(gap_options), std::invalid_argument);

  rclcpp::NodeOptions queue_options;
  queue_options.parameter_overrides({rclcpp::Parameter("scan_queue_size", 0)});
  EXPECT_THROW(std::make_shared<LidarClusteringNode>(queue_options), std::invalid_argument);
}

}  // namespace
}  // namespace perception_lidar_clustering

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
