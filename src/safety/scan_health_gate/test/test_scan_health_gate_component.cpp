#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scan_health_gate/scan_health_gate_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{

using namespace std::chrono_literals;

class ScanHealthGateComponentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("cmd_vel_in_topic", "/test/scan_health/cmd_in"),
       rclcpp::Parameter("cmd_vel_out_topic", "/test/scan_health/cmd_out"),
       rclcpp::Parameter("scan_in_topic", "/test/scan_health/scan"),
       rclcpp::Parameter("scan_timeout_sec", 0.25),
       rclcpp::Parameter("stop_publish_rate_hz", 20.0)});
    gate_ = std::make_shared<scan_health_gate::ScanHealthGateNode>(options);
    io_ = std::make_shared<rclcpp::Node>("scan_health_test_io");
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(io_);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = io_->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(transform);
    command_pub_ = io_->create_publisher<geometry_msgs::msg::Twist>("/test/scan_health/cmd_in", 10);
    scan_pub_ = io_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/test/scan_health/scan", rclcpp::SensorDataQoS());
    output_sub_ = io_->create_subscription<geometry_msgs::msg::Twist>(
      "/test/scan_health/cmd_out", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr message) { outputs_.push_back(*message); });
    executor_.add_node(gate_);
    executor_.add_node(io_);
    spin_for(200ms);
  }

  void TearDown() override
  {
    executor_.remove_node(io_);
    executor_.remove_node(gate_);
    io_.reset();
    tf_broadcaster_.reset();
    gate_.reset();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  bool wait_for_output_after(std::size_t before)
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (outputs_.size() > before) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  void publish_command(double speed = 0.4)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = speed;
    command_pub_->publish(command);
  }

  sensor_msgs::msg::LaserScan valid_scan()
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = io_->now();
    scan.header.frame_id = "base_link";
    scan.angle_min = -3.14F;
    scan.angle_increment = 0.01F;
    scan.range_min = 0.05F;
    scan.range_max = 10.0F;
    scan.ranges = {5.0F, 5.0F, 5.0F, 5.0F};
    return scan;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<scan_health_gate::ScanHealthGateNode> gate_;
  rclcpp::Node::SharedPtr io_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr output_sub_;
  std::vector<geometry_msgs::msg::Twist> outputs_;
};

TEST_F(ScanHealthGateComponentTest, BlocksBeforeFirstScanAndStopsOnTimeout)
{
  const auto before_start_command = outputs_.size();
  publish_command();
  ASSERT_TRUE(wait_for_output_after(before_start_command));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.0);

  scan_pub_->publish(valid_scan());
  spin_for(50ms);
  const auto before_healthy_command = outputs_.size();
  publish_command();
  ASSERT_TRUE(wait_for_output_after(before_healthy_command));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.4);

  const auto before_timeout = outputs_.size();
  ASSERT_TRUE(wait_for_output_after(before_timeout));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.0);

  const auto before_stale_command = outputs_.size();
  publish_command();
  ASSERT_TRUE(wait_for_output_after(before_stale_command));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.0);

  scan_pub_->publish(valid_scan());
  spin_for(50ms);
  const auto before_recovered_command = outputs_.size();
  publish_command();
  ASSERT_TRUE(wait_for_output_after(before_recovered_command));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.4);
}

TEST_F(ScanHealthGateComponentTest, RejectsOldAndUnusableScans)
{
  auto old_scan = valid_scan();
  old_scan.header.stamp = io_->now() - rclcpp::Duration::from_seconds(1.0);
  scan_pub_->publish(old_scan);
  spin_for(50ms);
  const auto before_old_command = outputs_.size();
  publish_command();
  ASSERT_TRUE(wait_for_output_after(before_old_command));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.0);

  auto future_scan = valid_scan();
  future_scan.header.stamp = io_->now() + rclcpp::Duration::from_seconds(1.0);
  scan_pub_->publish(future_scan);
  spin_for(50ms);
  const auto before_future_command = outputs_.size();
  publish_command();
  ASSERT_TRUE(wait_for_output_after(before_future_command));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.0);

  scan_pub_->publish(valid_scan());
  spin_for(50ms);
  const auto before_valid_command = outputs_.size();
  publish_command();
  ASSERT_TRUE(wait_for_output_after(before_valid_command));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.4);

  auto unusable_scan = valid_scan();
  unusable_scan.ranges.assign(4U, std::numeric_limits<float>::quiet_NaN());
  scan_pub_->publish(unusable_scan);
  spin_for(50ms);
  const auto before_unusable_command = outputs_.size();
  publish_command();
  ASSERT_TRUE(wait_for_output_after(before_unusable_command));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.0);
}

TEST_F(ScanHealthGateComponentTest, BlocksUntilScanFrameCanBeTransformed)
{
  auto scan = valid_scan();
  scan.header.frame_id = "lidar_link";
  scan_pub_->publish(scan);
  spin_for(50ms);
  const auto before_missing_tf = outputs_.size();
  publish_command();
  ASSERT_TRUE(wait_for_output_after(before_missing_tf));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.0);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = io_->now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "lidar_link";
  transform.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(transform);
  spin_for(50ms);
  scan = valid_scan();
  scan.header.frame_id = "lidar_link";
  scan_pub_->publish(scan);
  spin_for(50ms);
  const auto before_recovery = outputs_.size();
  publish_command();
  ASSERT_TRUE(wait_for_output_after(before_recovery));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.4);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
