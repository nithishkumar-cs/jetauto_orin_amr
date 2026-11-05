#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "stop_zone_recovery/stop_zone_recovery_node.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{

using namespace std::chrono_literals;
using stop_zone_recovery::count_returns_in_box;
using stop_zone_recovery::RearBox;

sensor_msgs::msg::LaserScan ring_scan(float range, float angle_min, float angle_max)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "lidar_link";
  scan.angle_min = angle_min;
  scan.angle_increment = 0.01F;
  scan.range_min = 0.05F;
  scan.range_max = 10.0F;
  for (float a = angle_min; a <= angle_max; a += 0.01F) {
    scan.ranges.push_back(range);
  }
  return scan;
}

TEST(CountReturns, CountsOnlyReturnsInsideBoxAndAppliesTransform)
{
  const RearBox box{-1.0, 0.0, 0.65};
  auto scan = ring_scan(0.5F, static_cast<float>(M_PI), static_cast<float>(M_PI));
  EXPECT_EQ(count_returns_in_box(scan, {0.0, 0.0, 0.0}, box), 1U);
  // The same beam points ahead if the lidar is mounted rotated by pi.
  EXPECT_EQ(count_returns_in_box(scan, {0.0, 0.0, M_PI}, box), 0U);
  scan.ranges = {std::numeric_limits<float>::infinity(), std::nanf(""), 0.01F, 20.0F};
  scan.angle_min = static_cast<float>(M_PI);
  EXPECT_EQ(count_returns_in_box(scan, {0.0, 0.0, 0.0}, box), 0U);
}

class RecoveryComponentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("raw_cmd_vel_in_topic", "/test/rec/raw"),
       rclcpp::Parameter("limited_cmd_vel_in_topic", "/test/rec/limited"),
       rclcpp::Parameter("cmd_vel_out_topic", "/test/rec/out"),
       rclcpp::Parameter("scan_in_topic", "/test/rec/scan"),
       rclcpp::Parameter("max_reverse_speed", 0.05)});
    node_ = std::make_shared<stop_zone_recovery::StopZoneRecoveryNode>(options);
    io_ = std::make_shared<rclcpp::Node>("recovery_test_io");
    tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(io_);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = io_->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "lidar_link";
    t.transform.rotation.w = 1.0;
    tf_->sendTransform(t);
    raw_pub_ = io_->create_publisher<geometry_msgs::msg::Twist>("/test/rec/raw", 10);
    limited_pub_ = io_->create_publisher<geometry_msgs::msg::Twist>("/test/rec/limited", 10);
    scan_pub_ = io_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/test/rec/scan", rclcpp::SensorDataQoS());
    sub_ = io_->create_subscription<geometry_msgs::msg::Twist>(
      "/test/rec/out", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr m) { outputs_.push_back(*m); });
    executor_.add_node(node_);
    executor_.add_node(io_);
    spin_for(300ms);
  }

  void TearDown() override
  {
    executor_.remove_node(io_);
    executor_.remove_node(node_);
  }

  void spin_for(std::chrono::milliseconds d)
  {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  // Obstacle ahead at 0.4 m; optional return directly behind at rear_range.
  void send_scan(float rear_range)
  {
    auto scan = ring_scan(5.0F, -3.14F, 3.14F);
    scan.header.stamp = io_->now();
    scan.ranges[scan.ranges.size() / 2] = 0.4F;
    if (rear_range > 0.0F) {
      scan.ranges[0] = rear_range;
    }
    scan_pub_->publish(scan);
    spin_for(100ms);
  }

  // Raw command, then the monitor output (limited_x), as collision_monitor would emit.
  geometry_msgs::msg::Twist run(double rx, double ry, double rz, double limited_x = 0.0)
  {
    geometry_msgs::msg::Twist raw;
    raw.linear.x = rx;
    raw.linear.y = ry;
    raw.angular.z = rz;
    raw_pub_->publish(raw);
    spin_for(50ms);
    geometry_msgs::msg::Twist limited;
    limited.linear.x = limited_x;
    outputs_.clear();
    limited_pub_->publish(limited);
    spin_for(100ms);
    EXPECT_EQ(outputs_.size(), 1U);
    return outputs_.empty() ? geometry_msgs::msg::Twist{} : outputs_.back();
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<stop_zone_recovery::StopZoneRecoveryNode> node_;
  rclcpp::Node::SharedPtr io_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr limited_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::vector<geometry_msgs::msg::Twist> outputs_;
};

TEST_F(RecoveryComponentTest, MonitorOutputPassesThroughWhenNonzero)
{
  send_scan(0.0F);
  EXPECT_DOUBLE_EQ(run(0.2, 0.0, 0.0, 0.07).linear.x, 0.07);
}

TEST_F(RecoveryComponentTest, ZeroedReverseIsCappedWhenRearClear)
{
  send_scan(0.0F);
  EXPECT_DOUBLE_EQ(run(-0.3, 0.0, 0.0).linear.x, -0.05);
  send_scan(0.0F);
  EXPECT_DOUBLE_EQ(run(-0.02, 0.0, 0.0).linear.x, -0.02);
}

TEST_F(RecoveryComponentTest, RearReturnKeepsStop)
{
  send_scan(0.9F);
  EXPECT_DOUBLE_EQ(run(-0.3, 0.0, 0.0).linear.x, 0.0);
}

TEST_F(RecoveryComponentTest, ForwardStrafeAndRotationKeepStop)
{
  send_scan(0.0F);
  EXPECT_DOUBLE_EQ(run(0.3, 0.0, 0.0).linear.x, 0.0);
  EXPECT_DOUBLE_EQ(run(0.0, 0.2, 0.0).linear.x, 0.0);
  send_scan(0.0F);
  EXPECT_DOUBLE_EQ(run(-0.3, 0.2, 0.0).linear.x, 0.0);
  EXPECT_DOUBLE_EQ(run(-0.3, 0.0, 0.5).linear.x, 0.0);
}

TEST_F(RecoveryComponentTest, StopHoldZeroesFlickerSlowdownThenExpires)
{
  send_scan(0.0F);
  EXPECT_DOUBLE_EQ(run(0.06, 0.0, 0.0, 0.0).linear.x, 0.0);
  // Monitor flickers to the 0.35 slowdown on the next scan: held at zero.
  EXPECT_DOUBLE_EQ(run(0.06, 0.0, 0.0, 0.021).linear.x, 0.0);
  spin_for(600ms);
  EXPECT_DOUBLE_EQ(run(0.06, 0.0, 0.0, 0.021).linear.x, 0.021);
}

TEST_F(RecoveryComponentTest, RepeatedFlickerStaysZeroWhileStopsRearmHold)
{
  send_scan(0.0F);
  for (int i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(run(0.06, 0.0, 0.0, 0.0).linear.x, 0.0);
    // Two consecutive slowdown decisions between stops (about 0.3 s) stay held.
    EXPECT_DOUBLE_EQ(run(0.06, 0.0, 0.0, 0.021).linear.x, 0.0);
    EXPECT_DOUBLE_EQ(run(0.06, 0.0, 0.0, 0.021).linear.x, 0.0);
  }
}

TEST_F(RecoveryComponentTest, StopHoldBlocksUnzeroedReverseDespiteClearRear)
{
  for (int i = 0; i < 2; ++i) {
    send_scan(0.0F);
    EXPECT_DOUBLE_EQ(run(0.06, 0.0, 0.0, 0.0).linear.x, 0.0);
    send_scan(0.0F);
    EXPECT_DOUBLE_EQ(run(-0.06, 0.0, 0.0, -0.06).linear.x, 0.0);
  }
  send_scan(0.0F);
  EXPECT_DOUBLE_EQ(run(-0.3, 0.0, 0.0, 0.0).linear.x, -0.05);
}

TEST_F(RecoveryComponentTest, SustainedSlowdownBeyondHoldWindowPasses)
{
  send_scan(0.0F);
  EXPECT_DOUBLE_EQ(run(0.06, 0.0, 0.0, 0.0).linear.x, 0.0);
  spin_for(600ms);
  // No stop re-arms the hold, so continuous slowdown is the monitor's decision again.
  EXPECT_DOUBLE_EQ(run(0.06, 0.0, 0.0, 0.021).linear.x, 0.021);
  EXPECT_DOUBLE_EQ(run(0.06, 0.0, 0.0, 0.021).linear.x, 0.021);
}

TEST_F(RecoveryComponentTest, NoHoldWithoutPriorStop)
{
  send_scan(0.0F);
  EXPECT_DOUBLE_EQ(run(0.06, 0.0, 0.0, 0.021).linear.x, 0.021);
}

TEST_F(RecoveryComponentTest, StaleScanKeepsStop)
{
  send_scan(0.0F);
  spin_for(500ms);
  EXPECT_DOUBLE_EQ(run(-0.3, 0.0, 0.0).linear.x, 0.0);
}

TEST_F(RecoveryComponentTest, StaleRawCommandKeepsStop)
{
  geometry_msgs::msg::Twist raw;
  raw.linear.x = -0.3;
  raw_pub_->publish(raw);
  spin_for(450ms);
  send_scan(0.0F);
  outputs_.clear();
  limited_pub_->publish(geometry_msgs::msg::Twist{});
  spin_for(100ms);
  ASSERT_EQ(outputs_.size(), 1U);
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
