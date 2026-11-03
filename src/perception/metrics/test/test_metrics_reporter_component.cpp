#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "perception_metrics/metrics_reporter.hpp"

using namespace std::chrono_literals;

namespace
{

class ReporterNode : public rclcpp::Node
{
public:
  explicit ReporterNode(bool export_metrics)
  : Node("metrics_test_node", rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("export_metrics", export_metrics)})),
    reporter(*this)
  {
  }

  perception_metrics::MetricsReporter reporter;
};

/// Publish one report whose input stamp trails the node clock by `stamp_lag`
/// and return everything received on `/diagnostics`.
std::vector<diagnostic_msgs::msg::DiagnosticArray> collect(
  const std::shared_ptr<ReporterNode> & node, bool expect_report,
  std::chrono::milliseconds stamp_lag = 0ms)
{
  auto io = std::make_shared<rclcpp::Node>("metrics_test_io");
  std::vector<diagnostic_msgs::msg::DiagnosticArray> reports;
  auto subscription = io->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10, [&reports](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
      reports.push_back(*message);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(io);
  executor.add_node(node);
  const auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
  while (expect_report && std::chrono::steady_clock::now() < discovery_deadline &&
         subscription->get_publisher_count() == 0U) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  node->reporter.note_drop();
  node->reporter.publish(node->now() - rclcpp::Duration(stamp_lag), {{"process_wall_ms", 1.5}, {"count", 3.0}});
  const auto deadline = std::chrono::steady_clock::now() + (expect_report ? 2s : 200ms);
  while (std::chrono::steady_clock::now() < deadline && reports.empty()) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  return reports;
}

}  // namespace

TEST(MetricsReporterTest, PublishesCallerValuesThenCommonKeysWhenEnabled)
{
  const auto reports = collect(std::make_shared<ReporterNode>(true), true);
  ASSERT_EQ(reports.size(), 1U);
  ASSERT_EQ(reports.front().status.size(), 1U);
  const auto & status = reports.front().status.front();
  EXPECT_EQ(status.name, "metrics_test_node/metrics");
  ASSERT_EQ(status.values.size(), 4U);
  EXPECT_EQ(status.values[0].key, "process_wall_ms");
  EXPECT_EQ(status.values[0].value, "1.500");
  EXPECT_EQ(status.values[1].key, "count");
  EXPECT_EQ(status.values[1].value, "3");
  EXPECT_EQ(status.values[2].key, "input_age_ms");
  EXPECT_GE(std::stod(status.values[2].value), 0.0);
  EXPECT_EQ(status.values[3].key, "dropped_total");
  EXPECT_EQ(status.values[3].value, "1");
}

TEST(MetricsReporterTest, InputAgeTracksWallClockStampLag)
{
  // Wall-clock node (no use_sim_time): a stamp 200 ms old must report about 200 ms,
  // unlike replay under sim /clock where the age is quantized by the clock.
  const auto reports = collect(std::make_shared<ReporterNode>(true), true, 200ms);
  ASSERT_EQ(reports.size(), 1U);
  const double age_ms = std::stod(reports.front().status.front().values[2].value);
  EXPECT_GE(age_ms, 200.0);
  EXPECT_LT(age_ms, 300.0);
}

TEST(MetricsReporterTest, PublishesNothingWhenDisabled)
{
  auto node = std::make_shared<ReporterNode>(false);
  EXPECT_FALSE(node->reporter.enabled());
  EXPECT_TRUE(collect(node, false).empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
