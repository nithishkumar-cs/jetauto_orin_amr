#ifndef PERCEPTION_METRICS__METRICS_REPORTER_HPP_
#define PERCEPTION_METRICS__METRICS_REPORTER_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception_metrics
{

/// One named numeric value in a metrics report.
struct Metric
{
  const char * key;
  double value;
};

/// Optional per-cycle metrics for a perception node on `/diagnostics`.
///
/// Declares the node parameter `export_metrics` (default false). When false no
/// publisher exists, `start()` does no clock read, and `publish()` returns
/// immediately, so disabled nodes pay only a pointer test. Each report is one
/// `DiagnosticStatus` named `<node name>/metrics`: the caller's values, then
/// `input_age_ms` (node clock minus the input stamp, i.e. sensor-to-node
/// latency) and `dropped_total` (inputs rejected since start, from `note_drop`).
class MetricsReporter
{
public:
  using Clock = std::chrono::steady_clock;

  /// Declare `export_metrics` on `node` and create the publisher when enabled.
  explicit MetricsReporter(rclcpp::Node & node)
  : node_(node)
  {
    if (node.declare_parameter<bool>("export_metrics", false)) {
      publisher_ = node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10));
    }
  }

  /// True when reports will be published.
  bool enabled() const {return publisher_ != nullptr;}

  /// Wall-clock start of a cycle; a zero time point when disabled.
  Clock::time_point start() const {return enabled() ? Clock::now() : Clock::time_point{};}

  /// Milliseconds from `begin` (from `start()`) to now; 0 when disabled.
  double elapsed_ms(Clock::time_point begin) const
  {
    return enabled() ? std::chrono::duration<double, std::milli>(Clock::now() - begin).count() : 0.0;
  }

  /// Count one input that was rejected or skipped without producing output.
  void note_drop() {dropped_total_.fetch_add(1U, std::memory_order_relaxed);}

  /// Publish one report for a completed cycle whose input carried `input_stamp`.
  void publish(const builtin_interfaces::msg::Time & input_stamp, std::initializer_list<Metric> values)
  {
    if (!publisher_) {
      return;
    }
    const auto now = node_.now();
    diagnostic_msgs::msg::DiagnosticArray report;
    report.header.stamp = now;
    auto & status = report.status.emplace_back();
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = std::string(node_.get_name()) + "/metrics";
    status.message = "cycle completed";
    status.values.reserve(values.size() + 2U);
    for (const auto & metric : values) {
      add(status, metric.key, metric.value);
    }
    add(status, "input_age_ms", (now - rclcpp::Time(input_stamp, now.get_clock_type())).seconds() * 1e3);
    add(status, "dropped_total", static_cast<double>(dropped_total_.load(std::memory_order_relaxed)));
    publisher_->publish(std::move(report));
  }

private:
  static void add(diagnostic_msgs::msg::DiagnosticStatus & status, const char * key, double value)
  {
    char text[32];
    if (std::isfinite(value) && value == std::floor(value) && std::fabs(value) < 1e15) {
      std::snprintf(text, sizeof(text), "%lld", static_cast<long long>(value));
    } else {
      std::snprintf(text, sizeof(text), "%.3f", value);
    }
    auto & entry = status.values.emplace_back();
    entry.key = key;
    entry.value = text;
  }

  rclcpp::Node & node_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  std::atomic<std::uint64_t> dropped_total_{0U};
};

}  // namespace perception_metrics

#endif  // PERCEPTION_METRICS__METRICS_REPORTER_HPP_
