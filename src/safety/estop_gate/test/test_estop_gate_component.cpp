#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "estop_gate/estop_gate_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{

using namespace std::chrono_literals;

class EstopGateComponentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("cmd_vel_in_topic", "/test/estop_gate/cmd_in"),
       rclcpp::Parameter("cmd_vel_out_topic", "/test/estop_gate/cmd_out"),
       rclcpp::Parameter("estop_in_topic", "/test/estop_gate/engaged"),
       rclcpp::Parameter("reset_in_topic", "/test/estop_gate/reset")});
    gate_ = std::make_shared<estop_gate::EstopGateNode>(options);
    io_node_ = std::make_shared<rclcpp::Node>("estop_gate_test_io");

    command_publisher_ =
      io_node_->create_publisher<geometry_msgs::msg::Twist>("/test/estop_gate/cmd_in", 10);
    estop_publisher_ =
      io_node_->create_publisher<std_msgs::msg::Bool>("/test/estop_gate/engaged", 10);
    reset_publisher_ =
      io_node_->create_publisher<std_msgs::msg::Bool>("/test/estop_gate/reset", 10);
    output_subscription_ = io_node_->create_subscription<geometry_msgs::msg::Twist>(
      "/test/estop_gate/cmd_out", 10, [this](geometry_msgs::msg::Twist::SharedPtr message) {
        std::scoped_lock lock(outputs_mutex_);
        outputs_.push_back(*message);
      });

    executor_.add_node(gate_);
    executor_.add_node(io_node_);
    spin_for(200ms);
  }

  void TearDown() override
  {
    executor_.remove_node(io_node_);
    executor_.remove_node(gate_);
    io_node_.reset();
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

  bool wait_for_output()
  {
    std::size_t previous_count;
    {
      std::scoped_lock lock(outputs_mutex_);
      previous_count = outputs_.size();
    }
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      {
        std::scoped_lock lock(outputs_mutex_);
        if (outputs_.size() > previous_count) {
          return true;
        }
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  bool wait_for_output_after(std::size_t previous_count, bool spin_executor)
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (spin_executor) {
        executor_.spin_some();
      }
      {
        std::scoped_lock lock(outputs_mutex_);
        if (outputs_.size() > previous_count) {
          return true;
        }
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  void publish_command(double linear_x)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear_x;
    command_publisher_->publish(command);
  }

  void publish_bool(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher, bool value)
  {
    std_msgs::msg::Bool message;
    message.data = value;
    publisher->publish(message);
  }

  void clear_outputs()
  {
    std::scoped_lock lock(outputs_mutex_);
    outputs_.clear();
  }

  std::size_t output_count()
  {
    std::scoped_lock lock(outputs_mutex_);
    return outputs_.size();
  }

  geometry_msgs::msg::Twist last_output()
  {
    std::scoped_lock lock(outputs_mutex_);
    return outputs_.back();
  }

  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(), 3};
  std::shared_ptr<estop_gate::EstopGateNode> gate_;
  rclcpp::Node::SharedPtr io_node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr output_subscription_;
  std::mutex outputs_mutex_;
  std::vector<geometry_msgs::msg::Twist> outputs_;
};

TEST_F(EstopGateComponentTest, ForwardsCommandWhileClear)
{
  publish_command(0.4);
  ASSERT_TRUE(wait_for_output());
  EXPECT_DOUBLE_EQ(last_output().linear.x, 0.4);
}

TEST_F(EstopGateComponentTest, LatchesStopAndBlocksCommands)
{
  publish_bool(estop_publisher_, true);
  ASSERT_TRUE(wait_for_output());
  EXPECT_DOUBLE_EQ(last_output().linear.x, 0.0);

  publish_command(0.4);
  ASSERT_TRUE(wait_for_output());
  EXPECT_DOUBLE_EQ(last_output().linear.x, 0.0);
}

TEST_F(EstopGateComponentTest, ResetAllowsCommandsAgain)
{
  publish_bool(estop_publisher_, true);
  ASSERT_TRUE(wait_for_output());

  publish_bool(reset_publisher_, true);
  spin_for(50ms);
  publish_command(0.4);
  ASSERT_TRUE(wait_for_output());
  EXPECT_DOUBLE_EQ(last_output().linear.x, 0.4);
}

TEST_F(EstopGateComponentTest, HandlesConcurrentControlTraffic)
{
  constexpr int message_count = 100;
  std::thread executor_thread([this] { executor_.spin(); });
  std::thread estop_thread([this] {
    for (int i = 0; i < message_count; ++i) {
      publish_bool(estop_publisher_, true);
    }
  });
  std::thread reset_thread([this] {
    for (int i = 0; i < message_count; ++i) {
      publish_bool(reset_publisher_, true);
    }
  });
  std::thread command_thread([this] {
    for (int i = 0; i < message_count; ++i) {
      publish_command(0.4);
    }
  });

  estop_thread.join();
  reset_thread.join();
  command_thread.join();
  std::this_thread::sleep_for(300ms);

  clear_outputs();
  const auto before_stop = output_count();
  publish_bool(estop_publisher_, true);
  const bool received_stop = wait_for_output_after(before_stop, false);
  const auto before_command = output_count();
  publish_command(0.4);
  const bool received_blocked_command = wait_for_output_after(before_command, false);

  executor_.cancel();
  executor_thread.join();

  ASSERT_TRUE(received_stop);
  ASSERT_TRUE(received_blocked_command);
  EXPECT_DOUBLE_EQ(last_output().linear.x, 0.0);
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
