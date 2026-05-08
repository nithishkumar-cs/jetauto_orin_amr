#include <amr_interfaces/msg/node_health.hpp>
#include <amr_interfaces/msg/obstacle.hpp>
#include <amr_interfaces/msg/obstacle_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

namespace tracking_fusion
{
namespace
{

double distance_xy(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double stamp_to_sec(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

class ObstacleTrackerNode final : public rclcpp::Node
{
public:
  explicit ObstacleTrackerNode(const rclcpp::NodeOptions& options)
  : Node("obstacle_tracker_node", options)
  {
    association_distance_m_ = declare_parameter<double>("association_distance_m", 0.75);
    max_track_age_s_ = declare_parameter<double>("max_track_age_s", 0.80);
    velocity_alpha_ = declare_parameter<double>("velocity_alpha", 0.35);
    dynamic_speed_threshold_mps_ = declare_parameter<double>("dynamic_speed_threshold_mps", 0.20);

    const auto input_topic = declare_parameter<std::string>("input_topic", "/perception/obstacles_3d");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/perception/tracked_obstacles");

    pub_ = create_publisher<amr_interfaces::msg::ObstacleArray>(output_topic, 10);
    health_pub_ = create_publisher<amr_interfaces::msg::NodeHealth>("/diagnostics/tracking/health", 10);
    sub_ = create_subscription<amr_interfaces::msg::ObstacleArray>(
      input_topic,
      10,
      std::bind(&ObstacleTrackerNode::obstacles_callback, this, std::placeholders::_1));
  }

private:
  struct Track
  {
    amr_interfaces::msg::Obstacle obstacle;
    double last_seen_s{0.0};
  };

  void obstacles_callback(const amr_interfaces::msg::ObstacleArray::ConstSharedPtr msg)
  {
    const double now_s = stamp_to_sec(msg->header.stamp);
    std::unordered_set<std::size_t> matched_tracks;

    for (const auto& detection : msg->obstacles) {
      const auto best = nearest_track(detection);
      if (best.index < tracks_.size() && best.distance <= association_distance_m_) {
        update_track(tracks_[best.index], detection, now_s);
        matched_tracks.insert(best.index);
      } else {
        Track track;
        track.obstacle = detection;
        track.obstacle.track_id = next_track_id_++;
        track.last_seen_s = now_s;
        tracks_.push_back(std::move(track));
      }
    }

    prune_tracks(now_s);
    publish_tracks(*msg);
    publish_health(amr_interfaces::msg::NodeHealth::OK, "tracking active");
  }

  struct Match
  {
    std::size_t index{std::numeric_limits<std::size_t>::max()};
    double distance{std::numeric_limits<double>::infinity()};
  };

  Match nearest_track(const amr_interfaces::msg::Obstacle& detection) const
  {
    Match best;
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      if (tracks_[i].obstacle.class_id != detection.class_id) {
        continue;
      }
      const auto distance = distance_xy(tracks_[i].obstacle.position, detection.position);
      if (distance < best.distance) {
        best.index = i;
        best.distance = distance;
      }
    }
    return best;
  }

  void update_track(Track& track, const amr_interfaces::msg::Obstacle& detection, double now_s)
  {
    const double dt = std::max(1e-3, now_s - track.last_seen_s);
    const auto previous = track.obstacle.position;
    const auto previous_velocity = track.obstacle.velocity;
    const auto track_id = track.obstacle.track_id;

    track.obstacle = detection;
    track.obstacle.track_id = track_id;
    track.obstacle.velocity.x =
      velocity_alpha_ * ((detection.position.x - previous.x) / dt) +
      (1.0 - velocity_alpha_) * previous_velocity.x;
    track.obstacle.velocity.y =
      velocity_alpha_ * ((detection.position.y - previous.y) / dt) +
      (1.0 - velocity_alpha_) * previous_velocity.y;
    track.obstacle.velocity.z =
      velocity_alpha_ * ((detection.position.z - previous.z) / dt) +
      (1.0 - velocity_alpha_) * previous_velocity.z;

    const double speed_xy = std::hypot(track.obstacle.velocity.x, track.obstacle.velocity.y);
    track.obstacle.dynamic = detection.dynamic || speed_xy >= dynamic_speed_threshold_mps_;
    track.last_seen_s = now_s;
  }

  void prune_tracks(double now_s)
  {
    tracks_.erase(
      std::remove_if(
        tracks_.begin(),
        tracks_.end(),
        [&](const Track& track) { return now_s - track.last_seen_s > max_track_age_s_; }),
      tracks_.end());
  }

  void publish_tracks(const amr_interfaces::msg::ObstacleArray& input)
  {
    amr_interfaces::msg::ObstacleArray output;
    output.header = input.header;
    output.obstacles.reserve(tracks_.size());
    for (const auto& track : tracks_) {
      output.obstacles.push_back(track.obstacle);
    }
    pub_->publish(std::move(output));
  }

  void publish_health(uint8_t status, const std::string& message)
  {
    amr_interfaces::msg::NodeHealth health;
    health.header.stamp = now();
    health.node_name = get_name();
    health.status = status;
    health.message = message;
    health_pub_->publish(std::move(health));
  }

  double association_distance_m_{0.75};
  double max_track_age_s_{0.80};
  double velocity_alpha_{0.35};
  double dynamic_speed_threshold_mps_{0.20};
  uint32_t next_track_id_{1};
  std::vector<Track> tracks_;

  rclcpp::Subscription<amr_interfaces::msg::ObstacleArray>::SharedPtr sub_;
  rclcpp::Publisher<amr_interfaces::msg::ObstacleArray>::SharedPtr pub_;
  rclcpp::Publisher<amr_interfaces::msg::NodeHealth>::SharedPtr health_pub_;
};

}  // namespace tracking_fusion

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tracking_fusion::ObstacleTrackerNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
