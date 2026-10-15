#include "legbot_exploration/exploration_core.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace le = legbot_exploration;
using namespace std::chrono_literals;

namespace {
double xy_distance(const le::XY &a, const le::XY &b) {
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}
double wall_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
}  // namespace

class Explorer : public rclcpp::Node {
 public:
  Explorer() : Node("legbot_explorer"), map_(declare_settings()) {
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/fast_lio/cloud_registered");
    auto sensor_topic = declare_parameter<std::string>("sensor_odom_topic", "/fast_lio/odometry_lidar");
    auto base_topic = declare_parameter<std::string>("base_odom_topic", "/fast_lio/odometry_base");
    auto waypoint_topic = declare_parameter<std::string>("waypoint_topic", "/scan/waypoint_input");
    auto goal_topic = declare_parameter<std::string>("scan_goal_topic", "/scan/goal");
    auto ready_topic = declare_parameter<std::string>("ready_topic", "/go2/demo_ready");
    point_stride_ = declare_parameter<int>("point_stride", 8);
    max_points_ = declare_parameter<int>("max_points_per_cloud", 1000);
    sync_tolerance_ = declare_parameter<double>("sensor_sync_tolerance", 0.03);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 1.0);
    cloud_timeout_ = declare_parameter<double>("cloud_timeout", 2.0);
    goal_reach_xy_ = declare_parameter<double>("goal_reach_xy", 0.55);
    progress_distance_ = declare_parameter<double>("progress_distance", 0.35);
    progress_timeout_ = declare_parameter<double>("progress_timeout", 12.0);
    goal_timeout_ = declare_parameter<double>("goal_timeout", 90.0);
    forward_timeout_ = declare_parameter<double>("goal_forward_timeout", 3.0);
    cooldown_ = declare_parameter<double>("failed_frontier_cooldown", 45.0);
    no_frontier_hold_ = declare_parameter<double>("no_frontier_hold", 8.0);
    observation_only_ = declare_parameter<bool>("observation_only", true);
    if (cfg_.resolution <= 0.0 || cfg_.local_search_radius <= 0.0 || point_stride_ < 1 ||
        max_points_ < 1 || cfg_.max_voxels < 1) {
      throw std::invalid_argument("resolution, search radius, stride, max points and max voxels must be positive");
    }
    auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(waypoint_topic, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/exploration/frontiers", state_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("/exploration/status", state_qos);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_cloud(msg); });
    sensor_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        sensor_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_sensor(*msg); });
    base_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        base_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_base(*msg); });
    ready_sub_ = create_subscription<std_msgs::msg::Bool>(
        ready_topic, state_qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) { ready_ = msg->data; });
    input_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        waypoint_topic, 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { on_input(*msg); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { on_goal(*msg); });
    ack_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/scan/goal_stop_ack", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { on_ack(*msg); });
    complete_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/scan/mission_complete", state_qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) { on_complete(*msg); });
    pause_service_ = create_service<std_srvs::srv::SetBool>(
        "/exploration/set_paused",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          if (request->data && active_ && !clearing_) start_clear("paused");
          paused_ = request->data;
          response->success = true;
          response->message = paused_ ? "exploration paused" : "exploration resumed";
        });
    clear_client_ = create_client<std_srvs::srv::Trigger>("/scan/waypoints/clear");
    timer_ = create_wall_timer(1s, [this]() { tick(); });
    report(observation_only_ ? "observation_only" : "waiting_for_data");
  }

 private:
  struct Active {
    le::XY xy;
    double sent = 0.0;
    int64_t input_stamp = 0;
    std::optional<int64_t> stamp;
    double best_distance = 0.0;
    double progress_wall = 0.0;
    bool ack = false;
    bool armed = false;
    bool complete = false;
  };
  struct Suppressed { le::XY xy; double expiry; };

  le::Settings declare_settings() {
    le::Settings cfg;
    cfg.resolution = declare_parameter<double>("resolution", cfg.resolution);
    cfg.max_ray_range = declare_parameter<double>("max_ray_range", cfg.max_ray_range);
    int max_voxels = declare_parameter<int>("max_voxels", 900000);
    if (max_voxels < 1) throw std::invalid_argument("max_voxels must be positive");
    cfg.max_voxels = static_cast<std::size_t>(max_voxels);
    cfg.obstacle_min_relative_z = declare_parameter<double>(
        "obstacle_min_relative_z", cfg.obstacle_min_relative_z);
    cfg.obstacle_max_relative_z = declare_parameter<double>(
        "obstacle_max_relative_z", cfg.obstacle_max_relative_z);
    cfg.free_min_relative_z = declare_parameter<double>(
        "free_min_relative_z", cfg.free_min_relative_z);
    cfg.robot_radius = declare_parameter<double>("robot_radius", cfg.robot_radius);
    cfg.local_search_radius = declare_parameter<double>(
        "local_search_radius", cfg.local_search_radius);
    cfg.min_goal_distance = declare_parameter<double>("min_goal_distance", cfg.min_goal_distance);
    cfg.max_goal_distance = declare_parameter<double>("max_goal_distance", cfg.max_goal_distance);
    cfg.min_frontier_cells = declare_parameter<int>("min_frontier_cells", cfg.min_frontier_cells);
    cfg.frontier_gain_weight = declare_parameter<double>(
        "frontier_gain_weight", cfg.frontier_gain_weight);
    cfg.path_cost_weight = declare_parameter<double>("path_cost_weight", cfg.path_cost_weight);
    cfg.unknown_path_multiplier = declare_parameter<double>(
        "unknown_path_multiplier", cfg.unknown_path_multiplier);
    cfg_ = cfg;
    return cfg;
  }
  void report(const std::string &state, const std::string &detail = "") {
    std::ostringstream stream;
    stream << "{\"state\":\"" << state << "\",\"implementation\":\"cpp\","
           << "\"map_version\":" << map_.version() << ",\"voxel_count\":" << map_.size()
           << ",\"paused\":" << (paused_ ? "true" : "false")
           << ",\"observation_only\":" << (observation_only_ ? "true" : "false");
    if (!detail.empty()) stream << ",\"detail\":\"" << detail << "\"";
    stream << '}';
    if (stream.str() == last_status_) return;
    std_msgs::msg::String msg;
    msg.data = stream.str();
    status_pub_->publish(msg);
    last_status_ = msg.data;
  }
  void on_sensor(const nav_msgs::msg::Odometry &msg) {
    if (msg.header.frame_id != frame_id_) return;
    const auto &p = msg.pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
    sensor_poses_.push_back({stamp_ns(msg.header.stamp), {p.x, p.y, p.z}});
    if (sensor_poses_.size() > 100) sensor_poses_.pop_front();
    process_clouds();
  }
  void on_base(const nav_msgs::msg::Odometry &msg) {
    if (msg.header.frame_id != frame_id_) return;
    const auto &p = msg.pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
    base_ = le::Point{p.x, p.y, p.z};
    base_wall_ = wall_now();
    process_clouds();
  }
  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (msg->header.frame_id != frame_id_) return;
    pending_clouds_.push_back(msg);
    if (pending_clouds_.size() > 5) pending_clouds_.pop_front();
    process_clouds();
  }
  void process_clouds() {
    if (!base_ || sensor_poses_.empty() || wall_now() - base_wall_ > odom_timeout_) return;
    for (auto it = pending_clouds_.begin(); it != pending_clouds_.end();) {
      int64_t stamp = stamp_ns((*it)->header.stamp);
      if (integrate_cloud(**it)) it = pending_clouds_.erase(it);
      else if (sensor_poses_.back().first - stamp > sync_tolerance_ * 1e9)
        it = pending_clouds_.erase(it);
      else ++it;
    }
  }
  bool integrate_cloud(const sensor_msgs::msg::PointCloud2 &msg) {
    int64_t stamp = stamp_ns(msg.header.stamp);
    auto nearest = std::min_element(sensor_poses_.begin(), sensor_poses_.end(),
                                    [&](const auto &a, const auto &b) {
                                      return std::llabs(a.first - stamp) < std::llabs(b.first - stamp);
                                    });
    if (std::llabs(nearest->first - stamp) > sync_tolerance_ * 1e9) return false;
    bool x_found = false, y_found = false, z_found = false;
    for (const auto &field : msg.fields) {
      if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) x_found = true;
      if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) y_found = true;
      if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) z_found = true;
    }
    if (!x_found || !y_found || !z_found) return true;
    std::vector<le::Point> points;
    points.reserve(max_points_);
    sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
    const std::size_t total = static_cast<std::size_t>(msg.width) * msg.height;
    for (std::size_t i = 0; i < total && static_cast<int>(points.size()) < max_points_;
         ++i, ++x, ++y, ++z) {
      if (i % static_cast<std::size_t>(point_stride_) == 0)
        points.push_back({*x, *y, *z});
    }
    if (map_.integrate(nearest->second, points, (*base_)[2])) cloud_wall_ = wall_now();
    return true;
  }
  void on_input(const geometry_msgs::msg::PoseStamped &msg) {
    le::XY xy{msg.pose.position.x, msg.pose.position.y};
    if (active_ && xy_distance(xy, active_->xy) < 0.05 &&
        stamp_ns(msg.header.stamp) == active_->input_stamp) return;
    if (!clearing_) {
      active_.reset();
      paused_ = true;
      report("paused_for_manual_goal");
    }
  }
  void on_goal(const geometry_msgs::msg::PoseStamped &msg) {
    if (clearing_ || msg.header.frame_id != frame_id_) return;
    le::XY xy{msg.pose.position.x, msg.pose.position.y};
    if (base_ && wall_now() - last_clear_wall_ < 2.0 &&
        xy_distance(xy, {(*base_)[0], (*base_)[1]}) < 0.25) return;
    if (!active_) {
      paused_ = true;
      report("paused_for_manual_goal");
      return;
    }
    if (xy_distance(xy, active_->xy) < 0.05) {
      if (stamp_ns(msg.header.stamp) < active_->input_stamp) return;
      active_->stamp = stamp_ns(msg.header.stamp);
    } else {
      active_.reset();
      paused_ = true;
      report("paused_for_manual_goal");
    }
  }
  void on_ack(const geometry_msgs::msg::PoseStamped &msg) {
    if (!active_ || !active_->stamp || stamp_ns(msg.header.stamp) != *active_->stamp) return;
    if (msg.header.frame_id != frame_id_) return;
    if (xy_distance({msg.pose.position.x, msg.pose.position.y}, active_->xy) > goal_reach_xy_) return;
    active_->ack = true;
    finish_if_complete();
  }
  void on_complete(const std_msgs::msg::Bool &msg) {
    if (!active_) return;
    if (!msg.data) active_->armed = true;
    else if (active_->armed) {
      active_->complete = true;
      finish_if_complete();
    }
  }
  void finish_if_complete() {
    if (active_ && active_->ack && active_->complete) {
      RCLCPP_INFO(get_logger(), "Frontier goal reached: %.2f, %.2f", active_->xy[0], active_->xy[1]);
      active_.reset();
      empty_since_.reset();
      report("goal_reached");
    }
  }
  void request_clear() {
    if (!clearing_ || clear_inflight_ || !clear_client_->service_is_ready()) return;
    clear_inflight_ = true;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    clear_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          clear_inflight_ = false;
          try {
            if (future.get()->success) {
              clearing_ = false;
              last_clear_wall_ = wall_now();
              active_.reset();
              report("goal_cleared");
            }
          } catch (const std::exception &exc) {
            RCLCPP_WARN(get_logger(), "Waypoint clear failed: %s", exc.what());
          }
        });
  }
  void start_clear(const std::string &reason) {
    if (!active_ || clearing_) return;
    if (reason != "paused") suppressed_.push_back({active_->xy, wall_now() + cooldown_});
    clearing_ = true;
    report("clearing_goal", reason);
    request_clear();
  }
  void publish_target(const le::XY &xy) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = xy[0];
    msg.pose.position.y = xy[1];
    // SCAN's same-floor manual-goal interface uses XY and rejects z < -0.1.
    msg.pose.position.z = 0.0;
    msg.pose.orientation.w = 1.0;
    double current = wall_now();
    active_ = Active{xy, current, stamp_ns(msg.header.stamp), std::nullopt,
                     xy_distance(xy, {(*base_)[0], (*base_)[1]}), current,
                     false, false, false};
    waypoint_pub_->publish(msg);
    report("goal_sent");
  }
  void publish_marker(const std::optional<le::XY> &goal) {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "selected_frontier";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = goal ? visualization_msgs::msg::Marker::ADD :
                           visualization_msgs::msg::Marker::DELETE;
    marker.pose.orientation.w = 1.0;
    if (goal) {
      marker.pose.position.x = (*goal)[0];
      marker.pose.position.y = (*goal)[1];
      marker.pose.position.z = (*base_)[2];
      marker.scale.x = marker.scale.y = marker.scale.z = 0.35;
      marker.color.r = 0.0; marker.color.g = 0.9; marker.color.b = 1.0; marker.color.a = 0.9;
    }
    array.markers.push_back(marker);
    marker_pub_->publish(array);
  }
  void tick() {
    double current = wall_now();
    if (clearing_) { request_clear(); return; }
    if (!base_ || current - base_wall_ > odom_timeout_ ||
        cloud_wall_ == 0.0 || current - cloud_wall_ > cloud_timeout_) {
      empty_since_.reset();
      report("waiting_for_fresh_data");
      return;
    }
    suppressed_.erase(std::remove_if(suppressed_.begin(), suppressed_.end(),
        [current](const Suppressed &s) { return s.expiry <= current; }), suppressed_.end());
    if (active_) {
      if (!active_->stamp) {
        if (current - active_->sent > forward_timeout_) {
          active_.reset();
          report("goal_not_forwarded");
        }
        return;
      }
      double remaining = xy_distance(active_->xy, {(*base_)[0], (*base_)[1]});
      if (remaining < active_->best_distance - progress_distance_) {
        active_->best_distance = remaining;
        active_->progress_wall = current;
      }
      if (current - active_->sent > goal_timeout_) start_clear("goal_timeout");
      else if (current - active_->progress_wall > progress_timeout_) start_clear("no_progress");
      return;
    }
    if (map_.size() >= cfg_.max_voxels) {
      report("map_capacity_reached");
      return;
    }
    auto projected = map_.project((*base_)[2], {(*base_)[0], (*base_)[1]});
    bool has_free = std::any_of(projected.begin(), projected.end(),
                                [](const auto &item) { return item.second == 0; });
    if (!has_free) {
      report("waiting_for_free_space");
      return;
    }
    std::vector<le::XY> failed;
    for (const auto &s : suppressed_) failed.push_back(s.xy);
    auto selection = le::select_frontier(map_, *base_, cfg_, failed);
    publish_marker(selection.goal);
    if (!selection.goal) {
      if (selection.clusters > 0) {
        empty_since_.reset();
        report("frontiers_unreachable_or_suppressed");
      } else {
        if (!empty_since_) empty_since_ = current;
        report(current - *empty_since_ >= no_frontier_hold_ ?
               "current_region_complete" : "waiting_for_frontier");
      }
      return;
    }
    empty_since_.reset();
    if (observation_only_) report("observation_only");
    else if (paused_) report("paused");
    else if (!ready_ || waypoint_pub_->get_subscription_count() == 0)
      report("waiting_for_waypoint_coordinator");
    else publish_target(*selection.goal);
  }

  le::Settings cfg_;
  le::VoxelMap map_;
  std::string frame_id_, last_status_;
  int point_stride_ = 8, max_points_ = 1000;
  double sync_tolerance_ = 0.03, odom_timeout_ = 1.0, cloud_timeout_ = 2.0;
  double goal_reach_xy_ = 0.55, progress_distance_ = 0.35, progress_timeout_ = 12.0;
  double goal_timeout_ = 90.0, forward_timeout_ = 3.0, cooldown_ = 45.0;
  double no_frontier_hold_ = 8.0;
  bool observation_only_ = true, ready_ = false, paused_ = false;
  bool clearing_ = false, clear_inflight_ = false;
  double base_wall_ = 0.0, cloud_wall_ = 0.0, last_clear_wall_ = 0.0;
  std::optional<le::Point> base_;
  std::optional<Active> active_;
  std::optional<double> empty_since_;
  std::deque<std::pair<int64_t, le::Point>> sensor_poses_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_clouds_;
  std::vector<Suppressed> suppressed_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sensor_sub_, base_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_, complete_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input_sub_, goal_sub_, ack_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Explorer>());
  } catch (const std::exception &exc) {
    RCLCPP_ERROR(rclcpp::get_logger("legbot_exploration"), "Explorer failed: %s", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
