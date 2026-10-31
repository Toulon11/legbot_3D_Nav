#include "legbot_exploration/multi_layer_map.hpp"
#include "legbot_exploration/surface_graph.hpp"
#include "legbot_exploration/exploration_node_factories.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace le = legbot_exploration;
using namespace std::chrono_literals;

namespace {
double seconds_apart(const builtin_interfaces::msg::Time &a,
                     const builtin_interfaces::msg::Time &b) {
  return std::abs(static_cast<double>(a.sec - b.sec) +
                  1e-9 * static_cast<double>(static_cast<int64_t>(a.nanosec) - b.nanosec));
}

uint32_t color(const le::LayerSurface &surface) {
  if (surface.clearance == le::ClearanceKind::blocked) return 0xff3030u;
  switch (surface.kind) {
    case le::SurfaceKind::flat:
      return surface.clearance == le::ClearanceKind::observed_clear ? 0x20c860u : 0x3080f0u;
    case le::SurfaceKind::step_candidate: return 0xffbb30u;
    case le::SurfaceKind::steep_or_disconnected: return 0xd040d0u;
    case le::SurfaceKind::insufficient_support: return 0x909090u;
  }
  return 0x909090u;
}

sensor_msgs::msg::PointCloud2 make_cloud(
    const std::vector<le::LayerPoint> &points, const std::vector<uint32_t> &colors,
    const std_msgs::msg::Header &header) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z"), rgb(msg, "rgb");
  for (std::size_t i = 0; i < points.size(); ++i, ++x, ++y, ++z, ++rgb) {
    *x = static_cast<float>(points[i].x);
    *y = static_cast<float>(points[i].y);
    *z = static_cast<float>(points[i].z);
    float packed;
    std::memcpy(&packed, &colors[i], sizeof(packed));
    *rgb = packed;
  }
  msg.is_dense = true;
  return msg;
}

sensor_msgs::msg::PointCloud2 make_surface_cloud(
    const std::vector<le::LayerSurface> &surfaces, std::size_t stride,
    double xy_resolution, const std_msgs::msg::Header &header) {
  using sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  msg.height = 1;
  msg.width = static_cast<uint32_t>((surfaces.size() + stride - 1) / stride);
  msg.is_dense = true;
  msg.is_bigendian = false;
  msg.point_step = 40;
  msg.row_step = msg.width * msg.point_step;
  auto field = [&](const char *name, uint32_t offset, uint8_t datatype) {
    PointField item;
    item.name = name;
    item.offset = offset;
    item.datatype = datatype;
    item.count = 1;
    msg.fields.push_back(item);
  };
  field("x", 0, PointField::FLOAT32);
  field("y", 4, PointField::FLOAT32);
  field("z", 8, PointField::FLOAT32);
  field("rgb", 12, PointField::FLOAT32);
  field("slice_index", 16, PointField::INT32);
  field("surface_kind", 20, PointField::UINT8);
  field("clearance_kind", 21, PointField::UINT8);
  field("support_columns", 22, PointField::UINT16);
  field("ceiling_z", 24, PointField::FLOAT32);
  field("hit_observations", 28, PointField::UINT32);
  field("last_observed", 32, PointField::FLOAT64);
  msg.data.resize(msg.row_step);
  std::size_t out = 0;
  for (std::size_t i = 0; i < surfaces.size(); i += stride, ++out) {
    const auto &s = surfaces[i];
    uint8_t *p = msg.data.data() + out * msg.point_step;
    const auto write = [p](uint32_t offset, const auto &value) {
      std::memcpy(p + offset, &value, sizeof(value));
    };
    const float x = static_cast<float>((s.x_cell + 0.5) * xy_resolution);
    const float y = static_cast<float>((s.y_cell + 0.5) * xy_resolution);
    const float z = static_cast<float>(s.height);
    const uint32_t rgb = color(s);
    const int32_t slice = s.slice_index;
    const uint8_t kind = static_cast<uint8_t>(s.kind);
    const uint8_t clearance = static_cast<uint8_t>(s.clearance);
    const uint16_t support = static_cast<uint16_t>(std::min(s.support_columns, 65535));
    const float ceiling = static_cast<float>(s.ceiling);
    const uint32_t hits = static_cast<uint32_t>(std::max(0, s.hit_observations));
    const double last_observed = s.last_observed;
    write(0, x); write(4, y); write(8, z); write(12, rgb);
    write(16, slice); write(20, kind); write(21, clearance); write(22, support);
    write(24, ceiling); write(28, hits); write(32, last_observed);
  }
  return msg;
}

geometry_msgs::msg::Point marker_point(const le::LayerPoint &point, double lift = 0.0) {
  geometry_msgs::msg::Point result;
  result.x = point.x;
  result.y = point.y;
  result.z = point.z + lift;
  return result;
}

visualization_msgs::msg::Marker make_marker(
    const std_msgs::msg::Header &header, int id, int type,
    float red, float green, float blue, float alpha, double scale) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "surface_graph_preview";
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = alpha;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  return marker;
}
}  // namespace

class OnlineLayerMapNode : public rclcpp::Node {
 public:
  explicit OnlineLayerMapNode(std::shared_ptr<le::SharedLayerMap> shared = {})
      : Node("online_layer_map"), map_(read_config()), shared_(std::move(shared)) {
    if (shared_) shared_->map = &map_;
    xy_resolution_ = get_parameter("xy_resolution").as_double();
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const auto cloud_topic = declare_parameter<std::string>(
        "cloud_topic", "/slam/global_cloud_registered");
    const auto sensor_topic = declare_parameter<std::string>(
        "sensor_odom_topic", "/slam/odometry_lidar");
    const auto base_topic = declare_parameter<std::string>(
        "base_odom_topic", "/slam/global_odometry_base");
    point_stride_ = declare_parameter<int>("point_stride", 1);
    max_points_ = declare_parameter<int>("max_points_per_cloud", 12000);
    max_rays_ = declare_parameter<int>("max_rays_per_cloud", 1200);
    sync_tolerance_ = declare_parameter<double>("sensor_sync_tolerance", 0.05);
    publish_period_ = declare_parameter<double>("publish_period", 1.0);
    publish_map_topics_ = declare_parameter<bool>("publish_map_topics", true);
    max_display_surfaces_ = declare_parameter<int>("max_display_surfaces", 120000);
    graph_config_.xy_resolution = xy_resolution_;
    graph_config_.max_step_height = get_parameter("max_step_height").as_double();
    graph_config_.max_slope_radians = get_parameter("max_slope_radians").as_double();
    graph_config_.min_support_columns = get_parameter("min_support_columns").as_int();
    graph_config_.search_radius = declare_parameter<double>("graph_search_radius", 7.0);
    graph_config_.min_height_gain = declare_parameter<double>("graph_min_height_gain", 0.07);
    graph_config_.nominal_body_height = declare_parameter<double>("graph_nominal_body_height", 0.32);
    graph_config_.start_xy_tolerance = declare_parameter<double>("graph_start_xy_tolerance", 0.45);
    graph_config_.start_z_tolerance = declare_parameter<double>("graph_start_z_tolerance", 0.30);
    graph_config_.max_nodes = declare_parameter<int>("graph_max_nodes", 25000);
    graph_config_.max_preview_candidates = declare_parameter<int>("graph_max_preview_candidates", 128);
    graph_config_.max_preview_nodes = declare_parameter<int>("graph_max_preview_nodes", 4000);
    graph_config_.max_preview_edges = declare_parameter<int>("graph_max_preview_edges", 6000);
    if (point_stride_ < 1 || max_points_ < 1 || max_rays_ < 1 || sync_tolerance_ <= 0 ||
        publish_period_ <= 0 || max_display_surfaces_ < 1)
      throw std::invalid_argument("invalid layer map node configuration");

    auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    surface_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/exploration/layers/surfaces", map_qos);
    planning_surface_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/exploration/layers/planning_surfaces", map_qos);
    planning_occupied_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/exploration/layers/planning_occupied", map_qos);
    ceiling_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/exploration/layers/ceilings", map_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/exploration/layers/status", map_qos);
    graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/exploration/layers/graph_preview", map_qos);
    graph_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/exploration/layers/candidate_path", map_qos);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_cloud(*msg); });
    sensor_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        sensor_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          push_odom(sensor_history_, *msg);
        });
    base_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        base_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          push_odom(base_history_, *msg);
        });
    timer_ = create_wall_timer(std::chrono::duration<double>(publish_period_),
                               [this]() { publish_map(); });
    RCLCPP_INFO(get_logger(), "Online layer map observes %s in %s; no navigation goals are published",
                cloud_topic.c_str(), frame_id_.c_str());
  }

 private:
  le::LayerMapConfig read_config() {
    le::LayerMapConfig cfg;
    cfg.xy_resolution = declare_parameter<double>("xy_resolution", cfg.xy_resolution);
    cfg.z_resolution = declare_parameter<double>("z_resolution", cfg.z_resolution);
    cfg.slice_height = declare_parameter<double>("slice_height", cfg.slice_height);
    cfg.max_ray_range = declare_parameter<double>("max_ray_range", cfg.max_ray_range);
    cfg.surface_merge_gap = declare_parameter<double>("surface_merge_gap", cfg.surface_merge_gap);
    cfg.support_height_tolerance = declare_parameter<double>(
        "support_height_tolerance", cfg.support_height_tolerance);
    cfg.min_clearance = declare_parameter<double>("min_clearance", cfg.min_clearance);
    cfg.max_slope_radians = declare_parameter<double>("max_slope_radians", cfg.max_slope_radians);
    cfg.max_step_height = declare_parameter<double>("max_step_height", cfg.max_step_height);
    cfg.support_radius_cells = declare_parameter<int>(
        "support_radius_cells", cfg.support_radius_cells);
    cfg.min_support_columns = declare_parameter<int>(
        "min_support_columns", cfg.min_support_columns);
    cfg.min_hit_observations = declare_parameter<int>(
        "min_hit_observations", cfg.min_hit_observations);
    cfg.max_surfaces_per_column = declare_parameter<int>(
        "max_surfaces_per_column", cfg.max_surfaces_per_column);
    cfg.max_voxels = declare_parameter<int>("max_voxels", cfg.max_voxels);
    return cfg;
  }

  using Odom = nav_msgs::msg::Odometry;
  void push_odom(std::deque<Odom> &history, const Odom &odom) {
    if (odom.header.frame_id != frame_id_) return;
    history.push_back(odom);
    while (history.size() > 60) history.pop_front();
  }
  const Odom *nearest(const std::deque<Odom> &history,
                      const builtin_interfaces::msg::Time &stamp) const {
    const Odom *best = nullptr;
    double error = sync_tolerance_;
    for (const Odom &odom : history) {
      const double delta = seconds_apart(odom.header.stamp, stamp);
      if (delta <= error) { error = delta; best = &odom; }
    }
    return best;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2 &cloud) {
    ++cloud_messages_seen_;
    if (shared_ && shared_->planning_active.load()) {
      ++shared_->dropped_planning;
      return;
    }
    if (cloud.header.frame_id != frame_id_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cloud frame %s differs from layer map frame %s",
                           cloud.header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    const Odom *sensor = nearest(sensor_history_, cloud.header.stamp);
    const Odom *base = nearest(base_history_, cloud.header.stamp);
    if (!sensor || !base) { ++dropped_sync_; return; }
    const auto &p = sensor->pose.pose.position;
    const le::LayerPoint origin{p.x, p.y, p.z};
    std::vector<le::LayerPoint> points;
    points.reserve(max_points_);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      // Sample across the entire registered cloud instead of truncating its
      // first scan rings when the cloud has many points.
      const std::size_t count = static_cast<std::size_t>(cloud.width) * cloud.height;
      const std::size_t stride = std::max<std::size_t>(
          point_stride_, (count + static_cast<std::size_t>(max_points_) - 1) /
                             static_cast<std::size_t>(max_points_));
      std::size_t index = 0;
      for (; x != x.end() && points.size() < static_cast<std::size_t>(max_points_);
           ++x, ++y, ++z, ++index) {
        if (index % stride != 0) continue;
        points.push_back({*x, *y, *z});
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cannot read XYZ from cloud: %s", error.what());
      return;
    }
    if (points.empty()) return;
    const double stamp_seconds = static_cast<double>(cloud.header.stamp.sec) +
        static_cast<double>(cloud.header.stamp.nanosec) * 1e-9;
    const std::size_t ray_stride = std::max<std::size_t>(
        1, (points.size() + static_cast<std::size_t>(max_rays_) - 1) /
               static_cast<std::size_t>(max_rays_));
    const auto started = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> map_lock;
    if (shared_) {
      map_lock = std::unique_lock<std::shared_mutex>(shared_->mutex, std::try_to_lock);
      if (!map_lock.owns_lock() || shared_->planning_active.load()) {
        ++shared_->dropped_planning;
        return;
      }
    }
    integrated_points_ += map_.integrate(origin, points, stamp_seconds, ray_stride);
    if (shared_) shared_->current_version.store(map_.version());
    last_integration_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    last_raw_cloud_points_ = static_cast<std::size_t>(cloud.width) * cloud.height;
    last_sampled_points_ = points.size();
    last_rays_requested_ = (points.size() + ray_stride - 1) / ray_stride;
    ++clouds_received_;
  }

  void publish_map() {
    const auto started = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> map_lock;
    if (shared_) {
      if (shared_->planning_active.load()) return;
      map_lock = std::unique_lock<std::shared_mutex>(shared_->mutex);
      if (shared_->planning_active.load()) return;
    }
    const std::size_t rebuilt = map_.rebuild_dirty();
    const auto all = publish_map_topics_ ? map_.surfaces() :
        std::vector<le::LayerSurface>{};
    const auto occupied = publish_map_topics_ ? map_.occupied_voxels() :
        std::vector<le::LayerPoint>{};
    const le::LayerMapStats stats = map_.stats();
    if (map_lock.owns_lock()) map_lock.unlock();
    if (shared_) {
      shared_->published_surfaces.store(stats.surfaces);
      shared_->published_wall_seconds.store(
          std::chrono::duration<double>(std::chrono::steady_clock::now()
              .time_since_epoch()).count());
    }
    const std::size_t stride = std::max<std::size_t>(
        1, (all.size() + static_cast<std::size_t>(max_display_surfaces_) - 1) /
               static_cast<std::size_t>(max_display_surfaces_));
    std::vector<le::LayerPoint> ceilings;
    std::vector<uint32_t> ceiling_colors;
    for (std::size_t i = 0; i < all.size(); i += stride) {
      const le::LayerSurface &surface = all[i];
      const double x = (surface.x_cell + 0.5) * xy_resolution_;
      const double y = (surface.y_cell + 0.5) * xy_resolution_;
      if (std::isfinite(surface.ceiling)) {
        ceilings.push_back({x, y, surface.ceiling});
        ceiling_colors.push_back(0x40e0e0u);
      }
    }
    std_msgs::msg::Header header;
    header.frame_id = frame_id_;
    header.stamp = now();
    if (publish_map_topics_) {
      surface_pub_->publish(make_surface_cloud(all, stride, xy_resolution_, header));
      // Display decimation never changes the full planning topics.
      planning_surface_pub_->publish(make_surface_cloud(all, 1, xy_resolution_, header));
      planning_occupied_pub_->publish(make_cloud(
          occupied, std::vector<uint32_t>(occupied.size(), 0xff3030u), header));
      ceiling_pub_->publish(make_cloud(ceilings, ceiling_colors, header));
    }
    le::SurfaceGraphPreview graph;
    if (!publish_map_topics_) {
      graph.state = "map_topics_disabled";
    } else if (base_history_.empty()) {
      graph.state = "waiting_for_base_odometry";
    } else if (seconds_apart(base_history_.back().header.stamp, header.stamp) > 2.0) {
      graph.state = "base_odometry_stale";
    } else {
      const auto &p = base_history_.back().pose.pose.position;
      graph = le::search_rising_surface(all, {p.x, p.y, p.z}, graph_config_);
    }
    if (publish_map_topics_) publish_graph(graph, header);
    ++publish_cycles_;
    if (stats.version == 0 && (publish_cycles_ == 1 || publish_cycles_ % 10 == 0)) {
      RCLCPP_WARN(get_logger(),
                  "Waiting for synchronized map inputs: cloud messages=%llu, "
                  "lidar odometry samples=%zu, base odometry samples=%zu, "
                  "dropped synchronization=%llu. Start Gazebo and one localization adapter first.",
                  static_cast<unsigned long long>(cloud_messages_seen_),
                  sensor_history_.size(), base_history_.size(),
                  static_cast<unsigned long long>(dropped_sync_));
    }
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    std::ostringstream json;
    json << "{\"version\":" << stats.version << ",\"voxels\":" << stats.voxels
         << ",\"columns\":" << stats.columns << ",\"surfaces\":" << stats.surfaces
         << ",\"flat\":" << stats.flat << ",\"step_candidates\":"
         << stats.step_candidates << ",\"insufficient_support\":"
         << stats.insufficient_support << ",\"blocked_clearance\":"
         << stats.blocked_clearance << ",\"unobserved_clearance\":"
         << stats.unobserved_clearance << ",\"map_full\":"
         << (stats.map_full ? "true" : "false") << ",\"clouds_received\":"
         << clouds_received_ << ",\"dropped_sync\":" << dropped_sync_
         << ",\"integrated_points\":" << integrated_points_
         << ",\"cloud_messages_seen\":" << cloud_messages_seen_
         << ",\"dropped_planning\":"
         << (shared_ ? shared_->dropped_planning.load() : 0)
         << ",\"last_layer_planning_ms\":"
         << (shared_ ? shared_->last_planning_ms.load() : 0.0)
         << ",\"last_raw_cloud_points\":" << last_raw_cloud_points_
         << ",\"last_sampled_points\":" << last_sampled_points_
         << ",\"last_rays_requested\":" << last_rays_requested_
         << ",\"integration_ms\":" << last_integration_ms_
         << ",\"rebuilt_columns\":" << rebuilt << ",\"update_ms\":" << elapsed
         << ",\"graph_state\":\"" << graph.state
         << "\",\"graph_nodes\":" << graph.graph_nodes
         << ",\"graph_edges\":" << graph.graph_edges
         << ",\"graph_nodes_truncated\":"
         << (graph.nodes_truncated ? "true" : "false")
         << ",\"graph_rising_candidates\":" << graph.rising_candidates
         << ",\"graph_expanded_nodes\":" << graph.expanded_nodes
         << ",\"graph_path_points\":" << graph.path.size()
         << ",\"graph_path_clearance_unobserved\":"
         << (graph.contains_unobserved_clearance ? "true" : "false") << '}';
    std_msgs::msg::String status;
    status.data = json.str();
    status_pub_->publish(status);
  }

  void publish_graph(const le::SurfaceGraphPreview &graph,
                     const std_msgs::msg::Header &header) {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray message;
    // Replacing the complete preview also clears a stale path after a map edit.
    Marker clear;
    clear.header = header;
    clear.action = Marker::DELETEALL;
    message.markers.push_back(clear);
    auto nodes = make_marker(header, 0, Marker::POINTS, 0.75f, 0.75f, 0.75f, 0.45f, 0.035);
    for (const auto &point : graph.preview_nodes)
      nodes.points.push_back(marker_point(point, 0.025));
    message.markers.push_back(std::move(nodes));
    auto edges = make_marker(header, 1, Marker::LINE_LIST, 0.15f, 0.8f, 0.35f, 0.45f, 0.015);
    for (const auto &[from, to] : graph.preview_edges) {
      edges.points.push_back(marker_point(from, 0.035));
      edges.points.push_back(marker_point(to, 0.035));
    }
    message.markers.push_back(std::move(edges));
    auto candidates = make_marker(header, 2, Marker::SPHERE_LIST,
                                  1.0f, 0.65f, 0.0f, 0.9f, 0.11);
    for (const auto &point : graph.preview_candidates)
      candidates.points.push_back(marker_point(point, 0.075));
    message.markers.push_back(std::move(candidates));
    auto route = make_marker(header, 3, Marker::LINE_STRIP,
                             1.0f, 0.1f, 0.9f, 1.0f, 0.075);
    for (const auto &point : graph.path)
      route.points.push_back(marker_point(point, 0.10));
    message.markers.push_back(std::move(route));
    graph_pub_->publish(message);

    nav_msgs::msg::Path path;
    path.header = header;
    for (const auto &point : graph.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z + 0.10;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(std::move(pose));
    }
    graph_path_pub_->publish(path);
  }

  le::OnlineLayerMap map_;
  std::shared_ptr<le::SharedLayerMap> shared_;
  le::SurfaceGraphConfig graph_config_;
  double xy_resolution_ = 0.10;
  std::string frame_id_;
  int point_stride_ = 1, max_points_ = 12000, max_rays_ = 1200;
  int max_display_surfaces_ = 120000;
  bool publish_map_topics_ = true;
  double sync_tolerance_ = 0.05, publish_period_ = 1.0;
  double last_integration_ms_ = 0.0;
  uint64_t clouds_received_ = 0, dropped_sync_ = 0, integrated_points_ = 0;
  uint64_t cloud_messages_seen_ = 0, publish_cycles_ = 0;
  std::size_t last_raw_cloud_points_ = 0, last_sampled_points_ = 0;
  std::size_t last_rays_requested_ = 0;
  std::deque<Odom> sensor_history_, base_history_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Odom>::SharedPtr sensor_sub_, base_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_pub_, ceiling_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planning_surface_pub_,
      planning_occupied_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr graph_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<rclcpp::Node> make_online_layer_map_node(
    std::shared_ptr<le::SharedLayerMap> shared) {
  return std::make_shared<OnlineLayerMapNode>(std::move(shared));
}
