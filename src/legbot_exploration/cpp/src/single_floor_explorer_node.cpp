#include "legbot_exploration/single_floor_exploration.hpp" // 包含项目核心头文件，提供地图和设置等功能
#include "legbot_exploration/same_floor_route_planner.hpp"
#include "legbot_exploration/route_orchestrator.hpp"
#include "legbot_exploration/recovery_planner.hpp"
#include "legbot_exploration/exploration_node_factories.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp> // ROS2 消息：姿态（位置+方向）
#include <geometry_msgs/msg/twist.hpp> // 起步原地转向请求
#include <nav_msgs/msg/odometry.hpp> // ROS2 消息：里程计信息
#include <nav_msgs/msg/occupancy_grid.hpp> // 全局逐格验收地图
#include <nav_msgs/msg/path.hpp> // 实际走过的机体轨迹
#include <rclcpp/rclcpp.hpp> // ROS2 C++ 客户端库
#include <scan_planner/msg/bspline.hpp> // SCAN 实际下发的 B 样条轨迹
#include <sensor_msgs/msg/point_cloud2.hpp> // ROS2 点云消息类型
#include <sensor_msgs/point_cloud2_iterator.hpp> // 点云迭代器工具
#include <std_msgs/msg/bool.hpp> // 布尔型标准消息
#include <std_msgs/msg/int32.hpp> // GO2 步态模式
#include <std_msgs/msg/string.hpp> // 字符串标准消息
#include <std_srvs/srv/set_bool.hpp> // 设置布尔服务（用于暂停）
#include <visualization_msgs/msg/marker_array.hpp> // RViz 可视化 Marker

#include <algorithm> // 常用算法库
#include <array>
#include <charconv> // 精确解析 SCAN 目标时间戳
#include <chrono> // 时间工具
#include <cmath> // 数学函数（hypot）
#include <cstdint> // 固定宽度整数类型
#include <deque> // 双端队列容器
#include <iomanip> // 保留倒退目标的坐标精度
#include <limits> // 数值极限
#include <map> // 按拒绝原因汇总本轮脱困规划结果
#include <memory> // 智能指针
#include <optional> // 可选值类型
#include <sstream> // 字符流，用于构造 JSON
#include <cstring> // 字符串操作
#include <utility> // std::pair 等
#include <vector> // 动态数组容器

namespace le = legbot_exploration; // 项目命名空间别名，简化代码
using namespace std::chrono_literals; // 允许使用 1s、500ms 等时间字面量

// 匿名命名空间：仅在本文件内部使用的工具函数，防止外部链接冲突
// 进入本包的 C++ 命名空间，避免与别的包重名。
namespace {
  // 计算二维欧氏距离，返回 double 精度
  double xy_distance(const le::XY &a, const le::XY &b) {
    // 返回水平面两点的欧氏距离。
    return std::hypot(a[0] - b[0], a[1] - b[1]);
  }
  // 获取单调时钟的当前时间（秒），用于所有超时检测
  double wall_now() {
    // 把此函数算出的值交还给调用者。
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  // ROS Time（sec + nanosec）转纳秒整数，便于比较大小
  int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp) {
    // 把此函数算出的值交还给调用者。
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }
} // namespace

// ===========================================================================
// Explorer 节点：完成地图维护、前沿搜索、目标发布、状态上报等功能
// ===========================================================================
// 定义保存相关数据或行为的 C++ 类型。
class Explorer : public rclcpp::Node {
 public:
   explicit Explorer(std::shared_ptr<le::SharedLayerMap> shared = {})
       : Node("legbot_explorer"),               // ROS2 节点名
         map_(declare_settings()), shared_layer_(std::move(shared)) { // 读取参数并创建体素地图

     // ---------- 参数读取 ----------  RMK:fastlivo2发布是话题名称需要与参数一致 后面需要修改
     frame_id_ = declare_parameter<std::string>("frame_id", "map"); // 坐标系 ID
     auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/slam/global_cloud_registered"); // 点云主题名
     auto sensor_topic = declare_parameter<std::string>("sensor_odom_topic", "/slam/odometry_lidar"); // LiDAR 里程计主题
     auto base_topic = declare_parameter<std::string>("base_odom_topic", "/slam/global_odometry_base"); // 底盘里程计主题
     auto waypoint_topic = declare_parameter<std::string>("waypoint_topic", "/scan/waypoint_input"); // 手动 waypoint 主题
     auto goal_topic = declare_parameter<std::string>("scan_goal_topic", "/scan/goal"); // 上层规划目标主题
     auto ready_topic = declare_parameter<std::string>("ready_topic", "/go2/demo_ready"); // 探测准备状态主题
     auto occupancy_topic = declare_parameter<std::string>(
         "planner_occupancy_topic", "/local_planner/grid_map/occupancy_inflate");
     auto raw_occupancy_topic = declare_parameter<std::string>(
         "planner_raw_occupancy_topic", "/local_planner/grid_map/occupancy");
     point_stride_ = declare_parameter<int>("point_stride", 8);                       // 点云抽稀步长
     max_points_ = declare_parameter<int>("max_points_per_cloud", 1000);           // 单帧最大采点数
     sync_tolerance_ = declare_parameter<double>("sensor_sync_tolerance", 0.03);   // 同步容差（秒）
     odom_timeout_ = declare_parameter<double>("odom_timeout", 1.0);
     cloud_timeout_ = declare_parameter<double>("cloud_timeout", 2.0);
     goal_reach_xy_ = declare_parameter<double>("goal_reach_xy", 0.55);
     cooldown_ = declare_parameter<double>("failed_frontier_cooldown", 45.0);
     low_yield_retry_cooldown_ = declare_parameter<double>("low_yield_retry_cooldown", 8.0);
     no_frontier_hold_ = declare_parameter<double>("no_frontier_hold", 8.0);
     int recovery_min_new_cells = declare_parameter<int>("recovery_min_new_cells", 8);
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (recovery_min_new_cells < 1)
       // 发现非法输入，抛出错误并停止当前流程。
       throw std::invalid_argument("recovery_min_new_cells must be positive");
     if (!std::isfinite(low_yield_retry_cooldown_) || low_yield_retry_cooldown_ < 0.0)
       throw std::invalid_argument("low_yield_retry_cooldown must be nonnegative");
     min_new_cells_ = static_cast<std::size_t>(recovery_min_new_cells);
     goal_progress_gate_ = le::GoalProgressGate(static_cast<std::size_t>(recovery_min_new_cells));
     occupancy_timeout_ = declare_parameter<double>("planner_occupancy_timeout", 2.0);
     occupancy_height_tolerance_ = declare_parameter<double>("planner_occupancy_height_tolerance", 0.5);
     require_planner_occupancy_ = declare_parameter<bool>("require_planner_occupancy", false);
     observation_only_ = declare_parameter<bool>("observation_only", true);
     bounds_relative_to_start_ = declare_parameter<bool>("bounds_relative_to_start", false);
     layer_resolution_ = declare_parameter<double>("layer_surface_resolution", 0.10);
     layer_body_height_ = declare_parameter<double>("layer_nominal_body_height", 0.32);
     layer_timeout_ = declare_parameter<double>("layer_surface_timeout", 2.5);
     layer_planning_timeout_ms_ = declare_parameter<double>(
         "layer_planning_timeout_ms", 2000.0);
     route_segment_length_ = declare_parameter<double>("route_segment_length", 3.0);
     const int recovery_failure_limit = declare_parameter<int>("recovery_failure_limit", 3);
     const double recovery_area_radius = declare_parameter<double>("recovery_area_radius", 2.0);
     recovery_ = le::RecoveryPlanner(recovery_failure_limit, recovery_area_radius);
     recovery_yaw_rate_ = declare_parameter<double>("recovery_yaw_rate", 0.35);
     recovery_turn_step_ = declare_parameter<double>("recovery_turn_step", 0.70);
     recovery_turn_timeout_ = declare_parameter<double>("recovery_turn_timeout", 8.0);
     recovery_goal_min_distance_ = declare_parameter<double>("recovery_goal_min_distance", 0.65);
     recovery_goal_max_distance_ = declare_parameter<double>("recovery_goal_max_distance", 2.0);
     recovery_reverse_distance_ = declare_parameter<double>("recovery_reverse_distance", 0.35);
     recovery_max_turn_steps_ = declare_parameter<int>("recovery_max_turn_steps", 4);
     recovery_retry_interval_ = declare_parameter<double>("recovery_retry_interval", 10.0);
     if (layer_resolution_ <= 0.0 || layer_body_height_ <= 0.0 ||
         layer_planning_timeout_ms_ <= 0.0 ||
         layer_timeout_ <= 0.0 || route_segment_length_ <= 0.0 ||
         recovery_failure_limit < 1 || recovery_area_radius <= 0.0 ||
         recovery_yaw_rate_ <= 0.0 || recovery_yaw_rate_ > 0.6 ||
         recovery_turn_step_ <= 0.0 || recovery_turn_timeout_ <= 0.0 ||
         recovery_goal_min_distance_ <= 0.0 ||
         recovery_goal_max_distance_ < recovery_goal_min_distance_ ||
         recovery_reverse_distance_ < 0.20 || recovery_reverse_distance_ > 0.60 ||
         recovery_max_turn_steps_ < 0 || recovery_retry_interval_ <= 0.0)
       throw std::invalid_argument("invalid layer or recovery parameters");

     // 参数合法性检查（分辨率、搜索半径、抽稀步长、点数、体素上限必须 > 0）
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (cfg_.resolution <= 0.0 || cfg_.local_search_radius <= 0.0 || point_stride_ < 1 ||
         max_points_ < 1 || cfg_.max_voxels < 1) {
       // 发现非法输入，抛出错误并停止当前流程。
       throw std::invalid_argument("resolution, search radius, stride, max points and max voxels must be positive");
     }
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (cfg_.global_search && (cfg_.exploration_min_x >= cfg_.exploration_max_x ||
         cfg_.exploration_min_y >= cfg_.exploration_max_y))
       // 发现非法输入，抛出错误并停止当前流程。
       throw std::invalid_argument("global exploration bounds must have positive area");

     // ---------- ROS 接口 ----------
     auto state_qos = rclcpp::QoS(1).reliable().transient_local(); // QoS 设置，确保状态可靠传递
     waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(waypoint_topic, 10); // 发布前沿探索目标（PoseStamped）
     // 建立 ROS 发布者，用来向指定话题发送消息。
     marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/exploration/frontiers", state_qos);
     // 建立 ROS 发布者，用来向指定话题发送消息。
     status_pub_ = create_publisher<std_msgs::msg::String>("/exploration/status", state_qos);
     goal_diagnostic_pub_ = create_publisher<std_msgs::msg::String>(
         "/exploration/diagnostics/goal_event", state_qos);
     diagnostic_reference_pub_ = create_publisher<nav_msgs::msg::Path>(
         "/exploration/diagnostics/last_reference_path", state_qos);
     diagnostic_actual_pub_ = create_publisher<nav_msgs::msg::Path>(
         "/exploration/diagnostics/last_actual_path", state_qos);
     diagnostic_bspline_pub_ = create_publisher<scan_planner::msg::Bspline>(
         "/exploration/diagnostics/last_scan_bspline", state_qos);
     planning_goal_pub_ = create_publisher<std_msgs::msg::String>(
         "/exploration/planning_goal", state_qos);
     planning_result_pub_ = create_publisher<std_msgs::msg::String>(
         "/exploration/planning_results", state_qos);
     execution_result_pub_ = create_publisher<std_msgs::msg::String>(
         "/exploration/execution_feedback", state_qos);
     coverage_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/exploration/coverage_grid", state_qos);
     // 建立 ROS 发布者，用来向指定话题发送消息。
     trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("/exploration/trajectory", state_qos);
     planned_path_pub_ = create_publisher<nav_msgs::msg::Path>(
         "/exploration/planned_path", state_qos);
     selected_occupancy_path_pub_ = create_publisher<nav_msgs::msg::Path>(
         "/exploration/selected_routes/occupancy", state_qos);
     selected_layer_path_pub_ = create_publisher<nav_msgs::msg::Path>(
         "/exploration/selected_routes/layer", state_qos);
     selected_recovery_path_pub_ = create_publisher<nav_msgs::msg::Path>(
         "/exploration/selected_routes/recovery", state_qos);
     occupancy_plan_pub_ = create_publisher<nav_msgs::msg::Path>(
         "/exploration/plans/occupancy", state_qos);
     layer_plan_pub_ = create_publisher<nav_msgs::msg::Path>(
         "/exploration/plans/layer", state_qos);
     recovery_plan_pub_ = create_publisher<nav_msgs::msg::Path>(
         "/exploration/plans/recovery", state_qos);
     recovery_turn_pub_ = create_publisher<geometry_msgs::msg::Twist>(
         "/exploration/bootstrap_turn_cmd", 10);
     recovery_reverse_pub_ = create_publisher<std_msgs::msg::String>(
         "/scan/recovery_reverse_request", 10);
     control_mode_pub_ = create_publisher<std_msgs::msg::Int32>(
         "/go2/control_mode", 10);
     trajectory_.header.frame_id = frame_id_;

     // 订阅点云、里程计、手动目标等主题
     // 订阅一个 ROS 话题，收到数据时调用对应回调。
     cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
         cloud_topic, rclcpp::SensorDataQoS(),
         [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_cloud(msg); });
     // 订阅一个 ROS 话题，收到数据时调用对应回调。
     sensor_sub_ = create_subscription<nav_msgs::msg::Odometry>(
         sensor_topic, rclcpp::SensorDataQoS(),
         [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_sensor(*msg); });
     // 订阅一个 ROS 话题，收到数据时调用对应回调。
     base_sub_ = create_subscription<nav_msgs::msg::Odometry>(
         base_topic, rclcpp::SensorDataQoS(),
         [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_base(*msg); });
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (!occupancy_topic.empty())
       // 订阅一个 ROS 话题，收到数据时调用对应回调。
       occupancy_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
           occupancy_topic, rclcpp::SensorDataQoS(),
           [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_occupancy(*msg); });
     if (!raw_occupancy_topic.empty())
       raw_occupancy_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
           raw_occupancy_topic, rclcpp::SensorDataQoS(),
           [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
             if (read_occupancy_points(*msg, raw_obstacles_))
               raw_occupancy_wall_ = wall_now();
           });
     scan_bspline_sub_ = create_subscription<scan_planner::msg::Bspline>(
         "/scan/planning/bspline", 10,
         [this](scan_planner::msg::Bspline::ConstSharedPtr msg) {
           if (active_ && !active_->reverse &&
               stamp_ns(msg->start_time) + 500000000LL >= active_->input_stamp)
             active_scan_bspline_ = *msg;
         });
     if (!shared_layer_) {
       layer_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
           "/exploration/layers/planning_surfaces", state_qos,
           [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_layers(*msg); });
       layer_occupied_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
           "/exploration/layers/planning_occupied", state_qos,
           [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
             on_layer_occupied(*msg);
           });
     }
     // 订阅一个 ROS 话题，收到数据时调用对应回调。
     ready_sub_ = create_subscription<std_msgs::msg::Bool>(
         ready_topic, state_qos,
         [this](std_msgs::msg::Bool::ConstSharedPtr msg) { ready_ = msg->data; });
     // 订阅一个 ROS 话题，收到数据时调用对应回调。
     input_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
         waypoint_topic, 10,
         [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { on_input(*msg); });
     // 订阅一个 ROS 话题，收到数据时调用对应回调。
     goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
         goal_topic, 10,
         [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { on_goal(*msg); });
     // 订阅一个 ROS 话题，收到数据时调用对应回调。
     ack_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
         "/scan/goal_stop_ack", 10,
         [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { on_ack(*msg); });
     // 订阅一个 ROS 话题，收到数据时调用对应回调。
     complete_sub_ = create_subscription<std_msgs::msg::Bool>(
         "/scan/mission_complete", state_qos,
         [this](std_msgs::msg::Bool::ConstSharedPtr msg) { on_complete(*msg); });
     plan_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
         "/scan/plan_ready", state_qos,
         [this](std_msgs::msg::Bool::ConstSharedPtr msg) { scan_plan_ready_ = msg->data; });
     scan_failure_sub_ = create_subscription<std_msgs::msg::String>(
         "/scan/goal_plan_failed", 10,
         [this](std_msgs::msg::String::ConstSharedPtr msg) { on_scan_failure(*msg); });
     recovery_reverse_result_sub_ = create_subscription<std_msgs::msg::String>(
         "/scan/recovery_reverse_result", 10,
         [this](std_msgs::msg::String::ConstSharedPtr msg) {
           on_recovery_reverse_result(*msg);
         });

     // 暂停服务 /exploration/set_paused
     // 建立 ROS 服务，让外部可以暂停或恢复探索。
     pause_service_ = create_service<std_srvs::srv::SetBool>(
         "/exploration/set_paused",
         [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
           paused_ = request->data;
           response->success = true;
           response->message = paused_ ? "exploration paused" : "exploration resumed";
         });

     // 每秒调用一次的主循环计时器
     timer_ = create_wall_timer(1s, [this]() { tick(); });
     motion_timer_ = create_wall_timer(
         50ms, [this]() { motion_tick(); });

     // 初始状态上报（观测模式或等待数据）
     report(observation_only_ ? "observation_only" : "waiting_for_data");
   }

 private:
   // ---------------------------------------------------------------
   // 结构体：记录当前活动的目标信息
   // ---------------------------------------------------------------
   // 定义保存相关数据或行为的 C++ 类型。
   struct Active {
     le::XY xy;                    // 目标 XY 坐标
     int64_t input_stamp = 0;     // 手动输入目标的时间戳（纳秒）
     std::optional<int64_t> stamp; // 外部 Goal 消息的时间戳（纳秒）
     bool ack = false;             // 是否收到前进确认（Ack）
     bool armed = false;           // 是否已收到前进前的 “armed” 标记（mission_complete false）
     bool complete = false;        // 是否已收到完成信号（mission_complete true）
     bool recovery = false;
     bool reverse = false;
     std::size_t observed_cells_at_send = 0;
     double initial_distance = 0.0; // SCAN 完成回执后核算脱困位移。
     le::Point pose_at_send{0.0, 0.0, 0.0};
     std::size_t trajectory_start_index = 0;
     std::size_t map_version_at_send = 0, layer_version_at_send = 0;
     std::uint64_t goal_id = 0, attempt_id = 0;
     std::size_t stage_index = 0;
     std::optional<double> raw_distance_at_send, inflated_distance_at_send;
     std::string source_at_send;
     std::vector<le::XY> reference_route;
   };
   struct Suppressed { le::XY xy; double expiry; }; // 被抑制的前沿（冷却）

   // ---------------------------------------------------------------
   // 参数读取与 Settings 初始化
   // ---------------------------------------------------------------
   le::Settings declare_settings() {
     le::Settings cfg;
     cfg.resolution = declare_parameter<double>("resolution", cfg.resolution);
     cfg.max_ray_range = declare_parameter<double>("max_ray_range", cfg.max_ray_range);
     int max_voxels = declare_parameter<int>("max_voxels", 900000);
     // 地图已经达到体素数量上限，不能继续插入。
     if (max_voxels < 1) throw std::invalid_argument("max_voxels must be positive");
     cfg.max_voxels = static_cast<std::size_t>(max_voxels);
     cfg.obstacle_min_relative_z = declare_parameter<double>("obstacle_min_relative_z", cfg.obstacle_min_relative_z);
     cfg.obstacle_max_relative_z = declare_parameter<double>("obstacle_max_relative_z", cfg.obstacle_max_relative_z);
     cfg.free_min_relative_z = declare_parameter<double>("free_min_relative_z", cfg.free_min_relative_z);
     cfg.robot_radius = declare_parameter<double>("robot_radius", cfg.robot_radius);
     cfg.planner_occupancy_radius = declare_parameter<double>(
         "planner_occupancy_radius", cfg.planner_occupancy_radius);
     cfg.planner_occupancy_resolution = declare_parameter<double>(
         "planner_occupancy_resolution", cfg.planner_occupancy_resolution);
     cfg.waypoint_safety_margin = declare_parameter<double>(
         "waypoint_safety_margin", cfg.waypoint_safety_margin);
     if (!std::isfinite(cfg.planner_occupancy_radius) || cfg.planner_occupancy_radius < 0.0)
       throw std::invalid_argument("planner_occupancy_radius must be finite and nonnegative");
     if (!std::isfinite(cfg.planner_occupancy_resolution) ||
         cfg.planner_occupancy_resolution <= 0.0 ||
         !std::isfinite(cfg.waypoint_safety_margin) || cfg.waypoint_safety_margin < 0.0)
       throw std::invalid_argument("waypoint safety parameters must be finite and valid");
     cfg.local_search_radius = declare_parameter<double>("local_search_radius", cfg.local_search_radius);
     cfg.global_search = declare_parameter<bool>("global_search", cfg.global_search);
     cfg.exploration_min_x = declare_parameter<double>("exploration_min_x", cfg.exploration_min_x);
     cfg.exploration_max_x = declare_parameter<double>("exploration_max_x", cfg.exploration_max_x);
     cfg.exploration_min_y = declare_parameter<double>("exploration_min_y", cfg.exploration_min_y);
     cfg.exploration_max_y = declare_parameter<double>("exploration_max_y", cfg.exploration_max_y);
     cfg.min_goal_distance = declare_parameter<double>("min_goal_distance", cfg.min_goal_distance);
     cfg.max_goal_distance = declare_parameter<double>("max_goal_distance", cfg.max_goal_distance);
     cfg.min_frontier_cells = declare_parameter<int>("min_frontier_cells", cfg.min_frontier_cells);
     cfg.frontier_gain_weight = declare_parameter<double>("frontier_gain_weight", cfg.frontier_gain_weight);
     cfg.path_cost_weight = declare_parameter<double>("path_cost_weight", cfg.path_cost_weight);
     cfg.unknown_path_multiplier = declare_parameter<double>("unknown_path_multiplier", cfg.unknown_path_multiplier);
     cfg.free_observation_saturation = declare_parameter<int>(
         "free_observation_saturation", cfg.free_observation_saturation);
     cfg.low_evidence_cost_weight = declare_parameter<double>(
         "low_evidence_cost_weight", cfg.low_evidence_cost_weight);
     cfg.shortcut_cost_tolerance = declare_parameter<double>(
         "shortcut_cost_tolerance", cfg.shortcut_cost_tolerance);
     if (cfg.free_observation_saturation < 1 ||
         cfg.free_observation_saturation > 255 ||
         !std::isfinite(cfg.low_evidence_cost_weight) ||
         cfg.low_evidence_cost_weight < 0.0 ||
         !std::isfinite(cfg.shortcut_cost_tolerance) ||
         cfg.shortcut_cost_tolerance < 0.0)
       throw std::invalid_argument("invalid occupancy evidence parameters");
     cfg_ = cfg;
     // 把此函数算出的值交还给调用者。
     return cfg;
   }

   // ---------------------------------------------------------------
   // 状态上报（向 /exploration/status 主题发布 JSON）
   // ---------------------------------------------------------------
   // 把当前探索状态发布为 JSON 文本。
   void report(const std::string &state, const std::string &detail = "") {
     std::ostringstream stream;
     stream << "{\"state\":\"" << state << "\",\"implementation\":\"cpp\",";
     stream << "\"map_version\":" << map_.version() << ",\"voxel_count\":" << map_.size();
     stream << ",\"paused\":" << (paused_ ? "true" : "false");
     stream << ",\"observation_only\":" << (observation_only_ ? "true" : "false");
     stream << ",\"global_search\":" << (cfg_.global_search ? "true" : "false");
     stream << ",\"global_frontier_valid\":" << (global_frontiers_.valid ? "true" : "false");
     stream << ",\"global_frontier_cells\":" << global_frontiers_.frontier_cells;
     stream << ",\"indexed_frontier_cells\":" << frontier_index_.cells().size();
     stream << ",\"frontier_index_repairs\":" << frontier_index_repairs_;
     stream << ",\"global_reachable_free_cells\":" << global_frontiers_.reachable_free_cells;
     stream << ",\"frontier_cells\":" << last_frontier_cells_;
     stream << ",\"frontier_clusters\":" << last_frontier_clusters_;
     stream << ",\"reachable_frontier_cells\":" << last_reachable_frontier_cells_;
     stream << ",\"candidate_goals\":" << last_candidate_goals_;
     stream << ",\"suppressed_goals\":" << last_suppressed_goals_;
     stream << ",\"open_start_neighbors\":" << last_open_start_neighbors_;
     stream << ",\"total_cells\":" << coverage_.total_cells;
     stream << ",\"known_free_cells\":" << coverage_.known_free_cells;
     stream << ",\"known_occupied_cells\":" << coverage_.known_occupied_cells;
     stream << ",\"unknown_cells\":" << coverage_.unknown_cells;
     stream << ",\"coverage_ratio\":" << coverage_.ratio();
     stream << ",\"coverage_valid\":" << (coverage_.valid ? "true" : "false");
     stream << ",\"reachability_valid\":" << (coverage_.reachability_valid ? "true" : "false");
     stream << ",\"reachable_cells\":" << coverage_.reachable_cells;
     stream << ",\"reachable_unknown_cells\":" << coverage_.reachable_unknown_cells;
     stream << ",\"excluded_unknown_cells\":" << coverage_.excluded_unknown_cells;
     stream << ",\"reachable_coverage_ratio\":" << coverage_.reachable_ratio();
     stream << ",\"unresolved_unknown_cells\":" << unresolved_unknown_cells_;
     stream << ",\"route_source\":\"" << path_processor_.source() << "\"";
     stream << ",\"control_mode\":\"" << le::mode_name(control_mode_) << "\"";
     stream << ",\"goal_id\":" << goal_id_;
     stream << ",\"planning_attempt_id\":" << planning_attempt_id_;
     stream << ",\"layer_map_version\":" << layer_version_;
     stream << ",\"route_stages\":" << path_processor_.waypoints().size();
     stream << ",\"route_stage_index\":" << path_processor_.index();
     stream << ",\"route_skipped_nearby\":" << route_skipped_nearby_;
     stream << ",\"scan_failure_count\":" << scan_failure_count_;
     if (!scan_last_failure_.empty())
       stream << ",\"scan_last_failure\":\"" << scan_last_failure_ << "\"";
     if (route_selection_target_)
       stream << ",\"selected_goal_x\":" << (*route_selection_target_)[0]
              << ",\"selected_goal_y\":" << (*route_selection_target_)[1];
     if (active_ && base_)
       stream << ",\"active_waypoint_x\":" << active_->xy[0]
              << ",\"active_waypoint_y\":" << active_->xy[1]
              << ",\"active_waypoint_distance\":"
              << xy_distance(active_->xy, {(*base_)[0], (*base_)[1]});
     stream << ",\"recovery_phase\":\"" << le::recovery_phase_name(recovery_.phase()) << "\"";
     stream << ",\"recovery_failures\":" << recovery_.failures();
     stream << ",\"recovery_turn_steps\":" << recovery_turn_steps_;
     stream << ",\"recovery_nearby_attempts\":" << recovery_nearby_attempts_;
     stream << ",\"recovery_history_attempts\":" << recovery_history_attempts_;
     stream << ",\"recovery_reverse_attempted\":"
            << (recovery_reverse_attempted_ ? "true" : "false");
     stream << ",\"recovery_reverse_sent\":"
            << (recovery_reverse_sent_ ? "true" : "false");
     stream << ",\"recovery_reverse_last_distance\":"
            << recovery_reverse_last_distance_;
     stream << ",\"recovery_reverse_next_index\":"
            << recovery_reverse_next_index_;
     stream << ",\"recovery_last_target_distance\":" << recovery_last_target_distance_;
     stream << ",\"recovery_direct_limit\":" << route_segment_length_;
     if (recovery_.active() && !recovery_turn_block_reason_.empty())
       stream << ",\"recovery_turn_block_reason\":\"" << recovery_turn_block_reason_ << "\"";
     const auto append_rejections = [&](const char *name,
         const std::map<std::string, int> &rejections) {
       stream << ",\"" << name << "\":{";
       bool first = true;
       for (const auto &[reason, count] : rejections) {
         if (!first) stream << ',';
         stream << '\"' << reason << "\":" << count;
         first = false;
       }
       stream << '}';
     };
     if (recovery_.active()) {
       append_rejections("recovery_strict_rejections", recovery_strict_rejections_);
       append_rejections("recovery_direct_rejections", recovery_direct_rejections_);
       append_rejections("recovery_reverse_rejections", recovery_reverse_rejections_);
     }
     if (recovery_.active() && !recovery_reverse_last_failure_.empty())
       stream << ",\"recovery_reverse_last_failure\":\""
              << recovery_reverse_last_failure_ << "\"";
     if (recovery_.active() && !recovery_last_failure_.empty())
       stream << ",\"recovery_last_failure\":\"" << recovery_last_failure_ << "\"";
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (!detail.empty()) stream << ",\"detail\":\"" << detail << "\"";
     stream << '}';
     if (stream.str() == last_status_) return; // 去重：若未变化则不发送
     std_msgs::msg::String msg;
     msg.data = stream.str();
     status_pub_->publish(msg);
     last_status_ = msg.data;
   }

   void publish_coverage_grid(const std::unordered_map<le::Cell, int, le::CellHash> &grid) {
     if (!coverage_.valid) return;
     const double r = cfg_.resolution;
     const int x0 = static_cast<int>(std::ceil(cfg_.exploration_min_x / r - 0.5));
     const int x1 = static_cast<int>(std::floor(cfg_.exploration_max_x / r - 0.5));
     const int y0 = static_cast<int>(std::ceil(cfg_.exploration_min_y / r - 0.5));
     const int y1 = static_cast<int>(std::floor(cfg_.exploration_max_y / r - 0.5));
     nav_msgs::msg::OccupancyGrid msg;
     msg.header.frame_id = frame_id_;
     msg.header.stamp = now();
     msg.info.map_load_time = msg.header.stamp;
     msg.info.resolution = r;
     msg.info.width = static_cast<uint32_t>(x1 - x0 + 1);
     msg.info.height = static_cast<uint32_t>(y1 - y0 + 1);
     msg.info.origin.position.x = x0 * r;
     msg.info.origin.position.y = y0 * r;
     msg.info.origin.orientation.w = 1.0;
     msg.data.reserve(coverage_.total_cells);
     for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) {
       auto found = grid.find({x, y});
       msg.data.push_back(found == grid.end() ? -1 : (found->second == 0 ? 0 : 100));
     }
     coverage_pub_->publish(msg);
   }

   // ---------------------------------------------------------------
   // 里程计（LiDAR）回调
   // ---------------------------------------------------------------
   // 接收雷达位姿并尝试处理排队的点云。
   void on_sensor(const nav_msgs::msg::Odometry &msg) {
     if (msg.header.frame_id != frame_id_) return; // 检查坐标系
     const auto &p = msg.pose.pose.position;
     if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return; // 数据有效性检查
     // 把本次结果放入容器，供下一轮处理。
     sensor_poses_.push_back({stamp_ns(msg.header.stamp), {p.x, p.y, p.z}});
     if (sensor_poses_.size() > 100) sensor_poses_.pop_front(); // 只保留最近 100 条
     process_clouds();
   }

   // ---------------------------------------------------------------
   // 基座里程计回调（机器人底盘位置）
   // ---------------------------------------------------------------
   // 接收机体位姿、更新轨迹并尝试处理点云。
   void on_base(const nav_msgs::msg::Odometry &msg) {
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (msg.header.frame_id != frame_id_) return;
     const auto &p = msg.pose.pose.position;
     // 过滤包含无穷大或 NaN 的无效坐标。
     if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
     const auto &q = msg.pose.pose.orientation;
     const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
     if (std::isfinite(q_norm) && q_norm >= 0.5 && q_norm <= 1.5) {
       const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
       if (recovery_.phase() == le::RecoveryPhase::turning && base_yaw_) {
         const double delta = std::remainder(yaw - *base_yaw_, 2.0 * M_PI);
         if (std::abs(delta) < 0.5) recovery_rotation_ += delta;
       }
       base_yaw_ = yaw;
       yaw_wall_ = wall_now();
     }
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (cfg_.global_search && bounds_relative_to_start_ && !bounds_initialized_) {
       cfg_.exploration_min_x += p.x;
       cfg_.exploration_max_x += p.x;
       cfg_.exploration_min_y += p.y;
       cfg_.exploration_max_y += p.y;
       map_.set_exploration_bounds(cfg_.exploration_min_x, cfg_.exploration_max_x,
                                   cfg_.exploration_min_y, cfg_.exploration_max_y);
       bounds_initialized_ = true;
     }
     base_ = le::Point{p.x, p.y, p.z};
     // 把本次结果放入容器，供下一轮处理。
     base_poses_.push_back({stamp_ns(msg.header.stamp), *base_});
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (base_poses_.size() > 100) base_poses_.pop_front();
     base_wall_ = wall_now();
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (trajectory_.poses.empty() ||
         xy_distance({p.x, p.y}, {trajectory_.poses.back().pose.position.x,
                                  trajectory_.poses.back().pose.position.y}) >= 0.1) {
       geometry_msgs::msg::PoseStamped pose;
       pose.header = msg.header;
       pose.pose = msg.pose.pose;
       // 把本次结果放入容器，供下一轮处理。
       trajectory_.poses.push_back(pose);
       // 只有表达式为真，才执行下面缩进或花括号内的代码。
       if (trajectory_.poses.size() > 20000) trajectory_.poses.erase(trajectory_.poses.begin());
     }
     process_clouds();
   }

   bool read_occupancy_points(const sensor_msgs::msg::PointCloud2 &msg,
                              std::vector<le::XY> &output) {
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (msg.header.frame_id != frame_id_ || !base_) return false;
     bool x_found = false, y_found = false, z_found = false;
     // 依次处理收到或待发送的 ROS 消息。
     for (const auto &field : msg.fields) {
       // 只有表达式为真，才执行下面缩进或花括号内的代码。
       if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) x_found = true;
       // 只有表达式为真，才执行下面缩进或花括号内的代码。
       if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) y_found = true;
       // 只有表达式为真，才执行下面缩进或花括号内的代码。
       if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) z_found = true;
     }
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (!x_found || !y_found || !z_found) return false;
     // 清空临时状态，准备下一轮处理。
     output.clear();
     sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
     const std::size_t total = static_cast<std::size_t>(msg.width) * msg.height;
     // 依次处理集合中的每一项。
     for (std::size_t i = 0; i < total; ++i, ++x, ++y, ++z) {
       // 过滤包含无穷大或 NaN 的无效坐标。
       if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z) &&
           std::abs(*z - (*base_)[2]) <= occupancy_height_tolerance_)
         // 把本次结果放入容器，供下一轮处理。
         output.push_back({*x, *y});
     }
     return true;
   }

   // 接收 SCAN 规划器提供的膨胀障碍点。
   void on_occupancy(const sensor_msgs::msg::PointCloud2 &msg) {
     if (read_occupancy_points(msg, external_obstacles_))
       occupancy_wall_ = wall_now();
   }

   // 接收未抽稀的层析表面快照；层析路线的安全检查只使用层析证据。
   void on_layers(const sensor_msgs::msg::PointCloud2 &msg) {
     if (msg.header.frame_id != frame_id_ || msg.is_bigendian || msg.point_step < 29)
       return;
     const auto offset = [&](const char *name, uint8_t type) -> std::optional<uint32_t> {
       for (const auto &field : msg.fields)
         if (field.name == name && field.datatype == type &&
             field.offset < msg.point_step) return field.offset;
       return std::nullopt;
     };
     using Field = sensor_msgs::msg::PointField;
     const auto ox = offset("x", Field::FLOAT32), oy = offset("y", Field::FLOAT32);
     const auto oz = offset("z", Field::FLOAT32);
     const auto ok = offset("surface_kind", Field::UINT8);
     const auto oc = offset("clearance_kind", Field::UINT8);
     const auto os = offset("support_columns", Field::UINT16);
     const auto oh = offset("hit_observations", Field::UINT32);
     if (!ox || !oy || !oz || !ok || !oc || !os || !oh) return;
     if (*ox + 4 > msg.point_step || *oy + 4 > msg.point_step ||
         *oz + 4 > msg.point_step || *os + 2 > msg.point_step ||
         *oh + 4 > msg.point_step) return;
     const auto total = static_cast<std::size_t>(msg.width) * msg.height;
     if (msg.data.size() < total * msg.point_step) return;
     std::vector<le::LayerSurface> surfaces;
     surfaces.reserve(total);
     for (std::size_t i = 0; i < total; ++i) {
       const auto *p = msg.data.data() + i * msg.point_step;
       float x, y, z;
       uint16_t support;
       uint32_t hits;
       std::memcpy(&x, p + *ox, 4); std::memcpy(&y, p + *oy, 4);
       std::memcpy(&z, p + *oz, 4); std::memcpy(&support, p + *os, 2);
       std::memcpy(&hits, p + *oh, 4);
       if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
           p[*ok] > static_cast<uint8_t>(le::SurfaceKind::steep_or_disconnected) ||
           p[*oc] > static_cast<uint8_t>(le::ClearanceKind::unobserved)) continue;
       le::LayerSurface surface;
       surface.x_cell = static_cast<int>(std::floor(x / layer_resolution_));
       surface.y_cell = static_cast<int>(std::floor(y / layer_resolution_));
       surface.height = z;
       surface.kind = static_cast<le::SurfaceKind>(p[*ok]);
       surface.clearance = static_cast<le::ClearanceKind>(p[*oc]);
       surface.support_columns = support;
       surface.hit_observations = static_cast<int>(std::min<uint32_t>(hits, 1000000));
       surfaces.push_back(surface);
     }
     layer_surfaces_ = std::move(surfaces);
     layer_wall_ = wall_now();
     layer_stamp_ = stamp_ns(msg.header.stamp);
     ++layer_version_;
   }

   void on_layer_occupied(const sensor_msgs::msg::PointCloud2 &msg) {
     if (msg.header.frame_id != frame_id_) return;
     std::vector<le::Point> occupied;
     occupied.reserve(static_cast<std::size_t>(msg.width) * msg.height);
     try {
       sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
       for (; x != x.end(); ++x, ++y, ++z)
         if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z))
           occupied.push_back({*x, *y, *z});
     } catch (const std::runtime_error &) {
       return;
     }
     layer_occupied_ = std::move(occupied);
     layer_occupied_wall_ = wall_now();
     layer_occupied_stamp_ = stamp_ns(msg.header.stamp);
   }

   // 判断 SCAN 障碍点云是否仍在有效时限内。
   bool planner_occupancy_fresh(double current) const {
     // 把此函数算出的值交还给调用者。
     return occupancy_wall_ > 0.0 && current - occupancy_wall_ <= occupancy_timeout_;
   }

   // 判断给定目标附近是否存在规划器障碍。
   bool planner_blocks(const le::XY &xy) const {
     // 把此函数算出的值交还给调用者。
     return std::any_of(external_obstacles_.begin(), external_obstacles_.end(),
         [this, &xy](const le::XY &obstacle) {
           // 把此函数算出的值交还给调用者。
           return xy_distance(xy, obstacle) <=
               std::max(0.0, cfg_.robot_radius - cfg_.planner_occupancy_radius);
         });
   }

   // ---------------------------------------------------------------
   // 点云回调 将当前帧雷达点云缓存起来，等待与最近的里程计匹配后再融合到体素地图
   // ---------------------------------------------------------------
   // 接收一帧已配准点云并排队处理。
   void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (msg->header.frame_id != frame_id_) return;
     // 把本次结果放入容器，供下一轮处理。
     pending_clouds_.push_back(msg);
     if (pending_clouds_.size() > 5) pending_clouds_.pop_front(); // 最多缓存 5 帧点云
     process_clouds();
   }

   // ---------------------------------------------------------------
   // 处理缓存的点云：与最近的里程计匹配并融合到体素地图
   // ---------------------------------------------------------------
   // 用时间对齐的里程计处理排队的点云。
   void process_clouds() {
     // 必须同时拥有基座、里程计且基座未超时才能继续
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (!base_ || sensor_poses_.empty() || base_poses_.empty() ||
         wall_now() - base_wall_ > odom_timeout_) return;
     // 依次处理集合中的每一项。
     for (auto it = pending_clouds_.begin(); it != pending_clouds_.end();) {
       int64_t stamp = stamp_ns((*it)->header.stamp);
       // 只有表达式为真，才执行下面缩进或花括号内的代码。
       if (integrate_cloud(**it))
         // 从容器中取走已处理或失效的数据。
         it = pending_clouds_.erase(it);
       // 上一条件不满足时，再检查这个分支。
       else if (sensor_poses_.back().first - stamp > sync_tolerance_ * 1e9 &&
                base_poses_.back().first - stamp > sync_tolerance_ * 1e9)
         it = pending_clouds_.erase(it); // 超出同步容差，直接丢弃
       // 前面的条件不成立时，执行这个备选分支。
       else
         ++it; // 仍等待更合适的里程计匹配
     }
   }

   // ---------------------------------------------------------------
   // 将单帧点云投影到体素地图中
   // ---------------------------------------------------------------
   // 将一帧点云融合到稀疏体素地图。
   bool integrate_cloud(const sensor_msgs::msg::PointCloud2 &msg) {
     int64_t stamp = stamp_ns(msg.header.stamp);
     // 找到时间戳最接近的里程计姿态
     auto nearest = std::min_element(sensor_poses_.begin(), sensor_poses_.end(),
                                 [&stamp](const auto &a, const auto &b) {
                                   return std::llabs(a.first - stamp) < std::llabs(b.first - stamp); // 绝对值比较，找到最接近的时间戳
                                 });
     if (std::llabs(nearest->first - stamp) > sync_tolerance_ * 1e9) return false; // 同步容差外直接返回 false
     auto nearest_base = std::min_element(base_poses_.begin(), base_poses_.end(),
         [&stamp](const auto &a, const auto &b) {
           // 把此函数算出的值交还给调用者。
           return std::llabs(a.first - stamp) < std::llabs(b.first - stamp);
         });
     // 只有时间戳与本轮消息匹配时才采用这份数据。
     if (std::llabs(nearest_base->first - stamp) > sync_tolerance_ * 1e9) return false;

     // 检查点云是否包含 x/y/z 三个字段（必须为 FLOAT32）
     bool x_found = false, y_found = false, z_found = false;
     // 依次处理收到或待发送的 ROS 消息。
     for (const auto &field : msg.fields) {
       // 只有表达式为真，才执行下面缩进或花括号内的代码。
       if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) x_found = true;
       // 只有表达式为真，才执行下面缩进或花括号内的代码。
       if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) y_found = true;
       // 只有表达式为真，才执行下面缩进或花括号内的代码。
       if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) z_found = true;
     }
     if (!x_found || !y_found || !z_found) return true; // 若缺失字段直接跳过（算作成功）

     // 抽稀采样点云，最多 max_points_ 个点
     std::vector<le::Point> points;
     points.reserve(max_points_);
     sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
     const std::size_t total = static_cast<std::size_t>(msg.width) * msg.height;
     // 依次处理一帧点云点集。
     for (std::size_t i = 0; i < total && static_cast<int>(points.size()) < max_points_; ++i, ++x, ++y, ++z) {
       // 只有表达式为真，才执行下面缩进或花括号内的代码。
       if (i % static_cast<std::size_t>(point_stride_) == 0)
         // 把本次结果放入容器，供下一轮处理。
         points.push_back({*x, *y, *z});
     }
     // 将抽稀后的点云融合进体素地图（使用最近的里程计姿态）
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (map_.integrate(nearest->second, points, nearest_base->second[2]))
       cloud_wall_ = wall_now();
     // 把此函数算出的值交还给调用者。
     return true;
   }

   // ---------------------------------------------------------------
   // 手动输入（Waypoint）回调：用于调试或手动指定目标点
   // ---------------------------------------------------------------
   // 识别手动输入目标，必要时暂停自动探索。
   void on_input(const geometry_msgs::msg::PoseStamped &msg) {
     le::XY xy{msg.pose.position.x, msg.pose.position.y};
     // 若已有活动目标且与本次输入距离 < 0.05 m 且时间戳相同，则认为是重复输入，直接忽略
     // 只有时间戳与本轮消息匹配时才采用这份数据。
     if (active_ && xy_distance(xy, active_->xy) < 0.05 && stamp_ns(msg.header.stamp) == active_->input_stamp) return;
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (recovery_.phase() == le::RecoveryPhase::turning) stop_recovery_turn();
     recovery_.finish();
     control_mode_ = le::ExplorationMode::flat;
     // An external manual goal supersedes the exploration route.
     active_.reset();
     reset_planned_route();
     paused_ = true;
     report("paused_for_manual_goal");
   }

   // ---------------------------------------------------------------
   // 外部规划目标（Goal）回调：来自上层规划模块的 XY 目标点
   // ---------------------------------------------------------------
   // 确认 SCAN 已接收或转发当前目标。
   void on_goal(const geometry_msgs::msg::PoseStamped &msg) {
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (msg.header.frame_id != frame_id_) return;
     le::XY xy{msg.pose.position.x, msg.pose.position.y};
     // 尚无对应数据或目标时，进入此分支。
     if (!active_) {
       paused_ = true;
       report("paused_for_manual_goal");
       // 结束当前函数，不再执行后面的语句。
       return;
     }
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (xy_distance(xy, active_->xy) < 0.05) {
       // 与当前活动目标几乎相同，只更新时间戳（保证是最新的 Goal）
       if (stamp_ns(msg.header.stamp) < active_->input_stamp) return; // 老旧消息直接忽略
       active_->stamp = stamp_ns(msg.header.stamp);
       path_processor_.bind_scan_goal(*active_->stamp);
       if (pending_scan_failure_ && pending_scan_failure_->first == *active_->stamp) {
         const auto pending = *pending_scan_failure_;
         pending_scan_failure_.reset();
         handle_scan_failure(pending.first, pending.second);
       }
     } else {
       // 完全不同的目标：先清除当前目标并暂停
       // 清空临时状态，准备下一轮处理。
       active_.reset();
       reset_planned_route();
       paused_ = true;
       report("paused_for_manual_goal");
     }
   }

   // ---------------------------------------------------------------
   // 前往确认回执（Ack）回调：确认机器人已到达目标点
   // ---------------------------------------------------------------
   // 确认 SCAN 已停车并回执目标。
   void on_ack(const geometry_msgs::msg::PoseStamped &msg) {
     // 尚无对应数据或目标时，进入此分支。
     if (!active_ || !active_->stamp || stamp_ns(msg.header.stamp) != *active_->stamp) return;
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (msg.header.frame_id != frame_id_) return;
     // 根据当前目标是否存在或是否完成来决定下一步。
     if (xy_distance({msg.pose.position.x, msg.pose.position.y}, active_->xy) > goal_reach_xy_) return;
     active_->ack = true;
     finish_if_complete();
   }

   // ---------------------------------------------------------------
   // 任务完成信号（Mission Complete）回调
   // ---------------------------------------------------------------
   // 接收 SCAN 的任务完成信号。
   void on_complete(const std_msgs::msg::Bool &msg) {
     // 尚无对应数据或目标时，进入此分支。
     if (!active_) return;
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (!msg.data)
       active_->armed = true; // 接收到 false，先进入 armed 状态，等待下一个 true 才算真正完成
     // 上一条件不满足时，再检查这个分支。
     else if (active_->armed) {
       active_->complete = true;
       finish_if_complete();
     }
   }

   // ---------------------------------------------------------------
   // 检查是否同时满足 Ack 与 Complete，若满足则报告目标已达成
   // ---------------------------------------------------------------
   // 在回执和完成信号齐备时结算目标。
   void finish_if_complete() {
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (active_ && active_->ack && active_->complete && active_->stamp) {
       auto result = path_processor_.resolve_execution(*active_->stamp, true, "goal_reached");
       if (!result) return;
       publish_execution_result(*result);
       const bool recovered = active_->recovery && base_ &&
           active_->initial_distance -
               xy_distance(active_->xy, {(*base_)[0], (*base_)[1]}) >= 0.30;
       RCLCPP_INFO(get_logger(), "Frontier goal reached: %.2f, %.2f", active_->xy[0], active_->xy[1]);
       finish_active_goal();
       // 清空临时状态，准备下一轮处理。
       active_.reset();
       advance_route();
       if (recovered && !path_processor_.active()) {
         recovery_.finish();
         recovery_turn_steps_ = 0;
         control_mode_ = le::ExplorationMode::flat;
       }
       // 清空临时状态，准备下一轮处理。
       empty_since_.reset();
       report("goal_reached");
     }
   }

   void publish_execution_result(const le::ExecutionResult &result) {
     std_msgs::msg::String message;
     std::ostringstream out;
     out << "{\"goal_id\":" << result.goal_id
         << ",\"attempt_id\":" << result.attempt_id
         << ",\"stage_index\":" << result.stage_index
         << ",\"scan_goal_stamp\":" << result.scan_goal_stamp
         << ",\"source\":\"" << result.source
         << "\",\"status\":\"" << (result.success ? "succeeded" : "failed")
         << "\",\"reason\":\"" << result.reason << "\"}";
     message.data = out.str();
     execution_result_pub_->publish(message);
   }

   std::optional<double> nearest_obstacle_distance(
       const le::XY &point, const std::vector<le::XY> &obstacles,
       double last_update) const {
     if (last_update <= 0.0 ||
         wall_now() - last_update > occupancy_timeout_ || obstacles.empty())
       return std::nullopt;
     double nearest = std::numeric_limits<double>::infinity();
     for (const auto &obstacle : obstacles)
       nearest = std::min(nearest, xy_distance(point, obstacle));
     return nearest;
   }

   void publish_goal_diagnostic(const Active &goal, const char *event,
                                const std::string &reason) {
     const auto append_distance = [](std::ostringstream &out, const char *key,
                                     std::optional<double> value) {
       out << ",\"" << key << "\":";
       if (value && std::isfinite(*value)) out << *value;
       else out << "null";
     };
     const le::XY current = base_ ?
         le::XY{(*base_)[0], (*base_)[1]} :
         le::XY{goal.pose_at_send[0], goal.pose_at_send[1]};
     std::ostringstream out;
     out << std::setprecision(9) << "{\"event\":\"" << event
         << "\",\"reason\":\"" << reason
         << "\",\"goal_id\":" << goal.goal_id
         << ",\"attempt_id\":" << goal.attempt_id
         << ",\"stage_index\":" << goal.stage_index
         << "\",\"input_stamp\":" << goal.input_stamp
         << ",\"scan_goal_stamp\":" << goal.stamp.value_or(0)
         << ",\"source\":\"" << goal.source_at_send
         << "\",\"map_version_at_send\":" << goal.map_version_at_send
         << ",\"map_version_now\":" << map_.version()
         << ",\"layer_version_at_send\":" << goal.layer_version_at_send
         << ",\"layer_version_now\":" << layer_version_
         << ",\"start_x\":" << goal.pose_at_send[0]
         << ",\"start_y\":" << goal.pose_at_send[1]
         << ",\"start_z\":" << goal.pose_at_send[2]
         << ",\"current_x\":" << current[0]
         << ",\"current_y\":" << current[1]
         << ",\"target_x\":" << goal.xy[0]
         << ",\"target_y\":" << goal.xy[1]
         << ",\"target_distance_now\":" << xy_distance(current, goal.xy)
         << ",\"reference_points\":" << goal.reference_route.size()
         << ",\"scan_bspline_traj_id\":"
         << (active_scan_bspline_ ? active_scan_bspline_->traj_id : -1);
     append_distance(out, "raw_distance_at_send", goal.raw_distance_at_send);
     append_distance(out, "inflated_distance_at_send", goal.inflated_distance_at_send);
     append_distance(out, "raw_distance_now",
                     nearest_obstacle_distance(current, raw_obstacles_, raw_occupancy_wall_));
     append_distance(out, "inflated_distance_now",
                     nearest_obstacle_distance(current, external_obstacles_, occupancy_wall_));
     out << '}';
     std_msgs::msg::String message;
     message.data = out.str();
     goal_diagnostic_pub_->publish(message);
     RCLCPP_INFO(get_logger(), "Goal diagnostic: %s", message.data.c_str());
     if (std::string(event) != "failed") return;
     nav_msgs::msg::Path reference, actual;
     reference.header.frame_id = actual.header.frame_id = frame_id_;
     reference.header.stamp = actual.header.stamp = now();
     const auto add_reference = [&](const le::XY &p) {
       geometry_msgs::msg::PoseStamped pose;
       pose.header = reference.header;
       pose.pose.position.x = p[0];
       pose.pose.position.y = p[1];
       pose.pose.position.z = goal.pose_at_send[2];
       pose.pose.orientation.w = 1.0;
       reference.poses.push_back(pose);
     };
     add_reference({goal.pose_at_send[0], goal.pose_at_send[1]});
     for (const auto &point : goal.reference_route) add_reference(point);
     diagnostic_reference_pub_->publish(reference);
     geometry_msgs::msg::PoseStamped start_pose;
     start_pose.header = actual.header;
     start_pose.pose.position.x = goal.pose_at_send[0];
     start_pose.pose.position.y = goal.pose_at_send[1];
     start_pose.pose.position.z = goal.pose_at_send[2];
     start_pose.pose.orientation.w = 1.0;
     actual.poses.push_back(start_pose);
     for (std::size_t i = goal.trajectory_start_index;
          i < trajectory_.poses.size(); ++i)
       actual.poses.push_back(trajectory_.poses[i]);
     diagnostic_actual_pub_->publish(actual);
     if (active_scan_bspline_)
       diagnostic_bspline_pub_->publish(*active_scan_bspline_);
   }

   void handle_scan_failure(std::int64_t stamp, const std::string &reason) {
     if (!active_ || !active_->stamp || *active_->stamp != stamp) return;
     auto result = path_processor_.resolve_execution(stamp, false, reason);
     if (!result) return;
     publish_goal_diagnostic(*active_, "failed", reason);
     publish_execution_result(*result);
     ++scan_failure_count_;
     scan_last_failure_ = reason;
     const bool recovering = active_->recovery;
     const bool reversing = active_->reverse;
     const le::XY failed_target = route_selection_target_.value_or(active_->xy);
     active_.reset();
     reset_planned_route();
     scan_plan_ready_ = false;
     if (recovering) {
       recovery_last_failure_ = reason;
       if (reversing) {
         recovery_reverse_last_failure_ = reason;
         ++recovery_reverse_rejections_[reason];
         recovery_reverse_attempted_ = false; // try the next short distance
       }
       report(reversing ? "recovery_reverse_failed" : "recovery_scan_failed", reason);
       return;
     }
     suppressed_.push_back({failed_target, wall_now() + cooldown_});
     if (base_) {
       recovery_.record_failure({(*base_)[0], (*base_)[1]}, reason);
       if (recovery_.limit_reached()) start_recovery(reason);
     }
     report("scan_plan_failed", reason);
   }

   void on_scan_failure(const std_msgs::msg::String &msg) {
     const auto separator = msg.data.find('|');
     if (separator == std::string::npos || separator == 0 || !active_) return;
     std::int64_t stamp = 0;
     const auto *begin = msg.data.data();
     const auto *end = begin + separator;
     const auto parsed = std::from_chars(begin, end, stamp);
     if (parsed.ec != std::errc{} || parsed.ptr != end || stamp <= 0) return;
     const auto reason = msg.data.substr(separator + 1);
     if (reason.empty() || reason.size() > 80 ||
         !std::all_of(reason.begin(), reason.end(), [](unsigned char ch) {
           return (ch >= 'a' && ch <= 'z') || ch == '_';
         })) return;
     if (!active_->stamp) {
       if (stamp >= active_->input_stamp)
         pending_scan_failure_ = std::make_pair(stamp, reason);
       return;
     }
     handle_scan_failure(stamp, reason);
   }

   void on_recovery_reverse_result(const std_msgs::msg::String &msg) {
     if (!active_ || !active_->reverse || !active_->stamp) return;
     const auto separator = msg.data.find('|');
     if (separator == std::string::npos ||
         msg.data.substr(separator + 1) != "succeeded") return;
     std::int64_t stamp = 0;
     const auto *begin = msg.data.data();
     const auto parsed = std::from_chars(begin, begin + separator, stamp);
     if (parsed.ec != std::errc{} || parsed.ptr != begin + separator ||
         stamp != *active_->stamp) return;
     auto result = path_processor_.resolve_execution(
         stamp, true, "reverse_completed");
     if (!result) return;
     publish_execution_result(*result);
     active_.reset();
     reset_planned_route();
     recovery_reverse_next_index_ = 0;
     recovery_reverse_attempted_ = false;
     recovery_nearby_attempts_ = 0;
     recovery_history_attempts_ = 0;
     recovery_.reset_attempts();
     recovery_reverse_map_version_ = map_.version();
     recovery_reverse_refresh_pending_ = true;
     recovery_.set_phase(le::RecoveryPhase::settling);
     report("recovery_reverse_waiting_for_map");
   }

   // ---------------------------------------------------------------
   // 将选中的前沿目标发布为 Waypoint（PoseStamped）
   // ---------------------------------------------------------------
   // 把选中的探索目标发送给 SCAN 航点协调器。
   void publish_target(const le::XY &xy, const std::string &detail = "") {
     // A latched plan_ready from the preceding waypoint must not arm this one.
     scan_plan_ready_ = false;
     pending_scan_failure_.reset();
     path_processor_.begin_execution();
     geometry_msgs::msg::PoseStamped msg;
     msg.header.stamp = now();
     msg.header.frame_id = frame_id_;
     msg.pose.position.x = xy[0];
     msg.pose.position.y = xy[1];
     // SCAN 的手动目标接口只接受 XY，Z 设为 0
     msg.pose.position.z = 0.0;
     msg.pose.orientation.w = 1.0;
     active_ = Active{};
     active_->xy = xy;
     active_->input_stamp = stamp_ns(msg.header.stamp);
     active_->initial_distance = xy_distance(
         xy, {(*base_)[0], (*base_)[1]});
     active_->pose_at_send = *base_;
     active_->trajectory_start_index = trajectory_.poses.size();
     active_->map_version_at_send = map_.version();
     active_->layer_version_at_send = layer_version_;
     active_->goal_id = path_processor_.goal_id();
     active_->attempt_id = path_processor_.attempt_id();
     active_->stage_index = path_processor_.index();
     active_->source_at_send = path_processor_.source();
     active_->reference_route = path_processor_.waypoints();
     const le::XY here{(*base_)[0], (*base_)[1]};
     active_->raw_distance_at_send = nearest_obstacle_distance(
         here, raw_obstacles_, raw_occupancy_wall_);
     active_->inflated_distance_at_send = nearest_obstacle_distance(
         here, external_obstacles_, occupancy_wall_);
     active_->recovery = control_mode_ == le::ExplorationMode::recovery;
     active_->observed_cells_at_send = map_.project({(*base_)[0], (*base_)[1]}).size();
     active_scan_bspline_.reset();
     waypoint_pub_->publish(msg);
     publish_goal_diagnostic(*active_, "sent", "");
     report("goal_sent", detail);
   }

   void publish_planned_path(const le::PlannedRoute &route) {
     nav_msgs::msg::Path path;
     path.header.frame_id = frame_id_;
     path.header.stamp = now();
     for (const auto &xy : route.points) {
       geometry_msgs::msg::PoseStamped pose;
       pose.header = path.header;
       pose.pose.position.x = xy[0];
       pose.pose.position.y = xy[1];
       pose.pose.position.z = base_ ? (*base_)[2] : 0.0;
       pose.pose.orientation.w = 1.0;
       path.poses.push_back(pose);
     }
     planned_path_pub_->publish(path);
     // Candidate paths have separate diagnostic topics. These three topics
     // show only the route that the path processor actually selected.
     nav_msgs::msg::Path empty;
     empty.header = path.header;
     const auto &source = path_processor_.source();
     const bool has_route = !path.poses.empty();
     selected_occupancy_path_pub_->publish(
         has_route && source == "occupancy" ? path : empty);
     selected_layer_path_pub_->publish(
         has_route && source == "layer" ? path : empty);
     selected_recovery_path_pub_->publish(
         has_route && source.rfind("recovery", 0) == 0 ? path : empty);
   }

   void publish_candidate_path(const le::PlannerResult &result) {
     nav_msgs::msg::Path path;
     path.header.frame_id = frame_id_;
     path.header.stamp = now();
     if (result.usable()) {
       for (const auto &xy : result.route.points) {
         geometry_msgs::msg::PoseStamped pose;
         pose.header = path.header;
         pose.pose.position.x = xy[0];
         pose.pose.position.y = xy[1];
         pose.pose.position.z = base_ ? (*base_)[2] : 0.0;
         pose.pose.orientation.w = 1.0;
         path.poses.push_back(pose);
       }
     }
     switch (result.kind) {
       case le::PlannerKind::occupancy: occupancy_plan_pub_->publish(path); break;
       case le::PlannerKind::layer: layer_plan_pub_->publish(path); break;
       case le::PlannerKind::recovery: recovery_plan_pub_->publish(path); break;
     }
   }

   // Reset only exploration's local path display/state. This never calls SCAN.
   void reset_planned_route() {
     path_processor_.clear();
     route_selection_target_.reset();
     route_skipped_nearby_ = 0;
     publish_planned_path({});
     nav_msgs::msg::Path empty;
     empty.header.frame_id = frame_id_;
     empty.header.stamp = now();
     occupancy_plan_pub_->publish(empty);
     layer_plan_pub_->publish(empty);
     recovery_plan_pub_->publish(empty);
   }

   // 路点已经落在当前位置的到达范围内时，不再把它交给 SCAN 规划零距离轨迹。
   bool skip_nearby_route_waypoints() {
     if (!base_) return false;
     // Layer execution waypoints are owned by the layer planner. Forward them
     // unchanged, including their first point.
     if (path_processor_.source() == "layer") return path_processor_.active();
     const le::XY position{(*base_)[0], (*base_)[1]};
     route_skipped_nearby_ += path_processor_.skip_within(position, goal_reach_xy_);
     return path_processor_.active();
   }

   // SCAN 的转向适配器仍订阅旧话题名；转向只可能发生在脱困状态。
   void motion_tick() {
     if (active_ && active_->stamp && (scan_plan_ready_ || active_->reverse) &&
         !active_->complete && !paused_ && ready_ &&
         !observation_only_) {
       std_msgs::msg::Int32 mode;
       mode.data = 3;
       control_mode_pub_->publish(mode);
     }
     if (recovery_.phase() != le::RecoveryPhase::turning) return;
     geometry_msgs::msg::Twist cmd;
     if (!active_ && base_ &&
         wall_now() - base_wall_ <= odom_timeout_ &&
         wall_now() - cloud_wall_ <= cloud_timeout_ && ready_ &&
         !paused_ && !observation_only_) {
       cmd.angular.z = recovery_yaw_rate_;
       std_msgs::msg::Int32 mode;
       mode.data = 3;
       control_mode_pub_->publish(mode);
     }
     recovery_turn_pub_->publish(cmd);
   }

   void stop_recovery_turn() {
     recovery_turn_pub_->publish(geometry_msgs::msg::Twist{});
   }

   // The robot already occupies this position. Turning is allowed with an
   // observed-free centre and no measured obstacle in the swept body disk;
   // unobserved ground under its feet is not asserted to be traversable.
   bool can_turn_here(const std::unordered_map<le::Cell, int, le::CellHash> &grid,
                      const std::vector<le::XY> &obstacles) const {
     if (!base_ || cfg_.resolution <= 0.0) return false;
     const le::XY xy{(*base_)[0], (*base_)[1]};
     const le::Cell centre{static_cast<int>(std::floor(xy[0] / cfg_.resolution)),
                           static_cast<int>(std::floor(xy[1] / cfg_.resolution))};
     const auto at = grid.find(centre);
     if (at != grid.end() && at->second != 0) return false;
     bool nearby_free = false;
     const double radius = cfg_.robot_radius + cfg_.waypoint_safety_margin;
     for (const auto &[cell, state] : grid) {
       const le::XY p{(cell.x + 0.5) * cfg_.resolution,
                      (cell.y + 0.5) * cfg_.resolution};
       if (state == 0 && xy_distance(p, xy) <= 1.0) nearby_free = true;
       if (state != 1) continue;
       if (xy_distance(p, xy) <= radius + cfg_.resolution * 0.7072) return false;
     }
     if (!nearby_free) return false;
     const double extra = std::max(0.0, radius - cfg_.planner_occupancy_radius) +
         cfg_.planner_occupancy_resolution * 0.7072;
     return std::none_of(obstacles.begin(), obstacles.end(),
                         [&](const le::XY &p) { return xy_distance(p, xy) <= extra; });
   }

   void start_recovery(const std::string &reason) {
     if (!base_ || observation_only_ || paused_) return;
     const le::XY xy{(*base_)[0], (*base_)[1]};
     if (!recovery_.active()) {
       recovery_.start(xy);
       recovery_turn_steps_ = 0;
       recovery_reverse_attempted_ = false;
       recovery_reverse_next_index_ = 0;
       recovery_reverse_last_distance_ = 0.0;
       recovery_reverse_refresh_pending_ = false;
       recovery_reverse_sent_ = false;
       recovery_reverse_last_failure_.clear();
       recovery_reverse_rejections_.clear();
       recovery_nearby_attempts_ = 0;
       recovery_history_attempts_ = 0;
       recovery_last_failure_.clear();
       recovery_turn_block_reason_.clear();
       recovery_last_target_distance_ = 0.0;
       recovery_strict_rejections_.clear();
       recovery_direct_rejections_.clear();
     }
     control_mode_ = le::ExplorationMode::recovery;
     report("recovery_started", reason);
   }

   bool try_recovery_reverse(
       const std::unordered_map<le::Cell, int, le::CellHash> &grid,
       const std::vector<le::XY> &obstacles, double current,
       double reverse_distance) {
     recovery_reverse_last_distance_ = reverse_distance;
     const auto reject = [this](const std::string &reason) {
       recovery_reverse_last_failure_ = reason;
       recovery_last_failure_ = reason;
       ++recovery_reverse_rejections_[reason];
       return false;
     };
     if (!base_) return reject("reverse_pose_unavailable");
     if (!base_yaw_) return reject("reverse_yaw_unavailable");
     if (current - yaw_wall_ > odom_timeout_)
       return reject("reverse_yaw_stale");
     if (!planner_occupancy_fresh(current))
       return reject("reverse_scan_occupancy_stale");
     if (recovery_reverse_pub_->get_subscription_count() == 0)
       return reject("reverse_executor_unavailable");
     const le::XY start{(*base_)[0], (*base_)[1]};
     auto route = le::plan_recovery_reverse_route(
         grid, start, *base_yaw_, cfg_, obstacles, reverse_distance);
     if (route.reason != "ok") {
       return reject(route.reason);
     }
     le::PlanningGoal goal;
     goal.goal_id = ++goal_id_;
     goal.attempt_id = ++planning_attempt_id_;
     goal.occupancy_version = map_.version();
     goal.layer_version = layer_version_;
     goal.mode = le::ExplorationMode::recovery;
     goal.target = route.points.back();
     goal.observation = goal.target;
     goal.request_occupancy = false;
     goal.request_layer = false;
     goal.request_recovery = true;
     le::PlannerResult result;
     result.goal = goal;
     result.kind = le::PlannerKind::recovery;
     result.route = route;
     result.waypoints = {goal.target};
     result.source = "recovery_reverse";
     result.reason = "ok";
     if (!path_processor_.load(goal, result, true))
       return reject("reverse_route_load_failed");
     const auto stamp = now().nanoseconds();
     if (stamp <= 0) {
       reset_planned_route();
       return reject("reverse_clock_unavailable");
     }
     Active activity;
     activity.xy = goal.target;
     activity.input_stamp = stamp;
     activity.stamp = stamp;
     activity.recovery = true;
     activity.reverse = true;
     activity.initial_distance = reverse_distance;
     activity.pose_at_send = *base_;
     activity.trajectory_start_index = trajectory_.poses.size();
     activity.map_version_at_send = map_.version();
     activity.layer_version_at_send = layer_version_;
     activity.goal_id = goal.goal_id;
     activity.attempt_id = goal.attempt_id;
     activity.source_at_send = "recovery_reverse";
     activity.reference_route = route.points;
     activity.raw_distance_at_send = nearest_obstacle_distance(
         start, raw_obstacles_, raw_occupancy_wall_);
     activity.inflated_distance_at_send = nearest_obstacle_distance(
         start, external_obstacles_, occupancy_wall_);
     active_ = activity;
     path_processor_.bind_scan_goal(stamp);
     route_selection_target_ = goal.target;
     recovery_last_target_distance_ = reverse_distance;
     publish_candidate_path(result);
     publish_planned_path(route);
     std_msgs::msg::String request;
     std::ostringstream out;
     out << std::setprecision(17) << stamp << '|' << start[0] << '|'
         << start[1] << '|' << goal.target[0] << '|' << goal.target[1]
         << '|' << *base_yaw_;
     request.data = out.str();
     recovery_reverse_pub_->publish(request);
     active_scan_bspline_.reset();
     publish_goal_diagnostic(*active_, "sent", "");
     recovery_reverse_sent_ = true;
     recovery_reverse_last_failure_.clear();
     report("recovery_reverse_sent");
     return true;
   }

   bool handle_recovery(
       const std::unordered_map<le::Cell, int, le::CellHash> &grid,
       const std::vector<le::XY> &obstacles, double current) {
     if (!recovery_.active()) return false;
     if (active_) return true;
     if (path_processor_.active()) return false;
     if (cfg_.global_search && global_frontiers_.valid &&
         global_frontiers_.frontier_cells == 0) {
       if (recovery_.phase() == le::RecoveryPhase::turning) stop_recovery_turn();
       recovery_.finish();
       control_mode_ = le::ExplorationMode::flat;
       return false;
     }
     if (!ready_ || paused_ || observation_only_ ||
         waypoint_pub_->get_subscription_count() == 0 ||
         (require_planner_occupancy_ && !planner_occupancy_fresh(current))) {
       if (recovery_.phase() == le::RecoveryPhase::turning) stop_recovery_turn();
       report("recovery_waiting_for_control");
       return true;
     }
     const le::XY start{(*base_)[0], (*base_)[1]};
     if (recovery_.phase() == le::RecoveryPhase::turning) {
       if (current - yaw_wall_ > odom_timeout_ ||
           xy_distance(start, recovery_turn_origin_) > 0.25 ||
           current - recovery_turn_started_ > recovery_turn_timeout_) {
         stop_recovery_turn();
         recovery_.set_phase(le::RecoveryPhase::history);
         report("recovery_turn_interrupted");
       } else if (recovery_rotation_ >= recovery_turn_step_) {
         stop_recovery_turn();
         ++recovery_turn_steps_;
         recovery_turn_started_ = current;
         recovery_.set_phase(le::RecoveryPhase::settling);
         report("recovery_turn_settling");
       } else report("recovery_turning");
       return true;
     }
     if (recovery_.phase() == le::RecoveryPhase::settling) {
       if (recovery_reverse_refresh_pending_) {
         if (map_.version() <= recovery_reverse_map_version_ ||
             current - cloud_wall_ > cloud_timeout_) {
           report("recovery_reverse_waiting_for_map");
           return true;
         }
         recovery_reverse_refresh_pending_ = false;
         if (le::waypoint_footprint_safety(start, grid, cfg_, obstacles) ==
             le::WaypointSafety::safe) {
           recovery_.finish();
           recovery_turn_steps_ = 0;
           control_mode_ = le::ExplorationMode::flat;
           report("recovery_reverse_completed");
           return true;
         }
         recovery_.set_phase(le::RecoveryPhase::nearby);
         report("recovery_reverse_partial");
         return true;
       }
       if (current - recovery_turn_started_ < 1.5) {
         report("recovery_turn_settling");
         return true;
       }
       recovery_.reset_attempts();
       recovery_nearby_attempts_ = 0;
       recovery_.set_phase(le::RecoveryPhase::nearby);
     }
     if (recovery_.phase() == le::RecoveryPhase::nearby) {
       const auto candidates = recovery_.nearby_candidates(
           grid, start, cfg_, obstacles, recovery_goal_min_distance_,
           recovery_goal_max_distance_);
       const auto batch = std::min<std::size_t>(candidates.size(), 8);
       for (std::size_t i = 0; i < batch; ++i) {
         if (!recovery_.mark_attempted(candidates[i])) continue;
         ++recovery_nearby_attempts_;
         if (prepare_route(candidates[i], grid, obstacles, "recovery_nearby")) return true;
       }
       if (candidates.size() > batch && recovery_nearby_attempts_ < 24) {
         report("recovery_trying_nearby");
         return true;
       }
       if (!recovery_reverse_attempted_) {
         std::array<double, 4> distances{
             0.22, recovery_reverse_distance_, 0.50, 0.60};
         std::sort(distances.begin(), distances.end());
         while (recovery_reverse_next_index_ < distances.size()) {
           const std::size_t index = recovery_reverse_next_index_++;
           const double candidate = distances[index];
           if (index > 0 && candidate <= distances[index - 1] + 1e-6)
             continue;
           if (try_recovery_reverse(grid, obstacles, current, candidate)) return true;
         }
         recovery_reverse_attempted_ = true;
       }
       if (recovery_turn_steps_ < recovery_max_turn_steps_ &&
           base_yaw_ && current - yaw_wall_ <= odom_timeout_ &&
           recovery_turn_pub_->get_subscription_count() > 0 &&
           can_turn_here(grid, obstacles)) {
         recovery_turn_block_reason_.clear();
         recovery_rotation_ = 0.0;
         recovery_turn_origin_ = start;
         recovery_turn_started_ = current;
         recovery_.set_phase(le::RecoveryPhase::turning);
         report("recovery_turning");
         return true;
       }
       if (recovery_turn_steps_ >= recovery_max_turn_steps_)
         recovery_turn_block_reason_ = "turn_limit_reached";
       else if (!base_yaw_ || current - yaw_wall_ > odom_timeout_)
         recovery_turn_block_reason_ = "yaw_unavailable_or_stale";
       else if (recovery_turn_pub_->get_subscription_count() == 0)
         recovery_turn_block_reason_ = "turn_subscriber_missing";
       else
         recovery_turn_block_reason_ = "unsafe_turn_footprint";
       recovery_.set_phase(le::RecoveryPhase::history);
     }
     if (recovery_.phase() == le::RecoveryPhase::history) {
       std::vector<le::XY> history;
       history.reserve(trajectory_.poses.size());
       for (const auto &p : trajectory_.poses)
         history.push_back({p.pose.position.x, p.pose.position.y});
       const auto candidates = recovery_.historical_candidates(
           history, start, grid, cfg_, obstacles);
       const auto batch = std::min<std::size_t>(candidates.size(), 8);
       for (std::size_t i = 0; i < batch; ++i) {
         if (!recovery_.mark_attempted(candidates[i])) continue;
         ++recovery_history_attempts_;
         if (prepare_route(candidates[i], grid, obstacles, "recovery_history")) return true;
       }
       if (candidates.size() > batch && recovery_history_attempts_ < 24) {
         report("recovery_trying_history");
         return true;
       }
       recovery_.set_phase(le::RecoveryPhase::blocked);
       recovery_blocked_observed_ = grid.size();
       recovery_blocked_wall_ = current;
     }
     if (recovery_.phase() == le::RecoveryPhase::blocked &&
         (grid.size() > recovery_blocked_observed_ + min_new_cells_ ||
          current - recovery_blocked_wall_ >= recovery_retry_interval_)) {
       recovery_.reset_attempts();
       recovery_nearby_attempts_ = 0;
       recovery_history_attempts_ = 0;
       recovery_.set_phase(le::RecoveryPhase::nearby);
       recovery_reverse_attempted_ = false;
       recovery_reverse_next_index_ = 0;
       recovery_blocked_observed_ = grid.size();
       recovery_blocked_wall_ = current;
       report("recovery_recheck");
       return true;
     }
     report("recovery_blocked", recovery_last_failure_.empty() ?
            "no_safe_recovery_route" : recovery_last_failure_);
     return true;
   }

   // 控制器先选安全观察点，再让两路规划器独立规划到同一个点。
   bool prepare_route(const le::XY &target,
                      const std::unordered_map<le::Cell, int, le::CellHash> &grid,
                      const std::vector<le::XY> &obstacles,
                      const std::string &detail = "") {
     const bool recovery = control_mode_ == le::ExplorationMode::recovery;
     if (recovery && base_)
       recovery_last_target_distance_ = xy_distance(
           {(*base_)[0], (*base_)[1]}, target);
     if (shared_layer_)
       layer_version_ = shared_layer_->current_version.load();
     const bool layer_fresh = shared_layer_ ?
         (shared_layer_->map != nullptr &&
          shared_layer_->published_surfaces.load() > 0 &&
          wall_now() - shared_layer_->published_wall_seconds.load() <= layer_timeout_) :
         (layer_wall_ > 0.0 && layer_occupied_wall_ > 0.0 &&
         wall_now() - layer_wall_ <= layer_timeout_ &&
         wall_now() - layer_occupied_wall_ <= layer_timeout_ &&
         layer_stamp_ == layer_occupied_stamp_ && !layer_surfaces_.empty());
     auto viewpoints = recovery ? std::vector<le::XY>{target} :
         le::observation_viewpoints(grid, target, cfg_, obstacles);
     if (viewpoints.empty()) {
       if (recovery) recovery_last_failure_ = "no_safe_recovery_viewpoint";
       if (!recovery) suppressed_.push_back({target, wall_now() + cooldown_});
       if (!recovery && base_) {
         recovery_.record_failure({(*base_)[0], (*base_)[1]},
                                  "no_safe_observation_position");
         if (recovery_.limit_reached())
           start_recovery("no_safe_observation_position");
       }
       report("route_unavailable", "no_safe_observation_position");
       return false;
     }
     nav_msgs::msg::Path empty;
     empty.header.frame_id = frame_id_;
     empty.header.stamp = now();
     occupancy_plan_pub_->publish(empty);
     layer_plan_pub_->publish(empty);
     recovery_plan_pub_->publish(empty);
     const auto &free_observations = map_.free_observation_frames();
     const le::PlanningSnapshot snapshot{grid, obstacles, layer_surfaces_, cfg_,
         *base_, layer_resolution_, layer_body_height_, route_segment_length_,
         layer_fresh, &free_observations, &layer_occupied_,
         shared_layer_.get(), layer_planning_timeout_ms_};
     std::string failure = "no_planner_result";
     bool layer_timed_out = false;
     for (std::size_t i = 0; i < std::min<std::size_t>(viewpoints.size(), 12); ++i) {
       le::PlanningGoal goal;
       goal.goal_id = ++goal_id_;
       goal.attempt_id = ++planning_attempt_id_;
       goal.occupancy_version = map_.version();
       goal.layer_version = layer_version_;
       goal.target = target;
       goal.observation = viewpoints[i];
       goal.mode = control_mode_;
       goal.request_layer = layer_fresh && !recovery && !layer_timed_out;
       goal.request_recovery = recovery;
       {
         std_msgs::msg::String message;
         std::ostringstream out;
         out << "{\"goal_id\":" << goal.goal_id << ",\"attempt_id\":"
             << goal.attempt_id << ",\"occupancy_version\":"
             << goal.occupancy_version << ",\"layer_version\":"
             << goal.layer_version << ",\"mode\":\""
             << le::mode_name(goal.mode) << "\",\"planner_mask\":\""
             << (recovery ? "recovery" : goal.request_layer ? "occupancy,layer" : "occupancy")
             << "\",\"x\":" << target[0] << ",\"y\":" << target[1]
             << ",\"observation_x\":" << goal.observation[0]
             << ",\"observation_y\":" << goal.observation[1] << '}';
         message.data = out.str();
         planning_goal_pub_->publish(message);
       }
       auto results = le::plan_candidates(goal, snapshot);
       for (const auto &result : results) {
         if (result.kind == le::PlannerKind::layer &&
             result.reason == "layer_planning_timeout")
           layer_timed_out = true;
         publish_candidate_path(result);
         std_msgs::msg::String message;
         std::ostringstream out;
         out << "{\"goal_id\":" << result.goal.goal_id
             << ",\"attempt_id\":" << result.goal.attempt_id
             << ",\"occupancy_version\":" << result.goal.occupancy_version
             << ",\"layer_version\":" << result.goal.layer_version
             << ",\"planner\":\"" << le::planner_name(result.kind)
             << "\",\"usable\":" << (result.usable() ? "true" : "false")
             << ",\"waypoints\":" << result.waypoints.size()
             << ",\"planning_ms\":" << result.planning_ms
             << ",\"reason\":\"" << result.reason << "\"";
         if (recovery) {
           out << ",\"target_distance\":" << recovery_last_target_distance_
               << ",\"direct_limit\":" << route_segment_length_
               << ",\"strict_reason\":\"" << result.strict_reason
               << "\",\"direct_reason\":\"" << result.direct_reason << "\"";
         }
         out << '}';
         message.data = out.str();
         planning_result_pub_->publish(message);
         if (recovery) {
           if (!result.strict_reason.empty() && result.strict_reason != "ok")
             ++recovery_strict_rejections_[result.strict_reason];
           if (!result.direct_reason.empty() &&
               result.direct_reason != "unverified_recovery_connection")
             ++recovery_direct_rejections_[result.direct_reason];
         }
         if (!result.usable()) failure = result.reason;
       }
       const auto *chosen = le::select_plan(goal, results);
       if (!chosen || !path_processor_.load(goal, *chosen, recovery, &snapshot)) continue;
       route_selection_target_ = target;
       route_skipped_nearby_ = 0;
       if (!skip_nearby_route_waypoints()) {
         reset_planned_route();
         failure = "observation_position_already_reached";
         continue;
       }
       control_mode_ = goal.mode;
       le::PlannedRoute sent_route;
       sent_route.points = path_processor_.waypoints();
       publish_planned_path(sent_route);
       RCLCPP_INFO(get_logger(),
           "Route goal=%llu frontier=(%.2f, %.2f) observation=(%.2f, %.2f) planner=%s stages=%zu",
           static_cast<unsigned long long>(goal.goal_id), target[0], target[1],
           goal.observation[0], goal.observation[1], path_processor_.source().c_str(),
           path_processor_.waypoints().size());
       publish_target(path_processor_.current(),
                      recovery ? "recovery_nearby" : path_processor_.source() + "_route" +
                      (detail == "low_yield_frontier" ? "_low_yield" : ""));
       if (recovery) recovery_last_failure_.clear();
       return true;
     }
     if (!recovery) suppressed_.push_back({target, wall_now() + cooldown_});
     if (recovery) recovery_last_failure_ = failure;
     if (!recovery && base_) {
       recovery_.record_failure({(*base_)[0], (*base_)[1]}, failure);
       if (recovery_.limit_reached()) start_recovery(failure);
     }
     report("route_unavailable", failure);
     return false;
   }

   void advance_route() {
     if (!path_processor_.active()) return;
     if (!path_processor_.advance()) reset_planned_route();
   }

   // 记录目标结束时地图是否增长。
   void finish_active_goal() {
     // 尚无对应数据或目标时，进入此分支。
     if (!active_ || !base_) return;
     auto observed = map_.project({(*base_)[0], (*base_)[1]}).size();
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (path_processor_.waypoints().size() > path_processor_.index() + 1) return;
     if (!active_->recovery) {
       goal_progress_gate_.finish(active_->xy, active_->observed_cells_at_send, observed);
       // 低收益目标可以在没有高收益目标时重试，但短时间内不重复同一点。
       const auto gained = observed - std::min(observed, active_->observed_cells_at_send);
       if (gained < min_new_cells_ && low_yield_retry_cooldown_ > 0.0)
         suppressed_.push_back({route_selection_target_.value_or(active_->xy),
                                wall_now() + low_yield_retry_cooldown_});
     }
   }

   // ---------------------------------------------------------------
   // 在 RViz 中发布或删除前沿的 Marker（球体）
   // ---------------------------------------------------------------
   // 更新 RViz 中的前沿目标标记。
   void publish_marker(const std::optional<le::XY> &goal) {
     visualization_msgs::msg::MarkerArray array;
     visualization_msgs::msg::Marker marker;
     marker.header.frame_id = frame_id_;
     marker.header.stamp = now();
     marker.ns = "selected_frontier";
     marker.id = 0;
     marker.type = visualization_msgs::msg::Marker::SPHERE;
     marker.action = goal ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
     marker.pose.orientation.w = 1.0;
     // 根据当前目标是否存在或是否完成来决定下一步。
     if (goal) {
       marker.pose.position.x = (*goal)[0];
       marker.pose.position.y = (*goal)[1];
       marker.pose.position.z = (*base_)[2];
       marker.scale.x = marker.scale.y = marker.scale.z = 0.35;
       marker.color.r = 0.0; marker.color.g = 0.9; marker.color.b = 1.0; marker.color.a = 0.9;
     }
     // 把本次结果放入容器，供下一轮处理。
     array.markers.push_back(marker);
     marker_pub_->publish(array);
   }

   // ---------------------------------------------------------------
   // 主循环 tick：每秒执行状态调度、前沿搜索与目标发布
   // ---------------------------------------------------------------
   // 每秒执行一次探索状态机。
   void tick() {
     double current = wall_now();
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (!trajectory_.poses.empty()) {
       trajectory_.header.stamp = now();
       trajectory_pub_->publish(trajectory_);
     }
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     // 数据失效检查：基座、里程计、点云必须在对应超时范围内
     // 感知数据过期时暂停新的选点；已下发路点仍由 SCAN 执行。
     if (!base_ || current - base_wall_ > odom_timeout_ ||
         cloud_wall_ == 0.0 || current - cloud_wall_ > cloud_timeout_) {
       if (recovery_.phase() == le::RecoveryPhase::turning) {
         stop_recovery_turn();
         recovery_.set_phase(le::RecoveryPhase::history);
       }
       // 清空临时状态，准备下一轮处理。
       empty_since_.reset();
       report("waiting_for_fresh_data");
       // 结束当前函数，不再执行后面的语句。
       return;
     }
     // 清理已过期的 suppressed 前沿（冷却结束）
     // 从容器中取走已处理或失效的数据。
     suppressed_.erase(std::remove_if(suppressed_.begin(), suppressed_.end(),
         [current](const Suppressed &s) { return s.expiry <= current; }), suppressed_.end());
     auto projected = map_.project({(*base_)[0], (*base_)[1]});
     auto route_grid = cfg_.global_search ? map_.project_all() : projected;
     coverage_ = le::assess_reachable_coverage(projected, cfg_,
                                               {(*base_)[0], (*base_)[1]});
     publish_coverage_grid(projected);
     if (cfg_.global_search) {
       auto dirty = map_.take_dirty_cells();
       if (!frontier_index_initialized_) {
         frontier_index_.rebuild(route_grid);
         frontier_index_initialized_ = true;
       } else if (!dirty.empty()) {
         frontier_index_.update(route_grid, dirty);
       }
       global_frontiers_ = le::assess_global_frontiers(
           route_grid, cfg_, {(*base_)[0], (*base_)[1]},
           &frontier_index_.cells());
     }

     // A sent waypoint stays active until SCAN acknowledges completion or an
     // external manual goal takes over. Map changes and elapsed time do not
     // invalidate it here.
     if (active_) {
       empty_since_.reset();
       report(active_->stamp ? "waiting_for_scan_completion" :
              "waiting_for_scan_forward");
       return;
     }

     // ------------------- 没有活动目标：尝试寻找新前沿 -------------------
     // 地图已经达到体素数量上限，不能继续插入。
     if (map_.size() >= cfg_.max_voxels) {
       empty_since_.reset();
       report("map_capacity_reached");
       // 结束当前函数，不再执行后面的语句。
       return;
     }
     if (control_mode_ == le::ExplorationMode::waiting_initial_data)
       control_mode_ = le::ExplorationMode::flat;
     static const std::vector<le::XY> no_recovery_obstacles;
     const auto &recovery_obstacles = planner_occupancy_fresh(current) ?
         external_obstacles_ : no_recovery_obstacles;
     if (handle_recovery(route_grid, recovery_obstacles, current)) return;
     // 投影到水平面，检查是否还有 free（未占据）空间
     // 矩形覆盖率只供显示；全局完成不依赖预设矩形是否覆盖整片场地。
     unresolved_unknown_cells_ = 0;
     bool has_free = cfg_.global_search ? global_frontiers_.valid :
         std::any_of(projected.begin(), projected.end(),
                     [](const auto &item) { return item.second == 0; });
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (!has_free) {
       empty_since_.reset();
       if (!route_grid.empty() && !observation_only_ && !paused_ && ready_ &&
           waypoint_pub_->get_subscription_count() > 0) {
         start_recovery("no_reachability_seed");
         if (handle_recovery(route_grid, recovery_obstacles, current)) return;
       }
       report("waiting_for_free_space");
       // 结束当前函数，不再执行后面的语句。
       return;
     }
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (require_planner_occupancy_ && !planner_occupancy_fresh(current)) {
       empty_since_.reset();
       report("waiting_for_planner_occupancy");
       // 结束当前函数，不再执行后面的语句。
       return;
     }
     static const std::vector<le::XY> no_route_obstacles;
     const auto &route_obstacles = planner_occupancy_fresh(current) ?
         external_obstacles_ : no_route_obstacles;
     if (path_processor_.active()) {
       if (observation_only_ || paused_ || !ready_ ||
           waypoint_pub_->get_subscription_count() == 0) {
         report(paused_ ? "paused" : "waiting_for_waypoint_coordinator");
         return;
       }
       // The route advances only after the preceding SCAN completion callback.
       publish_target(path_processor_.current(),
                      path_processor_.recovery() ? "recovery_route" :
                      path_processor_.source() + "_route");
       return;
     }
     if (cfg_.global_search && !global_frontiers_.valid) {
       empty_since_.reset();
       if (!route_grid.empty() && !observation_only_ && !paused_ && ready_ &&
           waypoint_pub_->get_subscription_count() > 0) {
         start_recovery("no_reachability_seed");
         if (handle_recovery(route_grid, route_obstacles, current)) return;
       }
       report("waiting_for_reachability_seed");
       return;
     }
     if (cfg_.global_search && global_frontiers_.frontier_cells == 0) {
       if (paused_) {
         empty_since_.reset();
         report("paused");
         return;
       }
       if (!empty_since_) empty_since_ = current;
       if (current - *empty_since_ >= no_frontier_hold_) {
         // 只有完成前才独立全图复核；缓存遗漏会触发修复而非误报完成。
         auto audit = le::assess_global_frontiers(
             route_grid, cfg_, {(*base_)[0], (*base_)[1]});
         if (!audit.valid) {
           empty_since_.reset();
           report("waiting_for_reachability_seed");
           return;
         }
         if (audit.frontier_cells > 0) {
           frontier_index_.rebuild(route_grid);
           ++frontier_index_repairs_;
           global_frontiers_ = std::move(audit);
           empty_since_.reset();
           report("frontier_index_repaired");
           return;
         }
       }
       unresolved_unknown_cells_ = 0;
       last_frontier_cells_ = last_frontier_clusters_ = 0;
       last_reachable_frontier_cells_ = last_candidate_goals_ = 0;
       last_suppressed_goals_ = last_open_start_neighbors_ = 0;
       publish_marker(std::nullopt);
       report(current - *empty_since_ >= no_frontier_hold_ ?
              "exploration_complete" : "waiting_for_coverage_confirmation");
       return;
     }
     // 构造被抑制的前沿列表（用于前沿选择的过滤）
     std::vector<le::XY> failed;
     // 依次处理水平坐标。
     for (const auto &s : suppressed_) failed.push_back(s.xy);
     const auto temporary_failures = failed;
     auto unproductive = goal_progress_gate_.blocked(projected.size());
     failed.insert(failed.end(), unproductive.begin(), unproductive.end());
     static const std::vector<le::XY> no_external_obstacles;
     const auto &obstacles = planner_occupancy_fresh(current) ?
         external_obstacles_ : no_external_obstacles;
     // 将右侧计算值存入 selection（前沿搜索结果）。
     auto selection = le::select_frontier(
         map_, *base_, cfg_, failed, obstacles, false,
         cfg_.global_search ? &global_frontiers_.cells : nullptr);
     bool search_all_observed = false;
     if (cfg_.global_search && !selection.goal && global_frontiers_.frontier_cells > 0) {
       auto beyond_bounds = le::select_frontier(map_, *base_, cfg_, failed,
                                                obstacles, true, &global_frontiers_.cells);
       if (beyond_bounds.goal || beyond_bounds.frontier_cells > selection.frontier_cells) {
         selection = std::move(beyond_bounds);
         search_all_observed = true;
       }
     }
     // 高收益目标用尽时，允许重新考虑低收益目标；仍避开正在冷却的失败目标。
     bool low_yield_goal = false;
     if (!selection.goal && !unproductive.empty()) {
       auto lower_yield = le::select_frontier(
           map_, *base_, cfg_, temporary_failures, obstacles, false,
           cfg_.global_search ? &global_frontiers_.cells : nullptr);
       bool lower_search_all = false;
       if (cfg_.global_search && !lower_yield.goal && global_frontiers_.frontier_cells > 0) {
         auto beyond_bounds = le::select_frontier(
             map_, *base_, cfg_, temporary_failures, obstacles, true,
             &global_frontiers_.cells);
         if (beyond_bounds.goal || beyond_bounds.frontier_cells > lower_yield.frontier_cells) {
           lower_yield = std::move(beyond_bounds);
           lower_search_all = true;
         }
       }
       if (lower_yield.goal) {
         selection = std::move(lower_yield);
         search_all_observed = lower_search_all;
         low_yield_goal = true;
       }
     }
     bool residual_goal = false;
     // 检查未知边界是否仍存在或是否满足簇规模。
     if (cfg_.global_search &&
         (selection.frontier_cells > 0 || global_frontiers_.frontier_cells > 0) &&
         selection.clusters == 0) {
       // 只有表达式为真，才执行下面缩进或花括号内的代码。
       if (!residual_since_) residual_since_ = current;
       // 检查未知边界是否仍存在或是否满足簇规模。
       if (current - *residual_since_ >= no_frontier_hold_) {
         auto residual_cfg = cfg_;
         residual_cfg.min_frontier_cells = 1;
         auto residual = le::select_frontier(
             map_, *base_, residual_cfg, failed, obstacles, search_all_observed,
             &global_frontiers_.cells);
         if (!residual.goal && !search_all_observed &&
             global_frontiers_.frontier_cells > 0)
           residual = le::select_frontier(map_, *base_, residual_cfg, failed,
                                          obstacles, true, &global_frontiers_.cells);
         // 根据当前目标是否存在或是否完成来决定下一步。
         if (residual.goal) {
           selection.goal = residual.goal;
           selection.score = residual.score;
           residual_goal = true;
         }
       }
     } else {
       // 清空临时状态，准备下一轮处理。
       residual_since_.reset();
     }
     last_frontier_cells_ = selection.frontier_cells;
     last_frontier_clusters_ = selection.clusters;
     last_reachable_frontier_cells_ = selection.reachable_frontier_cells;
     last_candidate_goals_ = selection.candidate_goals;
     last_suppressed_goals_ = selection.suppressed_goals;
     last_open_start_neighbors_ = selection.open_start_neighbors;
     bool observation_goal = false;
     if (cfg_.global_search && !selection.goal && selection.frontier_cells == 0 &&
         global_frontiers_.frontier_cells > 0 && coverage_.reachable_unknown_cells > 0) {
       selection.goal = le::select_observation_goal(map_, *base_, cfg_, projected,
                                                     failed, obstacles);
       observation_goal = selection.goal.has_value();
     }
     // 在 RViz 中展示选中的前沿（若有）
     publish_marker(selection.goal);
     // 根据当前目标是否存在或是否完成来决定下一步。
     if (!selection.goal) {
       if (cfg_.global_search && global_frontiers_.frontier_cells > 0 &&
           !observation_only_ && !paused_ && ready_ &&
           waypoint_pub_->get_subscription_count() > 0) {
         start_recovery("no_reachable_frontier_goal");
         if (handle_recovery(route_grid, obstacles, current)) return;
       }
       if (cfg_.global_search) unresolved_unknown_cells_ = coverage_.reachable_unknown_cells;
       // 检查未知边界是否仍存在或是否满足簇规模。
       if (cfg_.global_search ? global_frontiers_.frontier_cells > 0 :
           selection.frontier_cells > 0) {
         // 清空临时状态，准备下一轮处理。
         empty_since_.reset();
         report(selection.frontier_cells == 0 ? "global_frontiers_unresolved" :
                selection.clusters > 0 ? "frontiers_unreachable_or_suppressed" :
                "frontiers_below_cluster_threshold");
       } else {
         // 全图模式已在上方通过独立复核退出；这里只处理局部模式。
         if (!empty_since_) empty_since_ = current;
         report(current - *empty_since_ >= no_frontier_hold_ ?
                "current_region_complete" : "waiting_for_frontier");
       }
       // 结束当前函数，不再执行后面的语句。
       return;
     }
     // 清空临时状态，准备下一轮处理。
     empty_since_.reset();
     // 只有表达式为真，才执行下面缩进或花括号内的代码。
     if (observation_only_) report("observation_only");
     // 上一条件不满足时，再检查这个分支。
     else if (paused_) report("paused");
     // 上一条件不满足时，再检查这个分支。
     else if (!ready_ || waypoint_pub_->get_subscription_count() == 0)
       report("waiting_for_waypoint_coordinator");
     // 前面的条件不成立时，执行这个备选分支。
     else
     prepare_route(*selection.goal, route_grid, obstacles,
                   observation_goal ? "supplemental_observation" :
                   (residual_goal ? "residual_frontier" :
                    low_yield_goal ? "low_yield_frontier" : ""));
   }

   // ===============================================================
   // 成员变量（配置、地图、状态、ROS 句柄等）
   // ===============================================================
   le::Settings cfg_; // 参数设置对象
   le::VoxelMap map_; // 体素地图实例
   std::string frame_id_, last_status_; // 坐标系 ID 与最近一次状态字符串
   int point_stride_ = 8, max_points_ = 1000; // 点云抽稀步长与最大点数
   double sync_tolerance_ = 0.03, odom_timeout_ = 1.0, cloud_timeout_ = 2.0; // 同步容差与超时阈值
   double goal_reach_xy_ = 0.55, cooldown_ = 45.0;
   double low_yield_retry_cooldown_ = 8.0;
   std::size_t min_new_cells_ = 8;
   double no_frontier_hold_ = 8.0; // 前沿缺失时的保持时间阈值
   double occupancy_timeout_ = 2.0, occupancy_height_tolerance_ = 0.5; // SCAN 障碍数据有效期与高度筛选距离
   bool require_planner_occupancy_ = false; // true 时没有新鲜 SCAN 障碍图就不发新目标
   bool observation_only_ = true, ready_ = false, paused_ = false; // 运行模式标志
   bool bounds_relative_to_start_ = false, bounds_initialized_ = false; // 是否把探索边界平移到起始机体坐标
   double base_wall_ = 0.0, cloud_wall_ = 0.0; // 时间戳记录
   double occupancy_wall_ = 0.0; // 最近收到规划器障碍图的本机时间
   double raw_occupancy_wall_ = 0.0;
   double layer_wall_ = 0.0;
   double layer_resolution_ = 0.10, layer_body_height_ = 0.32;
   double layer_timeout_ = 2.5, route_segment_length_ = 3.0;
   double recovery_yaw_rate_ = 0.35, recovery_turn_step_ = 0.70;
   double recovery_turn_timeout_ = 8.0, recovery_retry_interval_ = 10.0;
   double recovery_goal_min_distance_ = 0.65, recovery_goal_max_distance_ = 2.0;
   double recovery_reverse_distance_ = 0.35;
   double recovery_reverse_last_distance_ = 0.0;
   std::size_t recovery_reverse_next_index_ = 0;
   std::size_t recovery_reverse_map_version_ = 0;
   bool recovery_reverse_refresh_pending_ = false;
   bool recovery_reverse_attempted_ = false;
   bool recovery_reverse_sent_ = false;
   int recovery_max_turn_steps_ = 4, recovery_turn_steps_ = 0;
   int recovery_nearby_attempts_ = 0, recovery_history_attempts_ = 0;
   double recovery_rotation_ = 0.0, recovery_turn_started_ = 0.0;
   double recovery_blocked_wall_ = 0.0;
   double yaw_wall_ = 0.0;
   std::size_t recovery_blocked_observed_ = 0;
   std::string recovery_last_failure_;
   std::string recovery_reverse_last_failure_;
   std::string recovery_turn_block_reason_;
   double recovery_last_target_distance_ = 0.0;
   std::map<std::string, int> recovery_strict_rejections_;
   std::map<std::string, int> recovery_direct_rejections_;
   std::map<std::string, int> recovery_reverse_rejections_;
   le::XY recovery_turn_origin_{0.0, 0.0};
   std::optional<double> base_yaw_;
   le::PathProcessor path_processor_;
   std::optional<le::XY> route_selection_target_; // 本轮选中的探索目标，而非分段路点。
   std::size_t route_skipped_nearby_ = 0;
   le::ExplorationMode control_mode_ = le::ExplorationMode::waiting_initial_data;
   std::uint64_t goal_id_ = 0, planning_attempt_id_ = 0;
   std::size_t layer_version_ = 0;
   std::vector<le::LayerSurface> layer_surfaces_;
   std::shared_ptr<le::SharedLayerMap> shared_layer_;
   double layer_planning_timeout_ms_ = 2000.0;
   std::vector<le::Point> layer_occupied_;
   std::int64_t layer_stamp_ = 0, layer_occupied_stamp_ = 0;
   double layer_occupied_wall_ = 0.0;
   std::optional<le::Point> base_; // 基座位置（x,y,z）
   std::optional<Active> active_; // 当前活动目标信息
   std::optional<scan_planner::msg::Bspline> active_scan_bspline_;
   std::optional<std::pair<std::int64_t, std::string>> pending_scan_failure_;
   std::size_t scan_failure_count_ = 0;
   std::string scan_last_failure_;
   std::optional<double> empty_since_; // 空前沿持续时间记录
   std::optional<double> residual_since_; // 零散前沿持续出现的起始时间；空值表示尚未开始计时
   std::size_t last_frontier_cells_ = 0, last_frontier_clusters_ = 0; // 上轮原始前沿格数与有效簇数
   std::size_t last_reachable_frontier_cells_ = 0, last_candidate_goals_ = 0; // 上轮可达格与候选目标数
   std::size_t last_suppressed_goals_ = 0, last_open_start_neighbors_ = 0; // 上轮被屏蔽目标与起点开放邻格数
   le::Coverage coverage_; // 全局边界内逐格验收统计
   le::GlobalFrontiers global_frontiers_; // 完整累计地图的原始前沿验收
   le::FrontierIndex frontier_index_; // 随二维占据状态增删的当前原始前沿。
   bool frontier_index_initialized_ = false;
   std::size_t frontier_index_repairs_ = 0;
   std::size_t unresolved_unknown_cells_ = 0; // 当前没有可用目标时尚未覆盖的格数
   nav_msgs::msg::Path trajectory_; // 积累的机体行走轨迹，发布到 RViz
   std::deque<std::pair<int64_t, le::Point>> sensor_poses_; // 里程计姿态缓存
   std::deque<std::pair<int64_t, le::Point>> base_poses_; // 带时间戳的近期机体位姿缓存
   std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_clouds_; // 点云缓存
   std::vector<le::XY> external_obstacles_; // SCAN 膨胀障碍点的水平坐标
   std::vector<le::XY> raw_obstacles_;
   std::vector<Suppressed> suppressed_; // 被抑制的前沿列表（冷却）
   le::RecoveryPlanner recovery_{3, 2.0}; // 同一物理区域连续失败的脱困状态
   le::GoalProgressGate goal_progress_gate_{8}; // 屏蔽到达后无地图收益的目标

   // ROS 接口句柄
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_; // 探索目标发布者
   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // RViz 前沿标记发布者
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_; // JSON 状态发布者
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_diagnostic_pub_;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr diagnostic_reference_pub_;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr diagnostic_actual_pub_;
   rclcpp::Publisher<scan_planner::msg::Bspline>::SharedPtr diagnostic_bspline_pub_;
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_goal_pub_;
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_result_pub_;
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_result_pub_;
   rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr coverage_pub_; // 全局验收网格
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_; // 行走轨迹发布者
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_occupancy_path_pub_;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_layer_path_pub_;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_recovery_path_pub_;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr occupancy_plan_pub_;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr layer_plan_pub_;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr recovery_plan_pub_;
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr recovery_turn_pub_;
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_reverse_pub_;
   rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr control_mode_pub_;
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_; // 已配准点云订阅者
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_sub_; // SCAN 障碍图订阅者
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_occupancy_sub_;
   rclcpp::Subscription<scan_planner::msg::Bspline>::SharedPtr scan_bspline_sub_;
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr layer_sub_;
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr layer_occupied_sub_;
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sensor_sub_, base_sub_; // 雷达与机体位姿订阅者
   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_, complete_sub_, plan_ready_sub_; // 就绪与完成信号订阅者
   bool scan_plan_ready_ = false;
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scan_failure_sub_;
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_reverse_result_sub_;
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input_sub_, goal_sub_, ack_sub_; // 输入、转发、停车回执订阅者
   rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_; // 手动暂停/恢复服务
   rclcpp::TimerBase::SharedPtr timer_; // 每秒执行 tick 的 ROS 计时器
   rclcpp::TimerBase::SharedPtr motion_timer_;
};

std::shared_ptr<rclcpp::Node> make_explorer_node(
    std::shared_ptr<le::SharedLayerMap> shared) {
  return std::make_shared<Explorer>(std::move(shared));
}

#ifndef LEGBOT_EXPLORATION_NO_MAIN
int main(int argc, char **argv) { // 程序入口，接受命令行参数（ROS2 会通过它解析 remap、参数等）
  rclcpp::init(argc, argv); // 初始化 ROS 2 客户端库，必须在创建任何 Node 之前调用
  try {  // 使用 try/catch 捕获后续可能抛出的异常，防止异常导致进程直接崩溃
    rclcpp::spin(std::make_shared<Explorer>()); // 创建 Explorer 节点实例（派生自 rclcpp::Node）并进入事件循环
                                                // spin 会一直阻塞（主动轮询），持续调用节点内部的计时器、回调等，直到收到 shutdown 信号
  } catch (const std::exception &exc) {         // 捕获所有 std::exception 派生的异常
    RCLCPP_ERROR(rclcpp::get_logger("legbot_exploration"), "Explorer failed: %s", exc.what()); // 使用 ROS2 日志宏在控制台输出错误信息，便于调试 将异常的描述 (`exc.what()`) 拼进日志
    rclcpp::shutdown(); // 发生致命错误时，主动关闭 ROS2 客户端库，释放底层资源（DDS、线程等）
    // 把此函数算出的值交还给调用者。
    return 1;
  }
  rclcpp::shutdown(); // 正常退出时也要调用 shutdown，确保资源释放
  return 0; // 返回 0 表示程序正常退出
}
#endif
