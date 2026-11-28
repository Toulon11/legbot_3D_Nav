#include "exploration_controller.hpp"
#include "legbot_exploration/exploration_node_factories.hpp"

#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_listener.hpp>

using namespace std::chrono_literals;
namespace le = legbot_exploration;

namespace {

template<typename Message>
auto publish_with(rclcpp::Node &node, const std::string &topic, const rclcpp::QoS &qos) {
  return [publisher = node.create_publisher<Message>(topic, qos)](const Message &message) {
    publisher->publish(message);
  };
}

le::Settings declare_map_settings(rclcpp::Node &node) {
  le::Settings cfg;
  cfg.resolution = node.declare_parameter<double>("resolution", cfg.resolution);
  cfg.max_ray_range = node.declare_parameter<double>("max_ray_range", cfg.max_ray_range);
  int max_voxels = node.declare_parameter<int>("max_voxels", 900000);
  // 地图已经达到体素数量上限，不能继续插入。
  if (max_voxels < 1) throw std::invalid_argument("max_voxels must be positive");
  cfg.max_voxels = static_cast<std::size_t>(max_voxels);
  cfg.obstacle_min_relative_z = node.declare_parameter<double>("obstacle_min_relative_z", cfg.obstacle_min_relative_z);
  cfg.obstacle_max_relative_z = node.declare_parameter<double>("obstacle_max_relative_z", cfg.obstacle_max_relative_z);
  cfg.free_min_relative_z = node.declare_parameter<double>("free_min_relative_z", cfg.free_min_relative_z);
  cfg.robot_radius = node.declare_parameter<double>("robot_radius", cfg.robot_radius);
  cfg.planner_occupancy_radius = node.declare_parameter<double>(
      "planner_occupancy_radius", cfg.planner_occupancy_radius);
  cfg.planner_occupancy_resolution = node.declare_parameter<double>(
      "planner_occupancy_resolution", cfg.planner_occupancy_resolution);
  cfg.waypoint_safety_margin = node.declare_parameter<double>(
      "waypoint_safety_margin", cfg.waypoint_safety_margin);
  if (!std::isfinite(cfg.planner_occupancy_radius) || cfg.planner_occupancy_radius < 0.0)
    throw std::invalid_argument("planner_occupancy_radius must be finite and nonnegative");
  if (!std::isfinite(cfg.planner_occupancy_resolution) ||
      cfg.planner_occupancy_resolution <= 0.0 ||
      !std::isfinite(cfg.waypoint_safety_margin) || cfg.waypoint_safety_margin < 0.0)
    throw std::invalid_argument("waypoint safety parameters must be finite and valid");
  cfg.local_search_radius = node.declare_parameter<double>("local_search_radius", cfg.local_search_radius);
  cfg.global_search = node.declare_parameter<bool>("global_search", cfg.global_search);
  cfg.exploration_min_x = node.declare_parameter<double>("exploration_min_x", cfg.exploration_min_x);
  cfg.exploration_max_x = node.declare_parameter<double>("exploration_max_x", cfg.exploration_max_x);
  cfg.exploration_min_y = node.declare_parameter<double>("exploration_min_y", cfg.exploration_min_y);
  cfg.exploration_max_y = node.declare_parameter<double>("exploration_max_y", cfg.exploration_max_y);
  cfg.min_frontier_cells = node.declare_parameter<int>("min_frontier_cells", cfg.min_frontier_cells);
  cfg.frontier_gain_weight = node.declare_parameter<double>("frontier_gain_weight", cfg.frontier_gain_weight);
  cfg.path_cost_weight = node.declare_parameter<double>("path_cost_weight", cfg.path_cost_weight);
  cfg.unknown_path_multiplier = node.declare_parameter<double>("unknown_path_multiplier", cfg.unknown_path_multiplier);
  cfg.free_observation_saturation = node.declare_parameter<int>(
      "free_observation_saturation", cfg.free_observation_saturation);
  cfg.low_evidence_cost_weight = node.declare_parameter<double>(
      "low_evidence_cost_weight", cfg.low_evidence_cost_weight);
  cfg.shortcut_cost_tolerance = node.declare_parameter<double>(
      "shortcut_cost_tolerance", cfg.shortcut_cost_tolerance);
  if (cfg.free_observation_saturation < 1 ||
      cfg.free_observation_saturation > 255 ||
      !std::isfinite(cfg.low_evidence_cost_weight) ||
      cfg.low_evidence_cost_weight < 0.0 ||
      !std::isfinite(cfg.shortcut_cost_tolerance) ||
      cfg.shortcut_cost_tolerance < 0.0)
    throw std::invalid_argument("invalid occupancy evidence parameters");
  return cfg;
}

le::ExplorationSettings declare_exploration_settings(
    rclcpp::Node &node, const std::shared_ptr<le::SharedLayerMap> &shared) {
  le::ExplorationSettings options;
  options.map = declare_map_settings(node);
  options.frame_id_ = node.declare_parameter<std::string>("frame_id", "map");
  options.point_stride_ = node.declare_parameter<int>("point_stride", 8);                       // 点云抽稀步长
  options.max_points_ = node.declare_parameter<int>("max_points_per_cloud", 1000);           // 单帧最大采点数
  options.sync_tolerance_ = node.declare_parameter<double>("sensor_sync_tolerance", 0.03);   // 同步容差（秒）
  options.odom_timeout_ = node.declare_parameter<double>("odom_timeout", 1.0);
  options.cloud_timeout_ = node.declare_parameter<double>("cloud_timeout", 2.0);
  options.goal_reach_xy_ = node.declare_parameter<double>("goal_reach_xy", 0.55);
  options.observation_min_distance_=node.declare_parameter<double>("observation_min_distance",0.50);
  options.observation_goal_xy_tolerance_=node.declare_parameter<double>("observation_goal_xy_tolerance",0.15);
  if(!std::isfinite(options.observation_min_distance_) || !std::isfinite(options.observation_goal_xy_tolerance_) ||
     options.observation_goal_xy_tolerance_<scan_planner::msg::GoalRequest::MIN_XY_TOLERANCE ||
     options.observation_goal_xy_tolerance_>scan_planner::msg::GoalRequest::MAX_XY_TOLERANCE ||
     options.observation_min_distance_<=options.observation_goal_xy_tolerance_)
    throw std::invalid_argument("observation_min_distance must exceed valid observation_goal_xy_tolerance");
  options.cooldown_ = node.declare_parameter<double>("failed_frontier_cooldown", 45.0);
  options.pct_round_budget_=node.declare_parameter<double>("pct_candidate_round_budget",1.0);
  options.pct_detour_ratio_=node.declare_parameter<double>("pct_detour_ratio",2.0);
  options.pct_detour_excess_=node.declare_parameter<double>("pct_detour_excess",1.5);
  options.pct_compare_clusters_=node.declare_parameter<int>("pct_compare_clusters",3);
  if(!std::isfinite(options.pct_detour_ratio_) || options.pct_detour_ratio_<=1. ||
     !std::isfinite(options.pct_detour_excess_) || options.pct_detour_excess_<0. ||
     options.pct_compare_clusters_<1 || options.pct_compare_clusters_>3)
    throw std::invalid_argument("invalid PCT detour thresholds or comparison count (1..3)");
  options.pct_local_above_=node.declare_parameter<double>("pct_local_height_above",3.5);
  options.pct_local_below_=node.declare_parameter<double>("pct_local_height_below",1.5);
  options.pct_direction_weight_=node.declare_parameter<double>("pct_direction_weight",.5);
  options.cross_layer_enabled_=node.declare_parameter<bool>("cross_layer_enabled",true);
  options.floor_exploration_=node.declare_parameter<bool>("pct_floor_exploration",false);
  options.skip_start_floor_exploration_=node.declare_parameter<bool>("skip_start_floor_exploration",false);
  options.mission_return_home_=node.declare_parameter<bool>("mission_return_home",false);
  options.mission_target_height_=node.declare_parameter<double>("mission_target_height",4.43);
  const auto home=node.declare_parameter<std::vector<double>>("mission_home",{-6.75,9.60,0.});
  if(home.size()!=3 || !std::all_of(home.begin(),home.end(),[](double v){return std::isfinite(v);}) ||
     !std::isfinite(options.mission_target_height_))throw std::invalid_argument("invalid home mission geometry");
  std::copy(home.begin(),home.end(),options.mission_home_.begin());
  options.floor_height_tolerance_=node.declare_parameter<double>("pct_floor_height_tolerance",.08);
  options.foot_force_min_=node.declare_parameter<double>("pct_foot_contact_force",12.);
  if(!std::isfinite(options.floor_height_tolerance_) || options.floor_height_tolerance_<=0. ||
     !std::isfinite(options.foot_force_min_) || options.foot_force_min_<=0.)
    throw std::invalid_argument("invalid PCT floor settings");
  options.cross_layer.minimum_rise=options.floor_height_tolerance_+1e-7;
  options.cross_layer.height_slack=node.declare_parameter<double>("cross_layer_height_slack",.05);
  options.cross_layer.progress_slack=options.observation_goal_xy_tolerance_;
  if(!std::isfinite(options.cross_layer.minimum_rise) || !std::isfinite(options.cross_layer.height_slack) ||
     options.cross_layer.minimum_rise<=0. ||
     options.cross_layer.height_slack<0.)
    throw std::invalid_argument("invalid cross-layer task policy");
  if(!std::isfinite(options.pct_local_above_) || !std::isfinite(options.pct_local_below_) ||
     !std::isfinite(options.pct_direction_weight_) || options.pct_local_above_<=0 || options.pct_local_below_<=0 || options.pct_direction_weight_<0)
    throw std::invalid_argument("invalid PCT local window or direction weight");
  if(!std::isfinite(options.pct_round_budget_) || options.pct_round_budget_<=0.)
    throw std::invalid_argument("pct_candidate_round_budget must be positive");
  options.low_yield_retry_cooldown_ = node.declare_parameter<double>("low_yield_retry_cooldown", 8.0);
  options.no_frontier_hold_ = node.declare_parameter<double>("no_frontier_hold", 8.0);
  int recovery_min_new_cells = node.declare_parameter<int>("recovery_min_new_cells", 8);
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (recovery_min_new_cells < 1)
    // 发现非法输入，抛出错误并停止当前流程。
    throw std::invalid_argument("recovery_min_new_cells must be positive");
  if (!std::isfinite(options.low_yield_retry_cooldown_) || options.low_yield_retry_cooldown_ < 0.0)
    throw std::invalid_argument("low_yield_retry_cooldown must be nonnegative");
  options.min_new_cells_ = static_cast<std::size_t>(recovery_min_new_cells);
  options.occupancy_timeout_ = node.declare_parameter<double>("planner_occupancy_timeout", 2.0);
  options.occupancy_height_tolerance_ = node.declare_parameter<double>("planner_occupancy_height_tolerance", 0.5);
  options.require_planner_occupancy_ = node.declare_parameter<bool>("require_planner_occupancy", false);
  options.observation_only_ = node.declare_parameter<bool>("observation_only", true);
  options.bounds_relative_to_start_ = node.declare_parameter<bool>("bounds_relative_to_start", false);
  options.layer_body_height_ = node.declare_parameter<double>("robot_reference_height", 0.32);
  options.layer_timeout_ = node.declare_parameter<double>("layer_surface_timeout", 2.5);
  options.observation_wait_=node.declare_parameter<double>("observation_frame_wait",2.0);
  options.observation_flat_wait_=node.declare_parameter<double>("observation_flat_frame_wait",1.0);
  options.observation_timeout_=node.declare_parameter<double>("observation_task_timeout",60.0);
  options.observation_pose_wait_timeout_=node.declare_parameter<double>("observation_pose_wait_timeout",5.0);
  options.observation_sweep_=node.declare_parameter<double>("observation_sweep_half_angle",.35);
  options.observation_full_sweep_=node.declare_parameter<bool>("observation_full_sweep_fallback",false);
  options.pct_query_budget_=node.declare_parameter<double>("pct_query_budget_seconds",.20);
  if(!std::isfinite(options.pct_query_budget_) || options.pct_query_budget_<=0.)
    throw std::invalid_argument("pct_query_budget_seconds must be positive");
  options.pct_query_resume_delay_=node.declare_parameter<double>("pct_query_resume_delay_seconds",.10);
  if(!std::isfinite(options.pct_query_resume_delay_) || options.pct_query_resume_delay_<.05 ||
     options.pct_query_resume_delay_>1.)
    throw std::invalid_argument("pct_query_resume_delay_seconds must be in [0.05, 1.0]");
  options.pct_floor_view_limits_={options.observation_goal_xy_tolerance_,
      node.declare_parameter<double>("pct_floor_view_budget_seconds",2.),
      node.declare_parameter<double>("pct_floor_review_view_budget_seconds",4.),
      node.declare_parameter<double>("pct_floor_review_query_budget_seconds",.40)};
  for(const double budget:{options.pct_floor_view_limits_.first_budget,options.pct_floor_view_limits_.review_budget,
                           options.pct_floor_view_limits_.review_slice})
    if(!std::isfinite(budget) || budget<=0.)throw std::invalid_argument("invalid PCT floor viewpoint budget");
  options.pct_floor_review_wait_budget_=node.declare_parameter<double>("pct_floor_review_wait_budget_seconds",0.);
  if(!std::isfinite(options.pct_floor_review_wait_budget_) || options.pct_floor_review_wait_budget_<0.)
    throw std::invalid_argument("invalid PCT floor review wait budget");
  if(!std::isfinite(options.observation_wait_) || !std::isfinite(options.observation_flat_wait_) ||
     options.observation_flat_wait_<=0. || options.observation_timeout_<=options.observation_flat_wait_ ||
     !std::isfinite(options.observation_timeout_) ||
     !std::isfinite(options.observation_pose_wait_timeout_) || options.observation_pose_wait_timeout_<=0. ||
     !std::isfinite(options.observation_sweep_) || options.observation_wait_<=0 || options.observation_timeout_<=options.observation_wait_ ||
     options.observation_sweep_<0 || options.observation_sweep_>M_PI)
    throw std::invalid_argument("invalid observation task settings");
  options.planner_mode_ = node.declare_parameter<std::string>("planner_mode", "both");
  if (options.planner_mode_!="both" && options.planner_mode_!="occupancy" && options.planner_mode_!="layer")
    throw std::invalid_argument("planner_mode must be both, occupancy or layer");
  options.frontier_source_ = node.declare_parameter<std::string>("frontier_source", "occupancy");
  if (options.frontier_source_!="occupancy" && options.frontier_source_!="pct")
    throw std::invalid_argument("frontier_source must be occupancy or pct");
  if (options.frontier_source_=="pct" &&
      (options.planner_mode_=="occupancy" || !shared || !shared->pct))
    throw std::invalid_argument("frontier_source=pct requires in-process project PCT and planner_mode=layer or both");
  options.recovery_goal_xy_tolerance_=node.declare_parameter<double>("recovery_goal_xy_tolerance",0.15);
  if(!std::isfinite(options.recovery_goal_xy_tolerance_) || options.recovery_goal_xy_tolerance_<scan_planner::msg::GoalRequest::MIN_XY_TOLERANCE || options.recovery_goal_xy_tolerance_>scan_planner::msg::GoalRequest::MAX_XY_TOLERANCE)
    throw std::invalid_argument("recovery_goal_xy_tolerance must be in [0.05, 1.0]");
  if(options.layer_body_height_<=0. || options.layer_timeout_<=0.)throw std::invalid_argument("invalid layer geometry or timeout");

  // 参数合法性检查（分辨率、搜索半径、抽稀步长、点数、体素上限必须 > 0）
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (options.map.resolution <= 0.0 || options.map.local_search_radius <= 0.0 || options.point_stride_ < 1 ||
      options.max_points_ < 1 || options.map.max_voxels < 1) {
    // 发现非法输入，抛出错误并停止当前流程。
    throw std::invalid_argument("resolution, search radius, stride, max points and max voxels must be positive");
  }
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (options.map.global_search && (options.map.exploration_min_x >= options.map.exploration_max_x ||
      options.map.exploration_min_y >= options.map.exploration_max_y))
    // 发现非法输入，抛出错误并停止当前流程。
    throw std::invalid_argument("global exploration bounds must have positive area");

  return options;
}

class Explorer final : public rclcpp::Node {
 public:
  explicit Explorer(std::shared_ptr<le::SharedLayerMap> shared)
      : Node("legbot_explorer") {
    auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/slam/global_cloud_registered"); // 点云主题名
    auto sensor_topic = declare_parameter<std::string>("sensor_odom_topic", "/slam/odometry_lidar"); // LiDAR 里程计主题
    auto base_topic = declare_parameter<std::string>("base_odom_topic", "/slam/global_odometry_base"); // 底盘里程计主题
    auto waypoint_topic = declare_parameter<std::string>("waypoint_topic", "/scan/waypoint_request"); // 手动 waypoint 主题
    auto goal_topic = declare_parameter<std::string>("scan_goal_topic", "/scan/goal"); // 上层规划目标主题
    auto ready_topic = declare_parameter<std::string>("ready_topic", "/go2/demo_ready"); // 探测准备状态主题
    auto occupancy_topic = declare_parameter<std::string>(
        "planner_occupancy_topic", "/local_planner/grid_map/occupancy_inflate");
    auto raw_occupancy_topic = declare_parameter<std::string>(
        "planner_raw_occupancy_topic", "/local_planner/grid_map/occupancy");
    auto options = declare_exploration_settings(*this, shared);
    le::ExplorationOutput output;
    auto state_qos = rclcpp::QoS(1).reliable().transient_local(); // QoS 设置，确保状态可靠传递
    waypoint_pub_ = create_publisher<scan_planner::msg::GoalRequest>(waypoint_topic, 10); // 原子发布路点及可选容差（GoalRequest）
    output.marker = publish_with<visualization_msgs::msg::MarkerArray>(*this, "/exploration/frontiers", state_qos);
    output.pct_frontier = publish_with<visualization_msgs::msg::MarkerArray>(*this,
        "/exploration/pct/frontiers", state_qos);
    output.status = publish_with<std_msgs::msg::String>(*this, "/exploration/status", state_qos);
    output.task_active = publish_with<std_msgs::msg::Bool>(*this, "/exploration/task_active", state_qos);
    output.goal_diagnostic = publish_with<std_msgs::msg::String>(*this,
        "/exploration/diagnostics/goal_event", state_qos);
    output.diagnostic_reference = publish_with<nav_msgs::msg::Path>(*this,
        "/exploration/diagnostics/last_reference_path", state_qos);
    output.diagnostic_actual = publish_with<nav_msgs::msg::Path>(*this,
        "/exploration/diagnostics/last_actual_path", state_qos);
    output.diagnostic_bspline = publish_with<scan_planner::msg::Bspline>(*this,
        "/exploration/diagnostics/last_scan_bspline", state_qos);
    output.planning_goal = publish_with<std_msgs::msg::String>(*this,
        "/exploration/planning_goal", state_qos);
    output.planning_result = publish_with<std_msgs::msg::String>(*this,
        "/exploration/planning_results", state_qos);
    output.execution_result = publish_with<std_msgs::msg::String>(*this,
        "/exploration/execution_feedback", state_qos);
    output.coverage = publish_with<nav_msgs::msg::OccupancyGrid>(*this, "/exploration/coverage_grid", state_qos);
    output.trajectory = publish_with<nav_msgs::msg::Path>(*this, "/exploration/trajectory", state_qos);
    output.planned_path = publish_with<nav_msgs::msg::Path>(*this,
        "/exploration/planned_path", state_qos);
    output.selected_occupancy_path = publish_with<nav_msgs::msg::Path>(*this,
        "/exploration/selected_routes/occupancy", state_qos);
    output.selected_layer_path = publish_with<nav_msgs::msg::Path>(*this,
        "/exploration/selected_routes/layer", state_qos);
    output.selected_recovery_path = publish_with<nav_msgs::msg::Path>(*this,
        "/exploration/selected_routes/recovery", state_qos);
    output.occupancy_plan = publish_with<nav_msgs::msg::Path>(*this,
        "/exploration/plans/occupancy", state_qos);
    output.layer_plan = publish_with<nav_msgs::msg::Path>(*this,
        "/exploration/plans/layer", state_qos);
    output.recovery_layer_plan = publish_with<nav_msgs::msg::Path>(*this,
        "/exploration/plans/recovery_layer", state_qos);
    output.layer_diagnostic = publish_with<std_msgs::msg::String>(*this,
        "/exploration/diagnostics/layer_planning", state_qos);
    output.observation_feedback = publish_with<std_msgs::msg::String>(*this, "/exploration/observation_feedback",rclcpp::QoS(10).transient_local());
    output.recovery_turn = publish_with<std_msgs::msg::String>(*this,
        "/scan/recovery_turn_request", rclcpp::QoS(10));
    recovery_reverse_pub_ = create_publisher<std_msgs::msg::String>(
        "/scan/recovery_reverse_request", 10);
    output.recovery_cancel = publish_with<std_msgs::msg::String>(*this,
        "/scan/recovery_action_cancel", rclcpp::QoS(10));
    output.waypoint = [this](const auto &message) { waypoint_pub_->publish(message); };
    output.recovery_reverse = [this](const auto &message) { recovery_reverse_pub_->publish(message); };
    output.waypoint_subscribers = [this] { return waypoint_pub_->get_subscription_count(); };
    output.recovery_reverse_subscribers = [this] { return recovery_reverse_pub_->get_subscription_count(); };
    const bool pct_mode = options.frontier_source_ == "pct";
    if (options.floor_exploration_ && pct_mode) {
      foot_tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      foot_tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*foot_tf_);
    }
    controller_ = std::make_unique<le::ExplorationController>(
        std::move(options), std::move(output), std::move(shared), get_clock(), get_logger(), foot_tf_.get());


    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { controller_->on_cloud(msg); });
    sensor_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        sensor_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { controller_->on_sensor(*msg); });
    base_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        base_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { controller_->on_base(*msg); });
    if (foot_tf_) {
      foot_force_sub_ = create_subscription<control_msgs::msg::DynamicJointState>(
          "/dynamic_joint_states", rclcpp::SensorDataQoS(),
          [this](control_msgs::msg::DynamicJointState::ConstSharedPtr msg) { controller_->on_foot_forces(*msg); });
    }
    if (!occupancy_topic.empty())
      // 订阅一个 ROS 话题，收到数据时调用对应回调。
      occupancy_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          occupancy_topic, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { controller_->on_occupancy(*msg); });
    if (!raw_occupancy_topic.empty())
      raw_occupancy_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          raw_occupancy_topic, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { controller_->on_raw_occupancy(*msg); });
    scan_bspline_sub_ = create_subscription<scan_planner::msg::Bspline>(
        "/scan/planning/bspline", 10,
        [this](scan_planner::msg::Bspline::ConstSharedPtr msg) { controller_->on_scan_reference(*msg); });
    ready_sub_ = create_subscription<std_msgs::msg::Bool>(
        ready_topic, state_qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) { controller_->on_ready(msg->data); });
    input_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/scan/waypoint_input", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { controller_->on_input(*msg); });
    request_input_sub_=create_subscription<scan_planner::msg::GoalRequest>(
        waypoint_topic,10,[this](scan_planner::msg::GoalRequest::ConstSharedPtr m,
                                 const rclcpp::MessageInfo &info) {
          if(*waypoint_pub_==info.get_rmw_message_info().publisher_gid)return;
          controller_->on_input(m->target);
        });
    request_goal_sub_=create_subscription<scan_planner::msg::GoalRequest>(
        "/scan/goal_request",10,[this](scan_planner::msg::GoalRequest::ConstSharedPtr m){ controller_->on_forwarded_request(*m); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { controller_->on_goal(*msg); });
    ack_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/scan/goal_stop_ack", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { controller_->on_ack(*msg); });
    complete_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/scan/mission_complete", state_qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) { controller_->on_complete(*msg); });
    scan_failure_sub_ = create_subscription<std_msgs::msg::String>(
        "/scan/goal_plan_failed", 10,
        [this](std_msgs::msg::String::ConstSharedPtr msg) { controller_->on_scan_failure(*msg); });
    recovery_reverse_result_sub_ = create_subscription<std_msgs::msg::String>(
        "/scan/recovery_reverse_result", 10,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          controller_->on_recovery_action_result(*msg, false);
        });

    recovery_turn_result_sub_ = create_subscription<std_msgs::msg::String>(
        "/scan/recovery_turn_result", 10,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          controller_->on_recovery_action_result(*msg, true);
        });

    pause_service_ = create_service<std_srvs::srv::SetBool>(
        "/exploration/set_paused",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          controller_->set_paused(request->data);
          response->success = true;
          response->message = request->data ? "exploration paused" : "exploration resumed";
        });

    timer_ = create_wall_timer(1s, [this] { controller_->tick(); });
    if (pct_mode) {
      observation_timer_ = create_wall_timer(100ms, [this] { controller_->tick_observation(); });
      selection_timer_ = create_wall_timer(50ms, [this] { controller_->tick_pending_selection(); });
    }
    controller_->start();
  }

 private:
  rclcpp::Publisher<scan_planner::msg::GoalRequest>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_reverse_pub_;
  std::unique_ptr<tf2_ros::Buffer> foot_tf_;
  std::unique_ptr<tf2_ros::TransformListener> foot_tf_listener_;
  std::unique_ptr<le::ExplorationController> controller_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr foot_force_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_; // 已配准点云订阅者
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_sub_; // SCAN 障碍图订阅者
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_occupancy_sub_;
  rclcpp::Subscription<scan_planner::msg::Bspline>::SharedPtr scan_bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sensor_sub_, base_sub_; // 雷达与机体位姿订阅者
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_, complete_sub_; // 就绪与完成信号订阅者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scan_failure_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_reverse_result_sub_;
  rclcpp::Subscription<scan_planner::msg::GoalRequest>::SharedPtr request_input_sub_,request_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input_sub_, goal_sub_, ack_sub_; // 输入、转发、停车回执订阅者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_turn_result_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_; // 手动暂停/恢复服务
  rclcpp::TimerBase::SharedPtr timer_; // 每秒执行 tick 的 ROS 计时器
  rclcpp::TimerBase::SharedPtr observation_timer_;
  rclcpp::TimerBase::SharedPtr selection_timer_;
};

}  // namespace

std::shared_ptr<rclcpp::Node> make_explorer_node(
    std::shared_ptr<le::SharedLayerMap> shared) {
  return std::make_shared<Explorer>(std::move(shared));
}
