#pragma once
#include <tuple>

#include "exploration_controller_modules.hpp"

#include <scan_planner/msg/goal_request.hpp>
#include "legbot_exploration/single_floor_exploration.hpp" // 包含项目核心头文件，提供地图和设置等功能
#include "legbot_exploration/same_floor_route_planner.hpp"
#include "legbot_exploration/route_orchestrator.hpp"
#include "legbot_exploration/recovery_planner.hpp"
#include "legbot_exploration/cross_layer_task.hpp"

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // ROS2 消息：姿态（位置+方向）
#include <nav_msgs/msg/odometry.hpp> // ROS2 消息：里程计信息
#include <nav_msgs/msg/occupancy_grid.hpp> // 全局逐格验收地图
#include <nav_msgs/msg/path.hpp> // 实际走过的机体轨迹
#include <rclcpp/rclcpp.hpp> // ROS2 C++ 客户端库
#include <scan_planner/msg/bspline.hpp> // SCAN 实际下发的 B 样条轨迹
#include <sensor_msgs/msg/point_cloud2.hpp> // ROS2 点云消息类型
#include <sensor_msgs/point_cloud2_iterator.hpp> // 点云迭代器工具
#include <std_msgs/msg/bool.hpp> // 布尔型标准消息
#include <std_msgs/msg/string.hpp> // 字符串标准消息
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.hpp>
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
#include <set>
#include <sstream> // 字符流，用于构造 JSON
#include <cstring> // 字符串操作
#include <utility> // std::pair 等
#include <functional>
#include <vector> // 动态数组容器

namespace legbot_exploration {

struct ExplorationSettings {
  Settings map;
  CrossLayerTask cross_layer;
  std::string frame_id_ ;
  int point_stride_ = 8, max_points_ = 1000;
  double sync_tolerance_ = 0.03, odom_timeout_ = 1.0, cloud_timeout_ = 2.0;
  double goal_reach_xy_ = 0.55, cooldown_ = 45.0;
  double pct_round_budget_=1.0;
  double pct_local_above_=3.5, pct_local_below_=1.5, pct_direction_weight_=.5;
  bool cross_layer_enabled_=true;
  bool floor_exploration_=false;
  bool skip_start_floor_exploration_=false;
  bool mission_return_home_=false;
  double mission_target_height_=4.43;
  Point mission_home_{-6.75,9.60,0.};
  double floor_height_tolerance_=.08, foot_force_min_=12.;
  double observation_min_distance_=0.50, observation_goal_xy_tolerance_=0.15;
  double low_yield_retry_cooldown_ = 8.0;
  std::size_t min_new_cells_ = 8;
  double no_frontier_hold_ = 8.0;
  double occupancy_timeout_ = 2.0, occupancy_height_tolerance_ = 0.5;
  bool require_planner_occupancy_ = false;
  bool observation_only_ = true;
  bool bounds_relative_to_start_ = false;
  double layer_body_height_ = 0.32;
  double layer_timeout_ = 2.5;
  std::string planner_mode_="both";
  std::string frontier_source_="occupancy";
  double observation_wait_=2., observation_flat_wait_=1., observation_timeout_=60., observation_sweep_=.35;
  double observation_pose_wait_timeout_=5.;
  bool observation_full_sweep_=false;
  double pct_query_budget_=.20;
  double pct_query_resume_delay_=.10;
  FloorViewLimits pct_floor_view_limits_{.15,2.,4.,.40};
  double pct_floor_review_wait_budget_=0.;
  double recovery_goal_xy_tolerance_=0.15;
};

struct ExplorationOutput {
  std::function<void(const scan_planner::msg::GoalRequest &)> waypoint;
  std::function<void(const visualization_msgs::msg::MarkerArray &)> marker;
  std::function<void(const visualization_msgs::msg::MarkerArray &)> pct_frontier;
  std::function<void(const std_msgs::msg::String &)> status;
  std::function<void(const std_msgs::msg::Bool &)> task_active;
  std::function<void(const std_msgs::msg::String &)> goal_diagnostic;
  std::function<void(const nav_msgs::msg::Path &)> diagnostic_reference;
  std::function<void(const nav_msgs::msg::Path &)> diagnostic_actual;
  std::function<void(const scan_planner::msg::Bspline &)> diagnostic_bspline;
  std::function<void(const std_msgs::msg::String &)> planning_goal;
  std::function<void(const std_msgs::msg::String &)> planning_result;
  std::function<void(const std_msgs::msg::String &)> execution_result;
  std::function<void(const nav_msgs::msg::OccupancyGrid &)> coverage;
  std::function<void(const nav_msgs::msg::Path &)> trajectory;
  std::function<void(const nav_msgs::msg::Path &)> planned_path;
  std::function<void(const nav_msgs::msg::Path &)> selected_occupancy_path;
  std::function<void(const nav_msgs::msg::Path &)> selected_layer_path;
  std::function<void(const nav_msgs::msg::Path &)> selected_recovery_path;
  std::function<void(const nav_msgs::msg::Path &)> occupancy_plan;
  std::function<void(const nav_msgs::msg::Path &)> layer_plan;
  std::function<void(const nav_msgs::msg::Path &)> recovery_layer_plan;
  std::function<void(const std_msgs::msg::String &)> layer_diagnostic;
  std::function<void(const std_msgs::msg::String &)> observation_feedback;
  std::function<void(const std_msgs::msg::String &)> recovery_turn;
  std::function<void(const std_msgs::msg::String &)> recovery_reverse;
  std::function<void(const std_msgs::msg::String &)> recovery_cancel;
  std::function<std::size_t()> waypoint_subscribers, recovery_reverse_subscribers;
};

// The ROS adapter delivers inputs on one executor. Only this object can
// dispatch an action, including commands proposed by the recovery module.
class ExplorationController {
 public:
  ExplorationController(ExplorationSettings options, ExplorationOutput output,
                        std::shared_ptr<SharedLayerMap> shared, rclcpp::Clock::SharedPtr clock,
                        rclcpp::Logger logger, tf2_ros::Buffer *foot_tf);
  void start();
  void set_paused(bool paused);
  void on_ready(bool ready);
  void on_scan_reference(const scan_planner::msg::Bspline &msg);
  void on_forwarded_request(const scan_planner::msg::GoalRequest &request);
  void on_raw_occupancy(const sensor_msgs::msg::PointCloud2 &msg);
  void on_foot_forces(const control_msgs::msg::DynamicJointState &msg);
  void tick_observation();
  void tick_pending_selection();
  void on_sensor(const nav_msgs::msg::Odometry &msg);
  void on_base(const nav_msgs::msg::Odometry &msg);
  void on_occupancy(const sensor_msgs::msg::PointCloud2 &msg);
  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void on_input(const geometry_msgs::msg::PoseStamped &msg);
  void on_goal(const geometry_msgs::msg::PoseStamped &msg);
  void on_ack(const geometry_msgs::msg::PoseStamped &msg);
  void on_complete(const std_msgs::msg::Bool &msg);
  void on_scan_failure(const std_msgs::msg::String &msg);
  void on_recovery_action_result(const std_msgs::msg::String &msg, bool turn);
  void tick();

 private:
  struct Suppressed { XY xy; double expiry; };

  struct Active {
    XY xy;                    // 目标 XY 坐标
    int64_t input_stamp = 0;     // 手动输入目标的时间戳（纳秒）
    std::optional<int64_t> stamp; // 外部 Goal 消息的时间戳（纳秒）
    std::optional<Point> stop_position;
    bool ack = false;             // 是否收到前进确认（Ack）
    bool armed = false;           // 是否已收到前进前的 “armed” 标记（mission_complete false）
    bool complete = false;        // 是否已收到完成信号（mission_complete true）
    bool identified_request = false; // GoalRequest IDs match this exact route stage.
    bool recovery = false;
    bool reverse = false;
    bool turn = false;
    std::size_t observed_cells_at_send = 0;
    Point pose_at_send{0.0, 0.0, 0.0};
    std::size_t trajectory_start_index = 0;
    std::size_t map_version_at_send = 0, layer_version_at_send = 0;
    std::uint64_t goal_id = 0, attempt_id = 0;
    std::size_t stage_index = 0;
    std::size_t reference_point_count = 0;
    std::optional<double> raw_distance_at_send, inflated_distance_at_send;
    std::string source_at_send;
    std::vector<XY> reference_route;
  };
  struct ObservationTask {
    PctFrontier frontier;
    double began=0.,waiting_since=0.,center=0.,coverage=0.;
    bool initialized=false,turning=false,arm_pending=false;
    std::optional<std::tuple<bool,std::string,bool>> pending_end;
    bool cross_layer=false,platform_frontier=false;
    bool support_scan=false;
    bool targeted_blocker_scan=false;
    bool fast_floor=false;
    int floor_attempt=0;
    std::optional<double> stair_heading;
    std::optional<Point> planned_pose;
    std::size_t next=0;
    std::size_t views_completed=0;
    std::vector<double> headings;
    std::optional<PathProcessor> continuation;
    std::optional<std::pair<std::uint64_t,std::uint64_t>> action_owner;
    std::optional<double> pose_wait_since;
    double pose_wait_seconds=0.,phase_since=0.;
    std::string phase;
    std::map<std::string,double> phase_seconds;
    bool directional() const {return cross_layer || stair_heading.has_value() || targeted_blocker_scan;}
  };
  enum class FrontierSelection { dispatched, pending, recovery, exhausted };

  using StairReposition = RecoveryController::StairReposition;
  using BlockerRecovery = RecoveryController::BlockerRecovery;

  bool try_dispatch_route();
  void observation_phase(const char *phase,double current);
  void publish_target(const XY &xy, const std::string &detail = "");
  void publish_planned_path(const PlannedRoute &route,const std::vector<Point> *reference_xyz=nullptr);
  void publish_candidate_path(const PlannerResult &result);
  void reset_planned_route();
  bool skip_nearby_route_waypoints();
  bool layer_map_fresh() const;
  PlanningSnapshot planning_snapshot(
      const std::unordered_map<Cell,int,CellHash> &grid,
      const std::vector<XY> &obstacles) const;
  struct PctObservationProgress;
  struct RouteSelection {
    const PctFrontier *frontier;
    const std::string &detail;
    PctObservationProgress *progress;
    double started;
    bool recovery;
    bool stair_segment;
    bool stair_climb;
    bool fast_floor;
    int floor_attempt;
    std::optional<XY> platform_floor;
    std::optional<double> departure_height;
  };
  bool accept_route(const PlannerResult &chosen, const RouteSelection &selection,
                    const Point &ascent_start, const std::vector<Point> &ground_route);
  bool prepare_route(const XY &target,
      const std::unordered_map<Cell, int, CellHash> &grid,
      const std::vector<XY> &obstacles,
      const std::string &detail = "", double selection_score=0.0,
      const PctFrontier *pct_candidate=nullptr,
      std::optional<Point> recovery_xyz=std::nullopt,
      RecoveryRequest request=RecoveryRequest::position,
      bool stair_short_action=false,
      std::optional<XY> platform_floor=std::nullopt,
      std::string *selection_failure=nullptr,
      std::vector<PctUnknownBlocker> *unknown_blockers=nullptr,
      PctFrontier *blocked_goal=nullptr);
  void advance_route();

  void cancel_recovery_action();
  std::optional<std::uint64_t> pct_recovery_evidence();
  void defer_cross_layer(const std::string &reason);
  void block_stair_reposition(const std::string &reason);
  void await_stair_stop();
  bool try_queue_stair_reposition(const std::string &reason);
  void handle_stair_reposition(double current);
  bool begin_blocker_recovery(const PctFrontier &goal,
      const std::vector<PctUnknownBlocker> &blockers,
      bool cross_layer,int floor_attempt=0,const char *policy="pct_frontier",
      std::optional<std::string> start_failure=std::nullopt,
      std::optional<XY> platform=std::nullopt,bool enqueue=false);
  bool resume_pending_blocker_recovery();
  void finish_blocker_recovery(const char *reason);
  void run_blocker_recovery(double current);
  void recover_blocked_ascent(const PctFrontier &forward={});

  bool refresh_exploration_height();
  std::optional<XY> current_pct_floor() const;
  bool settle_unavailable_floor_cluster(std::uint64_t id,const char *reason,
      std::optional<Point> viewpoint=std::nullopt,
      double planning_ms=0.);
  double cross_layer_minimum_travel() const;
  const char *floor_stair_motion_wait(double current) const;
  bool try_upward_support(PctFrontier *blocked=nullptr,
                          std::optional<double> minimum_height=std::nullopt);
  bool skipping_start_floor() const;
  void dispatch_ascent_targets(bool can_send,
      const char *waiting_state="pct_waiting_for_ascent");
  enum class FrontierMode { floor, cross_layer, floor_ascent };
  FrontierSelection select_exploration_target(std::optional<XY> platform, FrontierMode mode,
      std::optional<double> minimum_frontier_height=std::nullopt);
  void run_pct_cycle(double current);
  bool run_home_mission(double current);

  void report(const std::string &state, const std::string &detail = "");
  void publish_coverage_grid(const std::unordered_map<Cell, int, CellHash> &grid);
  void sample_support_height(const nav_msgs::msg::Odometry &msg,double q_norm);
  bool read_occupancy_points(const sensor_msgs::msg::PointCloud2 &msg,
      std::vector<XY> &output, std::vector<Point> *nearby=nullptr);
  bool scan_start_inflated(const Point &base,double heading) const;
  bool planner_occupancy_fresh(double current) const;
  bool planner_blocks(const XY &xy) const;
  void process_clouds();
  bool integrate_cloud(const sensor_msgs::msg::PointCloud2 &msg);
  void finish_if_complete();
  void publish_execution_result(const ExecutionResult &result);
  void bind_scan_goal(std::int64_t stamp);
  std::optional<double> nearest_obstacle_distance(
      const XY &point, const std::vector<XY> &obstacles,
      double last_update) const;
  void publish_goal_diagnostic(const Active &goal, const char *event,
      const std::string &reason);
  void handle_scan_failure(std::int64_t stamp, const std::string &reason);
  Point cross_ground_pose() const;
  bool stable_four_foot_support(double current) const;
  double refresh_stair_heading(double fallback);
  std::optional<std::uint64_t> blocker_recovery_evidence(const BlockerRecovery &task);
  void finish_active_goal();
  void publish_marker(const std::optional<XY> &goal);
  void publish_pct_frontiers();
  void arm_observation_view();
  void end_observation(bool success,const std::string &reason,bool floor_attempt_settled=false);
  bool dispatch_observation_turn(double angle);
  PctObservation stair_observation_progress();
  bool handle_observation(double current);
  void tick_pct(double current);

  const ExplorationSettings options_;
  const ExplorationOutput output_;
  const rclcpp::Clock::SharedPtr clock_;
  std::optional<double> route_stage_reached_wall_;
  const rclcpp::Logger logger_;
  tf2_ros::Buffer *const foot_tf_;
  ExplorationPolicy policy_;
  RecoveryController recovery_;
  RoutePlanner route_planner_;
  Settings cfg_; // 参数设置对象
  VoxelMap map_; // 体素地图实例
  std::string last_status_; // 坐标系 ID 与最近一次状态字符串
  std::map<std::string,double> selection_timings_;
  std::optional<double> selection_resume_at_;
  double selection_cycle_started_=0.;
  std::size_t selection_resume_count_=0;
  double selection_yielded_at_=0.,selection_resume_gap_ms_=0.;
  std::optional<bool> last_task_active_;
  double pct_round_deadline_=0.;
  std::size_t contacting_feet_=0;
  std::optional<double> stable_foot_height_,foot_stable_since_,four_feet_stable_since_;
  bool foot_support_stable_=false;
  unsigned contacting_foot_mask_=0,foot_reference_mask_=0;
  double foot_height_span_=0.;
  std::array<Point,4> foot_reference_contacts_{};
  std::deque<double> foot_height_samples_;
  double foot_sample_wall_=0.,foot_force_wall_=0.,foot_reference_yaw_=0.;
  bool pct_markers_dirty_=false;
  std::uint64_t pct_marker_version_=0;
  double pct_marker_wall_=0.;
  std::string height_review_status_="{}";
  std::set<std::pair<std::string,int>> pct_marker_keys_;
  int64_t foot_force_stamp_=0;
  std::string foot_sample_state_="no_contact_data";
  XY foot_reference_xy_{};
  const std::array<const char *,4> foot_frames_{"FL_foot","FR_foot","RL_foot","RR_foot"};
  std::array<Point,4> foot_contact_points_{};
  std::array<double,4> foot_forces_{};
  bool pct_round_exhausted_=false, pct_stair_search_pending_=false;
  struct PctObservationProgress {
    std::uint64_t revision=0;
    std::vector<Point> observations;
    std::size_t next=0;
    bool global=false;
  };
  std::map<std::uint64_t,PctObservationProgress> pct_observation_progress_;
  std::map<std::string,PctObservationProgress> stair_route_progress_;
  bool ready_ = false, paused_ = false; // 运行模式标志
  bool bounds_initialized_ = false; // 是否把探索边界平移到起始机体坐标
  double base_wall_ = 0.0, cloud_wall_ = 0.0; // 时间戳记录
  double occupancy_wall_ = 0.0; // 最近收到规划器障碍图的本机时间
  double raw_occupancy_wall_ = 0.0;
  double pct_cloud_wall_=0.0;
  std::optional<ObservationTask> observation_task_;
  bool map_read_retry_=false;
  std::string observation_metadata_="{}";
  std::optional<PctFrontier> selected_pct_;
  std::optional<Point> selected_pct_observation_;
  bool selected_platform_frontier_=false,selected_fast_floor_=false;
  int selected_floor_attempt_=0;
  double yaw_wall_=0.;
  std::optional<double> base_yaw_,base_quiet_since_;
  Point base_attitude_{};
  PathProcessor path_processor_;
  std::optional<XY> route_selection_target_; // 本轮选中的探索目标，而非分段路点。
  std::size_t route_skipped_nearby_ = 0;
  ExplorationMode control_mode_ = ExplorationMode::waiting_initial_data;
  std::uint64_t goal_id_ = 0, planning_attempt_id_ = 0;
  std::size_t layer_version_ = 0;
  std::shared_ptr<SharedLayerMap> shared_layer_;
  bool mission_returning_=false, mission_finished_=false, mission_exploration_finished_=false;
  double mission_route_retry_at_=0.;
  std::optional<Point> base_; // 基座位置（x,y,z）
  std::optional<Active> active_; // 当前活动目标信息
  std::optional<scan_planner::msg::Bspline> active_scan_bspline_;
  std::optional<std::pair<std::int64_t, std::string>> pending_scan_failure_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_scan_ack_;
  std::size_t scan_failure_count_ = 0;
  std::string scan_last_failure_;
  std::optional<double> empty_since_; // 空前沿持续时间记录
  std::optional<double> residual_since_; // 零散前沿持续出现的起始时间；空值表示尚未开始计时
  std::size_t last_frontier_cells_ = 0, last_frontier_clusters_ = 0; // 上轮原始前沿格数与有效簇数
  std::size_t last_reachable_frontier_cells_ = 0, last_candidate_goals_ = 0; // 上轮可达格与候选目标数
  std::size_t last_suppressed_goals_ = 0, last_open_start_neighbors_ = 0; // 上轮被屏蔽目标与起点开放邻格数
  Coverage coverage_; // 全局边界内逐格验收统计
  GlobalFrontiers global_frontiers_; // 完整累计地图的原始前沿验收
  FrontierIndex frontier_index_; // 随二维占据状态增删的当前原始前沿。
  bool frontier_index_initialized_ = false;
  std::size_t frontier_index_repairs_ = 0;
  std::size_t unresolved_unknown_cells_ = 0; // 当前没有可用目标时尚未覆盖的格数
  nav_msgs::msg::Path trajectory_; // 积累的机体行走轨迹，发布到 RViz
  std::deque<std::pair<int64_t, Point>> sensor_poses_; // 里程计姿态缓存
  std::deque<std::pair<int64_t, Point>> base_poses_; // 带时间戳的近期机体位姿缓存
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_clouds_; // 点云缓存
  std::vector<XY> external_obstacles_; // SCAN 膨胀障碍点的水平坐标
  std::vector<Point> nearby_scan_inflation_;
  std::vector<XY> raw_obstacles_;
  std::vector<Suppressed> suppressed_; // 被抑制的前沿列表（冷却）
  GoalProgressGate goal_progress_gate_; // 屏蔽到达后无地图收益的目标

};

}  // namespace legbot_exploration
