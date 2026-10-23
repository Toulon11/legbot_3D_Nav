#include "legbot_exploration/exploration_core.hpp" // 包含项目核心头文件，提供地图和设置等功能

#include <geometry_msgs/msg/pose_stamped.hpp> // ROS2 消息：姿态（位置+方向）
#include <nav_msgs/msg/odometry.hpp> // ROS2 消息：里程计信息
#include <nav_msgs/msg/path.hpp> // 实际走过的机体轨迹
#include <rclcpp/rclcpp.hpp> // ROS2 C++ 客户端库
#include <sensor_msgs/msg/point_cloud2.hpp> // ROS2 点云消息类型
#include <sensor_msgs/point_cloud2_iterator.hpp> // 点云迭代器工具
#include <std_msgs/msg/bool.hpp> // 布尔型标准消息
#include <std_msgs/msg/string.hpp> // 字符串标准消息
#include <std_srvs/srv/set_bool.hpp> // 设置布尔服务（用于暂停）
#include <std_srvs/srv/trigger.hpp> // 触发服务（用于清除前沿）
#include <visualization_msgs/msg/marker_array.hpp> // RViz 可视化 Marker

#include <algorithm> // 常用算法库
#include <chrono> // 时间工具
#include <cmath> // 数学函数（hypot）
#include <cstdint> // 固定宽度整数类型
#include <deque> // 双端队列容器
#include <limits> // 数值极限
#include <memory> // 智能指针
#include <optional> // 可选值类型
#include <sstream> // 字符流，用于构造 JSON
#include <cstring> // 字符串操作
#include <utility> // std::pair 等
#include <vector> // 动态数组容器

namespace le = legbot_exploration; // 项目命名空间别名，简化代码
using namespace std::chrono_literals; // 允许使用 1s、500ms 等时间字面量

// 匿名命名空间：仅在本文件内部使用的工具函数，防止外部链接冲突
namespace {
  // 计算二维欧氏距离，返回 double 精度
  double xy_distance(const le::XY &a, const le::XY &b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
  }
  // 获取单调时钟的当前时间（秒），用于所有超时检测
  double wall_now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  // ROS Time（sec + nanosec）转纳秒整数，便于比较大小
  int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }
} // namespace

// ===========================================================================
// Explorer 节点：完成地图维护、前沿搜索、目标发布、状态上报等功能
// ===========================================================================
class Explorer : public rclcpp::Node {
 public:
   Explorer()
       : Node("legbot_explorer"),               // ROS2 节点名
         map_(declare_settings()) {               // 读取参数并创建体素地图

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
     point_stride_ = declare_parameter<int>("point_stride", 8);                       // 点云抽稀步长
     max_points_ = declare_parameter<int>("max_points_per_cloud", 1000);           // 单帧最大采点数
     sync_tolerance_ = declare_parameter<double>("sensor_sync_tolerance", 0.03);   // 同步容差（秒）
     odom_timeout_ = declare_parameter<double>("odom_timeout", 1.0);
     cloud_timeout_ = declare_parameter<double>("cloud_timeout", 2.0);
     goal_reach_xy_ = declare_parameter<double>("goal_reach_xy", 0.55);
     progress_distance_ = declare_parameter<double>("progress_distance", 0.35);
     progress_timeout_ = declare_parameter<double>("progress_timeout", 12.0);
     goal_timeout_ = declare_parameter<double>("goal_timeout", 90.0);
     forward_timeout_ = declare_parameter<double>("goal_forward_timeout", 3.0);
     cooldown_ = declare_parameter<double>("failed_frontier_cooldown", 45.0);
     no_frontier_hold_ = declare_parameter<double>("no_frontier_hold", 8.0);
     int recovery_min_new_cells = declare_parameter<int>("recovery_min_new_cells", 8);
     if (recovery_min_new_cells < 1)
       throw std::invalid_argument("recovery_min_new_cells must be positive");
     recovery_gate_ = le::RecoveryGate(static_cast<std::size_t>(recovery_min_new_cells));
     goal_progress_gate_ = le::GoalProgressGate(static_cast<std::size_t>(recovery_min_new_cells));
     occupancy_timeout_ = declare_parameter<double>("planner_occupancy_timeout", 2.0);
     occupancy_height_tolerance_ = declare_parameter<double>("planner_occupancy_height_tolerance", 0.5);
     require_planner_occupancy_ = declare_parameter<bool>("require_planner_occupancy", false);
     near_goal_dwell_ = declare_parameter<double>("near_goal_dwell", 1.5);
     blocked_goal_dwell_ = declare_parameter<double>("blocked_goal_dwell", 2.0);
     observation_only_ = declare_parameter<bool>("observation_only", true);
     bounds_relative_to_start_ = declare_parameter<bool>("bounds_relative_to_start", false);

     // 参数合法性检查（分辨率、搜索半径、抽稀步长、点数、体素上限必须 > 0）
     if (cfg_.resolution <= 0.0 || cfg_.local_search_radius <= 0.0 || point_stride_ < 1 ||
         max_points_ < 1 || cfg_.max_voxels < 1) {
       throw std::invalid_argument("resolution, search radius, stride, max points and max voxels must be positive");
     }
     if (cfg_.global_search && (cfg_.exploration_min_x >= cfg_.exploration_max_x ||
         cfg_.exploration_min_y >= cfg_.exploration_max_y))
       throw std::invalid_argument("global exploration bounds must have positive area");

     // ---------- ROS 接口 ----------
     auto state_qos = rclcpp::QoS(1).reliable().transient_local(); // QoS 设置，确保状态可靠传递
     waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(waypoint_topic, 10); // 发布前沿探索目标（PoseStamped）
     marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/exploration/frontiers", state_qos);
     status_pub_ = create_publisher<std_msgs::msg::String>("/exploration/status", state_qos);
     trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("/exploration/trajectory", state_qos);
     trajectory_.header.frame_id = frame_id_;

     // 订阅点云、里程计、手动目标等主题
     cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
         cloud_topic, rclcpp::SensorDataQoS(),
         [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_cloud(msg); });
     sensor_sub_ = create_subscription<nav_msgs::msg::Odometry>(
         sensor_topic, rclcpp::SensorDataQoS(),
         [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_sensor(*msg); });
     base_sub_ = create_subscription<nav_msgs::msg::Odometry>(
         base_topic, rclcpp::SensorDataQoS(),
         [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_base(*msg); });
     if (!occupancy_topic.empty())
       occupancy_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
           occupancy_topic, rclcpp::SensorDataQoS(),
           [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_occupancy(*msg); });
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

     // 暂停服务 /exploration/set_paused
     pause_service_ = create_service<std_srvs::srv::SetBool>(
         "/exploration/set_paused",
         [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
           // 当收到暂停请求且当前有活动目标且未在清除时，先发起一次清除
           if (request->data && active_ && !clearing_) start_clear("paused");
           paused_ = request->data;
           response->success = true;
           response->message = paused_ ? "exploration paused" : "exploration resumed";
         });

     // 用于清除已发布前沿点的外部 Trigger 服务
     clear_client_ = create_client<std_srvs::srv::Trigger>("/scan/waypoints/clear");

     // 每秒调用一次的主循环计时器
     timer_ = create_wall_timer(1s, [this]() { tick(); });

     // 初始状态上报（观测模式或等待数据）
     report(observation_only_ ? "observation_only" : "waiting_for_data");
   }

 private:
   // ---------------------------------------------------------------
   // 结构体：记录当前活动的目标信息
   // ---------------------------------------------------------------
   struct Active {
     le::XY xy;                    // 目标 XY 坐标
     double sent = 0.0;            // 目标发送时的 wall 时间
     int64_t input_stamp = 0;     // 手动输入目标的时间戳（纳秒）
     std::optional<int64_t> stamp; // 外部 Goal 消息的时间戳（纳秒）
     double best_distance = 0.0;  // 迄今记录的最小距离（用于进度检测）
     double progress_wall = 0.0;   // 最近一次进度更新的 wall 时间
     bool ack = false;             // 是否收到前进确认（Ack）
     bool armed = false;           // 是否已收到前进前的 “armed” 标记（mission_complete false）
     bool complete = false;        // 是否已收到完成信号（mission_complete true）
     std::optional<double> near_since;
     std::optional<double> blocked_since;
     bool recovery = false;
     std::size_t observed_cells_at_send = 0;
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
     if (max_voxels < 1) throw std::invalid_argument("max_voxels must be positive");
     cfg.max_voxels = static_cast<std::size_t>(max_voxels);
     cfg.obstacle_min_relative_z = declare_parameter<double>("obstacle_min_relative_z", cfg.obstacle_min_relative_z);
     cfg.obstacle_max_relative_z = declare_parameter<double>("obstacle_max_relative_z", cfg.obstacle_max_relative_z);
     cfg.free_min_relative_z = declare_parameter<double>("free_min_relative_z", cfg.free_min_relative_z);
     cfg.robot_radius = declare_parameter<double>("robot_radius", cfg.robot_radius);
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
     cfg_ = cfg;
     return cfg;
   }

   // ---------------------------------------------------------------
   // 状态上报（向 /exploration/status 主题发布 JSON）
   // ---------------------------------------------------------------
   void report(const std::string &state, const std::string &detail = "") {
     std::ostringstream stream;
     stream << "{\"state\":\"" << state << "\",\"implementation\":\"cpp\",";
     stream << "\"map_version\":" << map_.version() << ",\"voxel_count\":" << map_.size();
     stream << ",\"paused\":" << (paused_ ? "true" : "false");
     stream << ",\"observation_only\":" << (observation_only_ ? "true" : "false");
     stream << ",\"global_search\":" << (cfg_.global_search ? "true" : "false");
     stream << ",\"frontier_cells\":" << last_frontier_cells_;
     stream << ",\"frontier_clusters\":" << last_frontier_clusters_;
     stream << ",\"reachable_frontier_cells\":" << last_reachable_frontier_cells_;
     stream << ",\"candidate_goals\":" << last_candidate_goals_;
     stream << ",\"suppressed_goals\":" << last_suppressed_goals_;
     stream << ",\"open_start_neighbors\":" << last_open_start_neighbors_;
     if (!detail.empty()) stream << ",\"detail\":\"" << detail << "\"";
     stream << '}';
     if (stream.str() == last_status_) return; // 去重：若未变化则不发送
     std_msgs::msg::String msg;
     msg.data = stream.str();
     status_pub_->publish(msg);
     last_status_ = msg.data;
   }

   // ---------------------------------------------------------------
   // 里程计（LiDAR）回调
   // ---------------------------------------------------------------
   void on_sensor(const nav_msgs::msg::Odometry &msg) {
     if (msg.header.frame_id != frame_id_) return; // 检查坐标系
     const auto &p = msg.pose.pose.position;
     if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return; // 数据有效性检查
     sensor_poses_.push_back({stamp_ns(msg.header.stamp), {p.x, p.y, p.z}});
     if (sensor_poses_.size() > 100) sensor_poses_.pop_front(); // 只保留最近 100 条
     process_clouds();
   }

   // ---------------------------------------------------------------
   // 基座里程计回调（机器人底盘位置）
   // ---------------------------------------------------------------
   void on_base(const nav_msgs::msg::Odometry &msg) {
     if (msg.header.frame_id != frame_id_) return;
     const auto &p = msg.pose.pose.position;
     if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
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
     base_poses_.push_back({stamp_ns(msg.header.stamp), *base_});
     if (base_poses_.size() > 100) base_poses_.pop_front();
     base_wall_ = wall_now();
     if (trajectory_.poses.empty() ||
         xy_distance({p.x, p.y}, {trajectory_.poses.back().pose.position.x,
                                  trajectory_.poses.back().pose.position.y}) >= 0.1) {
       geometry_msgs::msg::PoseStamped pose;
       pose.header = msg.header;
       pose.pose = msg.pose.pose;
       trajectory_.poses.push_back(pose);
       if (trajectory_.poses.size() > 20000) trajectory_.poses.erase(trajectory_.poses.begin());
     }
     process_clouds();
   }

   void on_occupancy(const sensor_msgs::msg::PointCloud2 &msg) {
     if (msg.header.frame_id != frame_id_ || !base_) return;
     bool x_found = false, y_found = false, z_found = false;
     for (const auto &field : msg.fields) {
       if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) x_found = true;
       if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) y_found = true;
       if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) z_found = true;
     }
     if (!x_found || !y_found || !z_found) return;
     external_obstacles_.clear();
     sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
     const std::size_t total = static_cast<std::size_t>(msg.width) * msg.height;
     for (std::size_t i = 0; i < total; ++i, ++x, ++y, ++z) {
       if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z) &&
           std::abs(*z - (*base_)[2]) <= occupancy_height_tolerance_)
         external_obstacles_.push_back({*x, *y});
     }
     occupancy_wall_ = wall_now();
   }

   bool planner_occupancy_fresh(double current) const {
     return occupancy_wall_ > 0.0 && current - occupancy_wall_ <= occupancy_timeout_;
   }

   bool planner_blocks(const le::XY &xy) const {
     return std::any_of(external_obstacles_.begin(), external_obstacles_.end(),
         [this, &xy](const le::XY &obstacle) {
           return xy_distance(xy, obstacle) <= cfg_.robot_radius;
         });
   }

   // ---------------------------------------------------------------
   // 点云回调 将当前帧雷达点云缓存起来，等待与最近的里程计匹配后再融合到体素地图
   // ---------------------------------------------------------------
   void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
     if (msg->header.frame_id != frame_id_) return;
     pending_clouds_.push_back(msg);
     if (pending_clouds_.size() > 5) pending_clouds_.pop_front(); // 最多缓存 5 帧点云
     process_clouds();
   }

   // ---------------------------------------------------------------
   // 处理缓存的点云：与最近的里程计匹配并融合到体素地图
   // ---------------------------------------------------------------
   void process_clouds() {
     // 必须同时拥有基座、里程计且基座未超时才能继续
     if (!base_ || sensor_poses_.empty() || base_poses_.empty() ||
         wall_now() - base_wall_ > odom_timeout_) return;
     for (auto it = pending_clouds_.begin(); it != pending_clouds_.end();) {
       int64_t stamp = stamp_ns((*it)->header.stamp);
       if (integrate_cloud(**it))
         it = pending_clouds_.erase(it);
       else if (sensor_poses_.back().first - stamp > sync_tolerance_ * 1e9 &&
                base_poses_.back().first - stamp > sync_tolerance_ * 1e9)
         it = pending_clouds_.erase(it); // 超出同步容差，直接丢弃
       else
         ++it; // 仍等待更合适的里程计匹配
     }
   }

   // ---------------------------------------------------------------
   // 将单帧点云投影到体素地图中
   // ---------------------------------------------------------------
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
           return std::llabs(a.first - stamp) < std::llabs(b.first - stamp);
         });
     if (std::llabs(nearest_base->first - stamp) > sync_tolerance_ * 1e9) return false;

     // 检查点云是否包含 x/y/z 三个字段（必须为 FLOAT32）
     bool x_found = false, y_found = false, z_found = false;
     for (const auto &field : msg.fields) {
       if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) x_found = true;
       if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) y_found = true;
       if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) z_found = true;
     }
     if (!x_found || !y_found || !z_found) return true; // 若缺失字段直接跳过（算作成功）

     // 抽稀采样点云，最多 max_points_ 个点
     std::vector<le::Point> points;
     points.reserve(max_points_);
     sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
     const std::size_t total = static_cast<std::size_t>(msg.width) * msg.height;
     for (std::size_t i = 0; i < total && static_cast<int>(points.size()) < max_points_; ++i, ++x, ++y, ++z) {
       if (i % static_cast<std::size_t>(point_stride_) == 0)
         points.push_back({*x, *y, *z});
     }
     // 将抽稀后的点云融合进体素地图（使用最近的里程计姿态）
     if (map_.integrate(nearest->second, points, nearest_base->second[2]))
       cloud_wall_ = wall_now();
     return true;
   }

   // ---------------------------------------------------------------
   // 手动输入（Waypoint）回调：用于调试或手动指定目标点
   // ---------------------------------------------------------------
   void on_input(const geometry_msgs::msg::PoseStamped &msg) {
     le::XY xy{msg.pose.position.x, msg.pose.position.y};
     // 若已有活动目标且与本次输入距离 < 0.05 m 且时间戳相同，则认为是重复输入，直接忽略
     if (active_ && xy_distance(xy, active_->xy) < 0.05 && stamp_ns(msg.header.stamp) == active_->input_stamp) return;
     if (!clearing_) {
       active_.reset();
       paused_ = true;
       report("paused_for_manual_goal");
     }
   }

   // ---------------------------------------------------------------
   // 外部规划目标（Goal）回调：来自上层规划模块的 XY 目标点
   // ---------------------------------------------------------------
   void on_goal(const geometry_msgs::msg::PoseStamped &msg) {
     if (clearing_ || msg.header.frame_id != frame_id_) return;
     le::XY xy{msg.pose.position.x, msg.pose.position.y};
     // 若最近一次清除结束后很短时间内且目标与基座距离 < 0.25 m，视为重复目标，直接返回
     if (base_ && wall_now() - last_clear_wall_ < 2.0 && xy_distance(xy, {(*base_)[0], (*base_)[1]}) < 0.25) return;
     if (!active_) {
       paused_ = true;
       report("paused_for_manual_goal");
       return;
     }
     if (xy_distance(xy, active_->xy) < 0.05) {
       // 与当前活动目标几乎相同，只更新时间戳（保证是最新的 Goal）
       if (stamp_ns(msg.header.stamp) < active_->input_stamp) return; // 老旧消息直接忽略
       active_->stamp = stamp_ns(msg.header.stamp);
     } else {
       // 完全不同的目标：先清除当前目标并暂停
       active_.reset();
       paused_ = true;
       report("paused_for_manual_goal");
     }
   }

   // ---------------------------------------------------------------
   // 前往确认回执（Ack）回调：确认机器人已到达目标点
   // ---------------------------------------------------------------
   void on_ack(const geometry_msgs::msg::PoseStamped &msg) {
     if (!active_ || !active_->stamp || stamp_ns(msg.header.stamp) != *active_->stamp) return;
     if (msg.header.frame_id != frame_id_) return;
     if (xy_distance({msg.pose.position.x, msg.pose.position.y}, active_->xy) > goal_reach_xy_) return;
     active_->ack = true;
     finish_if_complete();
   }

   // ---------------------------------------------------------------
   // 任务完成信号（Mission Complete）回调
   // ---------------------------------------------------------------
   void on_complete(const std_msgs::msg::Bool &msg) {
     if (!active_) return;
     if (!msg.data)
       active_->armed = true; // 接收到 false，先进入 armed 状态，等待下一个 true 才算真正完成
     else if (active_->armed) {
       active_->complete = true;
       finish_if_complete();
     }
   }

   // ---------------------------------------------------------------
   // 检查是否同时满足 Ack 与 Complete，若满足则报告目标已达成
   // ---------------------------------------------------------------
   void finish_if_complete() {
     if (active_ && active_->ack && active_->complete) {
       RCLCPP_INFO(get_logger(), "Frontier goal reached: %.2f, %.2f", active_->xy[0], active_->xy[1]);
       finish_active_goal();
       active_.reset();
       empty_since_.reset();
       report("goal_reached");
     }
   }

   // ---------------------------------------------------------------
   // 请求清除已发布前沿点的 Trigger 服务
   // ---------------------------------------------------------------
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
               finish_active_goal();
               active_.reset();
               if (clearing_reason_ == "proximity_reached") {
                 empty_since_.reset();
                 report("goal_reached", "proximity_fallback");
               } else {
                 report("goal_cleared");
               }
               clearing_reason_.clear();
             }
           } catch (const std::exception &exc) {
             RCLCPP_WARN(get_logger(), "Waypoint clear failed: %s", exc.what());
           }
         });
   }

   // ---------------------------------------------------------------
   // 开始清除流程（因超时、无进度或手动暂停）
   // ---------------------------------------------------------------
   void start_clear(const std::string &reason) {
     if (!active_ || clearing_) return;
     if (reason != "paused" && reason != "proximity_reached")
       suppressed_.push_back({active_->xy, wall_now() + cooldown_});
     clearing_ = true;
     clearing_reason_ = reason;
     report("clearing_goal", reason);
     request_clear();
   }

   // ---------------------------------------------------------------
   // 将选中的前沿目标发布为 Waypoint（PoseStamped）
   // ---------------------------------------------------------------
   void publish_target(const le::XY &xy, const std::string &detail = "") {
     geometry_msgs::msg::PoseStamped msg;
     msg.header.stamp = now();
     msg.header.frame_id = frame_id_;
     msg.pose.position.x = xy[0];
     msg.pose.position.y = xy[1];
     // SCAN 的手动目标接口只接受 XY，Z 设为 0
     msg.pose.position.z = 0.0;
     msg.pose.orientation.w = 1.0;
     double current = wall_now();
     active_ = Active{xy, current, stamp_ns(msg.header.stamp), std::nullopt,
                     xy_distance(xy, {(*base_)[0], (*base_)[1]}), current,
                     false, false, false, std::nullopt, std::nullopt};
     active_->recovery = detail == "recovery_backtrack";
     active_->observed_cells_at_send = map_.project({(*base_)[0], (*base_)[1]}).size();
     if (active_->recovery)
       recovery_gate_.start(active_->observed_cells_at_send);
     waypoint_pub_->publish(msg);
     report("goal_sent", detail);
   }

   void finish_active_goal() {
     if (!active_ || !base_) return;
     auto observed = map_.project({(*base_)[0], (*base_)[1]}).size();
     if (active_->recovery)
       recovery_gate_.finish(observed);
     else if (clearing_reason_.empty() || clearing_reason_ == "proximity_reached")
       goal_progress_gate_.finish(active_->xy, active_->observed_cells_at_send, observed);
   }

   std::optional<le::XY> backtrack_goal() const {
     if (!base_ || trajectory_.poses.empty()) return std::nullopt;
     auto grid = map_.project({(*base_)[0], (*base_)[1]});
     for (auto it = trajectory_.poses.rbegin(); it != trajectory_.poses.rend(); ++it) {
       le::XY xy{it->pose.position.x, it->pose.position.y};
       double range = xy_distance(xy, {(*base_)[0], (*base_)[1]});
       if (range < std::max(2.0, cfg_.min_goal_distance) ||
           range > cfg_.max_goal_distance) continue;
       le::Cell cell{static_cast<int>(std::floor(xy[0] / cfg_.resolution)),
                     static_cast<int>(std::floor(xy[1] / cfg_.resolution))};
       auto state = grid.find(cell);
       if (state == grid.end() || state->second != 0 || planner_blocks(xy)) continue;
       return xy;
     }
     return std::nullopt;
   }

   // ---------------------------------------------------------------
   // 在 RViz 中发布或删除前沿的 Marker（球体）
   // ---------------------------------------------------------------
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

   // ---------------------------------------------------------------
   // 主循环 tick：每秒执行一次，负责状态调度、超时检测、前沿搜索、目标发布等
   // ---------------------------------------------------------------
   void tick() {
     double current = wall_now();
     if (!trajectory_.poses.empty()) {
       trajectory_.header.stamp = now();
       trajectory_pub_->publish(trajectory_);
     }
     if (clearing_) { request_clear(); return; }
     // 数据失效检查：基座、里程计、点云必须在对应超时范围内
     if (!base_ || current - base_wall_ > odom_timeout_ ||
         cloud_wall_ == 0.0 || current - cloud_wall_ > cloud_timeout_) {
       empty_since_.reset();
       report("waiting_for_fresh_data");
       return;
     }
     // 清理已过期的 suppressed 前沿（冷却结束）
     suppressed_.erase(std::remove_if(suppressed_.begin(), suppressed_.end(),
         [current](const Suppressed &s) { return s.expiry <= current; }), suppressed_.end());

     // ------------------- 已有活动目标的情况 -------------------
     if (active_) {
       if (!active_->stamp) {
         // 尚未收到外部 Goal 时间戳，若超出 forward_timeout_ 则放弃该目标
         if (current - active_->sent > forward_timeout_) {
           if (active_->recovery && base_)
             recovery_gate_.finish(map_.project({(*base_)[0], (*base_)[1]}).size());
           active_.reset();
           report("goal_not_forwarded");
         }
         return;
       }
       double remaining = xy_distance(active_->xy, {(*base_)[0], (*base_)[1]});
       if (remaining <= goal_reach_xy_) {
         active_->blocked_since.reset();
         if (!active_->near_since) active_->near_since = current;
         else if (current - *active_->near_since >= near_goal_dwell_) {
           start_clear("proximity_reached");
           return;
         }
       } else {
         active_->near_since.reset();
         if (planner_occupancy_fresh(current) && planner_blocks(active_->xy)) {
           if (!active_->blocked_since) active_->blocked_since = current;
           else if (current - *active_->blocked_since >= blocked_goal_dwell_) {
             start_clear("target_occupied_in_scan");
             return;
           }
         } else {
           active_->blocked_since.reset();
         }
       }
       // 进度检测：若当前距离比历史最佳距离再小 progress_distance_，更新进度计时
       if (remaining < active_->best_distance - progress_distance_) {
         active_->best_distance = remaining;
         active_->progress_wall = current;
       }
       // 超时处理：整体目标超时或长时间没有进度
       if (current - active_->sent > goal_timeout_) start_clear("goal_timeout");
       else if (current - active_->progress_wall > progress_timeout_) start_clear("no_progress");
       return;
     }

     // ------------------- 没有活动目标：尝试寻找新前沿 -------------------
     if (map_.size() >= cfg_.max_voxels) {
       report("map_capacity_reached");
       return;
     }
     // 投影到水平面，检查是否还有 free（未占据）空间
     auto projected = map_.project({(*base_)[0], (*base_)[1]});
     bool has_free = std::any_of(projected.begin(), projected.end(),
                                 [](const auto &item) { return item.second == 0; });
     if (!has_free) {
       report("waiting_for_free_space");
       return;
     }
     if (require_planner_occupancy_ && !planner_occupancy_fresh(current)) {
       report("waiting_for_planner_occupancy");
       return;
     }
     // 构造被抑制的前沿列表（用于前沿选择的过滤）
     std::vector<le::XY> failed;
     for (const auto &s : suppressed_) failed.push_back(s.xy);
     auto unproductive = goal_progress_gate_.blocked(projected.size());
     failed.insert(failed.end(), unproductive.begin(), unproductive.end());
     static const std::vector<le::XY> no_external_obstacles;
     const auto &obstacles = planner_occupancy_fresh(current) ?
         external_obstacles_ : no_external_obstacles;
     auto selection = le::select_frontier(map_, *base_, cfg_, failed, obstacles);
     bool residual_goal = false;
     if (cfg_.global_search && selection.frontier_cells > 0 && selection.clusters == 0) {
       if (!residual_since_) residual_since_ = current;
       if (current - *residual_since_ >= no_frontier_hold_) {
         auto residual_cfg = cfg_;
         residual_cfg.min_frontier_cells = 1;
         auto residual = le::select_frontier(map_, *base_, residual_cfg, failed, obstacles);
         if (residual.goal) {
           selection.goal = residual.goal;
           selection.score = residual.score;
           residual_goal = true;
         }
       }
     } else {
       residual_since_.reset();
     }
     last_frontier_cells_ = selection.frontier_cells;
     last_frontier_clusters_ = selection.clusters;
     last_reachable_frontier_cells_ = selection.reachable_frontier_cells;
     last_candidate_goals_ = selection.candidate_goals;
     last_suppressed_goals_ = selection.suppressed_goals;
     last_open_start_neighbors_ = selection.open_start_neighbors;
     // 在 RViz 中展示选中的前沿（若有）
     publish_marker(selection.goal);
     if (!selection.goal) {
       if (cfg_.global_search &&
           recovery_gate_.eligible(selection, projected.size()) &&
           !observation_only_ && !paused_ && ready_ &&
           waypoint_pub_->get_subscription_count() > 0) {
         auto recovery = backtrack_goal();
         if (recovery) {
           suppressed_.push_back({{(*base_)[0], (*base_)[1]}, current + cooldown_});
           publish_target(*recovery, "recovery_backtrack");
           return;
         }
       }
       if (selection.frontier_cells > 0) {
         empty_since_.reset();
         report(selection.clusters > 0 ? "frontiers_unreachable_or_suppressed" :
                "frontiers_below_cluster_threshold",
                selection.clusters > 0 &&
                        recovery_gate_.waiting_for_new_cells(projected.size()) ?
                    "recovery_waiting_for_new_observations" : "");
       } else {
         if (!empty_since_) empty_since_ = current;
         report(current - *empty_since_ >= no_frontier_hold_ ?
                (cfg_.global_search ? "exploration_complete" : "current_region_complete") :
                "waiting_for_frontier");
       }
       return;
     }
     empty_since_.reset();
     if (observation_only_) report("observation_only");
     else if (paused_) report("paused");
     else if (!ready_ || waypoint_pub_->get_subscription_count() == 0)
       report("waiting_for_waypoint_coordinator");
     else
     publish_target(*selection.goal, residual_goal ? "residual_frontier" : "");
   }

   // ===============================================================
   // 成员变量（配置、地图、状态、ROS 句柄等）
   // ===============================================================
   le::Settings cfg_; // 参数设置对象
   le::VoxelMap map_; // 体素地图实例
   std::string frame_id_, last_status_; // 坐标系 ID 与最近一次状态字符串
   int point_stride_ = 8, max_points_ = 1000; // 点云抽稀步长与最大点数
   double sync_tolerance_ = 0.03, odom_timeout_ = 1.0, cloud_timeout_ = 2.0; // 同步容差与超时阈值
   double goal_reach_xy_ = 0.55, progress_distance_ = 0.35, progress_timeout_ = 12.0; // 目标达成阈值与进度检测
   double goal_timeout_ = 90.0, forward_timeout_ = 3.0, cooldown_ = 45.0; // 各类超时与清除冷却
   double no_frontier_hold_ = 8.0; // 前沿缺失时的保持时间阈值
   double occupancy_timeout_ = 2.0, occupancy_height_tolerance_ = 0.5;
   bool require_planner_occupancy_ = false;
   double near_goal_dwell_ = 1.5, blocked_goal_dwell_ = 2.0;
   bool observation_only_ = true, ready_ = false, paused_ = false; // 运行模式标志
   bool bounds_relative_to_start_ = false, bounds_initialized_ = false;
   bool clearing_ = false, clear_inflight_ = false; // 清除状态标志
   double base_wall_ = 0.0, cloud_wall_ = 0.0, last_clear_wall_ = 0.0; // 时间戳记录
   double occupancy_wall_ = 0.0;
   std::string clearing_reason_;
   std::optional<le::Point> base_; // 基座位置（x,y,z）
   std::optional<Active> active_; // 当前活动目标信息
   std::optional<double> empty_since_; // 空前沿持续时间记录
   std::optional<double> residual_since_;
   std::size_t last_frontier_cells_ = 0, last_frontier_clusters_ = 0;
   std::size_t last_reachable_frontier_cells_ = 0, last_candidate_goals_ = 0;
   std::size_t last_suppressed_goals_ = 0, last_open_start_neighbors_ = 0;
   nav_msgs::msg::Path trajectory_;
   std::deque<std::pair<int64_t, le::Point>> sensor_poses_; // 里程计姿态缓存
   std::deque<std::pair<int64_t, le::Point>> base_poses_;
   std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_clouds_; // 点云缓存
   std::vector<le::XY> external_obstacles_;
   std::vector<Suppressed> suppressed_; // 被抑制的前沿列表（冷却）
   le::RecoveryGate recovery_gate_{8};
   le::GoalProgressGate goal_progress_gate_{8};

   // ROS 接口句柄
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_sub_;
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sensor_sub_, base_sub_;
   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_, complete_sub_;
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input_sub_, goal_sub_, ack_sub_;
   rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_client_;
   rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) { // 程序入口，接受命令行参数（ROS2 会通过它解析 remap、参数等）
  rclcpp::init(argc, argv); // 初始化 ROS 2 客户端库，必须在创建任何 Node 之前调用
  try {  // 使用 try/catch 捕获后续可能抛出的异常，防止异常导致进程直接崩溃
    rclcpp::spin(std::make_shared<Explorer>()); // 创建 Explorer 节点实例（派生自 rclcpp::Node）并进入事件循环
                                                // spin 会一直阻塞（主动轮询），持续调用节点内部的计时器、回调等，直到收到 shutdown 信号
  } catch (const std::exception &exc) {         // 捕获所有 std::exception 派生的异常
    RCLCPP_ERROR(rclcpp::get_logger("legbot_exploration"), "Explorer failed: %s", exc.what()); // 使用 ROS2 日志宏在控制台输出错误信息，便于调试 将异常的描述 (`exc.what()`) 拼进日志
    rclcpp::shutdown(); // 发生致命错误时，主动关闭 ROS2 客户端库，释放底层资源（DDS、线程等）
    return 1;
  }
  rclcpp::shutdown(); // 正常退出时也要调用 shutdown，确保资源释放
  return 0; // 返回 0 表示程序正常退出
}
