// 本头文件只声明数据结构和函数接口；具体计算见 exploration_core.cpp。
#pragma once  // 防止同一个头文件被重复包含。

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace legbot_exploration {  // 把本包的名字与其他 C++ 包隔开，避免重名。

// 算法参数：可由 ROS YAML 覆盖。
struct Settings {
  double resolution = 0.25;             // 一个网格边长为 0.25 米。
  double max_ray_range = 9.0;           // 跳过超过 9 米的雷达点。
  std::size_t max_voxels = 900000;      // 稀疏地图的体素数量上限。
  double obstacle_min_relative_z = -0.15; // 相对机体高度的障碍下界。
  double obstacle_max_relative_z = 0.85;  // 相对机体高度的障碍上界。
  double free_min_relative_z = -0.45;   // 投影二维空闲格的最低高度。
  double robot_radius = 0.35;           // 避障时使用的机器人半径。
  double planner_occupancy_radius = 0.0; // SCAN 障碍图已膨胀的半径；外部障碍只补足差额。
  double planner_occupancy_resolution = 0.10; // SCAN 障碍格边长，用于核对格子的实际范围。
  double waypoint_safety_margin = 0.05; // 下发路点时在机体半径外再留出的余量。
  double local_search_radius = 10.0;    // 局部前沿搜索半径，单位米。
  bool global_search = false;           // 由 ROS 配置决定是否进行全图搜索。
  double exploration_min_x = -31.5;     // 全图边界：X 最小值。
  double exploration_max_x = -0.5;      // 全图边界：X 最大值。
  double exploration_min_y = -15.5;     // 全图边界：Y 最小值。
  double exploration_max_y = 15.5;      // 全图边界：Y 最大值。
  double min_goal_distance = 1.0;       // 太近的目标不选，单位米。
  double max_goal_distance = 8.0;       // 单次下发目标不能超过此距离。
  int min_frontier_cells = 3;           // 一个有效前沿簇至少含 3 格。
  double frontier_gain_weight = 1.0;    // 较大前沿簇在评分中更有利。
  double path_cost_weight = 1.0;        // 较长路径在评分中扣分。
  double unknown_path_multiplier = 1.5; // 未知格路径代价相对已知格的倍数。
};

// Cell 只含 X/Y 整数格号；Voxel 还多一个高度 Z 格号。
struct Cell {
  int x = 0, y = 0; // 两个整数是网格编号，并非米制坐标。
  bool operator==(const Cell &rhs) const { return x == rhs.x && y == rhs.y; } // 两个格号相同即同格。
};
struct Voxel {
  int x = 0, y = 0, z = 0; // 三维整数格号。
  bool operator==(const Voxel &rhs) const {
    return x == rhs.x && y == rhs.y && z == rhs.z;
  }
};
struct CellHash { // 散列函数使 Cell 可以充当 unordered_map 的键。
  std::size_t operator()(const Cell &c) const;
};
struct VoxelHash { // 散列函数使 Voxel 可以充当 unordered_map 的键。
  std::size_t operator()(const Voxel &v) const;
};
using Point = std::array<double, 3>; // 实际空间中的 [x,y,z]，单位米。
using XY = std::array<double, 2>;    // 水平面中的 [x,y]，单位米。

// 稀疏体素地图：只为被射线扫到的格子存储证据，未存储的格子就是未知。
class VoxelMap {
 public:
  explicit VoxelMap(Settings settings) : cfg_(settings) {} // 构造时保存算法参数。
  Voxel key(const Point &point) const; // 米制坐标转三维格号。
  Point center(const Voxel &voxel) const; // 三维格号转格子中心坐标。
  XY center(const Cell &cell) const; // 二维格号转水平面格子中心。
  std::size_t integrate(const Point &origin, const std::vector<Point> &points, double base_z); // 融合一帧射线。
  // 高度证据相对“采集该帧时”的机体高度保存。
  std::unordered_map<Cell, int, CellHash> project(const XY &center_xy) const; // 三维证据压成二维格子。
  std::unordered_map<Cell, int, CellHash> project_all() const; // 不按预设矩形裁剪累计地图。
  void set_exploration_bounds(double min_x, double max_x, double min_y, double max_y); // 更新全局搜索边界。
  std::size_t size() const { return voxels_.size(); } // 已保存的体素数。
  std::size_t version() const { return version_; } // 地图更新次数。

 private:
  std::unordered_map<Cell, int, CellHash> project_impl(const XY &center_xy,
                                                       bool all_observed) const;
  void mark(const Voxel &voxel, bool hit); // hit 表示射线末端命中障碍。
  Settings cfg_; // 当前参数副本。
  std::unordered_map<Voxel, int, VoxelHash> voxels_; // 三维格号到证据值。
  std::size_t version_ = 0; // 每融合一次有效点云就加 1。
};

// 一次选点的结果，也包含状态诊断所需的各类计数。
struct Selection {
  std::optional<XY> goal; // 可选目标；空值表示没找到可下发目标。
  double score = 0.0; // 目标评分，越大越优先。
  std::size_t clusters = 0; // 通过最小规模门槛的前沿簇数。
  std::size_t frontier_cells = 0; // 原始未知边界格数量。
  std::size_t reachable_frontier_cells = 0; // 搜索路径可到达的前沿格数量。
  std::size_t candidate_goals = 0; // 满足距离要求的候选目标数量。
  std::size_t suppressed_goals = 0; // 因近期失败而被暂时排除的目标数量。
  std::size_t open_start_neighbors = 0; // 起点周围未被障碍封住的方向数。
};

// 对指定全局矩形逐格验收；valid=false 时不能宣布探索完成。
struct Coverage {
  bool valid = false;
  std::size_t total_cells = 0;
  std::size_t known_free_cells = 0;
  std::size_t known_occupied_cells = 0;
  std::size_t unknown_cells = 0;
  bool reachability_valid = false;
  std::size_t reachable_cells = 0;
  std::size_t reachable_unknown_cells = 0;
  std::size_t excluded_unknown_cells = 0;
  double ratio() const {
    return total_cells ? static_cast<double>(total_cells - unknown_cells) / total_cells : 0.0;
  }
  double reachable_ratio() const {
    return reachable_cells ? static_cast<double>(reachable_cells - reachable_unknown_cells) /
                                 reachable_cells : 0.0;
  }
};

Coverage assess_coverage(const std::unordered_map<Cell, int, CellHash> &grid,
                         const Settings &cfg);
Coverage assess_reachable_coverage(const std::unordered_map<Cell, int, CellHash> &grid,
                                   const Settings &cfg, const XY &robot_xy);

// 从机器人附近的已知空闲格出发，只沿四邻接的已知空闲格搜索。
// 未知邻格是未经选点门槛过滤的原始前沿；invalid 时禁止宣布完成。
struct GlobalFrontiers {
  bool valid = false;
  std::size_t reachable_free_cells = 0;
  std::size_t frontier_cells = 0;
  std::unordered_set<Cell, CellHash> cells; // 仅机器人连通空闲区边上的未知格。
};
GlobalFrontiers assess_global_frontiers(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const Settings &cfg, const XY &robot_xy);

// 普通前沿耗尽而仍有未知格时，从已观察且连通的空闲格找补充观测位置。
std::optional<XY> select_observation_goal(const VoxelMap &map, const Point &robot,
                                          const Settings &cfg,
                                          const std::unordered_map<Cell, int, CellHash> &grid,
                                          const std::vector<XY> &suppressed,
                                          const std::vector<XY> &external_obstacles = {});

// 只有确实存在前沿簇、但当前不可达时才允许沿旧轨迹回退。
// 回退若没有带来足够的新观察格，地图增长之前就暂不重复回退。
class RecoveryGate {
 public:
  explicit RecoveryGate(std::size_t min_new_cells) : min_new_cells_(min_new_cells) {}
  bool eligible(const Selection &selection, std::size_t observed_cells) const;
  bool waiting_for_new_cells(std::size_t observed_cells) const;
  void start(std::size_t observed_cells);
  void finish(std::size_t observed_cells);

 private:
  std::size_t min_new_cells_;
  std::optional<std::size_t> started_at_;
  std::optional<std::size_t> blocked_at_;
};

// 对已到达却没有带来新观察格的目标进行暂时屏蔽。
// 等地图继续增长到门槛后，再允许选到该区域。
class GoalProgressGate {
 public:
  explicit GoalProgressGate(std::size_t min_new_cells) : min_new_cells_(min_new_cells) {}
  void finish(const XY &goal, std::size_t before, std::size_t after);
  std::vector<XY> blocked(std::size_t observed_cells);

 private:
  struct Entry { XY goal; std::size_t unlock_at; };
  std::size_t min_new_cells_;
  std::vector<Entry> entries_;
};

Selection select_frontier(const VoxelMap &map, const Point &robot, const Settings &cfg,
                          const std::vector<XY> &suppressed,
                          const std::vector<XY> &external_obstacles = {},
                          bool all_observed = false,
                          const std::unordered_set<Cell, CellHash> *allowed_frontiers = nullptr);

}  // namespace legbot_exploration
