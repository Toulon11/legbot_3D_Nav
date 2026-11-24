#include "a_star/a_star_search.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <unistd.h>

using std::cout;
using std::endl;

// 9 neighbors in 2d
static std::vector<Eigen::Vector2i> kNeighbors = std::vector<Eigen::Vector2i>{
    Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(-1, 1),
    Eigen::Vector2i(0, -1),  Eigen::Vector2i(0, 1),  Eigen::Vector2i(1, -1),
    Eigen::Vector2i(1, 0),   Eigen::Vector2i(1, 1),
};



void Astar::Init(const double cost_threshold, const int num_layers,
                 const double resolution,  const double step_cost_weight, const Eigen::MatrixXd& cost_map,
                 const Eigen::MatrixXd& height_map,
                 const Eigen::MatrixXd& ele_map) {
  auto t0 = std::chrono::high_resolution_clock::now();

  cost_threshold_ = cost_threshold;
  step_cost_weight_  = step_cost_weight;
  reader_ = {};
  timed_out_ = false;
  blocked_edges_.clear();
  search_result_.clear();
  sparse_nodes_.clear();
  sparse_aliases_.clear();
  resolution_ = resolution;

  max_x_ = cost_map.cols();
  max_y_ = cost_map.rows() / num_layers;
  max_layers_ = num_layers;
  xy_size_ = max_x_ * max_y_;

  int row_offset = 0;
  grid_map_.resize(max_layers_);
  for (size_t i = 0; i < max_layers_; ++i) {
    row_offset = i * max_y_;
    grid_map_[i].resize(max_y_);
    for (size_t j = 0; j < max_y_; ++j) {
      grid_map_[i][j].resize(max_x_);
      for (size_t k = 0; k < max_x_; ++k) {
        double height = height_map(j + row_offset, k);
        double z = static_cast<int>(height / resolution);
        grid_map_[i][j][k] = Node(Eigen::Vector3i(z, j, k), nullptr);
        grid_map_[i][j][k].cost = cost_map(j + row_offset, k);
        grid_map_[i][j][k].height = height;
        grid_map_[i][j][k].ele = ele_map(j + row_offset, k);
        grid_map_[i][j][k].layer = i;
      }
    }
  }
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);

  search_layers_offset_.clear();
  search_layers_offset_.emplace_back(0);
  for (int i = 0; i < num_layers; ++i) {
    search_layers_offset_.emplace_back(-(i + 1));
    search_layers_offset_.emplace_back(i + 1);
  }

  printf(
      "Astar initialized, max_x: %d, max_y: %d, max_layers: %d, time elapsed: "
      "%f ms\n",
      max_x_, max_y_, max_layers_, duration.count() / 1000.0);
}

void Astar::Reset() {
  if (reader_) {
    search_result_.clear();
    sparse_nodes_.clear();
    sparse_aliases_.clear();
    return;
  }
  for (size_t i = 0; i < grid_map_.size(); ++i) {
    for (size_t j = 0; j < grid_map_[i].size(); ++j) {
      for (size_t k = 0; k < grid_map_[i][j].size(); ++k) {
        grid_map_[i][j][k].Reset();
      }
    }
  }
}

void Astar::InitAccessor(double threshold, int layers, int rows, int columns,
                         double resolution, double weight, CellReader reader) {
  InitSurfaceAccessor(threshold, layers, rows, columns, resolution, weight,
      [reader = std::move(reader)](int layer, int row, int column) {
        const auto [cost, ground, gateway] = reader(layer, row, column);
        return std::make_tuple(cost, ground, gateway, std::int64_t{-1});
      });
}

void Astar::InitSurfaceAccessor(double threshold, int layers, int rows, int columns,
                               double resolution, double weight, SurfaceCellReader reader) {
  InitIntervalAccessor(threshold,layers,rows,columns,resolution,weight,
    [reader=std::move(reader)](int layer,int row,int column) {
      const auto [cost,ground,gateway,identity]=reader(layer,row,column);
      return std::make_tuple(cost,ground,gateway,identity,
                             std::numeric_limits<double>::infinity(),ground,-1);
    });
  passage_clearance_=0.;
  interval_accessor_=false;
}

void Astar::InitIntervalAccessor(double threshold, int layers, int rows, int columns,
                                double resolution, double weight, IntervalCellReader reader) {
  interval_accessor_=true;
  timed_out_ = false;
  search_result_.clear();
  sparse_nodes_.clear();
  sparse_aliases_.clear();
  grid_map_.clear();
  cost_threshold_ = threshold;
  max_layers_ = layers; max_y_ = rows; max_x_ = columns;
  resolution_ = resolution; step_cost_weight_ = weight;
  reader_ = std::move(reader);
  search_layers_offset_ = {0};
  for (int i = 1; i <= layers; ++i) {
    search_layers_offset_.push_back(-i);
    search_layers_offset_.push_back(i);
  }
}

Node* Astar::NodeAt(int layer, int row, int column) const {
  if (!reader_) return const_cast<Node*>(&grid_map_[layer][row][column]);
  const auto key = GetHash(Eigen::Vector3i(layer, row, column));
  auto found = sparse_nodes_.find(key);
  if (found != sparse_nodes_.end()) return found->second.get();
  const auto alias=sparse_aliases_.find(key);
  if(alias!=sparse_aliases_.end())return sparse_nodes_.at(alias->second).get();
  const auto [cost, ground, gateway, surface_id, ceiling, center, owner] = reader_(layer, row, column);
  const auto physical=owner>=0?GetHash(Eigen::Vector3i(owner,row,column)):key;
  if(physical!=key) {
    sparse_aliases_[key]=physical;
    found=sparse_nodes_.find(physical);
    if(found!=sparse_nodes_.end())return found->second.get();
  }
  auto node = std::make_unique<Node>(Eigen::Vector3i(
      static_cast<int>(ground / resolution_), row, column), nullptr);
  node->cost = cost;
  node->height = ground;
  node->ceiling=ceiling;
  node->interval_center=center;
  node->ele = gateway;
  node->layer = owner>=0?owner:layer;
  node->surface_id = surface_id;
  auto* result = node.get();
  sparse_nodes_.emplace(physical, std::move(node));
  return result;
}

std::int64_t Astar::GetHash(const Eigen::Vector3i& idx) const {
  return static_cast<std::int64_t>(idx[0]) * max_y_ * max_x_ +
         static_cast<std::int64_t>(idx[1]) * max_x_ + idx[2];
}

std::int64_t Astar::VisitKey(const Node* node) const {
  if(interval_accessor_ && node->surface_id>0)
    return GetHash(Eigen::Vector3i(static_cast<int>(node->surface_id),node->idx[1],node->idx[2]));
  if (node->surface_id < 0) return GetHash(node->idx);
  return GetHash(Eigen::Vector3i(node->layer, node->idx[1], node->idx[2]));
}

// Surface IDs are local to their XY column.
static bool SameGroundAtXY(const Node* first, const Node* second) {
  return first->surface_id == second->surface_id &&
         (first->surface_id > 0 || std::abs(first->height - second->height) <= 1e-7);
}


// 判断坐标是否在地图合法边界内（图层、y轴、x轴）
bool Astar::IsValidNodeCoord(int layer, int y, int x) const {
    // 校验图层边界（0 <= layer < 最大图层数）
    if (layer < 0 || layer >= max_layers_) {
        std::cerr << "Error: 图层越界，合法范围[0, " << max_layers_ - 1 << "]，输入值：" << layer << std::endl;
        return false;
    }
    // 校验y轴边界（对应grid_map_[layer][y][x]的第二个维度）
    if (y < 0 || y >= max_y_) {
        std::cerr << "Error: y轴坐标越界，合法范围[0, " << max_y_ - 1 << "]，输入值：" << y << std::endl;
        return false;
    }
    // 校验x轴边界（对应grid_map_[layer][y][x]的第三个维度）
    if (x < 0 || x >= max_x_) {
        std::cerr << "Error: x轴坐标越界，合法范围[0, " << max_x_ - 1 << "]，输入值：" << x << std::endl;
        return false;
    }
    // 所有坐标均合法
    return true;
}


void Astar::SetTimeLimit(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0.)
    throw std::invalid_argument("A* time limit must be finite and nonnegative");
  time_limit_seconds_ = seconds;
}

bool Astar::Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal) {
  auto t0 = std::chrono::high_resolution_clock::now();
  const auto began = std::chrono::steady_clock::now();
  timed_out_ = false;
  blocked_edges_.clear();

  if (reader_) Reset();

  if (!search_result_.empty()) {
    Reset();
    search_result_.clear();
  }
  
  //检测是否起始点和终点都符合要求
  if (!IsValidNodeCoord(start[0],start[2],start[1]) || !IsValidNodeCoord(goal[0],goal[2],goal[1]))
  {
    return false;
  }
  
  auto start_node = NodeAt(start[0],start[2],start[1]);
  auto goal_node = NodeAt(goal[0],goal[2],goal[1]);
  start_node->g = 0.0;

  if (goal_node->cost > cost_threshold_) {
    printf("goal node is not reachable, cost: %f, layer: %d\n", goal_node->cost, goal_node->layer);
    return false;
  }
  // 优先队列（open_set）：按节点的f值（g+h）从小到大排序（通过NodeCompare实现）
  std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set;
  // 哈希表（closed_set）：存储已处理的节点（避免重复访问）
  std::unordered_map<std::int64_t, Node*> closed_set;

  open_set.push(start_node);  // 起点加入开放集

  printf("start searching\n");

  Node* best_node = start_node;
  std::size_t iterations = 0;
  while (!open_set.empty()) {
    if ((iterations++ & 63) == 0 && time_limit_seconds_ > 0. &&
        std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count() >= time_limit_seconds_) {
      timed_out_ = true;
      search_result_.clear();
      return false;
    }
    // 步骤1：从开放集取出 f 值最小的节点（当前节点）
    Node* current_node = open_set.top();
    open_set.pop();

    // 步骤2：判断是否到达终点
    const bool reached_goal = goal_node->surface_id < 0
        ? current_node->idx == goal_node->idx
        : current_node->idx.tail<2>() == goal_node->idx.tail<2>() &&
          SameGroundAtXY(current_node, goal_node);
    if (reached_goal) {
      while (current_node->parent != nullptr) {
        search_result_.emplace_back(current_node);
        current_node = current_node->parent;
      }
      std::reverse(search_result_.begin(), search_result_.end());
      if (debug_) ConvertClosedSetToMatrix(closed_set);
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now() - t0);
      printf("path found, time elapsed: %f ms\n",
             duration.count() / 1000.0);
      return true;
    }

    // 步骤3：将当前节点加入封闭集（标记为已访问）
    closed_set[VisitKey(current_node)] = current_node;
    // 步骤4：确定当前节点所在的图层（基于地形特征动态调整）
    int layer = DecideLayer(current_node);
    // 步骤5：遍历所有邻居节点（8方向+对角线，共9个邻居）
    int i, j = 0;
    double tentative_g = 0.0;
    // 遍历2D邻域（8个方向+中心？实际是8个方向，见kNeighbors定义）
    int idx=0;
    for (const auto& neighbor : kNeighbors) {
      // 计算邻居节点的坐标（i为行，j为列）
      idx++;
      i = current_node->idx[1] + neighbor[0];//向xy方向添加节点
      j = current_node->idx[2] + neighbor[1];

          // 检查坐标是否超出地图边界（越界则跳过）
      if (i < 0 || i >= max_y_ || j < 0 || j >= max_x_) {
        continue;
      }
      // 获取邻居节点（基于当前层layer），这个节点在grid_map中会被赋予权重，即costmap的cost
      auto neighbor_node = NodeAt(layer,i,j);
      const bool measured = reader_ && surface_step_limit_ > 0.;
      const auto supported_edge = [&](const Node* node) {
        return node->cost >= 0. && node->cost <= cost_threshold_ &&
               std::abs(node->height - current_node->height) <= surface_step_limit_ + 1e-5 &&
               (passage_clearance_<=0. ||
                std::min(node->ceiling,current_node->ceiling)-
                  std::max(node->height,current_node->height)+1e-7>=passage_clearance_);
      };
      // Sparse measured cuts need no duplicated tread cells at a seam.
      // A horizontal move may end on the adjacent cut's actual support.
      const bool missing = neighbor_node->cost >= 1e6;
      const bool other_surface = std::abs(neighbor_node->height - current_node->height) >
                                 surface_step_limit_ + 1e-5;
      if (measured && (missing || other_surface)) {
        for (const int next_layer : {layer - 1, layer + 1}) {
          if (next_layer < 0 || next_layer >= max_layers_) continue;
          auto* candidate = NodeAt(next_layer, i, j);
          if (supported_edge(candidate) &&
              (!missing || std::abs(candidate->height - current_node->height) > 1e-5) &&
              (!supported_edge(neighbor_node) || candidate->cost < neighbor_node->cost)) {
            neighbor_node = candidate;
          }
        }
      }

      // 检查邻居节点是否可通行
      if ((measured && !supported_edge(neighbor_node)) || (!measured && neighbor_node->cost > cost_threshold_ &&
          (abs(neighbor_node->ele) < 0.5 ||
           std::abs(neighbor_node->height - current_node->height) > 0.3))) {
        if (blocked_edge_radius_ > 0 && blocked_edges_.size() < 256 &&
            std::hypot(double(i) - start[2], double(j) - start[1]) <= blocked_edge_radius_ &&
            (double(i) - start[2]) * (double(goal[2]) - start[2]) +
            (double(j) - start[1]) * (double(goal[1]) - start[1]) > 0.) {
          const auto& from = current_node->idx;
          blocked_edges_.try_emplace(GetHash(Eigen::Vector3i(layer, i, j)),
              std::array<int, 6>{layer, i, j, current_node->layer, from[1], from[2]});
        }
        continue;
      }
      // 计算当前节点到邻居节点的代价
      auto diff = neighbor_node->idx - current_node->idx;// 坐标差      
      double step_cost = step_cost_weight_ * neighbor_node->cost;


      double diff_cost=std::sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]);

      // 总代价 = 当前节点g值 + 欧氏距离（坐标差的3D距离,用于靠近下一个点） + 步骤成本（远离障碍物）
      tentative_g = current_node->g + diff_cost +step_cost;
      

      // 检查邻居是否已在closed_set中
      auto p_neighbor = closed_set.find(VisitKey(neighbor_node));
      if (p_neighbor != closed_set.end()) {
        if (tentative_g >= p_neighbor->second->g) {
          continue; // 若新成本更高，无需更新
        }
      }
      // 若新成本更低，更新邻居节点的g、f值和父节点，并加入open_set，未探索过的邻居点一开始的权重是非常高的1e+9
      if (tentative_g < neighbor_node->g) {
        neighbor_node->g = tentative_g;// 相当于更新上一个节点的成本
        neighbor_node->f = tentative_g + GetHeuristic(neighbor_node, goal_node);// f = g + 启发式成本h
        neighbor_node->parent = current_node;// 记录父节点（用于回溯路径）
        open_set.push(neighbor_node);// 加入开放集等待探索
      }
    }  

    if (open_set.empty())
    {
      /* code */
      while (current_node->parent != nullptr) {
        // search_result_.emplace_back(Eigen::Vector3i(
        //     current_node->layer, current_node->idx[1],
        //     current_node->idx[2]));
        search_result_.emplace_back(current_node);
        current_node = current_node->parent;
      }
    }
  }

// 若open_set为空仍未找到终点，说明无路径
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);
  printf("path not found\n, time elapsed: %f ms\n",
         duration.count() / 1000.0);
  if (debug_) {
    ConvertClosedSetToMatrix(closed_set);// 调试模式下记录访问过的节点
  }
  return false;
}


int Astar::DecideLayer(const Node* cur_node) const {
  // Interval queries all borrow the same owner's costs and identity. There
  // is no alternate representation to optimize at this XY.
  if(interval_accessor_)return cur_node->layer;
  int original_layer = cur_node->layer;  // 初始图层（默认返回值）
  int i = cur_node->idx[1];
  int j = cur_node->idx[2];
  double cur_height = cur_node->height;

  int best_layer = original_layer;
  double min_cost = NodeAt(original_layer,i,j)->cost;

  // 遍历多层偏移（扩大图层搜索范围）
  for (const auto offset : search_layers_offset_) {
    int cur_layer = original_layer + offset;

    // 检查图层是否越界
    if (cur_layer < 0 || cur_layer >= max_layers_) {
      continue;
    }

    // 获取当前搜索节点
    // const Node& search_node = grid_map_[cur_layer][ni][nj];
    const Node& search_node = *NodeAt(cur_layer,i,j);

    const bool same_ground = cur_node->surface_id < 0
        ? std::abs(search_node.height - cur_height) <= 0.2
        : SameGroundAtXY(cur_node, &search_node);
    if (!same_ground || search_node.cost > cost_threshold_ || search_node.cost < 0) {
      continue;
    }

    // 更新最小代价图层
    if (search_node.cost < min_cost) {
      min_cost = search_node.cost;
      best_layer = cur_layer;
    }
  }

  // 若未找到更优图层（如所有邻居都越界或高度差过大），返回原始图层
  return best_layer;
}



double Astar::CalculateStepCost(const Node* node1, const Node* node2) const {}

double Astar::GetHeuristic(const Node* node1, const Node* node2) const {
  double cost = 0.0;

  if (h_type_ == kEuclidean) {
    // l2 distance
    cost = (node1->idx - node2->idx).norm();
  } else if (h_type_ == kDiagonal) {
    // octile distance
    Eigen::Vector3i d = node1->idx - node2->idx;
    int dx = abs(d(0)), dy = abs(d(1)), dz = abs(d(2));
    int dmin = std::min(dx, std::min(dy, dz));
    int dmax = std::max(dx, std::max(dy, dz));
    int dmid = dx + dy + dz - dmin - dmax;
    double h =
        std::sqrt(3) * dmin + std::sqrt(2) * (dmid - dmin) + (dmax - dmid);
    cost = h;
  } else if (h_type_ == kManhattan) {
    cost = (node1->idx - node2->idx).lpNorm<1>();
  } else {
    assert(false && "not implemented");
  }

  // cost += std::abs(node1->idx[0] - node2->idx[0]) * 10;
  return cost;
}


std::vector<PathPoint> Astar::GetPathPoints() const {
  std::vector<PathPoint> path_points;

  auto size = search_result_.size();
  path_points.resize(size);

  if (size == 0) {
    printf("path is empty\n, convert to path points failed\n");
    return path_points;
  }

  for (size_t i = 0; i < size; ++i) {
    // path_points[i].layer = search_result_[i][0];
    // path_points[i].x = search_result_[i][2];
    // path_points[i].y = search_result_[i][1];
    // if (i > 0) {
    //   path_points[i].heading =
    //       std::atan2(search_result_[i][1] - search_result_[i - 1][1],
    //                  search_result_[i][2] - search_result_[i - 1][2]);
    // }
    path_points[i].layer = search_result_[i]->layer;
    path_points[i].x = search_result_[i]->idx(2);
    path_points[i].y = search_result_[i]->idx(1);
    path_points[i].height = search_result_[i]->height;
    if (i > 0) {
      path_points[i].heading =
          std::atan2(search_result_[i]->idx(1) - search_result_[i - 1]->idx(1),
                     search_result_[i]->idx(2) - search_result_[i - 1]->idx(2));
    }
  }

  if (size > 1) {
    path_points[0].heading = path_points[1].heading;
  }

  return path_points;
}

Eigen::MatrixXd Astar::GetResultMatrix() const {
  if (search_result_.empty()) {
    printf("path is empty\n, convert to matrix failed\n");
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd path_matrix(search_result_.size(), 3);
  for (size_t i = 0; i < search_result_.size(); ++i) {
    path_matrix(i, 0) = search_result_[i]->layer;
    path_matrix(i, 1) = search_result_[i]->idx[1];
    path_matrix(i, 2) = search_result_[i]->idx[2];
  }
  return path_matrix;
}

void Astar::ConvertClosedSetToMatrix(
    const std::unordered_map<std::int64_t, Node*>& closed_set) {
  visited_set_ = Eigen::MatrixXi(closed_set.size(), 3);
  int count = 0;
  for (auto i = closed_set.begin(); i != closed_set.end(); ++i) {
    visited_set_(count, 0) = i->second->layer;
    visited_set_(count, 1) = i->second->idx[1];
    visited_set_(count, 2) = i->second->idx[2];
    count += 1;
  }
}

std::vector<Eigen::Vector3i> Astar::GetNeighbors(Node* node) const {

}

Eigen::MatrixXd Astar::GetCostLayer(int layer) const {
  Eigen::MatrixXd cost_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      cost_layer(i, j) = NodeAt(layer,i,j)->cost;
    }
  }
  return cost_layer;
}

Eigen::MatrixXd Astar::GetEleLayer(int layer) const {
  Eigen::MatrixXd ele_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      ele_layer(i, j) = NodeAt(layer,i,j)->ele;
    }
  }
  return ele_layer;
}
