#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "common/data_types.h"

enum HeuristicType : int { kEuclidean = 0, kManhattan = 1, kDiagonal = 2 };

class Node {
 public:
  Node() = default;
  Node(Eigen::Vector3i idx, Node* parent) : idx(idx), parent(parent) {}
  ~Node() = default;

  bool operator==(const Node& other) const { return idx == other.idx; }

  void Reset() {
    f = 0.0;
    g = 1e9;
    parent = nullptr;
  }

  double f = 1e9;
  double g = 1e9;
  double height = 0.0;
  double ceiling = std::numeric_limits<double>::infinity();
  double interval_center = 0.;
  double ele = 0;
  double cost = 0.0;
  std::int64_t surface_id = -1;  // Legacy map entries have no surface metadata.
  int layer = 0;
  Eigen::Vector3i idx = Eigen::Vector3i(0, 0, 0);  // layer, row, col
  Node* parent = nullptr;
};

struct NodeCompare {
  bool operator()(const Node* a, const Node* b) const { return a->f > b->f; }
};

using MultiLayerGridMap = std::vector<std::vector<std::vector<Node>>>;

class Astar {
 public:
  Astar(const HeuristicType h_type = kDiagonal) : h_type_(h_type) {
    switch (h_type) {
      case kEuclidean:
        printf("Euclidean heuristic is used\n");
        break;
      case kManhattan:
        printf("Manhattan heuristic is used\n");
        break;
      case kDiagonal:
        printf("Diagonal heuristic is used\n");
        break;
    };
  }
  ~Astar() = default;
  Astar(const Astar&) = delete;
  Astar& operator=(const Astar&) = delete;

  void Init(const double cost_threshold, const int num_layers,
            const double resolution, const double step_cost_weight,  const Eigen::MatrixXd& cost_map,
            const Eigen::MatrixXd& height_map, const Eigen::MatrixXd& ele_map);

  // Online maps can lend a read accessor instead of materialising dense
  // Eigen matrices. Only visited search nodes are allocated, per search.
  using CellReader = std::function<std::array<double, 3>(int, int, int)>;
  void InitAccessor(double cost_threshold, int num_layers, int rows, int columns,
                    double resolution, double step_cost_weight, CellReader reader);
  using SurfaceCellReader = std::function<std::tuple<double, double, double, std::int64_t>(int, int, int)>;
  void InitSurfaceAccessor(double cost_threshold, int num_layers, int rows, int columns,
                           double resolution, double step_cost_weight, SurfaceCellReader reader);
  using IntervalCellReader = std::function<std::tuple<double,double,double,std::int64_t,double,double,int>(int,int,int)>;
  void InitIntervalAccessor(double cost_threshold, int num_layers, int rows, int columns,
                            double resolution, double step_cost_weight, IntervalCellReader reader);
  std::size_t GetSearchNodeCount() const { return sparse_nodes_.size(); }
  void SetTimeLimit(double seconds);
  bool TimedOut() const { return timed_out_; }
  void SetBlockedEdgeRadius(int cells) { blocked_edge_radius_ = cells; }
  void SetSurfaceStepLimit(double metres) { surface_step_limit_ = metres; }
  void SetPassageClearance(double metres) { passage_clearance_ = metres; }
  const auto& GetBlockedEdges() const { return blocked_edges_; }

  void Reset();

  void Debug() { debug_ = true; }

  bool Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal);

  std::vector<PathPoint> GetPathPoints() const;

  Eigen::MatrixXd GetResultMatrix() const;
  Eigen::MatrixXi GetVisitedSet() const { return visited_set_; }

  Eigen::MatrixXd GetCostLayer(int layer) const;
  Eigen::MatrixXd GetEleLayer(int layer) const;

 private:
  double time_limit_seconds_ = 0.; // Original API is unlimited unless requested.
  bool timed_out_ = false;
  int blocked_edge_radius_ = 0;
  double surface_step_limit_ = 0.;
  double passage_clearance_ = 0.;
  bool interval_accessor_ = false;
  std::unordered_map<std::int64_t, std::array<int, 6>> blocked_edges_;
  double CalculateStepCost(const Node* node1, const Node* node2) const;

  int DecideLayer(const Node* cur_node) const;

  std::int64_t GetHash(const Eigen::Vector3i& idx) const;
  std::int64_t VisitKey(const Node* node) const;
  Node* NodeAt(int layer, int row, int column) const;

  std::vector<Eigen::Vector3i> GetNeighbors(Node* node) const;

  double GetHeuristic(const Node* node1, const Node* node2) const;

  Eigen::MatrixXd PathToMatrix(const std::vector<Eigen::Vector3i>& path);

  void ToPathPoints(const std::vector<Eigen::Vector3i>& path,
                    std::vector<PathPoint>& path_points);

  void ConvertClosedSetToMatrix(
      const std::unordered_map<std::int64_t, Node*>& closed_set);

  double GetNeighborAverageCost(const Node* node1);

  bool IsValidNodeCoord(int layer, int y, int x) const;

  //后端路径重新优化，选取代价更低的路径
  bool RefinePathSerach();



 private:
  HeuristicType h_type_ = kDiagonal;

  int max_x_ = 0;
  int max_y_ = 0;
  int max_layers_ = 0;
  int xy_size_ = 0;
  double resolution_ = 0;
  MultiLayerGridMap grid_map_;
  IntervalCellReader reader_;
  mutable std::unordered_map<std::int64_t, std::unique_ptr<Node>> sparse_nodes_;
  mutable std::unordered_map<std::int64_t, std::int64_t> sparse_aliases_;
  double cost_threshold_ = 35;
  double step_cost_weight_ = 1.0;

  // int search_layer_depth_ = 1;
  std::vector<int> search_layers_offset_;

  bool debug_ = false;
  Eigen::MatrixXi visited_set_;

  // std::vector<Eigen::Vector3i> search_result_;
  std::vector<Node*> search_result_;
};
