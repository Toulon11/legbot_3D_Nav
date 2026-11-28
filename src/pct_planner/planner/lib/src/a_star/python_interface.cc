#include "a_star/a_star_search.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/functional.h"
#include "pybind11/stl.h"

namespace py = pybind11;

PYBIND11_MODULE(a_star, m) {
  // enum
  py::enum_<HeuristicType>(m, "HeuristicType")
      .value("EUCLIDEAN", HeuristicType::kEuclidean)
      .value("MANHATTAN", HeuristicType::kManhattan)
      .value("DIAGONAL", HeuristicType::kDiagonal)
      .export_values();

  auto pyAstar = py::class_<Astar>(m, "Astar");
  pyAstar
      .def(py::init<HeuristicType>(),
           py::arg("h_type") = HeuristicType::kDiagonal)
      .def("init", &Astar::Init)
      .def("init_accessor", &Astar::InitAccessor)
      .def("init_surface_accessor", &Astar::InitSurfaceAccessor)
      .def("init_interval_accessor", &Astar::InitIntervalAccessor)
      .def("get_search_node_count", &Astar::GetSearchNodeCount)
      .def("set_time_limit", &Astar::SetTimeLimit)
      .def("timed_out", &Astar::TimedOut)
      .def("set_blocked_edge_radius", &Astar::SetBlockedEdgeRadius)
      .def("set_surface_step_limit", &Astar::SetSurfaceStepLimit)
      .def("set_passage_clearance", &Astar::SetPassageClearance)
      .def("get_blocked_edges", &Astar::GetBlockedEdges)
      .def("search", &Astar::Search)
      .def("search_towards", &Astar::SearchTowards,py::arg("start"),py::arg("goal"),
           py::arg("goal_height"),py::arg("stop"),py::arg("accept_goal")=true)
      .def("get_partial_result_matrix", &Astar::GetPartialResultMatrix)
      .def("debug", &Astar::Debug)
      .def("get_result_matrix", &Astar::GetResultMatrix)
      .def("get_cost_layer", &Astar::GetCostLayer)
      .def("get_ele_layer", &Astar::GetEleLayer)
      .def("get_visited_set", &Astar::GetVisitedSet);
}
