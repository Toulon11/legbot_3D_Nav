#pragma once

#include "legbot_exploration/multi_layer_map.hpp"

#include <atomic>
#include <shared_mutex>

namespace legbot_exploration {

// A co-located map writer and layer planner share one map. The planner marks
// its short read phase before taking the lock so cloud callbacks drop frames
// instead of queuing stale scans behind A*.
struct SharedLayerMap {
  mutable std::shared_mutex mutex;
  OnlineLayerMap *map = nullptr;  // Owned by the co-located map node.
  std::atomic<bool> planning_active{false};
  std::atomic<std::uint64_t> dropped_planning{0};
  std::atomic<std::uint64_t> current_version{0};
  std::atomic<std::size_t> published_surfaces{0};
  std::atomic<double> published_wall_seconds{0.0};
  std::atomic<double> last_planning_ms{0.0};
};

}  // namespace legbot_exploration
