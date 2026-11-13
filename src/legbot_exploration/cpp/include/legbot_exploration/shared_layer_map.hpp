#pragma once


#include "legbot_exploration/pct_backend.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <shared_mutex>
#include <unordered_set>

namespace legbot_exploration {

// A co-located map writer and layer planner share one map. The planner marks
// its short read phase before taking the lock so cloud callbacks drop frames
// instead of queuing stale scans behind A*.
struct SharedLayerMap {
  mutable std::shared_mutex mutex;
  std::shared_ptr<PctBackend> pct;
  double xy_resolution=.10;  // Set by the map node before either executor starts.
  double surface_noise_tolerance=.05, max_step_height=.16;
  std::atomic<unsigned> planning_active{0};
  std::atomic<std::uint64_t> dropped_planning{0};
  std::atomic<std::uint64_t> current_version{0};
  std::atomic<double> published_wall_seconds{0.0};
  std::atomic<double> last_sensor_wall_seconds{0.0};
  std::atomic<double> last_integration_wall_seconds{0.0};
  std::atomic<std::uint64_t> committed_version{0};
  std::atomic<double> last_planning_ms{0.0};
  std::atomic<double> last_read_wait_ms{0.0},last_read_commit_ms{0.0},last_read_phase_ms{0.0};
};

// Commit pending point evidence under the write lock, then lend the same map
// under a read lock. Cloud/timer writers skip the complete phase, including
// the lock handoff, so readers never rebuild geometry under a shared lock.
class LayerMapReadPhase {
 public:
  explicit LayerMapReadPhase(SharedLayerMap &map):map_(map) {
    // The controller can lend one coherent version through frontier choice
    // and A*. Backend helpers borrow that phase instead of recursively
    // acquiring the non-recursive shared_mutex or trying to commit a writer.
    if(held_.count(&map_)) {borrowed_=true;return;}
    started_=std::chrono::steady_clock::now();
    map_.planning_active.fetch_add(1);
    try {
      {
        std::unique_lock<std::shared_mutex> write(map_.mutex);
        const auto acquired=std::chrono::steady_clock::now();
        map_.last_read_wait_ms.store(std::chrono::duration<double,std::milli>(acquired-started_).count());
        const auto version=map_.current_version.load();
        if(map_.pct && map_.pct->point_count() && map_.pct->refresh(version)) {
          map_.committed_version.store(version);
          map_.published_wall_seconds.store(std::chrono::duration<double>(
              std::chrono::steady_clock::now().time_since_epoch()).count());
        }
        map_.last_read_commit_ms.store(std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-acquired).count());
      }
      lock_=std::shared_lock<std::shared_mutex>(map_.mutex);
      held_.insert(&map_);
    } catch(...) {map_.planning_active.fetch_sub(1);throw;}
  }
  ~LayerMapReadPhase() {
    if(borrowed_)return;
    map_.last_read_phase_ms.store(std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-started_).count());
    if(lock_.owns_lock())lock_.unlock();
    held_.erase(&map_);
    map_.planning_active.fetch_sub(1);
  }
  LayerMapReadPhase(const LayerMapReadPhase&)=delete;
  LayerMapReadPhase& operator=(const LayerMapReadPhase&)=delete;
 private:
  SharedLayerMap &map_;
  std::chrono::steady_clock::time_point started_;
  std::shared_lock<std::shared_mutex> lock_;
  bool borrowed_=false;
  inline static thread_local std::unordered_set<SharedLayerMap*> held_;
};

}  // namespace legbot_exploration
