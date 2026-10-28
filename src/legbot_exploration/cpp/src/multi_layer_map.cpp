#include "legbot_exploration/multi_layer_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace legbot_exploration {
namespace {
std::size_t mix(std::size_t seed, int value) {
  return seed ^ (std::hash<int>{}(value) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u));
}
bool finite(const LayerPoint &p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
}  // namespace

std::size_t OnlineLayerMap::VoxelHash::operator()(const Voxel &v) const {
  return mix(mix(mix(0, v.x), v.y), v.z);
}
std::size_t OnlineLayerMap::CellHash::operator()(const Cell &c) const {
  return mix(mix(0, c.x), c.y);
}

OnlineLayerMap::OnlineLayerMap(LayerMapConfig config) : cfg_(config) {
  if (!(cfg_.xy_resolution > 0.0 && cfg_.z_resolution > 0.0 &&
        cfg_.slice_height > 0.0 && cfg_.max_ray_range > 0.0 &&
        cfg_.surface_merge_gap > 0.0 && cfg_.support_height_tolerance > 0.0 &&
        cfg_.min_clearance > 0.0 && cfg_.max_slope_radians > 0.0 &&
        cfg_.max_step_height > 0.0 && cfg_.support_radius_cells > 0 &&
        cfg_.min_support_columns > 0 && cfg_.min_hit_observations > 0 &&
        cfg_.max_surfaces_per_column > 0 && cfg_.max_voxels > 0)) {
    throw std::invalid_argument("invalid online layer map configuration");
  }
}

OnlineLayerMap::Voxel OnlineLayerMap::key(const LayerPoint &p) const {
  return {static_cast<int>(std::floor(p.x / cfg_.xy_resolution)),
          static_cast<int>(std::floor(p.y / cfg_.xy_resolution)),
          static_cast<int>(std::floor(p.z / cfg_.z_resolution))};
}

void OnlineLayerMap::mark_dirty(const Cell &cell) { dirty_.insert(cell); }

void OnlineLayerMap::update(const Voxel &v, bool hit, bool downward_hit, double point_z,
                            double stamp_seconds) {
  auto it = evidence_.find(v);
  if (it == evidence_.end()) {
    if (evidence_.size() >= static_cast<std::size_t>(cfg_.max_voxels)) {
      map_full_ = true;
      return;
    }
    it = evidence_.emplace(v, Evidence{}).first;
    column_z_[{v.x, v.y}].push_back(v.z);
  }
  Evidence &e = it->second;
  e.last_observed = stamp_seconds;
  const bool was_occupied = e.score > 0;
  if (hit) {
    e.score = std::min(8, e.score + 3);
    if (e.hits < std::numeric_limits<int>::max()) {
      ++e.hits;
      e.z_sum += point_z;
    }
    if (downward_hit && e.downward_hits < std::numeric_limits<int>::max()) {
      ++e.downward_hits;
      e.downward_z_sum += point_z;
    }
  } else {
    e.score = std::max(-5, e.score - 1);
    if (e.rays < std::numeric_limits<int>::max()) ++e.rays;
    // Old surface samples must not certify a surface after rays cleared it.
    if (was_occupied && e.score <= 0) {
      e.hits = 0;
      e.z_sum = 0.0;
      e.downward_hits = 0;
      e.downward_z_sum = 0.0;
    }
  }
  if (hit || was_occupied != (e.score > 0) || e.rays == 3)
    mark_dirty({v.x, v.y});
}

std::size_t OnlineLayerMap::integrate(const LayerPoint &origin,
                                      const std::vector<LayerPoint> &points,
                                      double stamp_seconds,
                                      std::size_t ray_stride) {
  if (!finite(origin)) return 0;
  std::size_t used = 0;
  ray_stride = std::max<std::size_t>(1, ray_stride);
  const double step_size = 0.5 * std::min(cfg_.xy_resolution, cfg_.z_resolution);
  for (const LayerPoint &p : points) {
    if (!finite(p)) continue;
    const double dx = p.x - origin.x, dy = p.y - origin.y, dz = p.z - origin.z;
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (range < cfg_.xy_resolution || range > cfg_.max_ray_range) continue;
    const Voxel endpoint = key(p);
    // Keep dense endpoints for surface support, but trace only a budgeted
    // subset of rays to keep incremental updates responsive.
    if (used % ray_stride == 0) {
      const int steps = std::max(1, static_cast<int>(std::ceil(range / step_size)));
      Voxel previous{};
      bool have_previous = false;
      for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        const Voxel v = key({origin.x + t * dx, origin.y + t * dy, origin.z + t * dz});
        if (v == endpoint || (have_previous && v == previous)) continue;
        update(v, false, false, 0.0, stamp_seconds);
        previous = v;
        have_previous = true;
      }
    }
    // A return observed from below may be a ceiling underside, not a floor.
    update(endpoint, true, origin.z - p.z >= cfg_.z_resolution,
           p.z, stamp_seconds);
    ++used;
  }
  if (used) ++version_;
  return used;
}

std::vector<OnlineLayerMap::Candidate> OnlineLayerMap::candidates(const Cell &cell) const {
  const auto col = column_z_.find(cell);
  if (col == column_z_.end()) return {};
  std::vector<Candidate> raw;
  raw.reserve(col->second.size());
  for (int z : col->second) {
    const auto it = evidence_.find({cell.x, cell.y, z});
    if (it == evidence_.end() || it->second.score <= 0 ||
        it->second.downward_hits == 0) continue;
    raw.push_back({it->second.downward_z_sum / it->second.downward_hits,
                   it->second.downward_hits,
                   it->second.last_observed});
  }
  std::sort(raw.begin(), raw.end(), [](const Candidate &a, const Candidate &b) {
    return a.height < b.height;
  });
  std::vector<Candidate> merged;
  for (const Candidate &entry : raw) {
    if (!merged.empty() && entry.height - merged.back().height <= cfg_.surface_merge_gap) {
      Candidate &last = merged.back();
      last.height = (last.height * last.hits + entry.height * entry.hits) /
                    (last.hits + entry.hits);
      last.hits += entry.hits;
      last.last_observed = std::max(last.last_observed, entry.last_observed);
    } else {
      merged.push_back(entry);
    }
  }
  if (merged.size() > static_cast<std::size_t>(cfg_.max_surfaces_per_column)) {
    std::stable_sort(merged.begin(), merged.end(), [](const Candidate &a, const Candidate &b) {
      return a.hits > b.hits;
    });
    merged.resize(cfg_.max_surfaces_per_column);
    std::sort(merged.begin(), merged.end(), [](const Candidate &a, const Candidate &b) {
      return a.height < b.height;
    });
  }
  return merged;
}

ClearanceKind OnlineLayerMap::clearance(const Cell &cell, double ground,
                                         double *ceiling) const {
  *ceiling = std::numeric_limits<double>::quiet_NaN();
  const auto col = column_z_.find(cell);
  if (col == column_z_.end()) return ClearanceKind::unobserved;
  int free_bins = 0;
  const int required_bins = std::max(1, static_cast<int>(std::ceil(
      cfg_.min_clearance / cfg_.z_resolution)));
  for (int z : col->second) {
    const auto it = evidence_.find({cell.x, cell.y, z});
    if (it == evidence_.end()) continue;
    const Evidence &e = it->second;
    const double height = e.hits > 0 && e.score > 0 ? e.z_sum / e.hits :
                          (z + 0.5) * cfg_.z_resolution;
    if (height <= ground + cfg_.z_resolution * 0.5) continue;
    if (e.score > 0 && e.hits > 0) {
      if (std::isnan(*ceiling) || height < *ceiling) *ceiling = height;
    } else if (e.score <= 0 && e.rays >= 3 &&
               height < ground + cfg_.min_clearance) {
      ++free_bins;
    }
  }
  if (std::isfinite(*ceiling) && *ceiling - ground < cfg_.min_clearance)
    return ClearanceKind::blocked;
  if (free_bins >= std::max(1, required_bins / 2))
    return ClearanceKind::observed_clear;
  return ClearanceKind::unobserved;
}

std::vector<LayerSurface> OnlineLayerMap::derive(const Cell &cell) const {
  std::vector<LayerSurface> result;
  const auto here = candidates(cell);
  for (const Candidate &candidate : here) {
    LayerSurface surface;
    surface.x_cell = cell.x;
    surface.y_cell = cell.y;
    surface.height = candidate.height;
    surface.slice_index = static_cast<int>(std::floor(candidate.height / cfg_.slice_height));
    surface.hit_observations = candidate.hits;
    surface.last_observed = candidate.last_observed;
    surface.clearance = clearance(cell, candidate.height, &surface.ceiling);
    double max_neighbor_delta = 0.0;
    int adjacent = 0;
    for (int dx = -cfg_.support_radius_cells; dx <= cfg_.support_radius_cells; ++dx) {
      for (int dy = -cfg_.support_radius_cells; dy <= cfg_.support_radius_cells; ++dy) {
        if (dx == 0 && dy == 0) continue;
        const Cell neighbor{cell.x + dx, cell.y + dy};
        bool matched = false;
        double best_delta = std::numeric_limits<double>::infinity();
        for (const Candidate &other : candidates(neighbor)) {
          if (other.hits < cfg_.min_hit_observations) continue;
          const double delta = std::abs(other.height - candidate.height);
          if (delta <= cfg_.support_height_tolerance && delta < best_delta) {
            best_delta = delta;
            matched = true;
          }
        }
        if (!matched) continue;
        ++surface.support_columns;
        if (std::abs(dx) + std::abs(dy) == 1) {
          ++adjacent;
          max_neighbor_delta = std::max(max_neighbor_delta, best_delta);
        }
      }
    }
    if (candidate.hits < cfg_.min_hit_observations ||
        surface.support_columns < cfg_.min_support_columns || adjacent == 0) {
      surface.kind = SurfaceKind::insufficient_support;
    } else if (max_neighbor_delta <= std::tan(cfg_.max_slope_radians) * cfg_.xy_resolution) {
      surface.kind = SurfaceKind::flat;
    } else if (max_neighbor_delta <= cfg_.max_step_height) {
      surface.kind = SurfaceKind::step_candidate;
    } else {
      surface.kind = SurfaceKind::steep_or_disconnected;
    }
    result.push_back(surface);
  }
  return result;
}

std::size_t OnlineLayerMap::rebuild_dirty() {
  if (dirty_.empty()) return 0;
  std::unordered_set<Cell, CellHash> affected;
  for (const Cell &cell : dirty_) {
    for (int dx = -cfg_.support_radius_cells; dx <= cfg_.support_radius_cells; ++dx)
      for (int dy = -cfg_.support_radius_cells; dy <= cfg_.support_radius_cells; ++dy)
        affected.insert({cell.x + dx, cell.y + dy});
  }
  dirty_.clear();
  std::size_t rebuilt = 0;
  for (const Cell &cell : affected) {
    if (column_z_.find(cell) == column_z_.end()) continue;
    auto next = derive(cell);
    if (next.empty()) surfaces_.erase(cell);
    else surfaces_[cell] = std::move(next);
    ++rebuilt;
  }
  return rebuilt;
}

std::vector<LayerSurface> OnlineLayerMap::surfaces() const {
  std::vector<LayerSurface> all;
  for (const auto &entry : surfaces_)
    all.insert(all.end(), entry.second.begin(), entry.second.end());
  return all;
}

std::vector<LayerSurface> OnlineLayerMap::surfaces_at(double x, double y) const {
  if (!std::isfinite(x) || !std::isfinite(y)) return {};
  const Cell cell{static_cast<int>(std::floor(x / cfg_.xy_resolution)),
                  static_cast<int>(std::floor(y / cfg_.xy_resolution))};
  const auto it = surfaces_.find(cell);
  return it == surfaces_.end() ? std::vector<LayerSurface>{} : it->second;
}

LayerMapStats OnlineLayerMap::stats() const {
  LayerMapStats result;
  result.voxels = evidence_.size();
  result.columns = column_z_.size();
  result.map_full = map_full_;
  result.version = version_;
  for (const auto &entry : surfaces_) {
    for (const LayerSurface &surface : entry.second) {
      ++result.surfaces;
      if (surface.kind == SurfaceKind::flat) ++result.flat;
      if (surface.kind == SurfaceKind::step_candidate) ++result.step_candidates;
      if (surface.kind == SurfaceKind::insufficient_support) ++result.insufficient_support;
      if (surface.clearance == ClearanceKind::blocked) ++result.blocked_clearance;
      if (surface.clearance == ClearanceKind::unobserved) ++result.unobserved_clearance;
    }
  }
  return result;
}

}  // namespace legbot_exploration
