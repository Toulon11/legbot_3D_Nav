#pragma once

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <vector>

namespace scan_planner {
// Shared by polynomial sampling and old-spline resampling. Wall time remains
// bounded even if simulation time stops while the planner is computing.
struct InitializationBudget {
  static constexpr size_t max_samples = 20000;
  static constexpr unsigned max_attempts = 24;
  std::chrono::steady_clock::time_point deadline;
  explicit InitializationBudget(double seconds)
      : deadline(std::chrono::steady_clock::now() +
                 std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                     std::chrono::duration<double>(seconds))) {}
  bool expired() const { return std::chrono::steady_clock::now() >= deadline; }
};

inline bool resampleInitialPolyline(const std::vector<Eigen::Vector3d> &points,
                                   const Eigen::Vector3d &target, double spacing,
                                   const InitializationBudget &budget,
                                   std::vector<Eigen::Vector3d> &sampled) {
  sampled.clear();
  if (points.size() < 2 || points.size() > budget.max_samples ||
      !target.allFinite() || !std::isfinite(spacing) || spacing <= 0) return false;
  std::vector<double> arc(points.size(), 0.);
  for (size_t i = 0; i < points.size(); ++i) {
    if (budget.expired() || !points[i].allFinite()) return false;
    if (!i) continue;
    const double length = (points[i] - points[i-1]).norm();
    // Repeated points / depleted curves require polynomial initialization.
    if (!std::isfinite(length) || length <= 1e-9) return false;
    arc[i] = arc[i-1] + length;
    if (!std::isfinite(arc[i])) return false;
  }
  for (unsigned attempt = 0; attempt < budget.max_attempts; ++attempt, spacing /= 1.5) {
    if (budget.expired() || spacing < 1e-9 || arc.back()/spacing > budget.max_samples-1)
      return false;
    sampled.clear();
    size_t segment = 0;
    for (double distance = 0.; distance < arc.back(); distance += spacing) {
      if (budget.expired() || sampled.size() >= budget.max_samples-1) return false;
      while (segment+1 < arc.size()-1 && arc[segment+1] <= distance) ++segment;
      const double f = (distance-arc[segment])/(arc[segment+1]-arc[segment]);
      sampled.push_back((1.-f)*points[segment] + f*points[segment+1]);
    }
    sampled.push_back(target);
    if (sampled.size() >= 7) return true;
  }
  return false;
}
} // namespace scan_planner
