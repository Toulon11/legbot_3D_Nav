#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <cstdint>
#include <tuple>
#include <cmath>
#include <map>
#include <set>
#include <vector>
#include <limits>

namespace py = pybind11;
using namespace pybind11::literals;
using XY = std::pair<int, int>;
using Keys = std::set<XY>;

enum Field {
  F_G, F_C, F_passage_cost, F_relief_cost, F_terrain_cost, F_inflated_terrain_cost, F_gx, F_gy, F_q, F_m,
  F_terrain_inflation_reference, F_measured, F_feature_valid, F_flat, F_solid, F_unknown, F_solid_hard,
  F_unknown_hard, F_hard, F_g_surface_id, F_c_surface_id, F_g_mature, F_c_mature,
  F_C_clear, F_interval_node, field_count
};

// Borrow only one slice's existing sparse buffers. No Python calls are made
// while the GIL is released, and workers never write another slice's buffers.
struct Grid {
  int b;
  std::map<XY, py::array> buffers;
  std::array<int, field_count> offsets{};
  struct Block {
    char *p;
    py::ssize_t sx, sy;
  };
  std::map<XY, Block> blocks;
  Grid(py::dict input, int size) : b(size)
  {
    for (auto item : input) {
      XY key = py::cast<XY>(item.first);
      auto a = py::cast<py::array>(item.second);
      if (a.ndim() != 2 || a.shape(0) != b || a.shape(1) != b || !a.writeable()) {
        throw std::invalid_argument("invalid slice block buffer");
      }
      if (blocks.empty()) {
        auto fields = py::cast<py::dict>(a.dtype().attr("fields"));
        constexpr std::array names{ "G", "C", "passage_cost", "relief_cost", "terrain_cost",
                                    "inflated_terrain_cost", "gx", "gy", "q", "m",
                                    "terrain_inflation_reference", "measured", "feature_valid", "flat",
                                    "solid", "unknown", "solid_hard", "unknown_hard", "hard",
                                    "g_surface_id", "c_surface_id", "g_mature", "c_mature",
                                    "C_clear", "interval_node" };
        for (size_t i = 0; i < names.size(); ++i) {
          offsets[i] = fields[names[i]].cast<py::tuple>()[1].cast<int>();
        }
      }
      buffers.emplace(key, a);
      blocks[key] = { static_cast<char*>(a.mutable_data()), a.strides(0), a.strides(1) };
    }
  }

  static int div(int x, int b)
  {
    return x / b - (x % b < 0 ? 1 : 0);
  }

  char *at(int x, int y) const
  {
    auto bx = div(x, b), by = div(y, b);
    auto i = blocks.find({ bx, by });
    return i ==
           blocks.end() ? nullptr : i->second.p + (x - bx * b) * i->second.sx + (y - by * b) * i->second.sy;
  }

  // NumPy's structured records are packed and can start at unaligned addresses.
  struct Scalar {
    char *address;
    operator double() const {
      double value;
      std::memcpy(&value, address, sizeof(value));
      return value;
    }
    void operator=(double value) const
    {
      std::memcpy(address, &value, sizeof(value));
    }

  };
  Scalar f(char *p, Field field) const
  {
    return { p + offsets[field] };
  }

  unsigned char &flag(char *p, Field field) const
  {
    return *reinterpret_cast<unsigned char *>(p + offsets[field]);
  }

};
bool different(double a, double b)
{
  return !(a == b || (std::isnan(a) && std::isnan(b)));
}

using Surface = std::tuple<double, std::int64_t, bool>;

std::pair<Keys, Keys> update_geometry(
  py::dict input, int block_size, const std::map<XY, std::pair<Surface, Surface>> &evidence)
{
  Grid grid(input, block_size);
  Keys base_changed, ground_changed;
  py::gil_scoped_release release;
  const auto commit = [&](char *row, const Surface &surface, Field height_field,
                          Field identity_field, Field maturity_field) {
    const auto [height, identity, mature] = surface;
    std::int64_t previous_identity;
    std::memcpy(&previous_identity, row + grid.offsets[identity_field], sizeof(previous_identity));
    const bool changed = different(grid.f(row, height_field), height) ||
                         previous_identity != identity || grid.flag(row, maturity_field) != mature;
    grid.f(row, height_field) = height;
    std::memcpy(row + grid.offsets[identity_field], &identity, sizeof(identity));
    grid.flag(row, maturity_field) = mature;
    return changed;
  };
  for (const auto &[xy, surfaces] : evidence) {
    char *row = grid.at(xy.first, xy.second);
    if (!row) {
      throw std::invalid_argument("missing slice geometry block");
    }
    grid.flag(row, F_measured) = true;
    const auto &[ground_surface,ceiling_surface] = surfaces;
    const bool ground = commit(row,ground_surface,F_G,F_g_surface_id,F_g_mature);
    const bool ceiling = commit(row,ceiling_surface,F_C,F_c_surface_id,F_c_mature);
    if (ground) {
      ground_changed.insert(xy);
    }
    if (ground || ceiling) {
      base_changed.insert(xy);
    }
  }
  return {std::move(base_changed), std::move(ground_changed)};
}

int category(double v)
{
  return v < 0 ? -1 : v >= 50 ? 2 : v > 20 ? 1 : 0;
}

bool changed_cost(double a, double b, double threshold)
{
  return a != b && (category(a) != category(b) || std::abs(a - b) > threshold);
}

Keys grow(const Keys &keys, int r)
{
  Keys out;
  for (auto [x, y] : keys) {
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        out.emplace(x + dx, y + dy);
      }
    }
  }
  return out;
}

struct Settings {
  double resolution, radius, margin, slope, step, minimum, desired, ratio, relief_deadband, terrain_deadband,
         inflation_deadband;
  int kernel, inflation;
  Settings(py::dict s)
  {
    auto f = [&](const char *n){
               return py::cast<double>(s[n]);
             };
    resolution = f("resolution");
    radius = f("robot_radius");
    margin = f("safety_margin");
    slope = f("slope_max");
    step = f("step_max");
    minimum = f("minimum_clearance");
    desired = f("desired_clearance");
    ratio = f("standable_ratio");
    relief_deadband = f("relief_update_threshold");
    terrain_deadband = f("terrain_update_threshold");
    inflation_deadband = f("inflation_update_threshold");
    kernel = py::cast<int>(s["traversability_kernel_size"]);
    inflation = py::cast<int>(s["inflation_kernel_size"]);
  }

};

py::dict update_costs(py::dict input, int block_size, const Keys &base, const Keys &ground, py::dict options,
                     py::dict lower, py::dict upper, const Keys &neighbor_changes)
{
  Grid g(input, block_size);
  Grid below(lower, block_size), above(upper, block_size);
  Settings s(options);
  Keys changed, state_changed, terrain_changed, features = ground, relief = base, flat_changed;
  int relief_work = 0, inflation_work = 0, hard_work = 0;
  std::map<XY, std::array<int, 6>> counts;
  {
    py::gil_scoped_release release;
    Keys updated = ground;
    updated.insert(neighbor_changes.begin(), neighbor_changes.end());
    features.insert(updated.begin(), updated.end());
    const auto height_at = [&](int x, int y, double reference) -> double {
      auto* row = g.at(x, y);
      const double current = row ? g.f(row, F_G) : NAN;
      if (std::abs(current-reference) <= s.step + 1e-5) return current;
      double best = current, distance = s.step + 1e-5;
      for (const auto* slice : {&below, &above}) {
        auto* p = slice->at(x, y);
        if (!p) continue;
        const double height = slice->f(p, F_G);
        const double gap = std::abs(height - reference);
        if (gap <= distance) {
          best = height;
          distance = gap;
        }
      }
      return best;
    };
    const auto differences = [&](int x, int y, double z) {
      std::array<double, 2> d{NAN, NAN};
      if (!std::isfinite(z)) return d;
      for (int axis = 0; axis < 2; ++axis) {
        for (int sign : {-1, 1}) {
          const double height = height_at(x + (axis == 0 ? sign : 0),
                                          y + (axis == 1 ? sign : 0), z);
          if (!std::isfinite(height)) continue;
          const double v = (height - z) * sign;
          if (!std::isfinite(d[axis]) || std::abs(v) > std::abs(d[axis])) d[axis] = v;
        }
      }
      return d;
    };
    for (auto [x, y] : updated) {
      for (auto [dx, dy] : { XY{ 1, 0 }, XY{ -1, 0 }, XY{ 0, 1 }, XY{ 0, -1 } }) {
        features.emplace(x + dx, y + dy);
      }
    }
    const double stand_sq = std::pow(1.2 * s.resolution * s.slope, 2);
    for (auto [x, y] : features) {
      char *p = g.at(x, y);
      if (!p) {
        continue;
      }
      const bool old_valid = g.flag(p, F_feature_valid), old_flat = g.flag(p, F_flat);
      const double old_q = g.f(p, F_q), old_m = g.f(p, F_m), z = g.f(p, F_G);
      const auto d = differences(x, y, z);
      const bool valid = std::isfinite(d[0]) && std::isfinite(d[1]);
      double q = d[0] * d[0] + d[1] * d[1];
      // Python max preserves a NaN in its first position.
      double m = d[0] * d[0];
      if (d[1] * d[1] > m) {
        m = d[1] * d[1];
      }
      bool flat = valid && q <= stand_sq;
      g.flag(p, F_feature_valid) = valid;
      g.flag(p, F_flat) = flat;
      g.f(p, F_gx) = d[0] / s.resolution;
      g.f(p, F_gy) = d[1] / s.resolution;
      g.f(p, F_q) = q;
      g.f(p, F_m) = m;
      if (old_valid != valid || old_flat != flat || different(old_q, q) || different(old_m, m)) {
        relief.emplace(x, y);
        changed.emplace(x, y);
      }
      if (old_valid != valid || old_flat != flat) {
        flat_changed.emplace(x, y);
      }
    }
    auto expanded = grow(flat_changed, s.kernel / 2);
    relief.insert(expanded.begin(), expanded.end());
    expanded = grow(neighbor_changes, s.kernel / 2 + 1);
    relief.insert(expanded.begin(), expanded.end());
    auto inflation_boundary = [&](int x, int y, double old, double value) {
      if (old == value || old < 0 || value < 0) {
        return false;
      }
      for (int dx = -s.inflation / 2; dx <= s.inflation / 2; ++dx) {
        for (int dy = -s.inflation / 2; dy <= s.inflation / 2; ++dy) {
          char *n = g.at(x + dx, y + dy);
          if (!n || g.f(n, F_inflated_terrain_cost) < 0) {
            continue;
          }
          double w = std::clamp(1. - (std::hypot(dx,
                                                 dy) * s.resolution - s.radius) /
                                (s.margin + s.resolution), 0., 1.);
          double before = old * w, after = value * w,
                 current = g.f(n, F_inflated_terrain_cost);
          for (double boundary : { 20., 50. }) {
            if ((current <= boundary && boundary < after && before <= boundary) ||
                (current > boundary && boundary >= after &&
                 before >= current - 1e-8)) {
              return true;
            }
          }
        }
      }
      return false;
    };
    for (auto [x, y] : relief) {
      char *p = g.at(x, y);
      if (!p || !g.flag(p, F_measured)) {
        continue;
      }
      ++relief_work;
      bool old_solid = g.flag(p, F_solid), old_unknown = g.flag(p, F_unknown);
      const bool old_pending=g.f(p,F_passage_cost)<0.;
      bool has_ground = std::isfinite(g.f(p, F_G));
      double passage = -1, terrain_relief = -1;
      if (has_ground) {
        const double upper=std::isfinite(g.f(p,F_C))?g.f(p,F_C):
          g.flag(p,F_interval_node)?g.f(p,F_C_clear):g.f(p,F_C);
        const double clearance=upper-g.f(p,F_G);
        if(std::isfinite(upper))passage=clearance+1e-7<s.minimum?50.:std::max(0.,20.*(s.desired-clearance));
        else if(!g.flag(p,F_interval_node))passage=0.;
      }
      if (g.flag(p, F_feature_valid)) {
        if (g.flag(p, F_flat)) {
          terrain_relief = 15. * g.f(p, F_q) / stand_sq;
        }else if (g.f(p, F_m) <= s.step * s.step && s.step > 0) {
          int count = 0;
          for (int dx = -s.kernel / 2; dx <= s.kernel / 2; ++dx) {
            for (int dy = -s.kernel / 2; dy <= s.kernel / 2; ++dy) {
              char *n = g.at(x + dx, y + dy);
              const double height = height_at(x+dx, y+dy, g.f(p,F_G));
              if (n && height == g.f(n, F_G)) {
                count += g.flag(n, F_flat) && g.f(n, F_q) < stand_sq;
              } else {
                const auto d = differences(x+dx, y+dy, height);
                count += d[0]*d[0] + d[1]*d[1] < stand_sq;
              }
            }
          }
          terrain_relief = count >= int(s.ratio * (s.kernel * s.kernel)) - 1 ? 20. * g.f(p,
                                                                                         F_m) /
                           (s.step * s.step) : 50.;
        } else{
          terrain_relief = 50.;
        }
      } else if (passage >= 50) {
        terrain_relief = 0.;
      }
      if (changed_cost(g.f(p, F_relief_cost), terrain_relief, s.relief_deadband)) {
        g.f(p, F_relief_cost) = terrain_relief;
        changed.emplace(x, y);
      }
      if (g.f(p, F_passage_cost) != passage) {
        g.f(p, F_passage_cost) = passage;
        changed.emplace(x, y);
      }
      double terrain = passage >= 0 && g.f(p, F_relief_cost) >= 0 ? passage + g.f(p, F_relief_cost) : -1.;
      if (g.f(p, F_terrain_cost) != terrain) {
        g.f(p, F_terrain_cost) = terrain;
        changed.emplace(x, y);
      }
      double old = g.f(p, F_terrain_inflation_reference);
      if (changed_cost(old, terrain, s.terrain_deadband) || inflation_boundary(x, y, old, terrain)) {
        g.f(p, F_terrain_inflation_reference) = terrain;
        terrain_changed.emplace(x, y);
      }
      g.flag(p, F_solid) = terrain >= 50;
      g.flag(p, F_unknown) = !has_ground;
      if (old_solid != g.flag(p, F_solid) || old_unknown != g.flag(p, F_unknown) ||
          old_pending!=(passage<0.)) {
        state_changed.emplace(x, y);
      }
    }
    std::vector<std::tuple<int, int, double> > weights;
    for (int dx = -s.inflation / 2; dx <= s.inflation / 2; ++dx) {
      for (int dy = -s.inflation / 2; dy <= s.inflation / 2; ++dy) {
        double w = std::clamp(1. - (std::hypot(dx, dy) * s.resolution - s.radius) / (s.margin + s.resolution),
                              0., 1.);
        if (w > 0) {
          weights.emplace_back(dx, dy, w);
        }
      }
    }
    for (auto [x, y] : grow(terrain_changed, s.inflation / 2)) {
      char *p = g.at(x, y);
      if (!p || !g.flag(p, F_measured)) {
        continue;
      }
      ++inflation_work;
      double value = -1.;
      if (g.f(p, F_terrain_cost) >= 0) {
        for (auto [dx, dy, w] : weights) {
          char *n = g.at(x + dx, y + dy);
          if (n && g.f(n, F_terrain_cost) >= 0) {
            value = std::max(value, g.f(n, F_terrain_cost) * w);
          }
        }
      }
      if (changed_cost(g.f(p, F_inflated_terrain_cost), value, s.inflation_deadband)) {
        g.f(p, F_inflated_terrain_cost) = value;
        changed.emplace(x, y);
      }
    }
    double radius = s.radius + s.margin;
    int half = int(std::ceil(radius / s.resolution + .5));
    std::vector<XY> disk;
    for (int dx = -half; dx <= half; ++dx) {
      for (int dy = -half; dy <= half; ++dy) {
        if (std::hypot(std::max(std::abs(dx) * s.resolution - .5 * s.resolution, 0.),
                       std::max(std::abs(dy) * s.resolution - .5 * s.resolution, 0.)) <= radius + 1e-8) {
          disk.emplace_back(dx, dy);
        }
      }
    }
    for (auto [x, y] : grow(state_changed, half)) {
      char *p = g.at(x, y);
      if (!p || !g.flag(p, F_measured)) {
        continue;
      }
      ++hard_work;
      bool unknown = false, solid = false;
      for (auto [dx, dy] : disk) {
        char *n = g.at(x + dx, y + dy);
        unknown |= !n || g.flag(n, F_unknown) || g.f(n,F_passage_cost)<0.;
        solid |= n && g.flag(n, F_solid);
      }
      if (unknown != g.flag(p, F_unknown_hard) || solid != g.flag(p, F_solid_hard)) {
        g.flag(p, F_unknown_hard) = unknown;
        g.flag(p, F_solid_hard) = solid;
        g.flag(p, F_hard) = unknown || solid;
        changed.emplace(x, y);
      }
    }
    Keys touched = base;
    touched.insert(changed.begin(), changed.end());
    Keys blocks;
    for (auto [x, y] : touched) {
      blocks.emplace(Grid::div(x, g.b), Grid::div(y, g.b));
    }
    for (auto key : blocks) {
      auto b = g.blocks.find(key);
      if (b == g.blocks.end()) {
        continue;
      }
      auto &c = counts[key];
      for (int x = 0; x < g.b; ++x) {
        for (int y = 0; y < g.b; ++y) {
          char *p = b->second.p + x * b->second.sx + y * b->second.sy;
          bool finite = std::isfinite(g.f(p, F_G));
          c[0] += finite;
          c[1] += g.flag(p, F_solid);
          c[2] += finite && g.flag(p, F_hard);
          c[3] += finite && !g.flag(p, F_unknown) && !g.flag(p, F_solid) && !g.flag(p, F_hard) &&
                             g.f(p, F_inflated_terrain_cost) >= 0 && g.f(p, F_inflated_terrain_cost) <= 20;
          c[4] += finite && !g.flag(p, F_feature_valid);
          c[5] += finite && g.f(p, F_C) - g.f(p, F_G) < s.minimum;
        }
      }
    }
  }
  py::dict result;
  result["changed"] = changed;
  constexpr std::array count_names{
    "measured_surface_cells", "solid_cells", "unknown_inflated_cells",
    "safe_cells", "unassessed_gradient_cells", "low_clearance_cells"
  };
  py::dict block_counts;
  for (const auto &[tile, values] : counts) {
    py::dict fields;
    for (size_t i = 0; i < values.size(); ++i) {
      fields[count_names[i]] = values[i];
    }
    block_counts[py::cast(tile)] = std::move(fields);
  }
  result["counts"] = std::move(block_counts);
  result["work"] = py::dict("base_cells"_a = base.size(), "feature_cells"_a = features.size(),
                            "terrain_cells"_a = relief_work, "inflation_cells"_a = inflation_work,
                            "hard_cells"_a = hard_work);
  return result;
}

// The native and Python paths share exactly one boundary rule: an unknown
// cut/XY next to a known non-obstacle ground. Sources already carry terrain
// qualification. Python applies the shared same-G auxiliary height filter to
// these proposals before committing members; footprint/visibility checks
// belong to later pose selection.
using Source = std::tuple<int, int, double, std::int64_t>;
using Potential = std::pair<double, std::int64_t>;
using Column = std::tuple<std::vector<Potential>, bool, bool>;
using Boundary = std::tuple<double, std::vector<Source> >;

std::map<XY, std::vector<Boundary> > find_boundaries(
  const Keys &columns, const std::map<XY, Column> &summaries)
{
  std::map<XY, std::vector<Boundary> > result;
  py::gil_scoped_release release;
  for (const auto &[x,y] : columns) {
    const auto own=summaries.find({x,y});
    if (own!=summaries.end() && std::get<1>(own->second)) continue;
    std::vector<Source> sources;
    std::vector<double> heights;
    bool adjacent_obstacle=false;
    for (int dx=-1; dx<=1; ++dx) {
      for (int dy=-1; dy<=1; ++dy) {
        if (!dx && !dy) continue;
        const auto neighbor=summaries.find({x+dx,y+dy});
        if (neighbor==summaries.end()) continue;
        adjacent_obstacle |= std::get<2>(neighbor->second);
        for (const auto &[height,id] : std::get<0>(neighbor->second)) {
          sources.emplace_back(x+dx,y+dy,height,id);
          heights.push_back(height);
        }
      }
    }
    if (adjacent_obstacle || sources.empty()) continue;
    std::sort(heights.begin(),heights.end());
    const auto mid=heights.size()/2;
    const double height=heights.size()%2 ? heights[mid] : (heights[mid-1]+heights[mid])/2.;
    std::sort(sources.begin(),sources.end());
    result[{x,y}].emplace_back(height,std::move(sources));
  }
  return result;
}

#include "interval_rays.hpp"

PYBIND11_MODULE(_legbot_layer_kernels, m) {
  m.def("trace_free_columns", &trace_free_columns);
  m.def("update_geometry", &update_geometry);
  m.def("update_costs", &update_costs);
  m.def("find_boundaries", &find_boundaries, py::arg("columns"), py::arg("summaries"));
}
