#include <plan_env/grid_map.h>
#include <cassert>

struct GridMapCollisionTest {
  static void initialize(GridMap &map) {
    auto &p=map.mp_;
    p.resolution_=.1;p.resolution_inv_=10.;
    p.map_voxel_num_=Eigen::Vector3i::Constant(41);
    p.map_bound_min_idx_=Eigen::Vector3i::Constant(-20);
    p.map_bound_max_idx_=Eigen::Vector3i::Constant(20);
    p.map_min_boundary_=Eigen::Vector3d::Constant(-2.);
    p.map_max_boundary_=Eigen::Vector3d::Constant(2.1);
    p.double_cylinder_radius_=.20;p.double_cylinder_offset_=.07;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.03;
    p.collision_check_enabled_=true;
    p.min_occupancy_log_=.5;p.clamp_min_log_=-2.;p.clamp_max_log_=2.;
    const auto size=41*41*41;
    map.md_.occupancy_buffer_.assign(size,p.clamp_min_log_);
    map.md_.occupancy_buffer_inflate_.assign(size,0);
    map.md_.occupancy_buffer_inflate_cnt_.assign(size,0);
    map.rebuildInflationOffsets();
  }
  static bool coarse(GridMap &map,const Eigen::Vector3d &position) {
    Eigen::Vector3i id;map.posToIndex(position,id);
    return map.md_.occupancy_buffer_inflate_[map.toAddress(id)];
  }
};

int main() {
  GridMap map;GridMapCollisionTest::initialize(map);
  // Two real tread/edge heights in one XY remain stored independently.
  map.setOccupied({.05,.05,.85});map.setOccupied({.05,.05,.95});
  assert(map.getOccupancy(Eigen::Vector3d(.05,.05,.85))==1);
  assert(map.getOccupancy(Eigen::Vector3d(.05,.05,.95))==1);
  const Eigen::Vector3d base(.05,.05,1.098);
  assert(GridMapCollisionTest::coarse(map,base));
  assert(map.getInflateOccupancy(base,0.)==0);
  // Actual 3 cm padding above the full occupied voxel remains blocked.
  assert(map.getInflateOccupancy({.05,.05,1.02},0.)==1);
  // A wall in the body-height band is still a collision.
  map.setOccupied({.05,.05,1.05});
  assert(map.getInflateOccupancy(base,0.)==1);
  map.setOccupancy({.05,.05,1.05},0.);
  assert(map.getInflateOccupancy(base,0.)==0);
  // The same rounding correction applies below overhanging obstacles.
  map.setOccupied({.05,.05,1.25});
  assert(map.getInflateOccupancy({.05,.05,1.175},0.)==1);
  assert(map.getInflateOccupancy({.05,.05,1.165},0.)==0);
  assert(map.getInflateOccupancy({3.,0.,1.},0.)==-1);
}
