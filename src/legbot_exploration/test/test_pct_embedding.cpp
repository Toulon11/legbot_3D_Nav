#include "legbot_exploration/pct_backend.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

namespace le=legbot_exploration;
int main(int argc,char **argv) {
  assert(argc==2);
  le::PctBackend backend(argv[1],0.32,100000,200000,
      "{\"maturity_min_points\":1}");
  std::vector<le::LayerPoint> cloud;
  for(int i=-40;i<40;++i) for(int j=-40;j<40;++j) {
    cloud.push_back({i*0.05,j*0.05,0.0});
    cloud.push_back({i*0.05,j*0.05,1.5});
  }
  backend.append(cloud);
  // Map-timer refresh and planner read must address the same cached version.
  backend.refresh(1);
  auto result=backend.plan(1,{-0.5,0,0.32},{0.5,0,0.32});
  if(result.reason=="pct_backend_unavailable") {
    std::cerr << result.metadata << '\n';
    // A restricted test runner may hide the GPU. Library/API/geometry errors
    // still fail; only explicit CUDA driver availability failures may skip.
    const auto &error=backend.last_error();
    if(error.find("cudaErrorSystemDriverMismatch")!=std::string::npos ||
       error.find("cudaErrorInsufficientDriver")!=std::string::npos ||
       error.find("cudaErrorNoDevice")!=std::string::npos) return 77;
  }
  assert(result.reason=="ok" && !result.points.empty());
  assert(result.start_safe && *result.start_safe && result.goal_safe && *result.goal_safe);
  for(const auto &point:result.points) assert(std::abs(point[2]-0.32)<1e-9);
  assert(backend.start_ready(1,{-0.5,0,0.32}));
  const auto inspection=backend.inspect(1,{-0.5,0,0.32},true,1000,true,3.,.07,8,1000);
  if(!inspection.valid) std::cerr<<backend.last_error()<<'\n';
  assert(inspection.valid && !inspection.surfaces.empty() && !inspection.display.empty());
  assert(inspection.surfaces.size()%56==0 && inspection.returns.size()%16==0);
  assert(inspection.display.size()/56<=1000);
  assert(inspection.preview_state=="no_rising_surface" || inspection.preview_state=="no_usable_ascent_route");
  assert(!backend.observation_poses(1,{-0.5,0,0.32},{0.,0.,0.},{{0.5,0,0}}).empty());
  // A fixed, already observed tread can be a navigation stance without
  // pretending the sensor sees the near-filtered surface under itself.
  const le::Point stance{0.,0.,.32};
  const auto fixed=backend.observation_poses(1,{-.5,0.,.32},{0.,0.,0.},{{0.,0.,0.}},
      std::nullopt,std::nullopt,le::XY{0.,.04},stance,.15,0);
  assert(!fixed.empty() && fixed.front().stance==stance);
  assert(backend.observation_poses(1,{-.5,0.,.32},{0.,0.,0.},{{0.,0.,0.}},
      std::nullopt,std::nullopt,le::XY{1.5,.04},stance,.15,0).empty());
  const auto frontiers=backend.frontiers(1);
  assert(frontiers.valid && frontiers.version==1);
  assert(frontiers.frontier_cells>0 && !frontiers.clusters.empty());
  bool observation_planned=false;
  for(const auto &c:frontiers.clusters) for(const auto &point:c.observations) {
    const le::Point body{point[0],point[1],point[2]+0.32};
    auto selected=backend.plan(1,{-0.5,0,0.32},body,true);
    if(selected.reason=="ok") {
      assert(std::abs(selected.points.back()[2]-body[2])<0.02);
      observation_planned=true;break;
    }
  }
  assert(observation_planned);
  const auto records=backend.point_count();
  backend.append(cloud);
  assert(backend.point_count()==records && records<cloud.size());
  auto duplicate=backend.plan(2,{-0.5,0,0.32},{0.5,0,0.32});
  assert(duplicate.reason=="ok");
  assert(duplicate.metadata.find("unchanged_geometry")!=std::string::npos);
  std::vector<le::LayerPoint> obstacle;
  for(int k=1;k<25;++k) obstacle.push_back({0,0,k*0.05});
  backend.append(obstacle);
  // Place the start inside the configured 0.35 m body + 0.05 m margin.
  auto unsafe=backend.plan(3,{-0.3,0,0.32},{0.5,0,0.32});
  assert(unsafe.reason=="pct_unsafe_start");
  assert(unsafe.start_safe && !*unsafe.start_safe);
  auto around=backend.plan(3,{-1,0,0.32},{1,0,0.32});
  assert(around.reason=="ok");
  assert(around.metadata.find("incremental_blocks")!=std::string::npos);
  le::PctBackend raw(argv[1],0.32,100000,200000,"{\"maturity_min_points\":1}");
  std::vector<le::LayerPoint> noisy_floor;
  for(int i=-20;i<=20;++i) for(int j=-20;j<=20;++j)
    for(double z:{-.003,.004}) noisy_floor.push_back({i*.1,j*.1,z});
  raw.append(noisy_floor);
  const auto raw_check=raw.point_check(1,{0,0,.32});
  assert(raw_check.ready);
  assert(raw_check.metadata.find("ready")!=std::string::npos);
  auto on_raw=raw.plan(1,{-.8,0,.32},{.8,0,.32});
  assert(on_raw.reason=="ok" && !on_raw.points.empty());
  le::PctBackend heights(argv[1],0.32,100000,200000,"{\"maturity_min_points\":1}");
  std::vector<le::LayerPoint> floor_only;
  for(const auto &point:cloud)if(point.z==0.)floor_only.push_back(point);
  heights.append(floor_only);
  assert(!heights.observation_task(1,"observe_height",0,{0.,0.,.32}).ready);
  std::vector<le::LayerPoint> upper_floor;
  for(int i=-5;i<=5;++i)for(int j=-5;j<=5;++j)upper_floor.push_back({i*.1,j*.1,1.46});
  heights.append(upper_floor);
  const auto review=heights.observation_task(2,"observe_height",0,{0.,0.,1.78});
  assert(review.ready && review.metadata.find("new_measured_height")!=std::string::npos);
  assert(!heights.observation_task(2,"observe_height",0,{0.,0.,1.78}).ready);
  const auto still_high=heights.point_check(2,{0.,0.,1.78},false,std::nullopt,0.,false,std::nullopt,true);
  assert(still_high.ready && still_high.ground_z && std::abs(*still_high.ground_z-1.46)<1e-5);
  std::cout << "PASS embedded C++ -> fixed online PCT -> native A* -> XYZ\n";
}
