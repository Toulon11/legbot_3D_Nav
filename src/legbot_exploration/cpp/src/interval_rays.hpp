#pragma once
#include <array>
#include <algorithm>
#include <map>
#include <set>
#include <limits>
#include <cmath>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

// Free evidence is traced from an actual, timestamp-matched sensor origin.
// A range cap truncates evidence; it does not create an unobserved ceiling.
inline std::map<std::pair<int,int>,std::vector<std::pair<int,int>>> trace_free_columns(
    pybind11::array_t<float,pybind11::array::c_style|pybind11::array::forcecast> points,
    std::array<double,3> origin,double resolution,double ox,double oy,double oz,
    double max_range,std::size_t max_voxels) {
  if(points.ndim()!=2 || points.shape(1)!=3 || !std::isfinite(resolution) || resolution<=0 ||
     !std::isfinite(max_range) || max_range<=0 || !max_voxels)
    throw std::invalid_argument("pct_invalid_ray_configuration");
  const std::array<double,3> offset{ox-resolution*.5,oy-resolution*.5,oz};
  for(int i=0;i<3;++i) if(!std::isfinite(origin[i]) || !std::isfinite(offset[i]) ||
      std::abs((origin[i]-offset[i])/resolution)+max_range/resolution>
        std::numeric_limits<int>::max()-2.)
    throw std::invalid_argument("pct_ray_coordinate_limit");
  const auto input=points.unchecked<2>();
  std::map<std::pair<int,int>,std::set<int>> voxels;
  std::map<std::pair<int,int>,std::vector<std::pair<int,int>>> result;
  pybind11::gil_scoped_release release;
  std::size_t count=0;
  for(pybind11::ssize_t n=0;n<points.shape(0);++n) {
    std::array<double,3> delta{},direction{},crossing{},dt{};
    std::array<int,3> cell{},hit{},step{};
    bool finite=true;
    for(int i=0;i<3;++i) {
      const double value=input(n,i);
      finite &= std::isfinite(value) && std::abs((value-offset[i])/resolution)<
        std::numeric_limits<int>::max()-2.;
      delta[i]=value-origin[i];
    }
    if(!finite)continue;
    const double length=std::hypot(delta[0],delta[1],delta[2]);
    if(length<=1e-9)continue;
    const double reach=std::min(length,max_range);
    for(int i=0;i<3;++i) {
      direction[i]=delta[i]/length;
      cell[i]=static_cast<int>(std::floor((origin[i]-offset[i])/resolution));
      hit[i]=static_cast<int>(std::floor((input(n,i)-offset[i])/resolution));
      step[i]=direction[i]>0?1:direction[i]<0?-1:0;
      dt[i]=step[i]?resolution/std::abs(direction[i]):std::numeric_limits<double>::infinity();
      crossing[i]=step[i]?(offset[i]+(cell[i]+(step[i]>0))*resolution-origin[i])/direction[i]:dt[i];
    }
    double distance=0.;
    while(distance<reach-1e-9 && cell!=hit) {
      const double next=*std::min_element(crossing.begin(),crossing.end());
      if(std::min(next,reach)-distance>1e-9) {
        if(voxels[{cell[0],cell[1]}].insert(cell[2]).second && ++count>max_voxels)
          throw std::invalid_argument("pct_free_voxel_limit");
      }
      for(int i=0;i<3;++i) if(crossing[i]<=next+1e-10) {
        cell[i]+=step[i];crossing[i]+=dt[i];
      }
      distance=next;
    }
  }
  for(const auto &[xy,zs]:voxels) {
    auto &spans=result[xy];
    for(int z:zs) {
      if(!spans.empty() && z<=spans.back().second+1)spans.back().second=z;
      else spans.emplace_back(z,z);
    }
  }
  return result;
}
