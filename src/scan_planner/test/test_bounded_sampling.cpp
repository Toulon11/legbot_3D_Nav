#include <plan_manage/bounded_sampling.h>
#include <iostream>
#include <limits>

int main() {
  using namespace scan_planner;
  using V=Eigen::Vector3d;
  std::vector<V> result;
  unsigned checks=0;
  const auto check=[&](bool value) { ++checks; if(!value)throw std::runtime_error("sampling check "+std::to_string(checks)); };
  InitializationBudget budget(.1);
  check(!resampleInitialPolyline({},V::Zero(),.1,budget,result));
  check(!resampleInitialPolyline({V::Zero()},V::Ones(),.1,budget,result));
  check(!resampleInitialPolyline({V::Zero(),V::Zero()},V::Ones(),.1,budget,result));
  check(!resampleInitialPolyline({V::Zero(),V::Constant(std::numeric_limits<double>::quiet_NaN())},V::Ones(),.1,budget,result));
  check(!resampleInitialPolyline({V::Zero(),V::Ones()},V::Ones(),0.,budget,result));
  check(!resampleInitialPolyline({V::Zero(),V::Ones()},V::Ones(),1e-12,budget,result));
  InitializationBudget expired(-1.);
  check(!resampleInitialPolyline({V::Zero(),V::Ones()},V::Ones(),.1,expired,result));
  check(!resampleInitialPolyline(std::vector<V>(20001,V::Ones()),V::Ones(),.1,budget,result));
  check(resampleInitialPolyline({V::Zero(),V(.01,0.,.02)},V(.01,0.,.02),.1,budget,result));
  check(result.size()>=7 && result.size()<=20000 && result.front().norm()<1e-12);
  check((result.back()-V(.01,0.,.02)).norm()<1e-12);
  for(size_t i=1;i<result.size();++i)check(result[i].allFinite() && result[i].z()>=result[i-1].z());
  std::cout<<checks<<" bounded sampling checks passed\n";
}
