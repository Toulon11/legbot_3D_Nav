#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include "legbot_exploration/single_floor_exploration.hpp"
#include <map>
#include <string>
#include <utility>

namespace legbot_exploration {
double wall_now();
double xy_distance(const XY &a, const XY &b);
std::string json_string(const std::string &value);
int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp);

class SelectionPhaseTimer {
 public:
  SelectionPhaseTimer(std::map<std::string,double> &times, const char *phase)
      : times_(times), phase_(phase), began_(wall_now()) {}
  ~SelectionPhaseTimer() { times_[phase_]+=1000.*(wall_now()-began_); }
 private:
  std::map<std::string,double> &times_;
  const char *phase_;
  double began_;
};

template<class Call>
decltype(auto) selection_phase(std::map<std::string,double> &times, const char *name, Call &&call) {
  SelectionPhaseTimer timer(times,name);
  return std::forward<Call>(call)();
}
}  // namespace legbot_exploration
