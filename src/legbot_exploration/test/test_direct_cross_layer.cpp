#include "legbot_exploration/cross_layer_task.hpp"
#include <cassert>

int main() {
  using namespace legbot_exploration;
  CrossLayerTask task;
  const Point origin{0.,0.,0.};
  assert(!task.accept(origin,{{1.,0.,.06}}));
  // Selecting a higher frontier can first require a level observation pose.
  assert(task.accept(origin,{{.5,0.,0.}},0.,true));
  assert(task.engaged() && task.phase()=="approaching");
  task.update({.5,0.,.06});
  assert(task.phase()=="approaching");
  task.update({.8,0.,.13});
  assert(task.phase()=="ascending");
  // Verified routes may turn back or sideways. The native planner, rather
  // than a fixed flight corridor, decides whether the route is executable.
  assert(task.allows({-.5,2.,.13}));
  assert(task.route_allowed({.8,0.,.13},{{.5,1.,.13},{-.5,2.,.26}}));
  assert(task.accept({.8,0.,.13},{{.5,1.,.13},{-.5,2.,.26}}));
  task.update({-.5,2.,.26});
  task.exploring_platform(.26);
  task.observed();
  assert(task.phase()=="exploring_platform");
  assert(task.platform_height() && *task.platform_height()==.26);
  assert(task.allows_recovery({0.,0.,.26},{2.,-2.,.13},.16));
  assert(!task.allows_recovery({0.,0.,.26},{2.,-2.,.09},.16));
  assert(task.accept({-.5,2.,.26},{{-1.,2.,.39}}));
  assert(!task.platform_height());
  task.complete();
  assert(!task.engaged());
  task.start_from_support({.8,0.,.13},0.);
  assert(task.phase()=="ascending");
  assert(task.highest_height()==.13);
}
