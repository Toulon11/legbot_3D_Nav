#include "legbot_exploration/route_orchestrator.hpp"
#include "legbot_exploration/cross_layer_task.hpp"
#include "legbot_exploration/recovery_planner.hpp"
#include "../cpp/src/exploration_controller_modules.hpp"

#include <cassert>
#include <algorithm>
#include <cmath>
#include <vector>

namespace le = legbot_exploration;

int main(int argc,char **argv) {
  assert(argc==2);
  {
    le::RecoveryController recovery;
    le::RecoveryController::BlockerRecovery first,other;
    first.cross_layer=true;first.floor_attempt=2;
    first.goal.id=11;first.goal.frontier={1.,0.,.13};
    first.blockers={{{1.5,0.,.26},1,1}};
    first.retry_wait=le::RecoveryController::BlockerRecovery::RetryWait{
        {1.,0.,.45},10,20,.13,false};
    first.replan=true;
    other=first;other.goal.id=12;other.goal.frontier={2.,0.,.26};
    recovery.postpone(first);recovery.postpone(other);
    // Re-numbering a physical frontier refreshes only that pending task.
    first.goal.id=99;first.goal.frontier={1.05,0.,.13};
    first.blockers.front().version=2;
    recovery.postpone(first);
    auto resumed=recovery.take_pending();
    assert(resumed && resumed->goal.id==99 && resumed->floor_attempt==0);
    assert(resumed->blockers.front().version==2);
    assert(resumed->replan && resumed->retry_wait && resumed->retry_wait->geometry==10 &&
           resumed->retry_wait->support_height==.13 && !resumed->retry_wait->stable_contacts);
    resumed=recovery.take_pending();
    assert(resumed && resumed->goal.id==12);
    assert(!recovery.take_pending());
    recovery.postpone(first);
    // Nearby independent blockers must not disappear through spatial dedup.
    other.goal.id=13;other.goal.frontier=first.goal.frontier;
    other.blockers.front().point[1]+=.1;
    recovery.postpone(other);
    assert(recovery.take_pending()->goal.id==99);
    assert(recovery.take_pending()->goal.id==13);
    recovery.postpone(other);recovery.clear_pending();
    assert(!recovery.take_pending());
  }

  {
    // A pending surface inside the near blind region cannot be resolved by
    // turning. Short actions are unavailable; the shared pose search must
    // still find a routed observation stance without relaxing arrival safety.
    le::SharedLayerMap shared;
    shared.pct=std::make_shared<le::PctBackend>(argv[1],.32,100000,200000,
                                             "{\"maturity_min_points\":1}");
    std::vector<le::LayerPoint> plane;
    for(int x=-30;x<=30;++x)for(int y=-30;y<=30;++y)plane.push_back({x*.1,y*.1,0.});
    shared.pct->append(plane);shared.pct->refresh(1);shared.current_version.store(1);
    const le::Point base{0.,0.,.32};
    const std::vector<le::PctUnknownBlocker> blockers{{{.5,0.,.13},1,1}};
    assert(shared.pct->observation_task(1,"begin_blocker_review",0,base,0.,0.,&blockers).ready);
    assert(shared.pct->observation_task(1,"arm_blocker_review",0,base).ready);
    std::unordered_map<le::Cell,int,le::CellHash> grid;
    std::vector<le::XY> obstacles;
    le::Settings settings;
    le::PlanningSnapshot snapshot{grid,obstacles,settings,base,.32,true,nullptr,&shared,1,nullptr};
    le::PctRecoveryConstraints limits;limits.ascent=0.;
    limits.permits_goal=[](const le::Point &){return false;};
    le::PctBlockerRecovery recovery;
    const auto action=recovery.next(snapshot,1,{0.,0.,0.},blockers,limits,true,
                                     [](const le::Point &){return true;});
    assert(action && action->kind==le::PctRecoveryAction::Kind::delegate && action->targeted);
    assert(action->gap==blockers.front().point);
    assert(shared.pct->arrival_area(1,action->position,.15,action->yaw).ready);
    assert(shared.pct->plan(1,base,action->position).reason=="ok");
    limits.permits_scan=[](const le::Point &){return false;};
    assert(!recovery.next(snapshot,1,{0.,0.,0.},blockers,limits,true,
                          [](const le::Point &){return true;}));
  }
  {
    // C++ recovery must select a stationary view even when a hole in the
    // outer arrival disc prevents navigating to this same pose anew.
    le::SharedLayerMap shared;
    shared.pct=std::make_shared<le::PctBackend>(argv[1],.32,100000,200000,
                                             "{\"maturity_min_points\":1}");
    std::vector<le::LayerPoint> plane;
    for(int x=-30;x<=30;++x)for(int y=-30;y<=30;++y)
      if(x!=7 || y!=0)plane.push_back({x*.1,y*.1,0.});
    shared.pct->append(plane);shared.pct->refresh(1);shared.current_version.store(1);
    const le::Point base{0.,0.,.32};
    const std::vector<le::PctUnknownBlocker> blockers{{{1.4,0.,.13},1,1}};
    assert(shared.pct->observation_task(1,"begin_blocker_review",0,base,0.,0.,&blockers).ready);
    assert(shared.pct->observation_task(1,"arm_blocker_review",0,base).ready);
    assert(!shared.pct->arrival_area(1,base,.15,0.).ready);
    std::unordered_map<le::Cell,int,le::CellHash> grid;
    std::vector<le::XY> obstacles;
    le::Settings settings;
    le::PlanningSnapshot snapshot{grid,obstacles,settings,base,.32,true,nullptr,&shared,1,nullptr};
    le::PctBlockerRecovery recovery;le::PctRecoveryConstraints limits;
    const auto action=recovery.next(snapshot,1,{0.,0.,0.},blockers,limits,false,
                                     [](const le::Point &){return true;});
    assert(action && action->kind==le::PctRecoveryAction::Kind::scan && action->position==base);
  }
  {
    // Startup near filtering can leave a hole beneath the robot. Failed PCT
    // observer routes must not consume safe endpoints of SCAN recovery.
    le::SharedLayerMap shared;
    shared.pct=std::make_shared<le::PctBackend>(argv[1],.32,100000,200000,
                                             "{\"maturity_min_points\":1}");
    std::vector<le::LayerPoint> ring;
    for(int x=-30;x<=30;++x)for(int y=-30;y<=30;++y)
      if(std::hypot(x*.1,y*.1)>=.8)ring.push_back({x*.1,y*.1,.13});
    shared.pct->append(ring);shared.pct->refresh(1);shared.current_version.store(1);
    const le::Point base{0.,0.,.32},goal{-1.9,0.,.45};
    std::unordered_map<le::Cell,int,le::CellHash> grid;
    std::vector<le::XY> obstacles;
    le::Settings settings;
    le::PlanningSnapshot snapshot{grid,obstacles,settings,base,.32,true,nullptr,&shared,1,nullptr};
    assert(!shared.pct->start_ready(1,base));
    le::PctRecoveryConstraints limits;limits.height_band=le::XY{0.,.16};
    le::PctBlockerRecovery recovery;
    const std::vector<le::PctUnknownBlocker> blockers{{{0.,0.,0.},1,1}};
    assert(!recovery.next(snapshot,1,{0.,0.,0.},blockers,limits,true,
                          [](const le::Point &){return true;}));
    assert(!recovery.pending());
    assert(recovery.rejections().count("targeted_route_start_unverified"));
    auto floor_band=limits;floor_band.height_band=le::XY{0.,.08};
    assert(!recovery.delegate(snapshot,1,floor_band,{goal},.15));
    const auto delegated=recovery.delegate(snapshot,1,limits,{goal},.15);
    assert(delegated && !delegated->targeted && delegated->position==goal);
    assert(shared.pct->arrival_area(1,goal,.15,M_PI).ready);
  }
  {
    le::CrossLayerTask task;
    assert(task.accept({0.,0.,0.},{{1.,0.,.13}}));
    // Recovery can retreat one step inside the corridor; normal ascent
    // targets must still obey monotonic progress and height.
    const le::Point support{1.,0.,.13}, retreat{.6,0.,-.01};
    task.update(support);
    assert(!task.allows(retreat));
    assert(task.allows_recovery(support,retreat,.16));
    assert(!task.allows_recovery(support,retreat,.12));
    assert(!task.allows_recovery(support,{.6,0.,-.04},.16));
    assert(!task.allows_recovery(support,{.6,2.,.13},.16));
    task.exploring_platform(.13);
    task.exploring_platform(.16);
    assert(task.platform_height() && *task.platform_height()==.13);
    task.observed();
    assert(task.platform_height() && *task.platform_height()==.13);
    task.clear();
    assert(!task.platform_height());
  }
  {
    le::StairScanBudget budget;
    const le::Point start{0.,0.,0.}, gap{.9,0.,.13};
    assert(budget.reserve(gap,start));
    assert(budget.reserve({.95,.12,.16},{0.,.3,0.}));
    budget.advance({0.,.3,0.},0.,.25,.05,.05);
    assert(!budget.reserve(gap,{0.,.3,0.}));
    budget.advance({-.3,0.,0.},0.,.25,.05,.05);
    assert(!budget.allows(gap));
    budget.advance({.3,0.,.13},0.,.25,.05,.05);
    assert(budget.reserve(gap,{.3,0.,.13}));
  }
  {
    le::CrossLayerTask task;
    const le::Point base{0.,0.,0.};
    assert(!task.accept(base,{{1.,0.,.05}}));
    assert(task.accept(base,{{1.,0.,.06}}));
  }
  {
    le::CrossLayerTask task;
    const le::Point first_tread{0.,0.,.0767};
    assert(task.accept(first_tread,{{.35,0.,.093}},0.));
    assert(task.engaged() && task.rise()==first_tread[2]);
    assert(task.accept(first_tread,{{.35,0.,.093},{.90,0.,.23},{1.45,0.,.36}}));
  }
  {
    le::CrossLayerTask task;
    const le::Point foot{0.,0.,0.}, tread{1.,0.,.14};
    assert(!task.accept(foot,{{1.,0.,0.}}));
    assert(!task.begin_support_scan());
    assert(task.accept(foot,{{.5,0.,0.},tread}));
    task.update(tread);
    assert(task.phase()=="ascending" && task.rise()>.13);
    assert(task.begin_support_scan());
    assert(task.engaged() && task.phase()=="observing_support");
    assert(!task.begin_support_scan());
    task.observed();
    assert(!task.allows({.5,0.,0.}));
    assert(task.allows({1.,.3,.14}));
    assert(!task.route_allowed(tread,{{.5,0.,.14},{2.,0.,.28}}));
    assert(task.accept(tread,{{1.,.3,.14}}));
    assert(std::abs(task.forward_yaw())<1e-6);
    assert(!task.route_allowed(tread,{{1.,-.3,.14}}));
    const double progress=task.progress();
    assert(task.accept(tread,{{2.,0.,.28}}));
    assert(task.progress()==progress);
    task.update({2.,0.,.28});
    assert(task.begin_support_scan());
    task.observed();
    task.defer("no_forward_route");
    assert(!task.can_resume(tread));
    assert(task.can_resume({2.,0.,.28}));
    task.resume();task.complete();
    assert(!task.engaged() && task.phase()=="completed");
  }
  {
    le::CrossLayerTask task;
    const le::Point start{0.,0.,0.}, tread{1.,0.,.14}, backed{.7,0.,.14}, next{1.5,0.,.28};
    assert(task.accept(start,{{.5,0.,0.},tread,next}));
    assert(!task.begin_reposition(start));
    task.update(tread);
    assert(!task.route_allowed(tread,{backed,next}));
    const double progress=task.progress();
    assert(task.begin_reposition(tread));
    assert(!task.engaged() && !task.begin_reposition(tread));
    task.finish_reposition(backed);
    assert(task.progress()>=progress && task.highest_height()==tread[2]);
    assert(task.route_allowed(backed,{backed,next}));
    assert(!task.begin_reposition(backed));
    assert(task.accept(backed,{next}));
    task.update(next);
    assert(task.begin_reposition(next));
    task.defer("unsafe_stair_reverse");
    assert(task.deferred());
  }
  le::SharedLayerMap shared_phase;
  {
    le::LayerMapReadPhase decision(shared_phase);
    assert(shared_phase.planning_active.load()==1);
    {
      le::LayerMapReadPhase planner(shared_phase);
      assert(shared_phase.planning_active.load()==1);
    }
    assert(shared_phase.planning_active.load()==1);
  }
  assert(shared_phase.planning_active.load()==0);
  le::PlanningGoal goal;
  {
    std::unordered_map<le::Cell,int,le::CellHash> grid;
    std::vector<le::XY> obstacles;
    le::Settings settings;settings.resolution=.1;
    le::PlanningSnapshot snapshot{grid,obstacles,settings,{0.,0.,.32}};
    le::PlanningGoal ordinary;ordinary.xy_tolerance=.15;
    le::PlannerResult route;route.goal=ordinary;route.kind=le::PlannerKind::layer;route.source="layer";
    route.xyz={{0.,0.,.32},{1.,0.,.32},{2.,.2,.32},{2.,1.,.32}};
    for(const auto &p:route.xyz)route.waypoints.push_back({p[0],p[1]});
    route.route.points=route.waypoints;
    le::PathProcessor path;assert(path.load(ordinary,route,false,&snapshot));
    assert(path.waypoints().size()==2);
    auto through=path.dispatch(1,snapshot.base);
    assert(through->reference_points.size()==2 && through->target==route.waypoints[2]);
    assert(!through->xy_tolerance);
    path.advance();
    assert(path.dispatch(2,snapshot.base)->xy_tolerance==.15);
    assert(path.dispatch(2,snapshot.base)->reference_points.empty());
    route.xyz={{0.,0.,.32},{.3,0.,.47},{.6,0.,.62}};
    route.waypoints={{0.,0.},{.3,0.},{.6,0.}};route.route.points=route.waypoints;
    assert(path.load(ordinary,route,false,&snapshot));
    assert(path.waypoints().size()==2 && path.dispatch(3,snapshot.base)->reference_points.empty());
  }
  {
    le::PlanningGoal stair;stair.stair_segment=true;stair.xy_tolerance=.15;
    le::PlannerResult route;route.goal=stair;route.kind=le::PlannerKind::layer;
    route.xyz={{0.,0.,.32},{.3,0.,.45}};
    route.waypoints={{0.,0.},{.3,0.}};route.route.points=route.waypoints;
    le::PathProcessor path;
    assert(path.load(stair,route,false));
    auto dispatch=path.dispatch(1,route.xyz.front());
    assert(dispatch && dispatch->target==route.waypoints.back() && dispatch->xy_tolerance==.15);
    assert(path.current_z()==.45 && dispatch->reference_points.empty());
    route.xyz.insert(route.xyz.begin()+1,{.15,0.,.32});
    route.waypoints.insert(route.waypoints.begin()+1,{.15,0.});
    route.route.points=route.waypoints;
    assert(path.load(stair,route,false));
    dispatch=path.dispatch(2,route.xyz.front());
    assert(dispatch && dispatch->reference_points.size()==2);
    assert(dispatch->reference_points.back()==route.xyz.back());
  }
  // Ordinary navigation keeps an explicit tolerance on its final point too.
  {
    le::PlanningGoal ordinary;ordinary.xy_tolerance=.15;
    le::PlannerResult route;route.goal=ordinary;route.kind=le::PlannerKind::layer;
    route.source="layer";route.waypoints={{.1,0.},{1.,0.}};
    route.xyz={{.1,0.,.32},{1.,0.,.32}};route.route.points=route.waypoints;
    le::PathProcessor path;assert(path.load(ordinary,route,false));
    assert(!path.dispatch(1,{0.,0.,.32})->xy_tolerance);
    path.advance();assert(path.dispatch(2,{0.,0.,.32})->xy_tolerance==.15);
  }
  goal.goal_id = 7;
  goal.attempt_id = 3;
  goal.occupancy_version = 91;
  goal.layer_version = 12;
  goal.observation = {6.5, 0.5};
  goal.request_layer = true;
  le::PlannerResult occupancy;
  occupancy.goal = goal;
  occupancy.kind = le::PlannerKind::occupancy;
  occupancy.route.points = {{0.0, 0.0}, {1.0, 0.0}};
  occupancy.route.reason="ok";
  occupancy.waypoints = {{1.0, 0.0}};
  le::PlannerResult layer = occupancy;
  layer.kind = le::PlannerKind::layer;
  std::vector<le::PlannerResult> results{occupancy, layer};
  assert(le::select_plan(goal, results) == &results[1]);
  auto cross=goal;
  cross.mode=le::ExplorationMode::cross_layer;
  assert(le::select_plan(cross,results)==&results[1]);
  assert(le::select_plan(cross,{occupancy})==nullptr);
  le::PathProcessor processor;
  assert(processor.load(goal, results[1], false));
  assert(processor.active() && processor.goal_id() == 7);
  processor.begin_execution();
  processor.bind_scan_goal(123456);
  assert(!processor.resolve_execution(123455, false, "global_reference_failed"));
  const auto failed = processor.resolve_execution(123456, false, "global_reference_failed");
  assert(failed && !failed->success && failed->goal_id == 7 &&
         failed->attempt_id == 3 && failed->stage_index == 0);
  assert(!processor.resolve_execution(123456, false, "global_reference_failed"));
  assert(!processor.advance() && !processor.active());
  processor.clear();
  results[1].goal.occupancy_version++;
  assert(le::select_plan(goal, results) == &results[0]);

  // A finite execution action has no fabricated geometric route, but uses
  // the same stamped, exactly-once result ownership as navigation waypoints.
  auto turn_goal = goal;
  turn_goal.mode = le::ExplorationMode::flat;
  turn_goal.recovery_request=le::RecoveryRequest::turn;
  le::PlannerResult turn;
  turn.goal = turn_goal;
  turn.kind = le::PlannerKind::observation_turn;
  turn.source = "observation_turn";
  turn.waypoints = {{0.0, 0.0}};
  assert(processor.load(turn_goal, turn, true));
  assert(processor.skip_within({0.0, 0.0}, 0.4) == 0);
  processor.bind_scan_goal(777);
  assert(!processor.resolve_execution(776, true, "turn_completed"));
  auto turned = processor.resolve_execution(777, true, "turn_completed");
  assert(turned && turned->source == "observation_turn" && turned->success);
  assert(!processor.resolve_execution(777, false, "turn_timeout"));
  processor.clear();

  le::Settings cfg;
  cfg.resolution = 1.0;
  cfg.robot_radius = 0.0;
  std::unordered_map<le::Cell, int, le::CellHash> grid;
  for (int x = 0; x <= 7; ++x)
    for (int y = -2; y <= 2; ++y) grid[{x, y}] = 0;
  const std::vector<le::XY> no_obstacles;
  const le::PlanningSnapshot snapshot{grid, no_obstacles, cfg,
      {1.5, 0.5, 0.32}, 0.32, false};
  goal.request_layer = false;
  auto planned = le::plan_candidates(goal, snapshot);
  assert(planned.size() == 1 && planned[0].usable());
  assert(planned[0].route.points.back() == goal.observation);
  le::PathProcessor occupancy_processor;
  assert(occupancy_processor.load(goal, planned[0], false, &snapshot));
  assert(occupancy_processor.waypoints().size() <= planned[0].waypoints.size());
  assert(occupancy_processor.waypoints().back() == goal.observation);
  grid.erase({1, 0});
  planned = le::plan_candidates(goal, snapshot);
  assert(planned.size() == 1 && !planned[0].usable());
  assert(planned[0].reason == "unsafe_start");
  assert(planned[0].failure_class == le::PlannerFailureClass::unsafe_start);
  assert(planned[0].source == "occupancy");
  le::Settings layer_cfg;
  const std::unordered_map<le::Cell,int,le::CellHash> empty_grid;
  goal.request_layer=true;
  goal.request_occupancy=false;

  // Native PCT XYZ survives loading/advancing; no nearby skip or 2-D rewrite.
  goal.mode=le::ExplorationMode::flat;
  le::PlannerResult pct_layer;
  pct_layer.goal=goal;pct_layer.kind=le::PlannerKind::layer;
  pct_layer.route.points={{0.1,0.2},{0.3,0.4},{0.5,0.6}};
  pct_layer.waypoints=pct_layer.route.points;
  pct_layer.xyz={{0.1,0.2,0.32},{0.3,0.4,0.48},{0.5,0.6,0.64}};
  assert(processor.load(goal,pct_layer,false));
  assert(processor.xyz()==pct_layer.xyz);
  auto precise=goal;precise.mode=le::ExplorationMode::recovery;precise.xy_tolerance=.15;
  auto precise_route=pct_layer;precise_route.goal=precise;
  le::PathProcessor precise_processor;
  assert(precise_processor.load(precise,precise_route,true));
  assert(!precise_processor.dispatch(900,{0,0,.32})->xy_tolerance);
  assert(precise_processor.advance());assert(!precise_processor.dispatch(901,{0,0,.32})->xy_tolerance);
  assert(precise_processor.advance());assert(precise_processor.dispatch(902,{0,0,.32})->xy_tolerance==.15);
  precise_processor.bind_scan_goal(902);
  const auto precise_done=precise_processor.resolve_execution(902,true,"goal_reached",le::Point{.6,.6,.64});
  assert(precise_done && precise_done->requested_xy_tolerance==.15 && std::abs(*precise_done->goal_distance-.1)<1e-8);
  precise_processor.clear();assert(precise_processor.load(goal,pct_layer,false));
  assert(!precise_processor.dispatch(903,{0,0,.32})->xy_tolerance);
  assert(processor.current_z() && *processor.current_z()==0.32);
  assert(processor.skip_within({0.1,0.2},10.0)==0);
  assert(processor.advance() && *processor.current_z()==0.48);
  processor.clear();assert(!processor.current_z() && processor.xyz().empty());
  pct_layer.xyz.pop_back();assert(!processor.load(goal,pct_layer,false));
  pct_layer.route.points={{0.1,0.2}};pct_layer.waypoints=pct_layer.route.points;
  pct_layer.xyz={{0.1,0.2,0.32}};
  assert(pct_layer.usable() && processor.load(goal,pct_layer,false));
  assert(processor.xyz()==pct_layer.xyz);processor.clear();

  le::SharedLayerMap shared;
  const le::PlanningSnapshot direct_snapshot{empty_grid,no_obstacles,
      layer_cfg,{0.05,0.05,0.32},0.32,true,nullptr,&shared};
  goal.observation={0.85,0.05};
  planned=le::plan_candidates(goal,direct_snapshot);
  assert(planned.size()==1 && planned[0].reason=="layer_map_unavailable");
  assert(!shared.planning_active.load());
  const auto checks=le::check_normal_starts(direct_snapshot,false,true);
  assert(!checks.layer && !checks.occupancy);
  // A map updated between request creation and read-lock acquisition must
  // never be silently labelled with the older request's map version.
  shared.pct=std::make_shared<le::PctBackend>("/nonexistent/pct",.32,10,1000);
  goal.layer_version=shared.current_version.load()+1;
  const auto stale_map=le::plan_candidates(goal,direct_snapshot);
  assert(stale_map.size()==1 && !stale_map[0].usable());
  assert(stale_map[0].reason=="layer_map_version_changed");
  assert(stale_map[0].failure_class==le::PlannerFailureClass::unavailable);
  // The embedded interpreter reports errors as failed planner results, never
  // an exception escaping the exploration executor or stale successful routes.
  le::PctBackend missing("/nonexistent/pct",0.32,10,1000);
  missing.append({{0,0,0},{1,1,1}});
  auto unavailable=missing.plan(1,{0,0,0.32},{1,1,0.32});
  assert(unavailable.reason=="pct_backend_unavailable" && unavailable.points.empty());
  assert(unavailable.metadata.find("error")!=std::string::npos);
  // A delegate request has no fabricated path, preserves XYZ and accepts only
  // feedback for its own currently bound SCAN goal.
  auto recovery_grid=grid;
  for(int x=0;x<=7;++x) for(int y=-2;y<=2;++y) recovery_grid[{x,y}]=0;
  const le::PlanningSnapshot delegate_snapshot{recovery_grid,no_obstacles,cfg,{-10,0,.32},.32,false,nullptr,nullptr,91};
  goal.recovery_request=le::RecoveryRequest::delegate;goal.mode=le::ExplorationMode::recovery;
  goal.request_layer=false;goal.request_occupancy=true;goal.occupancy_version=91;
  goal.observation_xyz=le::Point{6.5,.5,.32};goal.observation={6.5,.5};
  const auto invalid_delegate=le::plan_candidates(goal,delegate_snapshot);
  assert(invalid_delegate.size()==1 && !invalid_delegate[0].usable());
  le::PlannerResult delegate;delegate.goal=goal;delegate.kind=le::PlannerKind::scan_direct;
  delegate.reason="delegated_to_scan";delegate.waypoints={goal.observation};
  delegate.xyz={*goal.observation_xyz};
  const std::vector<le::PlannerResult> delegated{delegate};
  assert(delegated.size()==1 && delegated[0].kind==le::PlannerKind::scan_direct);
  assert(delegated[0].route.points.empty() && delegated[0].usable());
  assert(le::select_plan(goal,delegated)==&delegated[0]);
  assert(processor.load(goal,delegated[0],true));
  assert(processor.skip_within({0,0},100)==0);
  assert(processor.current_z() && *processor.current_z()==.32);
  auto command=processor.dispatch(9999,delegate_snapshot.base);
  assert(command && command->request==le::RecoveryRequest::delegate && command->target==goal.observation);
  processor.bind_scan_goal(9999);
  assert(!processor.resolve_execution(9998,false,"execution_no_progress"));
  assert(processor.resolve_execution(9999,false,"execution_no_progress"));
  assert(!processor.dispatch(9999,delegate_snapshot.base));
  goal.mode=le::ExplorationMode::flat;
  assert(le::plan_candidates(goal,delegate_snapshot).empty());

  // Action geometry is checked by the recovery module; only the processor
  // serializes the final execution request. The controller supplies parameters.
  goal.mode=le::ExplorationMode::recovery;
  goal.recovery_request=le::RecoveryRequest::turn;goal.recovery_turn_angle=.7;
  goal.observation={3.5,.5};goal.observation_xyz.reset();
  const le::PlanningSnapshot action_snapshot{recovery_grid,no_obstacles,cfg,{3.5,.5,.32},.32,false,nullptr,nullptr,91};
  auto action=le::plan_candidates(goal,action_snapshot);
  assert(action.size()==1 && action[0].usable());
  assert(le::select_plan(goal,action)==&action[0]);
  assert(processor.load(goal,action[0],true));
  command=processor.dispatch(11,action_snapshot.base);
  assert(command && command->action_payload=="11|0.69999999999999996");
  goal.mode=le::ExplorationMode::flat;
  action=le::plan_candidates(goal,action_snapshot);
  assert(action.size()==1 && action[0].kind==le::PlannerKind::observation_turn && action[0].usable());
  assert(le::select_plan(goal,action)==&action[0]);
  assert(processor.load(goal,action[0],false) && !processor.recovery());
  assert(processor.dispatch(15,action_snapshot.base)->request==le::RecoveryRequest::turn);
  goal.recovery_turn_angle=2*std::acos(-1.0);
  action=le::plan_candidates(goal,action_snapshot);
  assert(processor.load(goal,action[0],false));
  assert(processor.waypoints().size()==7);
  double total_angle=0.;
  for(int stage=0;stage<7;++stage) {
    const auto dispatch=processor.dispatch(100+stage,action_snapshot.base);
    assert(dispatch && dispatch->request==le::RecoveryRequest::turn);
    const auto angle=std::stod(dispatch->action_payload.substr(dispatch->action_payload.find('|')+1));
    assert(angle>0 && angle<=1.);total_angle+=angle;
    processor.bind_scan_goal(100+stage);
    assert(processor.resolve_execution(100+stage,true,"turn_completed"));
    processor.advance();
  }
  assert(!processor.active() && std::abs(total_angle-2*std::acos(-1.0))<1e-10);
  goal.recovery_turn_angle=.7;
  goal.mode=le::ExplorationMode::recovery;
  recovery_grid[{3,0}]=1;
  assert(!le::plan_candidates(goal,action_snapshot)[0].usable());
  recovery_grid[{3,0}]=0;
  le::StartChecks causes;
  causes.layer=le::StartAdmission{};causes.layer->reason="no_ground_sample";
  assert(causes.blind());
  causes.layer->physical_conflict=true;assert(!causes.blind());
  causes.layer->physical_conflict=false;
  causes.layer->reason="solid_or_steep_surface";
  assert(!causes.blind());
  causes.layer->reason="unknown_footprint";
  causes.occupancy=le::StartAdmission{};
  causes.occupancy->actual.reject(le::SafetyIssue::occupied);
  assert(!causes.blind());
  return 0;
}
