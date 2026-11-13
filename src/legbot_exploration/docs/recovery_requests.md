# Unified recovery requests

## Ownership

The exploration controller supplies the recovery target and map mask.
`plan_candidates` routes each `RecoveryRequest` (`position`, `reverse`, `shift`,
`turn`, `delegate`) through `plan_recovery_request` in
`recovery_request_planner.cpp`. That function validates the supplied action
and returns a planner result for the controller to dispatch.

`PathProcessor` selects/loads a result, preserves recovery geometry, serializes
the action request, binds the execution timestamp and accepts one matching
terminal result. The node's publisher bridge sends either a waypoint, the SCAN
reverse request or the SCAN turn request. Only execution publishes velocities or
gait modes. Existing pause/manual takeover and explicit action cancellation remain.

- Position: selected backend plans a recovery route to the supplied target.
- Reverse: validate fixed-heading rear geometry and the 0.20–0.60m executor
  contract, then check the corresponding map's retreat corridor. Both masks
  return independent results; layer results do not consult the 2-D projection.
- Turn: check the circular body footprint, which covers all intermediate headings.
  This returns an action, not a fabricated A* line. Turn direction/magnitude is
  supplied by the controller.
- Delegate: validate the supplied destination with ordinary point safety using a
  requested map, without admitting the start or planning the connection. Return
  exactly that target and `path_validated:false`. SCAN decides actual feasibility.

Geometry checks and action construction previously inside `request_recovery_turn`,
`can_turn_here` and `try_recovery_reverse` have been removed from the controller.
Those controller wrappers now supply action parameters to the same planning pipeline.

## Unknown blockers in PCT

Flat and cross-layer PCT exploration pass the original observation target and
the cells rejected by endpoint checks or native A* to `PctBlockerRecovery`.
`PctBackend::observation_poses` is the same pose interface used by ordinary
frontiers and stair recognition. It checks the LiDAR mounting pose, range,
field of view, measured occlusion and the observed arrival/blind area. Recovery
cannot bypass the near-point filter or substitute an unchecked yaw-only view.

The recovery planner returns one action. It prefers observing from the current
pose, then verifies a short move to another visible pose. Stair requests supply
the ascent direction and actual foot contacts to verify supported short moves. The
controller waits for a matching stop, fresh odometry and integrated point cloud
before rechecking the view or replanning to the original target.

High-relief cells with sufficient clearance also produce observation targets.
The targets include the lower neighbours participating in the height difference,
with the expected tread height used only to aim the observation. The cells remain
blocked until actual returns update G and the ordinary cost and arrival checks
pass. Low-clearance cells can nominate their blocking C for one observation;
their clearance rejection remains in force.

The map owns one persistent `BlockerReview` registry, shared by flat and climb
tasks. A key uses XY and the expected surface height, with the existing noise
band for aliases. After turning, arming records the integrated-frame serial.
Completion requires a later real return in that exact XY cell at a related
height. The return band uses the existing step capability (at most 0.16 m), or
the noise band if larger, because the estimated next tread can differ from the
actual one. It cannot credit a far lower floor or a neighbour's hit. The measured
height is never replaced by this estimate. The registry also retains the actual
matched height so another slice or a changed estimate cannot repeat that review.
A scan without target returns times out as `blocker_no_target_return`; an armed
attempt is not reopened just because another map version or task appears.

Each arrival cut collects its own failure evidence. A successful parking check
returns no blockers from failed alternatives. Blind-area rejection returns only
its missing area. An ordinary floor with an admitted actual start settles failed
observer searches/routes and moves to other clusters instead of entering directed
recovery. Its first failure is deferred and the global review is the second
attempt. An unsafe actual start still uses recovery; climb recovery is retained.

Arrival observation diagnostics distinguish parking/height rejection, repeated
headings, range/FOV exclusion, occlusion and insufficient visible targets. Feedback
includes the planned and actual position plus selection/observation map versions.

Stair reverse checks use the configured step capability, capped at 0.16 m,
instead of a fixed 0.05 m same-tread band. This applies to candidate selection,
early/late stop positions, the rear-foot corridor and actual-stop verification.
Collision, measured support and executor distance checks still apply.

Every reached cross-layer planning waypoint performs a mandatory centre, left,
right, centre sweep. Each view waits for its dwell and a subsequently integrated
cloud frame. A safe next tread does not skip this sweep. For intermediate
waypoints, the remaining route is suspended, its next destination is checked
again on the new map, then execution resumes or enters directed recovery.

If the checked views are exhausted, the planner can request one direct SCAN
attempt for the original safe target. Flat-floor height and cross-layer ascent
constraints and the shared arrival/blind-area check also apply to that endpoint;
the unsafe starting pose is not required to pass ordinary start admission.
SCAN receives the target and arrival
tolerance with an empty reference path. A successful stop is checked against
the requested height and the fresh map before exploration resumes. An
exhausted flat-floor attempt returns to the normal frontier review policy.
An exhausted cross-layer attempt is deferred.

## Controller policy and targets

The status fields `recovery_safe_candidates`, `recovery_target_map_version`
and `recovery_goal_xy_tolerance` describe the controller candidate query. A blind
start without a safe destination reports `no_safe_recovery_destination`; it
does not quietly reinstate the former weak destination fallback.

Ordinary start admission is shared with recovery-exit checks. It now retains the
PCT cause in `normal_start_checks.layer.reason`. `StartChecks::blind` requires all
available backends to report absent observations (including unknown footprint),
not measured physical conflict. A backend error or missing fresh map is not a
blind-start authorization. Simultaneous unknown and occupied footprint evidence
must not be downgraded to pure blindness.

For a blind start, the controller chooses a recovery destination and explicitly
requests delegation immediately. Otherwise it requests a position route. If a
position request reports unsafe start with a safe destination, the controller
rechecks ordinary start conditions and may retry the **same** destination as an
explicit delegate request, with a new attempt. Neither planner does that implicitly.

Candidate map queries and delegation use ordinary destination safety. There is no additional parking-disk inflation. The controller requests a per-final-waypoint XY tolerance using `recovery_goal_xy_tolerance` (default 0.15m); the normal SCAN default remains 0.40m. Normal goals and intermediate stages omit the override. PCT returns spatially diverse candidates; blind-start control prefers farther candidates and filters unsuccessful revisits. See [per-goal tolerance](goal_tolerance.md) for the atomic interface and feedback.

There is no robot-distance min/max gate on recovery destinations. The retired
`recovery_goal_min_distance` and `recovery_goal_max_distance` parameters were
removed. No additional hit-count requirement was introduced. Sparse strips and
single hits are not enough for a full ordinary body footprint. There is no weaker
`delegate_targets` list and no automatic nearest-0.5m replacement anymore.

Historical targets are selected by the controller without a minimum distance.
Fixed-heading reverse and turn remain separate controller strategies; historical
position recovery does not imply backward playback of the recorded curve.

## Feedback and lifetime

Requests/results carry goal/attempt IDs, request type and map versions. Mismatched
map versions reject pending planner results; changed maps do not cancel an executing
route. Delegated endpoints are checked again on the requested current map before
loading. Reverse/turn use the same path-processor result ownership as navigation.

After the final matching successful receipt, wait for a newer map and apply the
ordinary admission function to the actual pose. Success at the destination alone
does not finish recovery. If still unsafe, record that attempt and choose again.
Normal success resumes normal exploration; exhausted strategies report blocked
and use the existing bounded attempt batches/periodic recheck, without reporting
exploration complete. Failure/progress/timeout reporting while moving remains
SCAN's responsibility.

The layer-selection boundary where millimetre ground noise crosses a slice cut
was identified separately during diagnosis. This change does not weaken ordinary
admission or change PCT surface/layer interpretation to hide that issue.

## Tests

- `exploration_planning_pipeline`: stamped result ownership, independent masks,
  version rejection, validated action geometry/dispatch, unchanged delegated
  endpoint, and physical-conflict vs blind-start classification.
- `exploration_fixed_layer_map` / `exploration_pct_adapter`: ordinary-safe destinations,
  sparse strip rejection, far safe destinations and unchanged ordinary native A*.
- `exploration_pct_recovery`: real combined ROS node with a synthetic blind start
  and a safe region >3m away; verifies immediate delegation, failure matching,
  successful arrival without false recovery exit, and fresh ordinary-safe exit.
- `exploration_pct_controller` / `exploration_pct_completion`: normal PCT and
  independent completion behavior.

The current interface extends scan_waypoint_mission forwarding and SCAN per-goal arrival handling. Gait/stand policy remains unchanged.

## Historical Building verification before per-goal tolerance (2026-11-03)

A default Building run with the new candidate rules was observed for ~87 wall
seconds (~80 simulation seconds). At the static initial pose the live PCT map
continued to update but still had zero ordinary safe cells, hence no destination
with additional parking allowance. No motion goal was sent, no fall occurred,
and the test processes were cleaned up. This is not successful normal exploration.
The previous diagnostic's 3.11m safe candidate appeared after the old weak
initial motion; it cannot be assumed to exist in the unmodified initial snapshot.
The isolated ROS test separately provides a real safe region >3m away and verifies
that it is delegated and that recovery exit requires fresh actual-position admission.
Resolving the default scene's initial observation coverage is additional work;
this implementation does not fill its blind region, inject ground truth or relax
normal map safety to claim it is solved.
