# GO2 task idle and fixed-stand handover

`go2_demo_sequencer` owns navigation/recovery gait activation. Exploration does
not switch gait modes. No arrival or planner safety tolerance changes are needed.

## Task lifetime and idle

- A new `/scan/goal` cancels pending stand immediately. A task stays active while
  waiting for planning and through temporary zero-velocity commands.
- A matching `/scan/goal_stop_ack` or terminal `/scan/goal_plan_failed` settles
  that execution request. Turn/reverse use their matching execution `finished`
  event. Ignore older nonzero stamps and repeated terminal acknowledgments.
- `/exploration/task_active` is a reliable, transient-local Bool describing
  overall task lifetime. Exploration publishes true through mapping, planning,
  recovery and inter-goal waits. It publishes false on confirmed completion,
  observation-only mode, or explicit `/exploration/set_paused` pause. It does
  not command gait or velocity. A pause does not fabricate a SCAN stop receipt;
  any current execution must still settle before stand is allowed.
- While that signal is true, keep the current RL mode at waypoint/goal completion.
  It never activates RL by itself: initial activation still requires an accepted
  SCAN trajectory or recovery action. Without an exploration publisher, standalone
  SCAN tasks retain the existing per-task idle policy. A lost activity publisher
  is not interpreted as task completion; its last known true value holds RL.
- After execution settles **and** exploration permits idle, wait at least
  `goal_settle_seconds` (default **1.5 seconds of wall time**). A new task or an
  exploration resume cancels this wait. The idle timer cannot by itself trigger
  mode 2: fresh zero commands and physical stability must also pass.
- An already-reached goal while standing does not pulse mode 2 again (that could
  toggle the controller to crouch).
- Unstamped Bool mission feedback remains a fallback when no stamped navigation
  goal is in use. It cannot finish a newer stamped goal.

## Physical settling gate

Defaults are in `config/go2_execution.yaml`, loaded by `go2_demo_control.launch.py`:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `stand_stable_seconds` | 0.5 s | Continuous physical stability and zero-command wait |
| `stand_motion_window` | 0.2 s | Body-pose velocity estimation window |
| `stand_feedback_timeout` | 0.3 s | Maximum command/pose age and pose sample gap |
| `stand_max_linear_speed` | 0.03 m/s | Maximum XYZ speed derived from poses |
| `stand_max_angular_speed` | 0.10 rad/s | Maximum quaternion angular speed |
| `stand_max_tilt` | 0.17453 rad | 10° tilt from world up; terrain-dependent setting |
| `stand_cmd_epsilon` | 0.001 | Near-zero tolerance for all six Twist components |

The estimator reads the configured body odometry's **poses**, not its Twist.
It compares the current pose against multiple poses at least half a window old,
using the largest measured linear/angular speed. It checks position including Z,
orientation change and absolute tilt. These are motion/attitude checks, not a
contact-force or whole-body balance proof. The world-up tilt threshold targets
the current flat-floor demo; inclines require an appropriate terrain setting.

Missing, invalid, old or repeated-timestamp poses cannot establish stability.
Frame changes, backwards timestamps and data gaps restart the pose window. A
missing/stale command also blocks stand; a repeated zero command alone proves
neither completion nor physical stopping. A rejected switch keeps RL active and
does not issue a forced stand after a timeout. Normal emergency-stop/passive
handling remains separate.

`/go2/execution_state` publishes the state, task flags, pose-derived speeds/tilt
and `stand_gate` reason (e.g. `exploration_active`, `body_still_moving`,
`body_still_rotating`, `body_pose_stale`, `command_stale`, `body_settling`, `ready`).
The gate is rechecked immediately before the first mode-2 pulse. Once the pulse
has entered the controller, that controller owns its stand interpolation.

## Joint-command transition

The GO2-specific `StateFixedStand` captures the last output q, dq, kp, kd and
feedforward torque. It preserves those commands on entry, then blends all five
fields toward stand_pos, zero velocity/torque and the configured stand gains.
A quintic easing curve gives zero blend velocity/acceleration at both endpoints.
`stand_transition_seconds` defaults to **1.5 seconds of controller time**, advanced
using the actual update period, independently of the idle timer. Both duration
parameters are read at startup; restart the corresponding process after changing
its launch/YAML value.

The public BaseFixedStand implementation for other controllers is unchanged.
A stand-entry mode-2 pulse cannot inadvertently toggle back to crouch. If a new
walking pulse arrives during joint interpolation, finish that interpolation and
honor the saved request even if its pulse has ended. Mode 1/passive bypasses this
wait. Existing emergency-stop handling still has priority.

Command continuity does not by itself guarantee whole-body balance on all
terrain. Gazebo measurements and real-robot validation must be distinguished.

## Verification

With the ROS environment sourced, build these packages with tests enabled, then:

```bash
ctest --test-dir build/rl_quadruped_controller -R '^go2_stand_command_transition$' --output-on-failure
ctest --test-dir build/legbot_bringup -R '^go2_execution_idle$' --output-on-failure
python src/legbot_exploration/test/test_execution_actions_ros.py
```

The ROS tests use an isolated domain without motors or Gazebo. They cover new
requests during idle, active zero-command tasks, old feedback, terminal failures,
recovery preemption, exploration holds, goals already within arrival range,
pose-derived movement despite a zero odometry Twist, angular movement, tilt,
replayed pose stamps, stale commands and stable task-end handover. The C++ test checks
actuator-command continuity, actual-period interpolation and deferred mode input.

### 2026-11-06 Building validation

The default `multi_layer_exploration_demo.launch.py` ran for 83 seconds. Six
waypoints succeeded, with no failure receipts and no RL-to-stand switch while
exploration was active. An explicit pause was requested at t=71.818 s; the
current waypoint finished before the stand wait. Mode 2 was first requested at
t=76.039 s, with pose-derived linear/angular speed both zero, task inactive and
the settling gate ready. Maximum measured body tilt over the next seven seconds
was 1.113 degrees; no fall was observed in this run.

This verifies this handover sequence, not all terrain or real-robot stability.
Artifacts: `build/legbot_bringup/Testing/gazebo_settle_20261106/summary.json`,
`telemetry.jsonl` and `launch.log`. Isolated `go2_execution_idle` and
`exploration_pct_controller` tests also passed, including task activity pause/resume.
