# Optional front-ground LiDAR experiment

## Start

```bash
ros2 launch legbot_exploration multi_layer_exploration_demo.launch.py
```

`enable_ground_lidar` defaults to true in this multi-layer demo. It is forwarded
through the exploration demo to the simulation and combined map node. Both
original point-cloud input and the additional input update the same sparse
online PCT map. Use `enable_ground_lidar:=false` to disable it explicitly.
The lower-level simulation and other launch entry points retain their own defaults.

## Sensor and data path

- Simulation sensor frame: `ground_lidar_link`, fixed to `trunk` at
  `(0.35, 0, 0.20)` m, RPY `(0, 35 degrees, 0)`; local +X points forward
  and 35 degrees downward relative to body +X.
- Scan: 256 horizontal by 64 vertical samples, 10 Hz, 0.05–12 m range.
  Sensor-frame angular ranges are ±45° horizontally and -5° to +20° vertically.
  This window targets the floor ahead of the 0.70 m sensor-origin cutoff.
  At a nominal 0.32 m base height on a level floor, the sensor is 0.52 m high;
  ideal ground ranges are approximately 0.81–5.15 m. Body attitude and obstacles
  change the actual footprint. The earlier live runs below used the old mount.
- `/ground_lidar/points_raw`: independent Gazebo-to-ROS point cloud.
- `/ground_lidar/points_registered`: transformed into the navigation frame
  (`map` in the global demo), using TF at the cloud timestamp.
- `/ground_lidar/odometry_lidar`: the corresponding sensor position/orientation
  at that same timestamp.
- Online map parameter: `auxiliary_cloud_topic`. Empty auxiliary cloud disables
  this input. Point clouds must already be registered to the map frame.
- `/exploration/layers/status.cloud_sources`: per-input seen/integrated frames.
  The map fuses real XYZ directly, without pose queues or inferred surfaces.

The auxiliary cloud is consumed by the online PCT map. SCAN and the 2-D explorer
continue receiving their existing main cloud. Only real hit points form G/C.

The extra sensor is an optical simulation frame without added mass, collision
or rendered housing. A trial housing generated 72 nearby self returns in a
sample main-LiDAR frame and blocked SCAN; removing that visualization avoids
introducing this artificial obstacle. This experiment does not model payload
dynamics or provide a physical-device driver. A real installation needs its
calibrated extrinsics and appropriate robot self-return handling.

## Checks

The isolated `exploration_pct_cloud_sources` test verifies that a closed gap
stays unknown until the additional sensor supplies real returns, that all
published surfaces have measured source=1, and that wrong-frame inputs are
rejected. `exploration_pct_controller` checks existing single-input operation.

Live test artifacts are in
`build/legbot_exploration/Testing/ground_lidar_final_20261106/` and the comparison
summary is `build/legbot_exploration/Testing/ground_lidar_comparison_20261106.json`.
The comparison counts unique mature, measured ground XY cells within 1 m of the
spawn position and ±0.05 m of the actual z=0 floor. Different live runs are not
identical sensor replays; these counts demonstrate coverage, not a controlled
benchmark of exploration speed.

### Historical 2026-11-06 result before inference removal

The following measurements precede removal of plane inference. They are not
acceptance results for the current real-only surface map.

The final two-LiDAR run lasted 130 seconds, with 17 successful execution receipts,
zero terminal execution failures, three usable normal layer-planning results and
one initial SCAN delegation. Maximum horizontal displacement from spawn was
4.42 m; maximum recorded body tilt was 7.77 degrees. This is a bounded single-run
check, not evidence that multi-floor exploration is complete.

Within the first 30 seconds of recorded snapshots, mature measured ground cells
within 1 m of spawn increased from zero in the earlier main-LiDAR-only run to
199 with the second sensor. The final published snapshot contained 313 such XY
cells. All 20 primary and 27 auxiliary integrated clouds in the last map status
had matched origins. Many frames were still dropped during map reads/building;
the additional sensor does not remove that processing bottleneck or under-body
occlusion. Existing inference, maturity, safety margins and 0.10 m resolution
were left in place.

### Real-only surfaces after removal, 2026-11-06

A new 100-second run with `enable_ground_lidar:=true` recorded 16 successful
execution receipts, no terminal execution failures, and three usable normal
layer-planning results. Every sampled surface had source=1 (981,040 records
across snapshots, including repeated cells/slices). Primary/auxiliary inputs
integrated 299/485 frames. Maximum horizontal displacement was 3.22 m and maximum
recorded body tilt was 11.61 degrees; no fall was reported in this bounded run.

The run ended in `recovery_blocked` / `pct_unsafe_start`. The last diagnostic
reported measured G=-0.00458 m and clearance=0.01073 m at the current cell,
classified `solid_or_steep_surface`. Removing inference therefore does not
resolve real-hit G/C classification near a slice boundary. No merging or
safety relaxation was introduced in this cleanup.

Artifacts: `build/legbot_exploration/Testing/real_surfaces_only_20261106/`
(`telemetry.jsonl`, `analysis.json`, point snapshots and `launch.log`).
ROS launch and its residual Gazebo server/gui were stopped after the run.

Build passed. All 15 current CTest cases passed in separate groups, including
core mapping/native A*, GPU G/C comparison, map-only operation, two real cloud
inputs, normal control, completion, recovery and execution feedback. The normal
control fixture now supplies three real samples per cell before expecting
ordinary planning; it no longer depends on removed surface filling.

## Shared static surfaces

The current map uses shared ordered height records and a running scalar mean,
with three points to maturity and a current YAML trial tolerance of 0.04 m
(the earlier runs below used 0.02 m). Earlier live
results above precede this change. Current build, regression and recorded-cloud
replay results are in [shared-surface validation](shared_surface_validation_20261106.md).

## Near-range filtering using shared geometry

Both simulation cloud transformers load the same
`legbot_bringup/config/robot_geometry.yaml` as exploration and online mapping.
`robot_radius` is required at transformer startup (file value 0.35 m).
`lidar_min_range` independently controls point clipping (current trial value 0.70 m).
When omitted it falls back to robot_radius; no safety margin is added.

Filtering uses finite XYZ and three-dimensional distance from each sensor's own
origin, before TF conversion. Points strictly closer than lidar_min_range are
removed. Raw topics are unchanged; all fields of retained points and cloud/pose
timestamps are preserved. Registered clouds are compact, including valid empty
messages when every point is removed. This is distance clipping, not a full
robot mesh/leg self-filter.

For the auxiliary Gazebo sensor only, `lidar_edge_columns: 1` removes the first
and last horizontal columns of each organized scan before TF/compaction.
The setting is scoped to `/ground_lidar_transform` in the shared YAML; other
transformers default to zero. It removes 128 of 16384 scan positions (0.78%).
Unorganized inputs have no scan-column topology and are not edge-clipped.
A front-ground trial found 21 below-floor outliers in one moving frame, all
in the leftmost scan column. These became frozen negative-G surfaces and false
low-clearance obstacles. Edge clipping addresses that sampled input artifact;
the immature-candidate commit issue remains separate.

The corrected front-ground trial recorded about 17.82 m of travel in 150 s.
Runtime TF/parameters and measured scan angles matched the new mount/FOV.
In 70 raw samples, 347 below-floor outliers were all in column zero; none
remained in 70 filtered samples. Previously inspected false negative-G/C pairs
did not recur after rebuilding. Median estimated post-startup ground-point
loss from range and edge clipping was 0.66%; 35 filtered mesh checks found no
calf/foot returns. This is a bounded test, not complete building exploration.
Artifacts: `build/legbot_exploration/Testing/front_ground_edges_20261107/`.

The shared geometry file was moved from exploration to bringup so simulation
and exploration use one configuration without a reverse package dependency.
Changing it takes effect after restart. Previously accumulated static surfaces
are not retroactively removed by input filtering; restart to rebuild the map.

Live inspection on 2026-11-07 confirmed residual calf returns beyond 0.35 m:
62 filtered points were within 2 mm of the current calf visual meshes, with
sensor distances up to 0.4985 m. A 0.50 m cutoff removed them in the recorded
pose, at the cost of about 15% of the downward scan's ground samples. This
single pose does not establish coverage over the whole walking gait.
Artifacts: `build/legbot_exploration/Testing/gray_cells_20261107/`.
