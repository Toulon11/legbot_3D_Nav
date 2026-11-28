"""Isolated native-PCT ROS controller test; SCAN receipts are simulated.

A deliberately full 2-D map and missing SCAN occupancy must not block pure PCT.
This checks target origin/height and feedback routing, not Gazebo locomotion.
"""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def main():
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import TransformStamped
    from control_msgs.msg import DynamicJointState, InterfaceValue
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry,Path as PathMessage
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header,String,Bool
    from scan_planner.msg import GoalRequest
    from visualization_msgs.msg import MarkerArray
    from std_srvs.srv import SetBool
    source=Path(__file__).resolve().parents[1]
    os.environ['ROS_DOMAIN_ID']=str(150+os.getpid()%70)
    os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='pct_controller_ros_')
    publication_load_test='--publication-load' in sys.argv
    flat_escape_test='--flat-support-escape' in sys.argv
    home_retry_test='--home-retry' in sys.argv
    home_progress_test='--home-progress' in sys.argv
    home_mission_test='--home-mission' in sys.argv or home_retry_test or home_progress_test
    completion_test='--completion' in sys.argv
    occupied_recovery_test='--recovery-occupied' in sys.argv
    recovery_test='--recovery' in sys.argv or occupied_recovery_test
    global_test='--local-global' in sys.argv
    pending_test='--local-pending' in sys.argv
    cross_test='--cross-layer' in sys.argv
    floor_deferred_test='--floor-deferred' in sys.argv
    stationary_frontier_test='--stationary-frontier' in sys.argv
    route_resolved_test='--route-resolved' in sys.argv
    floor_blacklist_test='--floor-blacklist' in sys.argv or route_resolved_test or stationary_frontier_test
    low_yield_floor_test='--low-yield-floor' in sys.argv
    expanded_global_first_test='--floor-global-first-after-local' in sys.argv
    ordinary_floor_trial='--ordinary-floor-trial' in sys.argv or low_yield_floor_test or expanded_global_first_test
    bounded_review_test='--bounded-floor-review' in sys.argv
    floor_no_observation_test='--floor-no-observation' in sys.argv or ordinary_floor_trial or bounded_review_test
    floor_disconnected_test='--floor-disconnected' in sys.argv
    floor_blind_test='--floor-blind-observers' in sys.argv
    flat_gap_test='--flat-gap-scan' in sys.argv
    floor_unavailable_test=floor_no_observation_test or floor_disconnected_test or floor_blind_test
    partial_stair_reverse='--stair-reverse-partial' in sys.argv
    stair_reverse_test='--stair-reverse' in sys.argv or partial_stair_reverse
    stair_scout_test='--stair-frontier-scout' in sys.argv
    stair_priority_test='--stair-priority' in sys.argv
    local_stair_probe_test='--local-stair-probe' in sys.argv
    floor_ascent_handoff_test='--floor-ascent-handoff' in sys.argv
    floor_descent_arrival_test='--floor-descent-arrival' in sys.argv
    startup_low_body_test='--startup-low-body' in sys.argv
    stair_start_refresh_test='--stair-start-refresh' in sys.argv
    multilevel_support_test='--multilevel-support' in sys.argv
    ascent_superseded_test='--ascent-superseded' in sys.argv
    navigation_yield_test='--navigation-yield' in sys.argv or ascent_superseded_test
    stair_gap_test='--stair-gap-scan' in sys.argv or navigation_yield_test
    platform_shift_test='--landing-shift' in sys.argv
    short_floor_test='--short-floor' in sys.argv or platform_shift_test
    floor_skip_test=('--floor-skip' in sys.argv or stair_reverse_test or stair_scout_test or
                     stair_priority_test or stair_gap_test or short_floor_test or floor_ascent_handoff_test or stair_start_refresh_test)
    floor_exploration_test='--floor-exploration' in sys.argv
    fast_floor_test='--fast-floor' in sys.argv
    fast_floor_observation_test='--fast-floor-observation' in sys.argv
    floor_test=(flat_escape_test or home_mission_test or floor_exploration_test or floor_descent_arrival_test or floor_deferred_test or floor_blacklist_test or
                floor_unavailable_test or floor_skip_test or fast_floor_test or fast_floor_observation_test or flat_gap_test or local_stair_probe_test or startup_low_body_test or multilevel_support_test)
    observation_timeout_test='--observation-timeout' in sys.argv
    observation_resolved_test='--observation-resolved' in sys.argv
    observation_test='--observation' in sys.argv or observation_timeout_test or observation_resolved_test
    command=[sys.argv[1],'--exploration-mode=combined','--ros-args']
    for name in ('exploration.yaml','multi_layer_map.yaml'):
        command+=['--params-file',str(source/'config'/name)]
    command+=['--params-file',str(source.parent/'legbot_bringup/config/robot_geometry.yaml')]
    # Preconstructed accumulated-map fixture, not a sensor-range test.
    for item in ('online_layer_map:pct_reliable_observation_range:='+('12.0' if stair_gap_test else '0.0'),
                 'use_sim_time:=false','robot_reference_height:=0.38','publish_period:=0.3',
                 'legbot_explorer:frontier_source:=pct','legbot_explorer:planner_mode:=layer',
                 'legbot_explorer:pct_floor_exploration:='+('true' if floor_test else 'false'),
                 'legbot_explorer:observation_only:='+('true' if completion_test or floor_descent_arrival_test else 'false'),
                 'legbot_explorer:no_frontier_hold:=0.2','legbot_explorer:max_voxels:=1',
                 'legbot_explorer:require_planner_occupancy:=true'):
        command+=['-p',item]
    if publication_load_test:
        command+=['-p','publish_period:=0.1','-p','full_snapshot_period:=0.35',
                  '-p','online_layer_map:graph_preview_enabled:=false']
    if home_mission_test:
        command+=['-p','legbot_explorer:mission_return_home:=true',
                  '-p','legbot_explorer:mission_target_height:=0.0',
                  '-p','legbot_explorer:mission_home:=[-1.0,0.0,0.0]']
        if home_retry_test:
            command+=['-p','online_layer_map:pct_planning_budget_seconds:=0.000000001']
    if observation_test:
        command+=['-p','online_layer_map:graph_preview_enabled:=false','-p',
                  'legbot_explorer:odom_timeout:='+('0.25' if observation_timeout_test else '2.0')]
    if observation_timeout_test:
        command+=['-p','legbot_explorer:observation_pose_wait_timeout:=0.8',
                  '-p','legbot_explorer:observation_task_timeout:=8.0',
                  '-p','legbot_explorer:failed_frontier_cooldown:=0.2']
    if global_test:
        command+=['-p','legbot_explorer:local_search_radius:=0.2',
                  '-p','legbot_explorer:pct_local_height_above:=0.3',
                  '-p','legbot_explorer:pct_local_height_below:=0.3']
    if pending_test:
        command+=['-p','online_layer_map:pct_planning_budget_seconds:=0.000000001']
    if cross_test:
        command+=['-p','online_layer_map:pct_surface_noise_tolerance:=0.05',
                  '-p','online_layer_map:pct_maturity_min_points:=1',
                  '-p','legbot_explorer:local_search_radius:=3.0',
                  '-p','legbot_explorer:observation_frame_wait:=0.3',
                  '-p','legbot_explorer:observation_full_sweep_fallback:=true']
    if floor_test:
        command+=['-p','online_layer_map:graph_preview_enabled:=false',
                  '-p','legbot_explorer:observation_frame_wait:=0.2',
                  '-p','online_layer_map:pct_fast_exploration:='+
                  str(fast_floor_test or fast_floor_observation_test).lower()]
    if navigation_yield_test:
        command+=['-p','online_layer_map:pct_ascent_lookahead_slices:=5']
        if ascent_superseded_test:command+=['-p','online_layer_map:pct_landing_min_area:=100.0']
    if ordinary_floor_trial:
        command+=['-p','online_layer_map:pct_floor_global_review_enabled:=false']
    elif not bounded_review_test and (floor_deferred_test or floor_blacklist_test or floor_unavailable_test or
                                      flat_gap_test or fast_floor_test or fast_floor_observation_test):
        command+=['-p','online_layer_map:pct_floor_global_review_enabled:=true',
                  '-p','online_layer_map:pct_floor_four_direction_search:=false',
                  '-p','online_layer_map:pct_floor_failed_small_cluster_max_cells:=0',
                  '-p','legbot_explorer:pct_floor_review_wait_budget_seconds:=0.0']
    if bounded_review_test or (ordinary_floor_trial and not (low_yield_floor_test or expanded_global_first_test)):
        # Exercise legacy configurable review behavior independently of the
        # new default three-member first-pass deferral policy.
        command+=['-p','online_layer_map:pct_fast_min_visible_cells:=1']
    if bounded_review_test:
        command+=['-p','online_layer_map:pct_floor_global_review_enabled:=true']
    if low_yield_floor_test or expanded_global_first_test:
        command+=['-p','online_layer_map:pct_fast_exploration:=true']
    if stair_reverse_test:
        # This recovery fixture deliberately uses one-metre-deep treads.
        # Keep them below its platform threshold so recovery is exercised
        # before the genuinely larger terminal platform.
        command+=['-p','online_layer_map:pct_landing_min_depth:=1.2']
    if floor_skip_test:
        command+=['-p','legbot_explorer:skip_start_floor_exploration:=true']
    if local_stair_probe_test:
        command+=['-p','legbot_explorer:observation_frame_wait:=0.6']
    if floor_deferred_test:
        command+=['-p','online_layer_map:pct_planning_budget_seconds:=0.000000001']
    if floor_no_observation_test or fast_floor_test:
        command+=['-p','online_layer_map:ground_lidar_max_range:=0.1']
    if floor_no_observation_test:
        command+=['-p','legbot_explorer:no_frontier_hold:=60.0']
    with tempfile.TemporaryFile(mode='w+') as log:
        process=subprocess.Popen(command,stdout=log,stderr=log)
        rclpy.init();node=rclpy.create_node('pct_controller_test')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        states=[];goals=[];plans=[];waypoints=[];executions=[];markers=[];routes=[]
        surfaces=[];map_states=[];activity=[];observation_events=[];surface_updates=[]
        def subscribe(kind,topic,storage,profile=qos,decode=False):
            node.create_subscription(kind,topic,lambda m:storage.append(json.loads(m.data) if decode else m),profile)
        subscribe(String,'/exploration/status',states,decode=True)
        subscribe(Bool,'/exploration/task_active',activity)
        subscribe(String,'/exploration/observation_feedback',observation_events,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL),decode=True)
        subscribe(String,'/exploration/planning_goal',goals,decode=True)
        subscribe(String,'/exploration/planning_results',plans,decode=True)
        subscribe(String,'/exploration/execution_feedback',executions,decode=True)
        requests=[]
        def received_request(message):
            requests.append(message);waypoints.append(message.target)
        node.create_subscription(GoalRequest,'/scan/waypoint_request',received_request,10)
        subscribe(MarkerArray,'/exploration/pct/frontiers',markers)
        subscribe(PathMessage,'/exploration/planned_path',routes)
        subscribe(PointCloud2,'/exploration/layers/planning_surfaces',surfaces)
        subscribe(String,'/exploration/layers/status',map_states,decode=True)
        subscribe(String,'/exploration/layers/surface_updates',surface_updates,decode=True)
        base=node.create_publisher(Odometry,'/slam/global_odometry_base',10)
        sensor=node.create_publisher(Odometry,'/slam/odometry_lidar',10)
        cloud=node.create_publisher(PointCloud2,'/slam/global_cloud_registered',10)
        ready=node.create_publisher(Bool,'/go2/demo_ready',qos)
        if floor_test:
            feet=('FL_foot','FR_foot','RL_foot','RR_foot')
            foot_forces=node.create_publisher(DynamicJointState,'/dynamic_joint_states',10)
            foot_tf=node.create_publisher(TFMessage,'/tf',10)
        forwarded=node.create_publisher(GoalRequest,'/scan/goal_request',10)
        def forward(pose):
            nonlocal base_yaw
            if floor_test:
                dx,dy=pose.pose.position.x-base_position[0],pose.pose.position.y-base_position[1]
                if math.hypot(dx,dy)>.01:
                    base_yaw=math.atan2(dy,dx)
            request=next(r for r in requests if r.target.header.stamp==pose.header.stamp)
            forwarded.publish(request)
        ack=node.create_publisher(PoseStamped,'/scan/goal_stop_ack',10)
        complete=node.create_publisher(Bool,'/scan/mission_complete',qos)
        failure=node.create_publisher(String,'/scan/goal_plan_failed',10)
        if floor_test:
            raw=node.create_publisher(PointCloud2,'/local_planner/grid_map/occupancy',10)
            shifts=[]
            subscribe(String,'/scan/recovery_reverse_request',shifts,10)
            shift_result=node.create_publisher(String,'/scan/recovery_reverse_result',10)
        if floor_test:
            inflated=node.create_publisher(PointCloud2,'/local_planner/grid_map/occupancy_inflate',10)
            blocked=[]
        points=[(i*.05,j*.05,z) for i in range(-40,40) for j in range(-40,40) for z in (0.,1.5,3.)]
        if not recovery_test and not completion_test:
            # This case exercises normal planning from a fully observed map.
            # Supply three real samples even to cells receiving one grid point;
            # startup maturity gaps must not rely on removed surface filling.
            points=[(i*.1,j*.1,z) for i in range(-20,21) for j in range(-20,21)
                    for z in (0.,1.5,3.)]*3
        if recovery_test:
            points=[(i*.1,j*.1,1.5) for i in range(20,24) for j in range(-1,2)]
        if occupied_recovery_test:
            # Parking patches include the full .85 m observed arrival disc.
            points=([(i*.1,j*.1,1.5) for i in range(-10,1) for j in range(-5,6)]+
                    [(-.5,0.,1.74)]+[(i*.1,j*.1,1.5) for i in range(24,47) for j in range(-11,12)])*3
        if cross_test:
            points=[(i*.1,j*.1,0. if i<0 else .10 if i<7 else .20)
                    for i in range(-40,25) for j in range(-40,41)]
            # This is an offline room fixture (no paired sensor rays). Supply
            # actual upper bounds instead of assuming infinite clearance.
            points += [(x,y,z+1.5) for x,y,z in points]
        if completion_test:
            points += [(i*.05,j*.05,k*.05) for i in range(-40,40) for j in (-40,39) for k in range(1,60)]
            points += [(i*.05,j*.05,k*.05) for j in range(-40,40) for i in (-40,39) for k in range(1,60)]
        if floor_test:
            points=[(i*.1,j*.1,z) for i in range(-21,22) for j in range(-21,22)
                    for z in (0.,1.5)]*3
            if stair_gap_test:
                points=[(i*.1,j*.1,0.) for i in range(-20,8) for j in range(-18,19)]*3
            if local_stair_probe_test:
                points=([(i*.1,j*.1,0.) for i in range(-18,12) for j in range(-14,15)]+
                        [(i*.1,j*.1,.13) for i in range(12,18) for j in range(-9,10)])*3
            if floor_ascent_handoff_test:
                points=([(i*.1,j*.1,0.) for i in range(-18,10) for j in range(-14,15)]+
                        [(i*.1,j*.1,.13) for i in range(10,34) for j in range(-12,13)])*3
                points += [(x,y,z+1.5) for x,y,z in points]
            if startup_low_body_test:
                # Real ground is observed outside the laser's near blind area.
                # The footprint stays unknown even after four feet settle.
                points=[(i*.1,j*.1,0.) for i in range(-35,36) for j in range(-25,26)
                        if math.hypot(i*.1,j*.1)>=.8]*3
            if stair_start_refresh_test:
                points=[(i*.1,j*.1,z) for i in range(-25,26) for j in range(-18,19)
                        for z in (0.,1.5) if z>0 or math.hypot(i*.1,j*.1)>.65]*3
            if stair_priority_test:
                points=[(i*.05,j*.05,0.) for i in range(-60,0) for j in range(-62,63)]
                points.extend((i*.05,j*.05,z) for start,end,lo,hi,z in
                              ((0,10,-23,1,.06),(0,10,31,55,.06),(10,20,31,55,.12))
                              for i in range(start,end) for j in range(lo,hi))
            # Put a known low wall just beyond the measured floor only after
            # checking the open-floor behavior. The upper boundary stays unknown.
            closed_lower=points+[(i*.1,j*.1,.30) for i in range(-22,23) for j in range(-22,23)
                                 if abs(i)==22 or abs(j)==22]*3
            if route_resolved_test or stationary_frontier_test:
                closed_lower=points+[(i*.1,j*.1,z*.1) for i in range(-22,23) for j in range(-22,23)
                    if abs(i)==22 or abs(j)==22 for z in range(16)]*3
            if home_mission_test:
                points=points+[(i*.1,j*.1,z*.1) for i in range(-22,23) for j in range(-22,23)
                               if abs(i)==22 or abs(j)==22 for z in range(16)]*3
                if home_progress_test:
                    points=[p for p in points if not(abs(p[0]+1.)<.001 and abs(p[1])<.001 and abs(p[2])<.001)]
            if flat_escape_test:
                points=closed_lower+[(.3,j*.1,z*.1) for j in range(-10,11) for z in range(1,15)]*3
                blocked[:]=[(.3,j*.1,.45) for j in range(-10,11)]
            if floor_deferred_test:
                points=[p for p in closed_lower if p!=(1.3,0.,0.)]
            if floor_blacklist_test or fast_floor_observation_test:
                missing=range(-4,5) if fast_floor_observation_test else range(-1,3)
                points=[p for p in closed_lower if p not in {(1.3,j*.1,0.) for j in missing}]
            if floor_no_observation_test:
                holes={(1.3,0.,0.),(-1.3,0.,0.),(0.,1.3,0.),(0.,-1.3,0.),(1.3,1.3,0.),(-1.3,-1.3,0.)}
                if expanded_global_first_test:
                    holes={(1.3,j*.1,0.) for j in range(-2,3)}
                points=[p for p in closed_lower if p not in holes]
            if floor_blind_test:
                # The actual start has a full body footprint. Every distant
                # observer's larger near-blind disc extends outside this patch.
                points=[(i*.1,j*.1,0.) for i in range(-10,11) for j in range(-10,11)]*3
            if fast_floor_test:
                # More than one observer-query batch. The handoff must remain
                # ready when the next global pass yields between clusters.
                holes={(i*.1,j*.1,0.) for i in (-13,13) for center in (-12,0,12)
                       for j in range(center-2,center+3)}
                points=[p for p in closed_lower if p not in holes]
            if flat_gap_test:
                # The upper sheet spans a gap in the current floor. The
                # near edge has safe views but the far interior hole does not.
                points=[(i*.1,j*.1,1.5) for i in range(-23,60) for j in range(-19,20)]
                points.extend((i*.1,j*.1,0.) for i in range(-22,59) for j in range(-18,19)
                              if i!=11 and (i,j)!=(42,0))
                points.extend((i*.1,j*.1,.3) for i in range(-23,60) for j in range(-19,20)
                              if i in (-23,59) or abs(j)==19)
                points*=3
            if floor_disconnected_test:
                # The projected connectivity check sees an upper sheet
                # bridging the rooms. Same-floor A* cannot take that shortcut.
                holes={(4.2,0.,0.)}
                points=[(i*.1,j*.1,1.5) for i in range(-23,60) for j in range(-19,20)]
                for lo,hi in ((-22,19),(27,58)):
                    points.extend((i*.1,j*.1,0.) for i in range(lo,hi+1) for j in range(-18,19) if (i,j)!=(42,0))
                    points.extend((i*.1,j*.1,.3) for i in range(lo-1,hi+2) for j in range(-19,20)
                                  if i in (lo-1,hi+1) or abs(j)==19)
                points*=3
        next_send=0
        base_yaw=0.
        pending_turns=[]
        observed_turns=[]
        observed_stair_turns=[]
        canceled_turns=[]
        node.create_subscription(String,'/scan/recovery_action_cancel',canceled_turns.append,10)
        turn_results=node.create_publisher(String,'/scan/recovery_turn_result',10)
        def turn_requested(message):
            nonlocal base_yaw
            stamp,angle=message.data.split('|')
            observed_turns.append((stamp,float(angle)))
            if states and states[-1]['control_mode']=='cross_layer':
                observed_stair_turns.append((stamp,float(angle)))
            base_yaw+=float(angle)
            if not observation_timeout_test and not observation_resolved_test:
                pending_turns.append((time.monotonic()+.6,stamp))
        node.create_subscription(String,'/scan/recovery_turn_request',turn_requested,10)
        base_position=[-.5,0.,1.91]
        next_cloud=0.
        loaded_feet=2
        split_support=False
        split_height=.14
        base_speed=0.
        base_pause_until=0.
        if floor_test:base_position[:]=[0.,0.,.38]
        if route_resolved_test:base_position[:]=[-1.3,0.,.38]
        if floor_descent_arrival_test:base_position[:]=[0.,0.,1.86]
        if startup_low_body_test:base_position[:]=[0.,0.,.084]
        if stair_start_refresh_test:base_position[:]=[-1.5,0.,.38]
        if stair_priority_test:base_position[:]=[-1.6,0.,.38]
        if cross_test:base_position[:]=[-.4,0.,.38]
        def spin_until(condition,timeout=15):
            nonlocal next_send,next_cloud
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                if process.poll() is not None:
                    raise AssertionError('combined process exited '+str(process.returncode))
                t=time.monotonic()
                for at,action_stamp in list(pending_turns):
                    if t>=at:
                        turn_results.publish(String(data=action_stamp+'|succeeded'))
                        pending_turns.remove((at,action_stamp))
                if t>=next_send:
                    stamp=node.get_clock().now().to_msg()
                    a=Odometry();a.header=Header(stamp=stamp,frame_id='map')
                    a.pose.pose.position.x=base_position[0];a.pose.pose.position.y=base_position[1];a.pose.pose.position.z=base_position[2];a.pose.pose.orientation.w=__import__("math").cos(base_yaw/2);a.pose.pose.orientation.z=__import__("math").sin(base_yaw/2)
                    a.twist.twist.linear.x=base_speed
                    if flat_escape_test:a.twist.twist.angular.y=.4
                    b=Odometry();b.header=a.header;b.pose.pose.position.x=base_position[0];b.pose.pose.position.y=base_position[1];b.pose.pose.position.z=base_position[2]+.39
                    b.pose.pose.orientation.w=1.
                    if floor_test:
                        a.child_frame_id='base'
                        loads=[40. if i<loaded_feet else 0. for i in range(4)]
                        foot_forces.publish(DynamicJointState(header=a.header,joint_names=['foot_force'],
                            interface_values=[InterfaceValue(interface_names=[f+'_force' for f in feet],values=loads)]))
                        transforms=[]
                        for name in feet:
                            foot=TransformStamped(header=Header(stamp=stamp,frame_id='base'),child_frame_id=name)
                            foot.transform.translation.z=-.36+(split_height if split_support and name.startswith('R') else 0.)
                            if startup_low_body_test:
                                foot.transform.translation.z=.02-base_position[2]
                            foot.transform.rotation.w=1.
                            transforms.append(foot)
                        foot_tf.publish(TFMessage(transforms=transforms))
                    if t>=base_pause_until:base.publish(a)
                    sensor.publish(b);ready.publish(Bool(data=True))
                    rclpy.spin_once(node,timeout_sec=.02)
                    if t>=next_cloud:
                        measured=points
                        if recovery_test or startup_low_body_test:
                            # Offline fixtures need measured upper bounds;
                            # laser range alone no longer initializes C_clear.
                            ground=0. if startup_low_body_test else 1.5
                            measured=points+[(x,y,ground+1.5) for x,y,z in points if abs(z-ground)<.001]
                        cloud.publish(point_cloud2.create_cloud_xyz32(a.header,measured));next_cloud=t+(.01 if publication_load_test else .25)
                        if floor_test:
                            distant=[(8.05,8.05,.45)]
                            raw.publish(point_cloud2.create_cloud_xyz32(a.header,blocked if flat_escape_test else distant))
                            inflated.publish(point_cloud2.create_cloud_xyz32(a.header,blocked or distant))
                    next_send=t+(.01 if publication_load_test else .05 if observation_timeout_test else .08 if floor_test else .25)
                rclpy.spin_once(node,timeout_sec=.03)
                if condition():
                    assert not node.get_publishers_info_by_topic('/go2/control_mode'), 'exploration published gait mode'
                    assert not node.get_publishers_info_by_topic('/cmd_vel'), 'exploration published velocity'
                    assert not node.get_publishers_info_by_topic('/exploration/bootstrap_turn_cmd'), 'legacy turn publisher remains'
                    return
            raise AssertionError('timeout; last states='+str(states[-2:]))
        try:
            if stationary_frontier_test:
                spin_until(lambda:any(s['observation_task_active'] for s in states),15)
                assert not requests,'safe current stance should not publish a navigation waypoint'
                points += [(1.3,j*.1,z) for j in range(-1,3) for z in (0.,1.5)]*3
                spin_until(lambda:bool(observation_events),10)
                assert observation_events[0]['success'],observation_events
                assert not requests,'in-place view unexpectedly became a positional route'
                assert base_position==[0.,0.,.38]
                print('PASS safe current stance starts observation without navigation; real new returns settle the original frontier')
                return 0
            if publication_load_test:
                spin_until(lambda:len(surfaces)>=3 and len(surface_updates)>=8 and
                    any(s.get('capture_from_writer') for s in map_states),20)
                assert all(msg.width>0 and len(msg.data)==64*msg.width for msg in surfaces)
                versions=[s['exported_version'] for s in map_states if s['inspection_valid']]
                assert len(set(versions))>=3,versions
                ready=[s for s in map_states if s['inspection_valid']]
                assert max(s['lock_wait_ms'] for s in ready)<5000.,ready[-1]
                print('PASS continued full clouds and versioned coverage under incoming cloud load; writer handoff used; '+
                      json.dumps(dict(full_clouds=len(surfaces),updates=len(surface_updates),
                          writer_captures=sum(s.get('capture_from_writer',False) for s in ready),
                          maximum_capture_wait_ms=max(s['lock_wait_ms'] for s in ready))))
                return 0
            if route_resolved_test:
                spin_until(lambda:bool(requests) and bool(goals),20)
                goal_id=requests[0].goal_id
                points += [(1.3,j*.1,z) for j in range(-1,3) for z in (0.,1.5)]*3
                spin_until(lambda:any(any(abs(row[0]-1.3)<.01 and abs(row[1])<.01 and abs(row[2])<.01
                    for row in update['coverage']) for update in surface_updates),12)
                until=time.monotonic()+.5
                spin_until(lambda:time.monotonic()>=until,2)
                assert len(requests)==1 and not observed_turns
                stage=0
                def arrived():
                    return bool(stage and requests[stage-1].has_xy_tolerance and any(
                        e['goal_id']==goal_id and e['status']=='succeeded' and
                        e['stage_index']==requests[stage-1].waypoint_index for e in executions))
                while not arrived():
                    spin_until(lambda:len(requests)>stage,5)
                    request=requests[stage];assert request.goal_id==goal_id
                    p=request.target.pose.position;base_position[:]=[p.x,p.y,p.z]
                    forward(request.target)
                    until=time.monotonic()+.3;spin_until(lambda:time.monotonic()>=until,2)
                    ack.publish(request.target);complete.publish(Bool(data=True));stage+=1
                    spin_until(lambda:len(requests)>stage or arrived(),6)
                until=time.monotonic()+.3;spin_until(lambda:time.monotonic()>=until,2)
                assert not observed_turns and not observation_events
                assert not states[-1]['observation_task_active']
                print('PASS resolved frontier keeps accepted route through arrival; no redundant scan or failed observation')
                return 0
            if stair_start_refresh_test:
                spin_until(lambda:states and states[-1]['pct_floor']['number']==1,15)
                base_position[:]=[0.,0.,.38]
                spin_until(lambda:any(s['state']=='pct_stair_start_recovery' for s in states),15)
                count=len(requests)
                points += [(i*.1,j*.1,0.) for i in range(-7,8) for j in range(-7,8)]*3
                spin_until(lambda:any(s['state']=='pct_blocker_start_recovered' and
                    s.get('detail')=='fresh_measured_support' for s in states),15)
                assert len(requests)==count,'passive origin repair invented a recovery motion'
                print('PASS stair start repaired by measured returns exits recovery without a scan or motion')
                return 0
            if flat_escape_test:
                spin_until(lambda:bool(shifts),15)
                action,sx,sy,tx,ty,_,kind=shifts[0].data.split('|')
                assert kind=='shift',shifts[0].data
                assert .20<=math.hypot(float(tx)-float(sx),float(ty)-float(sy))<=.60+1e-6
                assert float(tx)<-.15,shifts[0].data
                assert not requests,'unsafe start was delegated before verified short escape'
                base_position[:2]=[float(tx),float(ty)]
                until=time.monotonic()+.4
                spin_until(lambda:time.monotonic()>=until,2)
                shift_result.publish(String(data=action+'|succeeded'))
                spin_until(lambda:any(s['state']=='pct_blocker_start_recovered' for s in states),8)
                assert any(s.get('detail')=='verified_short_support_escape' for s in states),states[-1]
                print('PASS no-gap unsafe start uses validated short escape and resumes ordinary exploration')
                return 0
            if home_mission_test:
                if home_retry_test:
                    spin_until(lambda:sum(p['reason']=='pct_planning_timeout' for p in plans)>=2,20)
                    failed=[p for p in plans if p['reason']=='pct_planning_timeout']
                    assert failed[0]['attempt_id']!=failed[1]['attempt_id'],failed
                    assert not requests and not states[-1]['mission']['returned_home']
                    assert any(s['state']=='navigation_compute_retry' for s in states),states[-10:]
                    assert not observed_turns and not shifts,'compute timeout alone triggered an observation action'
                    assert not any(s['state'] in ('navigation_target_retired','navigation_start_recovery')
                                   for s in states),states[-10:]
                    print('PASS failed home candidate performs a fresh planner call on retry; no false completion')
                    return 0
                reached_segment=False
                for stage in range(30 if home_progress_test else 12):
                    spin_until(lambda:len(requests)>stage or (states and states[-1]['state']=='mission_complete'),30)
                    if states[-1]['state']=='mission_complete':break
                    request=requests[stage];p=request.target.pose.position
                    planned=next(g for g in goals if g['goal_id']==request.goal_id and g['attempt_id']==request.attempt_id)
                    if not home_progress_test:assert planned['selection_policy']=='pct_return_home',planned
                    assert not states[-1]['mission']['returned_home'],states[-1]
                    partial=next(p for p in plans if p['goal_id']==request.goal_id and
                                 p['attempt_id']==request.attempt_id)['partial']
                    forward(request.target);base_position[:]=[p.x,p.y,p.z]
                    until=time.monotonic()+.4
                    spin_until(lambda:time.monotonic()>=until,2)
                    ack.publish(request.target);complete.publish(Bool(data=True))
                    if home_progress_test and planned['selection_policy']=='pct_return_home' and partial:
                        spin_until(lambda:any(s['state']=='navigation_segment_reached' for s in states),8)
                        assert not states[-1]['mission']['returned_home']
                        assert any(s.get('navigation_segment',{}).get('final_goal_pending') for s in states)
                        assert not math.hypot(base_position[0]+1,base_position[1])<=.2
                        points+=[(-1.,0.,0.)]*3
                        reached_segment=True
                spin_until(lambda:states[-1]['state']=='mission_complete',8)
                result=states[-1]['mission']
                assert result['exploration_finished'] and result['returned_home'],result
                assert math.hypot(base_position[0]+1,base_position[1])<=.2,base_position
                if home_progress_test:assert reached_segment,'no partial route exercised'
                print('PASS '+('safe partial arrival, measured gap closure, original home goal resumed and completed'
                    if home_progress_test else 'target task exhaustion, verified return route, stop receipts and home completion'))
                return 0
            if floor_descent_arrival_test:
                loaded_feet=4
                spin_until(lambda:states and states[-1]['pct_floor']['number']==1,15)
                assert abs(states[-1]['pct_floor']['height']-1.5)<.04,states[-1]
                base_position[2]=.86
                until=time.monotonic()+1.
                spin_until(lambda:time.monotonic()>=until,3)
                assert states[-1]['pct_floor']['number']==1,states[-1]
                base_position[2]=.36
                spin_until(lambda:states[-1]['pct_floor']['number']==2,15)
                arrived=states[-1]
                assert abs(arrived['pct_floor']['height'])<.04,arrived
                floors=arrived['height_review']['frontier_floors']
                assert len(floors)==2 and abs(floors[0]['height'])<.04,floors
                assert abs(floors[1]['height']-1.5)<.04,floors
                assert any(s['state']=='pct_height_band_changed' for s in states),states[-1]
                print('PASS measured lower support changes the scheduling height band')
                return 0
            if startup_low_body_test:
                spin_until(lambda:any(s['state']=='pct_blocker_recovery_waiting' and
                    s.get('detail')=='no_other_safe_delegate_goal' for s in states),20)
                assert not requests,'untrusted support must not authorize a destination'
                geometry=states[-1]['pct_map']['geometry_version']
                loaded_feet=4
                spin_until(lambda:bool(requests),15)
                request=requests[0]
                p=request.target.pose.position
                assert abs(p.z-.38)<.02,(p.x,p.y,p.z)
                assert math.hypot(p.x,p.y)>.8,'unknown footprint was fabricated as safe'
                assert states[-1]['pct_map']['geometry_version']==geometry,states[-1]
                forward(request.target)
                base_position[:]=[p.x,p.y,p.z]
                until=time.monotonic()+.4
                spin_until(lambda:time.monotonic()>=until,2)
                ack.publish(request.target)
                spin_until(lambda:any(s.get('pct_floor',{}).get('number')==1 for s in states),12)
                assert abs(states[-1]['pct_floor']['height'])<.02,states[-1]
                print('PASS crouched startup wakes recovery on stable support without filling the blind footprint')
                return 0
            if floor_test:
                spin_until(lambda:any(s.get('pct_floor',{}).get('number')==1 for s in states),20)
                loaded_feet=4
                if multilevel_support_test:
                    split_support=True
                    split_height=.16
                    spin_until(lambda:bool(states) and
                        states[-1].get('pct_floor',{}).get('foot_sample')=='stable_multilevel',8)
                    support=states[-1]['pct_floor']
                    assert support['support_stable'] and support['stable_foot_height'] is None,support
                    assert abs(support['foot_height_span']-.16)<1e-6,support
                    assert abs(support['height'])<.02,'mixed feet must not synthesize a new floor G'
                    split_height=.17
                    spin_until(lambda:states[-1]['pct_floor']['foot_sample']=='foot_support_exceeds_step_limit',8)
                    assert not states[-1]['pct_floor']['support_stable'],states[-1]
                    split_height=.14
                    spin_until(lambda:states[-1]['pct_floor']['foot_sample']=='stable_multilevel',8)
                    print('PASS 16 cm multilevel support is stable without an averaged ground; 17 cm stays rejected')
                    return 0
                if floor_ascent_handoff_test:
                    for stage in range(16):
                        spin_until(lambda:len(requests)>stage,12)
                        request=requests[stage]
                        p=request.target.pose.position
                        planned=next(g for g in goals if g['goal_id']==request.goal_id and
                                     g['attempt_id']==request.attempt_id)
                        assert planned['selection_policy'] in ('pct_floor_ascent_frontier',
                                                               'pct_upward_support'),planned
                        spin_until(lambda:states[-1]['control_mode']=='cross_layer',5)
                        forward(request.target)
                        base_position[:]=[p.x,p.y,p.z]
                        until=time.monotonic()+.35
                        spin_until(lambda:time.monotonic()>=until,2)
                        if request.has_xy_tolerance and p.z>.48:
                            loaded_feet=2
                        ack.publish(request.target)
                        if request.has_xy_tolerance and p.z>.48:
                            break
                    else:
                        raise AssertionError('no route to the higher measured platform')
                    spin_until(lambda:any(s['state']=='pct_height_band_changed' and
                                         s['pct_floor']['number']>=2 for s in states),14)
                    changed=next(s for s in states if s['state']=='pct_height_band_changed' and
                                 s['pct_floor']['number']>=2)
                    assert abs(changed['pct_floor']['height']-.13)<.03,changed
                    assert changed['control_mode']=='cross_layer',changed
                    assert changed['cross_layer_task']['phase']!='idle',changed
                    assert changed['cross_layer_task']['rise_m']>.08,changed
                    assert changed['pct_floor']['contacting_feet']<=2,changed
                    assert not any(s['state']=='cross_layer_platform_settling' for s in states)
                    before=len(requests)
                    spin_until(lambda:len(requests)>before,14)
                    next_request=requests[before]
                    next_goal=next(g for g in goals if g['goal_id']==next_request.goal_id and
                                   g['attempt_id']==next_request.attempt_id)
                    assert next_goal['selection_policy'] in ('pct_frontier','pct_frontier_fast',
                                                              'pct_frontier_review'),next_goal
                    assert abs(next_request.target.pose.position.z-.51)<.03,next_request
                    spin_until(lambda:states[-1]['control_mode']=='flat' and
                               states[-1]['cross_layer_task']['phase']=='exploring_platform',5)
                    print('PASS height rise preserves ascent context and permits same-height work when upward search is exhausted')
                    return 0
                if local_stair_probe_test:
                    spin_until(lambda:bool(requests) and any(
                        s.get('pct_floor',{}).get('pending_stair_candidates',0)>0 for s in states),20)
                    assert not any(s['control_mode']=='cross_layer' for s in states),states[-1]
                    assert not any(g['selection_policy'] in ('pct_local_stair_probe','pct_local_stair_return',
                                   'pct_stair_approach','pct_stair_climb') for g in goals),goals
                    assert states[-1]['pct_floor']['phase']=='exploring',states[-1]
                    assert not observation_events,'passive hints must not start stair scan actions'
                    print('PASS ascending candidates are retained while ordinary floor exploration continues uninterrupted')
                    return 0
                if navigation_yield_test:
                    # The far upward goal has a real safe surface, separated
                    # by a missing strip. Its partial approach stops on a broad
                    # intermediate platform that still has ordinary work.
                    points += ([(i*.1,j*.1,.13) for i in range(8,30) for j in range(-30,31)] +
                               [(i*.1,j*.1,.52) for i in range(35,72) for j in range(-14,15)])*3
                    points += [(x,y,1.5) for x,y,z in points if z!=1.5]
                    index=0;first=None
                    for _ in range(8):
                        spin_until(lambda:len(requests)>index,15)
                        request=requests[index];index+=1
                        result=next(p for p in plans if p['goal_id']==request.goal_id and p['attempt_id']==request.attempt_id)
                        forward(request.target);complete.publish(Bool(data=False))
                        p=request.target.pose.position;base_position[:]=[p.x,p.y,p.z]
                        until=time.monotonic()+.35;spin_until(lambda:time.monotonic()>=until,2)
                        ack.publish(request.target);complete.publish(Bool(data=True))
                        if result.get('partial'):
                            first=request;break
                    assert first is not None,'no partial platform approach exercised'
                    assert first.target.pose.position.x<3.,first
                    if ascent_superseded_test:
                        # Model a verified recovery stop reached after the accepted
                        # partial route. Pose Z alone must not settle the goal:
                        # fresh downward returns establish its actual upper support.
                        points += [(i*.1,j*.1,z) for i in range(37,64) for j in range(-16,17)
                                   for z in (1.5,3.)]*3
                        base_position[:]=[4.7,0.,1.88]
                        # Height refresh can supersede the transient report in
                        # this same callback; assert the persistent task state.
                        spin_until(lambda:any(s.get('cross_layer_task',{}).get('reason')=='ascent_target_superseded' and
                            not s['navigation_segment']['final_goal_pending'] for s in states),15)
                        settled=next(s for s in states if s.get('cross_layer_task',{}).get('reason')=='ascent_target_superseded' and
                                     not s['navigation_segment']['final_goal_pending'])
                        assert not settled['navigation_segment']['final_goal_pending'],settled
                        assert settled['navigation_segment']['deferred_targets']>=1,settled
                        print('PASS measured higher safe support supersedes stale automatic ascent target')
                        return 0
                    spin_until(lambda:any(s['state']=='navigation_platform_handoff' for s in states),12)
                    spin_until(lambda:any(s.get('pct_floor',{}).get('ordinary_started_wall') is not None
                        and s.get('cross_layer_task',{}).get('phase')=='exploring_platform' for s in states),12)
                    handed=next(s for s in states if s['state']=='navigation_platform_handoff')
                    assert not handed['navigation_segment']['final_goal_pending'],handed
                    assert handed['navigation_segment']['deferred_targets']>=1,handed
                    assert not any(g['selection_policy']=='pct_upward_support' and g['goal_id']>first.goal_id
                                   for g in goals),goals
                    print('PASS partial route finishes, pending ascent yields to confirmed platform exploration')
                    return 0
                if stair_gap_test:
                    # Treads are shallower than the platform exploration
                    # depth, so this isolates ascent recovery.
                    stairs=[(i*.1,j*.1,.13*min(4,1+(i-8)//8)) for i in range(8,72)
                            for j in range(-10,11) if i!=20]
                    # An actually unmeasured strip interrupts the next tread.
                    points += (stairs+[(x,y,1.5) for x,y,_ in stairs])*3
                    index=shift_index=0;filled=False;deadline=time.monotonic()+40.
                    while time.monotonic()<deadline:
                        until=time.monotonic()+.1
                        spin_until(lambda:time.monotonic()>=until,2)
                        recorded_gap=any(any(abs(b['point'][0]-2.)<.05
                            for b in s.get('pct_floor',{}).get('unknown_blockers',[])) for s in states)
                        observing=any(s['state'] in ('cross_layer_support_scan','cross_layer_stair_gap_scan',
                                                    'pct_blocker_scan') for s in states)
                        if recorded_gap and observing and not filled:
                            points += [(1.965+i*.015,-1.+j*.02,.26)
                                       for i in range(6) for j in range(101)]
                            filled=True
                        if shift_index<len(shifts):
                            action,sx,sy,tx,ty,heading,*kind=shifts[shift_index].data.split('|')
                            assert .20<=math.hypot(float(tx)-float(sx),float(ty)-float(sy))<=.60+1e-6
                            base_position[:2]=[float(tx),float(ty)]
                            shift_result.publish(String(data=action+'|succeeded'));shift_index+=1
                        if index>=len(requests):continue
                        request=requests[index];p=request.target.pose.position
                        if not filled:assert p.x<2. and p.z<=.65,'route crossed the unobserved strip'
                        if filled and p.x>2.1:
                            assert any(q['usable'] and q['goal_id']==request.goal_id for q in plans)
                            print('PASS safe partial movement and targeted scan resume ascent after measured gap closure')
                            return 0
                        forward(request.target);complete.publish(Bool(data=False))
                        base_position[:]=[p.x,p.y,p.z]
                        until=time.monotonic()+.35;spin_until(lambda:time.monotonic()>=until,2)
                        ack.publish(request.target);complete.publish(Bool(data=True));index+=1
                    raise AssertionError('no ascent after gap repair; filled='+str(filled)+' state='+str(states[-1])[:2000])
                    return 0
                if fast_floor_test:
                    spin_until(lambda:any(s['state']=='pct_waiting_for_ascent' for s in states),18)
                    done=next(s for s in states if s['state']=='pct_floor_handoff_with_deferred_frontiers')
                    assert done['pct_selection_scope']=='global' and not done['pct_query_pending'],done
                    assert done['pct_fast_skipped_clusters']>=6 and done['pct_fast_skipped_cells']>=30,done
                    assert not requests
                    print('PASS fast mode hands off a floor with unobservable frontiers still on the map')
                    return 0
                if floor_deferred_test:
                    spin_until(lambda:sum(p['reason']=='pct_planning_timeout' for p in plans)>=2,16)
                    assert states[-1]['pct_scoped_frontier_cells']>0,states[-1]
                    spin_until(lambda:any(s['state']=='pct_floor_handoff_with_deferred_frontiers' for s in states),16)
                    assert {g['floor_frontier_attempt'] for g in goals}=={1,2},goals
                    assert len(goals)<=6, 'each attempt must finish after its admitted candidate batch'
                    settled=[s for s in states if s.get('detail')=='floor_route_timeout' and
                             s['state']=='pct_floor_frontier_settled']
                    assert len(settled)==2 and not settled[0]['pct_reviewing'] and settled[1]['pct_reviewing'],settled
                    assert not requests,'a timed out plan must not be sent to SCAN'
                    print('PASS repeated PCT timeouts retire the task after one review')
                    return 0
                if flat_gap_test:
                    handled=0
                    while not any(s['state']=='pct_waiting_for_ascent' for s in states):
                        spin_until(lambda:len(requests)>handled or
                                   any(s['state']=='pct_waiting_for_ascent' for s in states),12)
                        if len(requests)==handled:break
                        near=requests[handled]
                        handled+=1
                        # Local first pass, global first pass after local
                        # failure, then this fixture's enabled legacy review.
                        assert handled<=3 and near.target.pose.position.x<1.1,requests
                        forward(near.target)
                        header=near.target.header.stamp
                        failure.publish(String(data=f'{header.sec*1000000000+header.nanosec}|execution_no_progress'))
                    assert handled and all(g['observation_x']<1.1 for g in goals),goals
                    assert not any(p['reason']=='pct_route_disconnected' for p in plans),plans
                    # The next ascent query overwrites the query-local counters;
                    # verify the ordinary/global result at its own handoff.
                    assert any(s['pct_disconnected_floor_frontier_cells']==1 for s in states),states[-1]
                    ended=next(s['pct_floor'] for s in states if s.get('pct_floor',{}).get('ordinary_ended_sim') is not None)
                    assert ended['ordinary_ended_sim']>=ended['ordinary_started_sim'],ended
                    assert ended['ordinary_ended_wall']>=ended['ordinary_started_wall'],ended
                    print('PASS near-side views run while the disconnected far-side hole is omitted')
                    return 0
                if floor_unavailable_test:
                    if floor_blind_test:
                        spin_until(lambda:any(s['state']=='pct_waiting_for_ascent' for s in states),30)
                        assert not goals and not requests and not shifts,goals
                        assert not any(s['state'].startswith('pct_blocker') for s in states),states[-1]
                        settled=[s for s in states if s['state']=='pct_floor_frontier_settled']
                        assert settled and any(s['pct_reviewing'] for s in settled),settled
                        assert any(not s['pct_reviewing'] for s in settled),settled
                        print('PASS safe ordinary start defers blind observers once and never enters recovery')
                        return 0
                    spin_until(lambda:any(s['state']=='pct_waiting_for_ascent' for s in states),18)
                    assert not goals and not requests and not plans,goals
                    if bounded_review_test:
                        settled=[s for s in states if s['state']=='pct_floor_frontier_settled']
                        first=[s for s in settled if not s['pct_reviewing']]
                        second=[s for s in settled if s['pct_reviewing']]
                        assert first and second and not any(s['pct_failed_small_cells'] for s in first),settled
                        done=next(s for s in states if s['state']=='pct_floor_handoff_with_deferred_frontiers')
                        assert done['pct_failed_small_cells']==len(holes),done
                        assert 0<done['pct_floor_review_wait_ms']<=8000.,done
                        assert not done['pct_pending_review_clusters'],done
                        print('PASS small clusters enter bounded review and are retired only after review')
                        return 0
                    if ordinary_floor_trial:
                        if expanded_global_first_test:
                            settled=[(n,s) for n,s in enumerate(states) if s['state']=='pct_floor_frontier_settled']
                            local=[n for n,s in settled if s['pct_selection_scope']=='local']
                            global_first=[n for n,s in settled if s['pct_selection_scope']=='global' and
                                s.get('pct_global_first_after_local_failure_cells',0)>=len(holes)]
                            handoff=next(n for n,s in enumerate(states) if s['state']=='pct_floor_handoff_with_deferred_frontiers')
                            assert local and global_first and max(local)<min(global_first)<handoff,[(n,s['pct_selection_scope']) for n,s in settled]
                            assert not any(s.get('pct_reviewing') for s in states)
                            print('PASS large local failures receive a global first pass before handoff; review remains disabled')
                            return 0
                        if low_yield_floor_test:
                            assert all(s.get('pct_low_yield_deferred_cells',0)==0 for s in states),states[-1]
                            assert any(s.get('pct_selection_timing_ms',{}).get('frontiers_ms',0)>0
                                       for s in states),states[-1]
                            assert not any(s.get('pct_reviewing') for s in states),states[-1]
                            print('PASS first-pass search runs without size deferral; no-view tasks hand off without review')
                            return 0
                        settled=[s for s in states if s['state']=='pct_floor_frontier_settled']
                        assert settled and all(not s['pct_reviewing'] for s in settled),settled
                        done=next(s for s in states if s['state']=='pct_floor_handoff_with_deferred_frontiers')
                        assert done['pct_failed_small_cells']==len(holes),done
                        assert done['pct_failed_small_clusters']==len(holes),done
                        # Strict per-column cuts can also retain the outer
                        # boundary above the fixture's low perimeter wall.
                        assert done['pct_deferred_floor_frontier_cells']>=len(holes),done
                        assert not done['pct_pending_review_clusters'],done
                        assert done['pct_floor_longest_selection_wait_ms']>0,done
                        print('PASS ordinary floor attempts small clusters once, then hands off without a review')
                        return 0
                    if floor_disconnected_test:
                        done=next(s for s in states if s['state']=='pct_floor_handoff_with_deferred_frontiers')
                        assert done['pct_disconnected_floor_frontier_cells']==len(holes),done
                        assert not done['pct_pending_review_clusters'] and done['pct_observer_visibility_ms']==0,done
                        settled=[s for s in states if s.get('detail')=='floor_disconnected']
                        assert len(settled)==2 and not settled[0]['pct_reviewing'] and settled[1]['pct_reviewing'],settled
                        assert settled[1]['pct_selection_scope']=='global',settled
                        assert settled[0]['pct_disconnected_floor_frontier_cells']==0,settled
                        assert any(s.get('pct_selection_resumes',0)>0 for s in states),states[-1]
                        assert any('frontiers_ms' in s.get('pct_selection_timing_ms',{}) for s in states),states[-1]
                        print('PASS same-floor disconnection skips LOS/A*, defers once, reviews once and hands off')
                        return 0
                    assert not any(s['state']=='pct_floor_frontiers_empty_hold' for s in states),states[-1]
                    settlements=[s for s in states if s.get('detail')=='floor_no_observation']
                    assert any(not s['pct_reviewing'] for s in settlements),settlements
                    assert any(s['pct_reviewing'] for s in settlements),settlements
                    assert all(s['pct_selection_scope']=='global' for s in settlements if s['pct_reviewing']),settlements
                    assert states[-1]['pct_deferred_floor_frontier_cells']==len(holes),states[-1]
                    print('PASS unavailable clusters are reviewed once, removed and hand off to stairs')
                    return 0
                if floor_blacklist_test or fast_floor_observation_test:
                    stage=0
                    expected_outcomes=('first_no_progress','blacklisted')
                    for expected in expected_outcomes:
                        for _ in range(8):
                            spin_until(lambda:len(requests)>stage,9)
                            request=requests[stage]
                            forward(request.target)
                            p=request.target.pose.position
                            base_position[:]=[p.x,p.y,p.z]
                            until=time.monotonic()+.3
                            spin_until(lambda:time.monotonic()>=until,2)
                            ack.publish(request.target)
                            stage+=1
                            if request.has_xy_tolerance:break
                        def outcome(e):
                            return e.get('settlement',e.get('evidence',{})).get('floor_attempt')
                        spin_until(lambda:any(outcome(e)==expected for e in observation_events),9)
                    if fast_floor_observation_test:
                        spin_until(lambda:any(s['state']=='pct_floor_handoff_with_deferred_frontiers' for s in states),12)
                        done=next(s for s in states if s['state']=='pct_floor_handoff_with_deferred_frontiers')
                        assert done['pct_fast_skipped_clusters']>=1 and done['pct_fast_skipped_cells']>=5,done
                        assert {g['floor_frontier_attempt'] for g in goals}=={1,2},goals
                        assert len({g['goal_id'] for g in goals})==2,goals
                        assert any(e['reason']=='sweep_executed' and e['views_completed']==1
                                   for e in observation_events),observation_events
                        print('PASS fast floor reviews one failed observation before handoff')
                        return 0
                    spin_until(lambda:any(s['state']=='pct_floor_handoff_with_deferred_frontiers'
                                          for s in states),12)
                    done=next(s for s in states if s['state']=='pct_floor_handoff_with_deferred_frontiers')
                    assert done['pct_selection_scope']=='global' and done['pct_scoped_frontier_cells']==0,done
                    assert done['pct_deferred_floor_frontier_cells']==4,done
                    assert done['pct_pending_review_clusters']==0,done
                    assert goals[0]['selection_policy']=='pct_frontier',goals
                    reviews=[goal for goal in goals if goal['selection_policy']=='pct_frontier_review']
                    assert len(reviews)==1 and reviews[0]['selection_scope']=='global',goals
                    assert any(s['pct_reviewing'] and s['pct_pending_review_clusters']==1 for s in states),states[-1]
                    print('PASS two completed no-progress views blacklist the cluster and allow stair handoff')
                    return 0
                if floor_skip_test:
                    if stair_priority_test:
                        spin_until(lambda:bool(requests),12)
                        sent=requests[0]
                        view=next(g for g in goals if g['goal_id']==sent.goal_id and
                                  g['attempt_id']==sent.attempt_id)
                        assert view['selection_policy']=='pct_local_stair_probe',goals
                        def selected_diagnostic():
                            return next((s['observation_evidence']['diagnostic'] for s in reversed(states)
                                if s.get('goal_id')==sent.goal_id and
                                'diagnostic' in s.get('observation_evidence',{})),None)
                        spin_until(lambda:selected_diagnostic() is not None,3)
                        flight=selected_diagnostic()
                        assert len(flight['treads'])==2 and flight['treads'][0][1]>1.5,flight
                        assert abs(view['observation_z']-.38)<.02,goals
                        print('PASS farther two-tread entrance is scouted before a nearby one-tread entrance')
                        return 0
                    # Skip mode shares ordinary exploration until a usable staircase
                    # is recognized. It must not wait in an independent entry queue.
                    assert states[-1]['pct_floor']['start_floor_skipped'],states[-1]
                    assert not any(s['state'].startswith('pct_floor_complete') for s in states)
                    if stair_scout_test:
                        points=points+[(i*.025,j*.025,.10)
                            for i in range(96,120) for j in range(-40,41)]
                        points.extend((i*.1,j*.1,0.) for i in (22,23) for j in range(-10,11))
                        upper_added=False
                        probe_events=lambda:[e for e in observation_events
                            if e['reason']=='floor_stair_probe_observed']
                        for index in range(16):
                            spin_until(lambda:len(requests)>index or bool(probe_events()),12)
                            if probe_events():break
                            request=requests[index]
                            if not upper_added and any(g['goal_id']==request.goal_id and
                                    g['selection_policy']=='pct_local_stair_probe' for g in goals):
                                points.extend((i*.025,j*.025,.23)
                                    for i in range(120,161) for j in range(-40,41))
                                upper_added=True
                            forward(request.target)
                            p=request.target.pose.position
                            assert p.z<=.65,p
                            base_position[:]=[p.x,p.y,p.z]
                            until=time.monotonic()+.3
                            spin_until(lambda:time.monotonic()>=until,2)
                            ack.publish(request.target)
                        spin_until(lambda:bool(probe_events()),10)
                        event=probe_events()[0]
                        assert event['success'] and event['views_completed']<=1,event
                        assert any(g['selection_policy']=='pct_local_stair_probe' for g in goals),goals
                        assert any(s['pct_floor']['local_stair_probe_active'] for s in states),states[-1]
                        assert not any(s['pct_floor']['phase']=='ascending' for s in states if 'pct_floor' in s)
                        print('PASS skip starts ordinary exploration and uses shared single-tread recognition')
                        return 0
                else:
                    spin_until(lambda:bool(requests),15)
                    assert abs(states[-1]['pct_floor']['height'])<.01
                    assert all(abs(g['observation_z']-.38)<.08 for g in goals),goals
                    assert not any(s['state']=='pct_floor_complete' for s in states)
                    # Restart ordinary selection after this fixture's active task.
                    pause=node.create_client(SetBool,'/exploration/set_paused')
                    assert pause.wait_for_service(timeout_sec=2.)
                    fut=pause.call_async(SetBool.Request(data=True))
                    spin_until(fut.done,5)
                    points=closed_lower
                    forward(requests[0].target)
                    until=time.monotonic()+.4
                    spin_until(lambda:time.monotonic()>=until,2)
                    stamp=requests[0].target.header.stamp
                    failure.publish(String(data=f'{stamp.sec*1000000000+stamp.nanosec}|execution_no_progress'))
                    spin_until(lambda:any(e['status']=='failed' for e in executions),5)
                    fut=pause.call_async(SetBool.Request(data=False))
                    spin_until(fut.done,5)
                    spin_until(lambda:any(s['state']=='pct_floor_complete' for s in states),20)
                    done=next(s for s in states if s['state']=='pct_floor_complete')
                    assert done['pct_selection_scope']=='global' and done['pct_frontier_cells']>0,done
                first_after_floor=0 if floor_skip_test else len(requests)
                tread_depth,steps=(10,4) if stair_reverse_test else (6,7 if short_floor_test else 9)
                upper_ground=.13*steps
                upper_stop_z=.38+upper_ground-.07
                tread_half_width=15 if stair_reverse_test else 10
                full_stairs=(points if floor_skip_test else closed_lower)+[(i*.1,j*.1,.13*min(steps,1+(i-8)//tread_depth))
                                         for i in range(8,85)
                                         for j in (range(-tread_half_width,tread_half_width+1)
                                                   if i<56 else range(-16,17))]*3
                # Withhold the upper flight until a bounded scan has finished
                # at a supported lower tread, then simulate newly fused returns.
                points=full_stairs if stair_reverse_test else [p for p in full_stairs if p[0]<3.9]
                spin_until(lambda:len(requests)>first_after_floor,14)
                # A same-floor entrance survey may be issued before the full
                # synthetic stair cloud arrives. Feed its stop like any other waypoint.
                assert states[-1]['pct_floor']['number']==1,states[-1]
                support_scan=False
                handled_survey_moves=0
                for index in range(first_after_floor,first_after_floor+24):
                    spin_until(lambda:len(requests)>index,12)
                    request=requests[index]
                    forward(request.target)
                    complete.publish(Bool(data=False))
                    p=request.target.pose.position
                    if stair_reverse_test and base_position[2]>.47 and p.z-base_position[2]>.07:
                        x,y,z=base_position
                        heading=math.atan2(p.y-y,p.x-x)
                        # The body is already facing up the flight while
                        # the failed waypoint can be diagonal on the tread.
                        base_yaw=heading*.5
                        direction=(math.cos(heading),math.sin(heading),0.)
                        blocked.append(tuple((math.floor((v+.07*d)*10)+.5)/10
                                             for v,d in zip((x,y,z),direction)))
                        split_support=True
                        split_height=.17
                        until=time.monotonic()+.5
                        spin_until(lambda:time.monotonic()>=until and
                                   states[-1]['cross_layer_task']['rise_m']>.08,5)
                        stamp=request.target.header.stamp
                        failure.publish(String(data=f'{stamp.sec*1000000000+stamp.nanosec}|local_replan_exhausted'))
                        spin_until(lambda:any(s['state']=='cross_layer_reposition_waiting_for_stable_feet'
                                              for s in states),5)
                        assert not shifts
                        split_height=.14
                        spin_until(lambda:bool(shifts) or states[-1]['state']=='cross_layer_deferred',10)
                        assert shifts, [(plan['reason'],plan['backend'].get('rear_support_check'),
                                         plan['backend'].get('stop_check')) for plan in plans[-4:]]
                        action,sx,sy,tx,ty,_=shifts[0].data.split('|')
                        assert .20<math.hypot(float(tx)-float(sx),float(ty)-float(sy))<=.40
                        assert any(s['state']=='cross_layer_reposition_pending' for s in states)
                        if partial_stair_reverse:
                            distance=math.hypot(float(tx)-float(sx),float(ty)-float(sy))
                            base_position[:2]=[float(s)+.167*(float(t)-float(s))/distance
                                               for s,t in ((sx,tx),(sy,ty))]
                            loaded_feet=3
                        else:
                            base_position[:2]=[float(tx),float(ty)]
                        blocked.clear()
                        until=time.monotonic()+.4
                        spin_until(lambda:time.monotonic()>=until,2)
                        if partial_stair_reverse:
                            failure.publish(String(data=action+'|reverse_no_progress'))
                            spin_until(lambda:any(e['status']=='failed' and e['reason']=='reverse_no_progress'
                                                  for e in executions),5)
                            until=time.monotonic()+.6
                            spin_until(lambda:time.monotonic()>=until,2)
                            assert len(requests)==index+1, 'must wait for four stable feet'
                            loaded_feet=4
                            spin_until(lambda:any(s['state']=='cross_layer_start_recovered' for s in states),10)
                        else:
                            shift_result.publish(String(data=action+'|succeeded'))
                        for retry_index in range(index+1,index+5):
                            spin_until(lambda:len(requests)>retry_index,10)
                            retry=requests[retry_index]
                            retry_point=retry.target.pose.position
                            assert retry.goal_id!=request.goal_id
                            forward(retry.target)
                            if retry_point.z>z+.07:
                                retry_stamp=retry.target.header.stamp
                                bx,by,bz=base_position
                                if partial_stair_reverse:
                                    # Both lateral positions are also inflated.
                                    blocked[:]=[(bx+i*.1,by+j*.1,bz) for i in range(-4,5) for j in range(-4,5)]
                                else:
                                    blocked[:]=[(bx+i*.1,by,bz) for i in range(-1,2)]
                                until=time.monotonic()+.3
                                spin_until(lambda:time.monotonic()>=until,2)
                                failure.publish(String(data=f'{retry_stamp.sec*1000000000+retry_stamp.nanosec}|local_replan_exhausted'))
                                if not partial_stair_reverse:
                                    spin_until(lambda:len(shifts)==2,8)
                                    action,sx,sy,tx,ty,_,kind=shifts[1].data.split('|')
                                    assert kind=='shift'
                                    assert .21<math.hypot(float(tx)-float(sx),float(ty)-float(sy))<.23
                                    base_position[:2]=[float(tx),float(ty)]
                                    count=len(requests)
                                    blocked.clear()
                                    shift_result.publish(String(data=action+'|succeeded'))
                                    spin_until(lambda:len(requests)>count,10)
                                    assert any(e['reason']=='reverse_completed' for e in executions)
                                    assert any(e['reason']=='shift_completed' for e in executions)
                                    print('PASS stair reverse -> failed uphill retry -> one verified lateral shift')
                                    return 0
                                spin_until(lambda:states[-1]['state']=='cross_layer_deferred' and
                                           states[-1]['cross_layer_task']['reason']=='stair_replan_failed:local_replan_exhausted',8)
                                assert len(shifts)==1
                                assert states[-1]['pct_floor']['number']==1
                                assert states[-1]['pct_floor']['phase']=='ascent_deferred'
                                count=len(requests)
                                until=time.monotonic()+1.3
                                spin_until(lambda:time.monotonic()>=until,3)
                                assert len(requests)==count and len(shifts)==1
                                blocked.clear()
                                spin_until(lambda:len(requests)>count,10)
                                assert len(shifts)==1
                                print('PASS stair reverse -> checked uphill retry -> evidence-gated defer')
                                return 0
                            base_position[:]=[retry_point.x,retry_point.y,retry_point.z]
                            until=time.monotonic()+.3
                            spin_until(lambda:time.monotonic()>=until,2)
                            ack.publish(retry.target)
                        raise AssertionError('replanned route did not return to the uphill waypoint')
                    stop=request.target
                    if stair_reverse_test and .47<p.z<.56:
                        # Stop within the first tread's waypoint tolerance
                        # with room both before the next riser and behind
                        # the rear feet for the short reverse being tested.
                        stop=copy.deepcopy(stop)
                        stop.pose.position.x+=.10
                    stopped=stop.pose.position
                    if short_floor_test and support_scan and request.has_xy_tolerance and p.z>upper_stop_z:
                        loaded_feet=3
                        if platform_shift_test:
                            # Establish the conflict before the stop acknowledgement
                            # can trigger platform confirmation.
                            obstacle=(round(stopped.x*10-3)*.1,round(stopped.y*10)*.1,upper_ground+.25)
                            points=full_stairs+[obstacle]*3
                    base_position[:]=[stopped.x,stopped.y,stopped.z]
                    until=time.monotonic()+.3
                    spin_until(lambda:time.monotonic()>=until,2)
                    assert states[-1]['pct_floor']['number']==1,states[-1]
                    ack.publish(stop)
                    complete.publish(Bool(data=True))
                    if stair_reverse_test or not request.has_xy_tolerance:
                        continue
                    if not support_scan:
                        assert p.z<upper_stop_z,base_position
                        spin_until(lambda:len(requests)>index+1 or
                                   len(shifts)>handled_survey_moves or any(
                                       s['state']=='cross_layer_support_scan' for s in states),12)
                        if len(shifts)>handled_survey_moves:
                            action,_,_,tx,ty,_,*kind=shifts[handled_survey_moves].data.split('|')
                            base_position[:2]=[float(tx),float(ty)]
                            shift_result.publish(String(data=action+'|succeeded'))
                            handled_survey_moves+=1
                            spin_until(lambda:any(s['state']=='cross_layer_stair_gap_scan' for s in states),8)
                        elif not any(s['state']=='cross_layer_support_scan' for s in states):
                            continue
                        # A fresh frame elsewhere no longer completes the scan.
                        # Supply returns on the expected tread before waiting.
                        points=full_stairs
                        spin_until(lambda:any(e['reason']=='stair_support_sweep_executed'
                                              for e in observation_events),12)
                        assert all(abs(angle)<=.71 for _,angle in observed_stair_turns),observed_stair_turns
                        support_scan=True
                        # Supply the entire stopping/near-blind disc before
                        # sending a final platform observation waypoint.
                        points=full_stairs
                    elif p.z>upper_stop_z:
                        break
                assert support_scan
                assert not any(g['selection_policy']=='pct_stair_frontier' for g in goals), \
                    'ascent must not sweep unrelated frontier clusters'
                assert any(s.get('pct_floor',{}).get('phase')=='ascending' for s in states),states[-1]
                assert any(g['mode']=='cross_layer' and g['observation_z']>.7 for g in goals),goals
                assert any(s.get('pct_floor',{}).get('saved_stair_entrances') for s in states),states[-1]
                assert base_position[2]>upper_stop_z,base_position
                if platform_shift_test:
                    spin_until(lambda:states[-1]['state']=='pct_blocker_recovery_waiting' and
                               states[-1]['pct_floor']['contacting_feet']==3,10)
                    assert states[-1]['pct_floor']['number']==1
                    assert len(shifts)==handled_survey_moves
                    loaded_feet=4
                    spin_until(lambda:len(shifts)>handled_survey_moves,10)
                    assert any(g['selection_policy']=='pct_landing_recovery_shift' for g in goals),goals
                    stamp,sx,sy,tx,ty,_,kind=shifts[handled_survey_moves].data.split('|')
                    assert kind=='shift' and math.hypot(float(tx)-obstacle[0],float(ty)-obstacle[1]) > \
                        math.hypot(float(sx)-obstacle[0],float(sy)-obstacle[1]),shifts[handled_survey_moves].data
                    base_position[:2]=[float(tx),float(ty)]
                    until=time.monotonic()+.45
                    spin_until(lambda:time.monotonic()>=until,2)
                    shift_result.publish(String(data=stamp+'|succeeded'))
                    spin_until(lambda:states[-1]['pct_floor']['number']==2,10)
                    assert states[-1]['control_mode']=='flat'
                    print('PASS verified platform shift -> fresh map -> confirmed upper floor')
                    return 0
                # An upper-height stop can still be at the edge of the
                # platform. Execute any verified route farther into it.
                for next_index in range(index+1,index+7):
                    spin_until(lambda:states[-1]['pct_floor']['number']==2 or
                               len(requests)>next_index,8)
                    if states[-1]['pct_floor']['number']==2:break
                    request=requests[next_index]
                    forward(request.target)
                    p=request.target.pose.position
                    base_position[:]=[p.x,p.y,p.z]
                    until=time.monotonic()+.5
                    spin_until(lambda:time.monotonic()>=until,2)
                    assert states[-1]['pct_floor']['number']==1,states[-1]
                    ack.publish(request.target)
                spin_until(lambda:any(s.get('pct_floor',{}).get('number')==2 for s in states),16)
                upper=next(s for s in states if s.get('pct_floor',{}).get('number')==2)
                assert abs(upper['pct_floor']['height']-upper_ground)<.01,upper
                assert upper['pct_floor']['phase']=='exploring' and upper['control_mode']=='flat',upper
                assert not upper['pct_floor']['start_floor_skipped'],upper
                floor_evidence=upper['height_review']['frontier_floors']
                assert len(floor_evidence)==2,floor_evidence
                assert abs(floor_evidence[0]['height'])<.01,floor_evidence
                assert abs(floor_evidence[1]['height']-upper_ground)<.01,floor_evidence
                if short_floor_test:
                    assert any(g['selection_policy']=='pct_stair_climb' for g in goals),goals
                    assert upper['pct_floor']['contacting_feet']==3,upper
                    print('PASS observed platform at 0.91 m starts ordinary floor exploration without four-foot confirmation')
                elif floor_skip_test:
                    assert all(g['selection_policy'].startswith(('pct_stair_', 'pct_blocker_')) or
                               (g['selection_policy']=='pct_platform_frontier' and g['mode']=='cross_layer')
                               for g in goals if g['mode']=='cross_layer'),goals
                    segment_scans=[e for e in observation_events if e['reason'] in
                                   ('stair_segment_scan_observed','stair_segment_sweep_no_related_returns',
                                    'stair_segment_scan_stance_unavailable')]
                    assert not segment_scans,observation_events
                    assert all(e['views_completed']<=1 for e in observation_events
                               if e['reason']=='stair_support_sweep_executed'),observation_events
                    climbs={g['goal_id'] for g in goals if g['selection_policy'] in ('pct_stair_climb','pct_stair_step')}
                    assert all(p['backend']['arrival_area']['ready'] for p in plans
                               if p['goal_id'] in climbs and p['usable']),plans
                    print('PASS skipped open floor 1 with pending frontiers and ascended to normal floor 2')
                print('PASS floor 1 -> verified staircase -> bounded support scan -> fresh treads -> floor 2')
                return 0
            if cross_test:
                spin_until(lambda:bool(requests),20)
                spin_until(lambda:any(s['control_mode']=='cross_layer' for s in states),5)
                handled=0
                # Simulate SCAN acknowledgments at the real planned XYZ.
                while True:
                    spin_until(lambda:len(requests)>handled,5)
                    request=requests[handled]
                    forward(request.target)
                    complete.publish(Bool(data=False))
                    p=request.target.pose.position
                    base_position[:]=[p.x,p.y,p.z]
                    until=time.monotonic()+.4
                    spin_until(lambda:time.monotonic()>=until,2)
                    ack.publish(request.target)
                    complete.publish(Bool(data=True))
                    handled+=1
                    if request.has_xy_tolerance:break
                assert base_position[2]>=.38+.10-1e-5,base_position
                # Known treads must lead to another route, without a fixed sweep.
                spin_until(lambda:len(requests)>handled or
                           any(s['state']=='cross_layer_waiting_for_forward_route' for s in states),8)
                for request in requests[handled:]:
                    assert request.target.pose.position.z>=base_position[2]-.05,request
                assert states[-1]['control_mode']=='cross_layer',states[-1]
                assert not observed_stair_turns,observed_stair_turns
                assert not any(e['reason'].startswith('stair_segment_') for e in observation_events),observation_events
                print('PASS verified ascent continues without fixed side sweeps; lower routes stay excluded')
                return 0
            if pending_test:
                spin_until(lambda:any(s['state'] in ('pct_local_search_pending','navigation_compute_retry') for s in states),25)
                until=time.monotonic()+3.
                spin_until(lambda:time.monotonic()>=until,5)
                assert goals and all(g['selection_scope']=='local' for g in goals),goals
                assert any(p['reason']=='pct_planning_timeout' for p in plans),plans
                assert not waypoints,'timeouts must not dispatch partial routes'
                assert not observed_turns,'compute timeout alone must not start a scan'
                assert not any(s['state'] in ('pct_local_exhausted','pct_region_complete') for s in states),states
                print('PASS local timeout stays pending; no global fallback or false completion')
                return 0
            if occupied_recovery_test:
                spin_until(lambda:bool(requests),12)
                assert any(s.get('detail') in ('solid_or_steep_surface','obstacle_footprint',
                                               'surface_height_mismatch')
                           for s in states),states[-1]
                handled=0
                while True:
                    request=requests[handled]
                    forward(request.target)
                    stamp=request.target.header.stamp
                    before=len(states)
                    failure.publish(String(data=f'{stamp.sec*1000000000+stamp.nanosec}|local_replan_exhausted'))
                    handled+=1
                    spin_until(lambda:len(requests)>handled or any(
                        s.get('detail')=='no_other_safe_delegate_goal' for s in states[before:]),10)
                    if len(requests)==handled:
                        break
                    assert handled<8, 'delegation must exhaust the finite observed patch'
                until=time.monotonic()+1.
                spin_until(lambda:time.monotonic()>=until,3)
                assert len(requests)==handled and states[-1]['recovery_phase']=='pct_blocker',states[-1]
                points=points+[(i*.1,j*.1,1.5) for i in range(-47,-23) for j in range(-11,12)]*3
                spin_until(lambda:len(requests)>handled,10)
                assert requests[handled].target.pose.position.x<-2.,requests[handled]
                assert all(g['recovery_request']=='delegate' for g in goals),goals
                print('PASS occupied start delegates, waits on exhaustion, and retries a new target after map changes')
                return 0
            if recovery_test:
                spin_until(lambda:any(s['state'] in ('pct_unsafe_start','pct_map_without_safe_surface')
                                      for s in states),20)
                assert not waypoints, 'sparse strip must not become an unverified weak destination'
                points=[(i*.1,j*.1,1.5) for i in range(20,81) for j in range(-25,26)]
                spin_until(lambda:bool(waypoints),25)
                spin_until(lambda:bool(plans) and bool(goals))
                delegated=[p for p in plans if p['planner']=='scan_direct']
                assert delegated and delegated[-1]['usable'],plans[-3:]
                assert delegated[-1]['backend']['path_validated'] is False,delegated[-1]
                assert delegated[-1]['recovery_request']=='delegate'
                assert delegated[-1]['backend']['goal_check']['ready']
                assert 'parking_radius' not in delegated[-1]['backend']
                assert requests[0].has_xy_tolerance and abs(requests[0].xy_tolerance-.15)<1e-9
                assert ((waypoints[0].pose.position.x-base_position[0])**2+
                        (waypoints[0].pose.position.y-base_position[1])**2)**.5>requests[0].xy_tolerance, 'delegated goal cannot produce displacement'
                assert not any(p['planner']=='recovery_layer' and p.get('recovery_request')=='position' for p in plans), 'blind start repeatedly entered position A*'
                assert not routes[-1].poses or len(routes[-1].poses)==1,'fabricated validated line'
                first=waypoints[0];forward(first)
                until=time.monotonic()+.15;spin_until(lambda:time.monotonic()>=until,2)
                stamp=first.header.stamp.sec*1000000000+first.header.stamp.nanosec
                failure.publish(String(data=str(stamp-1)+'|execution_no_progress'))
                until=time.monotonic()+.15;spin_until(lambda:time.monotonic()>=until,2)
                assert not any(e['status']=='failed' for e in executions)
                failure.publish(String(data=str(stamp)+'|execution_no_progress'))
                spin_until(lambda:any(e['status']=='failed' for e in executions))
                assert executions[-1]['requested_xy_tolerance']==.15,executions[-1]
                assert executions[-1]['source']=='recovery_scan_direct',executions[-1]
                spin_until(lambda:len(waypoints)>=2,8)
                second=waypoints[1]
                assert math.hypot(second.pose.position.x-first.pose.position.x,
                                  second.pose.position.y-first.pose.position.y)>=.39,waypoints
                forward(second)
                # A validly stamped goal receipt cannot end recovery if
                # fresh odometry still shows the robot at the unsafe start.
                ack.publish(second)
                spin_until(lambda:len(waypoints)>=3,8)
                third=waypoints[2]
                assert any(s['state']=='pct_delegate_stop_unverified' for s in states),states[-1]
                assert not any(s['state']=='pct_blocker_delegation_reached' for s in states)
                assert all(g['recovery_request']=='delegate' for g in goals),goals
                forward(third)
                stop=third.pose.position
                base_position[:]=[stop.x,stop.y,stop.z]
                ack.publish(third)
                spin_until(lambda:any(s['state']=='pct_blocker_delegation_reached' for s in states),8)
                assert next(s for s in reversed(states) if s['state']=='pct_blocker_delegation_reached')[
                    'recovery_phase']=='idle'
                print('PASS matched SCAN failure and unsafe stop both retry, and a verified actual stop ends recovery')
                return 0
            if completion_test:
                # Retaining all slices also retains the exposed roof. Its
                # outer boundary is unresolved even though lower rooms are
                # enclosed; it must not be hidden to declare completion.
                spin_until(lambda:bool(states) and states[-1]['pct_frontier_valid'] and
                    states[-1]['pct_safe_cells']>0 and states[-1]['pct_frontier_cells']>0)
                state=states[-1]
                assert state['frontier_source']=='pct' and state['completion_source']=='pct',state
                assert state['state']!='pct_region_complete',state
                assert state['pct_safe_cells']>0 and state['voxel_count']<=1,state
                assert not waypoints,'completion observation must not send a goal'
                spin_until(lambda:bool(markers) and any(p.z>3. for m in markers[-1].markers for p in m.points))
                print('PASS enclosed lower floors do not hide the retained roof frontier or falsely complete')
                return 0
            spin_until(lambda:bool(waypoints))
            spin_until(lambda:bool(goals) and bool(plans) and bool(markers) and bool(routes))
            first_is_final=requests[0].has_xy_tolerance
            if first_is_final:assert abs(requests[0].xy_tolerance-.15)<1e-9
            assert goals[-1]['candidate_source']=='layer_frontier',goals[-1]
            assert goals[-1]['planner_mask']=='layer',goals[-1]
            assert goals[-1]['xy_tolerance']==.15,goals[-1]
            if global_test:
                assert goals[-1]['selection_scope']=='global',goals[-1]
                spin_until(lambda:any(s.get('pct_local_fallback_count',0)>0 for s in states))
                assert not any(s['state']=='pct_region_complete' for s in states),states[-5:]
            else:
                assert goals[-1]['selection_scope']=='local',goals[-1]
                spin_until(lambda:any(s.get('pct_selection_scope')=='local' and s.get('pct_frontier_valid') for s in states))
                center=next(s['pct_local_window']['center'] for s in reversed(states)
                            if s.get('pct_selection_scope')=='local' and s.get('pct_frontier_valid'))
                assert abs(center[2]-1.5)<.02,center
            assert ((goals[-1]['observation_x']-base_position[0])**2+
                    (goals[-1]['observation_y']-base_position[1])**2)**.5>=.5,goals[-1]
            assert abs(goals[-1]['target_z']-1.5)<.02,goals[-1]
            assert abs(goals[-1]['observation_z']-1.88)<.02,goals[-1]
            assert plans[-1]['usable'] and plans[-1]['backend']['backend']=='project_pct_interval_online',plans[-1]
            spin_until(lambda:bool(surfaces) and bool(map_states))
            assert {'cost','source','flags','ceiling_z'} <= {f.name for f in surfaces[-1].fields}
            assert map_states[-1]['backend']=='project_pct_interval_online'
            assert map_states[-1]['inspection_valid'],map_states[-1]
            assert map_states[-1]['cloud_sources'][0]['integrated']>0
            assert abs(waypoints[0].pose.position.z-1.88)<.02,waypoints[0]
            assert all(abs(p.pose.position.z-1.88)<.02 for p in routes[-1].poses)
            state=states[-1]
            assert state['frontier_source']=='pct' and state['pct_frontier_cells']>0,state
            assert state['voxel_count']<=1 and state['global_frontier_cells']==0,state
            visible={round(p.z,1) for m in markers[-1].markers if m.ns=='pct_surface_frontiers' for p in m.points}
            assert 0.1 in visible and 1.6 in visible,visible
            until=time.monotonic()+1.2
            spin_until(lambda:time.monotonic()>=until,3)
            assert len(waypoints)==1,'advanced without SCAN feedback'
            if observation_test:
                stage=0
                while not any(s.get('observation_task_active') for s in states):
                    spin_until(lambda:len(waypoints)>stage)
                    pose=waypoints[stage]
                    base_position[:]=[pose.pose.position.x,pose.pose.position.y,pose.pose.position.z]
                    until=time.monotonic()+.3;spin_until(lambda:time.monotonic()>=until,2)
                    forward(pose)
                    until=time.monotonic()+.15;spin_until(lambda:time.monotonic()>=until,2)
                    ack.publish(pose);complete.publish(Bool(data=True));stage+=1
                    spin_until(lambda:len(waypoints)>stage or any(s.get('observation_task_active') for s in states))
                if observation_resolved_test:
                    spin_until(lambda:bool(observed_turns),10)
                    old_stamp=observed_turns[0][0]
                    points.extend((i*.1,j*.1,z) for i in range(-30,31) for j in range(-30,31)
                                  for z in (1.5,3.) for _ in range(3))
                    spin_until(lambda:any(e['reason']=='frontier_resolved_in_observation' for e in observation_events),12)
                    spin_until(lambda:any(m.data==old_stamp for m in canceled_turns),3)
                    event=next(e for e in observation_events if e['reason']=='frontier_resolved_in_observation')
                    assert event['success'] and event['duration_s']<12,event
                    for _ in range(2):turn_results.publish(String(data=old_stamp+'|succeeded'))
                    spin_until(lambda:len(waypoints)>stage,10)
                    assert requests[-1].goal_id>event['action_goal_id']
                    print('PASS committed targets resolve during an active turn; owned action canceled; late replies do not stall next dispatch')
                    return 0
                if observation_timeout_test:
                    spin_until(lambda:bool(observed_turns),10)
                    old_stamp=observed_turns[0][0]
                    # A short pose gap must pause rather than end the task.
                    base_pause_until=time.monotonic()+.45
                    spin_until(lambda:states[-1]['state']=='observation_waiting_for_pose',2)
                    base_pause_until=0.
                    spin_until(lambda:states[-1]['state']=='observation_turning',2)
                    assert not observation_events and not canceled_turns
                    # A persistent gap retires both the task and its owned turn.
                    base_pause_until=time.monotonic()+4.
                    spin_until(lambda:any(e['reason']=='pose_timeout' for e in observation_events),4)
                    failed=next(e for e in observation_events if e['reason']=='pose_timeout')
                    assert not failed['success'] and failed['pose_wait_s']>=.8,failed
                    assert failed['phase_seconds']['pose_wait']>=.8,failed
                    spin_until(lambda:any(m.data==old_stamp for m in canceled_turns),2)
                    assert any(e['status']=='failed' and e['reason']=='pose_timeout' and
                               str(e['scan_goal_stamp'])==old_stamp for e in executions),executions[-3:]
                    base_pause_until=0.
                    # Deliver duplicated late receipts before and during a new task.
                    for _ in range(2):turn_results.publish(String(data=old_stamp+'|succeeded'))
                    old_count=len(waypoints)
                    spin_until(lambda:len(waypoints)>old_count,15)
                    assert requests[-1].goal_id>failed['action_goal_id']
                    newer=waypoints[-1];forward(newer)
                    base_position[:]=[newer.pose.position.x,newer.pose.position.y,newer.pose.position.z]
                    until=time.monotonic()+.2;spin_until(lambda:time.monotonic()>=until,2)
                    ack.publish(newer);complete.publish(Bool(data=True))
                    spin_until(lambda:len(observed_turns)>1,8)
                    new_stamp=observed_turns[-1][0]
                    assert new_stamp!=old_stamp
                    for _ in range(2):turn_results.publish(String(data=old_stamp+'|succeeded'))
                    until=time.monotonic()+.25;spin_until(lambda:time.monotonic()>=until,2)
                    assert states[-1]['observation_task_active'],states[-1]
                    assert not any(e['status']=='succeeded' and str(e['scan_goal_stamp'])==new_stamp
                                   for e in executions)
                    turn_results.publish(String(data=new_stamp+'|succeeded'))
                    spin_until(lambda:any(e['status']=='succeeded' and str(e['scan_goal_stamp'])==new_stamp
                                          for e in executions),3)
                    spin_until(lambda:observed_turns[-1][0]!=new_stamp,3)
                    next_stage_stamp=observed_turns[-1][0]
                    turn_results.publish(String(data=new_stamp+'|succeeded'))
                    # Healthy odometry but no completion: the normal task
                    # deadline must retire the remaining turn stages too.
                    spin_until(lambda:any(e['reason']=='observation_timeout' for e in observation_events),11)
                    spin_until(lambda:any(m.data==next_stage_stamp for m in canceled_turns),2)
                    assert any(e['status']=='failed' and e['reason']=='observation_timeout' and
                               str(e['scan_goal_stamp'])==next_stage_stamp for e in executions)
                    print('PASS transient pose wait resumes; persistent and task deadlines cancel owned turns; late duplicates do not block or settle new tasks/stages')
                    return 0
                spin_until(lambda:any(s['state']=='observation_waiting_for_local_frames' for s in states),10)
                until=time.monotonic()+.45;spin_until(lambda:time.monotonic()>=until,3)
                assert not any(e['success'] for e in observation_events), 'arrival and old frames cannot complete observation'
                # Deliberately add no new ground. Finish after the primary
                # view and at most one optional side view.
                spin_until(lambda:any(e['success'] for e in observation_events),18)
                done=next(e for e in observation_events if e['success'])
                assert states[-1]['control_mode']=='flat' and done['reason']=='sweep_executed',done
                assert done['evidence']['gain']==0 and 'actual_yaw' in done,done
                assert 1<=done['views_completed']<=2 and done['duration_s']>=.98,done
                assert sum(abs(a) for _,a in observed_turns)<3.6
                print('PASS checked arrival and at most two views complete on schedule with zero gain')
                return 0
            first=waypoints[0]
            # Reproduce immediate-arrival ordering: the stamped receipt can
            # beat the forwarded GoalRequest callback, and depth-one Bool
            # may retain only true. Do not require a false arming edge.
            # The coordinator can be one /clock tick behind the sender.
            # Its own stamp is still authoritative for this identified stage.
            receipt=copy.deepcopy(first)
            sent_ns=first.header.stamp.sec*1000000000+first.header.stamp.nanosec
            receipt.header.stamp.sec,receipt.header.stamp.nanosec=divmod(sent_ns-1000000,1000000000)
            ack.publish(receipt);complete.publish(Bool(data=True))
            until=time.monotonic()+.15;spin_until(lambda:time.monotonic()>=until,2)
            assert len(waypoints)==1,'accepted arrival before binding the identified request'
            handoff_started=time.monotonic()
            confirmed=copy.deepcopy(requests[0])
            confirmed.target.header.stamp=receipt.header.stamp
            forwarded.publish(confirmed)
            spin_until(lambda:any(e['status']=='succeeded' for e in executions))
            if first_is_final:
                spin_until(lambda:any(s.get('observation_task_active') for s in states))
                spin_until(lambda:len(waypoints)>1,20)
                assert requests[1].goal_id>requests[0].goal_id
            else:
                spin_until(lambda:len(waypoints)>1)
                assert time.monotonic()-handoff_started<.5,'next waypoint waited for controller tick'
                assert requests[1].goal_id==requests[0].goal_id and requests[1].waypoint_index==1
            success=next(e for e in executions if e['status']=='succeeded')
            assert success['scan_goal_stamp']==sent_ns-1000000,success
            assert success['requested_xy_tolerance']==(.15 if first_is_final else None),success
            assert success['goal_distance']<1e-8,success
            second=waypoints[1];forward(second)
            until=time.monotonic()+.15;spin_until(lambda:time.monotonic()>=until,2)
            stamp=lambda m:m.header.stamp.sec*1000000000+m.header.stamp.nanosec
            failure.publish(String(data=str(stamp(first))+'|execution_no_progress'))
            until=time.monotonic()+.15;spin_until(lambda:time.monotonic()>=until,2)
            assert not any(e['status']=='failed' for e in executions),'accepted stale failure'
            failure.publish(String(data=str(stamp(second))+'|execution_no_progress'))
            spin_until(lambda:any(e['status']=='failed' for e in executions))
            assert executions[-1]['reason']=='execution_no_progress',executions[-1]
            assert activity and all(m.data for m in activity), 'execution/plan waits must retain task activity'
            pause=node.create_client(SetBool,'/exploration/set_paused')
            assert pause.wait_for_service(timeout_sec=3)
            pending=pause.call_async(SetBool.Request(data=True))
            spin_until(lambda:pending.done() and activity and not activity[-1].data)
            assert pending.result().success
            pending=pause.call_async(SetBool.Request(data=False))
            spin_until(lambda:pending.done() and activity[-1].data)
            assert pending.result().success
            manual_goal=node.create_publisher(GoalRequest,'/scan/waypoint_request',10)
            spin_until(lambda:manual_goal.get_subscription_count()>0)
            request=copy.deepcopy(requests[0])
            request.target.header.stamp=node.get_clock().now().to_msg()
            manual_goal.publish(request)
            spin_until(lambda:not activity[-1].data and any(s['state']=='paused_for_manual_goal' for s in states))
            print('PASS PCT-derived XYZ goal -> native A* -> continuous route stages -> matched SCAN feedback')
            print(json.dumps({k:state[k] for k in ('frontier_source','pct_frontier_cells','pct_frontier_clusters','voxel_count')}))
        except Exception:
            print('Observed turns:',observed_turns,'pending:',pending_turns);print('Task transitions:',[(s['state'],s.get('detail'),s.get('observation_evidence')) for s in states if s.get('observation_task_active') or s['state'].startswith('observation_')]);print('Last controller states:',json.dumps(states[-2:]))
            log.seek(0);print(log.read()[-12000:]);raise
        finally:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
            node.destroy_node();rclpy.shutdown()
    return 0


if __name__=='__main__':
    sys.exit(main())
