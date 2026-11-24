"""Quantitative metrics must not manufacture coverage or cross-floor successes."""
import json
from pathlib import Path
import tempfile
import unittest
import subprocess
import sys
import numpy as np

from legbot_exploration.benchmark_metrics import ExperimentMetrics,SurfaceCoverage,ResultWriter,distribution
from legbot_exploration.benchmark_reference import build_reference,save_reference


class BenchmarkTest(unittest.TestCase):
    def reference(self,root,points):
        path=Path(root)/'reference.npz'
        np.savez(path,xyz=np.asarray(points),resolution=.1,metadata=json.dumps({'test':True}))
        return str(path)

    def test_no_reference_is_unavailable_not_zero_or_full_coverage(self):
        c=SurfaceCoverage();c.observe([[0,0,0]])
        self.assertIsNone(c.summary()['coverage_ratio'])
        self.assertFalse(c.summary()['reference_coverage_valid'])
        self.assertAlmostEqual(c.area,.01)

    def test_same_ground_in_many_slices_only_counts_once(self):
        c=SurfaceCoverage();c.observe([[0,0,1.46]]*7+[[0,0,1.48],[0,0,2.96]])
        self.assertEqual(c.count,2)

    def test_unknown_and_immature_geometry_is_not_observed_area(self):
        c=SurfaceCoverage();c.observe([[0,0,0],[.1,0,0],[.2,0,0]],[32,34,0])
        self.assertEqual(c.count,1)

    def test_reference_coverage_needs_same_xy_and_height(self):
        with tempfile.TemporaryDirectory() as root:
            c=SurfaceCoverage(reference=self.reference(root,[[0,0,1.46],[.1,0,1.46],[0,0,2.96]]))
            c.observe([[0,0,1.46]]*8+[[.1,0,0.],[0,0,2.82]])
            self.assertAlmostEqual(c.summary()['coverage_ratio'],1/3)
            c.observe([[0,0,2.96],[.1,0,1.48]])
            self.assertEqual(c.summary()['coverage_ratio'],1.)

    def test_reference_coverage_is_monotonic_across_sparse_exports(self):
        with tempfile.TemporaryDirectory() as root:
            c=SurfaceCoverage(reference=self.reference(root,[[0,0,0],[.1,0,0]]))
            c.observe([[0,0,0]]);c.observe([[.1,0,0]])
            self.assertEqual(c.summary()['coverage_percent'],100.)

    def test_flat_coverage_grid_does_not_count_unknown_or_upper_floors(self):
        with tempfile.TemporaryDirectory() as root:
            c=SurfaceCoverage(reference=self.reference(root,[[0,0,0],[.1,0,0],[0,0,1.5]]))
            c.observe_grid((-.05,-.05),.1,2,1,[0,-1])
            self.assertAlmostEqual(c.summary()['coverage_ratio'],1/3)
            self.assertAlmostEqual(c.area,.01)
            c.observe_grid((-.05,-.05),.1,2,1,[0,100])
            self.assertAlmostEqual(c.summary()['coverage_ratio'],2/3)
            with self.assertRaises(ValueError):c.observe_grid((0,0),.1,2,1,[0,0])

    def test_clock_reset_finalizes_instead_of_negative_duration(self):
        m=ExperimentMetrics();m.start(0.,10.);m.tick(1.,11.);m.tick(2.,1.)
        self.assertTrue(m.finalized);self.assertEqual(m.termination,'clock_reset')
        self.assertEqual(m.summary()['elapsed_sim_seconds'],1.)

    def test_simulation_time_and_wall_time_are_distinct(self):
        m=ExperimentMetrics();m.start(0.,0.);m.tick(10.,5.)
        self.assertEqual(m.summary()['real_time_factor'],.5)

    def test_exploration_speed_uses_reference_area_when_available(self):
        with tempfile.TemporaryDirectory() as root:
            coverage=SurfaceCoverage(reference=self.reference(root,[[0,0,0],[.1,0,0]]))
            coverage.observe([[0,0,0]])
            m=ExperimentMetrics(coverage);m.start(0.,0.)
            coverage.observe([[.1,0,0],[100,100,0]])
            m.tick(2.,2.)
            self.assertAlmostEqual(m.summary()['exploration_speed_m2_per_sim_s'],.005)
            self.assertEqual(m.summary()['exploration_speed_basis'],'reference_covered_area')

    def test_floor_arrival_requires_reference_platform_at_that_xy(self):
        with tempfile.TemporaryDirectory() as root:
            coverage=SurfaceCoverage(reference=self.reference(root,[[0,0,0],[.1,0,1.5]]))
            floors=[dict(name='first',height=0.),dict(name='second',height=1.5)]
            self.assertIsNone(coverage.floor_at([0.,0.,1.5],floors))
            self.assertEqual(coverage.floor_at([.1,0.,1.5],floors),'second')

    def test_path_skips_duplicates_gaps_and_pose_jumps(self):
        m=ExperimentMetrics();m.start(0.,0.)
        for stamp,x in [(0.,0.),(1.,.5),(1.,.5),(2.,1.),(3.,50.),(6.,51.)]:
            m.odometry([x,0.,.32],stamp,stamp)
        self.assertAlmostEqual(m.path_xy,1.)
        self.assertEqual(m.odom_anomalies['out_of_order_or_duplicate'],1)
        self.assertEqual(m.odom_anomalies['position_jump'],1)
        self.assertEqual(m.odom_anomalies['gap_over_2s'],1)

    def test_long_stationary_stop_does_not_look_like_missing_odometry(self):
        m=ExperimentMetrics();m.start(0.,0.)
        for stamp in range(10):m.odometry([0.,0.,.32],float(stamp),float(stamp))
        m.odometry([.1,0.,.32],10.,10.)
        self.assertAlmostEqual(m.path_xy,.1)
        self.assertEqual(m.odom_anomalies['gap_over_2s'],0)

    def transition_model(self):
        m=ExperimentMetrics(floors=[dict(name='first',height=0.),dict(name='second',height=1.5)],
                            target_floor='second',floor_dwell=.5,transition_timeout=10.)
        m.start(0.,0.)
        m.odometry([0,0,.32],0.,0.);m.odometry([0,0,.32],1.,1.)
        return m

    def test_height_band_counter_cannot_mark_physical_floor_arrival(self):
        m=self.transition_model();m.status={'pct_floor':{'number':19,'height':1.5}}
        self.assertFalse(m.summary()['mission_success'])

    def test_cross_layer_requires_departure_and_dwell_on_target_floor(self):
        m=self.transition_model();m.odometry([0,0,.6],2.,2.)
        m.odometry([0,0,1.82],3.,3.)
        self.assertEqual(m.summary()['cross_layer']['succeeded'],0)
        m.odometry([0,0,1.82],4.,4.)
        self.assertEqual(m.summary()['cross_layer']['success_rate'],1.)
        self.assertTrue(m.summary()['mission_success'])

    def test_return_to_original_floor_records_failed_attempt(self):
        m=self.transition_model();m.odometry([0,0,.6],2.,2.)
        m.odometry([0,0,.32],3.,3.);m.odometry([0,0,.32],4.,4.)
        self.assertEqual(m.summary()['cross_layer']['failed'],1)

    def test_interruption_retains_pending_attempt_and_does_not_claim_success(self):
        m=self.transition_model();m.odometry([0,0,.6],2.,2.);m.finish('interrupted',3.,3.)
        self.assertEqual(m.summary()['cross_layer']['interrupted'],1)
        self.assertEqual(m.summary()['cross_layer']['success_rate'],0.)
        self.assertFalse(m.summary()['mission_success'])

    def test_timeout_does_not_repeat_attempts_without_another_departure(self):
        m=self.transition_model();m.odometry([0,0,.6],2.,2.);m.tick(13.,13.)
        m.odometry([0,0,.6],14.,14.)
        self.assertEqual(m.summary()['cross_layer']['started'],1)
        self.assertEqual(m.summary()['cross_layer']['failed'],1)

    def test_identical_planner_and_execution_receipts_are_deduplicated(self):
        m=ExperimentMetrics();event=dict(goal_id=1,attempt_id=1,planner='layer',usable=True,planning_ms=3.)
        m.planning_result(event);m.planning_result(event)
        m.execution_result(dict(goal_id=1,attempt_id=1,stage_index=0,scan_goal_stamp=5,status='succeeded'))
        m.execution_result(dict(goal_id=1,attempt_id=1,stage_index=0,scan_goal_stamp=5,status='succeeded'))
        self.assertEqual(m.summary()['planning']['attempts'],1)
        self.assertEqual(m.summary()['execution']['terminal_waypoints'],1)

    def test_result_files_share_root_without_overwriting_previous_test(self):
        with tempfile.TemporaryDirectory() as root:
            a=ResultWriter(root,dict(method='test'));b=ResultWriter(root,dict(method='test'))
            m=ExperimentMetrics();m.finish('interrupted',0,0)
            a.snapshot(m.summary());b.snapshot(m.summary());a.close();b.close()
            self.assertNotEqual(a.prefix,b.prefix)
            self.assertEqual(len(list(Path(root).glob('*.json'))),2)
            self.assertTrue(all(p.is_file() for p in Path(root).iterdir()))
            for p in Path(root).glob('*.json'):json.loads(p.read_text())

    def test_single_file_and_return_must_follow_exploration(self):
        with tempfile.TemporaryDirectory() as root:
            m=self.transition_model()
            m.odometry([0,0,.6],2.,2.);m.odometry([0,0,1.82],3.,3.);m.odometry([0,0,1.82],4.,4.)
            m.status={'mission':dict(return_enabled=True,exploration_finished=False,returned_home=False)}
            self.assertFalse(m.summary()['mission_success'])
            m.status['mission']['exploration_finished']=True
            self.assertFalse(m.summary()['mission_success'])
            m.status['mission']['returned_home']=True
            self.assertTrue(m.summary()['mission_success'])
            w=ResultWriter(root,{});w.event('test',0,{})
            w.row({'a':1});w.snapshot(m.summary());w.close()
            self.assertEqual([p.suffix for p in Path(root).iterdir()],['.json'])

    def test_patent_result_aggregates_surfaces_and_includes_failed_planning_times(self):
        with tempfile.TemporaryDirectory() as root:
            c=SurfaceCoverage(reference=self.reference(root,[[0,0,0],[0,0,1.5],[.1,0,1.5]]))
            c.observe([[0,0,0]]*5+[[0,0,1.5]]*4+[[100,100,0]])
            m=ExperimentMetrics(c);m.start(10,20)
            for i,(usable,elapsed) in enumerate([(True,10.),(False,30.)]):
                event=dict(goal_id=i,attempt_id=1,planner='layer',usable=usable,planning_ms=elapsed)
                m.planning_result(event);m.planning_result(event)
            m.finish('interrupted',18,24)
            w=ResultWriter(root,{});result=w.snapshot(m.summary());w.snapshot(m.summary());w.close()
            values=result['metrics']
            self.assertAlmostEqual(values['reference_area_m2'],.03)
            self.assertAlmostEqual(values['observed_area_m2'],.02)
            self.assertAlmostEqual(values['coverage_percent'],200/3)
            self.assertEqual(values['planning_success_count'],1)
            self.assertEqual(values['planning_attempt_count'],2)
            self.assertEqual(values['planning_success_percent'],50.)
            self.assertEqual(values['planning_mean_ms'],20.)
            self.assertEqual(values['planning_p95_ms'],29.)
            self.assertEqual(values['exploration_sim_seconds'],4.)
            self.assertEqual(values['exploration_wall_seconds'],8.)
            self.assertEqual(len(result['metric_labels']),len(values))
            self.assertNotIn('floors',result)
            self.assertNotIn('cross_layer',result)
            self.assertNotIn('attempts',result)
            self.assertEqual(len(list(Path(root).glob('*.json'))),1)
            self.assertFalse(list(Path(root).glob('*.tmp')))

    def test_patent_upstairs_excludes_downstairs_and_keeps_interrupted_attempt(self):
        with tempfile.TemporaryDirectory() as root:
            m=self.transition_model()
            for stamp,ground in [(2,.28),(3,1.5),(4,1.5),(5,1.),(6,0.),(7,0.),(8,.28)]:
                m.odometry([0,0,ground+.32],float(stamp),float(stamp))
            m.finish('interrupted',9,9)
            self.assertEqual(m.summary()['cross_layer']['succeeded'],2)
            w=ResultWriter(root,{});result=w.snapshot(m.summary());w.close()
            values=result['metrics']
            self.assertEqual(values['upward_attempt_count'],2)
            self.assertEqual(values['upward_success_count'],1)
            self.assertEqual(values['upward_success_percent'],50.)
            self.assertEqual(values['upward_mean_sim_seconds'],2.)
            self.assertEqual(result['accounting']['upward_interrupted_count'],1)
            self.assertEqual(result['accounting']['upward_pending_count'],0)

    def test_patent_undefined_measurements_are_null_not_fabricated(self):
        with tempfile.TemporaryDirectory() as root:
            m=ExperimentMetrics();m.finish('interrupted',0,0)
            w=ResultWriter(root,{});result=w.snapshot(m.summary());w.close()
            for key in ('reference_area_m2','observed_area_m2','coverage_percent',
                        'planning_success_percent','planning_mean_ms','planning_p95_ms',
                        'upward_success_percent','upward_mean_sim_seconds',
                        'exploration_sim_seconds','exploration_wall_seconds'):
                self.assertIsNone(result['metrics'][key],key)
            self.assertEqual(result['metrics']['planning_attempt_count'],0)
            self.assertFalse(result['accounting']['coverage_reference_available'])

    def test_incomplete_or_negative_planning_timings_do_not_bias_the_mean(self):
        for bad in (None,-1.,float('nan'),True):
            with self.subTest(timing=bad):
                m=ExperimentMetrics()
                m.planning_result(dict(goal_id=1,usable=True,planning_ms=10.))
                m.planning_result(dict(goal_id=2,usable=False,planning_ms=bad))
                plan=m.summary()['planning']
                self.assertEqual(plan['attempts'],2)
                self.assertEqual(plan['timing_samples'],1)
                self.assertIsNone(plan['mean_ms'])
                self.assertIsNone(plan['p95_ms'])

    def test_floor_metrics_count_unique_reference_and_exclude_return_time(self):
        with tempfile.TemporaryDirectory() as root:
            c=SurfaceCoverage(reference=self.reference(root,[[0,0,0],[.1,0,0],[0,0,1.5]]))
            m=ExperimentMetrics(c,[dict(name='one',height=0),dict(name='two',height=1.5)],floor_dwell=0)
            m.start(0,0);m.odometry([0,0,.32],0,0);c.observe([[0,0,0]]*5)
            m.tick(2,1);m.status={'mission':dict(returning=True)};m.tick(4,2)
            row=m.summary()['floors'][0]
            self.assertEqual(row['coverage_percent'],50)
            self.assertEqual(row['exploration_sim_seconds'],1)
            self.assertEqual(row['exploration_wall_seconds'],2)

    def scoped_model(self,root):
        coverage=SurfaceCoverage(reference=self.reference(root,
            [[0,0,0],[0,0,1.46],[.1,0,1.46],[0,0,2.96]]))
        m=ExperimentMetrics(coverage,[dict(name='floor_1',height=0.),dict(name='floor_2',height=1.46)],
                            single_floor='floor_2',floor_dwell=0.)
        m.start(0.,0.)
        return m

    def floor_status(self,started=10.,ended=None,height=1.46):
        return dict(state='pct_floor_exploration_started' if ended is None else 'pct_floor_handoff_with_deferred_frontiers',
            pct_floor=dict(number=99,height=height,ordinary_started_sim=started,ordinary_started_wall=2*started,
                ordinary_ended_sim=ended,ordinary_ended_wall=None if ended is None else 2*ended))

    def test_floor2_only_freezes_coverage_time_at_first_handoff(self):
        with tempfile.TemporaryDirectory() as root:
            m=self.scoped_model(root);m.coverage.observe([[0,0,0],[0,0,1.46]])
            # Pre-observed floor 2 ground counts; unrelated floors do not.
            m.odometry([0,0,1.78],10.,20.)
            m.update_status(self.floor_status(),20.,10.)
            m.update_status(self.floor_status(ended=15.),30.,15.)
            m.coverage.observe([[.1,0,1.46],[0,0,2.96]])
            m.update_status(self.floor_status(started=30.),60.,30.)
            m.status={'mission':dict(returning=True)};m.finish('interrupted',100.,50.)
            w=ResultWriter(root,dict(single_floor='floor_2'));r=w.snapshot(m.summary())
            self.assertEqual(r['single_floor']['state'],'settled')
            self.assertEqual(r['metrics']['coverage_percent'],50.)
            self.assertAlmostEqual(r['metrics']['reference_area_m2'],.02)
            self.assertEqual(r['metrics']['exploration_sim_seconds'],5.)
            self.assertEqual(r['metrics']['exploration_wall_seconds'],10.)
            self.assertEqual(len(list(Path(root).glob('*.json'))),1)

    def test_floor_number_and_stair_band_cannot_start_floor2_trial(self):
        with tempfile.TemporaryDirectory() as root:
            m=self.scoped_model(root);m.odometry([0,0,.32],1.,2.)
            m.update_status(self.floor_status(),20.,10.)
            self.assertIsNone(m.summary()['single_floor']['started_sim'])
            m.odometry([0,0,1.78],11.,22.)
            m.update_status(self.floor_status(height=1.30),22.,11.)
            # It started only when the genuine floor status and pose agree.
            self.assertEqual(m.summary()['single_floor']['started_sim'],10.)

    def test_floor2_interruption_and_not_started_are_explicit(self):
        with tempfile.TemporaryDirectory() as root:
            m=self.scoped_model(root);m.finish('interrupted',100.,50.)
            row=m.summary()['single_floor']
            self.assertEqual(row['state'],'not_started')
            self.assertIsNone(row['coverage_percent']);self.assertIsNone(row['exploration_sim_seconds'])
            m=self.scoped_model(root);m.odometry([0,0,1.78],10.,20.)
            m.update_status(self.floor_status(),20.,10.);m.finish('interrupted',26.,13.)
            row=m.summary()['single_floor']
            self.assertEqual(row['state'],'interrupted');self.assertFalse(row['completed'])
            self.assertEqual(row['exploration_sim_seconds'],3.)

    def test_leaving_floor2_without_handoff_is_incomplete_not_extra_stair_time(self):
        with tempfile.TemporaryDirectory() as root:
            m=self.scoped_model(root);m.odometry([0,0,1.78],10.,20.)
            m.update_status(self.floor_status(),20.,10.)
            m.odometry([0,0,2.32],12.,24.)
            m.update_status(self.floor_status(started=12.,height=2.),24.,12.)
            m.tick(60.,30.)
            row=m.summary()['single_floor']
            self.assertEqual(row['state'],'incomplete_left_floor');self.assertFalse(row['completed'])
            self.assertEqual(row['exploration_sim_seconds'],2.)

    def test_floor2_without_reference_never_manufactures_coverage(self):
        m=ExperimentMetrics(floors=[dict(name='floor_2',height=1.46)],single_floor='floor_2')
        m.start(0.,0.);m.odometry([0,0,1.78],10.,20.)
        m.update_status(self.floor_status(),20.,10.)
        self.assertIsNone(m.summary()['single_floor']['coverage_percent'])

    def test_distribution_accepts_iterators_and_empty_samples(self):
        self.assertEqual(distribution(iter([1,2,3]))['median'],2.)
        self.assertIsNone(distribution([])['mean'])

    def test_aggregation_retains_failures_and_separates_partial_runs(self):
        with tempfile.TemporaryDirectory() as root:
            for i in range(3):
                m=self.transition_model();m.partial_run=i==2
                m.odometry([0,0,.6],2.,2.)
                if i!=1:
                    m.odometry([0,0,1.82],3.,3.);m.odometry([0,0,1.82],4.,4.)
                m.finish('interrupted',5.,5.)
                writer=ResultWriter(root,dict(method='fixture'),run_id='trial_'+str(i))
                writer.snapshot(m.summary());writer.close()
            script=Path(__file__).resolve().parents[1]/'python/scripts/summarize_exploration_metrics'
            for _ in range(2):
                subprocess.run([sys.executable,str(script),root],check=True,capture_output=True)
            rows=json.loads((Path(root)/'comparison.json').read_text())
            self.assertEqual(len(rows),1)
            self.assertEqual(rows[0]['completed_full_trials'],2)
            self.assertEqual(rows[0]['mission_success_rate'],.5)
            self.assertEqual(rows[0]['upward_success_percent'],50.)
            self.assertEqual(rows[0]['partial_runs'],1)


class ReferenceTest(unittest.TestCase):
    def test_closed_box_floor_and_wall_are_independent_reference(self):
        sdf='''<sdf><world name="test"><model name="ground"><link name="a">
        <pose>0 0 -0.1 0 0 0</pose><collision name="g"><geometry><box><size>2 2 .2</size></box></geometry></collision>
        </link></model><model name="wall"><link name="a"><pose>0 0 .5 0 0 0</pose>
        <collision name="w"><geometry><box><size>.2 2 1</size></box></geometry></collision>
        </link></model></world></sdf>'''
        with tempfile.TemporaryDirectory() as root:
            world=Path(root)/'test.sdf';world.write_text(sdf)
            points,meta=build_reference(world,[],spawn=(-.5,0.,0.))
            self.assertTrue(len(points));self.assertTrue(np.all(points[:,0]<0))
            self.assertTrue(np.allclose(points[:,2],0.))
            path=Path(root)/'ref.npz';save_reference(path,points,meta)
            self.assertEqual(len(SurfaceCoverage(reference=path).reference),len(points))

    def test_material_split_mesh_does_not_fill_space_between_floors(self):
        def geometry(name,planes):
            vertices=[];faces=[]
            for height,up in planes:
                k=len(vertices);vertices += [(-1,-1,height),(1,-1,height),(1,1,height),(-1,1,height)]
                faces += [k+i for i in ((0,1,2,0,2,3) if up else (0,2,1,0,3,2))]
            values=' '.join(str(v) for point in vertices for v in point)
            indices=' '.join(map(str,faces))
            return f'<geometry id="{name}"><mesh><source id="{name}_p"><float_array>{values}</float_array><technique_common><accessor stride="3"/></technique_common></source><vertices id="{name}_v"><input semantic="POSITION" source="#{name}_p"/></vertices><triangles><input semantic="VERTEX" source="#{name}_v" offset="0"/><p>{indices}</p></triangles></mesh></geometry>'
        with tempfile.TemporaryDirectory() as root:
            root=Path(root)
            mesh='<COLLADA xmlns="urn:test"><asset><up_axis>Z_UP</up_axis><unit meter="1"/></asset><library_geometries>'+geometry('a',[(2,False),(4,False),(4.2,True)])+geometry('b',[(2.2,True)])+'</library_geometries><library_visual_scenes><visual_scene><node><instance_geometry url="#a"/><instance_geometry url="#b"/></node></visual_scene></library_visual_scenes></COLLADA>'
            (root/'floors.dae').write_text(mesh)
            world=root/'world.sdf';world.write_text('<sdf><world><model><link><collision name="slabs"><geometry><mesh><uri>floors.dae</uri></mesh></geometry></collision></link></model></world></sdf>')
            points,_=build_reference(world,[],spawn=(0,0,2.2))
            self.assertTrue(len(points));self.assertTrue(np.allclose(points[:,2],2.2))

    def test_unsupported_collision_geometry_is_not_silently_ignored(self):
        with tempfile.TemporaryDirectory() as root:
            world=Path(root)/'test.sdf'
            world.write_text('<sdf><world><model><link><collision name="sphere"><geometry><sphere><radius>1</radius></sphere></geometry></collision></link></model></world></sdf>')
            with self.assertRaises(ValueError):build_reference(world,[])


if __name__=='__main__':unittest.main()
