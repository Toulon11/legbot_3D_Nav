"""Incremental membership/components versus a complete unknown-side rebuild."""
import unittest
from collections import deque
from contextlib import ExitStack
from dataclasses import replace
from unittest.mock import patch
import numpy as np
from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
from legbot_exploration.pct_frontier_index import rebuild_frontiers


def floor(heights=(0.,),hole=(),bound=10):
    return np.asarray([(x*.1,y*.1,h) for x in range(-bound,bound+1)
                       for y in range(-bound,bound+1) for h in heights if (x,y) not in hole],dtype='float32')


def create(points,num=1):
    m=FixedLayerMap(MapSettings(robot_radius=0.,safety_margin=0.,maturity_min_points=num))
    m.integrate(np.repeat(points,num,axis=0));m.build()
    return m


class FrontierIndexTest(unittest.TestCase):
    def test_pose_visibility_cache_survives_a_base_cell_change(self):
        import time
        m=create(floor(hole={(0,0)},bound=15));index=m.frontier_index
        members=set(index.by_column[(0,0)]);cache=index.observer_cache
        cache.select(members,(0.,-.5,0.),floor=(0.,.08),deadline=time.monotonic()+.02)
        self.assertEqual(len(cache.searches),1)
        memo=next(iter(cache.searches.values()))
        cache.select(members,(.1,-.5,0.),floor=(0.,.08),deadline=time.monotonic()+.02)
        self.assertEqual(len(cache.searches),1)
        self.assertIs(next(iter(cache.searches.values())),memo)

    def test_soft_cost_drift_reuses_geometry_but_scores_the_current_cost(self):
        m=create(floor(hole={(0,0)},bound=12));index=m.frontier_index
        key=next(iter(index.by_column[(0,0)]));cache=index.observer_cache
        fid=index.components.node_fragment[key]
        group={n for n in index.components.fragments[fid] if n[0]==key[0]}
        data=cache.candidates(fid,key[0],group)
        self.assertTrue(len(data))
        pose=data[0,:3].copy();xy=tuple(m.indices(pose))
        row=m.cell((key[0],*xy));old=float(row['inflated_terrain_cost'])
        self.assertLess(old+1.,20.)
        entry=cache.cache[fid,key[0]];built=cache.built
        row['inflated_terrain_cost']=old+1.
        index.update({xy})
        self.assertIs(cache.cache[fid,key[0]],entry)
        updated=cache.candidates(fid,key[0],group)
        self.assertEqual(cache.built,built)
        self.assertAlmostEqual(updated[0,4],old+1.)
        row['inflated_terrain_cost']=50.
        index.update({xy})
        self.assertNotIn((fid,key[0]),cache.cache)

    def test_flat_selection_prefers_a_larger_nearby_task_without_removing_small_tasks(self):
        holes={(0,0)}|{(x,0) for x in range(12,18)}
        m=create(floor(hole=holes,bound=30));index=m.frontier_index;visited=[]
        def observes(members,*args,**kwargs):
            visited.append({k[1:3] for k in members})
            index.observer_cache.last_views={(0.,-.5,0.):dict(yaw=0.,gain=len(members))}
            return [[0.,-.5,0.]]
        with patch.object(index,'_observers',side_effect=observes):
            result=index.query((0.,-.5,0.),window_size=(2.,.5,.5),floor=(0.,.08),query_budget=.01)
        self.assertTrue(visited)
        self.assertTrue(any(xy in holes-{(0,0)} for xy in visited[0]))
        self.assertNotIn((0,0),visited[0])
        self.assertIn((0,0),index.by_column)
        self.assertTrue(any(any(k[1:3]==(0,0) for k in index.floor_query_members[c['id']])
                            for c in result['clusters']))

    def test_same_column_ground_incrementally_closes_the_tread_stripe_on_all_slices(self):
        height=1.425
        stripe={(x,0) for x in range(-9,10)}
        points=floor((height,),hole=stripe,bound=10).tolist()
        # Local high returns retain multiple cuts over the measured tread.
        points.append((-.9,-.2,2.8))
        results=[]
        for native in (False,True):
            with self.subTest(native=native):
                m=FixedLayerMap(MapSettings(robot_radius=0.,safety_margin=0.,maturity_min_points=1,
                                            step_max=.12),native_updates=native)
                if native:self.assertIsNotNone(m._boundary_kernel)
                m.integrate(points);m.build()
                index=m.frontier_index
                before={xy:set(index.by_column[xy]) for xy in stripe}
                self.assertGreater(len({key[0] for keys in before.values() for key in keys}),1)
                # Representative field stripe: already observed, even though
                # the return is a different height group and there is no edge fit.
                near=[(x*.1,0.,height-(.16 if x==-9 else .110)) for x in range(-9,9)]
                with patch.object(m,'iter_cells',side_effect=AssertionError('whole-map scan')):
                    m.integrate(near+[(.9,0.,height-.161)]);m.build()
                for xy in stripe-{(9,0)}:
                    self.assertFalse(before[xy] & index.nodes.keys(),str(xy))
                self.assertTrue(before[(9,0)] <= index.nodes.keys())
                self.assertTrue(index.nodes)
                self.assert_full_equal(m)
                results.append(dict(index.nodes))
        self.assertEqual(len(results),2)
        self.assertEqual(*results)

    def test_tread_boundary_suppresses_nearby_ordinary_frontier_overlapping_sheets(self):
        points=[(x*.1,y*.1,.915 if y>=0 else 1.025)
                for x in range(-3,4) for y in range(-3,4)]
        points.extend((x*.1,y*.1,.915) for x in range(-3,4) for y in range(-3,0))
        m=create(points)
        self.assertFalse(m.frontier_index.by_column.get((0,0)))
        self.assertTrue(m.surface_observation.matching((0,-1),.915))
        self.assertTrue(m.surface_observation.matching((0,-1),1.025))
        self.assert_full_equal(m)

    def test_first_floor_entry_treats_old_ground_as_unknown_until_its_source_is_reobserved(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=FixedLayerMap(MapSettings(maturity_min_points=1),native_updates=native)
                if native:self.assertIsNotNone(m._boundary_kernel)
                evidence=m.surface_observation
                high=2.95
                old={(0,0):2.82,(2,0):2.93}
                plane=[(x*.1,y*.1,high) for x in range(-3,4) for y in range(-3,4)
                       if (x,y) not in old]
                self.assertTrue(evidence.enter_frontier_floor((0.,0.,1.45),.08))
                evidence.visit_height((0.,0.,1.45))
                m.integrate([(x*.1,0,ground) for (x,_),z in old.items() for ground in (1.45,z)]);m.build()
                evidence.begin_crossing()
                cutoff=m.integrated_serial
                m.integrate(plane);m.build()
                index=m.frontier_index
                def members(xy):
                    return {key for key in index.by_column.get(xy,())
                            if abs(index.nodes[key].height-high)<1e-5}

                self.assertFalse(members((0,0)))
                self.assertFalse(members((2,0)))
                target=(0.,0.,high)
                inferred=evidence.expect(target,review=True,neighbor_inferred=True)
                view=evidence.expect((.1,.4,high),kind='frontier')
                blocker=evidence.expect(target,review=True)
                stair=evidence.expect((.1,.4,high+.2),kind='search',height_range=(high+.1,high+.3))
                with patch.object(m,'iter_cells',side_effect=AssertionError('whole-map scan')):
                    self.assertTrue(evidence.enter_frontier_floor((-.1,0.,high),.08))
                    m.build()
                self.assertEqual(evidence.floor_ground_band(high).cutoff,cutoff)
                self.assertEqual(set(evidence.records),{blocker.id,stair.id})
                self.assertEqual(evidence.by_column[(0,0)],[blocker.id])
                self.assertNotIn(view.id,evidence.by_column[(1,4)])
                self.assertNotIn(inferred.id,{identity for tile in evidence.pending_tiles.values() for identity in tile})
                self.assertEqual(evidence.visit_height((-.1,0.,high))['registered_targets'],0)
                self.assertTrue(members((0,0)))
                self.assertTrue(members((2,0)),'the 5 cm matching path must also honor the frame cutoff')
                key=next(iter(members((0,0))))
                model=index.observation_model
                model.task=dict(keys=[key],heights=[high])
                self.assertTrue(model.arm((-.1,0.,high),0.))
                self.assertEqual(set(evidence.records),{blocker.id,stair.id})
                self.assertFalse(model.progress()['ready'])
                m.integrate([(0.,0.,1.45)]);m.build()
                self.assertTrue(members((0,0)))
                self.assertFalse(model.progress()['ready'])
                version=m.revision
                m.integrate([(0.,0.,2.82)]);m.build()
                self.assertGreater(m.revision,version,'a new hit restores even a frozen raw G')
                self.assertFalse(members((0,0)))
                self.assertTrue(model.progress()['ready'])
                self.assertTrue(members((2,0)))
                m.integrate([(0.,0.,high),(0.2,0.,2.93)]);m.build()
                self.assertFalse(members((0,0)))
                self.assertTrue(model.progress()['ready'])
                self.assertFalse(members((2,0)),'a new hit restores even a frozen raw G')
                evidence.begin_crossing()
                self.assertFalse(evidence.enter_frontier_floor((-.1,0.,high),.08))
                self.assertEqual(evidence.floor_ground_band(high).cutoff,cutoff)
                self.assert_full_equal(m)

    def assert_full_equal(self,m):
        reference=rebuild_frontiers(m);index=m.frontier_index
        self.assertEqual(index.nodes,reference.nodes)
        groups=lambda obj:{frozenset(c['members']) for c in obj.clusters.values()}
        self.assertEqual(groups(index),groups(reference))
        # Independent whole-node traversal also checks the tiled component
        # implementation; rebuilding with the same tiling alone is no oracle.
        remaining=set(index.nodes);full=[]
        while remaining:
            first=remaining.pop();members={first};queue=deque([first])
            while queue:
                for key in index.neighbours(queue.popleft()):
                    if key in remaining:
                        remaining.remove(key);members.add(key);queue.append(key)
            full.append(frozenset(members))
        self.assertEqual(groups(index),set(full))
        observers=lambda obj:{frozenset(c['members']):c['observations'] for c in obj.clusters.values()}
        self.assertEqual(observers(index),observers(reference))
        self.assertEqual(index.known_count,reference.known_count)
        self.assertEqual(index.safe_count,reference.safe_count)

    def assert_slice_clusters(self, index, xy_keys, expected):
        per_slice={}
        for xy in xy_keys:
            for key in index.by_column.get(xy,()):
                per_slice.setdefault(key[0],set()).add(index.membership[key])
        self.assertTrue(per_slice)
        self.assertTrue(all(len(ids)==expected for ids in per_slice.values()))
        self.assertEqual(len(set.union(*per_slice.values())),len(per_slice)*expected)

    def test_unknown_side_is_removed_after_its_first_hit(self):
        m=create(floor(hole={(0,0)}),num=3)
        before=set(m.frontier_index.by_column[(0,0)])
        self.assertTrue(before)
        m.integrate(np.asarray([[0,0,0.]],dtype='float32'));m.build()
        self.assertNotIn((0,0),m.frontier_index.by_column)
        m.integrate(np.asarray([[0,0,0.]]*2,dtype='float32'));m.build()
        self.assertNotIn((0,0),m.frontier_index.by_column)
        self.assert_full_equal(m)

    def test_maturing_one_floor_keeps_the_other_floor_unknown(self):
        m=create(floor((0.,1.5),hole={(0,0)}),num=3)
        heights={m.frontier_index.nodes[k].height for k in m.frontier_index.by_column[(0,0)]}
        self.assertEqual(heights,{0.,1.5})
        m.integrate(np.asarray([[0,0,0.]]*3,dtype='float32'));m.build()
        heights={m.frontier_index.nodes[k].height for k in m.frontier_index.by_column[(0,0)]}
        self.assertEqual(heights,{1.5})
        self.assert_full_equal(m)

    def test_mature_underside_does_not_resolve_the_unknown_tread(self):
        # Captured building geometry: underside and top differ by 14.3 cm,
        # within step ability but outside the 5 cm surface noise tolerance.
        top=1.4608803;bottom=1.3181558
        m=create(floor((top,),hole={(0,0)},bound=4),num=3)
        index=m.frontier_index
        before={key for key in index.by_column[(0,0)] if abs(index.nodes[key].height-top)<1e-6}
        self.assertTrue(before)
        for orientation in (-1,-2):
            with patch('legbot_exploration.surface_orientation.classify',
                       return_value=np.full(3,orientation)):
                m.integrate([[0,0,bottom]]*3,sensor_origin=(0.,-.6,1.7))
            m.build()
        after={key for key in index.by_column[(0,0)] if abs(index.nodes[key].height-top)<1e-6}
        self.assertEqual(before,after)
        self.assert_full_equal(m)
        m.integrate(np.asarray([[0,0,top]],dtype='float32'));m.build()
        self.assertFalse(before & index.nodes.keys())
        self.assert_full_equal(m)

    def test_distinct_heights_in_one_cut_have_independent_frontiers(self):
        # Opposite sides of a gap have different surfaces in the SAME 0.5m cut.
        # No column-wise maximum may erase the lower side's unknown frontier.
        points=np.array([[x*.1,y*.1,.1 if x<0 else .3]
                         for x in range(-4,5) for y in range(-4,5) if x],dtype='float32')
        m=create(points,num=3);index=m.frontier_index
        keys={key for key in index.by_column[(0,0)] if key[0]==1}
        self.assertEqual(len(keys),2)
        self.assertEqual({round(index.nodes[key].height,2) for key in keys},{.1,.3})
        low=next(key for key in keys if index.nodes[key].height<.2)
        high=next(key for key in keys if index.nodes[key].height>.2)
        self.assertNotEqual(index.membership[low],index.membership[high])
        m.integrate(np.asarray([[0,0,.1]]*3,dtype='float32'));m.build()
        self.assertNotIn(low,index.nodes)
        self.assertIn(high,index.nodes)
        self.assert_full_equal(m)

    def test_close_parallel_frontiers_do_not_merge_by_step_ability(self):
        # Separate observed strips at different elevations seed parallel tasks;
        # their 14cm separation is walkable as a step but is not sheet identity.
        points=np.asarray([[x*.1,y*.1,.1 if x<0 else .24]
                           for x in range(-4,5) for y in range(-4,5) if x],dtype='float32')
        m=create(points);index=m.frontier_index
        keys={key for key in index.by_column[(0,0)] if key[0]==1}
        self.assertEqual(len(keys),2)
        self.assertEqual(len({index.membership[key] for key in keys}),2)
        self.assert_full_equal(m)

    def test_one_measured_ground_can_cover_two_ordinary_frontiers(self):
        points=np.asarray([[x*.1,y*.1,.1 if x<0 else .18]
                           for x in range(-3,4) for y in range(-3,4) if x],dtype='float32')
        m=create(points);index=m.frontier_index
        before={key for key in index.by_column[(0,0)] if key[0]==1}
        self.assertEqual(len(before),2)
        m.integrate(np.asarray([[0,0,.14]],dtype='float32'));m.build()
        self.assertFalse(before & index.nodes.keys())
        self.assert_full_equal(m)

    def test_small_stair_rises_still_form_a_continuous_frontier(self):
        # 1.4cm rises between adjacent XY columns remain connected. This is
        # distinct from two physical returns in the SAME XY being merged.
        points=np.asarray([[x*.1,y*.1,.014*(x+5)]
                           for x in range(-5,6) for y in range(-4,5)],dtype='float32')
        m=create(points);index=m.frontier_index
        edge=[key for x in range(-4,5) for key in index.by_column[(x,5)]]
        self.assertEqual(len({index.task_id(key) for key in edge}),1)
        self.assert_full_equal(m)

    def test_unselected_physical_surface_is_not_lost_between_cuts(self):
        # A coarse diagnostic slice spacing need not expose every surface as G.
        m=FixedLayerMap(MapSettings(slice_dh=2.,robot_radius=0.,safety_margin=0.,maturity_min_points=1))
        m.integrate(floor((.1,.8),hole={(0,0)},bound=3));m.build()
        heights={round(m.frontier_index.nodes[key].height,2)
                 for key in m.frontier_index.by_column[(0,0)]}
        self.assertEqual(heights,{.1,.8})
        self.assert_full_equal(m)

    def test_multi_commit_growth_and_obstacle_changes_match_full_rebuild(self):
        m=create(floor(hole={(0,0),(1,0),(2,0)}))
        for points in (np.asarray([[0,0,0.]],dtype='float32'),
                       np.asarray([[0,.3,.3]],dtype='float32'),
                       floor(bound=12),np.asarray([[0,0,1.5]],dtype='float32')):
            m.integrate(points);m.build();self.assert_full_equal(m)

    def test_deleting_a_frontier_bridge_splits_its_component(self):
        hole={(x,0) for x in range(-2,3)}
        m=create(floor(hole=hole));index=m.frontier_index
        self.assert_slice_clusters(index,hole,1)
        m.integrate(np.asarray([[0,0,0.]],dtype='float32'));m.build()
        self.assert_slice_clusters(index,hole,2)
        self.assert_full_equal(m)

        # Exercise a corrected observation becoming unknown again. Static
        # production never expires evidence, but the index must handle removal.
        for _,row in m.layers_at(0,0):row['G']=np.nan
        del m.columns[(0,0)]
        index.update({(0,0)})
        self.assert_slice_clusters(index,hole,1)
        self.assert_full_equal(m)

    def test_neighbour_terrain_becoming_blocked_updates_its_frontiers(self):
        m=create(floor())
        m.integrate(np.asarray([[0.,1.,.3]],dtype='float32'));m.build()
        self.assert_full_equal(m)

    def test_local_commit_does_not_rescan_map_and_keeps_far_cluster_id(self):
        m=create(np.vstack((floor(hole={(0,0)}),floor(hole={(0,0)})+[20,0,0])))
        index=m.frontier_index;key=next(iter(index.by_column[(200,0)]));cid=index.membership[key]
        version=index.clusters[cid]['revision']
        with patch.object(m,'iter_cells',side_effect=AssertionError('full map scan')):
            m.integrate(np.asarray([[0,0,0.]],dtype='float32'));m.build()
            index.query()
        self.assertEqual(index.membership[key],cid)
        self.assertEqual(index.clusters[cid]['revision'],version)
        self.assertLess(index.last_work['checked_columns'],100)

    def disconnected_rooms(self):
        points=np.asarray([(x*.1,y*.1,z) for x in range(-20,91) for y in range(-15,16)
                           for z in (0.,1.5) if z or (x<=20 or x>=50) and (x,y)!=(70,0)],dtype='float32')
        return create(points)

    def test_disconnected_floor_frontier_is_retired_only_after_global_review(self):
        m=self.disconnected_rooms();index=m.frontier_index
        base=(0.,0.,0.);band=(0.,.08);gap=(70,0)
        self.assertTrue(m.connectivity.possible((0,0),(70,1)))
        self.assertFalse(m.floor_connectivity(band).possible((0,0),(70,1)))

        def observes_remote(members,*args,**kwargs):
            self.assertFalse(any(key[1:3]==gap for key in members))
            return []

        with patch.object(index,'_observers',side_effect=observes_remote):
            first=index.query(base,floor=band,prune_disconnected=True)
        self.assertEqual(first['disconnected_floor_frontier_cells'],0)
        for cid in first['no_view_clusters']:
            entry=next(c for c in first['clusters'] if c['id']==cid)
            self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True,
                failure_reason=entry.get('floor_failure')),'first_no_progress')
        self.assertTrue(all(index._floor_records(index.floor_failures,band,key)
                            for key in index.by_column[gap] if index.nodes[key].height<.08))
        self.assertFalse(any(index._disconnected_on_floor(band,key) for key in index.by_column[gap]))
        with patch.object(index,'_observers',side_effect=observes_remote):
            review=index.query(base,floor=band,prune_disconnected=True)
        self.assertTrue(review['reviewing'])
        for cid in review['no_view_clusters']:
            entry=next(c for c in review['clusters'] if c['id']==cid)
            self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True,
                failure_reason=entry.get('floor_failure')),'blacklisted')
        self.assertTrue(all(index._disconnected_on_floor(band,key)
                            for key in index.by_column[gap] if index.nodes[key].height<.08))
        new_gap=(100,0)
        m.integrate(np.asarray([(x*.1,y*.1,0.) for x in (*range(21,50),*range(91,111))
                                for y in range(-15,16) if (x,y)!=new_gap],dtype='float32'))
        m.build()
        self.assertTrue(m.floor_connectivity(band).possible((0,0),(70,1)))
        with patch.object(index,'_observers',side_effect=observes_remote) as visibility:
            updated=index.query(base,floor=band,prune_disconnected=True)
        self.assertGreater(updated['disconnected_floor_frontier_cells'],0)
        self.assertTrue(any(key[1:3]==new_gap for call in visibility.call_args_list for key in call.args[0]))
        self.assertIn(gap,index.by_column)

    def test_first_disconnection_keeps_one_review_when_the_map_connects(self):
        m=self.disconnected_rooms();index=m.frontier_index
        base,band,gap=(0.,0.,0.),(0.,.08),(70,0)
        with patch.object(index,'_observers',return_value=[]):
            first=index.query(base,floor=band,prune_disconnected=True,review_remaining=8.)
        for cid in first['no_view_clusters']:
            entry=next(c for c in first['clusters'] if c['id']==cid)
            index.settle_floor_attempt(index.begin_floor_attempt(cid),True,
                                      failure_reason=entry.get('floor_failure'))
        gap_id=index.task_id(next(key for key in index.by_column[gap] if index.nodes[key].height<.08))
        self.assertTrue(any(c['id']==gap_id and c['floor_failure']=='floor_missing_connection'
                            for c in first['clusters']))
        with patch.object(index,'_observers',return_value=[]) as before:
            waiting=index.query(base,floor=band,prune_disconnected=True,review_remaining=8.)
        self.assertGreater(waiting['missing_connection_clusters'],0)
        self.assertFalse(any(key[1:3]==gap for call in before.call_args_list for key in call.args[0]))
        m.integrate(np.asarray([(x*.1,y*.1,0.) for x in range(21,50)
                                for y in range(-15,16)],dtype='float32'));m.build()
        self.assertTrue(m.floor_connectivity(band).possible((0,0),(70,1)))
        with patch.object(index,'_observers',return_value=[]) as observers:
            for _ in range(3):
                result=index.query(base,floor=band,prune_disconnected=True,review_remaining=8.)
                if result['reviewing']:break
                for cid in result['no_view_clusters']:
                    index.settle_floor_attempt(index.begin_floor_attempt(cid),True)
        self.assertTrue(result['reviewing'])
        self.assertTrue(any(key[1:3]==gap for call in observers.call_args_list for key in call.args[0]))
        cid=index.task_id(next(key for key in index.by_column[gap] if index.nodes[key].height<.08))
        self.assertTrue(index.begin_floor_attempt(cid).review)
        self.assertFalse(index._disconnected_on_floor(band,next(iter(index.by_column[gap]))))

    def test_tiny_clusters_get_two_attempts_and_local_queries_cannot_start_review(self):
        holes={(0,0),(5,0),(6,0)}
        wall=np.asarray([(x*.1,y*.1,.3) for x in range(-16,17) for y in range(-16,17)
                         if abs(x)==16 or abs(y)==16],dtype='float32')
        m=create(np.vstack((floor(hole=holes,bound=15),wall)));index=m.frontier_index
        base,band=(0.,0.,0.),(0.,.08)
        with patch.object(index,'_observers',return_value=[]) as observers:
            first=index.query(base,floor=band,fast=True)
        self.assertEqual(sorted(c['cells'] for c in first['clusters']),[1,2])
        self.assertEqual(sorted(call.kwargs['minimum_gain'] for call in observers.call_args_list),[1,2])
        for cid in first['no_view_clusters']:
            self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True),'first_no_progress')
        local=index.query(base,window_size=(.8,.2,.2),floor=band,fast=True)
        self.assertFalse(local['reviewing'])
        self.assertFalse(local['clusters'])
        self.assertEqual(local['pending_review_clusters'],2)
        with patch.object(index,'_observers',return_value=[]):
            review=index.query(base,floor=band,fast=True)
        self.assertTrue(review['reviewing'])
        self.assertEqual(sorted(c['cells'] for c in review['clusters']),[1,2])
        for cid in review['no_view_clusters']:
            self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True),'blacklisted')
        done=index.query(base,floor=band,fast=True)
        self.assertEqual(done['scoped_frontier_cells'],0)
        self.assertFalse(done['pending_review_clusters'] or done['query_pending'])
        self.assertTrue(all(xy in index.by_column for xy in holes),'retiring tasks must preserve map evidence')

    def test_bounded_review_keeps_small_tasks_until_an_actual_failure(self):
        holes={(0,0),(6,0)}
        wall=[(x*.1,y*.1,.3) for x in range(-16,17) for y in range(-16,17)
              if abs(x)==16 or abs(y)==16]
        m=create(np.vstack((floor(hole=holes,bound=15),wall)))
        m.settings=replace(m.settings,floor_failed_small_cluster_max_cells=2)
        index=m.frontier_index
        base,band=(0.,-.5,0.),(0.,.08)
        with patch.object(index,'_observers',return_value=[]) as observers:
            first=index.query(base,floor=band,fast=True,review_remaining=8.)
            for cid in first['no_view_clusters']:
                index.settle_floor_attempt(index.begin_floor_attempt(cid),True)
            expired=index.query(base,floor=band,fast=True,review_remaining=0.)
            self.assertEqual(expired['deferred_review_clusters'],2)
            self.assertFalse(expired['query_pending'])
            self.assertEqual(expired['small_skipped_cells'],0)
            second=index.query(base,floor=band,fast=True,review_remaining=8.)
            self.assertEqual(sorted(c['cells'] for c in second['clusters']),[1,1])
            self.assertEqual([c.kwargs['minimum_gain'] for c in observers.call_args_list],[1]*4)
            failed,timed_out=second['no_view_clusters']
            self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(failed),True,
                failure_reason='floor_no_observation'),'blacklisted')
            self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(timed_out),True,
                failure_reason='floor_observer_timeout'),'review_timed_out')
            done=index.query(base,floor=band,fast=True,review_remaining=8.)
        self.assertEqual(done['small_skipped_cells'],1)
        self.assertEqual(done['deferred_review_clusters'],1)
        self.assertTrue(all(xy in index.by_column for xy in holes))

    def test_diagnostic_requires_a_sealed_measured_barrier(self):
        import time
        from legbot_exploration.map_evidence import SurfaceColumn
        m=create(floor(bound=8))
        band=(0.,.08)
        connectivity=m.floor_connectivity(band)
        ring={(x,y) for x in range(3,8) for y in range(-2,3)
              if x in (3,7) or y in (-2,2)}
        with patch.object(connectivity,'node',return_value=None), patch.object(
                m.surface_relations,'measured_floor_barrier',side_effect=lambda xy,*a: xy in ring):
            self.assertTrue(connectivity.known_barrier((0,0),[(5,0)],time.monotonic()+1.))
            ring.remove((3,0))
            self.assertFalse(connectivity.known_barrier((0,0),[(5,0)],time.monotonic()+1.))
            self.assertFalse(connectivity.known_barrier((0,0),[(5,0)],time.monotonic()-1.))
        wall=SurfaceColumn()
        m.columns[(3,0)]=wall
        for z in np.arange(.05,.56,.05):
            wall.add(float(z),.05,1,1,orientation=-2)
        self.assertTrue(m.surface_relations.measured_floor_barrier((3,0),0.,0))
        self.assertFalse(m.surface_relations.measured_floor_barrier((3,0),0.,1))

    def test_known_barrier_retires_only_the_affected_task_on_its_first_attempt(self):
        m=self.disconnected_rooms()
        index=m.frontier_index
        base,band=(0.,0.,0.),(0.,.08)
        connectivity=m.floor_connectivity(band)
        remote=next(key for key in index.by_column[(70,0)] if index.nodes[key].height<.08)
        cid=index.task_id(remote)
        with patch.object(index,'_observers',return_value=[]), patch.object(
                connectivity,'known_barrier',side_effect=lambda start,stations,deadline:
                bool(stations) and all(x>=45 for x,y in stations)):
            first=index.query(base,floor=band,prune_disconnected=True,review_remaining=8.)
        row=next(c for c in first['clusters'] if c['id']==cid)
        self.assertEqual(row['floor_failure'],'floor_known_barrier')
        self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True,
            failure_reason=row['floor_failure']),'blacklisted')
        self.assertTrue(index._disconnected_on_floor(band,remote))
        self.assertIn(remote,index.nodes)
        with patch.object(index,'_observers',return_value=[]):
            after=index.query(base,floor=band,prune_disconnected=True,review_remaining=8.)
        self.assertFalse(any(c['id']==cid for c in after['clusters']))

    def test_failed_small_clusters_are_dropped_only_after_first_attempt(self):
        holes={(0,0),(1,0),(8,0),(9,0),(10,0)}
        wall=[(x*.1,y*.1,.3) for x in range(-16,17) for y in range(-16,17)
              if abs(x)==16 or abs(y)==16]
        m=create(np.vstack((floor(hole=holes,bound=15),wall)))
        m.settings=replace(m.settings,floor_failed_small_cluster_max_cells=2)
        index=m.frontier_index
        base,band=(0.,0.,0.),(0.,.08)
        with patch.object(index,'_observers',return_value=[]) as observers:
            local=index.query(base,window_size=(.25,.2,.2),floor=band)
            self.assertEqual([c['cells'] for c in local['clusters']],[2])
            self.assertEqual(local['small_skipped_clusters'],0)
            small=local['no_view_clusters'][0]
            index.settle_floor_attempt(index.begin_floor_attempt(small),True)
            first=index.query(base,floor=band)
            self.assertEqual([c['cells'] for c in first['clusters']],[3])
            self.assertFalse(first['reviewing'])
            self.assertEqual((first['small_skipped_clusters'],first['small_skipped_cells']),(1,2))
            large=first['no_view_clusters'][0]
            index.settle_floor_attempt(index.begin_floor_attempt(large),True)
            # The local window sees two cells from a failed three-cell cluster.
            clipped=index.query((.8,0.,0.),window_size=(.11,.2,.2),floor=band)
            self.assertEqual(clipped['small_skipped_clusters'],0)
            review=index.query(base,floor=band)
            self.assertTrue(review['reviewing'])
            self.assertEqual([c['cells'] for c in review['clusters']],[3])
            m.settings=replace(m.settings,floor_global_review_enabled=False)
            done=index.query(base,floor=band)
        self.assertEqual(observers.call_count,3)
        self.assertFalse(done['query_pending'] or done['pending_review_clusters'])
        self.assertEqual(done['deferred_floor_frontier_cells'],5)
        self.assertTrue(all(xy in index.by_column for xy in holes))

    def test_four_direction_search_resumes_after_each_rejected_route(self):
        m=create(floor(hole={(0,0)},bound=10))
        m.settings=replace(m.settings,floor_four_direction_search=True)
        index=m.frontier_index
        cache=index.observer_cache
        candidates=np.asarray([(x,y,0.,1.,3.,1.) for x,y in
                               ((-.5,-.5),(-.5,.5),(.5,-.5),(.5,.5))]+[(-.4,-.4,0.,1.,10.,1.)])
        tested=[]

        def find_poses(body,*args,**kwargs):
            tested.append(body[:2])
            return dict(views={body:dict(gain=1,yaw=0.)},rays={},unknown_blockers=[])

        options=(find_poses,lambda *args:(.5,.5,0),.15,2.,4.,.4)
        args=dict(base=(0.,0.,0.),floor=(0.,.08),window_size=(.75,.2,.2),floor_selection=options)
        with (patch.object(cache,'candidates',return_value=candidates),
              patch.object(index.observation_model,'_visibility_view',return_value=dict(gain=1))):
            for _ in range(4):
                result=index.query(**args)
                cluster,=result['clusters']
                self.assertTrue(cluster['direction_limited'])
                pose,=cluster['observations']
                self.assertTrue(index.reject_floor_view(cluster['id'],pose,False,()))
            final=index.query(**args)
        self.assertEqual(len(tested),4)
        self.assertEqual(set(tested),{(-.5,-.5),(-.5,.5),(.5,-.5),(.5,.5)})
        self.assertEqual(final['no_view_clusters'],[cluster['id']])
        self.assertEqual(final['clusters'][0]['floor_failure'],'floor_no_route')

    def test_new_local_ground_reopens_a_failed_frontier_but_distant_updates_do_not(self):
        m=create(floor(hole={(0,0),(6,0)},bound=12))
        index=m.frontier_index;band=(0.,.08)
        with patch.object(index,'_observers',return_value=[]):
            for _ in range(2):
                result=index.query((0.,-.5,0.),floor=band)
                for cid in result['no_view_clusters']:
                    index.settle_floor_attempt(index.begin_floor_attempt(cid),True,
                                               failure_reason='floor_no_observation')
        keys=set(index.by_column[(0,0)])
        self.assertTrue(all(index._floor_records(index.floor_blacklist,band,key) for key in keys))
        with patch.object(m,'iter_cells',side_effect=AssertionError('whole-map scan')):
            m.integrate([(8.,8.,0.)]);m.build()
            self.assertTrue(all(index._floor_records(index.floor_blacklist,band,key) for key in keys))
            m.integrate([(.6,0.,0.)]);m.build()
        remaining=set(index.by_column[(0,0)])
        self.assertTrue(remaining)
        self.assertTrue(all(not index._floor_records(index.floor_blacklist,band,key) for key in remaining))
        self.assertTrue(index.floor_work_available(band))
        self.assertGreater(index.floor_reopened,0)

    def test_direction_representative_prefers_observed_room_over_a_cheaper_edge(self):
        m=create(floor(hole={(0,0)},bound=12))
        m.settings=replace(m.settings,floor_four_direction_search=True)
        index=m.frontier_index;cache=index.observer_cache;tested=[]
        candidates=np.array([(.5,.5,0.,1.,0.,.4),(.7,.7,0.,1.,5.,1.)])
        def observe(body,*args,**kwargs):
            tested.append(tuple(body[:2]))
            return dict(views={body:dict(gain=1,yaw=0.)},rays={},unknown_blockers=[])
        options=(observe,lambda *args:(.55,.85,0),.15,1.5,1.,.2)
        with (patch.object(cache,'candidates',return_value=candidates),
              patch.object(index.observation_model,'_visibility_view',return_value=dict(gain=1))):
            index.query((0.,0.,0.),floor=(0.,.08),window_size=(1.,.2,.2),floor_selection=options)
        self.assertEqual(tested,[(.7,.7)])

    def test_four_direction_fallback_timeout_accumulates_across_batches(self):
        m=create(floor(hole={(0,0)},bound=10))
        m.settings=replace(m.settings,floor_four_direction_search=True,floor_direction_search_timeout=.5)
        index=m.frontier_index
        now=50.

        def search(*args,**kwargs):
            nonlocal now
            self.assertLessEqual(kwargs['direction_deadline']-now,.5)
            index.observer_cache.pending=True
            now+=.3
            return []

        args=dict(base=(0.,0.,0.),floor=(0.,.08),window_size=(.75,.2,.2),
                  query_budget=.2,floor_selection=(None,None,.15,2.,4.,.4))
        with (patch('legbot_exploration.pct_frontier_index.time.monotonic',side_effect=lambda:now),
              patch.object(index,'_observers',side_effect=search) as observers):
            self.assertTrue(index.query(**args)['query_pending'])
            now+=1.
            final=index.query(**args)
        self.assertEqual(observers.call_count,2)
        self.assertEqual(final['clusters'][0]['floor_failure'],'floor_observer_timeout')

    def test_split_clusters_inherit_failure_across_map_and_cut_updates(self):
        holes={(x,0) for x in range(5)}
        wall=np.asarray([(x*.1,y*.1,.3) for x in range(-16,17) for y in range(-16,17)
                         if abs(x)==16 or abs(y)==16],dtype='float32')
        m=create(np.vstack((floor(hole=holes,bound=15),wall)));index=m.frontier_index
        base,band=(0.,0.,0.),(0.,.08)
        with patch.object(index,'_observers',return_value=[]):
            first=index.query(base,window_size=(.8,.2,.2),floor=band)
        self.assertEqual(len(first['no_view_clusters']),1)
        cid=first['no_view_clusters'][0]
        self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True),'first_no_progress')
        m.integrate(np.asarray([[.2,0.,0.],[3.,3.,2.]],dtype='float32'));m.build()
        with patch.object(index,'_observers',return_value=[]):
            review=index.query(base,floor=band)
        self.assertTrue(review['reviewing'])
        self.assertEqual(len(review['no_view_clusters']),2)
        for cid in review['no_view_clusters']:
            attempt=index.begin_floor_attempt(cid)
            self.assertTrue(attempt.review)
            self.assertEqual(index.settle_floor_attempt(attempt,True),'blacklisted')

    def test_cross_layer_reuses_frontier_queue_with_four_attempts(self):
        holes={(0,0),(5,0),(6,0)}
        wall=np.asarray([(x*.1,y*.1,.3) for x in range(-16,17) for y in range(-16,17)
                         if abs(x)==16 or abs(y)==16],dtype='float32')
        mapping=create(np.vstack((floor(hole=holes,bound=15),wall)))
        index=mapping.frontier_index
        base,band=(0.,0.,0.),(0.,.08)
        options=dict(window_size=(.8,.2,.2),floor=band,continuation=(0.,-.02,.10),
                     attempt_scope=('cross_layer',1),max_attempts=4)
        with patch.object(index,'_observers',return_value=[]):
            for number,outcome in enumerate(
                    ('first_no_progress','deferred','deferred','blacklisted'),1):
                result=index.query(base,**options)
                self.assertEqual(len(result['no_view_clusters']),2)
                self.assertEqual({c['attempt'] for c in result['clusters']},{number})
                for cid in result['no_view_clusters']:
                    self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True),outcome)
            self.assertFalse(index.query(base,**options)['clusters'])
            self.assertFalse(index.query(base,floor=band)['clusters'])

    def test_platform_handoff_keeps_physical_failure_history_and_other_heights_separate(self):
        wall=np.asarray([(x*.1,y*.1,.3) for x in range(-16,17) for y in range(-16,17)
                         if abs(x)==16 or abs(y)==16],dtype='float32')
        mapping=create(np.vstack((floor((0.,1.5),hole={(0,0)},bound=15),wall)))
        index=mapping.frontier_index
        key=next(k for k in index.by_column[(0,0)] if index.nodes[k].height<.08)
        cid=index.task_id(key)
        with patch.object(index,'_observers',return_value=[]):
            index.query((-.5,0.,0.),floor=(0.,.05),continuation=(0.,-.02,.1),
                        attempt_scope=('cross_layer',(0.,.08)),max_attempts=4)
            self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True),'first_no_progress')
            platform=index.query((-.5,0.,0.),floor=(0.,.05))
            self.assertTrue(platform['reviewing'])
            attempt=index.begin_floor_attempt(cid)
            self.assertEqual(attempt.failures,1)
            self.assertEqual(index.settle_floor_attempt(attempt,True),'blacklisted')
            ordinary=index.query((-.5,0.,0.),floor=(.01,.08))
        self.assertNotIn(cid,[c['id'] for c in ordinary['clusters']])
        upper=[k for k in index.nodes if index.nodes[k].height>1.]
        self.assertTrue(upper)
        self.assertTrue(all(not index._floor_records(index.floor_blacklist,(1.5,.08),k) for k in upper))

    def test_global_review_continues_after_two_observer_slices_but_remains_bounded(self):
        base,band=(0.,0.,0.),(0.,.08)
        viewpoint=(.6,0.,0.)
        for available in (True,False):
            with self.subTest(viewpoint_available=available):
                mapping=create(floor(bound=10))
                index=mapping.frontier_index
                with patch.object(index,'_observers',return_value=[]):
                    first=index.query(base,floor=band)
                self.assertTrue(first['no_view_clusters'])
                for cid in first['no_view_clusters']:
                    self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True),
                                     'first_no_progress')

                def search(*args,**kwargs):
                    ready=available and calls.call_count==3
                    index.observer_cache.pending=not ready
                    index.observer_cache.last_views={viewpoint:dict(gain=3,yaw=0.)} if ready else {}
                    return [list(viewpoint)] if ready else []

                with patch.object(index,'_observers',side_effect=search) as calls:
                    for _ in range(2 if available else 3):
                        result=index.query(base,query_budget=.2,floor=band)
                        self.assertTrue(result['reviewing'] and result['query_pending'])
                        self.assertFalse(result['timed_out_clusters'])
                    final=index.query(base,query_budget=.2,floor=band)
                self.assertEqual(calls.call_count,3 if available else 4)
                if available:
                    self.assertTrue(any(c['observations']==[list(viewpoint)] for c in final['clusters']))
                    self.assertFalse(final['timed_out_clusters'])
                else:
                    self.assertTrue(final['timed_out_clusters'])

    def test_floor_observer_budgets_count_work_across_ticks(self):
        mapping=create(floor(bound=10))
        index=mapping.frontier_index
        base,band=(0.,0.,0.),(0.,.08)
        options=(None,None,.15,2.,4.,.40)
        now=100.

        def search(*args,**kwargs):
            nonlocal now
            index.observer_cache.pending=True
            now+=.75
            return []

        with (patch('legbot_exploration.pct_frontier_index.time.monotonic',side_effect=lambda:now),
              patch.object(index,'_observers',side_effect=search)):
            for review,rounds in ((False,3),(True,6)):
                for step in range(rounds):
                    now+=1.  # Waiting for the next controller tick does not consume search time.
                    result=index.query(base,query_budget=.2,floor=band,floor_selection=options)
                    self.assertEqual(result['reviewing'],review)
                    if step<rounds-1:
                        self.assertTrue(result['query_pending'])
                        self.assertFalse(result['no_view_clusters'])
                self.assertEqual(len(result['no_view_clusters']),1)
                cid=result['no_view_clusters'][0]
                entry=next(c for c in result['clusters'] if c['id']==cid)
                self.assertEqual(entry['floor_failure'],'floor_observer_timeout')
                self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True),
                                 'blacklisted' if review else 'first_no_progress')

    def test_local_observer_yields_to_another_cluster_without_losing_its_budget(self):
        index=create(floor(hole={(0,0),(10,0)},bound=20)).frontier_index
        options=(None,None,.15,2.,4.,.40)
        viewpoint=(.6,0.,0.)
        now=100.
        calls=[]

        def search(members,*args,**kwargs):
            nonlocal now
            calls.append(frozenset(members))
            now+=.25
            index.observer_cache.pending=len(calls)==1
            index.observer_cache.last_views={viewpoint:dict(gain=1,yaw=0.)}
            return [] if len(calls)==1 else [list(viewpoint)]

        args=dict(base=(0.,0.,0.),floor=(0.,.08),window_size=(3.,1.,1.),query_budget=.2,floor_selection=options)
        with (patch('legbot_exploration.pct_frontier_index.time.monotonic',side_effect=lambda:now),
              patch.object(index,'_observers',side_effect=search)):
            initial=index.query(**args)
            self.assertTrue(initial['query_pending'])
            self.assertFalse(initial['no_view_clusters'])
            completed=index.query(**args)
        self.assertEqual(len(calls),2)
        self.assertTrue(calls[0].isdisjoint(calls[1]))
        self.assertTrue(any(cluster['observations'] for cluster in completed['clusters']))
        suspended=next(s for s in index.floor_view_searches.values() if frozenset(s.members)==calls[0])
        self.assertAlmostEqual(suspended.elapsed,.25)

    def test_near_observers_are_regenerated_and_completion_survives_unrelated_changes(self):
        m=create(floor(hole={(0,0)},bound=20));index=m.frontier_index
        cid=index.task_id(next(iter(index.by_column[(0,0)])))
        cluster=index.clusters[cid]
        # Keep the complete safe candidate pool; distance filtering at query
        # time must still find alternatives to the initially preferred views.
        found=next(c for c in index.query(np.zeros(3),.5)['clusters'] if c['id']==cid)
        self.assertTrue(found['observations'])
        self.assertTrue(all(np.linalg.norm(p[:2])>=.5 for p in found['observations']))
        chosen=found['observations'][0];version=cluster['revision']
        index.complete(cid,version,chosen)
        m.integrate(np.asarray([[20,0,0.]],dtype='float32'));m.build()
        cluster=index.clusters[cid]
        self.assertEqual(cluster['revision'],version)
        self.assertTrue(cluster['completed'])
        found=next(c for c in index.query()['clusters'] if c['id']==cid)
        self.assertTrue(all(np.linalg.norm(np.asarray(p)-chosen)>.25 for p in found['observations']))

    def test_equivalent_cuts_keep_distinct_costs(self):
        m=create(floor(hole={(0,0)}));index=m.frontier_index
        layers=sorted(k for k,_ in m.layers_at(1,0) if np.isfinite(m.cell((k,1,0))['G']))
        self.assertGreater(len(layers),1)
        m.cell((layers[-1],1,0))['inflated_terrain_cost']=19.
        index.update({(1,0)})
        values=[(key[0],source[4]) for key in index.by_column[(0,0)]
                for source in index.source_states(key) if source[:2]==(1,0)]
        self.assertIn((layers[-1],19.),values)
        self.assertTrue(any(cost<19. for _,cost in values))
        self.assert_full_equal(m)

    def test_remote_change_in_the_same_component_keeps_completed_position(self):
        m=create(floor(bound=30));index=m.frontier_index
        cid=next(iter(index.clusters));cluster=index.clusters[cid]
        chosen=cluster['observations'][0];key=min(cluster['members'])
        index.complete(cid,cluster['revision'],chosen)
        x=-3.1 if chosen[0]>0 else 3.1;y=-3.1 if chosen[1]>0 else 3.1
        m.integrate(np.asarray([[x,y,0.]],dtype='float32'));m.build()
        self.assertEqual(index.membership[key],cid)
        self.assertIn(tuple(chosen),index.completed_points)
        self.assertTrue(all(np.linalg.norm(np.asarray(p)-chosen)>.25
                            for p in index.clusters[cid]['observations']))

    def test_another_floor_does_not_clear_completed_observation(self):
        m=create(floor((0.,1.5)));index=m.frontier_index
        cid=next(cid for cid,c in index.clusters.items() if abs(c['frontier'][2])<.01)
        cluster=index.clusters[cid];chosen=cluster['observations'][0]
        index.complete(cid,cluster['revision'],chosen)
        m.integrate(np.asarray([[1.1,1.1,1.5]],dtype='float32'));m.build()
        self.assertIn(tuple(chosen),index.completed_points)

    def test_later_hits_on_frozen_ground_do_not_recheck_frontier_neighbours(self):
        m=create(floor(hole={(0,0)}),num=3);index=m.frontier_index
        m.integrate(np.asarray([[0,0,0.]]*3,dtype='float32'));m.build()
        with ExitStack() as mocks:
            for layer in index.slices.values():
                mocks.enter_context(patch.object(layer,'_boundaries',side_effect=AssertionError('unchanged boundary update')))
            m.integrate(np.asarray([[0,0,0.]],dtype='float32'));m.build()
        self.assertEqual(index.last_work['checked_columns'],0)
        self.assertEqual(index.last_work['relabelled_tiles'],0)
        self.assert_full_equal(m)

    def test_height_change_without_edge_change_keeps_component_labels(self):
        m=create(floor(hole={(0,0)}));index=m.frontier_index
        previous={key:index.membership[key] for key in index.by_column[(0,0)]}
        changed={(x,y) for x in (-1,0,1) for y in (-1,0,1) if x or y}
        for x,y in changed:
            for _,row in m.layers_at(x,y):
                if row['g_mature']:row['G']+=.01
        index.update(changed)
        self.assertGreater(index.last_work['changed_frontier_layers'],0)
        self.assertEqual(index.last_work['topology_changed_layers'],0)
        self.assertEqual(index.last_work['relabelled_tiles'],0)
        self.assertEqual(previous,{key:index.membership[key] for key in index.by_column[(0,0)]})
        self.assert_full_equal(m)

    def test_map_commits_never_search_for_observation_positions(self):
        m=create(floor(hole={(0,0)}));index=m.frontier_index
        with patch.object(index,'_observers',side_effect=AssertionError('eager observer search')):
            m.integrate(np.asarray([[0,0,0.],[0,.3,.3]],dtype='float32'));m.build()
        self.assert_full_equal(m)

    def test_soft_cost_updates_do_not_recluster_or_change_boundary_members(self):
        m=create(floor(hole={(0,0)},bound=15));index=m.frontier_index
        before=dict(index.nodes);index.query()
        with ExitStack() as mocks:
            for layer in index.slices.values():
                mocks.enter_context(patch.object(layer,'_boundaries',side_effect=AssertionError('cost changed membership')))
                mocks.enter_context(patch.object(layer,'_refresh',side_effect=AssertionError('cost rebuilt cluster')))
            for k,row in m.layers_at(1,0):row['inflated_terrain_cost']+=.5
            index.update({(1,0)},set())
        self.assertEqual(index.nodes,before)
        self.assertEqual(index.last_work['relabelled_tiles'],0)
        self.assertEqual(index.last_work['observer_invalidated'],0)
        self.assert_full_equal(m)

    def test_other_floor_and_remote_observer_geometry_remain_cached(self):
        points=np.vstack((floor((0.,1.5),hole={(0,0)}),floor(hole={(0,0)})+[20,0,0]))
        m=create(points);index=m.frontier_index;index.query()
        cached=dict(index.observer_cache.cache)
        self.assertTrue(cached)
        for k,row in m.layers_at(1,0):
            if row['g_mature'] and abs(row['G'])<.01:row['inflated_terrain_cost']+=.5
        index.update({(1,0)},set())
        unaffected={key:entry for key,entry in cached.items() if entry['heights'][0]>.5 or entry['bounds'][0][0]>100}
        self.assertTrue(unaffected)
        for key,entry in unaffected.items():self.assertIs(index.observer_cache.cache[key],entry)
        self.assert_full_equal(m)

    def test_motion_and_completion_reuse_geometry_and_filter_at_query_time(self):
        m=create(floor(hole={(0,0)},bound=20));index=m.frontier_index
        first=index.query(np.zeros(3),.5,(2.,1.,1.))
        self.assertGreater(first['observer_cache_built'],0)
        c=next(c for c in first['clusters'] if c['observations']);point=c['observations'][0]
        index.complete(c['id'],c['revision'],point)
        after=index.query(np.asarray([.05,0.,0.]),.5,(2.,1.,1.))
        self.assertEqual(after['observer_cache_built'],0)
        for c in after['clusters']:
            for p in c['observations']:
                self.assertGreater(np.linalg.norm(np.asarray(p[:2])-point[:2]),m.settings.observation_revisit_radius)

    def test_bridge_at_tile_boundary_splits_and_merges_in_each_cut(self):
        # Construct the hole in world cell coordinates across x=32.
        points=np.asarray([(x*.1,y*.1,0.) for x in range(20,45) for y in range(-8,9)
                           if not (28<=x<=36 and y==0)],dtype='float32')
        m=create(points);index=m.frontier_index
        hole={(x,0) for x in range(28,37)}
        self.assert_slice_clusters(index,hole,1)
        keys={key for xy in hole for key in index.by_column.get(xy,())}
        self.assertGreater(len(keys),len(hole))
        self.assertEqual(len({index.task_id(key) for key in keys}),1)
        self.assertEqual(len(index.physical_targets(keys)),len(hole))
        m.integrate(np.asarray([[3.2,0.,0.]],dtype='float32'));m.build()
        self.assert_slice_clusters(index,hole,2);self.assert_full_equal(m)
        for _,row in m.layers_at(32,0):row['G']=np.nan
        del m.columns[(32,0)];index.update({(32,0)})
        self.assert_slice_clusters(index,hole,1);self.assert_full_equal(m)

    def test_random_multilayer_batches_match_full_membership_and_connectivity(self):
        rng=np.random.default_rng(2718)
        points=np.asarray([(x*.1,y*.1,h) for x in range(-10,11) for y in range(-10,11)
                           for h in (0.,1.5) if rng.random()<.6],dtype='float32')
        rng.shuffle(points)
        m=create(points[:200])
        for batch in np.array_split(points[200:],7):
            m.integrate(batch);m.build();self.assert_full_equal(m)

    def test_local_batches_with_close_sheets_match_complete_rebuild(self):
        rng=np.random.default_rng(810)
        points=np.asarray([[x*.1,y*.1,.1 if x<0 else .24]
                           for x in range(-7,8) for y in range(-7,8)
                           if x and rng.random()<.8],dtype='float32')
        rng.shuffle(points)
        m=create(points[:70])
        for batch in np.array_split(points[70:],4):
            m.integrate(batch);m.build();self.assert_full_equal(m)

    def test_local_growth_relabels_only_tiles_not_a_large_component(self):
        m=create(floor(bound=50));index=m.frontier_index
        m.integrate(np.asarray([[5.1,0.,0.]],dtype='float32'));m.build()
        self.assertGreater(index.last_work['topology_changed_layers'],0)
        self.assertLess(index.last_work['reclustered_layers'],len(index.nodes)//2)
        self.assertLess(index.last_work['checked_columns'],30)
        self.assert_full_equal(m)


if __name__=='__main__':unittest.main()
