"""Lifecycle checks use mocks: no arm service or chassis is contacted."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

path = Path(__file__).resolve().parents[1]/'scripts/explore_supervisor.py'
spec = importlib.util.spec_from_file_location('supervisor', str(path))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SupervisorStart(unittest.TestCase):
    def setUp(self):
        with patch.multiple(module.rospy, get_param=lambda name, default: default,
                            Publisher=Mock(), Subscriber=Mock(), ServiceProxy=Mock(), Service=Mock()):
            self.node = module.Supervisor()
        self.node.latched = False
        self.node.safety_healthy = True
        self.node.bridge_enabled = True
        self.node.frontier_valid = True
        self.node.available = True
        self.node.frontier_since = 97.0
        self.fresh(100.0)

    def fresh(self, now):
        self.node.safety_stamp = self.node.status_stamp = self.node.frontier_stamp = now
        self.node.map_stamp = now-0.1
        self.node.refresh_ok = True

    def test_launch_auto_starts_only_when_ready(self):
        self.assertTrue(self.node.should_arm(100))
        self.assertFalse(self.node.can_start(100))  # wait for gate acknowledgement
        self.node.armed = True
        self.assertTrue(self.node.can_start(100))

    def test_diagnostic_mode_remains_locked(self):
        self.node.auto_start = False
        self.assertFalse(self.node.should_arm(100))
        self.node.available = False
        self.assertFalse(self.node.completed(100))

    def test_no_start_on_stale_missing_invalid_inputs(self):
        for field, bad in [('map_stamp', 0), ('safety_stamp', 98),
                           ('status_stamp', 98), ('frontier_stamp', 96),
                           ('frontier_valid', False), ('refresh_ok', False),
                           ('latched', True)]:
            with self.subTest(field=field):
                good = getattr(self.node, field)
                setattr(self.node, field, bad)
                self.assertFalse(self.node.should_arm(100))
                setattr(self.node, field, good)

    def test_no_automatic_rearm_after_stop(self):
        self.node.arm_requested = True
        self.assertFalse(self.node.should_arm(100))
        self.node.arm_requested = False
        self.node.ever_armed = True
        self.assertFalse(self.node.should_arm(100))

    def test_successful_auto_authorization_is_consumed_once(self):
        self.node.arm_service = Mock(return_value=SimpleNamespace(success=True))
        self.node.enable_service = Mock(return_value=SimpleNamespace(success=True))
        with patch.object(module.rospy, 'loginfo'):
            self.node.request_arm(100)
        self.assertTrue(self.node.arm_requested)
        self.assertFalse(self.node.should_arm(103))
        self.node.arm_service.assert_called_once_with()

    def test_sdk_enable_refusal_is_final_for_this_launch(self):
        self.node.enable_service = Mock(return_value=SimpleNamespace(success=False,message='battery too low'))
        self.node.arm_service = Mock()
        self.node.request_arm(100)
        self.assertTrue(self.node.startup_failed)
        self.assertFalse(self.node.should_arm(110))
        self.node.arm_service.assert_not_called()
        self.assertEqual(self.node.enable_service.call_args_list[0].args,(True,))
        self.assertEqual(self.node.enable_service.call_args_list[-1].args,(False,))

    def test_stop_during_enable_disables_and_never_arms(self):
        self.node.stop_requested = True
        self.node.enable_service = Mock(return_value=SimpleNamespace(success=True))
        self.node.arm_service = Mock()
        self.node.request_arm(100)
        self.assertTrue(self.node.startup_failed)
        self.node.arm_service.assert_not_called()
        self.node.enable_service.assert_called_with(False)

    def post_enable_clock(self, update):
        clock=[0.0]
        self.node.post_enable_settle=5.;self.node.post_enable_stable=3.;self.node.post_enable_timeout=20.
        self.node.data_ready=Mock(return_value=True)
        self.node.publish_state=Mock()
        self.node.enable_service=Mock(return_value=SimpleNamespace(success=True))
        self.node.arm_service=Mock(return_value=SimpleNamespace(success=True))
        def sleep(seconds):
            self.node.arm_service.assert_not_called()
            self.assertFalse(self.node.armed)
            clock[0]+=seconds;update(clock[0])
        return clock,sleep

    def test_real_preparation_waits_for_continuous_fresh_health_before_arming(self):
        clock,sleep=self.post_enable_clock(lambda now:setattr(self.node,'safety_healthy',not 6<=now<7))
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(module.time,'sleep',side_effect=sleep),patch.object(module.rospy,'loginfo'):
            self.node.request_arm(0)
        self.assertGreaterEqual(clock[0],10.)
        self.assertLess(clock[0],10.2)
        self.node.arm_service.assert_called_once_with()
        self.assertEqual(self.node.enable_service.call_count,1)

    def test_control_loss_during_preparation_never_reenables(self):
        clock,sleep=self.post_enable_clock(lambda now:setattr(self.node,'bridge_enabled',now<1.))
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(module.time,'sleep',side_effect=sleep):
            self.node.request_arm(0)
        self.node.arm_service.assert_not_called()
        self.assertTrue(self.node.startup_failed)
        self.assertFalse(self.node.should_arm(100))
        self.assertEqual([x.args for x in self.node.enable_service.call_args_list],[(True,),(False,)])

    def test_unstable_preparation_times_out_disabled_without_arming(self):
        clock,sleep=self.post_enable_clock(lambda now:setattr(self.node,'safety_healthy',False))
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(module.time,'sleep',side_effect=sleep):
            self.node.request_arm(0)
        self.node.arm_service.assert_not_called()
        self.assertTrue(self.node.startup_failed)
        self.node.enable_service.assert_called_with(False)

    def test_operator_stop_during_settling_never_arms(self):
        clock,sleep=self.post_enable_clock(lambda now:setattr(self.node,'stop_requested',now>=1.))
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(module.time,'sleep',side_effect=sleep):
            self.node.request_arm(0)
        self.node.arm_service.assert_not_called()
        self.node.enable_service.assert_called_with(False)

    def test_completion_requires_time_and_two_new_maps(self):
        self.node.available = False
        self.assertFalse(self.node.completed(100))
        self.fresh(131)
        self.node.map_generation = 1
        self.assertFalse(self.node.completed(131))
        self.node.map_generation = 2
        self.assertTrue(self.node.completed(131))

    def test_active_goal_frontiers_and_invalid_data_reset_completion(self):
        for field, bad in [('available', True), ('active', True),
                           ('frontier_valid', False), ('latched', True),
                           ('refresh_ok', False), ('safety_stamp', 80)]:
            with self.subTest(field=field):
                self.node.available = False
                self.node.empty_since = 60
                self.node.empty_generation = 0
                self.node.map_generation = 4
                good = getattr(self.node, field)
                setattr(self.node, field, bad)
                self.assertFalse(self.node.completed(100))
                self.assertIsNone(self.node.empty_since)
                setattr(self.node, field, good)

    def test_duplicate_ready_message_is_not_new_map(self):
        message = module.String(data='ready: one_generation')
        self.node.map_status(message)
        self.node.map_status(message)
        self.assertEqual(self.node.map_generation, 1)
        self.node.map_status(module.String(data='refresh failed: no data'))
        self.assertFalse(self.node.refresh_ok)

    def test_single_failed_map_does_not_cancel_active_exploration(self):
        self.node.armed = self.node.ever_armed = self.node.active = True
        self.node.child = Mock()
        self.node.stop_child = Mock()
        self.node.pause_service = Mock()
        self.node.finish = Mock()
        self.node.map_status(module.String(data='refresh failed: brief TF delay'))
        self.assertTrue(self.node.data_ready(100))
        self.assertFalse(self.node.completed(100))
        with patch.object(module.rospy, 'is_shutdown', side_effect=[False, True]), \
             patch.object(module.time, 'monotonic', return_value=100.), \
             patch.object(module.time, 'sleep'), patch.object(module.rospy, 'loginfo'):
            self.node.run()
        self.assertEqual(self.node.last_state, 'EXPLORING')
        self.node.pause_service.assert_not_called()
        self.node.stop_child.assert_not_called()

    def test_repeated_failures_do_not_refresh_map_age_and_still_expire(self):
        self.node.map_stamp = 99.9
        for now in (100., 100.5, 101., 101.89, 101.91):
            self.node.safety_stamp = self.node.status_stamp = self.node.frontier_stamp = now
            self.node.map_status(module.String(data='refresh failed: TF unavailable'))
            self.assertEqual(self.node.map_stamp, 99.9)
            self.assertEqual(self.node.data_ready(now), now < 101.9)
        self.assertIn('observation_map_stale', self.node.data_problem(101.91))

    def test_failed_attempt_cannot_start_or_complete_without_new_success(self):
        self.node.empty_since = 60
        self.node.map_status(module.String(data='refresh failed: TF unavailable'))
        self.assertFalse(self.node.should_arm(100))
        self.assertIsNone(self.node.empty_since)
        self.node.available = False
        self.assertFalse(self.node.completed(100))

    def test_map_grace_does_not_hide_stale_safety_or_invalid_frontier(self):
        self.node.map_status(module.String(data='refresh failed: brief TF delay'))
        self.node.safety_stamp = 98
        self.assertEqual(self.node.data_problem(100), 'safety_status_stale')
        self.node.safety_stamp = 100
        self.node.frontier_valid = False
        self.assertEqual(self.node.data_problem(100), 'frontier_invalid')

    def test_online_memory_staleness_blocks_without_waiting_45_seconds(self):
        self.node.map_timeout = 2.0
        self.node.map_stamp = 97.5
        self.assertFalse(self.node.data_ready(100))
        self.node.map_stamp = 99.0
        self.assertTrue(self.node.data_ready(100))

    def test_fault_parks_session_without_saving_or_exiting(self):
        self.node.latched = True
        self.node.stop_child = Mock()
        self.node.stop_service = Mock()
        self.node.save_service = Mock()
        with patch.object(module.rospy, 'loginfo'):
            self.node.hold_session(fault=True)
        self.assertEqual(self.node.last_state,'FAULT_STOPPED')
        self.node.stop_child.assert_called_once_with()
        self.node.stop_service.assert_not_called()
        self.node.save_service.assert_not_called()
        self.assertFalse(self.node.should_arm(100))

    def test_data_pause_requires_stable_recovery_and_no_active_goal(self):
        self.node.pause_service = Mock(return_value=SimpleNamespace(success=True))
        self.node.stop_child = Mock()
        self.node.resume_service = Mock(return_value=SimpleNamespace(success=True))
        self.node.armed = True
        with patch.object(module.rospy, 'loginfo'):
            self.node.hold_session(fault=False)
        self.assertTrue(self.node.paused)
        self.assertFalse(self.node.completed(100))
        self.assertFalse(self.node.resume_if_ready(100))
        self.fresh(103)
        self.assertTrue(self.node.resume_if_ready(103))
        self.assertFalse(self.node.paused)
        self.node.resume_service.assert_called_once_with()

    def test_obstacle_hold_cancels_producer_and_preserves_goal_cooldown(self):
        self.node.armed = self.node.ever_armed = True
        self.node.obstacle_hold = True
        self.node.obstacle_event = 1
        self.node.selected_goal = module.PoseStamped()
        self.node.selected_goal.header.frame_id = 'map'
        self.node.selected_goal.pose.position.x = 4.025
        self.node.stop_child = Mock()
        self.node.pause_service = Mock()
        self.node.arm_service = Mock()
        self.node.resume_service = Mock(return_value=SimpleNamespace(success=True))
        with patch.object(module.rospy, 'loginfo'), \
             patch.object(module.rospy.Time, 'now', return_value=module.rospy.Time(100)):
            self.node.hold_obstacle()
            self.node.hold_obstacle()
        self.assertEqual(self.node.last_state, 'WAITING_FOR_OBSTACLE')
        self.assertFalse(self.node.can_start(100))
        self.assertFalse(self.node.completed(100))
        self.assertFalse(self.node.resume_if_ready(100))
        self.node.pause_service.assert_not_called()
        self.node.arm_service.assert_not_called()
        self.node.stop_child.assert_called_once_with()
        self.assertEqual(len(self.node.recovery_blacklist), 1)
        self.assertEqual(self.node.recovery_blacklist[0]['until'], 130.)
        self.node.obstacle_hold = False
        self.assertFalse(self.node.resume_if_ready(100))
        self.fresh(103)
        self.assertTrue(self.node.resume_if_ready(103))
        with patch.object(module.rospy.Time, 'now', return_value=module.rospy.Time(103)), \
             patch.object(module.rospy, 'set_param') as param, \
             patch.object(module.subprocess, 'Popen'):
            self.node.start()
        param.assert_called_once_with('/explore/recovery_blacklist', self.node.recovery_blacklist)

    def test_hold_event_blocks_completion_even_if_clear_status_arrives_first(self):
        self.node.available = False
        self.node.obstacle_event = 1
        self.node.empty_since = 60
        self.assertFalse(self.node.completed(100))
        self.assertIsNone(self.node.empty_since)

    def test_valid_empty_is_distinct_from_missing_tf(self):
        self.node.frontier(module.String(data='INVALID'))
        self.assertFalse(self.node.frontier_valid)
        self.node.frontier(module.String(data='EMPTY'))
        self.assertTrue(self.node.frontier_valid)
        self.assertFalse(self.node.available)

    def test_finish_stops_before_snapshot_and_never_converts(self):
        order = []
        self.node.stop_service = Mock(side_effect=lambda: order.append('stop'))
        self.node.stop_child = Mock(side_effect=lambda: order.append('child'))
        self.node.save_service = Mock(side_effect=lambda: (order.append('snapshot') or
                                                          SimpleNamespace(success=True)))
        self.node.result = 'COMPLETED'
        with patch.object(module.rospy, 'is_shutdown', return_value=False), \
             patch.object(module.rospy, 'loginfo'), patch.object(module.subprocess, 'Popen') as spawn:
            self.node.finish()
        self.assertEqual(order, ['stop', 'child', 'snapshot'])
        self.assertEqual(self.node.result, 'COMPLETED')
        spawn.assert_not_called()

    def test_snapshot_failure_is_not_reported_as_success(self):
        self.node.stop_service = Mock()
        self.node.stop_child = Mock()
        self.node.save_service = Mock(return_value=SimpleNamespace(success=False, message='disk full'))
        self.node.result = 'COMPLETED'
        with patch.object(module.rospy, 'is_shutdown', return_value=False), \
             patch.object(module.rospy, 'loginfo'), patch.object(module.rospy, 'logerr'):
            self.node.finish()
        self.assertEqual(self.node.result, 'SAVE_FAILED')


adapter_path = path.parent/'frontier_costmap_adapter.py'
adapter_spec = importlib.util.spec_from_file_location('adapter', str(adapter_path))
adapter_module = importlib.util.module_from_spec(adapter_spec)
adapter_spec.loader.exec_module(adapter_module)


class FrontierEvidence(unittest.TestCase):
    def setUp(self):
        self.node = adapter_module.FrontierCostmapAdapter.__new__(adapter_module.FrontierCostmapAdapter)
        self.node.min_frontier_cells = 5
        self.node.robot_base_frame = 'base_link'
        self.node.tf_buffer = Mock()

    def test_missing_tf_is_invalid_not_empty(self):
        self.node.tf_buffer.lookup_transform.side_effect = adapter_module.tf2_ros.TransformException('missing')
        grid = adapter_module.OccupancyGrid()
        with patch.object(adapter_module.rospy, 'logwarn_throttle'):
            self.assertIsNone(self.node.has_reachable_frontier_cluster(grid))

    def test_physical_frontier_size_and_row_edges(self):
        self.assertFalse(self.node.has_large_cluster(set(range(9)), 20, required=10))
        self.assertTrue(self.node.has_large_cluster(set(range(10)), 20, required=10))
        # Right edge of one row must not connect to left edge of the next.
        self.assertFalse(self.node.has_large_cluster({9, 10}, 10, required=2))

    def test_rear_coverage_gap_prevents_empty_without_blocking_collision_map(self):
        collision = [100]*400
        reachable = {y*20+x for y in range(2,18) for x in range(2,18)}
        for i in reachable:
            collision[i]=0
        coverage = [0]*400
        rear = {y*20+x for y in range(4,16) for x in range(3,9)}
        for i in rear:
            coverage[i]=-1
        output,targets,n = self.node.coverage_frontiers(collision,20,20,reachable,coverage,.15,.05)
        self.assertEqual(n,len(rear))
        self.assertTrue(self.node.has_large_cluster(set(targets),20,10))
        self.assertTrue(all(collision[i]==0 and output[i]==-1 for i in rear))
        # After a real observation/traversal of the same area, completion can resume.
        output,targets,n = self.node.coverage_frontiers(collision,20,20,reachable,[0]*400,.15,.05)
        self.assertEqual(n,0)
        self.assertFalse(self.node.has_large_cluster(targets,20,10))

    def test_small_sampling_holes_and_disconnected_rooms_do_not_create_goals(self):
        collision = [0]*400
        coverage = [-1]*400
        reachable=set(range(10))
        _,targets,n=self.node.coverage_frontiers(collision,20,20,reachable,coverage,.15,.05)
        self.assertEqual((n,len(targets)),(0,0))

    def test_missing_coverage_invalidates_completion_even_with_fresh_collision_map(self):
        from nav_msgs.msg import OccupancyGrid
        grid=OccupancyGrid()
        grid.header.frame_id='map';grid.header.stamp=module.rospy.Time(100)
        grid.info.width=20;grid.info.height=20;grid.info.resolution=.05
        grid.info.origin.orientation.w=1.;grid.data=[0]*400
        self.node.map=grid;self.node.coverage=None;self.node.lethal_threshold=99
        self.node.validated_status=Mock();self.node.frontier_status=Mock();self.node.coverage_summary=Mock()
        self.node.reachable_cells=Mock(return_value={210})
        with patch.object(adapter_module.rospy.Time,'now',return_value=module.rospy.Time(100)):
            self.node.publish_map()
        self.assertEqual(self.node.validated_status.publish.call_args[0][0].data,'INVALID')


if __name__ == '__main__':
    unittest.main()
