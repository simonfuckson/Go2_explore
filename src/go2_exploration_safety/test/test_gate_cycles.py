"""Actual gate callbacks/cycles with no ROS transport, services or threads."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
import yaml
import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gate_cycles', str(ROOT/'scripts/cmd_vel_safety_gate.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GateCycles(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        config = yaml.safe_load((ROOT/'config/cmd_vel_safety.yaml').read_text())
        config['allow_stationary_command_wait'] = True
        self.identity = TransformStamped()
        self.identity.header.stamp = rospy.Time(100)
        self.identity.transform.rotation.w = 1
        patches = [
            patch.multiple(module.rospy, get_name=lambda:'/go2_exploration_safety',
                remap_name=lambda n:n, get_param=lambda n,d=None:
                    [[.35,.155],[.35,-.155],[-.35,-.155],[-.35,.155]] if n.endswith('/footprint') else
                    .03 if n.endswith('/footprint_padding') else config.get(n.lstrip('~'),d),
                Publisher=Mock(side_effect=lambda *a,**k:Mock()), Subscriber=Mock(), Service=Mock(), on_shutdown=Mock(),
                logwarn=Mock(), logerr=Mock(), loginfo=Mock()),
            patch.object(module.threading.Thread, 'start'),
            patch.object(module.tf2_ros, 'Buffer', return_value=Mock(lookup_transform=Mock(return_value=self.identity))),
            patch.object(module.tf2_ros, 'TransformListener'),
            patch.object(module.time, 'monotonic', side_effect=lambda:self.now),
            patch.object(module.rospy.Time, 'now', side_effect=lambda:rospy.Time.from_sec(self.now)),
        ]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.gate = module.CmdVelSafetyGate()
        self.gate.armed = True  # isolated object, never a ROS service
        self.gate.bridge_enabled = True
        # These inherited tests isolate collision/generation handling. Additional
        # GO2 tests below exercise the real upstream heartbeat checks.
        self.gate.upstream.problem = Mock(return_value=None)
        self.shaped_only = self.gate.command_callback
        def both_commands(message):
            raw=module.copy_twist(message)
            raw._connection_header={'callerid':'/move_base'}
            self.gate.raw_command_callback(raw)
            self.shaped_only(message)
        self.gate.command_callback = both_commands
        self.goal = MoveBaseActionGoal()
        self.goal.goal_id.id = 'isolated_test_goal'
        self.goal.goal_id.stamp = rospy.Time(1)
        self.goal.goal.target_pose.header.frame_id = 'map'
        self.goal.goal.target_pose.pose.position.x = 4.025
        self.goal.goal.target_pose.pose.position.y = -3.175
        self.gate.move_base_goal_callback(self.goal)
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.ACTIVE)]))
        self.command = self.twist(.2)
        self.gate.command_callback(self.command)
        self.gate.local_plan_callback(NavPath(poses=[PoseStamped(),PoseStamped()]))
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'base_link'
        self.grid.info.width = self.grid.info.height = 120
        self.grid.info.resolution = .05
        self.grid.info.origin.position.x = self.grid.info.origin.position.y = -3
        self.grid.info.origin.orientation.w = 1
        self.grid.data = [0]*14400
        self.gate.costmap_callback(self.grid)
        self.empty = self.cloud([])
        self.floor = self.cloud([(.5+i*.01,.5,-.15) for i in range(20)])
        self.gate.cloud_callback(self.empty)
        self.gate.raw_cloud_callback(self.floor)
        self.gate.output_publisher.reset_mock()

    @staticmethod
    def twist(v, w=0):
        result=Twist()
        result.linear.x, result.angular.z = v,w
        result._connection_header={'callerid':'/go2_velocity_shaper'}
        return result

    def cloud(self, points):
        return point_cloud2.create_cloud_xyz32(Header(frame_id='base_link',stamp=rospy.Time(100)),points)

    def inject(self, callback, repeat=False):
        original = self.gate.costmap_stop_check
        used = [False]
        def check(*args):
            result = original(*args)
            if not used[0] or repeat:
                used[0] = True
                callback()
            return result
        self.gate.costmap_stop_check = check

    def test_normal_refreshes_do_not_insert_any_zero(self):
        def refresh():
            self.gate.command_callback(self.command)
            self.gate.cloud_callback(self.empty)
            self.gate.raw_cloud_callback(self.floor)
            self.gate.costmap_callback(self.grid)
        self.inject(refresh, repeat=True)
        for _ in range(500):
            self.now += .05
            self.identity.header.stamp = rospy.Time.from_sec(self.now)
            self.gate.local_plan_callback(NavPath(poses=[PoseStamped(),PoseStamped()]))
            self.gate.move_base_status_callback(GoalStatusArray(status_list=[
                GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.ACTIVE)]))
            refresh()
            self.gate.run_cycle()
            self.assertEqual(self.gate.last_output.linear.x, .2)
            self.assertFalse(self.gate.latched_stop)
        outputs=[c.args[0] for c in self.gate.output_publisher.publish.call_args_list]
        self.assertEqual(len(outputs),500)
        self.assertTrue(all(o.linear.x==.2 for o in outputs))

    def test_changed_command_is_checked_again_in_same_cycle(self):
        self.inject(lambda:self.gate.command_callback(self.twist(.15,.1)))
        self.gate.run_cycle()
        self.assertEqual(self.gate.last_recheck_count,1)
        self.assertEqual(self.gate.last_output.linear.x,.15)
        self.assertEqual(self.gate.last_output.angular.z,.1)
        self.assertEqual(self.gate.output_publisher.publish.call_count,1)

    def test_new_hazard_during_check_stops_in_same_cycle(self):
        self.inject(lambda:self.gate.cloud_callback(self.cloud([(.35,0,.1)])))
        self.gate.run_cycle()
        self.assertTrue(self.gate.latched_stop)
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.assertEqual(self.gate.latched_reason,'obstacle_in_cloud_stop_region')

    def test_new_costmap_hazard_is_rechecked(self):
        def obstacle():
            self.grid.data[60*120+67]=100
            self.gate.costmap_callback(self.grid)
        self.inject(obstacle)
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'obstacle_in_costmap_stop_region')

    def test_stop_and_zero_commands_cannot_be_overwritten(self):
        for callback in [lambda:self.gate.command_callback(self.twist(0)),
                         lambda:self.gate.stop_callback(None)]:
            self.inject(callback)
            self.gate.run_cycle()
            self.assertEqual(self.gate.last_output.linear.x,0)

    def test_first_latch_reason_survives_refresh_and_operator_stop(self):
        self.gate.cloud_callback(self.cloud([(.35,0,.1)]))
        self.gate.run_cycle()
        reason=self.gate.latched_reason
        self.inject(lambda:self.gate.costmap_callback(self.grid),repeat=True)
        self.gate.run_cycle()
        self.gate.stop_callback(None)
        self.assertEqual(self.gate.last_reason,reason)
        self.assertEqual(self.gate.latched_reason,reason)

    def test_stale_command_is_still_latched(self):
        self.now += .21
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'command_stream_stale')

    def test_reset_cannot_use_clearance_from_an_outdated_snapshot(self):
        self.gate.stop_callback(None)
        self.inject(lambda:self.gate.cloud_callback(self.cloud([(.35,0,.1)])))
        self.gate.run_cycle()
        self.assertFalse(self.gate.reset_callback(None).success)
        self.assertTrue(self.gate.latched_stop)

    def test_reverse_arc_is_not_executed_as_rotation(self):
        self.gate.command_callback(self.twist(-.01,.2))
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'reverse_command_inhibited')
        self.assertEqual(self.gate.last_output.angular.z,0)

    def test_unbounded_changing_data_has_bounded_retries_and_stops(self):
        n=[0]
        def update():
            n[0]+=1
            self.gate.cloud_callback(self.cloud([(.8,.8,n[0]*.01)]))
        self.inject(update,repeat=True)
        self.gate.run_cycle()
        self.assertEqual(n[0],3)
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.assertEqual(self.gate.last_reason,'collision_check_budget_exhausted')

    def test_pause_cancels_without_resetting_or_rearming(self):
        self.gate.pause_callback(None)
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.PREEMPTED)]))
        self.gate.run_cycle()
        self.assertTrue(self.gate.armed)
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.assertTrue(self.gate.resume_callback(None).success)
        self.gate.run_cycle()
        self.assertEqual(self.gate.last_output.linear.x,0)


class TerminalStopDrain(GateCycles):
    def terminal(self):
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.SUCCEEDED)]))

    def refresh_inputs(self):
        self.identity.header.stamp=rospy.Time.from_sec(self.now)
        self.gate.cloud_callback(self.empty)
        self.gate.raw_cloud_callback(self.floor)
        self.gate.costmap_callback(self.grid)
        self.terminal()

    def test_terminal_stops_immediately_and_discards_shaper_tail(self):
        self.terminal()
        self.gate.run_cycle()
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual(self.gate.last_output.linear.x,0)
        for velocity in (.18,.12,.06,0):
            self.now+=.1
            self.refresh_inputs()
            self.shaped_only(self.twist(velocity))
            self.gate.run_cycle()
            self.assertFalse(self.gate.latched_stop)
            self.assertEqual(self.gate.last_output.linear.x,0)

    def test_repeated_terminal_status_cannot_hide_persistent_motion(self):
        self.terminal()
        self.now+=1.51
        self.refresh_inputs()
        self.shaped_only(self.twist(.2))
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'motion_without_live_goal')

    def test_invalid_tail_and_stale_cloud_still_fault(self):
        self.terminal()
        self.shaped_only(self.twist(-.1))
        self.assertEqual(self.gate.latched_reason,'reverse_command_inhibited')

    def test_cloud_timeout_during_drain_is_not_masked(self):
        self.terminal()
        self.now+=.8
        self.shaped_only(self.twist(.05))
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'obstacle_cloud_stale')

    def test_new_goal_requires_new_handshake(self):
        self.terminal()
        self.goal.goal_id.id='new_after_success'
        self.gate.move_base_goal_callback(self.goal)
        self.shaped_only(self.twist(.2))
        self.gate.run_cycle()
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.ACTIVE)]))
        self.gate.command_callback(self.twist(.2))
        self.gate.local_plan_callback(NavPath(poses=[PoseStamped(),PoseStamped()]))
        self.gate.run_cycle()
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual(self.gate.last_output.linear.x,.2)


class ObstacleRecovery(GateCycles):
    """Real gate cycle/callbacks; outputs and ROS transport are mocked."""
    def hold(self):
        self.gate.recover_obstacle_stop = True
        self.gate.cloud_callback(self.cloud([(.35,0,.1)]))
        self.gate.run_cycle()
        self.assertTrue(self.gate.armed)
        self.assertTrue(self.gate.obstacle_hold)
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual(self.gate.last_output.linear.x, 0)
        self.assertGreater(self.gate.cancel_publisher.publish.call_count, 0)
        self.assertEqual(self.gate.obstacle_goal_target, ('map', 4.025, -3.175))

    def tick(self, blocked=False, active=False, step=.05):
        self.now += step
        self.identity.header.stamp = rospy.Time.from_sec(self.now)
        self.gate.cloud_callback(self.cloud([(.35,0,.1)]) if blocked else self.empty)
        self.gate.raw_cloud_callback(self.floor)
        self.gate.costmap_callback(self.grid)
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id, status=GoalStatus.ACTIVE if active else GoalStatus.PREEMPTED)]))
        self.gate.run_cycle()
        self.assertEqual(self.gate.last_output.linear.x, 0)
        self.assertEqual(self.gate.last_output.angular.z, 0)

    def test_clear_requires_cancel_continuous_dwell_and_new_goal(self):
        self.hold()
        for _ in range(45): self.tick(active=True)
        self.assertTrue(self.gate.obstacle_hold)
        for _ in range(20): self.tick()
        self.assertFalse(self.gate.resume_callback(None).success)
        self.tick(blocked=True)
        for _ in range(30): self.tick()
        self.assertTrue(self.gate.obstacle_hold)
        for _ in range(15): self.tick()
        self.assertFalse(self.gate.obstacle_hold)
        self.assertTrue(self.gate.paused)
        self.assertEqual(self.gate.obstacle_event, 1)
        self.assertTrue(self.gate.resume_callback(None).success)
        # A command left over from the cancelled goal must not start the chassis.
        self.gate.run_cycle()
        self.assertEqual(self.gate.last_output.linear.x, 0)
        self.goal.goal_id.id = 'fresh_recovery_goal'
        self.goal.goal_id.stamp = rospy.Time.from_sec(self.now)
        self.gate.move_base_goal_callback(self.goal)
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.ACTIVE)]))
        self.gate.command_callback(self.command)
        self.gate.local_plan_callback(NavPath(poses=[PoseStamped(),PoseStamped()]))
        self.gate.run_cycle()
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual(self.gate.last_output.linear.x,.2)

    def test_old_command_cannot_move_after_resume_without_new_goal(self):
        self.hold()
        for _ in range(45): self.tick()
        self.assertTrue(self.gate.resume_callback(None).success)
        self.gate.command_callback(self.command)
        self.gate.run_cycle()
        self.assertEqual(self.gate.last_output.linear.x, 0)
        self.assertEqual(self.gate.latched_reason, 'motion_without_live_goal')

    def test_stale_sensor_during_hold_is_a_fault_not_auto_resume(self):
        self.hold()
        for _ in range(10): self.tick()
        self.now += .8
        self.gate.run_cycle()
        self.assertTrue(self.gate.latched_stop)
        self.assertEqual(self.gate.latched_reason, 'obstacle_cloud_stale')
        for _ in range(45): self.tick()
        self.assertFalse(self.gate.resume_callback(None).success)

    def test_operator_stop_always_supersedes_recoverable_hold(self):
        self.hold()
        self.gate.stop_callback(None)
        for _ in range(45): self.tick()
        self.assertTrue(self.gate.latched_stop)
        self.assertEqual(self.gate.latched_reason, 'operator_stop')
        self.assertFalse(self.gate.resume_callback(None).success)

    def test_returning_costmap_unknown_blocks_resume(self):
        self.hold()
        for _ in range(45): self.tick()
        self.assertFalse(self.gate.obstacle_hold)
        self.grid.data[60*120+67] = -1
        self.tick()
        self.assertTrue(self.gate.obstacle_hold)
        self.assertFalse(self.gate.resume_callback(None).success)
        self.assertEqual(self.gate.obstacle_event, 1)

    def test_gap_in_checks_restarts_clear_dwell(self):
        self.hold()
        for _ in range(30): self.tick()
        self.tick(step=.5)
        for _ in range(20): self.tick()
        self.assertTrue(self.gate.obstacle_hold)

    def test_zero_shaper_heartbeats_allow_clear_dwell_but_never_motion(self):
        # The shaper keeps publishing stop commands after cancellation. Force a
        # heartbeat to arrive during every collision check, as in the live race.
        self.hold()
        self.shaped_only(self.twist(0))
        self.inject(lambda:self.shaped_only(self.twist(0)), repeat=True)
        for _ in range(45): self.tick()
        self.assertFalse(self.gate.obstacle_hold)
        self.assertTrue(self.gate.paused)
        self.assertTrue(self.gate.resume_callback(None).success)
        self.gate.run_cycle()
        self.assertEqual(self.gate.last_output.linear.x, 0)
        self.assertEqual(self.gate.last_output.angular.z, 0)

    def test_invalid_command_during_hold_remains_latched(self):
        self.hold()
        self.gate.command_callback(self.twist(float('nan')))
        for _ in range(45): self.tick()
        self.assertEqual(self.gate.latched_reason, 'invalid_command')

    def test_simultaneous_command_timeout_is_not_downgraded_to_obstacle_wait(self):
        self.gate.recover_obstacle_stop = True
        self.now += .21
        self.gate.cloud_callback(self.cloud([(.35,0,.1)]))
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason, 'command_stream_stale')
        self.assertFalse(self.gate.obstacle_hold)

    def test_missing_status_during_hold_is_a_fault(self):
        self.hold()
        for _ in range(45): self.tick()
        self.gate.last_move_base_status_receive = self.now - 1.01
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason, 'move_base_status_stale')
        self.assertFalse(self.gate.resume_callback(None).success)

    def test_cancelling_an_old_goal_is_not_a_new_handshake_timeout(self):
        self.gate.live_goal_since = self.now - 30.0
        self.hold()
        self.tick(active=True)
        self.gate.command_callback(self.twist(0))
        self.assertFalse(self.gate.latched_stop)
        for _ in range(45): self.tick()
        self.assertTrue(self.gate.resume_callback(None).success)

    def recorded_unknown_hold(self):
        import json
        f=json.loads((ROOT/'test/fixtures/persistent_side_unknown.json').read_text())
        self.grid.header.frame_id=f['frame']
        self.grid.info.width,self.grid.info.height=f['width'],f['height']
        self.grid.info.resolution=f['resolution']
        self.grid.info.origin.position.x,self.grid.info.origin.position.y=f['origin']
        self.grid.data=list(f['data'])
        t=self.identity.transform
        t.translation.x,t.translation.y=f['pose']['x'],f['pose']['y']
        t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w=f['pose']['q']
        self.gate.costmap_callback(self.grid)
        self.gate.recover_obstacle_stop=True
        self.gate.command_callback(self.twist(*f['trip']))
        self.gate.run_cycle()
        self.assertTrue(self.gate.obstacle_hold)
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual(self.gate.last_reason,'obstacle_in_costmap_stop_region')
        self.assertTrue(self.gate.costmap_stop_check(self.grid,self.identity,self.twist(*f['trip']))[0])
        self.assertFalse(self.gate.costmap_stop_check(self.grid,self.identity,self.twist(.3,0))[0])

    def stopped_ticks(self, count=90):
        for _ in range(count):
            self.shaped_only(self.twist(0))
            self.tick()

    def recovery_goal(self, w=0):
        self.goal.goal_id.id='new_after_stationary_check'
        self.goal.goal_id.stamp=rospy.Time.from_sec(self.now)
        self.gate.move_base_goal_callback(self.goal)
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.ACTIVE)]))
        self.gate.command_callback(self.twist(.3,w))
        self.gate.local_plan_callback(NavPath(poses=[PoseStamped(),PoseStamped()]))
        self.gate.run_cycle()

    def test_recorded_unknown_turn_replans_only_after_stop_and_new_goal(self):
        self.recorded_unknown_hold()
        original=tuple(self.grid.data)
        self.stopped_ticks(60)
        self.assertTrue(self.gate.obstacle_hold)
        self.assertFalse(self.gate.resume_callback(None).success)
        self.stopped_ticks(30)
        self.assertFalse(self.gate.obstacle_hold)
        self.assertTrue(self.gate.obstacle_replan_ready)
        self.assertEqual(self.gate.last_state,'OBSTACLE_REPLAN_READY')
        self.assertTrue(self.gate.paused)
        self.assertEqual(tuple(self.grid.data),original)
        self.assertTrue(self.gate.resume_callback(None).success)
        self.gate.run_cycle()
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.recovery_goal()
        self.assertEqual(self.gate.last_output.linear.x,.3)
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual(self.gate.obstacle_event,1)

    def test_new_goal_cannot_reuse_blocked_turn_after_stationary_recovery(self):
        self.recorded_unknown_hold();self.stopped_ticks()
        self.assertTrue(self.gate.resume_callback(None).success)
        self.recovery_goal(-.3)
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.assertEqual(self.gate.last_output.angular.z,0)
        self.assertTrue(self.gate.obstacle_hold)
        self.assertEqual(self.gate.obstacle_event,2)

    def test_no_fresh_zero_shaper_stream_cannot_confirm_stop(self):
        self.recorded_unknown_hold()
        for _ in range(100):self.tick()
        self.assertFalse(self.gate.stopped_window.ready)
        self.assertTrue(self.gate.obstacle_hold)

    def test_moving_pose_cannot_enable_alternative_replan(self):
        import math
        self.recorded_unknown_hold()
        base=self.identity.transform.translation.x
        for i in range(100):
            self.identity.transform.translation.x=base+.035*math.sin(i*.2)
            self.shaped_only(self.twist(0));self.tick()
        self.assertFalse(self.gate.obstacle_replan_ready)
        self.assertTrue(self.gate.obstacle_hold)

    def test_shaper_tail_does_not_count_as_verified_zero(self):
        self.recorded_unknown_hold()
        for _ in range(100):
            self.shaped_only(self.twist(.1));self.tick()
        self.assertFalse(self.gate.stopped_window.ready)
        self.assertFalse(self.gate.obstacle_replan_ready)

    def test_persistent_forward_obstacle_still_blocks_replan(self):
        self.recorded_unknown_hold()
        for _ in range(100):
            self.shaped_only(self.twist(0));self.tick(blocked=True)
        self.assertTrue(self.gate.obstacle_hold)
        self.assertFalse(self.gate.obstacle_replan_ready)

    def test_obstacle_arriving_during_replan_check_revokes_clearance(self):
        self.recorded_unknown_hold();self.stopped_ticks()
        self.assertTrue(self.gate.obstacle_replan_ready)
        self.inject(lambda:self.gate.cloud_callback(self.cloud([(.35,0,.1)])))
        self.shaped_only(self.twist(0));self.tick()
        self.assertTrue(self.gate.obstacle_hold)
        self.assertFalse(self.gate.obstacle_replan_ready)
        self.assertFalse(self.gate.resume_callback(None).success)

    def test_nonzero_shaper_between_ready_and_resume_cannot_resume(self):
        self.recorded_unknown_hold();self.stopped_ticks()
        self.shaped_only(self.twist(.1))
        self.assertFalse(self.gate.resume_callback(None).success)
        self.assertEqual(self.gate.last_output.linear.x,0)

    def test_stop_and_takeover_after_replan_ready_remain_latched(self):
        self.recorded_unknown_hold();self.stopped_ticks()
        from std_msgs.msg import Bool
        self.gate.bridge_callback(Bool(data=False))
        self.gate.stop_callback(None)
        self.assertEqual(self.gate.latched_reason,'control_disabled_or_manual_takeover')
        self.assertFalse(self.gate.obstacle_replan_ready)
        self.assertFalse(self.gate.resume_callback(None).success)

    def test_stale_sensor_after_replan_ready_does_not_auto_recover(self):
        self.recorded_unknown_hold();self.stopped_ticks()
        self.now+=.8;self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'obstacle_cloud_stale')
        self.stopped_ticks()
        self.assertFalse(self.gate.resume_callback(None).success)


if __name__=='__main__':
    unittest.main()
