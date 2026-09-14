"""Regress the real trial's planner-stop/shaper-tail and recovery races."""
import unittest
from unittest.mock import Mock
import test_gate_cycles as fixture
import test_auto_recovery as recovery_fixture
from actionlib_msgs.msg import GoalStatus,GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath


class PlannerStop(unittest.TestCase):
    setUp=fixture.GateCycles.setUp
    cloud=fixture.GateCycles.cloud
    twist=staticmethod(fixture.GateCycles.twist)
    inject=fixture.GateCycles.inject

    def raw(self,v=0,w=0):
        message=self.twist(v,w);message._connection_header={'callerid':'/move_base'}
        self.gate.raw_command_callback(message)

    def refresh(self):
        self.identity.header.stamp=fixture.rospy.Time.from_sec(self.now)
        self.gate.cloud_callback(self.empty);self.gate.raw_cloud_callback(self.floor)
        self.gate.costmap_callback(self.grid)
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.ACTIVE)]))

    def test_explicit_planner_stop_cuts_sdk_output_before_shaper_finishes(self):
        self.gate.run_cycle();self.assertEqual(self.gate.last_output.linear.x,.2)
        self.now+=.36;self.refresh();self.raw(0)
        self.assertEqual(self.gate.output_publisher.publish.call_args.args[0].linear.x,0)
        self.shaped_only(self.twist(.18));self.gate.run_cycle()
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.assertEqual(self.gate.last_reason,'planner_requested_stop')

    def test_stop_arriving_during_collision_check_cannot_publish_old_motion(self):
        self.inject(lambda:self.raw(0))
        self.gate.run_cycle()
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual(self.gate.last_output.linear.x,0)

    def test_repeated_stop_heartbeats_do_not_extend_nonzero_tail_grace(self):
        self.raw(0)
        for _ in range(16):
            self.now+=.1;self.refresh();self.raw(0)
            self.shaped_only(self.twist(.1));self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'shaper_stop_timeout')
        self.assertTrue(self.gate.auto_recovery_inhibited)

    def test_raw_stop_does_not_hide_lost_planner_or_sensor_stream(self):
        self.raw(0);self.now+=.21;self.refresh()
        self.shaped_only(self.twist(.18));self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'command_stream_stale')

    def test_stale_cloud_still_latches_during_stop_tail(self):
        self.raw(0);self.now+=.7;self.raw(0)
        self.shaped_only(self.twist(.05));self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'obstacle_cloud_stale')

    def test_explicit_stop_cannot_hide_an_invalid_raw_command(self):
        bad=self.twist(0);bad.linear.y=.1;bad._connection_header={'callerid':'/move_base'}
        self.gate.raw_command_callback(bad)
        self.assertEqual(self.gate.latched_reason,'unsupported_command_axis')

    def test_new_motion_still_requires_a_fresh_plan(self):
        self.raw(0);self.now+=.36;self.refresh()
        self.raw(.2);self.shaped_only(self.twist(.2));self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'local_plan_stale')

    def test_fresh_plan_and_raw_motion_resume_normally(self):
        self.raw(0);self.shaped_only(self.twist(.1));self.gate.run_cycle()
        self.now+=.1;self.refresh();self.raw(.2)
        self.gate.local_plan_callback(NavPath(poses=[PoseStamped(),PoseStamped()]))
        self.shaped_only(self.twist(.2));self.gate.run_cycle()
        self.assertFalse(self.gate.latched_stop);self.assertEqual(self.gate.last_output.linear.x,.2)

    def test_stop_evidence_survives_cancelled_goal_and_new_callbacks(self):
        self.now+=.36;self.refresh();self.raw(.2)
        self.shaped_only(self.twist(.2));self.gate.run_cycle()
        evidence=self.gate.stop_context
        self.assertEqual(evidence['reason'],'local_plan_stale')
        self.assertAlmostEqual(evidence['local_plan_age_s'],.36)
        self.assertEqual(evidence['goal_id'][0],'isolated_test_goal')
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.PREEMPTED)]))
        self.raw(0);self.shaped_only(self.twist(0))
        self.assertEqual(self.gate.stop_context,evidence)


class RecoverySnapshot(unittest.TestCase):
    setUp=fixture.GateCycles.setUp
    cloud=fixture.GateCycles.cloud
    twist=staticmethod(fixture.GateCycles.twist)
    held=recovery_fixture.AutoResetGate.held
    refresh=recovery_fixture.AutoResetGate.refresh
    inject=fixture.GateCycles.inject

    def test_one_new_clear_cloud_rechecks_without_resetting_recovery_dwell(self):
        self.held();self.assertTrue(self.gate.auto_recovery_ready)
        self.inject(lambda:self.gate.cloud_callback(self.cloud([(.8,.8,.1)])))
        self.refresh()
        self.assertEqual(self.gate.last_recheck_count,1)
        self.assertTrue(self.gate.last_health_ok)
        self.assertTrue(self.gate.auto_recovery_ready)
        self.assertEqual(self.gate.last_output.linear.x,0)

    def test_continuous_changes_still_exhaust_budget_and_block_reset(self):
        self.held();counter=[0]
        def changed():
            counter[0]+=1
            self.gate.cloud_callback(self.cloud([(.8,.8,counter[0]*.01)]))
        self.inject(changed,repeat=True);self.refresh()
        self.assertEqual(counter[0],3)
        self.assertFalse(self.gate.last_health_ok)
        self.assertFalse(self.gate.auto_recovery_ready)
        self.assertEqual(self.gate.last_health_reason,'collision_check_budget_exhausted')
        self.assertFalse(self.gate.auto_reset_callback(None).success)

if __name__=='__main__':unittest.main()
