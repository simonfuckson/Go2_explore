import unittest
from types import SimpleNamespace
import test_gate_cycles as fixture
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from std_msgs.msg import Bool, String
from go2_exploration_safety.fault_recovery import RecoveryDwell, sdk_recovery_problem


class AutoResetGate(unittest.TestCase):
    setUp = fixture.GateCycles.setUp
    cloud = fixture.GateCycles.cloud
    twist = staticmethod(fixture.GateCycles.twist)

    def held(self, reason='terrain_health_unavailable'):
        self.gate.latch_locked(reason, self.command)
        self.gate.bridge_callback(Bool(data=False))
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.PREEMPTED)]))
        for i in range(50):
            self.refresh()

    def refresh(self):
        self.now += .05
        self.identity.header.stamp = fixture.rospy.Time.from_sec(self.now)
        self.gate.move_base_status_callback(GoalStatusArray())
        self.gate.command_callback(self.twist(0))
        self.gate.cloud_callback(self.empty)
        self.gate.raw_cloud_callback(self.floor)
        self.gate.costmap_callback(self.grid)
        self.gate.run_cycle()

    def test_healthy_stationary_input_failure_can_reset_but_never_arm(self):
        self.held()
        self.assertTrue(self.gate.auto_recovery_ready)
        self.assertTrue(self.gate.auto_reset_callback(None).success)
        self.assertFalse(self.gate.armed)
        self.assertFalse(self.gate.bridge_enabled)
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual(self.gate.last_output.linear.x, 0)
        self.assertIsNone(self.gate.last_command_receive)

    def test_operator_stop_after_original_fault_irrevocably_blocks_auto_reset(self):
        self.held()
        self.gate.stop_callback(None)
        self.assertEqual(self.gate.latched_reason, 'terrain_health_unavailable')
        self.assertTrue(self.gate.auto_recovery_inhibited)
        self.assertFalse(self.gate.auto_reset_callback(None).success)

    def test_remote_takeover_after_fault_remains_blocked_when_sticks_return(self):
        self.held()
        self.gate.control_state_callback(String(data='manual_override'))
        self.gate.control_state_callback(String(data='disabled'))
        self.refresh()
        self.assertFalse(self.gate.auto_reset_callback(None).success)

    def test_health_loss_between_ready_and_reset_is_rejected(self):
        self.held()
        self.gate.upstream.problem.return_value = 'odom_health_unavailable'
        self.assertFalse(self.gate.auto_reset_callback(None).success)

    def test_no_automatic_reset_of_command_or_control_faults(self):
        self.held('control_disabled_or_manual_takeover')
        self.assertFalse(self.gate.auto_recovery_ready)
        self.assertFalse(self.gate.auto_reset_callback(None).success)

    def test_stale_check_zero_or_goal_cannot_reset(self):
        self.held()
        self.now += .3
        self.assertFalse(self.gate.auto_reset_callback(None).success)

    def test_unknown_current_footprint_is_not_automatically_cleared(self):
        self.grid.data[60*120+60] = -1
        self.held()
        self.assertFalse(self.gate.auto_recovery_ready)
        self.assertFalse(self.gate.auto_reset_callback(None).success)

    def test_robot_motion_restarts_stationary_evidence(self):
        self.held()
        self.identity.transform.translation.x += .04
        self.refresh()
        self.assertFalse(self.gate.auto_recovery_ready)

    def test_old_nonzero_shaper_tail_revokes_recovery_immediately(self):
        self.held()
        self.gate.command_callback(self.twist(.1))
        self.assertFalse(self.gate.auto_reset_callback(None).success)


class RecoveryPolicy(unittest.TestCase):
    def test_full_dwell_and_new_maps_required(self):
        state = RecoveryDwell()
        for i in range(51):
            self.assertFalse(state.update(100+i*.2, 'terrain', True, 1))
        self.assertTrue(state.update(110.2, 'terrain', True, 3))

    def test_gap_health_or_new_fault_restarts_dwell(self):
        for scenario in ('gap', 'health', 'fault'):
            state=RecoveryDwell()
            for i in range(40):state.update(100+i*.2,'terrain',True,i)
            if scenario=='health':state.update(107.9,'terrain',False,40)
            self.assertFalse(state.update(111 if scenario=='gap' else 108,
                                         'odom' if scenario=='fault' else 'terrain',True,41))

    def test_original_sdk_health_policy_and_real_firmware_status(self):
        self.assertIsNotNone(sdk_recovery_problem({}))
        values=dict(manual_resume_pending='false',mode_override_api='0',remote_buttons='0',
                    remote_axis_max='0',no_step_response='false',low_state_age_sec='.1',
                    sport_state_age_sec='.1',remote_age_sec='.1',sport_state_error_code='2010',
                    _diagnostic_level='0',
                    last_gait_error='',active_motion_mode='mcf',required_motion_mode='mcf',
                    battery_soc_percent='40',min_enable_battery_percent='25')
        self.assertIsNone(sdk_recovery_problem(values))
        for key, value in [('_diagnostic_level','2'),('battery_soc_percent','nan'),
                           ('no_step_response','true'),('remote_buttons','1'),
                           ('low_state_age_sec','2'),('remote_age_sec','-.2'),('active_motion_mode','ai')]:
            with self.subTest(key=key):
                self.assertIsNotNone(sdk_recovery_problem(dict(values, **{key:value})))

if __name__=='__main__':unittest.main()
