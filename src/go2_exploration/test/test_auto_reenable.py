import unittest
from unittest.mock import Mock,patch
from types import SimpleNamespace
import test_supervisor_start as fixture


class AutomaticReenable(unittest.TestCase):
    setUp=fixture.SupervisorStart.setUp
    fresh=fixture.SupervisorStart.fresh

    def fault(self):
        self.node.ever_armed=self.node.bridge_ever_enabled=True
        self.node.latched=True
        self.node.safety_reason='terrain_health_unavailable'
        self.node.armed=self.node.bridge_enabled=False
        self.node.paused=True
        self.node.gate_recovery_ready=True
        self.node.publish_state=Mock()

    def progress(self,start=100,n=51):
        for i in range(n):
            now=start+i*.2
            self.fresh(now)
            self.node.map_generation=i
            with patch.object(fixture.module.time,'monotonic',return_value=now):
                self.node.recover_if_ready(now)

    def test_qualified_recovery_retains_map_and_requests_fresh_goal(self):
        self.fault()
        def reset():
            self.node.latched=False
            return SimpleNamespace(success=True)
        def arm(*args,**kwargs):
            self.node.armed=self.node.bridge_enabled=True
            return True
        self.node.auto_reset_service=Mock(side_effect=reset)
        self.node.request_arm=Mock(side_effect=arm)
        self.node.stop_child=Mock()
        self.node.save_service=Mock()
        original_map=self.node.map_name
        self.progress()
        self.node.auto_reset_service.assert_called_once_with()
        self.node.request_arm.assert_called_once_with(110.,recovery=True)
        self.assertFalse(self.node.paused)
        self.assertEqual(self.node.map_name,original_map)
        self.node.save_service.assert_not_called()
        self.assertEqual(self.node.child,None)

    def test_stop_and_takeover_after_terrain_fault_prevent_reset(self):
        for takeover in ('operator_stop','manual_control_takeover'):
            self.fault();self.node.recovery_block=takeover
            self.node.auto_reset_service=Mock()
            self.progress()
            self.node.auto_reset_service.assert_not_called()

    def test_odom_latch_not_overridden_after_terrain_recovers(self):
        self.fault();self.node.safety_healthy=False
        self.node.health_reason='odom_health_unavailable'
        self.progress()
        self.node.auto_reset_service.assert_not_called()

    def test_explicit_disable_option_prevents_reenable(self):
        self.fault();self.node.auto_reenable=False
        self.progress()
        self.node.auto_reset_service.assert_not_called()

    def test_sdk_refusal_and_unknown_faults_never_auto_retry(self):
        self.fault();self.node.startup_failed=True
        self.progress()
        self.node.auto_reset_service.assert_not_called()
        self.node.startup_failed=False
        self.node.safety_reason='invalid_command'
        self.progress(120)
        self.node.auto_reset_service.assert_not_called()

    def test_transient_unhealthy_diagnostic_resets_recovery_timer(self):
        self.fault();self.progress(n=40)
        from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
        def message(healthy):
            values=dict(latched_stop='True',armed='False',latched_reason='terrain_health_unavailable',
                        health_ok=str(healthy),auto_recovery_ready=str(healthy))
            return DiagnosticArray(status=[DiagnosticStatus(name='go2_exploration_safety',
                       values=[KeyValue(k,v) for k,v in values.items()])])
        self.node.safety(message(False));self.node.safety(message(True))
        self.progress(108,n=20)
        self.node.auto_reset_service.assert_not_called()

    def test_fault_during_existing_obstacle_pause_also_disables_sdk(self):
        self.node.paused=True
        self.node.disable_bridge=Mock()
        self.node.publish_state=Mock()
        self.node.hold_session(fault=True)
        self.node.disable_bridge.assert_called_once_with()

    def test_no_reenable_storm(self):
        self.fault();self.node.recovery_attempts=[90,91,92]
        self.progress()
        self.node.auto_reset_service.assert_not_called()

    def test_operator_stop_between_reset_and_enable_wins(self):
        self.fault()
        def reset():
            self.node.latched=False
            self.node.stop_requested=True
            return SimpleNamespace(success=True)
        self.node.auto_reset_service=Mock(side_effect=reset)
        self.node.request_arm=Mock()
        self.progress()
        self.node.request_arm.assert_not_called()

    def test_real_recovery_uses_original_sdk_diagnostic_level_with_recorded_2010(self):
        self.fault();self.node.use_real_sdk=True
        from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
        # Values seen in the bounded real walking trial. That recorder omitted
        # DiagnosticStatus.level; the original bridge's telemetry OK branch is 0.
        values=dict(manual_resume_pending='false',mode_override_api='0',remote_buttons='0',
                    remote_axis_max='0',no_step_response='false',low_state_age_sec='.00185895',
                    sport_state_age_sec='.00226736',remote_age_sec='-1.3113e-05',
                    sport_state_error_code='2010',last_gait_error='',active_motion_mode='mcf',
                    required_motion_mode='mcf',battery_soc_percent='75',min_enable_battery_percent='25')
        for level in (0,2):
            with patch.object(fixture.module.time,'monotonic',return_value=100.):
                self.node.sdk_diagnostics(DiagnosticArray(status=[DiagnosticStatus(
                    name='GO2 SDK bridge',level=level,values=[KeyValue(k,v) for k,v in values.items()])]))
            self.assertEqual(self.node.recovery_problem(100.),None if level==0 else 'sdk_or_gait_fault')

if __name__=='__main__':unittest.main()
