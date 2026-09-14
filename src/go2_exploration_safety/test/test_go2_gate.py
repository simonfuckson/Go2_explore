"""GO2-specific faults through the real gate callbacks and collision cycle."""
import sys
from pathlib import Path
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).parent))
from test_gate_cycles import GateCycles,module
from std_msgs.msg import Bool,String


class Go2Gate(unittest.TestCase):
    twist=staticmethod(GateCycles.twist)
    cloud=GateCycles.cloud

    def setUp(self):
        GateCycles.setUp(self)
        del self.gate.upstream.problem
        self.fresh_health()

    def fresh_health(self):
        self.gate.health_callback('odom',Bool(data=True))
        self.gate.health_callback('terrain',Bool(data=True))
        self.gate.map_health_callback(String(data='ready: generation='+str(self.now)))

    def test_normal_go2_command_and_sdk_floor(self):
        self.gate.command_callback(self.twist(.3,.02))
        self.gate.run_cycle()
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual((self.gate.last_output.linear.x,self.gate.last_output.angular.z),(.3,.04))

    def test_collision_uses_go2_body_and_actual_point_three_speed(self):
        self.gate.command_callback(self.twist(.3))
        self.gate.cloud_callback(self.cloud([(.63,0,.1)]))
        self.gate.run_cycle()
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.assertTrue(self.gate.latched_stop)

    def test_shaper_heartbeat_cannot_hide_raw_planner_timeout(self):
        self.now+=.21
        self.fresh_health()
        self.shaped_only(self.twist(.2))
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'command_stream_stale')
        self.assertEqual(self.gate.last_output.linear.x,0)

    def test_raw_reverse_is_rejected_even_if_shaper_returns_a_turn(self):
        raw=module.copy_twist(self.twist(-.01,.2));raw._connection_header={'callerid':'/move_base'}
        self.gate.raw_command_callback(raw)
        self.shaped_only(self.twist(0,.5))
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'reverse_command_inhibited')
        self.assertEqual(self.gate.last_output.angular.z,0)

    def test_odom_failure_cannot_rearm_when_healthy_again(self):
        self.gate.health_callback('odom',Bool(data=False))
        self.fresh_health()
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'odom_health_unavailable')
        self.assertFalse(self.gate.arm_callback(None).success)

    def test_stale_map_is_a_fault_despite_fresh_sensors(self):
        self.gate.upstream.map_time=self.now-2.01
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'observation_map_stale')

    def test_manual_takeover_stops_without_automatic_enable(self):
        self.gate.bridge_callback(Bool(data=False))
        self.gate.bridge_callback(Bool(data=True))
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'control_disabled_or_manual_takeover')
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.assertFalse(self.gate.arm_callback(None).success)

    def test_unsupported_axis_and_excess_speed_latch(self):
        message=self.twist(.1);message.linear.y=.01
        self.shaped_only(message)
        self.assertEqual(self.gate.latched_reason,'unsupported_command_axis')


if __name__=='__main__': unittest.main()
