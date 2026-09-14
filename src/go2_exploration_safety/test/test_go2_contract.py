import math
from types import SimpleNamespace as S
import unittest
from go2_exploration_safety.go2_contract import body_envelope,command_problem,sdk_effective_velocity,UpstreamHealth
from go2_exploration_safety.odometry_health import OdometryHealth,quaternion_distance


def twist(v=0,w=0):
    return S(linear=S(x=v,y=0,z=0),angular=S(x=0,y=0,z=w))


class Go2Contract(unittest.TestCase):
    def test_real_envelope_and_invalid_geometry(self):
        self.assertEqual(body_envelope([[.35,.155],[.35,-.155],[-.35,-.155],[-.35,.155]]),(.35,.35,.155))
        self.assertEqual(body_envelope('[[0.35, 0.155], [0.35, -0.155], [-0.35, -0.155], [-0.35, 0.155]]'),(.35,.35,.155))
        for footprint in ([],[[0,0]]*3,[[math.nan,0]]*3):
            with self.assertRaises(ValueError): body_envelope(footprint)

    def test_forbidden_commands_are_rejected_before_shaping_can_hide_them(self):
        for message,reason in ((twist(-.01),'reverse_command_inhibited'),
                               (twist(.31),'command_limit_exceeded'),
                               (twist(0,.51),'command_limit_exceeded'),
                               (twist(math.nan),'invalid_command')):
            self.assertEqual(command_problem(message,'/move_base','/move_base'),reason)
        message=twist(.1);message.linear.y=.01
        self.assertEqual(command_problem(message,'/move_base','/move_base'),'unsupported_command_axis')
        self.assertEqual(command_problem(twist(.1),'/teleop','/move_base'),'unexpected_command_publisher')
        self.assertIsNone(command_problem(twist(.3,.5),'/move_base','/move_base'))

    def test_sdk_floor_is_checked_as_the_effective_command(self):
        self.assertEqual(sdk_effective_velocity(.02,.009),(0.,0.))
        self.assertEqual(sdk_effective_velocity(.1,.02),(.1,.04))
        self.assertEqual(sdk_effective_velocity(.3,-.02),(.3,-.04))

    def test_republished_map_status_does_not_refresh_a_stale_map(self):
        state=UpstreamHealth()
        state.heartbeat('odom',True,100);state.heartbeat('terrain',True,100)
        state.map_status('ready: generation=1',100)
        self.assertIsNone(state.problem(100))
        state.heartbeat('odom',True,103);state.heartbeat('terrain',True,103)
        state.map_status('ready: generation=1',103)
        self.assertEqual(state.problem(103),'observation_map_stale')
        state.map_status('ready: generation=2',103)
        self.assertIsNone(state.problem(103))

    def test_each_upstream_stream_has_independent_freshness(self):
        state=UpstreamHealth()
        for stream in ('odom','terrain'): state.heartbeat(stream,True,100)
        state.map_status('ready: 1',100)
        self.assertEqual(state.problem(100.51),'odom_health_unavailable')
        state.heartbeat('odom',True,100.8)
        self.assertEqual(state.problem(100.8),'terrain_health_unavailable')
        state.raw_time=100;state.raw_generation=2
        self.assertTrue(math.isinf(state.raw_age(100,3)))


class OdomTests(unittest.TestCase):
    def test_recorded_stair_turn_is_small_rotation_despite_large_euler_yaw(self):
        state=OdometryHealth()
        before=(-.6623203588797157,-.09343086906080779,.7430395859046043,-.022239350231086075)
        after=(-.6633807033432366,-.10360713072486892,.7325908055924942,-.11181375790184833)
        self.assertAlmostEqual(quaternion_distance(before,after),.1815829615091944)
        state.observe('lio',100.,(0,0,0),before,(0,0,0),100.)
        state.observe('lio',100.100045,(.005,-.028,-.002),after,(0,0,0),100.1)
        self.assertIsNone(state.fault)

    def test_quaternion_sign_and_normalization_do_not_create_rotation(self):
        q=(.5,.5,.5,.5)
        self.assertAlmostEqual(quaternion_distance(q,tuple(-x*1.005 for x in q)),0)

    def test_true_roll_jump_still_latches_even_without_yaw_change(self):
        state=self.seed()
        state.observe('lio',100.1,(0,0,0),(math.sin(.4),0,0,math.cos(.4)),(0,0,0),100.1)
        self.assertEqual(state.fault,'odometry_discontinuity')
        self.assertAlmostEqual(state.first_fault_context['rotation_delta_rad'],.8)
        self.assertAlmostEqual(state.first_fault_context['yaw_delta_rad'],0)

    def test_ten_centimetre_step_does_not_require_relaxing_jump_protection(self):
        state=self.seed()
        state.observe('lio',100.1,(.03,0,.10),(0,0,0,1),(.3,0,1),100.1)
        self.assertIsNone(state.fault)
        state.observe('lio',100.2,(1.,0,.1),(0,0,0,1),(.3,0,0),100.2)
        context=state.first_fault_context.copy()
        self.assertAlmostEqual(context['translation_delta_m'],.97)
        self.assertAlmostEqual(context['translation_limit_m'],.37)
        state.observe('lio',99,(0,0,0),(0,0,0,1),(0,0,0),100.3)
        self.assertEqual(state.fault,'odometry_discontinuity')
        self.assertEqual(state.first_fault_context,context)

    def seed(self):
        state=OdometryHealth()
        for stream in ('lio','nav'): state.observe(stream,100,(0,0,0),(0,0,0,1),(0,0,0),100)
        return state

    def test_initialization_clock_freeze_and_sensor_freeze(self):
        state=self.seed()
        self.assertIsNone(state.problem(100,100,True,100))
        self.assertEqual(state.problem(100,100,False,100),'waiting_for_initialization')
        for stream in ('lio','nav'): state.observe(stream,100,(0,0,0),(0,0,0,1),(0,0,0),100.7)
        self.assertEqual(state.problem(100,100.7,True,100),'lio_odometry_stale')

    def test_jump_and_backward_time_latch_until_new_session(self):
        for stamp,position,reason in ((99.9,(0,0,0),'odometry_time_reversed'),
                                     (100.1,(1,0,0),'odometry_discontinuity')):
            state=self.seed();state.observe('nav',stamp,position,(0,0,0,1),(0,0,0),100.1)
            state.observe('nav',100.2,(0,0,0),(0,0,0,1),(0,0,0),100.2)
            self.assertEqual(state.problem(100.2,100.2,True,100.2),reason)

    def test_bad_quaternion_and_dynamic_tf_are_rejected(self):
        state=self.seed();state.observe('nav',100.1,(0,0,0),(0,0,0,0),(0,0,0),100.1)
        self.assertEqual(state.problem(100.1,100.1,True,100.1),'invalid_odometry_quaternion')
        self.assertEqual(self.seed().problem(100,100,True,0),'robot_tf_stale')


if __name__=='__main__': unittest.main()
