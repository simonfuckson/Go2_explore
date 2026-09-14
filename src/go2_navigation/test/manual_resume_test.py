#!/usr/bin/env python3
"""Exercise the real supervisor against fake services/action server, never DDS."""
import os
import time
import unittest
from urllib.parse import urlparse
import actionlib
import rospy
import rostest
from actionlib_msgs.msg import GoalID, GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse


class ManualResumeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if urlparse(os.environ.get('ROS_MASTER_URI', '')).port == 11311:
            raise RuntimeError('Refusing production/default ROS master')
        rospy.init_node('manual_resume_test')
        cls.goals = []
        cls.loc_ok = True
        cls.terrain_ok = True
        cls.ready = False
        cls.resume_calls = 0
        cls.resume_ok = True
        cls.resume_delay = 0
        cls.control = rospy.Publisher('/go2/control/state', String, queue_size=10, latch=True)
        cls.legacy = rospy.Publisher('/go2/control/enabled', Bool, queue_size=10, latch=True)
        cls.loc = rospy.Publisher('/localization/ok', Bool, queue_size=10, latch=True)
        cls.terrain = rospy.Publisher('/terrain/healthy', Bool, queue_size=10, latch=True)
        cls.simple = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        cls.cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        cls.ready_sub = rospy.Subscriber('/navigation/ready', Bool,
                                        lambda msg: setattr(cls, 'ready', msg.data))
        cls.server = actionlib.SimpleActionServer('/move_base_internal', MoveBaseAction, auto_start=False)
        def goal():
            cls.goals.append(cls.server.accept_new_goal())
        def preempt():
            if not cls.server.is_new_goal_available() and cls.server.is_active():
                cls.server.set_preempted(MoveBaseResult())
        cls.server.register_goal_callback(goal)
        cls.server.register_preempt_callback(preempt)
        cls.server.start()
        cls.clear_service = rospy.Service('/move_base/clear_costmaps', Empty, lambda _: EmptyResponse())
        def resume(_):
            cls.resume_calls += 1
            time.sleep(cls.resume_delay)
            if cls.resume_ok:
                cls.control.publish(String('enabled'))
            return TriggerResponse(cls.resume_ok, 'fake SDK acknowledged' if cls.resume_ok else 'SDK rejected')
        cls.resume_service = rospy.Service('/go2_sdk_bridge_real/resume_after_manual', Trigger, resume)
        cls.heartbeat = rospy.Timer(rospy.Duration(.1),
            lambda _: (cls.loc.publish(Bool(cls.loc_ok)), cls.terrain.publish(Bool(cls.terrain_ok))))
        cls.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        if not cls.client.wait_for_server(rospy.Duration(10)):
            raise RuntimeError('Supervisor unavailable')
        rospy.wait_for_service('/go2_navigation_supervisor/reset', 10)
        cls.reset = rospy.ServiceProxy('/go2_navigation_supervisor/reset', Trigger)

    def wait(self, condition, seconds=5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if condition():
                return
            time.sleep(.02)
        self.assertTrue(condition(), 'Timed out waiting for expected state')

    def setUp(self):
        cls = type(self)
        cls.reset()
        cls.loc_ok = cls.terrain_ok = cls.resume_ok = True
        cls.resume_delay = 0
        cls.control.publish(String('enabled'))
        cls.legacy.publish(Bool(True))
        self.wait(lambda: cls.ready)
        time.sleep(.25)
        cls.goals.clear()
        cls.resume_calls = 0

    def target(self, x):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = x
        g.target_pose.pose.orientation.w = 1
        return g

    def start_goal(self, simple=False):
        if simple:
            self.simple.publish(self.target(1).target_pose)
        else:
            self.client.send_goal(self.target(1))
        self.wait(lambda: len(self.goals) == 1)

    def pause(self):
        self.control.publish(String('manual_override'))
        self.legacy.publish(Bool(False))
        self.wait(lambda: not self.ready and not self.server.is_active())
        time.sleep(.15)  # Deliver old private PREEMPTED result.

    def assert_no_resume(self):
        self.control.publish(String('manual_ready'))
        time.sleep(.65)
        self.assertEqual(1, len(self.goals))
        self.assertEqual(0, type(self).resume_calls)

    def test_action_goal_survives_remote_and_replans(self):
        self.start_goal()
        self.pause()
        self.assertIn(self.client.get_state(), (GoalStatus.ACTIVE, GoalStatus.PENDING))
        self.control.publish(String('manual_ready'))
        self.wait(lambda: len(self.goals) == 2)
        self.assertEqual(1, self.goals[-1].target_pose.pose.position.x)
        self.assertEqual(1, type(self).resume_calls)
        self.server.set_succeeded(MoveBaseResult())
        self.wait(lambda: self.client.get_state() == GoalStatus.SUCCEEDED)

    def test_simple_goal_is_replaced_while_manual(self):
        self.start_goal(simple=True)
        self.pause()
        self.simple.publish(self.target(2).target_pose)
        time.sleep(.2)
        self.assertEqual(1, len(self.goals))
        self.control.publish(String('manual_ready'))
        self.wait(lambda: len(self.goals) == 2)
        self.assertEqual(2, self.goals[-1].target_pose.pose.position.x)

    def test_action_replacement_is_retained(self):
        self.start_goal()
        self.pause()
        self.client.send_goal(self.target(3))
        time.sleep(.2)
        self.control.publish(String('manual_ready'))
        self.wait(lambda: len(self.goals) == 2)
        self.assertEqual(3, self.goals[-1].target_pose.pose.position.x)

    def test_simple_cancel_discards_retained_goal(self):
        self.start_goal(simple=True)
        self.pause()
        self.cancel.publish(GoalID())
        time.sleep(.2)
        self.assert_no_resume()

    def test_action_cancel_discards_retained_goal(self):
        self.start_goal()
        self.pause()
        self.client.cancel_goal()
        self.wait(lambda: self.client.get_state() == GoalStatus.PREEMPTED)
        self.assert_no_resume()

    def test_reset_discards_retained_goal(self):
        self.start_goal()
        self.pause()
        self.reset()
        self.assert_no_resume()

    def test_localization_loss_prevents_resume_after_recovery(self):
        self.start_goal()
        self.pause()
        type(self).loc_ok = False
        self.wait(lambda: self.client.get_state() == GoalStatus.ABORTED)
        type(self).loc_ok = True
        self.assert_no_resume()

    def test_terrain_loss_prevents_resume_after_recovery(self):
        self.start_goal()
        self.pause()
        type(self).terrain_ok = False
        self.wait(lambda: self.client.get_state() == GoalStatus.ABORTED)
        type(self).terrain_ok = True
        self.assert_no_resume()

    def test_explicit_disable_does_not_resume(self):
        self.start_goal()
        self.pause()
        self.control.publish(String('disabled'))
        self.wait(lambda: self.client.get_state() == GoalStatus.ABORTED)
        self.assert_no_resume()

    def test_resume_failure_is_bounded_and_retains_goal(self):
        self.start_goal()
        self.pause()
        type(self).resume_ok = False
        self.control.publish(String('manual_ready'))
        self.wait(lambda: type(self).resume_calls == 1)
        time.sleep(.65)
        self.assertEqual(1, type(self).resume_calls)
        self.assertEqual(1, len(self.goals))
        self.assertEqual(GoalStatus.ACTIVE, self.client.get_state())

    def test_late_legacy_false_cannot_cancel_current_goal(self):
        self.start_goal()
        self.legacy.publish(Bool(False))
        time.sleep(.3)
        self.assertTrue(self.server.is_active())
        self.assertEqual(GoalStatus.ACTIVE, self.client.get_state())

    def test_slow_sdk_resume_does_not_starve_health_callbacks(self):
        self.start_goal()
        self.pause()
        type(self).resume_delay = 1.3
        self.control.publish(String('manual_ready'))
        self.wait(lambda: len(self.goals) == 2)
        self.assertEqual(GoalStatus.ACTIVE, self.client.get_state())

    def test_cancel_during_slow_resume_never_resends_goal(self):
        self.start_goal()
        self.pause()
        type(self).resume_delay = 1.3
        self.control.publish(String('manual_ready'))
        self.wait(lambda: type(self).resume_calls == 1)
        self.client.cancel_goal()
        self.wait(lambda: self.client.get_state() == GoalStatus.PREEMPTED)
        time.sleep(1.7)
        self.assertEqual(1, len(self.goals))


if __name__ == '__main__':
    rostest.rosrun('go2_navigation', 'manual_resume', ManualResumeTest)
