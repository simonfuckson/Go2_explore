import importlib.util
import io
from pathlib import Path
import threading
import unittest
from unittest.mock import patch,Mock
from nav_msgs.msg import Odometry, Path as NavPath
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

path=Path(__file__).resolve().parents[1]/'scripts/exploration_dashboard.py'
spec=importlib.util.spec_from_file_location('dashboard',str(path))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class DashboardEvidence(unittest.TestCase):
    def test_no_command_is_not_reported_as_successful_stepping(self):
        text,level=module.motion_evidence({'last_nonzero_command_age_sec':'-1','no_step_response':'false'})
        self.assertIn('无法判断',text);self.assertEqual(level,1)
        text,level=module.motion_evidence({'last_nonzero_command_age_sec':'-1','no_step_response':'true'})
        self.assertIn('触发保护',text);self.assertEqual(level,2)
    def setUp(self):
        self.node=module.Dashboard.__new__(module.Dashboard)
        self.node.lock=threading.RLock();self.node.latest={};self.node.first_failure=None
        self.node.saw_enabled=False;self.node.log=io.StringIO()
    def terrain(self,healthy,reason):
        s=DiagnosticStatus(name='terrain',message=reason,level=0 if healthy else 2,
          values=[KeyValue(key='health_gate_open',value='true' if healthy else 'false')])
        with patch.object(module.rospy.Time,'now',return_value=module.rospy.Time(123)):
            self.node.receive(DiagnosticArray(status=[s]),'/terrain/status')
    def test_preserves_first_failure_even_if_disable_arrives_first(self):
        self.node.receive(Bool(data=True),'/go2/control/enabled')
        self.node.receive(Bool(data=False),'/go2/control/enabled')
        self.terrain(False,'terrain input stale')
        self.terrain(True,'healthy')
        self.terrain(False,'different later failure')
        self.assertEqual(self.node.first_failure['diagnostic']['message'],'terrain input stale')
        self.assertEqual(len(self.node.log.getvalue().splitlines()),1)
    def test_initial_sensor_startup_does_not_replace_operational_failure(self):
        self.terrain(False,'waiting for input')
        self.assertIsNone(self.node.first_failure)
    def test_missing_and_stale_velocity_are_visible(self):
        with patch.object(module.time,'monotonic',return_value=10.):
            self.node.receive(Twist(),'/cmd_vel_safe')
        with patch.object(module.time,'monotonic',return_value=11.), \
             patch.object(module.rospy.Time,'now',return_value=module.rospy.Time(123)):
            rows={r['name']:r for r in self.node.snapshot()['rows']}
        self.assertEqual(rows['规划速度']['level'],3)
        self.assertEqual(rows['安全输出速度']['level'],3)
        self.assertIn('过期',rows['安全输出速度']['value'])

    def test_public_trajectory_preserves_body_frame_and_ignores_raw_lio(self):
        self.node.trajectory=NavPath();self.node.trajectory_pub=Mock()
        message=Odometry();message.header.frame_id='lio_odom'
        self.node.odometry(message)
        self.assertEqual(len(self.node.trajectory.poses),0)
        message.header.frame_id='odom';message.header.stamp=module.rospy.Time(100)
        message.child_frame_id='base_link';message.pose.pose.orientation.w=1
        self.node.odometry(message)
        self.assertEqual(self.node.trajectory.header.frame_id,'odom')
        self.assertEqual(self.node.trajectory.poses[0].header.stamp,message.header.stamp)
        self.assertEqual(len(self.node.trajectory.poses),1)
        message=Odometry();message.header.frame_id='odom';message.pose.pose.position.x=.01
        self.node.odometry(message)
        self.assertEqual(len(self.node.trajectory.poses),1)
        message.pose.pose.position.x=.04
        self.node.odometry(message)
        self.assertEqual(len(self.node.trajectory.poses),2)

    def test_flat_mode_labels_support_as_information_and_keeps_stream_fault_visible(self):
        values=[KeyValue(key='ground_geometry_checks',value='disabled'),
                KeyValue(key='near_support_area_m2',value='0.1575')]
        for level in (0,2):
            status=DiagnosticStatus(name='terrain',level=level,
                message='terrain input stale' if level else 'perception ready',values=values)
            self.node.receive(DiagnosticArray(status=[status]),'/terrain/status')
            with patch.object(module.rospy.Time,'now',return_value=module.rospy.Time(123)):
                rows={r['name']:r for r in self.node.snapshot()['rows']}
            self.assertNotIn('地形健康',rows)
            self.assertEqual(rows['避障感知']['level'],level)
            self.assertEqual(rows['地面支撑（仅显示）']['level'],0)
            if level:self.assertIn('stale',rows['避障感知']['value'])

if __name__=='__main__':unittest.main()
