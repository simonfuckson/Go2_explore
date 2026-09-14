#!/usr/bin/env python3
import json
import threading
import time
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from go2_exploration_safety.odometry_health import OdometryHealth


class HealthNode:
    def __init__(self):
        self.lock=threading.RLock()
        self.state=OdometryHealth()
        self.last_reason=object()
        self.require_sensor_health=rospy.get_param('~require_sensor_health',True)
        self.sensor_ok=False;self.sensor_received=0
        self.sensor_sub=rospy.Subscriber('/exploration/sensor_ok',Bool,self.sensor,queue_size=1)
        self.tf=tf2_ros.Buffer()
        self.listener=tf2_ros.TransformListener(self.tf)
        self.publisher=rospy.Publisher('/exploration/odom_ok',Bool,queue_size=1)
        self.status=rospy.Publisher('/exploration/odom_status',String,queue_size=1,latch=True)
        self.subs=[rospy.Subscriber(topic,Odometry,self.receive,callback_args=stream,queue_size=1)
                   for stream,topic in (('lio','/lio/odometry'),('nav','/odom_nav'))]

    def sensor(self,message):
        with self.lock:
            self.sensor_ok=message.data;self.sensor_received=time.monotonic()

    def receive(self,message,stream):
        expected=('lio_odom','body_lio') if stream=='lio' else ('odom','base_footprint')
        with self.lock:
            previous_fault=self.state.fault
            previous=self.state.samples.get(stream)
            stamp=message.header.stamp.to_sec()
            if (message.header.frame_id,message.child_frame_id) != expected:
                self.state.fault='unexpected_odometry_frame'
            else:
                p,q,t=message.pose.pose.position,message.pose.pose.orientation,message.twist.twist
                self.state.observe(stream,stamp,(p.x,p.y,p.z),
                    (q.x,q.y,q.z,q.w),(t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z),time.monotonic())
            if self.state.fault and not previous_fault:
                rospy.logerr('Odometry first continuity fault: %s',json.dumps({
                    'reason':self.state.fault,'stream':stream,'stamp':stamp,
                    'previous_stamp':previous[0] if previous else None,
                    'delta_sec':stamp-previous[0] if previous else None,
                    'received_ros':rospy.Time.now().to_sec(),'received_monotonic':time.monotonic(),
                    'frame':message.header.frame_id,'child':message.child_frame_id,
                    'publisher':getattr(message,'_connection_header',{}).get('callerid'),
                    'motion_evidence':self.state.first_fault_context},sort_keys=True))

    def run(self):
        while not rospy.is_shutdown():
            try:
                stamp=self.tf.lookup_transform('odom','base_link',rospy.Time(0),rospy.Duration(.02)).header.stamp.to_sec()
            except tf2_ros.TransformException:
                stamp=0
            with self.lock:
                now_ros,now_wall=rospy.Time.now().to_sec(),time.monotonic()
                reason=self.state.problem(now_ros,now_wall,
                    rospy.get_param_cached('/go2_pose_adapter/initialized',False),stamp)
                if self.require_sensor_health and (not self.sensor_ok or now_wall-self.sensor_received>.3):
                    reason='sensor_stream_unhealthy: see /exploration/sensor_status'
                if reason != self.last_reason:
                    details={'reason':reason or 'healthy','ros_now':now_ros,'monotonic_now':now_wall,'tf_stamp':stamp,
                        'streams':{key:{'stamp':v[0],'stamp_age_sec':now_ros-v[0],'receipt_age_sec':now_wall-v[3]}
                                   for key,v in self.state.samples.items()}}
                    (rospy.logwarn if reason else rospy.loginfo)('Odometry health transition: %s',json.dumps(details,sort_keys=True))
                    self.last_reason=reason
            self.publisher.publish(Bool(data=reason is None))
            self.status.publish(String(data=reason or 'healthy'))
            time.sleep(.05)  # wall clock remains active if /clock or sensors freeze


if __name__=='__main__':
    rospy.init_node('go2_exploration_odom_health')
    HealthNode().run()
