#!/usr/bin/env python3
"""Repeat a measured stationary scene on a mock-only ROS graph."""
import copy,gzip,json,sys,time
import numpy as np
import rospy,tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointField
from sensor_msgs.point_cloud2 import create_cloud
from std_msgs.msg import Header,Bool
from tf.transformations import quaternion_matrix,quaternion_from_euler

rospy.init_node('recorded_stationary_scene')
if rospy.get_param('/exploration/runtime_mode','')!='simulation':
    raise RuntimeError('Recorded scene requires simulation mode')
d=json.load(gzip.open(sys.argv[1],'rt'));p=d['pose'];q=p['q']
yaw=np.arctan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
tf=tf2_ros.TransformBroadcaster();static=tf2_ros.StaticTransformBroadcaster()
def transform(parent,child,xyz,quat):
    t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=rospy.Time.now()
    t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=xyz
    t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=quat
    return t
# Terrain frame is gravity aligned at the measured lidar origin.
rot=quaternion_matrix(q)[:3,:3];origin=np.array([p['x'],p['y'],p['z']]);sensor=origin+rot.dot([.187,0,.16])
static.sendTransform([transform('base_link','lidar_link',[.187,0,.16],quaternion_from_euler(-.1*np.pi/180,39*np.pi/180,0)),transform('odom','lio_odom',[0,0,0],[0,0,0,1])])
clouds=dict(d['clouds']);base=np.asarray(clouds['/cloud_registered_base']['xyz'])
clouds['/cloud_registered_odom']={'frame':'odom','xyz':base.dot(rot.T)+origin}
fields=[PointField(n,i*4,PointField.FLOAT32,1) for i,n in enumerate(('x','y','z','intensity'))]
messages={t:create_cloud(Header(frame_id=v['frame']),fields,np.column_stack((v['xyz'],np.ones(len(v['xyz'])))).tolist()) for t,v in clouds.items()}
pubs={t:rospy.Publisher(t,type(m),queue_size=1) for t,m in messages.items()}
odoms={t:rospy.Publisher(t,Odometry,queue_size=1) for t in ('/lio/odometry','/odom_nav','/odom_robot')}
healthy=rospy.Publisher('/terrain/healthy',Bool,queue_size=1)
rospy.set_param('/go2_pose_adapter/initialized',True)
while not rospy.is_shutdown():
    stamp=rospy.Time.now()
    tf.sendTransform([transform('odom','base_link',origin,q),transform('odom','base_footprint',[p['x'],p['y'],0],quaternion_from_euler(0,0,yaw)),transform('odom','terrain_sensor',sensor,quaternion_from_euler(0,0,yaw))])
    for t,pub in odoms.items():
        m=Odometry();m.header.stamp=stamp;m.header.frame_id='lio_odom' if t=='/lio/odometry' else 'odom'
        m.child_frame_id={'/lio/odometry':'body_lio','/odom_nav':'base_footprint','/odom_robot':'base_link'}[t]
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=origin
        m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=q
        pub.publish(m)
    for t,m in messages.items():m.header.stamp=stamp;pubs[t].publish(m)
    healthy.publish(Bool(data=True));time.sleep(.1)
