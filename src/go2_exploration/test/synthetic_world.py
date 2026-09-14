#!/usr/bin/env python3
"""Deterministic, entirely virtual sensor world. Contains no Unitree SDK calls."""
import math
import threading
import time
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped,Twist
from nav_msgs.msg import Odometry
from sensor_msgs.point_cloud2 import create_cloud_xyz32,create_cloud
from sensor_msgs.msg import PointField
from std_msgs.msg import Bool,Header
from tf.transformations import quaternion_from_euler


class World:
    def __init__(self):
        self.x=self.y=self.yaw=0.
        self.command=Twist();self.command_time=0.;self.enabled=False
        self.tf=tf2_ros.TransformBroadcaster()
        self.static=tf2_ros.StaticTransformBroadcaster()
        self.static.sendTransform([self.transform('base_link','terrain_sensor',.187,0,.16,0),
            self.transform('base_link','lidar_link',.187,0,.16,0),
            self.transform('odom','lio_odom',0,0,0,0)])
        self.clouds={topic:rospy.Publisher(topic,create_cloud_xyz32(Header(),[]).__class__,queue_size=1)
                     for topic in ('/cloud_registered_base','/cloud_registered_odom','/cloud_registered_terrain',
                                   '/terrain/ground_points','/terrain/obstacle_points','/terrain/clearing_points')}
        self.odom={topic:rospy.Publisher(topic,Odometry,queue_size=1) for topic in ('/lio/odometry','/odom_nav','/odom_robot')}
        self.healthy=rospy.Publisher('/terrain/healthy',Bool,queue_size=1)
        self.subs=[rospy.Subscriber('/cmd_vel_safe',Twist,self.velocity,queue_size=1),
                   rospy.Subscriber('/go2/control/enabled',Bool,self.control,queue_size=1)]
        rospy.set_param('/go2_pose_adapter/initialized',True)

    def velocity(self,message):
        self.command=message;self.command_time=time.monotonic()

    def control(self,message): self.enabled=message.data

    @staticmethod
    def transform(parent,child,x,y,z,yaw):
        t=TransformStamped();t.header.stamp=rospy.Time.now();t.header.frame_id=parent;t.child_frame_id=child
        t.transform.translation.x=x;t.transform.translation.y=y;t.transform.translation.z=z
        q=quaternion_from_euler(0,0,yaw)
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=q
        return t

    def run(self):
        last=time.monotonic();tick=0
        while not rospy.is_shutdown():
            now=time.monotonic();dt=min(.1,now-last);last=now
            fault=rospy.get_param('/simulation/fault','')
            moving=self.enabled and now-self.command_time<.25
            v=self.command.linear.x if moving else 0
            w=self.command.angular.z if moving else 0
            self.x+=v*math.cos(self.yaw)*dt;self.y+=v*math.sin(self.yaw)*dt;self.yaw+=w*dt
            stamp=rospy.Time.now()
            if fault!='odom':
                if fault!='tf':
                    self.tf.sendTransform([self.transform('odom','base_link',self.x,self.y,0,self.yaw),
                                           self.transform('odom','base_footprint',self.x,self.y,0,self.yaw)])
                for topic,pub in self.odom.items():
                    msg=Odometry();msg.header.stamp=stamp
                    msg.header.frame_id='lio_odom' if topic=='/lio/odometry' else 'odom'
                    msg.child_frame_id={'/lio/odometry':'body_lio','/odom_nav':'base_footprint','/odom_robot':'base_link'}[topic]
                    msg.pose.pose.position.x=self.x;msg.pose.pose.position.y=self.y
                    msg.pose.pose.orientation.z=math.sin(self.yaw/2);msg.pose.pose.orientation.w=math.cos(self.yaw/2)
                    msg.twist.twist.linear.x=v;msg.twist.twist.angular.z=w
                    pub.publish(msg)
            self.healthy.publish(Bool(data=fault!='terrain'))
            if tick%2==0 and fault!='cloud': self.publish_clouds(stamp,fault)
            tick+=1
            time.sleep(.05)

    def publish_clouds(self,stamp,fault):
        xx,yy=np.meshgrid(np.arange(-3,5,.15),np.arange(-3,3,.15))
        world=np.column_stack((xx.ravel(),yy.ravel(),np.full(xx.size,-.35)))
        distance=np.linalg.norm(world[:,:2]-[self.x,self.y],axis=1)
        world=world[(distance<4.5)&(distance>.40)]
        walls=[]
        for value in np.arange(-3,5.01,.12):
            for z in (-.25,.1,.5): walls.extend(((value,-3,z),(value,3,z)))
        for value in np.arange(-3,3.01,.12):
            for z in (-.25,.1,.5): walls.extend(((-3,value,z),(5,value,z)))
        walls=np.asarray(walls)
        walls=walls[np.linalg.norm(walls[:,:2]-[self.x,self.y],axis=1)<4.5]
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        def to_base(points):
            shifted=points-[self.x,self.y,0]
            return np.column_stack((c*shifted[:,0]+s*shifted[:,1],-s*shifted[:,0]+c*shifted[:,1],shifted[:,2]))
        floor=to_base(world);obstacles=to_base(walls)
        if fault=='obstacle':
            obstacles=np.vstack((obstacles,[[.48,y,z] for y in (-.03,0,.03) for z in (-.1,0,.1,.2)]))
        raw=np.vstack((floor,obstacles));offset=np.array([.187,0,.16])
        ground=floor[floor[:,0]>.187]
        support=ground-offset
        keys=np.unique(np.floor((support[:,:2]-[-5.,-4.])/.15).astype(int),axis=0)
        support=np.column_stack((np.array([-5.,-4.])+(keys+.5)*.15,np.full(len(keys),-.51)))
        support=support[np.linalg.norm(support[:,:2],axis=1)>=.35]
        values={'/cloud_registered_base':('base_link',raw),
                '/cloud_registered_odom':('odom',np.vstack((world,walls))),
                '/cloud_registered_terrain':('terrain_sensor',raw-offset),
                '/terrain/ground_points':('terrain_sensor',support),
                '/terrain/obstacle_points':('terrain_sensor',obstacles-offset),
                '/terrain/clearing_points':('terrain_sensor',support)}
        for topic,(frame,points) in values.items():
            fields=[PointField(name,index*4,PointField.FLOAT32,1) for index,name in enumerate(('x','y','z','intensity'))]
            self.clouds[topic].publish(create_cloud(Header(stamp=stamp,frame_id=frame),fields,
                np.column_stack((points,np.ones(len(points)))).tolist()))


if __name__=='__main__':
    rospy.init_node('synthetic_world')
    World().run()
