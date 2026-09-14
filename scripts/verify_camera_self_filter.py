#!/usr/bin/env python3
"""ROS transport test with synthetic Livox/IMU; no driver, planner or SDK."""
import copy,fcntl,io,json,math,os,signal,subprocess,threading,time
from pathlib import Path
import numpy as np
WS=Path(__file__).resolve().parents[1]
from session import LOCK,active_session,conflicts
lock=LOCK.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
if active_session() or conflicts():raise RuntimeError('Another stack is active')
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11324';os.environ['ROS_IP']='127.0.0.1'
import rosgraph,rospy,yaml
from livox_ros_driver2.msg import CustomMsg,CustomPoint
from sensor_msgs.msg import Imu,PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32,read_points
from std_msgs.msg import Bool,String,Header
out=WS/'artifacts/camera_install_4be378d3';log=(out/'transport_test.log').open('wb')
if rosgraph.is_master_online():raise RuntimeError('Test master occupied')
master=subprocess.Popen(['roscore','-p','11324'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
child=None;stop=threading.Event();thread=None;result={};latest={};sent={};scenario=['normal'];seq=[0]
def await_condition(condition,seconds=8):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  if condition():return
  time.sleep(.02)
 raise AssertionError('condition timeout')
def serialized(m):
 b=io.BytesIO();m.serialize(b);return b.getvalue()
try:
 await_condition(rosgraph.is_master_online)
 rospy.init_node('self_filter_transport_test',anonymous=True,disable_signals=True)
 mount={'x':.187,'y':0.,'z':.16,'roll_deg':-.1,'pitch_deg':39.,'yaw_deg':0.}
 rospy.set_param('/mid360_mount/base_link_to_lidar_link',mount)
 config=yaml.safe_load((WS/'src/go2_exploration/config/camera_self_filter.yaml').read_text())
 config['calibration_id']='synthetic_fixture_only';config['boxes']=[dict(name='fixture_camera',center=[.45,0,.05],size=[.04,.10,.04],rpy_deg=[0.,0.,0.],padding=.005)]
 pubs={'lidar':rospy.Publisher('/livox/lidar',CustomMsg,queue_size=1), 'imu':rospy.Publisher('/livox/imu',Imu,queue_size=100), 'clearing':rospy.Publisher('/exploration/clearing_points',PointCloud2,queue_size=1), 'ground':rospy.Publisher('/terrain/ground_points',PointCloud2,queue_size=1)}
 def receive(m,key):latest[key]=m
 subs=[rospy.Subscriber(t,k,receive,callback_args=key,queue_size=100) for t,k,key in [('/exploration/livox/lidar_validated',CustomMsg,'lidar'),('/exploration/self_filter/clearing_points',PointCloud2,'clearing'),('/exploration/self_filter/ground_points',PointCloud2,'ground'),('/exploration/sensor_ok',Bool,'healthy'),('/exploration/self_filter/status',String,'status')]]
 from tf.transformations import euler_matrix
 rotation=euler_matrix(-.1*math.pi/180,39*math.pi/180,0)[:3,:3];origin=np.array([.187,0,.16])
 camera=np.array([.45,0,.05]);shadow=origin+2*(camera-origin);before=np.array([.42,0,.07]);side=np.array([.45,.061,.05]);floor=np.array([.45,0,-.3])
 fixtures=[camera,shadow,before,side,floor]
 def stream():
  tick=0
  while not stop.is_set():
   stamp=rospy.Time.now();imu=Imu();imu.header.stamp=stamp;imu.linear_acceleration.z=9.81;pubs['imu'].publish(imu)
   if tick%10==0:
    seq[0]+=1;m=CustomMsg();m.header=Header(seq=seq[0],stamp=stamp-rospy.Duration(.105),frame_id='livox_frame');m.timebase=m.header.stamp.to_nsec();m.lidar_id=7;m.rsvd=[1,2,3]
    for i in range(2000):
     p=CustomPoint();p.x=2.;p.y=1.;p.z=.2;p.offset_time=i*50000;p.line=i%4;p.tag=16;p.reflectivity=21;m.points.append(p)
    for i,b in enumerate(fixtures):
     v=rotation.T@(b-origin);p=CustomPoint();p.x,p.y,p.z=v.tolist();p.offset_time=100000000+i*10000;p.line=3;p.reflectivity=77+i;m.points.append(p)
    if scenario[0]=='excess':
     for i in range(500):m.points[i]=copy.deepcopy(m.points[2000])
    m.point_num=len(m.points);sent[m.header.stamp.to_nsec()]=copy.deepcopy(m)
    for key in sorted(k for k in sent if isinstance(k,int))[:-20]:del sent[key]
    pubs['lidar'].publish(m)
    support=create_cloud_xyz32(Header(seq=m.header.seq,stamp=stamp,frame_id='base_link'),[p.tolist() for p in fixtures])
    pubs['clearing'].publish(support);pubs['ground'].publish(support);sent['support']=support
   tick+=1;stop.wait(.01)
 thread=threading.Thread(target=stream);thread.start()
 for enabled in [False,True]:
  config['enabled']=enabled;rospy.set_param('/go2_exploration_sensor_guard/self_filter',config);latest.clear()
  child=subprocess.Popen(['rosrun','go2_exploration','go2_exploration_sensor_guard'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  await_condition(lambda:latest.get('healthy',Bool()).data)
  await_condition(lambda:'lidar' in latest and 'clearing' in latest and 'ground' in latest)
  received=latest['lidar'];source=copy.deepcopy(sent[received.header.stamp.to_nsec()])
  # ROS publishers assign their own sequence number. Sensor acquisition times
  # and all point metadata must survive; output publication sequence may change.
  source.header.seq=received.header.seq
  assert received.header==source.header and received.timebase==source.timebase and received.lidar_id==7 and list(received.rsvd)==[1,2,3]
  if not enabled:
   assert serialized(received)==serialized(source),'preview altered raw Livox data'
   assert len(list(read_points(latest['clearing'])))==5
   assert len(list(read_points(latest['ground'])))==5
   result['disabled_and_preview_preserve_input']=True
  else:
   assert received.point_num==source.point_num-2 and len(received.points)==received.point_num
   assert [p.reflectivity for p in received.points[-3:]]==[79,80,81],'outside camera obstacles or floor were removed'
   assert [p.offset_time for p in received.points[-3:]]==[p.offset_time for p in source.points[-3:]],'point timing changed'
   clear=np.array(list(read_points(latest['clearing'],field_names=('x','y','z'))))
   assert clear.shape==(3,3) and np.allclose(clear,np.array(fixtures[2:])),clear
   result['camera_and_shadow_removed_outside_obstacles_preserved']=True
   result['timestamps_fields_and_point_offsets_preserved']=True
   scenario[0]='excess'
   await_condition(lambda:latest.get('status',String()).data.startswith('FAULT:'))
   await_condition(lambda:not latest.get('healthy',Bool(data=True)).data)
   last_stamp=latest['lidar'].header.stamp;scenario[0]='normal';time.sleep(.5)
   assert latest['lidar'].header.stamp==last_stamp and not latest['healthy'].data
   result['excess_filter_fault_stops_feed_without_auto_recovery']=True
  os.killpg(child.pid,signal.SIGINT);child.wait(timeout=10);child=None;time.sleep(.3)
 result['passed']=True
finally:
 stop.set()
 if thread:thread.join(timeout=2)
 for proc in [child,master]:
  if proc and proc.poll() is None:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=15)
 log.close();(out/'transport_test.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
