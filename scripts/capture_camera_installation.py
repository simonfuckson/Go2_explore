#!/usr/bin/env python3
"""Bounded static calibration capture using real sensors and observe/mock only."""
import gzip,json,os,signal,subprocess,time,uuid
from pathlib import Path
WS=Path(__file__).resolve().parents[1]
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11321';os.environ['ROS_IP']='192.168.50.110'
from session import active_session,conflicts
if active_session() or conflicts():raise RuntimeError('An existing stack is active')
import rospy,rosgraph
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from livox_ros_driver2.msg import CustomMsg
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
name='camera_install_'+uuid.uuid4().hex[:8];directory=WS/'artifacts'/name;directory.mkdir()
log=(directory/'launcher.log').open('wb')
child=subprocess.Popen([str(WS/'run_go2_explore'),'observe',name],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
result={'name':name,'real_sdk':False,'auto_start':False,'frames':{},'tf':{},'statuses':{}}
last={}
try:
 begin=time.monotonic()
 while time.monotonic()-begin<30:
  if child.poll() is not None:raise RuntimeError('Observe exited during startup')
  session=active_session()
  if session and session['map_name']==name and rosgraph.is_master_online():break
  time.sleep(.1)
 assert session and session['map_name']==name and not session['real_sdk'] and not session['auto_start']
 result['session']=session
 rospy.init_node('capture_camera_installation',anonymous=True,disable_signals=True)
 def cloud(m,topic):
  now=time.monotonic()
  if now-last.get(topic,0)<.45:return
  last[topic]=now
  points=[list(p) for p in read_points(m,field_names=('x','y','z','intensity'),skip_nans=True)
          if p[0]*p[0]+p[1]*p[1]+p[2]*p[2]<2.25]
  result['frames'].setdefault(topic,[]).append({'stamp':m.header.stamp.to_sec(),'frame':m.header.frame_id,'points':points})
 def lidar(m):
  now=time.monotonic()
  if now-last.get('lidar',0)<.45:return
  last['lidar']=now
  points=[[p.x,p.y,p.z,p.reflectivity,p.tag,p.line,p.offset_time] for p in m.points
          if .01<p.x*p.x+p.y*p.y+p.z*p.z<2.25]
  result['frames'].setdefault('/livox/lidar',[]).append({'stamp':m.header.stamp.to_sec(),'frame':m.header.frame_id,'points':points})
 def tf(m):
  for t in m.transforms:
   p=t.transform.translation;q=t.transform.rotation
   result['tf'][t.child_frame_id]={'parent':t.header.frame_id,'xyz':[p.x,p.y,p.z],'xyzw':[q.x,q.y,q.z,q.w],'stamp':t.header.stamp.to_sec()}
 def status(m,topic):result['statuses'][topic]=m.data
 subs=[rospy.Subscriber(t,PointCloud2,cloud,callback_args=t,queue_size=1) for t in ['/cloud_registered_base','/cloud_registered_terrain','/terrain/obstacle_points','/terrain/ground_points']]
 subs += [rospy.Subscriber('/livox/lidar',CustomMsg,lidar,queue_size=1),rospy.Subscriber('/tf',TFMessage,tf,queue_size=100),rospy.Subscriber('/tf_static',TFMessage,tf,queue_size=20)]
 subs += [rospy.Subscriber(t,String,status,callback_args=t,queue_size=1) for t in ['/exploration/odom_status','/exploration/sensor_status','/exploration/clearing_status','/exploration/self_filter/status']]
 begin=time.monotonic()
 while time.monotonic()-begin<40:
  if child.poll() is not None:raise RuntimeError('Observe exited during capture')
  time.sleep(.2)
 result['parameters']={n:rospy.get_param(n) for n in ['/mid360_mount','/d435i_mount']}
finally:
 current=active_session()
 if current and current['map_name']==name and not current['real_sdk']:
  stop=subprocess.run([str(WS/'run_go2_explore'),'stop'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=100)
  result['stop_returncode']=stop.returncode;result['stop_output']=stop.stdout
 if child.poll() is None:child.wait(timeout=45)
 log.close()
 with gzip.open(str(directory/'capture.json.gz'),'wt') as out:json.dump(result,out)
 print(json.dumps({'directory':str(directory),'counts':{k:len(v) for k,v in result['frames'].items()},'statuses':result['statuses'],'stop_returncode':result.get('stop_returncode'),'bytes':(directory/'capture.json.gz').stat().st_size},ensure_ascii=False),flush=True)
