#!/usr/bin/env python3
"""Replay the same real cloud/TF interval through unchanged GO2 terrain code."""
import json,os,signal,subprocess,time,uuid
from pathlib import Path
import rosbag,rosgraph,rospy
from diagnostic_msgs.msg import DiagnosticArray
WS=Path(__file__).resolve().parents[1]
root=WS/'artifacts/engineering_20260912_203137_ef59c413'
source=WS/'logs/20260912_211901_cf3101a8/sensors.bag'
excerpt=root/'terrain_interval_v2.bag'
if not excerpt.exists():
 with rosbag.Bag(str(source)) as src,rosbag.Bag(str(excerpt),'w') as dst:
  begin=src.get_start_time()+45;end=begin+50;statics=[]
  for topic,msg,stamp in src.read_messages(topics=['/tf_static','/tf','/cloud_registered_base']):
   if topic=='/tf_static':statics.append(msg)
   if begin<=stamp.to_sec()<=end and topic!='/tf_static':dst.write(topic,msg,stamp)
  for msg in statics:dst.write('/tf_static',msg,rospy.Time.from_sec(begin),connection_header={'type':msg._type,'md5sum':msg._md5sum,'message_definition':msg._full_text,'latching':'1'})
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11323';os.environ['ROS_IP']='127.0.0.1'
if rosgraph.is_master_online():raise RuntimeError('Test master already in use')
for leaf in ['0.08','0.05']:
 folder=root/('terrain_density_'+leaf);folder.mkdir(exist_ok=True)
 launch=folder/'test.launch'
 launch.write_text('''<launch><param name="use_sim_time" value="true"/><include file="$(find go2_terrain)/launch/runtime_terrain.launch"/><param name="/go2_terrain_cloud_adapter/voxel_leaf_size" value="'''+leaf+'''"/></launch>''')
 env=os.environ.copy();env['ROS_HOME']=str(folder/'ros_home');env['ROS_LOG_DIR']=str(folder/'ros_logs')
 out=(folder/'console.log').open('wb');child=subprocess.Popen(['roslaunch','-p','11323',str(launch)],env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
 receiver=None;player=None
 try:
  for i in range(100):
   if rosgraph.is_master_online():break
   time.sleep(.1)
  code='''import rospy,json,time,sys
from diagnostic_msgs.msg import DiagnosticArray
rows=[]
rospy.init_node('terrain_density_receiver',anonymous=True,disable_signals=True)
def cb(m):
 for s in m.status:rows.append(dict(stamp=m.header.stamp.to_sec(),message=s.message,values={v.key:v.value for v in s.values}))
sub=rospy.Subscriber('/terrain/status',DiagnosticArray,cb,queue_size=100)
time.sleep(57)
open(sys.argv[1],'w').write(json.dumps(rows,indent=2))
'''
  receiver=subprocess.Popen(['python3','-c',code,str(folder/'diagnostics.json')],env=env,stdout=out,stderr=subprocess.STDOUT)
  time.sleep(2)
  player=subprocess.Popen(['rosbag','play','--clock','--delay=1',str(excerpt)],env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
  player.wait(timeout=57);receiver.wait(timeout=10)
 finally:
  for proc in [player,child]:
   if proc and proc.poll() is None:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=30)
  if receiver and receiver.poll() is None:receiver.terminate();receiver.wait(timeout=5)
  out.close()
 rows=json.loads((folder/'diagnostics.json').read_text())
 valid=[r for r in rows if r['values'].get('ground_plane_fit_status')=='valid']
 faults=[r for r in valid if r['values'].get('health_gate_open')=='false']
 result={'leaf':leaf,'frames':len(rows),'valid_planes':len(valid),'closed_gate_frames':len(faults),'reasons':sorted(set(r['message'] for r in faults)),
   'min_connected_area':min([float(r['values']['connected_ground_area_m2']) for r in valid] or [0])}
 (folder/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
