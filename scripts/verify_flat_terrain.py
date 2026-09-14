#!/usr/bin/env python3
"""Exercise both terrain modes with the same clouds; never starts an SDK node."""
import json,os,signal,subprocess,time,uuid
from pathlib import Path
import rosgraph,rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs.point_cloud2 import create_cloud,read_points
from std_msgs.msg import Header

WS=Path('/home/nvidia/go2_explore_ws')
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11326'
os.environ['ROS_IP']='127.0.0.1'
if rosgraph.is_master_online():raise RuntimeError('Test master 11326 is already in use')
folder=WS/'artifacts/flat_ground_20260914'/('perception_'+uuid.uuid4().hex[:8]);folder.mkdir(parents=True)
nodes=[]
for mode,geometry in [('strict','true'),('flat','false')]:
 nodes.append('''<group ns="%s"><node pkg="go2_terrain" type="go2_terrain_guard_node" name="guard" required="true" output="screen">
 <rosparam command="load" file="$(find go2_terrain)/config/terrain_guard_go2.yaml"/>
 <param name="health/require_ground_geometry" value="%s"/>
 <param name="output/publish_debug_clouds" value="true"/>
 <remap from="ground" to="/test/ground"/><remap from="nonground" to="/test/nonground"/>
 </node></group>'''%(mode,geometry))
launch=folder/'test.launch';launch.write_text('<launch>'+''.join(nodes)+'</launch>')
env=os.environ.copy();env['ROS_LOG_DIR']=str(folder/'ros')
log=(folder/'console.log').open('wb')
child=subprocess.Popen(['roslaunch','-p','11326',str(launch)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
latest={};clouds={};report={};subs=[]
def receive(m,mode):
 for s in m.status:latest[mode]=dict(message=s.message,level=s.level,values={v.key:v.value for v in s.values})
def cloud(m,key):clouds[key]=m
def wait(predicate,seconds=8):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  if child.poll() is not None:raise AssertionError('Perception process exited')
  if predicate():return
  time.sleep(.02)
 raise AssertionError('Timed out: '+json.dumps(latest))
try:
 wait(rosgraph.is_master_online,15)
 rospy.init_node('verify_flat_terrain',anonymous=True,disable_signals=True)
 for mode in ('strict','flat'):
  subs.append(rospy.Subscriber('/'+mode+'/diagnostics',DiagnosticArray,receive,callback_args=mode,queue_size=50))
  for topic in ('obstacles','clearing','safe_ground','unknown'):
   key=mode+'/'+topic
   subs.append(rospy.Subscriber('/'+key,PointCloud2,cloud,callback_args=key,queue_size=10))
 pubs=[rospy.Publisher('/test/'+topic,PointCloud2,queue_size=1) for topic in ('ground','nonground')]
 wait(lambda:all(p.get_num_connections()==2 for p in pubs))
 fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z','intensity'))]
 # Seven connected floor cells with 28 returns (0.1575 m2), plus a real box.
 floor=[(.475+.15*i+dx,-.025+dy,-.51,1.) for i in range(7) for dx,dy in ((-.02,-.02),(-.02,.02),(.02,-.02),(.02,.02))]
 obstacle=[(.60+.01*i,.02,-.30+.02*j,1.) for i in range(3) for j in range(5)]
 def publish(seconds,kind='normal',hz=10):
  deadline=time.monotonic()+seconds
  while time.monotonic()<deadline:
   h=Header(stamp=rospy.Time.now(),frame_id='wrong_frame' if kind=='wrong_frame' else 'terrain_sensor')
   for p,points in zip(pubs,(floor,obstacle)):
    p.publish(create_cloud(h,fields,[] if kind=='empty' else points))
   time.sleep(1./hz)
 publish(2.5)
 wait(lambda:latest.get('flat',{}).get('values',{}).get('health_gate_open')=='true')
 assert latest['strict']['values']['health_gate_open']=='false'
 assert latest['flat']['values']['ground_geometry_checks']=='disabled'
 report['sparse_ground']=dict(strict=latest['strict'],flat=latest['flat'])
 # Match outputs from exactly the same source stamp, even across callbacks.
 def same_clouds():
  for topic in ('obstacles','clearing','safe_ground','unknown'):
   a,b=clouds.get('strict/'+topic),clouds.get('flat/'+topic)
   if a is None or b is None or a.header.stamp!=b.header.stamp:return False
   schema=lambda m:[(f.name,f.offset,f.datatype,f.count) for f in m.fields]
   # PCL PointXYZI includes uninitialized alignment padding; compare every
   # published field, not padding bytes that have no geometric meaning.
   decode=lambda m:list(read_points(m,field_names=('x','y','z','intensity'),skip_nans=False))
   if (schema(a),a.width,a.height,decode(a))!=(schema(b),b.width,b.height,decode(b)):
    raise AssertionError('Obstacle/clearing geometry changed: '+topic)
   report.setdefault('cloud_comparison',{})[topic]=dict(points=a.width*a.height,
       all_fields_equal=True,padding_bytes_differ=a.data!=b.data)
  return True
 wait(same_clouds)
 assert clouds['flat/obstacles'].width>=len(obstacle),'Box disappeared from obstacle output'
 report['identical_obstacle_and_clearing_outputs']=True
 report['obstacle_points']=clouds['flat/obstacles'].width
 for fault in ('wrong_frame','empty','slow','stale'):
  if fault=='stale':time.sleep(.85)
  else:publish(1.8,kind=fault,hz=2 if fault=='slow' else 10)
  wait(lambda:latest['flat']['values']['health_gate_open']=='false',2)
  report[fault]=latest['flat']
  publish(2.5)
  wait(lambda:latest['flat']['values']['health_gate_open']=='true')
 report['passed']=True
finally:
 for sub in subs:sub.unregister()
 if child.poll() is None:
  os.killpg(child.pid,signal.SIGINT);child.wait(timeout=20)
 log.close();(folder/'result.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(dict(directory=str(folder),passed=report.get('passed',False))),flush=True)
