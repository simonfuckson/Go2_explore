#!/usr/bin/env python3
"""User-authorized bounded real trial, retaining the normal autonomy interlocks."""
import ctypes,json,math,os,signal,subprocess,threading,time,uuid
from pathlib import Path
WS=Path(__file__).resolve().parents[1]
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11321';os.environ['ROS_IP']='192.168.50.110'
from session import active_session,conflicts
if active_session() or conflicts():raise RuntimeError('Another robot stack is active')
import rospy,rosgraph
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Bool
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger
name='blind070_real_'+uuid.uuid4().hex[:8];directory=WS/'artifacts'/name;directory.mkdir()
log=(directory/'launcher.log').open('wb');parent=os.getpid()
def parent_guard():
 ctypes.CDLL(None).prctl(1,signal.SIGINT)
 if os.getppid()!=parent:os.kill(os.getpid(),signal.SIGINT)
child=subprocess.Popen([str(WS/'run_go2_explore'),'explore',name,'--real','--lidar-blind','0.70'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,preexec_fn=parent_guard)
lock=threading.RLock();latest={};states=[];goals=[];commands=[];origin=None;pose=None;motion_time=0.;max_displacement=0.;max_yaw=0.;last_command=None;session=None;snapshot={}
result={'name':name,'directory':str(directory),'real_sdk':True,'lidar_blind_m':.70,
 'limits':{'max_displacement_m':.30,'max_nonzero_command_seconds':2.,'max_yaw_change_rad':.20,'max_session_seconds':75}}
def cmd(m):
 global motion_time,last_command
 now=time.monotonic()
 with lock:
  if last_command and abs(last_command[1])+abs(last_command[2])>.001:motion_time+=min(.15,now-last_command[0])
  last_command=(now,m.linear.x,m.angular.z);commands.append(last_command)
def odom(m):
 global origin,pose,max_displacement,max_yaw
 p=m.pose.pose.position;q=m.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 with lock:
  pose=(p.x,p.y,yaw)
  if origin is None:origin=pose
  max_displacement=max(max_displacement,math.hypot(p.x-origin[0],p.y-origin[1]));max_yaw=max(max_yaw,abs(math.atan2(math.sin(yaw-origin[2]),math.cos(yaw-origin[2]))))
def status(m,topic):
 with lock:
  if isinstance(m,DiagnosticArray):latest[topic]=[dict(message=s.message,values={v.key:v.value for v in s.values}) for s in m.status]
  else:latest[topic]=m.data
  if topic=='/exploration/state':states.append(m.data)
try:
 start=time.monotonic()
 while time.monotonic()-start<25:
  if child.poll() is not None:raise RuntimeError('Trial exited during startup')
  session=active_session()
  if session and session['map_name']==name and rosgraph.is_master_online():break
  time.sleep(.05)
 assert session and session['map_name']==name and session['real_sdk'] and session['lidar_blind_m']==.7 and rosgraph.is_master_online()
 result['session']=session
 rospy.init_node('bounded_blind070_trial',anonymous=True,disable_signals=True)
 subs=[rospy.Subscriber('/cmd_vel_safe',Twist,cmd,queue_size=100),rospy.Subscriber('/odom_robot',Odometry,odom,queue_size=100),rospy.Subscriber('/explore/selected_goal',PoseStamped,lambda m:goals.append([m.pose.position.x,m.pose.position.y]),queue_size=10)]
 subs += [rospy.Subscriber(t,k,status,callback_args=t,queue_size=20) for t,k in [('/exploration/state',String),('/exploration/odom_status',String),('/exploration/sensor_status',String),('/exploration/clearing_status',String),('/explore/selection_status',String),('/go2/control/enabled',Bool),('/go2/diagnostics',DiagnosticArray),('/go2_exploration_safety/status',DiagnosticArray),('/terrain/status',DiagnosticArray)]]
 await_deadline=time.monotonic()+10
 while not rospy.has_param('/preprocess/blind') and time.monotonic()<await_deadline:time.sleep(.05)
 assert rospy.get_param('/preprocess/blind')==.7
 result['effective_blind_verified']=True
 last_print=0
 while child.poll() is None:
  elapsed=time.monotonic()-start
  with lock:
   reason=('displacement_limit' if max_displacement>=.30 else 'nonzero_command_time_limit' if motion_time>=2 else 'yaw_limit' if max_yaw>=.20 else 'fault_stop' if 'FAULT_STOPPED' in states else 'session_time_limit' if elapsed>=75 else None)
   snapshot={'seconds':round(elapsed,1),'state':states[-1:] ,'goals':len(goals),'nonzero_commands':sum(abs(v)+abs(w)>.001 for _,v,w in commands),'nonzero_seconds':motion_time,'max_displacement_m':max_displacement,'max_yaw_rad':max_yaw,'latest':dict(latest)}
  if time.monotonic()-last_print>=3:
   last_print=time.monotonic();(directory/'progress.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2));print(json.dumps({k:v for k,v in snapshot.items() if k!='latest'},ensure_ascii=False),flush=True)
  if reason:result['stop_trigger']=reason;break
  time.sleep(.01)
 result.update(snapshot);result['states']=states;result['selected_goals']=goals
finally:
 current=active_session()
 if current and current['map_name']==name:
  # Protective zero/cancel before any save or diagnostic work.
  try:
   rospy.wait_for_service('/go2_exploration_safety/stop',timeout=1.)
   rospy.ServiceProxy('/go2_exploration_safety/stop',Trigger)()
  except Exception as e:result['gate_stop_error']=str(e)
  stop=subprocess.run([str(WS/'run_go2_explore'),'stop'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=100)
  result['stop_returncode']=stop.returncode;result['stop_output']=stop.stdout
 if child.poll() is None:child.wait(timeout=45)
 log.close()
 if session:
  p=Path(session['session_dir'])/'session_result.json'
  if p.exists():result['session_result']=json.loads(p.read_text())
 (directory/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
