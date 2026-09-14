#!/usr/bin/env python3
"""Recorded unknown cells through real ROS/shaper/gate and mock SDK only."""
import fcntl,json,os,signal,subprocess,time,uuid
from pathlib import Path
WS=Path(__file__).resolve().parents[1]
from session import LOCK,active_session,conflicts
lock=LOCK.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert not active_session() and not conflicts(),'Another stack is active'
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11324';os.environ['ROS_IP']='127.0.0.1'
out=WS/'artifacts'/('stationary_replan_'+uuid.uuid4().hex[:8]);out.mkdir()
os.environ['ROS_LOG_DIR']=str(out/'ros')
launch=out/'fixture.launch'
launch.write_text('''<launch>
<param name="/exploration/runtime_mode" value="simulation"/>
<rosparam param="/move_base/global_costmap/footprint">[[0.35,0.155],[0.35,-0.155],[-0.35,-0.155],[-0.35,0.155]]</rosparam>
<param name="/move_base/global_costmap/footprint_padding" value="0.03"/>
<include file="$(find go2_exploration)/launch/include/control.launch.xml">
<arg name="use_real_sdk" value="false"/><arg name="require_terrain_health" value="true"/>
</include>
<include file="$(find go2_exploration_safety)/launch/include/cmd_vel_safety.launch.xml">
<arg name="allow_stationary_command_wait" value="true"/><arg name="recover_obstacle_stop" value="true"/>
</include></launch>''')
log=(out/'console.log').open('wb')
child=subprocess.Popen(['roslaunch','-p','11324',str(launch)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
result={'directory':str(out),'real_sdk':False,'fixture':'Recorded map and fixed pose; synthetic planning commands, real shaper/gate, mock SDK'}
import rospy,rosgraph,rosnode,tf2_ros
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist,PoseStamped,TransformStamped
from nav_msgs.msg import OccupancyGrid,Path as NavPath
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool,String,Header
from std_srvs.srv import Trigger,SetBool
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatus,GoalStatusArray
samples=[];gate={};timeline=[];pubs={};enable=None
try:
 deadline=time.monotonic()+15
 while not rosgraph.is_master_online():
  assert child.poll() is None and time.monotonic()<deadline,'Master startup failed'
  time.sleep(.05)
 rospy.init_node('move_base',disable_signals=True)
 def diag(m):
  if m.status:
   gate.clear();gate.update({v.key:v.value for v in m.status[0].values})
   timeline.append((time.monotonic(),dict(gate)))
 rospy.Subscriber('/go2_exploration_safety/status',DiagnosticArray,diag,queue_size=5)
 rospy.Subscriber('/cmd_vel_safe',Twist,lambda m:samples.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),queue_size=100)
 f=json.loads((WS/'src/go2_exploration_safety/test/fixtures/persistent_side_unknown.json').read_text())
 grid=OccupancyGrid();grid.header.frame_id=f['frame'];grid.info.width=f['width'];grid.info.height=f['height'];grid.info.resolution=f['resolution']
 grid.info.origin.position.x,grid.info.origin.position.y=f['origin'];grid.info.origin.orientation.w=1;grid.data=f['data']
 transform=TransformStamped();transform.header.frame_id=f['frame'];transform.child_frame_id='base_link'
 t=transform.transform;t.translation.x=f['pose']['x'];t.translation.y=f['pose']['y'];t.translation.z=f['pose']['z']
 t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w=f['pose']['q']
 tf=tf2_ros.TransformBroadcaster()
 clouds={'/terrain/obstacle_points':create_cloud_xyz32(Header(frame_id='base_link'),[]),
         '/cloud_registered_terrain':create_cloud_xyz32(Header(frame_id='base_link'),[(.5+i*.01,.5,-.15) for i in range(20)])}
 types={'/cmd_vel_nav':Twist,'/move_base/goal':MoveBaseActionGoal,'/move_base/status':GoalStatusArray,
        '/move_base/TebLocalPlannerROS/local_plan':NavPath,'/move_base/local_costmap/costmap':OccupancyGrid,
        '/terrain/healthy':Bool,'/exploration/odom_ok':Bool,'/exploration/map_status':String}
 types.update({topic:type(m) for topic,m in clouds.items()})
 pubs={topic:rospy.Publisher(topic,kind,queue_size=1) for topic,kind in types.items()}
 command=Twist();goal=None;active=False;generation=0
 def pump():
  global generation
  assert child.poll() is None,'Fixture launch exited'
  stamp=rospy.Time.now();transform.header.stamp=stamp;tf.sendTransform(transform)
  grid.header.stamp=stamp;pubs['/move_base/local_costmap/costmap'].publish(grid)
  for topic,m in clouds.items():m.header.stamp=stamp;pubs[topic].publish(m)
  for topic in ('/terrain/healthy','/exploration/odom_ok'):pubs[topic].publish(Bool(data=True))
  generation+=1;pubs['/exploration/map_status'].publish(String(data='ready: fixture='+str(generation)))
  status=[] if goal is None else [GoalStatus(goal_id=goal.goal_id,status=GoalStatus.ACTIVE if active else GoalStatus.PREEMPTED)]
  pubs['/move_base/status'].publish(GoalStatusArray(status_list=status))
  pubs['/cmd_vel_nav'].publish(command)
  if active:
   path=NavPath();path.header.stamp=stamp;path.header.frame_id=f['frame'];path.poses=[PoseStamped(),PoseStamped()]
   pubs['/move_base/TebLocalPlannerROS/local_plan'].publish(path)
  time.sleep(.05)
 def until(predicate,timeout):
  deadline=time.monotonic()+timeout
  while not predicate():
   assert time.monotonic()<deadline,dict(timeout=timeout,gate=gate)
   pump()
 def dwell(seconds):
  end=time.monotonic()+seconds
  until(lambda:time.monotonic()>=end,seconds+1)
 def new_goal(name,w):
  global goal,active,command
  goal=MoveBaseActionGoal();goal.goal_id.id=name;goal.goal_id.stamp=rospy.Time.now()
  goal.goal.target_pose.header.frame_id=f['frame'];goal.goal.target_pose.pose.position.x=1.
  goal.goal.target_pose.pose.orientation.w=1;active=True
  pubs['/move_base/goal'].publish(goal)
  command=Twist();command.linear.x=.3;command.angular.z=w
 until(lambda:gate.get('health_ok')=='True',10)
 names=rosnode.get_node_names()
 assert '/go2_sdk_bridge_mock' in names and '/go2_sdk_bridge_real' not in names,names
 assert rospy.get_param('/exploration/runtime_mode')=='simulation'
 enable=rospy.ServiceProxy('/go2_sdk_bridge_mock/enable',SetBool);assert enable(True).success
 until(lambda:gate.get('bridge_enabled')=='True',3)
 assert rospy.ServiceProxy('/go2_exploration_safety/arm',Trigger)().success
 new_goal('blocked_recorded_right_turn',-.3)
 until(lambda:gate.get('obstacle_hold')=='True',8)
 stopped_at=time.monotonic();active=False;command=Twist()
 result['interrupted_command']=[gate.get('trip_linear_x'),gate.get('trip_angular_z')]
 until(lambda:gate.get('state')=='OBSTACLE_REPLAN_READY',12)
 ready_at=time.monotonic();result['stop_to_replan_seconds']=ready_at-stopped_at
 assert all(abs(v)+abs(y)+abs(w)<1e-8 for now,v,y,w in samples if now>stopped_at+.2)
 assert list(grid.data)==f['data'],'Recorded unknown cells changed'
 assert rospy.ServiceProxy('/go2_exploration_safety/resume',Trigger)().success
 resume_at=time.monotonic();dwell(.3)
 assert all(abs(v)+abs(y)+abs(w)<1e-8 for now,v,y,w in samples if now>resume_at)
 new_goal('new_straight_goal',0.)
 until(lambda:any(now>resume_at+.3 and v>.04 for now,v,y,w in samples),5)
 result['new_straight_goal_moves_mock']=True
 stop_at=time.monotonic();assert rospy.ServiceProxy('/go2_exploration_safety/stop',Trigger)().success
 active=False;command=Twist();dwell(3.)
 assert gate.get('latched_reason')=='operator_stop'
 assert not rospy.ServiceProxy('/go2_exploration_safety/resume',Trigger)().success
 assert all(abs(v)+abs(y)+abs(w)<1e-8 for now,v,y,w in samples if now>stop_at+.2)
 assert all(0<=v<=.3001 and abs(y)<1e-8 and abs(w)<=.5001 for now,v,y,w in samples)
 result.update(passed=True,persistent_unknown_preserved=True,zero_until_new_goal=True,
               operator_stop_no_rearm=True,command_count=len(samples),states=list(dict.fromkeys(g.get('state') for _,g in timeline)))
except Exception as e:
 result.update(passed=False,error=repr(e),last_gate=dict(gate))
finally:
 if enable:
  try:enable(False)
  except Exception:pass
 if child.poll() is None:
  os.killpg(child.pid,signal.SIGINT)
  try:child.wait(timeout=30)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
 log.close();(out/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
raise SystemExit(0 if result.get('passed') else 1)
