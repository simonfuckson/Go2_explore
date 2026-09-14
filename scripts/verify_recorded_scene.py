#!/usr/bin/env python3
"""Compare first motion on a fixed real sensor snapshot; all control is mock."""
import argparse,fcntl,json,os,signal,subprocess,sys,time,uuid
from pathlib import Path
WS=Path(__file__).resolve().parents[1]
from session import LOCK,conflicts
parser=argparse.ArgumentParser();parser.add_argument('snapshot');parser.add_argument('--label',default='scene');parser.add_argument('--seconds',type=float,default=65)
args=parser.parse_args();lock=LOCK.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
if conflicts():raise RuntimeError('Another robot stack is active')
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11322';os.environ['ROS_IP']='127.0.0.1'
out=WS/'artifacts'/('recorded_'+args.label+'_'+uuid.uuid4().hex[:8]);out.mkdir()
os.environ['ROS_LOG_DIR']=str(out/'ros')
launch=out/'fixture.launch'
launch.write_text('''<launch>
<rosparam command="load" file="$(find go2_core)/config/extrinsics.yaml"/>
<rosparam command="load" ns="go2_terrain_guard" file="$(find go2_terrain)/config/terrain_guard_go2.yaml"/>
<param name="/go2_terrain_guard/sensor_height" value="0.51"/>
<include file="$(find go2_mapping)/launch/map_builder.launch"><arg name="map_name" value="fixture"/><arg name="map_root" value="%s"/></include>
<include file="$(find go2_exploration)/launch/go2_exploration.launch"><arg name="start_sensors" value="false"/><arg name="use_real_sdk" value="false"/><arg name="rviz" value="false"/><arg name="map_name" value="fixture"/><arg name="map_root" value="%s"/><arg name="session_dir" value="%s"/></include>
</launch>'''%(out/'maps',out/'maps',out))
import rospy,rosgraph
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray
log=(out/'console.log').open('wb');world=None;subs=[];commands=[];goals=[];modes=[];states=[];last={}
child=subprocess.Popen(['roslaunch','-p','11322',str(launch)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
    start=time.monotonic()
    while not rosgraph.is_master_online():
        if child.poll() is not None or time.monotonic()-start>15:raise RuntimeError('Master unavailable')
        time.sleep(.1)
    rospy.init_node('verify_recorded_scene',anonymous=True,disable_signals=True)
    world=subprocess.Popen([sys.executable,str(WS/'src/go2_exploration/test/recorded_scene_world.py'),args.snapshot],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    subs=[rospy.Subscriber('/cmd_vel_safe',Twist,lambda m:commands.append([m.linear.x,m.linear.y,m.angular.z]),queue_size=100),rospy.Subscriber('/explore/selected_goal',PoseStamped,lambda m:goals.append([m.pose.position.x,m.pose.position.y]),queue_size=10),rospy.Subscriber('/move_base/TebLocalPlannerROS/execution_mode',String,lambda m:modes.append(m.data),queue_size=20),rospy.Subscriber('/exploration/state',String,lambda m:states.append(m.data),queue_size=20),rospy.Subscriber('/go2_exploration_safety/status',DiagnosticArray,lambda m:last.update(gate=[{k.key:k.value for k in s.values} for s in m.status]),queue_size=5),rospy.Subscriber('/explore/selection_status',String,lambda m:last.update(selection=m.data),queue_size=10)]
    begin=time.monotonic()
    while time.monotonic()-begin<args.seconds and child.poll() is None:
        if world.poll() is not None:raise RuntimeError('Fixture publisher exited')
        time.sleep(.2)
    result={'label':args.label,'duration_sec':time.monotonic()-begin,'goal_count':len(goals),'goals':goals,'command_count':len(commands),'nonzero_commands':sum(abs(v)+abs(w)>.001 for v,y,w in commands),'max_forward':max([v for v,y,w in commands] or [0]),'max_yaw':max([abs(w) for v,y,w in commands] or [0]),'safe_limits':all(0<=v<=.3001 and y==0 and abs(w)<=.5001 for v,y,w in commands),'modes':modes,'states':states,**last,'fixture':'Measured stationary points republished with fresh timestamps; no physical motion and no simulated displacement.'}
    rospy.wait_for_service('/exploration/stop',timeout=3);rospy.ServiceProxy('/exploration/stop',Trigger)();child.wait(timeout=45)
    result['stop_result']=json.loads((out/'result.json').read_text())
    (out/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps({'directory':str(out),**result},indent=2),flush=True)
finally:
    for proc in (world,child):
        if proc and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=40)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
    log.close()
