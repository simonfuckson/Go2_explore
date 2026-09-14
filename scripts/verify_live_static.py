#!/usr/bin/env python3
"""Three-minute real perception check. CLI is deliberately used without --real."""
import argparse,json,os,signal,subprocess,time,uuid
from pathlib import Path
WS=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--no-camera',action='store_true')
parser.add_argument('--no-record',action='store_true',help='Keep JSON/status evidence without a duplicate full sensor bag')
options=parser.parse_args()
name='static_engineering_'+uuid.uuid4().hex[:8]
out=WS/'artifacts'/name;out.mkdir()
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11321';os.environ['ROS_IP']='192.168.50.110'
from session import active_session,conflicts
if active_session() or conflicts():raise RuntimeError('A robot stack is active')
import rospy,rosgraph
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import String,Bool
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
log=(out/'launcher.log').open('wb')
child=subprocess.Popen([str(WS/'run_go2_explore'),'explore',name]+([] if options.no_record else ['--record'])+(['--no-camera'] if options.no_camera else []),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
commands=[];goals=[];states=[];odom=[];clearing=[];maps=[];last={};result={'name':name,'real_sdk':False}
result['camera_expected']=not options.no_camera
result['full_bag_recorded']=not options.no_record
result['scope']='Static perception and simulated command availability; excludes physical motion and coverage acceptance'
poses=[];sensors=[];images=[]
result['link_changes_before']=Path('/sys/class/net/eth1/carrier_changes').read_text().strip()
try:
    start=time.monotonic();session=None
    while time.monotonic()-start<30:
        if child.poll() is not None:raise RuntimeError('Session exited during setup')
        session=active_session()
        if session and session['map_name']==name and rosgraph.is_master_online():break
        time.sleep(.1)
    if not session or session['map_name']!=name or session['real_sdk']:raise RuntimeError('Expected own mock session')
    result['session']=session
    rospy.init_node('verify_live_static',anonymous=True,disable_signals=True)
    def capture_gate(m):last['safety']=[dict(message=s.message,values={v.key:v.value for v in s.values}) for s in m.status]
    subs=[rospy.Subscriber('/cmd_vel_safe',Twist,lambda m:commands.append([m.linear.x,m.linear.y,m.angular.z]),queue_size=100),rospy.Subscriber('/explore/selected_goal',PoseStamped,lambda m:goals.append([m.pose.position.x,m.pose.position.y]),queue_size=20),rospy.Subscriber('/exploration/state',String,lambda m:states.append(m.data),queue_size=20),rospy.Subscriber('/exploration/odom_status',String,lambda m:odom.append(m.data),queue_size=50),rospy.Subscriber('/exploration/clearing_status',String,lambda m:clearing.append(m.data),queue_size=50),rospy.Subscriber('/exploration/map_status',String,lambda m:maps.append(m.data),queue_size=50),rospy.Subscriber('/go2_exploration_safety/status',DiagnosticArray,capture_gate,queue_size=5)]
    begin=time.monotonic();previous=-1
    subs.extend([rospy.Subscriber('/odom_nav',Odometry,lambda m:poses.append([m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]),queue_size=100),
                 rospy.Subscriber('/exploration/sensor_status',String,lambda m:sensors.append(m.data),queue_size=10),
                 rospy.Subscriber('/exploration_camera/color/image_raw',Image,lambda m:images.append([m.header.stamp.to_sec(),m.width,m.height]),queue_size=1)])
    while time.monotonic()-begin<185:
        if child.poll() is not None:raise RuntimeError('Session exited before static observation completed')
        elapsed=time.monotonic()-begin
        if int(elapsed//30)!=previous:
            previous=int(elapsed//30)
            print(json.dumps({'seconds':round(elapsed),'state':states[-1:] ,'goals':len(goals),'nonzero_commands':sum(abs(v)+abs(w)>.001 for v,y,w in commands),'odom':odom[-1:],'clearing':clearing[-1:]},ensure_ascii=False),flush=True)
        time.sleep(.25)
    result.update(duration_sec=time.monotonic()-begin,goal_count=len(goals),goals=goals,states=states,
      command_count=len(commands),nonzero_commands=sum(abs(v)+abs(w)>.001 for v,y,w in commands),
      max_forward=max([v for v,y,w in commands] or [0]),max_yaw=max([abs(w) for v,y,w in commands] or [0]),
      safe_limits=all(0<=v<=.3001 and y==0 and abs(w)<=.5001 for v,y,w in commands),
      odom_statuses=sorted(set(odom)),clearing_errors=sorted(set(v for v in clearing if not v.startswith('ready:'))),
      map_updates=sum(v.startswith('ready:') for v in maps),last=last)
    result['passed']=result['nonzero_commands']>=100 and result['max_forward']>=.29 and result['safe_limits'] and 'FAULT_STOPPED' not in states and not result['clearing_errors'] and 'healthy' in odom
    result['sensor_statuses']=sorted(set(sensors));result['image_count']=len(images)
    result['image_first_last']=[images[0],images[-1]] if images else []
    result['stationary_pose_extent_m']=[max(v[i] for v in poses)-min(v[i] for v in poses) for i in range(3)] if poses else []
    result['link_changes_after']=Path('/sys/class/net/eth1/carrier_changes').read_text().strip()
    result['passed']=result['passed'] and any(v.startswith('healthy:') for v in sensors) and not any(v.startswith('FAULT:') for v in sensors) and bool(poses) and max(result['stationary_pose_extent_m'])<.25 and result['link_changes_before']==result['link_changes_after']
finally:
    current=active_session()
    if current and current['map_name']==name and not current['real_sdk']:
        stopping=subprocess.run([str(WS/'run_go2_explore'),'stop'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=100)
        result['stop_output']=stopping.stdout;result['stop_returncode']=stopping.returncode
    if child.poll() is None:
        try:child.wait(timeout=20)
        except subprocess.TimeoutExpired:os.kill(child.pid,signal.SIGINT);child.wait(timeout=45)
    log.close()
    if result.get('session'):
        p=Path(result['session']['session_dir'])/'session_result.json'
        if p.exists():result['session_result']=json.loads(p.read_text())
    result['passed']=bool(result.get('passed') and result.get('stop_returncode')==0 and result.get('session_result',{}).get('success'))
    (out/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps({'directory':str(out),**result},ensure_ascii=False,indent=2),flush=True)
if not result['passed']:raise SystemExit(1)
