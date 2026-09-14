"""Read one running session; no publishers, services, or parameter writes."""
import collections,gzip,json,math,threading,time
from pathlib import Path
import numpy as np
import rospy,tf2_ros
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String
from go2_exploration_safety.safety_core import DirectionalStopRegion
from go2_exploration_safety.go2_contract import body_envelope

ws=Path('/home/nvidia/go2_explore_ws');session=json.loads((ws/'.state/session.json').read_text())
folder=ws/'artifacts'/('live_'+session['session_id']);folder.mkdir(exist_ok=True)
rospy.init_node('exploration_passive_capture',anonymous=True,disable_signals=True)
buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
lock=threading.RLock();data=collections.defaultdict(list);maps={};latest={};counts=collections.Counter();max_vel={}
log=(folder/'events.jsonl').open('a',buffering=1)
def receive(m,topic):
    if isinstance(m,DiagnosticArray):v=[dict(name=s.name,message=s.message,values={x.key:x.value for x in s.values}) for s in m.status]
    elif isinstance(m,Twist):v=[m.linear.x,m.linear.y,m.angular.z]
    elif isinstance(m,OccupancyGrid):v=dict(frame=m.header.frame_id,stamp=m.header.stamp.to_sec(),w=m.info.width,h=m.info.height,res=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y],data=list(m.data))
    elif isinstance(m,Odometry):
        q=m.pose.pose.orientation;p=m.pose.pose.position;v=dict(x=p.x,y=p.y,z=p.z,q=[q.x,q.y,q.z,q.w],stamp=m.header.stamp.to_sec())
    else:v=m.data
    with lock:
        counts[topic]+=1
        if isinstance(m,OccupancyGrid):maps[topic]=v;return
        if isinstance(m,Twist):
            previous=max_vel.setdefault(topic,[0.,0.,0.,0])
            for i in range(3):previous[i]=max(previous[i],abs(v[i]))
            previous[3]+=int(any(abs(x)>1e-7 for x in v))
        if v!=latest.get(topic):
            row=dict(t=time.time(),topic=topic,value=v)
            if not log.closed:log.write(json.dumps(row)+'\n')
            data[topic].append(v)
            if len(data[topic])>2500:data[topic]=data[topic][-2500:]
        latest[topic]=v
spec={t:DiagnosticArray for t in ('/go2/diagnostics','/terrain/status','/go2_exploration_safety/status')}
spec.update({t:String for t in ('/exploration/state','/exploration/odom_status','/explore/selection_status')})
spec.update({t:Twist for t in ('/cmd_vel_nav','/exploration/cmd_vel_shaped','/cmd_vel_safe')})
spec.update({t:OccupancyGrid for t in ('/nav_static_map','/move_base/global_costmap/costmap','/move_base/local_costmap/costmap')})
spec.update({'/odom_robot':Odometry,'/go2/control/enabled':Bool})
subs=[rospy.Subscriber(t,k,receive,callback_args=t,queue_size=5) for t,k in spec.items()]
def summary():
    with lock:
        gate=latest.get('/go2_exploration_safety/status',[{'message':'waiting','values':{}}])[0]
        sdk=latest.get('/go2/diagnostics',[{'values':{}}])[0]['values']
        return dict(state=latest.get('/exploration/state'),gate=gate['message'],velocities=max_vel,
          sdk={k:sdk.get(k) for k in ('active_motion_mode','classic_sdk_result','classic_feedback_verified','last_nonzero_command_age_sec','commanded_vx_m_s','commanded_wz_rad_s','min_foot_force','last_foot_unload_age_sec','max_joint_dq_rad_s','no_step_response','sport_state_error_code')})
start=time.monotonic();last=0
while time.monotonic()-start<60:
    time.sleep(.25)
    if time.monotonic()-last>10:
        print(json.dumps(summary()),flush=True);last=time.monotonic()
    if not (ws/'.state/session.json').exists():break
with lock:
    report=dict(session=session,counts=dict(counts),max_vel=max_vel,data=dict(data),maps=maps)
    if '/odom_robot' in latest:report['pose']=latest['/odom_robot']
with gzip.open(folder/'capture.json.gz','wt') as stream:json.dump(report,stream)
(folder/'summary.json').write_text(json.dumps(summary(),indent=2))
print('Saved '+str(folder),flush=True)
for sub in subs:sub.unregister()
with lock:log.close()
