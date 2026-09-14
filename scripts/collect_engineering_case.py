"""Passive ROS evidence: no command publication or parameter writes."""
import collections,gzip,json,threading,time
from pathlib import Path
import rospy,rosbag,roslib.message
from nav_msgs.msg import OccupancyGrid,Odometry,Path as NavPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from std_msgs.msg import String

ws=Path('/home/nvidia/go2_explore_ws')
session=json.loads((ws/'.state/session.json').read_text())
out=ws/'artifacts'/('engineering_'+session['session_id']);out.mkdir(exist_ok=True)
rospy.init_node('engineering_passive_capture',anonymous=True,disable_signals=True)
lock=threading.RLock();latest={};counts=collections.Counter();closed=False
bag=rosbag.Bag(str(out/'evidence.bag'),'w',compression='lz4')
topics=['/livox/lidar','/livox/imu','/lio/odometry','/odom_nav','/odom_robot','/tf','/tf_static',
 '/cloud_registered_base','/cloud_registered_terrain','/terrain/ground_points','/terrain/obstacle_points','/terrain/clearing_points',
 '/nav_static_map','/exploration/observed_local_map','/exploration/coverage_map','/exploration/frontier_map',
 '/move_base/global_costmap/costmap','/move_base/local_costmap/costmap','/move_base/GlobalPlanner/plan',
 '/move_base/TebLocalPlannerROS/local_plan','/move_base/TebLocalPlannerROS/execution_mode',
 '/explore/selected_goal','/explore/selection_status','/explore/view_candidates','/cmd_vel_nav',
 '/exploration/cmd_vel_shaped','/cmd_vel_safe','/exploration/odom_status','/exploration/state',
 '/go2/control/enabled','/go2/diagnostics','/go2_exploration_safety/status','/terrain/status','/rosout_agg']
def callback(m,topic):
    with lock:
        if closed:return
        bag.write(topic,m,rospy.Time.now());counts[topic]+=1;latest[topic]=m
types=dict(rospy.get_published_topics())
subs=[rospy.Subscriber(t,roslib.message.get_message_class(types[t]),callback,callback_args=t,queue_size=20) for t in topics if t in types]
params=rospy.get_param('/')
start=time.monotonic()
while time.monotonic()-start<35:time.sleep(.25)
for sub in subs:sub.unregister()
with lock:
    closed=True;bag.close();report={'session':session,'counts':dict(counts),'params':params,'maps':{},'clouds':{},'paths':{},'strings':{}}
    for topic,m in latest.items():
        if isinstance(m,OccupancyGrid):
            report['maps'][topic]=dict(frame=m.header.frame_id,stamp=m.header.stamp.to_sec(),w=m.info.width,h=m.info.height,res=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y],data=list(m.data))
        elif isinstance(m,PointCloud2):report['clouds'][topic]=dict(frame=m.header.frame_id,stamp=m.header.stamp.to_sec(),xyz=list(read_points(m,field_names=('x','y','z'),skip_nans=True)))
        elif isinstance(m,NavPath):report['paths'][topic]=[[p.pose.position.x,p.pose.position.y,p.pose.orientation.z,p.pose.orientation.w] for p in m.poses]
        elif isinstance(m,String):report['strings'][topic]=m.data
        elif isinstance(m,Odometry) and topic=='/odom_robot':
            p=m.pose.pose.position;q=m.pose.pose.orientation
            report['pose']=dict(x=p.x,y=p.y,z=p.z,q=[q.x,q.y,q.z,q.w],stamp=m.header.stamp.to_sec())
with gzip.open(out/'snapshot.json.gz','wt') as f:json.dump(report,f)
print(json.dumps({'folder':str(out),'counts':dict(counts),'strings':report['strings']},indent=2),flush=True)
