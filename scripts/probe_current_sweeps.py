#!/usr/bin/env python3
import json,math,time
from pathlib import Path
import numpy as np,rospy,tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from go2_exploration_safety.safety_core import DirectionalStopRegion
rospy.init_node('probe_current_sweeps',anonymous=True)
tf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(tf)
topics=['/nav_static_map','/exploration/observed_local_map','/move_base/local_costmap/costmap']
maps={t:rospy.wait_for_message(t,OccupancyGrid,timeout=5) for t in topics}
cloud=rospy.wait_for_message('/terrain/obstacle_points',PointCloud2,timeout=5)
tr=tf.lookup_transform('map','base_link',rospy.Time(0),rospy.Duration(2)).transform
q=tr.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));c,s=math.cos(yaw),math.sin(yaw)
params=rospy.get_param('/go2_exploration_safety');names=['min_forward_clearance','min_reverse_clearance','reaction_time','linear_deceleration','angular_deceleration','max_prediction_horizon','prediction_samples','linear_deadband','angular_deadband']
region=DirectionalStopRegion(front=.35,rear=.35,half_width=.155,footprint_margin=.03,**{k:params[k] for k in names})
report={'pose':[tr.translation.x,tr.translation.y,yaw],'maps':{}}
for topic,g in maps.items():
 v=np.array(g.data);ids=np.flatnonzero((v<0)|(v==100));x=g.info.origin.position.x+(ids%g.info.width+.5)*g.info.resolution;y=g.info.origin.position.y+(ids//g.info.width+.5)*g.info.resolution
 dx,dy=x-tr.translation.x,y-tr.translation.y;bx,by=c*dx+s*dy,-s*dx+c*dy;near=np.hypot(bx,by)<1.2;bx,by,ids=bx[near],by[near],ids[near]
 checks=[]
 for speed,w in [(0,0),(0,-.5),(0,.5),(.3,0),(.3,-.1),(.3,.1),(.15,-.2)]:
  poses=region.prediction_poses(speed,w);hits=np.array([region.contains(x,y,speed,w,poses) for x,y in zip(bx,by)],dtype=bool)
  checks.append(dict(v=speed,w=w,unknown=int(((v[ids]<0)&hits).sum()),obstacle=int(((v[ids]==100)&hits).sum()),hits=sorted([[float(x),float(y),int(z)] for x,y,z in zip(bx[hits],by[hits],v[ids][hits])],key=lambda p:math.hypot(*p[:2]))[:10]))
 report['maps'][topic]=checks
report['obstacles_near_sensor']=[list(p) for p in read_points(cloud,field_names=('x','y','z'),skip_nans=True) if math.hypot(p[0],p[1])<1.]
ws=Path('/home/nvidia/go2_explore_ws');session=json.loads((ws/'.state/session.json').read_text());out=ws/'artifacts'/('engineering_'+session['session_id']);out.mkdir(exist_ok=True)
(out/'sweeps.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
