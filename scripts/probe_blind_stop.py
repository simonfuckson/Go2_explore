#!/usr/bin/env python3
"""Read current braking sweeps and map cells; never publish or call services."""
import collections,json,math,time
from pathlib import Path
import numpy as np,rospy,tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from diagnostic_msgs.msg import DiagnosticArray
from go2_exploration_safety.safety_core import DirectionalStopRegion

ws=Path('/home/nvidia/go2_explore_ws')
session=json.loads((ws/'.state/session.json').read_text())
folder=ws/'artifacts'/('live_'+session['session_id']);folder.mkdir(exist_ok=True)
rospy.init_node('passive_blind_stop_probe',anonymous=True,disable_signals=True)
tf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(tf)
p=rospy.get_param('/go2_exploration_safety')
fp=rospy.get_param('/move_base/local_costmap/footprint')
padding=rospy.get_param('/move_base/local_costmap/footprint_padding')
names=['min_forward_clearance','min_reverse_clearance','reaction_time','linear_deceleration','angular_deceleration','max_prediction_horizon','prediction_samples','linear_deadband','angular_deadband']
region=DirectionalStopRegion(front=max(x for x,y in fp),rear=-min(x for x,y in fp),half_width=max(abs(y) for x,y in fp),footprint_margin=padding,**{k:p[k] for k in names})
rows=[]
for iteration in range(10):
 diag=rospy.wait_for_message('/go2_exploration_safety/status',DiagnosticArray,timeout=3)
 d={v.key:v.value for v in diag.status[0].values}
 g=rospy.wait_for_message('/move_base/local_costmap/costmap',OccupancyGrid,timeout=3)
 cloud=rospy.wait_for_message('/terrain/obstacle_points',PointCloud2,timeout=3)
 tr=tf.lookup_transform(g.header.frame_id,'base_link',rospy.Time(0),rospy.Duration(2)).transform
 q=tr.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));c,s=math.cos(yaw),math.sin(yaw)
 q=g.info.origin.orientation;gy=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));gc,gs=math.cos(gy),math.sin(gy)
 a=np.asarray(g.data);ids=np.flatnonzero((a<0)|(a>=p['costmap_lethal_threshold']))
 x=(ids%g.info.width+.5)*g.info.resolution;y=(ids//g.info.width+.5)*g.info.resolution
 dx=g.info.origin.position.x+gc*x-gs*y-tr.translation.x;dy=g.info.origin.position.y+gs*x+gc*y-tr.translation.y
 bx,by=c*dx+s*dy,-s*dx+c*dy;near=np.hypot(bx,by)<1.5;bx,by,ids=bx[near],by[near],ids[near]
 checks=[]
 trip=(float(d['trip_linear_x']),float(d['trip_angular_z']))
 for v,w in [trip,(.3,0),(.3,-.1),(.3,.1),(0,-.5),(0,.5)]:
  poses=region.prediction_poses(v,w)
  mask=np.asarray([region.contains(x,y,v,w,poses) for x,y in zip(bx,by)],dtype=bool)
  checks.append(dict(v=v,w=w,unknown=int(((a[ids]<0)&mask).sum()),obstacle=int(((a[ids]>=p['costmap_lethal_threshold'])&mask).sum()),hits=sorted([[float(x),float(y),int(z)] for x,y,z in zip(bx[mask],by[mask],a[ids][mask])],key=lambda r:math.hypot(*r[:2]))[:12]))
 row=dict(stamp=rospy.Time.now().to_sec(),pose=[tr.translation.x,tr.translation.y,yaw],gate=d,checks=checks,terrain_frame=cloud.header.frame_id,near_terrain_obstacles=[list(pt) for pt in read_points(cloud,field_names=('x','y','z'),skip_nans=True) if math.hypot(pt[0],pt[1])<1.])
 rows.append(row)
 print(json.dumps(dict(sample=iteration,reason=d['reason'],checks=[{k:v for k,v in ch.items() if k!='hits'} for ch in checks],near_obstacles=len(row['near_terrain_obstacles']))),flush=True)
 time.sleep(.4)
out={'session':session,'effective_blind':rospy.get_param('/preprocess/blind'),'params':p,'footprint':fp,'padding':padding,'samples':rows}
(folder/'blind_stop_probe.json').write_text(json.dumps(out,indent=2))
print('Saved '+str(folder/'blind_stop_probe.json'),flush=True)
