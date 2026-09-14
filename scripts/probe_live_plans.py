"""Read-only make_plan requests. No action goal or motion publication."""
import json
import math
from pathlib import Path
import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan,GetPlanRequest
from tf.transformations import euler_from_quaternion,quaternion_from_euler

rospy.init_node('probe_exploration_paths',anonymous=True,disable_signals=True)
buf=tf2_ros.Buffer();lis=tf2_ros.TransformListener(buf)
grid=rospy.wait_for_message('/move_base/global_costmap/costmap',OccupancyGrid,timeout=4)
local=rospy.wait_for_message('/move_base/local_costmap/costmap',OccupancyGrid,timeout=4)
t=buf.lookup_transform('map','base_link',rospy.Time(0),rospy.Duration(4)).transform
q=t.rotation;yaw=euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
def check(g,x,y,a,front=.38):
    bx,by=np.meshgrid(np.arange(-.38,front+.0125,.025),np.arange(-.185,.1975,.025))
    wx=x+np.cos(a)*bx-np.sin(a)*by;wy=y+np.sin(a)*bx+np.cos(a)*by
    ix=np.floor((wx-g.info.origin.position.x)/g.info.resolution).astype(int)
    iy=np.floor((wy-g.info.origin.position.y)/g.info.resolution).astype(int)
    inside=(ix>=0)&(iy>=0)&(ix<g.info.width)&(iy<g.info.height)
    cells=np.array(g.data).reshape(g.info.height,g.info.width);cost=np.full(ix.shape,-1);cost[inside]=cells[iy[inside],ix[inside]]
    bad=(cost==-1)|(cost==100)
    return dict(unknown=int((cost==-1).sum()),obstacle=int((cost==100).sum()),
        bad_points=np.column_stack([wx[bad],wy[bad],cost[bad]]).tolist()[:8])
result={'pose':[t.translation.x,t.translation.y,yaw], 'current_global':check(grid,t.translation.x,t.translation.y,yaw),'paths':[]}
service=rospy.ServiceProxy('/move_base/make_plan',GetPlan)
for dx,dy in ((.5,0),(1,0),(1.5,0),(.8,.3),(.8,-.3),(1.,.8),(1.,-.8)):
    req=GetPlanRequest();req.start.header.frame_id=req.goal.header.frame_id='map'
    req.start.pose.position.x=t.translation.x;req.start.pose.position.y=t.translation.y;req.start.pose.orientation=q
    req.goal.pose.position.x=t.translation.x+dx;req.goal.pose.position.y=t.translation.y+dy;req.goal.pose.orientation.w=1
    response=service(req);poses=response.plan.poses
    path=[[p.pose.position.x,p.pose.position.y] for p in poses]
    report=dict(goal=[dx,dy],poses=path)
    heading=yaw
    for i,p in enumerate(path):
        nxt=path[min(i+1,len(path)-1)]
        if math.hypot(nxt[0]-p[0],nxt[1]-p[1])>1e-6:heading=math.atan2(nxt[1]-p[1],nxt[0]-p[0])
        c=check(grid,*p,heading)
        if c['unknown'] or c['obstacle']:
            report['first_blocked']=dict(index=i,position=p,heading=heading,**c);break
    anchored=[[t.translation.x,t.translation.y]]+[p for p in path if math.hypot(p[0]-t.translation.x,p[1]-t.translation.y)>grid.info.resolution*math.sqrt(2)]
    heading=yaw;previous=anchored[0];previous_heading=yaw
    for i,p in enumerate(anchored):
        nxt=anchored[min(i+1,len(anchored)-1)]
        if i and math.hypot(nxt[0]-p[0],nxt[1]-p[1])>1e-6:heading=math.atan2(nxt[1]-p[1],nxt[0]-p[0])
        turn=math.atan2(math.sin(heading-previous_heading),math.cos(heading-previous_heading))
        steps=max(1,int(math.ceil(math.hypot(p[0]-previous[0],p[1]-previous[1])/.025)),int(math.ceil(abs(turn)/.05)))
        failed=None
        for step in range(steps+1):
            a=previous_heading+turn*step/steps
            x=previous[0]+(p[0]-previous[0])*step/steps;y=previous[1]+(p[1]-previous[1])*step/steps
            c=check(grid,x,y,a)
            if c['unknown'] or c['obstacle']:
                failed=dict(index=i,position=[x,y],heading=a,**c);break
        if failed:
            report['anchored_blocked']=failed;break
        previous=p;previous_heading=heading
    else: report['anchored_pass']=True
    result['paths'].append(report)
Path('/home/nvidia/go2_explore_ws/artifacts/path_probe_20260914.json').write_text(json.dumps(result,indent=2))
for p in result['paths']:p['poses_count']=len(p['poses']);p['poses']=p['poses'][:5]
print(json.dumps(result,indent=2))
