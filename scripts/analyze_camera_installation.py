import gzip,json,math
from pathlib import Path
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1];out=root/'artifacts/camera_install_4be378d3'
with gzip.open(out/'capture.json.gz','rt') as f:d=json.load(f)
def mat(t):
 x,y,z,w=t['xyzw'];r=np.eye(4)
 r[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
 r[:3,3]=t['xyz'];return r
edges={}
for b,t in d['tf'].items():
 a=t['parent'];m=mat(t);edges.setdefault(a,[]).append((b,m));edges.setdefault(b,[]).append((a,np.linalg.inv(m)))
def transform(frame):
 queue=[('base_link',np.eye(4))];seen=set()
 while queue:
  name,m=queue.pop(0)
  if name==frame:return m
  if name in seen:continue
  seen.add(name)
  for child,v in edges.get(name,[]):queue.append((child,m@v))
 raise RuntimeError(frame)
def cloud(topic):
 points=[];seen=Counter();samples=0
 for frame in d['frames'][topic]:
  p=np.array(frame['points'])
  if p.size==0:continue
  m=transform(frame['frame']);xyz=p[:,:3]@m[:3,:3].T+m[:3,3]
  xyz=xyz[(xyz[:,0]>-.3)&(xyz[:,0]<1.1)&(np.abs(xyz[:,1])<.6)&(xyz[:,2]>-.5)&(xyz[:,2]<.6)]
  if len(xyz):points.append(xyz);samples+=1
  for key in set(map(tuple,np.floor(xyz/.025).astype(int))):seen[key]+=1
 return np.vstack(points),seen,samples
raw,_,_=cloud('/cloud_registered_base');obs,bins,frames=cloud('/terrain/obstacle_points')
report={'obstacle_frames_with_near_points':frames,'total_obstacle_frames':len(d['frames']['/terrain/obstacle_points']),
 'common_obstacle_cells_base':[{'center':[(k+.5)*.025 for k in key],'frames':n} for key,n in bins.most_common(30)]}
(out/'point_analysis.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
fig,axes=plt.subplots(1,3,figsize=(15,5),constrained_layout=True)
for ax,ix,iy,title in zip(axes,[0,0,1],[1,2,2],['Top view (body X / Y)','Side view (body X / Z)','Front view (body Y / Z)']):
 ax.scatter(raw[::3,ix],raw[::3,iy],s=.8,c='#6a7789',alpha=.10,label='Measured cloud')
 ax.scatter(obs[:,ix],obs[:,iy],s=12,c='#e44f47',alpha=.6,label='Terrain obstacle returns')
 ax.scatter([0],[0],s=60,c='#13a5b1',marker='+',label='Body origin')
 old=np.array([.35,0,.10]);ax.scatter([old[ix]],[old[iy]],s=55,c='#8460df',marker='x',label='Old camera TF (unverified new mount)')
 ax.set_title(title);ax.set_xlabel(['X (m), forward','Y (m), left'][ix==1]);ax.set_ylabel('Y (m), left' if iy==1 else 'Z (m), up')
 ax.set_aspect('equal');ax.grid(alpha=.2)
 axes[0].set_xlim(-.4,1.1);axes[0].set_ylim(-.6,.6)
axes[1].set_xlim(-.4,1.1);axes[1].set_ylim(-.5,.6);axes[2].set_xlim(-.6,.6);axes[2].set_ylim(-.5,.6)
axes[0].plot([-.35,.35,.35,-.35,-.35],[-.155,-.155,.155,.155,-.155],c='#13a5b1',lw=1.5,label='Original navigation footprint')
axes[1].legend(loc='upper left',fontsize=7)
fig.suptitle('New camera installation: static measured points, no self mask enabled',fontsize=13)
fig.savefig(out/'mount_points.png',dpi=150)
