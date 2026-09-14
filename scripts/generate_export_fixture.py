#!/usr/bin/env python3
"""Dense synthetic room for export regression ONLY, never a real robot map."""
import hashlib
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parents[1]/'maps/virtual_dense_export_20260913'
root.mkdir(exist_ok=False)
xx,yy=np.meshgrid(np.arange(-3,5.001,.04),np.arange(-3,3.001,.04))
floor=np.column_stack((xx.ravel(),yy.ravel(),np.full(xx.size,-.35),np.ones(xx.size)))
walls=[]
for z in np.arange(-.35,1.21,.04):
    for x in np.arange(-3,5.001,.04):walls.extend(((x,-3,z,1),(x,3,z,1)))
    for y in np.arange(-3,3.001,.04):walls.extend(((-3,y,z,1),(5,y,z,1)))
path=[(0,0,0,0)]
for y in np.arange(-2.4,2.401,.3):
    for x in np.arange(-2.4,4.401,.1):path.append((x,y,0,len(path)*.1))
def write_pcd(name,points):
    header='# Synthetic test fixture, not sensor data\nVERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n'%(len(points),len(points))
    with (root/name).open('w') as out:
        out.write(header);np.savetxt(out,points,fmt='%.5f')
write_pcd('public_map.pcd',np.vstack((floor,np.asarray(walls))))
write_pcd('traversed_path_map.pcd',np.asarray(path))
names=('public_map.pcd','traversed_path_map.pcd')
(root/'mapping_snapshot.sha256').write_text(''.join(hashlib.sha256((root/n).read_bytes()).hexdigest()+'  '+n+'\n' for n in names))
(root/'SYNTHETIC_TEST_ONLY.txt').write_text('Artificial room for exporter regression. Not recorded on GO2; never use it for real localization.\n')
print(root)
