#!/usr/bin/env python3
"""Generate map-independent full-export fixtures, including an empty obstacle set."""
import argparse
import hashlib
import math
from pathlib import Path
import struct


def write_pcd(path, points):
    points = list(points)
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
              'COUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              'POINTS %d\nDATA binary\n') % (len(points), len(points))
    with path.open('wb') as stream:
        stream.write(header.encode())
        for point in points:
            stream.write(struct.pack('<fff', *point))


def make(root):
    width, height, resolution = 220, 80, .05
    for name in ('flat_no_obstacles', 'up_down_ramp'):
        directory = root/name
        directory.mkdir(parents=True, exist_ok=False)
        def ground(x):
            return -.35 if name == 'flat_no_obstacles' else \
                -.35 + math.tan(math.radians(20))*max(0, min(x-1, 7-x, 3))
        points = []
        for y in range(4, height-4):
            for x in range(4, width-4):
                wx, wy = -.5+(x+.5)*resolution, -2+(y+.5)*resolution
                points.append((wx, wy, ground(wx)))
                if name == 'up_down_ramp':
                    points.append((wx, wy, 2.55))
        if name == 'up_down_ramp':
            for x in range(4, width-4):
                wx = -.5+(x+.5)*resolution
                for wy in (-1.525, 1.525):
                    for k in range(16):
                        points.append((wx, wy, ground(wx)+.10+k*.08))
        write_pcd(directory/'public_map.pcd', points)
        write_pcd(directory/'traversed_path_map.pcd', ((i*.05, 0, 0) for i in range(181)))
        (directory/'map.pgm').write_bytes(('P5\n%d %d\n255\n' % (width, height)).encode() + bytes([205])*width*height)
        (directory/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [-0.5, -2.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        names = ('public_map.pcd', 'traversed_path_map.pcd')
        (directory/'mapping_snapshot.sha256').write_text(''.join(
            hashlib.sha256((directory/n).read_bytes()).hexdigest()+'  '+n+'\n' for n in names))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, required=True)
    make(parser.parse_args().output_root)
