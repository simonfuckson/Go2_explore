# GO2 terrain integration

Offline conversion produces occupancy and global slope costs. The runtime local
costmap continues to use ordinary obstacles only, without local slope costs.

## Production offline export, reconstruction revision 2

Use the existing workflow:

```bash
run_go2 mapping new_site
run_go2 save-map
# Stop mapping with Ctrl+C before exporting.
run_go2 export-map new_site
```

The same default profile applies to all new maps; there are no site names,
coordinates or per-map correction masks in the reconstruction algorithm.
Existing saved maps are not re-exported automatically. Metadata without
`reconstruction_revision` continues to validate as revision 1.

The hidden legacy occupancy export supplies exact grid geometry only.
Its absolute-Z black pixels are NOT inherited: a ramp can rise several metres
above the initial floor without becoming a 2D obstacle.

1. Measure the initial floor below the first trajectory XY; require base-to-floor
   height 0.20–0.55 m. Trajectory Z is flattened and is never used as elevation.
2. Approximate PMF supplies conservative distributed low-surface candidates.
   Robust local planes connect and complete bounded sampling gaps up to 35°.
   Retain only a height-continuous surface connected to the measured start floor.
   Distributed seeds alone cannot make a detached roof into floor.
3. Classify actual PCD points at 0.05–1.50 m above that surface as obstacles.
   A staircase cannot be accepted solely because its smoothed plane is shallow:
   coherent measured jumps of at least 0.05 m are retained as obstacle edges.
4. Where a wall lacks a same-cell floor, fit an obstacle-only reference from
   nearby accepted ground: search ≤1.0 m, nearest ground ≤0.8 m, at least eight
   cells, ≥75% inliers, RMSE ≤0.035 m, two-dimensional support, slope ≤35°.
   Require four consecutive 0.10 m measured vertical bins spanning ≥0.30 m
   inside the obstacle-height band. Separated floor and ceiling points do not
   qualify. This reference NEVER generates ground, free cells or terrain cost.
   Unsupported vertical columns remain unknown, rather than being cleared.
5. Construct PGM and all six terrain layers from this one surface. Ground free
   completion is bounded to 0.10 m; walked free evidence uses a 0.18 m radius,
   connects path samples only across gaps ≤0.50 m, and cannot erase obstacles.

The exporter requires at least 100 ground cells; ≥80% of recorded trajectory
samples must have reconstructed ground, ≥95% must lie in actual PGM free cells,
and ≥95% must be reachable from the first sample without crossing an occupied
or unknown cell. This is a grid connectivity check, not a full robot-footprint
motion guarantee. An input with missing floor, contradictory obstacles or bad
checksums fails explicitly; it is never forced to pass by clearing its walls.

## Outputs and transaction

```text
map.pgm                 map.yaml
terrain_2p5d.yaml        terrain_quality.yaml
terrain_elevation.f32   terrain_slope.f32
terrain_roughness.f32   terrain_step.f32
terrain_cost.u8         terrain_confidence.u8
terrain_ground.pcd      terrain_obstacles.pcd
terrain_preview.ppm     terrain_checksums.sha256
```

All arrays share the PGM's 0.05 m resolution, dimensions and origin. Float layers
are row-major little-endian; unknown is NaN or the format's documented sentinel.
Cost uses 0–254, with 255 unknown; confidence uses 0–100, with 255 unknown.
Step records height residual after subtracting the local slope, not ramp rise.
An empty obstacle PCD is valid for a genuinely obstacle-free scene.

Files are written and validated in a temporary directory. Source PCD snapshots
are checked both before and after reconstruction. Commit backs up old assets
with hard links, replaces files individually, and installs metadata last.
Normal I/O errors roll back already replaced files. This is not a single
filesystem transaction: power loss or forced termination during commit can
leave a mixed set. Validation refuses that set; recover the old files from
`.go2_terrain_recovery_*` before navigation. Never delete a recovery directory
until the map has passed validation. Source PCDs are never modified.

```bash
rosrun go2_terrain validate_terrain_map.py --map-dir ~/go2_nav_ws/maps/new_site
cat ~/go2_nav_ws/maps/new_site/terrain_quality.yaml
```

Configuration: `config/terrain_export.yaml`. The exporter records effective
surface settings in metadata. Global slope cost stays zero through 8°, increases
from 15 to 80 over 8–30°, and becomes lethal above 30° for a connected group of
at least four cells, expanded by 0.20 m. Static unknown/lethal cells retain
authority. Local costmap, extrinsics, footprint, TEB and chassis are unchanged.
Map obstacle inflation remains 0.03 m (rounded to one cardinal 0.05 m cell);
global/local costmap inflation remains the existing 0.10 m.

## Regression checks

```bash
catkin_make run_tests_go2_terrain -j2
catkin_test_results build/test_results/go2_terrain
python3 src/go2_terrain/test/offline_export_regression.py \
  --exporter "$PWD/devel/lib/go2_terrain/go2_terrain_exporter_node" \
  --output-root "$PWD/experiments/review_new_dataset" \
  --source-map "$PWD/maps/saved_site"
```

The integration test creates a private localhost ROS master, works on copies,
validates the full asset set, corrupts a copied input to test failure preservation,
and verifies source map hashes. It never starts navigation or a chassis bridge.
See `THIRD_PARTY.md` for the PDF Route 1 source provenance.

## Runtime data flow

```text
/cloud_registered_base (base_link, exact scan time)
  -> go2_terrain_cloud_adapter
/cloud_registered_terrain (terrain_sensor, gravity aligned)
  -> go2_navigation_patchworkpp
/terrain/patchwork_ground + /terrain/patchwork_nonground
  -> go2_terrain_guard
/terrain/obstacle_points + /terrain/clearing_points
  -> move_base/local_costmap/obstacle_layer
```

`terrain_sensor` is dynamically published under `odom`. Its origin is the
calibrated `lidar_link` origin and its orientation preserves robot yaw while
removing roll and pitch. All transforms use the cloud timestamp; there is no
latest-TF fallback.

The Patchwork wrapper and all external cloud publishers/subscribers use queue
depth one. The wrapper replaces upstream `demo`, which uses queue depth 100,
latched publishers, and per-frame logging. High/ceiling returns above 1.50 m
relative to nearby ground are excluded from both marking and clearing.
Nonground returns are marked only when a ground cell exists within 0.45 m.
A return without nearby ground is always unknown, so a Patchwork miss on a
ramp cannot turn that ramp into a hard local obstacle.

Patchwork ground is also treated as a candidate rather than trusted clearing
evidence. The guard selects the low support band in each cell, anchors near
the nominal floor below the MID360, and grows with four-connected cells only
across a spatially connected surface no steeper than 35 degrees. Four-way
growth prevents low steps from bypassing the grade limit through a diagonal.
A ceiling or other disconnected
horizontal surface mistakenly labelled ground is sent to the unknown/debug
class and cannot generate a clearing ray. Rejected candidates from either
Patchwork stream are returned to normal obstacle-height classification unless
they pass the strict continuous-steep-surface test below. This preserves
marking for boxes, table surfaces, legs, walls, and disconnected platform tops
when valid nearby ground exists.

Patchwork non-floor returns are checked once more for a sustained ground-like
profile.
A segment is downgraded to unknown only when it starts at connected ground,
continues monotonically in one of eight metric directions for at least three
cells, stays between 8 and 70 degrees, has consistent first differences, at
least two raw points per cell, and passes a point-level Huber fit of the full
two-dimensional plane `z=ax+by+c`. Only the robust low-support band from each
cell enters this fit, so a same-column ceiling or other high return cannot
invalidate an otherwise continuous ramp; all high returns still pass through
the final unknown/obstacle classifier. The fitted grade must remain within the
same angle range, RMSE must not exceed 0.025 m, and no point residual may
exceed 0.05 m. A thin wall, a flat-topped box, or
a regular stair profile therefore remains marking evidence. A long planar
object leaning at a ground-like angle is geometrically indistinguishable from
a real slope in one LiDAR frame; accepting that ambiguity is an inherent risk
of the selected design in which the local costmap has no slope layer.

Health is published on `/terrain/healthy` and `/terrain/status`. Diagnostics
include `ground_ratio`, `output_rate_hz`, point counts, processing latency, and
high-return counts. Health additionally requires at least 0.60 square metres
of one connected ground component, 0.18 square metres of support within 1.0 m
of the robot, and at least four of eight near-field sectors with two cells
each. A Huber-robust plane `z=ax+by+c` is fitted to the selected near-field
ground component within 1.50 m of the MID360. Diagnostics report height `-c`,
slope, weighted RMSE, sample count, and fit status. At least 12 samples with
non-degenerate planar spread are required, RMSE may not exceed 0.04 m, and the
measured height must remain within 0.43-0.59 m. The local-radius limit prevents
distant terrain or a slope transition from moving the height at the robot.
Sparse, fragmented, remote-only, one-sided, crouched, or abnormally elevated
support therefore cannot arm navigation. Health also remains false until at
least three valid frame-rate
samples have been collected and the EWMA rate is at least 8 Hz. Startup and
low-rate grace periods affect diagnostic severity only; they never bypass the
control gate.

## Dependency

The vendored GPL package owns both the algorithm and its GO2 wrapper:

```text
Repository: https://github.com/url-kaist/patchwork-plusplus-ros
Commit:     f8c070bf2774b2f3ef622644a511bdfe3f2f27bb
Path:       src/third_party/patchworkpp
License:    GPL-3.0
```

Its upstream CMake did not export `include`; this workspace adds
`catkin_package(INCLUDE_DIRS include)` and builds the bounded queue-one wrapper
inside that same GPL package. No Patchwork algorithm source is changed. The
`go2_terrain` runtime nodes communicate with it only through ROS messages and
do not include or link the GPL template implementation. See
`src/third_party/patchworkpp/LOCAL_CHANGES.md`.

## Modes

`go2_bringup/launch/navigation.launch` accepts `terrain_enabled` and
`terrain_metadata`:

- `terrain_enabled:=false` (default): preserves the old local costmap input and
  does not start this runtime path. This is compatible with existing maps.
- `terrain_enabled:=true`: starts this runtime path, requires terrain health,
  and does not start the old `/cloud_registered_costmap` branch. The top-level
  launcher must validate and pass `terrain_metadata` before selecting it.

The robot 1 control interface remains `eth0`. The nominal measured sensor
height is 0.51 m; startup fails outside 0.43-0.59 m. Re-measure it whenever the
standing height or mechanical mounting changes.
