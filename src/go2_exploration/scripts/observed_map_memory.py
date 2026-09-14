#!/usr/bin/env python3
"""Remember observed local occupancy. No PCD, terrain fitting or map-file IO."""
import math
import time
import threading
import numpy as np
from go2_exploration_safety.go2_contract import body_envelope


class OccupancyMemory:
    """Latest observed local state wins; absence of observation never clears."""
    def __init__(self, resolution=.05, initial_size_m=12., max_cells=4000000):
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('invalid resolution')
        self.resolution = resolution
        self.max_cells = max_cells
        side = int(math.ceil(initial_size_m/resolution))
        if side <= 0 or side*side > max_cells:
            raise ValueError('initial map exceeds cell budget')
        self.x0 = self.y0 = -(side//2)
        self.cells = np.full((side, side), -1, dtype=np.int8)
        self.last_seen = np.zeros((side, side), dtype=np.float64)
        self.generation = 0

    def integrate(self, x, y, values, observed_at=0.):
        if not math.isfinite(observed_at) or observed_at < 0:
            raise ValueError('invalid evidence timestamp')
        x, y = np.asarray(x), np.asarray(y)
        values = np.asarray(values)
        if x.shape != y.shape or x.shape != values.shape:
            raise ValueError('malformed observation')
        if not np.isfinite(x).all() or not np.isfinite(y).all():
            raise ValueError('nonfinite coordinates')
        if not np.isin(values, [-1, 0, 100]).all():
            raise ValueError('expected uninflated ternary occupancy')
        known = values != -1
        if not known.any():
            return False
        ix = np.floor(x[known]/self.resolution).astype(np.int64)
        iy = np.floor(y[known]/self.resolution).astype(np.int64)
        values = values[known]
        h, w = self.cells.shape
        # Expand in two-metre blocks, preserving the world grid alignment.
        block = max(1, int(math.ceil(2./self.resolution)))
        nx = min(self.x0, (int(ix.min())//block)*block)
        ny = min(self.y0, (int(iy.min())//block)*block)
        right = max(self.x0+w, ((int(ix.max())//block)+1)*block)
        top = max(self.y0+h, ((int(iy.max())//block)+1)*block)
        if (right-nx)*(top-ny) > self.max_cells:
            raise ValueError('exploration memory cell budget exceeded')
        if (nx, ny, right, top) != (self.x0, self.y0, self.x0+w, self.y0+h):
            expanded = np.full((top-ny, right-nx), -1, dtype=np.int8)
            expanded[self.y0-ny:self.y0-ny+h, self.x0-nx:self.x0-nx+w] = self.cells
            seen = np.zeros(expanded.shape, dtype=np.float64)
            seen[self.y0-ny:self.y0-ny+h, self.x0-nx:self.x0-nx+w] = self.last_seen
            self.last_seen = seen
            self.cells, self.x0, self.y0 = expanded, nx, ny
        # Obstacle wins if rotated source cells land on the same world cell.
        for value in (0, 100):
            mask = values == value
            self.cells[iy[mask]-self.y0, ix[mask]-self.x0] = value
            self.last_seen[iy[mask]-self.y0, ix[mask]-self.x0] = observed_at
        self.generation += 1
        return True


def ground_support(points, resolution=.05, grid_resolution=.15, grid_origin=(-5., -4.), min_range=.35, max_range=5.):
    """Measured safe-ground patches in terrain_sensor; never a free ray/sector."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    distance = np.linalg.norm(points[:, :2], axis=1)
    points = points[(points[:, 0] >= 0) & (distance >= min_range) & (distance <= max_range)]
    if not len(points):
        return points
    # Match Terrain Guard's 0.15 m support grid, with origin (-5,-4).
    keys = np.floor((points[:, :2]-grid_origin)/grid_resolution).astype(np.int64)
    keys, indices = np.unique(keys, axis=0, return_index=True)
    step = resolution*.5
    ox, oy = np.meshgrid(np.arange(step*.5, grid_resolution, step), np.arange(step*.5, grid_resolution, step))
    offsets = np.column_stack((ox.ravel(), oy.ravel()))
    xy = (keys[:, None, :]*grid_resolution + grid_origin + offsets[None, :, :]).reshape(-1, 2)
    output = np.column_stack((xy, np.repeat(points[indices, 2], len(offsets))))
    return output[output[:, 0] >= 0]


def transform_points(points, transform):
    """Apply a 3D TF without flattening pitch/roll on ramps."""
    from tf.transformations import quaternion_matrix
    q, t = transform.rotation, transform.translation
    matrix = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    return points.dot(matrix.T) + [t.x, t.y, t.z]


def cloud_xyz(cloud):
    fields = {field.name: field for field in cloud.fields}
    if any(name not in fields or fields[name].datatype != 7 for name in ('x','y','z')):
        raise ValueError('ground cloud requires FLOAT32 xyz')
    dtype = np.dtype({'names': ['x','y','z'], 'formats': [('>' if cloud.is_bigendian else '<')+'f4']*3,
                      'offsets': [fields[n].offset for n in ('x','y','z')], 'itemsize': cloud.point_step})
    array = np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                       strides=(cloud.row_step, cloud.point_step)).ravel()
    return np.column_stack([array[n] for n in ('x','y','z')])


class MemoryNode:
    def __init__(self):
        import rospy
        import tf2_ros
        from nav_msgs.msg import OccupancyGrid, MapMetaData
        from std_msgs.msg import String
        from sensor_msgs.msg import PointCloud2
        self.ros, self.Grid, self.String = rospy, OccupancyGrid, String
        self.frame = rospy.get_param('~global_frame', 'map')
        self.front, self.rear, self.half_width = body_envelope(rospy.get_param('/move_base/global_costmap/footprint'))
        self.support_grid = rospy.get_param('/go2_terrain_guard/grid')
        self.support_range = rospy.get_param('/go2_terrain_guard/vehicle')
        self.memory = OccupancyMemory(
            float(rospy.get_param('~resolution', .05)),
            float(rospy.get_param('~initial_size_m', 12.)),
            int(rospy.get_param('~max_cells', 4000000)))
        self.coverage = OccupancyMemory(self.memory.resolution,
            float(rospy.get_param('~initial_size_m', 12.)), self.memory.max_cells)
        self.ground = None
        self.ground_lock = threading.Lock()
        self.last_ground_stamp = rospy.Time(0)
        self.coverage_stamp = rospy.Time(0)
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf)
        self.last_stamp = rospy.Time(0)
        self.started = rospy.Time.now()
        self.map_pub = rospy.Publisher('/nav_static_map', OccupancyGrid, queue_size=1, latch=True)
        self.metadata_pub = rospy.Publisher('/nav_static_map_metadata', MapMetaData, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/exploration/map_status', String, queue_size=1, latch=True)
        self.coverage_pub = rospy.Publisher('/exploration/coverage_map', OccupancyGrid, queue_size=1, latch=True)
        self.coverage_age_pub = rospy.Publisher('/exploration/coverage_age', OccupancyGrid, queue_size=1, latch=True)
        # Bootstrap only: a fully unknown map lets move_base construct its local
        # costmap. It does not count as evidence or permit supervisor startup.
        self.publish(self.started)
        self.status_pub.publish(String(data='waiting: no local observation'))
        self.ground_sub = rospy.Subscriber('/terrain/ground_points', PointCloud2,
                                           self.ground_callback, queue_size=1)
        self.sub = rospy.Subscriber('/exploration/observed_local_map', OccupancyGrid,
                                   self.receive, queue_size=1)

    def ground_callback(self, cloud):
        with self.ground_lock:
            self.ground = cloud

    def update_coverage(self, stamp):
        with self.ground_lock:
            ground = self.ground
        if ground is None or not -.1 <= (self.ros.Time.now()-ground.header.stamp).to_sec() <= .6:
            raise ValueError('ground coverage input missing or stale')
        world = None
        if ground.header.stamp > self.last_ground_stamp:
            points = cloud_xyz(ground)
            if ground.header.frame_id != 'terrain_sensor':
                t = self.tf.lookup_transform('terrain_sensor', ground.header.frame_id,
                        ground.header.stamp, self.ros.Duration(.1)).transform
                points = transform_points(points, t)
            points = ground_support(points, self.coverage.resolution,
                self.support_grid['resolution'], (self.support_grid['min_x'], self.support_grid['min_y']),
                self.support_range['min_horizontal_range'], self.support_range['max_horizontal_range'])
            t = self.tf.lookup_transform(self.frame, 'terrain_sensor',
                    ground.header.stamp, self.ros.Duration(.1)).transform
            world = transform_points(points, t)
        # The local grid's publication time is not a measured base pose time.
        # Record the latest actual, fresh pose instead of extrapolating the base
        # to that slightly newer stamp. Ground points above retain exact-time TF.
        pose = self.tf.lookup_transform(self.frame, 'base_link', self.ros.Time(0),
                                        self.ros.Duration(.05))
        pose_stamp = pose.header.stamp
        pose_age = (self.ros.Time.now()-pose_stamp).to_sec()
        if pose_stamp.to_sec() <= 0 or not -.1 <= pose_age <= .5:
            raise ValueError('traversed footprint TF missing timestamp or stale')
        step = self.coverage.resolution*.5
        x, y = np.meshgrid(np.arange(-self.rear, self.front+step*.1, step), np.arange(-self.half_width, self.half_width+step*.1, step))
        footprint = transform_points(np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size))), pose.transform)
        # Resolve all transforms before accepting this observation, so a failed
        # TF lookup cannot mark a map generation as consumed without publication.
        if world is not None:
            self.coverage.integrate(world[:, 0], world[:, 1], np.zeros(len(world), dtype=np.int8),
                                    ground.header.stamp.to_sec())
            self.last_ground_stamp = ground.header.stamp
        self.coverage.integrate(footprint[:, 0], footprint[:, 1],
                                np.zeros(len(footprint), dtype=np.int8), pose_stamp.to_sec())
        self.coverage_stamp = ground.header.stamp

    @staticmethod
    def planar_yaw(q):
        if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
            raise ValueError('invalid orientation')
        if abs(q.x) > 1e-3 or abs(q.y) > 1e-3 or abs(q.z*q.z+q.w*q.w-1) > 1e-3:
            raise ValueError('occupancy map must be gravity aligned')
        return 2*math.atan2(q.z, q.w)

    def receive(self, message):
        started = time.monotonic()
        try:
            age = (self.ros.Time.now()-message.header.stamp).to_sec()
            if message.header.stamp <= self.last_stamp:
                return
            if not -.1 <= age <= 1.:
                raise ValueError('local observation stale')
            info = message.info
            if (info.width <= 0 or info.height <= 0
                    or len(message.data) != info.width*info.height
                    or abs(info.resolution-self.memory.resolution) > 1e-6
                    or not message.header.frame_id):
                raise ValueError('invalid local grid geometry')
            yaw = self.planar_yaw(info.origin.orientation)
            t = self.tf.lookup_transform(self.frame, message.header.frame_id,
                                         message.header.stamp, self.ros.Duration(.1)).transform
            rotation = self.planar_yaw(t.rotation)
            data = np.asarray(message.data, dtype=np.int8).reshape(info.height, info.width)
            yy, xx = np.nonzero(data != -1)
            lx, ly = (xx+.5)*info.resolution, (yy+.5)*info.resolution
            ox, oy = info.origin.position.x, info.origin.position.y
            x = ox+math.cos(yaw)*lx-math.sin(yaw)*ly
            y = oy+math.sin(yaw)*lx+math.cos(yaw)*ly
            wx = t.translation.x+math.cos(rotation)*x-math.sin(rotation)*y
            wy = t.translation.y+math.sin(rotation)*x+math.cos(rotation)*y
            if len(xx) == 0:
                self.status_pub.publish(self.String(data='refresh failed: no observed cells'))
                return
            self.update_coverage(message.header.stamp)
            self.memory.integrate(wx, wy, data[yy, xx])
            self.publish(message.header.stamp)
            self.last_stamp = message.header.stamp
            self.status_pub.publish(self.String(data='ready: generation=%d processing_ms=%.2f' %
                (self.memory.generation, (time.monotonic()-started)*1000)))
        except Exception as error:
            self.status_pub.publish(self.String(data='refresh failed: '+str(error)))
            self.ros.logwarn_throttle(2., 'Observation memory: %s', error)

    def publish(self, stamp):
        output = self.grid(self.memory, stamp)
        self.map_pub.publish(output)
        self.metadata_pub.publish(output.info)
        coverage = self.grid(self.coverage, self.coverage_stamp)
        self.coverage_pub.publish(coverage)
        known = self.coverage.cells != -1
        age = np.full(self.coverage.cells.shape, -1, dtype=np.int8)
        age[known] = np.clip(stamp.to_sec()-self.coverage.last_seen[known], 0, 100).astype(np.int8)
        # rospy retains the message object for latched subscribers. Never
        # mutate the already-published binary coverage into an age image.
        age_output = self.grid(self.coverage, stamp)
        age_output.data = age.ravel().tolist()
        self.coverage_age_pub.publish(age_output)

    def grid(self, memory, stamp):
        output = self.Grid()
        output.header.frame_id, output.header.stamp = self.frame, stamp
        output.info.map_load_time = self.started
        output.info.resolution = memory.resolution
        output.info.height, output.info.width = memory.cells.shape
        output.info.origin.position.x = memory.x0*memory.resolution
        output.info.origin.position.y = memory.y0*memory.resolution
        output.info.origin.orientation.w = 1.
        output.data = memory.cells.ravel().tolist()
        return output


if __name__ == '__main__':
    import rospy
    rospy.init_node('go2_observed_map_memory')
    MemoryNode()
    rospy.spin()
