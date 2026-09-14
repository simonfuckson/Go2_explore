#!/usr/bin/env python3

import collections
import math
import threading
import time
import numpy as np

import rospy
import tf2_ros
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String


class FrontierCostmapAdapter:
    def __init__(self):
        input_topic = rospy.get_param(
            "~input_topic", "/move_base/global_costmap/costmap"
        )
        output_topic = rospy.get_param("~output_topic", "/exploration/frontier_map")
        updates_topic = rospy.get_param(
            "~updates_topic", "/move_base/global_costmap/costmap_updates"
        )
        self.lethal_threshold = rospy.get_param("~lethal_threshold", 99)
        self.min_frontier_cells = rospy.get_param("~min_frontier_cells", 5)
        self.min_frontier_size = rospy.get_param('/explore/min_frontier_size', 0.50)
        self.completion_hold_time = rospy.get_param("~completion_hold_time", 30.0)
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "base_link")
        self.map = None
        self.coverage = None
        self.coverage_gap_min_area = rospy.get_param('~coverage_gap_min_area', .15)
        self.coverage_summary = rospy.Publisher('/exploration/coverage_status', String, queue_size=1, latch=True)
        self.last_frontier_time = None
        self.map_lock = threading.Lock()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            output_topic, OccupancyGrid, queue_size=1, latch=True
        )
        self.frontier_status = rospy.Publisher('/exploration/frontier_available', Bool, queue_size=1)
        self.validated_status = rospy.Publisher('/exploration/frontier_status', String, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, OccupancyGrid, self.map_callback, queue_size=1
        )
        self.update_subscriber = rospy.Subscriber(
            updates_topic, OccupancyGridUpdate, self.update_callback, queue_size=10
        )
        self.coverage_subscriber = rospy.Subscriber('/exploration/coverage_map', OccupancyGrid,
                                                    self.coverage_callback, queue_size=1)
        rospy.loginfo(
            "Frontier map adapter %s + %s -> %s "
            "(lethal >= %d, completion hold %.1f s)",
            input_topic,
            updates_topic,
            output_topic,
            self.lethal_threshold,
            self.completion_hold_time,
        )

    def coverage_callback(self, message):
        with self.map_lock:
            self.coverage = message

    def map_callback(self, source):
        with self.map_lock:
            self.map = OccupancyGrid()
            self.map.header = source.header
            self.map.info = source.info
            self.map.data = list(source.data)
            self.publish_map()

    def update_callback(self, update):
        with self.map_lock:
            if self.map is None:
                return
            width = self.map.info.width
            height = self.map.info.height
            if (update.x + update.width > width or update.y + update.height > height
                    or update.width == 0 or update.height == 0
                    or len(update.data) != update.width*update.height
                    or (update.header.frame_id and update.header.frame_id != self.map.header.frame_id)):
                rospy.logwarn_throttle(2.0, "Ignoring out-of-bounds costmap update")
                self.validated_status.publish(String(data='INVALID'))
                self.frontier_status.publish(Bool(data=False))
                return
            for row in range(update.height):
                source_start = row * update.width
                target_start = (update.y + row) * width + update.x
                self.map.data[target_start : target_start + update.width] = update.data[
                    source_start : source_start + update.width
                ]
            self.map.header.stamp = update.header.stamp
            self.publish_map()

    def publish_map(self):
        if (self.map.info.resolution <= 0 or self.map.info.width <= 0
                or self.map.info.height <= 0
                or len(self.map.data) != self.map.info.width*self.map.info.height
                or not self.map.header.frame_id
                or abs((rospy.Time.now()-self.map.header.stamp).to_sec()) > 2.5):
            self.validated_status.publish(String(data='INVALID'))
            self.frontier_status.publish(Bool(data=False))
            return
        output = OccupancyGrid()
        output.header = self.map.header
        output.info = self.map.info
        output.data = [
            -1 if value < 0 else 100 if value >= self.lethal_threshold else 0
            for value in self.map.data
        ]
        reachable = self.reachable_cells(output)
        try:
            if reachable is None:
                raise ValueError('reachable collision region unavailable')
            if self.coverage is None or abs((rospy.Time.now()-self.coverage.header.stamp).to_sec()) > 2.5:
                raise ValueError('coverage missing or stale')
            covered = self.project_coverage(self.coverage, output)
            output.data, cells, gap_count = self.coverage_frontiers(
                output.data, output.info.width, output.info.height, reachable, covered,
                self.coverage_gap_min_area, output.info.resolution)
            required = max(self.min_frontier_cells, int(math.ceil(self.min_frontier_size/output.info.resolution-1e-9)))
            has_frontier = self.has_large_cluster(cells, output.info.width, required)
            self.coverage_summary.publish(String(data='ready: reachable=%d uncovered_gap_cells=%d gap_min_area=%.3f' %
                (len(reachable), gap_count, self.coverage_gap_min_area)))
        except ValueError as error:
            self.validated_status.publish(String(data='INVALID'))
            self.frontier_status.publish(Bool(data=False))
            self.coverage_summary.publish(String(data='invalid: '+str(error)))
            return
        self.validated_status.publish(String(data='AVAILABLE' if has_frontier else 'EMPTY'))
        self.frontier_status.publish(Bool(data=has_frontier))
        now = time.monotonic()
        if has_frontier:
            self.last_frontier_time = now
        elif self.last_frontier_time is None:
            # A valid observed room can be covered before the first movement.
            pass
        # Publish the current map even when temporarily empty. Completion is
        # decided by the supervisor; retaining old geometry can send stale goals.
        self.publisher.publish(output)

    @staticmethod
    def project_coverage(coverage, grid):
        a, b = coverage.info, grid.info
        if (coverage.header.frame_id != grid.header.frame_id or a.resolution <= 0
                or abs(a.resolution-b.resolution) > 1e-6
                or len(coverage.data) != a.width*a.height or a.width <= 0 or a.height <= 0):
            raise ValueError('coverage geometry mismatch')
        for q in (a.origin.orientation, b.origin.orientation):
            if abs(q.x)+abs(q.y)+abs(q.z) > 1e-6 or abs(q.w-1.) > 1e-6:
                raise ValueError('coverage grid must be axis aligned')
        dx = (a.origin.position.x-b.origin.position.x)/b.resolution
        dy = (a.origin.position.y-b.origin.position.y)/b.resolution
        if abs(dx-round(dx))+abs(dy-round(dy)) > 1e-3:
            raise ValueError('coverage grid origin mismatch')
        dx, dy = int(round(dx)), int(round(dy))
        result = np.full((b.height,b.width), -1, dtype=np.int8)
        x0,y0,x1,y1 = max(0,dx),max(0,dy),min(b.width,dx+a.width),min(b.height,dy+a.height)
        if x1>x0 and y1>y0:
            result[y0:y1,x0:x1] = np.asarray(coverage.data, dtype=np.int8).reshape(a.height,a.width)[y0-dy:y1-dy,x0-dx:x1-dx]
        return result.ravel()

    @staticmethod
    def coverage_frontiers(data, width, height, reachable, covered, min_area, resolution):
        # Connectivity comes ONLY from the collision map, not ground visibility.
        gaps = {i for i in reachable if covered[i] < 0}
        large_gaps = set()
        required = max(1, int(math.ceil(min_area/(resolution*resolution)-1e-9)))
        while gaps:
            cluster = {gaps.pop()}
            pending = list(cluster)
            while pending:
                i = pending.pop()
                x,y = i%width,i//width
                for ny in range(max(0,y-1),min(height,y+2)):
                    for nx in range(max(0,x-1),min(width,x+2)):
                        n = ny*width+nx
                        if n in gaps:
                            gaps.remove(n);cluster.add(n);pending.append(n)
            if len(cluster) >= required:
                large_gaps.update(cluster)
        targets = set(large_gaps)
        for i in reachable:
            x,y = i%width,i//width
            for nx,ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                if 0<=nx<width and 0<=ny<height and data[ny*width+nx] < 0:
                    targets.add(ny*width+nx)
        result = list(data)
        for i in large_gaps:
            result[i] = -1
        return result, targets, len(large_gaps)

    def has_reachable_frontier_cluster(self, grid):
        reachable = self.reachable_cells(grid)
        if reachable is None:
            return None
        _, cells, _ = self.coverage_frontiers(grid.data, grid.info.width, grid.info.height,
            reachable, np.zeros(len(grid.data), dtype=np.int8), 1., grid.info.resolution)
        required = max(self.min_frontier_cells, int(math.ceil(self.min_frontier_size/grid.info.resolution-1e-9)))
        return self.has_large_cluster(cells, grid.info.width, required)

    def reachable_cells(self, grid):
        data = grid.data
        width = grid.info.width
        height = grid.info.height
        try:
            transform = self.tf_buffer.lookup_transform(
                grid.header.frame_id,
                self.robot_base_frame,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except tf2_ros.TransformException as error:
            rospy.logwarn_throttle(2.0, "Frontier reachability TF unavailable: %s", error)
            return None

        if abs((rospy.Time.now()-transform.header.stamp).to_sec()) > 0.5:
            return None

        robot_x = int(
            math.floor(
                (transform.transform.translation.x - grid.info.origin.position.x)
                / grid.info.resolution
            )
        )
        robot_y = int(
            math.floor(
                (transform.transform.translation.y - grid.info.origin.position.y)
                / grid.info.resolution
            )
        )
        if not (0 <= robot_x < width and 0 <= robot_y < height):
            return None

        start = robot_y * width + robot_x
        if data[start] != 0:
            start = self.nearest_free_cell(data, width, height, robot_x, robot_y)
            if start is None:
                return None

        reachable = {start}
        pending = collections.deque([start])
        while pending:
            index = pending.popleft()
            x = index % width
            y = index // width
            for neighbor_x, neighbor_y in (
                (x - 1, y),
                (x + 1, y),
                (x, y - 1),
                (x, y + 1),
            ):
                if not (0 <= neighbor_x < width and 0 <= neighbor_y < height):
                    continue
                neighbor = neighbor_y * width + neighbor_x
                if neighbor not in reachable and data[neighbor] == 0:
                    reachable.add(neighbor)
                    pending.append(neighbor)

        return reachable

    @staticmethod
    def nearest_free_cell(data, width, height, robot_x, robot_y):
        for radius in range(1, 11):
            for y in range(max(0, robot_y - radius), min(height, robot_y + radius + 1)):
                for x in range(max(0, robot_x - radius), min(width, robot_x + radius + 1)):
                    index = y * width + x
                    if data[index] == 0:
                        return index
        return None

    def has_large_cluster(self, frontier_cells, width, required=None):
        required = self.min_frontier_cells if required is None else required
        while frontier_cells:
            pending = [frontier_cells.pop()]
            cluster_size = 0
            while pending:
                index = pending.pop()
                cluster_size += 1
                if cluster_size >= required:
                    return True
                x = index % width
                y = index // width
                for neighbor_y in range(y - 1, y + 2):
                    for neighbor_x in range(x - 1, x + 2):
                        if not 0 <= neighbor_x < width or neighbor_y < 0:
                            continue
                        neighbor = neighbor_y * width + neighbor_x
                        if neighbor in frontier_cells:
                            frontier_cells.remove(neighbor)
                            pending.append(neighbor)
        return False


def main():
    rospy.init_node("go2_frontier_costmap_adapter")
    FrontierCostmapAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
