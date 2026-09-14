"""ROS-independent interlock, grid-update, and swept-footprint logic."""

import math
import numpy as np


NAV_INPUT_TOPIC = "/exploration/cmd_vel_shaped"
DRIVER_OUTPUT_TOPIC = "/cmd_vel_safe"
EXPECTED_NAV_CALLER_ID = "/go2_velocity_shaper"
EXPECTED_NODE_NAME = "/go2_exploration_safety"


def command_caller_is_expected(connection_header):
    return (
        isinstance(connection_header, dict)
        and connection_header.get("callerid") == EXPECTED_NAV_CALLER_ID
    )


class MoveBaseGoalTracker:
    """Bind command/plan handshakes to action goals received on /goal."""

    def __init__(self):
        self.current_id = None
        self.seen_ids = set()
        self.generation = 0
        self.live_since = None
        self.status_confirmed_generation = None

    @property
    def has_current_goal(self):
        return self.current_id is not None

    @property
    def status_confirmed(self):
        return self.status_confirmed_generation == self.generation

    def accept_goal(self, goal_id, now):
        if goal_id is None:
            raise ValueError("goal_id is required")
        if goal_id == self.current_id:
            return False
        self.current_id = goal_id
        self.seen_ids.add(goal_id)
        self.generation += 1
        self.live_since = now
        self.status_confirmed_generation = None
        return True

    def observe_status(self, live_ids, terminal_ids):
        live_ids = frozenset(live_ids)
        terminal_ids = frozenset(terminal_ids)
        if self.current_id is None:
            return "idle"
        if self.current_id in live_ids:
            if not self.status_confirmed:
                self.status_confirmed_generation = self.generation
                return "confirmed"
            return "live"
        if self.current_id in terminal_ids:
            self.current_id = None
            self.generation += 1
            self.live_since = None
            self.status_confirmed_generation = None
            return "terminal"
        return "missing"


class SafetyInterlock:
    def __init__(self):
        self.armed = False
        self.latched = False
        self.reason = "startup_lock"

    def arm(self, health_ok, has_live_goal):
        if self.latched:
            return False, "reset required after: " + self.reason
        if not health_ok:
            return False, "safety inputs are not healthy"
        if has_live_goal:
            return False, "cancel the current move_base goal before arming"
        self.armed = True
        self.reason = "armed"
        return True, "velocity gate armed"

    def latch(self, reason):
        self.armed = False
        self.latched = True
        self.reason = reason

    def reset(self, health_ok, collision_clear):
        if not health_ok:
            return False, "cannot reset unhealthy gate"
        if not collision_clear:
            return False, "stop region is still occupied"
        self.armed = False
        self.latched = False
        self.reason = "reset_lock"
        return True, "fault reset; gate remains locked until arm"


def motion_requested(linear_x, angular_z, linear_deadband, angular_deadband):
    return (
        abs(linear_x) >= linear_deadband
        or abs(angular_z) >= angular_deadband
    )


def goal_handshake_fault(
    has_live_goal,
    goal_age,
    handshake_timeout,
    command_matches_goal,
    local_plan_matches_goal,
    local_plan_poses,
):
    """Return a latched-fault reason when a new goal never becomes usable."""
    if not has_live_goal:
        return None
    complete = (
        command_matches_goal
        and local_plan_matches_goal
        and local_plan_poses > 0
    )
    if complete or goal_age <= handshake_timeout:
        return None
    return "goal_handshake_timeout"


def snapshot_is_current(
    snapshot_state_generation,
    snapshot_command_generation,
    snapshot_safety_input_generation,
    current_state_generation,
    current_command_generation,
    current_safety_input_generation,
):
    return (
        snapshot_state_generation == current_state_generation
        and snapshot_command_generation == current_command_generation
        and snapshot_safety_input_generation == current_safety_input_generation
    )


def motion_fault_reason(
    linear_x,
    angular_z,
    command_valid,
    command_age,
    command_timeout,
    has_live_goal,
    status_age,
    status_timeout,
    allow_reverse,
    linear_deadband,
    angular_deadband,
    require_local_plan,
    local_plan_age,
    local_plan_timeout,
    local_plan_poses,
    cloud_blocked,
    costmap_blocked,
    allow_stationary_command_wait=False,
):
    moving = motion_requested(
        linear_x, angular_z, linear_deadband, angular_deadband
    )
    if not command_valid:
        return "invalid_command"
    # A planner can pause while its last command already requests a stop.
    # The gate still emits zero on stale input, but an active goal alone must
    # not turn this stationary wait into an operator-reset fault. A stale
    # movement command remains a latched fault at the same short deadline.
    if command_age > command_timeout and (
        moving or (has_live_goal and not allow_stationary_command_wait)
    ):
        return "command_stream_stale"
    if status_age > status_timeout:
        return "move_base_status_stale"
    if moving and not has_live_goal:
        return "motion_without_live_goal"
    if linear_x < -1e-4 and not allow_reverse:
        return "reverse_command_inhibited"
    if moving and command_age <= command_timeout and require_local_plan:
        if local_plan_age > local_plan_timeout:
            return "local_plan_stale"
        if local_plan_poses == 0:
            return "local_plan_empty"
    if cloud_blocked:
        return "obstacle_in_cloud_stop_region"
    if costmap_blocked:
        return "obstacle_in_costmap_stop_region"
    return None


def apply_grid_update(data, width, height, x, y, update_width, update_height,
                      update_data):
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError("invalid base grid")
    if update_width <= 0 or update_height <= 0:
        raise ValueError("empty grid update")
    if x < 0 or y < 0 or x + update_width > width or y + update_height > height:
        raise ValueError("grid update is out of bounds")
    if len(update_data) != update_width * update_height:
        raise ValueError("invalid grid update length")
    output = list(data)
    for row in range(update_height):
        source = row * update_width
        target = (y + row) * width + x
        output[target : target + update_width] = update_data[
            source : source + update_width
        ]
    return output


def vertical_span_obstacle_cells(
    points,
    cell_size,
    min_points,
    min_vertical_span,
    min_z,
    max_z,
    body_front,
    body_rear,
    body_half_width,
):
    """Reduce a cropped cloud to XY cells containing vertical structure.

    A locally planar floor or ramp has little Z variation inside one small XY
    cell. Thin upright objects have returns at multiple heights in the same
    cell. The returned XY coordinate is the closest sample in each obstacle
    cell, which is conservative for clearance reporting.
    """
    if cell_size <= 0.0:
        raise ValueError("cell_size must be positive")
    if min_points < 2:
        raise ValueError("min_points must be at least two")
    if min_vertical_span <= 0.0 or min_z >= max_z:
        raise ValueError("invalid vertical-span limits")

    cells = {}
    for x, y, z in points:
        if not (
            math.isfinite(x)
            and math.isfinite(y)
            and math.isfinite(z)
        ):
            continue
        if z < min_z or z > max_z:
            continue
        if -body_rear <= x <= body_front and abs(y) <= body_half_width:
            continue
        key = (math.floor(x / cell_size), math.floor(y / cell_size))
        distance_squared = x * x + y * y
        current = cells.get(key)
        if current is None:
            cells[key] = [1, z, z, distance_squared, x, y]
            continue
        current[0] += 1
        current[1] = min(current[1], z)
        current[2] = max(current[2], z)
        if distance_squared < current[3]:
            current[3] = distance_squared
            current[4] = x
            current[5] = y

    obstacles = []
    for count, lowest, highest, _distance_squared, x, y in cells.values():
        vertical_span = highest - lowest
        if count >= min_points and vertical_span >= min_vertical_span:
            obstacles.append((x, y, vertical_span, count))
    return tuple(obstacles)


class StationaryWindow:
    """Require fresh distinct poses and bounded motion throughout a stop dwell."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.since = self.last_check = self.last_stamp = None
        self.anchor = self.frame = None
        self.samples = 0
        self.ready = False

    def update(self, now, stamp, frame, pose, eligible):
        if (not eligible or not frame or stamp <= 0
                or not all(math.isfinite(v) for v in (now, stamp, *pose))):
            self.reset()
            return False
        discontinuous = (self.last_check is not None and
                         (now < self.last_check or now-self.last_check > .20))
        reversed_stamp = self.last_stamp is not None and stamp < self.last_stamp
        moved = self.anchor is not None and (
            math.hypot(pose[0]-self.anchor[0], pose[1]-self.anchor[1]) > .02
            or abs(pose[2]-self.anchor[2]) > .03
            or abs(math.atan2(math.sin(pose[3]-self.anchor[3]),
                              math.cos(pose[3]-self.anchor[3]))) > .03)
        if discontinuous or reversed_stamp or self.frame != frame or moved:
            self.reset()
        if self.since is None:
            self.since, self.anchor, self.frame = now, tuple(pose), frame
        self.last_check = now
        if self.last_stamp is None or stamp > self.last_stamp:
            self.samples += 1
            self.last_stamp = stamp
        self.ready = now-self.since >= 2.0 and self.samples >= 10
        return self.ready


class DirectionalStopRegion:
    def stationary_clearance(self, points):
        """Check the current complete footprint, including its rear edge."""
        if len(points) == 0:
            return False, math.inf
        xy = np.asarray(points, dtype=float)
        x, y = xy[:, 0], xy[:, 1]
        inside = ((x >= -self.rear-self.footprint_margin)
                  & (x <= self.front+self.footprint_margin)
                  & (np.abs(y) <= self.half_width+self.footprint_margin))
        return ((True, float(np.min(np.hypot(x[inside], y[inside]))))
                if np.any(inside) else (False, math.inf))

    def clearance(self, points, linear_x, angular_z):
        """Vectorized equivalent of contains; preserves every swept rectangle."""
        poses = self.prediction_poses(linear_x, angular_z)
        if not poses or len(points) == 0:
            return False, math.inf
        xy = np.asarray(points, dtype=float)
        x, y = xy[:, 0], xy[:, 1]
        inside = np.zeros(len(x), dtype=bool)
        margin = self.footprint_margin
        width = self.half_width + margin
        if linear_x > self.linear_deadband:
            distance = max(self.min_forward_clearance,
                linear_x*self.reaction_time + linear_x**2/(2*self.linear_deceleration))
            inside |= (x >= self.front-margin) & (x <= self.front+distance) & (np.abs(y) <= width)
        elif linear_x < -self.linear_deadband:
            speed = abs(linear_x)
            distance = max(self.min_reverse_clearance,
                speed*self.reaction_time + speed**2/(2*self.linear_deceleration))
            inside |= (x >= -self.rear-distance) & (x <= -self.rear+margin) & (np.abs(y) <= width)
        for px, py, yaw in poses:
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = x-px, y-py
            lx, ly = c*dx+s*dy, -s*dx+c*dy
            inside |= (lx >= -self.rear-margin) & (lx <= self.front+margin) & (np.abs(ly) <= width)
        if not np.any(inside):
            return False, math.inf
        return True, float(np.min(np.hypot(x[inside], y[inside])))

    def __init__(
        self,
        front,
        rear,
        half_width,
        footprint_margin,
        min_forward_clearance,
        min_reverse_clearance,
        reaction_time,
        linear_deceleration,
        angular_deceleration,
        max_prediction_horizon,
        prediction_samples,
        linear_deadband,
        angular_deadband,
    ):
        self.front = front
        self.rear = rear
        self.half_width = half_width
        self.footprint_margin = footprint_margin
        self.min_forward_clearance = min_forward_clearance
        self.min_reverse_clearance = min_reverse_clearance
        self.reaction_time = reaction_time
        self.linear_deceleration = linear_deceleration
        self.angular_deceleration = angular_deceleration
        self.max_prediction_horizon = max_prediction_horizon
        self.prediction_samples = prediction_samples
        self.linear_deadband = linear_deadband
        self.angular_deadband = angular_deadband

    def prediction_poses(self, linear_x, angular_z):
        velocity = linear_x
        angular = angular_z
        if abs(velocity) < self.linear_deadband:
            velocity = 0.0
        if abs(angular) < self.angular_deadband:
            angular = 0.0
        if velocity == 0.0 and angular == 0.0:
            return []

        horizon = self.reaction_time
        if velocity != 0.0:
            horizon = max(
                horizon,
                self.reaction_time + abs(velocity) / self.linear_deceleration,
            )
        if angular != 0.0:
            horizon = max(
                horizon,
                self.reaction_time + abs(angular) / self.angular_deceleration,
            )
        horizon = min(self.max_prediction_horizon, horizon)
        poses = []
        for sample in range(1, self.prediction_samples + 1):
            sample_time = horizon * sample / self.prediction_samples
            theta = angular * sample_time
            if abs(angular) < 1e-6:
                x = velocity * sample_time
                y = 0.0
            else:
                radius = velocity / angular
                x = radius * math.sin(theta)
                y = radius * (1.0 - math.cos(theta))
            poses.append((x, y, theta))
        return poses

    def contains(self, x, y, linear_x, angular_z, poses=None):
        width = self.half_width + self.footprint_margin
        if linear_x > self.linear_deadband:
            stopping_distance = (
                linear_x * self.reaction_time
                + linear_x * linear_x / (2.0 * self.linear_deceleration)
            )
            forward = max(self.min_forward_clearance, stopping_distance)
            if (
                self.front - self.footprint_margin
                <= x
                <= self.front + forward
                and abs(y) <= width
            ):
                return True
        elif linear_x < -self.linear_deadband:
            speed = abs(linear_x)
            stopping_distance = (
                speed * self.reaction_time
                + speed * speed / (2.0 * self.linear_deceleration)
            )
            reverse = max(self.min_reverse_clearance, stopping_distance)
            if (
                -self.rear - reverse
                <= x
                <= -self.rear + self.footprint_margin
                and abs(y) <= width
            ):
                return True

        if poses is None:
            poses = self.prediction_poses(linear_x, angular_z)
        for pose_x, pose_y, pose_yaw in poses:
            cosine = math.cos(pose_yaw)
            sine = math.sin(pose_yaw)
            dx = x - pose_x
            dy = y - pose_y
            local_x = cosine * dx + sine * dy
            local_y = -sine * dx + cosine * dy
            if (
                -self.rear - self.footprint_margin
                <= local_x
                <= self.front + self.footprint_margin
                and abs(local_y) <= width
            ):
                return True
        return False
