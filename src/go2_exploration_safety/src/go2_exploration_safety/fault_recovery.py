"""Explicitly recoverable input failures; never clear estimator continuity faults."""
import math

RECOVERABLE_FAULTS = frozenset({
    'terrain_health_unavailable', 'odom_health_unavailable',
    'observation_map_stale', 'obstacle_cloud_stale', 'raw_cloud_stale',
    'raw_cloud_too_sparse', 'local_costmap_missing', 'local_costmap_stale',
    'robot_tf_stale', 'tf_unavailable',
    'command_stream_stale', 'move_base_status_stale', 'local_plan_stale',
    'local_plan_empty', 'goal_handshake_timeout',
})


class RecoveryDwell:
    """A missed/invalid check or changed fault restarts the entire dwell."""
    def __init__(self, seconds=10.0):
        self.seconds = max(10.0, float(seconds))
        if not math.isfinite(self.seconds):
            self.seconds = 10.0
        self.reset()

    def reset(self):
        self.since = self.checked_at = self.reason = None
        self.generation = 0

    def update(self, now, reason, ready, generation):
        if not ready:
            self.reset()
            return False
        if (self.since is None or reason != self.reason or self.checked_at is None
                or not 0 <= now-self.checked_at <= .75 or generation < self.generation):
            self.since, self.generation, self.reason = now, generation, reason
        self.checked_at = now
        return now-self.since >= self.seconds and generation-self.generation >= 2


def sdk_recovery_problem(values):
    """Real SDK telemetry must actively support recovery, not merely lack errors."""
    def number(key):
        try:
            result = float(values[key])
            return result if math.isfinite(result) else math.inf
        except (KeyError, ValueError, TypeError):
            return math.inf
    if (values.get('manual_resume_pending') != 'false'
            or number('mode_override_api') != 0
            or number('remote_buttons') != 0 or number('remote_axis_max') > .10):
        return 'remote_or_posture_override'
    if values.get('no_step_response') != 'false':
        return 'motion_response_fault'
    if not all(-.05 <= number(key) <= .30 for key in
               ('low_state_age_sec', 'sport_state_age_sec', 'remote_age_sec')):
        return 'sdk_telemetry_stale'
    # This firmware reported sport_state_error_code=2010 in the successful
    # measured walking trial too. Preserve the original bridge's health policy;
    # do not invent an interpretation of that opaque firmware field.
    if number('_diagnostic_level') != 0 or values.get('last_gait_error', ''):
        return 'sdk_or_gait_fault'
    if (values.get('active_motion_mode') != values.get('required_motion_mode')
            or not values.get('required_motion_mode')):
        return 'sdk_motion_mode_mismatch'
    battery, minimum = number('battery_soc_percent'), number('min_enable_battery_percent')
    if not 0 <= minimum <= battery <= 100:
        return 'battery_unavailable_or_low'
    return None
