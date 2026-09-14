"""GO2 geometry, command contract and independent stream freshness checks."""
import math


def body_envelope(footprint):
    if isinstance(footprint, str):
        import ast
        footprint = ast.literal_eval(footprint)
    points = [(float(p[0]), float(p[1])) for p in footprint]
    if len(points) < 3 or not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('invalid GO2 footprint')
    result = max(p[0] for p in points), -min(p[0] for p in points), max(abs(p[1]) for p in points)
    if min(result) <= 0:
        raise ValueError('GO2 footprint must surround base_link')
    return result


def command_problem(message, caller, expected, max_v=.30, max_w=.50):
    values = (message.linear.x, message.linear.y, message.linear.z,
              message.angular.x, message.angular.y, message.angular.z)
    if caller != expected:
        return 'unexpected_command_publisher'
    if not all(math.isfinite(v) for v in values):
        return 'invalid_command'
    if any(abs(v) > 1e-4 for v in (values[1], values[2], values[3], values[4])):
        return 'unsupported_command_axis'
    if values[0] < -1e-4:
        return 'reverse_command_inhibited'
    if values[0] > max_v + 1e-4 or abs(values[5]) > max_w + 1e-4:
        return 'command_limit_exceeded'
    return None


def sdk_effective_velocity(v, w):
    """Match the preserved SDK deadbands/floor BEFORE collision checking."""
    v = 0.0 if abs(v) < .025 else max(-.30, min(.30, v))
    w = 0.0 if abs(w) < .01 else math.copysign(min(.50, max(.04, abs(w))), w)
    return v, w


class UpstreamHealth:
    def __init__(self):
        self.odom = self.terrain = False
        self.odom_time = self.terrain_time = self.map_time = None
        self.map_token = None
        self.raw_time = None
        self.raw_generation = None

    def heartbeat(self, stream, value, now):
        setattr(self, stream, bool(value))
        setattr(self, stream+'_time', now)

    def map_status(self, text, now):
        if text.startswith('ready:') and text != self.map_token:
            self.map_time, self.map_token = now, text

    @staticmethod
    def age(now, then):
        return math.inf if then is None else max(0.0, now-then)

    def problem(self, now):
        for name, limit in (('odom', .50), ('terrain', .75)):
            if not getattr(self, name) or self.age(now, getattr(self, name+'_time')) > limit:
                return name+'_health_unavailable'
        if self.age(now, self.map_time) > 2.0:
            return 'observation_map_stale'
        return None

    def raw_age(self, now, generation):
        return self.age(now, self.raw_time) if self.raw_generation == generation else math.inf
