"""Continuity state independent of ROS transports, tested with recorded values."""
import math


def quaternion_distance(first, second):
    """Shortest 3-D rotation, invariant to frame tilt and quaternion sign."""
    dot=sum(a*b for a,b in zip(first,second))
    scale=math.sqrt(sum(a*a for a in first)*sum(b*b for b in second))
    return 2*math.acos(min(1.,abs(dot)/scale))


class OdometryHealth:
    def __init__(self):
        self.samples = {}
        self.fault = None
        self.first_fault_context = None
        self.orientations = {}

    def observe(self, stream, stamp, position, quaternion, twist, received):
        if self.fault:
            return  # Preserve the first fault and its measured before/after pair.
        values = tuple(position)+tuple(quaternion)+tuple(twist)+(stamp,)
        if not all(math.isfinite(v) for v in values) or stamp <= 0:
            self.fault = 'nonfinite_or_unstamped_odometry'
            return
        norm = sum(q*q for q in quaternion)
        if abs(norm-1.0) > .02:
            self.fault = 'invalid_odometry_quaternion'
            return
        x,y,z,w = quaternion
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        previous=self.samples.get(stream)
        if previous:
            dt=stamp-previous[0]
            if dt < 0:
                self.fault='odometry_time_reversed'
                return
            if dt == 0:
                return  # repeated messages cannot refresh a frozen sensor
            distance=math.sqrt(sum((a-b)**2 for a,b in zip(position,previous[1])))
            yaw_delta=abs(math.atan2(math.sin(yaw-previous[2]),math.cos(yaw-previous[2])))
            # body_lio is tilted relative to the body. Its Euler yaw can jump
            # near vertical pitch while the measured 3-D rotation stays small.
            angle=quaternion_distance(quaternion,self.orientations[stream])
            if distance > .25+1.2*dt or angle > .35+1.6*dt:
                self.fault='odometry_discontinuity'
                self.first_fault_context=dict(stream=stream,previous_stamp=previous[0],stamp=stamp,
                    delta_sec=dt,previous_position=list(previous[1]),position=list(position),
                    previous_quaternion=list(self.orientations[stream]),quaternion=list(quaternion),
                    translation_delta_m=distance,translation_limit_m=.25+1.2*dt,
                    yaw_delta_rad=yaw_delta,rotation_delta_rad=angle,
                    rotation_limit_rad=.35+1.6*dt,twist=list(twist))
                return
        self.samples[stream]=(stamp,tuple(position),yaw,received)
        self.orientations[stream]=tuple(quaternion)

    def problem(self, now_ros, now_wall, initialized, tf_stamp):
        if self.fault:
            return self.fault
        if not initialized:
            return 'waiting_for_initialization'
        for stream in ('lio','nav'):
            value=self.samples.get(stream)
            if not value or now_wall-value[3] > .50 or not -.10 <= now_ros-value[0] <= .50:
                return stream+'_odometry_stale'
        if tf_stamp <= 0 or not -.10 <= now_ros-tf_stamp <= .50:
            return 'robot_tf_stale'
        return None
