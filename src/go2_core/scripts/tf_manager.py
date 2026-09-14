#!/usr/bin/env python3
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


def require_param(name):
    if not rospy.has_param(name):
        rospy.logfatal("Missing required param: %s", name)
        raise RuntimeError(name)
    return rospy.get_param(name)


def validate_pose(name, cfg):
    for key in ["x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg"]:
        if key not in cfg:
            raise RuntimeError("{} missing key {}".format(name, key))
        if not math.isfinite(float(cfg[key])):
            raise RuntimeError("{}.{} is not finite".format(name, key))


def pose_tf(parent, child, cfg):
    validate_pose(child, cfg)
    q = quaternion_from_euler(
        math.radians(float(cfg["roll_deg"])),
        math.radians(float(cfg["pitch_deg"])),
        math.radians(float(cfg["yaw_deg"])),
    )
    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(cfg["x"])
    msg.transform.translation.y = float(cfg["y"])
    msg.transform.translation.z = float(cfg["z"])
    msg.transform.rotation.x = q[0]
    msg.transform.rotation.y = q[1]
    msg.transform.rotation.z = q[2]
    msg.transform.rotation.w = q[3]
    return msg


def identity_tf(parent, child):
    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.rotation.w = 1.0
    return msg


if __name__ == "__main__":
    rospy.init_node("go2_tf_manager")

    odom_frame = require_param("/frames/odom")
    robot_init_frame = require_param("/frames/robot_init")
    base_frame = require_param("/frames/base_link")
    lidar_frame = require_param("/frames/lidar_link")
    camera_frame = require_param("/frames/camera_link")

    base_to_lidar = require_param("/mid360_mount/base_link_to_lidar_link")
    base_to_camera = require_param("/d435i_mount/base_link_to_camera_link")

    frames = [odom_frame, robot_init_frame, base_frame, lidar_frame, camera_frame]
    if len(frames) != len(set(frames)):
        raise RuntimeError("Public frame names must be unique: {}".format(frames))

    tf_odom_init = identity_tf(odom_frame, robot_init_frame)
    tf_base_lidar = pose_tf(base_frame, lidar_frame, base_to_lidar)
    tf_base_camera = pose_tf(base_frame, camera_frame, base_to_camera)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform([tf_odom_init, tf_base_lidar, tf_base_camera])

    rospy.loginfo("Published static TF: %s -> %s (identity)",
                  odom_frame, robot_init_frame)
    rospy.loginfo("Published static TF: %s -> %s",
                  base_frame, lidar_frame)
    rospy.loginfo("Published static TF: %s -> %s",
                  base_frame, camera_frame)

    rospy.loginfo(
        "MID-360 mount base_link->lidar_link: "
        "xyz=[%.4f %.4f %.4f] m, rpy=[%.3f %.3f %.3f] deg",
        float(base_to_lidar["x"]), float(base_to_lidar["y"]),
        float(base_to_lidar["z"]), float(base_to_lidar["roll_deg"]),
        float(base_to_lidar["pitch_deg"]), float(base_to_lidar["yaw_deg"])
    )

    rospy.spin()
