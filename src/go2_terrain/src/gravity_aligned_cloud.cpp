#include <cmath>
#include <stdexcept>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace {

double yawFromQuaternion(const geometry_msgs::Quaternion& message) {
  tf2::Quaternion orientation;
  tf2::fromMsg(message, orientation);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  return yaw;
}

class GravityAlignedCloud {
 public:
  GravityAlignedCloud()
      : nh_(), pnh_("~"), tf_buffer_(), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("input_topic", input_topic_,
                            "/cloud_registered_base");
    pnh_.param<std::string>("output_topic", output_topic_,
                            "/cloud_registered_terrain");
    pnh_.param<std::string>("fixed_frame", fixed_frame_, "odom");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("sensor_frame", sensor_frame_, "lidar_link");
    pnh_.param<std::string>("terrain_frame", terrain_frame_,
                            "terrain_sensor");
    pnh_.param("tf_timeout_sec", tf_timeout_sec_, 0.10);
    pnh_.param("input_timeout_sec", input_timeout_sec_, 0.60);
    pnh_.param("voxel_leaf_size", voxel_leaf_size_, 0.08);
    pnh_.param("max_publish_rate_hz", max_publish_rate_hz_, 0.0);

    if (input_topic_.empty() || output_topic_.empty() || fixed_frame_.empty() ||
        base_frame_.empty() || sensor_frame_.empty() || terrain_frame_.empty()) {
      throw std::invalid_argument("terrain cloud frame/topic parameters are empty");
    }
    if (fixed_frame_ == terrain_frame_) {
      throw std::invalid_argument("fixed_frame and terrain_frame must differ");
    }
    if (tf_timeout_sec_ <= 0.0 || input_timeout_sec_ <= 0.0 ||
        voxel_leaf_size_ < 0.0 || max_publish_rate_hz_ < 0.0) {
      throw std::invalid_argument("invalid terrain cloud timing/filter parameter");
    }

    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1, false);
    subscriber_ = nh_.subscribe(
        input_topic_, 1, &GravityAlignedCloud::cloudCallback, this,
        ros::TransportHints().tcpNoDelay(true));
    watchdog_ = nh_.createWallTimer(
        ros::WallDuration(0.25), &GravityAlignedCloud::watchdogCallback, this);

    ROS_INFO("Gravity-aligned terrain cloud: %s -> %s (%s)",
             input_topic_.c_str(), output_topic_.c_str(), terrain_frame_.c_str());
  }

 private:
  void watchdogCallback(const ros::WallTimerEvent&) {
    if (last_input_wall_.isZero()) {
      ROS_WARN_THROTTLE(5.0, "Waiting for terrain input cloud on %s",
                        input_topic_.c_str());
      return;
    }
    const double age = (ros::WallTime::now() - last_input_wall_).toSec();
    if (age > input_timeout_sec_) {
      ROS_ERROR_THROTTLE(2.0, "Terrain input cloud is stale: %.3f s", age);
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    last_input_wall_ = ros::WallTime::now();
    if (message->header.stamp.isZero() || message->header.frame_id.empty()) {
      ROS_ERROR_THROTTLE(2.0,
                         "Rejecting terrain cloud with zero stamp/empty frame");
      return;
    }

    const ros::WallTime now = ros::WallTime::now();
    if (max_publish_rate_hz_ > 0.0 && !last_publish_wall_.isZero() &&
        (now - last_publish_wall_).toSec() < 1.0 / max_publish_rate_hz_) {
      return;
    }

    geometry_msgs::TransformStamped fixed_from_base;
    geometry_msgs::TransformStamped fixed_from_sensor;
    geometry_msgs::TransformStamped fixed_from_input;
    try {
      const ros::Duration timeout(tf_timeout_sec_);
      fixed_from_base = tf_buffer_.lookupTransform(
          fixed_frame_, base_frame_, message->header.stamp, timeout);
      fixed_from_sensor = tf_buffer_.lookupTransform(
          fixed_frame_, sensor_frame_, message->header.stamp, timeout);
      fixed_from_input = tf_buffer_.lookupTransform(
          fixed_frame_, message->header.frame_id, message->header.stamp, timeout);
    } catch (const tf2::TransformException& error) {
      ROS_ERROR_THROTTLE(1.0,
                         "Exact-stamp terrain TF unavailable at %.6f: %s",
                         message->header.stamp.toSec(), error.what());
      return;
    }

    geometry_msgs::TransformStamped fixed_from_terrain;
    fixed_from_terrain.header.stamp = message->header.stamp;
    fixed_from_terrain.header.frame_id = fixed_frame_;
    fixed_from_terrain.child_frame_id = terrain_frame_;
    fixed_from_terrain.transform.translation =
        fixed_from_sensor.transform.translation;
    tf2::Quaternion yaw_only;
    yaw_only.setRPY(0.0, 0.0,
                    yawFromQuaternion(fixed_from_base.transform.rotation));
    yaw_only.normalize();
    fixed_from_terrain.transform.rotation = tf2::toMsg(yaw_only);

    tf2::Transform fixed_t_terrain;
    tf2::Transform fixed_t_input;
    tf2::fromMsg(fixed_from_terrain.transform, fixed_t_terrain);
    tf2::fromMsg(fixed_from_input.transform, fixed_t_input);
    const geometry_msgs::Transform terrain_from_input =
        tf2::toMsg(fixed_t_terrain.inverseTimes(fixed_t_input));

    sensor_msgs::PointCloud2 transformed;
    try {
      pcl_ros::transformPointCloud(terrain_frame_, terrain_from_input,
                                   *message, transformed);
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(1.0, "Terrain cloud transform failed: %s",
                         error.what());
      return;
    }
    transformed.header.stamp = message->header.stamp;
    transformed.header.frame_id = terrain_frame_;

    sensor_msgs::PointCloud2 output = transformed;
    if (voxel_leaf_size_ > 0.0) {
      pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2());
      pcl::PCLPointCloud2 pcl_output;
      pcl_conversions::toPCL(transformed, *pcl_input);
      pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
      const float leaf = static_cast<float>(voxel_leaf_size_);
      filter.setInputCloud(pcl_input);
      filter.setLeafSize(leaf, leaf, leaf);
      filter.filter(pcl_output);
      pcl_conversions::fromPCL(pcl_output, output);
      output.header = transformed.header;
    }

    broadcaster_.sendTransform(fixed_from_terrain);
    publisher_.publish(output);
    last_publish_wall_ = now;
    ROS_INFO_THROTTLE(5.0, "Terrain cloud published in %s without backlog",
                      terrain_frame_.c_str());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster broadcaster_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  ros::WallTimer watchdog_;
  std::string input_topic_;
  std::string output_topic_;
  std::string fixed_frame_;
  std::string base_frame_;
  std::string sensor_frame_;
  std::string terrain_frame_;
  double tf_timeout_sec_ = 0.10;
  double input_timeout_sec_ = 0.60;
  double voxel_leaf_size_ = 0.08;
  double max_publish_rate_hz_ = 0.0;
  ros::WallTime last_input_wall_;
  ros::WallTime last_publish_wall_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "go2_gravity_aligned_cloud");
  try {
    GravityAlignedCloud node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Cannot start gravity-aligned terrain cloud: %s", error.what());
    return 1;
  }
  return 0;
}
