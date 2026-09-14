#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <string>


class CloudFrameAdapter
{
public:
  CloudFrameAdapter()
      : nh_(),
        pnh_("~"),
        tf_buffer_(),
        tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>(
        "input_topic",
        input_topic_,
        "/cloud_registered");

    pnh_.param<std::string>(
        "output_topic",
        output_topic_,
        "/cloud_registered_odom");

    pnh_.param<std::string>(
        "target_frame",
        target_frame_,
        "odom");

    pnh_.param<double>(
        "tf_timeout_sec",
        tf_timeout_sec_,
        0.10);

    pnh_.param<bool>(
        "allow_latest_tf_fallback",
        allow_latest_tf_fallback_,
        true);

    pnh_.param<double>(
        "max_fallback_age_sec",
        max_fallback_age_sec_,
        0.20);

    pnh_.param<double>(
        "voxel_leaf_size",
        voxel_leaf_size_,
        0.0);

    pnh_.param<double>(
        "max_publish_rate_hz",
        max_publish_rate_hz_,
        0.0);


    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        output_topic_,
        2);


    sub_ = nh_.subscribe(
        input_topic_,
        2,
        &CloudFrameAdapter::cloudCallback,
        this);


    ROS_INFO(
        "go2_cloud_adapter started");

    ROS_INFO(
        "input_topic=%s",
        input_topic_.c_str());

    ROS_INFO(
        "output_topic=%s",
        output_topic_.c_str());

    ROS_INFO(
        "target_frame=%s",
        target_frame_.c_str());

    ROS_INFO(
        "tf_timeout_sec=%.3f",
        tf_timeout_sec_);

    ROS_INFO(
        "allow_latest_tf_fallback=%s",
        allow_latest_tf_fallback_
            ? "true"
            : "false");

    ROS_INFO(
        "max_fallback_age_sec=%.3f",
        max_fallback_age_sec_);

    ROS_INFO(
        "voxel_leaf_size=%.3f, max_publish_rate_hz=%.3f",
        voxel_leaf_size_,
        max_publish_rate_hz_);
  }


private:

  void publishCloud(const sensor_msgs::PointCloud2& cloud)
  {
    const ros::WallTime now = ros::WallTime::now();
    if (max_publish_rate_hz_ > 0.0 && !last_publish_wall_.isZero() &&
        (now - last_publish_wall_).toSec() < 1.0 / max_publish_rate_hz_)
    {
      return;
    }

    sensor_msgs::PointCloud2 output = cloud;
    if (voxel_leaf_size_ > 0.0)
    {
      pcl::PCLPointCloud2::Ptr input(new pcl::PCLPointCloud2());
      pcl::PCLPointCloud2 filtered;
      pcl_conversions::toPCL(cloud, *input);

      pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
      voxel_filter.setInputCloud(input);
      const float leaf = static_cast<float>(voxel_leaf_size_);
      voxel_filter.setLeafSize(leaf, leaf, leaf);
      voxel_filter.filter(filtered);
      pcl_conversions::fromPCL(filtered, output);
      output.header = cloud.header;
    }

    pub_.publish(output);
    last_publish_wall_ = now;
  }

  void cloudCallback(
      const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (msg->header.frame_id.empty())
    {
      ROS_WARN_THROTTLE(
          1.0,
          "Input cloud has empty frame_id");

      return;
    }


    if (msg->header.frame_id == target_frame_)
    {
      sensor_msgs::PointCloud2 out = *msg;

      out.header.frame_id =
          target_frame_;

      publishCloud(out);

      return;
    }


    geometry_msgs::TransformStamped tf_msg;

    bool used_latest_tf =
        false;


    // ============================================================
    // First attempt:
    // use the transform corresponding to the cloud timestamp.
    // ============================================================

    try
    {
      tf_msg =
          tf_buffer_.lookupTransform(
              target_frame_,
              msg->header.frame_id,
              msg->header.stamp,
              ros::Duration(
                  tf_timeout_sec_));
    }
    catch (const tf2::TransformException& e)
    {
      if (!allow_latest_tf_fallback_)
      {
        ROS_WARN_THROTTLE(
            1.0,
            "TF failed %s -> %s at cloud stamp %.6f: %s",
            msg->header.frame_id.c_str(),
            target_frame_.c_str(),
            msg->header.stamp.toSec(),
            e.what());

        return;
      }


      // ==========================================================
      // Fallback:
      // use the latest available TF.
      //
      // This is useful for the current FAST-LIO / public-odom
      // architecture, where cloud and public odom are generated
      // by asynchronous branches and can differ by tens of ms.
      // ==========================================================

      try
      {
        tf_msg =
            tf_buffer_.lookupTransform(
                target_frame_,
                msg->header.frame_id,
                ros::Time(0),
                ros::Duration(
                    tf_timeout_sec_));

        used_latest_tf =
            true;
      }
      catch (const tf2::TransformException& e_latest)
      {
        ROS_WARN_THROTTLE(
            1.0,
            "TF failed %s -> %s. "
            "Exact stamp failed and latest TF fallback failed: %s",
            msg->header.frame_id.c_str(),
            target_frame_.c_str(),
            e_latest.what());

        return;
      }
    }


    // ============================================================
    // Guard fallback against excessively stale TF.
    // ============================================================

    if (used_latest_tf &&
        !msg->header.stamp.isZero() &&
        !tf_msg.header.stamp.isZero())
    {
      const double dt =
          std::fabs(
              (tf_msg.header.stamp -
               msg->header.stamp).toSec());


      if (dt >
          max_fallback_age_sec_)
      {
        ROS_WARN_THROTTLE(
            1.0,
            "Latest TF fallback rejected: "
            "|tf_stamp-cloud_stamp|=%.3f s > %.3f s "
            "for %s -> %s",
            dt,
            max_fallback_age_sec_,
            msg->header.frame_id.c_str(),
            target_frame_.c_str());

        return;
      }


      ROS_WARN_THROTTLE(
          5.0,
          "Using latest TF fallback for %s -> %s: "
          "dt=%.3f s",
          msg->header.frame_id.c_str(),
          target_frame_.c_str(),
          dt);
    }


    sensor_msgs::PointCloud2 out;


    try
    {
      pcl_ros::transformPointCloud(
          target_frame_,
          tf_msg.transform,
          *msg,
          out);
    }
    catch (const std::exception& e)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "PointCloud transform failed: %s",
          e.what());

      return;
    }


    /*
     * The points are now represented in target_frame.
     *
     * Preserve the original cloud measurement time so downstream
     * modules know when the scan was acquired.
     */
    out.header.stamp =
        msg->header.stamp;

    out.header.frame_id =
        target_frame_;


    publishCloud(out);


    ROS_INFO_THROTTLE(
        5.0,
        "Cloud transformed %s -> %s, "
        "stamp=%.6f%s",
        msg->header.frame_id.c_str(),
        target_frame_.c_str(),
        msg->header.stamp.toSec(),
        used_latest_tf
            ? " [latest-TF fallback]"
            : "");
  }


private:

  ros::NodeHandle nh_;

  ros::NodeHandle pnh_;


  tf2_ros::Buffer
      tf_buffer_;

  tf2_ros::TransformListener
      tf_listener_;


  ros::Subscriber sub_;

  ros::Publisher pub_;


  std::string input_topic_;

  std::string output_topic_;

  std::string target_frame_;


  double tf_timeout_sec_;

  bool allow_latest_tf_fallback_;

  double max_fallback_age_sec_;

  double voxel_leaf_size_;

  double max_publish_rate_hz_;

  ros::WallTime last_publish_wall_;
};


int main(
    int argc,
    char** argv)
{
  ros::init(
      argc,
      argv,
      "go2_cloud_adapter");


  CloudFrameAdapter node;


  ros::spin();


  return 0;
}
