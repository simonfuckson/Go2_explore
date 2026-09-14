#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#define PCL_NO_PRECOMPILE
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include "patchworkpp/patchworkpp.hpp"

namespace {

using Point = pcl::PointXYZI;

bool validSensorHeight(double sensor_height, double minimum_height,
                       double maximum_height) {
  return std::isfinite(sensor_height) && std::isfinite(minimum_height) &&
         std::isfinite(maximum_height) && minimum_height <= maximum_height &&
         sensor_height >= minimum_height && sensor_height <= maximum_height;
}

sensor_msgs::PointCloud2 toMessage(const pcl::PointCloud<Point>& cloud,
                                  const std_msgs::Header& header) {
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  output.header = header;
  return output;
}

class Go2PatchworkWrapper {
 public:
  Go2PatchworkWrapper() : nh_(), pnh_("~") {
    pnh_.param<std::string>("cloud_topic", cloud_topic_,
                            "/cloud_registered_terrain");
    pnh_.param<std::string>("expected_frame", expected_frame_,
                            "terrain_sensor");
    pnh_.param("sensor_height", sensor_height_, 0.51);
    pnh_.param("min_sensor_height", min_sensor_height_, 0.43);
    pnh_.param("max_sensor_height", max_sensor_height_, 0.59);
    pnh_.param("publish_input", publish_input_, false);

    if (!validSensorHeight(sensor_height_, min_sensor_height_,
                           max_sensor_height_)) {
      throw std::invalid_argument(
          "sensor_height must be finite and within the measured GO2 range [" +
          std::to_string(min_sensor_height_) + ", " +
          std::to_string(max_sensor_height_) + "] m");
    }
    if (cloud_topic_.empty() || expected_frame_.empty()) {
      throw std::invalid_argument("Patchwork topic/frame cannot be empty");
    }

    estimator_.reset(new PatchWorkpp<Point>(&pnh_));
    ground_publisher_ =
        pnh_.advertise<sensor_msgs::PointCloud2>("ground", 1, false);
    nonground_publisher_ =
        pnh_.advertise<sensor_msgs::PointCloud2>("nonground", 1, false);
    if (publish_input_) {
      input_publisher_ =
          pnh_.advertise<sensor_msgs::PointCloud2>("cloud", 1, false);
    }
    subscriber_ = nh_.subscribe(
        cloud_topic_, 1, &Go2PatchworkWrapper::cloudCallback, this,
        ros::TransportHints().tcpNoDelay(true));

    ROS_INFO("GO2 Patchwork++ wrapper: input=%s frame=%s height=%.3f m",
             cloud_topic_.c_str(), expected_frame_.c_str(), sensor_height_);
  }

 private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    if (message->header.stamp.isZero() ||
        message->header.frame_id != expected_frame_) {
      ROS_ERROR_THROTTLE(
          1.0, "Patchwork++ rejected cloud frame='%s' stamp=%.6f; expected %s",
          message->header.frame_id.c_str(), message->header.stamp.toSec(),
          expected_frame_.c_str());
      return;
    }

    pcl::PointCloud<Point> raw;
    pcl::PointCloud<Point> input;
    pcl::PointCloud<Point> ground;
    pcl::PointCloud<Point> nonground;
    pcl::fromROSMsg(*message, raw);
    std::vector<int> kept_indices;
    pcl::removeNaNFromPointCloud(raw, input, kept_indices);
    if (input.empty()) {
      ROS_WARN_THROTTLE(2.0, "Patchwork++ received an empty finite cloud");
      return;
    }
    input.header.frame_id = expected_frame_;

    double processing_seconds = 0.0;
    estimator_->estimate_ground(input, ground, nonground, processing_seconds);
    ground_publisher_.publish(toMessage(ground, message->header));
    nonground_publisher_.publish(toMessage(nonground, message->header));
    if (publish_input_) {
      input_publisher_.publish(toMessage(input, message->header));
    }

    ROS_INFO_THROTTLE(
        5.0,
        "Patchwork++ input=%zu ground=%zu nonground=%zu processing=%.1f ms",
        input.size(), ground.size(), nonground.size(),
        processing_seconds * 1000.0);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber subscriber_;
  ros::Publisher input_publisher_;
  ros::Publisher ground_publisher_;
  ros::Publisher nonground_publisher_;
  std::unique_ptr<PatchWorkpp<Point>> estimator_;
  std::string cloud_topic_;
  std::string expected_frame_;
  double sensor_height_ = 0.51;
  double min_sensor_height_ = 0.43;
  double max_sensor_height_ = 0.59;
  bool publish_input_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "go2_patchwork_wrapper");
  try {
    Go2PatchworkWrapper node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Cannot start GO2 Patchwork++ wrapper: %s", error.what());
    return 1;
  }
  return 0;
}
