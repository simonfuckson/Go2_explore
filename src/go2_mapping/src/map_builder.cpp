#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include <XmlRpcValue.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include "go2_mapping/dynamic_mapping.hpp"
#include "go2_mapping/map_storage.hpp"

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using go2_mapping::OccupancyVoxelState;
using go2_mapping::VoxelKey;
using go2_mapping::VoxelKeyHash;

struct MapVoxelState {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  double intensity_sum = 0.0;
  uint32_t sample_count = 0;
  VoxelKey occupancy_key{0, 0, 0};
  uint64_t occupancy_generation = 0;
  OccupancyVoxelState fine_evidence;
};

using MapVoxelTable =
    std::unordered_map<VoxelKey, MapVoxelState, VoxelKeyHash>;

struct PoseSample {
  ros::Time stamp;
  std::string frame_id;
  std::string child_frame_id;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

class PointcloudMapper {
 public:
  PointcloudMapper() : nh_(), pnh_("~") {
    loadParameters();

    static_scan_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(static_scan_topic_, 1);
    static_map_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(static_map_topic_, 1, true);
    if (publish_dynamic_) {
      dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(dynamic_topic_, 1);
    }

    pose_odom_sub_ = nh_.subscribe(pose_odom_topic_, 200,
                                   &PointcloudMapper::poseOdomCallback, this);
    trajectory_odom_sub_ =
        nh_.subscribe(trajectory_odom_topic_, 100,
                      &PointcloudMapper::trajectoryOdomCallback, this);
    // Mapping is CPU-heavy; never let stale registered scans build up in the
    // ROS transport queue. The small internal queue is only for odom matching.
    cloud_sub_ =
        nh_.subscribe(input_cloud_, 1, &PointcloudMapper::cloudCallback, this);
    save_service_ =
        pnh_.advertiseService("save_map", &PointcloudMapper::saveMap, this);
    reset_service_ =
        pnh_.advertiseService("reset_map", &PointcloudMapper::resetMap, this);
    if (publish_full_map_) {
      publish_timer_ = nh_.createTimer(ros::Duration(map_publish_period_),
                                       &PointcloudMapper::publishMapTimer,
                                       this);
    }
    if (autosave_period_ > 0.0) {
      autosave_timer_ = nh_.createTimer(ros::Duration(autosave_period_),
                                        &PointcloudMapper::autosaveTimer, this);
    }
    snapshot_status_timer_ = nh_.createTimer(
        ros::Duration(snapshot_status_period_),
        &PointcloudMapper::snapshotStatusTimer, this);

    ROS_INFO_STREAM("go2_pointcloud_mapper: cloud=" << input_cloud_
                    << ", 6DoF pose=" << pose_odom_topic_
                    << ", trajectory=" << trajectory_odom_topic_
                    << ", map=" << output_path_
                    << ", trajectory file=" << trajectory_output_path_);
    ROS_INFO_STREAM("MID360 ray origin base_link xyz=["
                    << base_to_lidar_translation_.transpose() << "] m");
    ROS_INFO_STREAM("Full accumulated-map publication is "
                    << (publish_full_map_ ? "enabled for validation"
                                          : "disabled"));
    pnh_.setParam("capacity_ok", true);
    pnh_.deleteParam("capacity_error");
    last_snapshot_status_ = "idle; no snapshot completed in this process";
    publishSnapshotStatus();
  }

  ~PointcloudMapper() {
    last_snapshot_status_ = "shutdown waiting for snapshot writer";
    publishSnapshotStatus();
    waitForBackgroundAutosave("shutdown");
    if (save_on_shutdown_ && dirty_) {
      std::string message;
      if (!saveMapToDisk(&message, "Shutdown")) {
        ROS_ERROR_STREAM("Final filtered-map save failed: " << message);
      } else {
        ROS_INFO_STREAM(message);
      }
    } else if (!dirty_ &&
               last_snapshot_status_ ==
                   "shutdown waiting for snapshot writer") {
      last_snapshot_status_ = "shutdown complete; snapshot is clean";
    } else if (!save_on_shutdown_ && dirty_) {
      last_snapshot_status_ =
          "shutdown complete with dirty snapshot; save_on_shutdown is false";
    }
    publishSnapshotStatus();
  }

 private:
  static double getNumeric(const XmlRpc::XmlRpcValue& config,
                           const std::string& key) {
    if (!config.hasMember(key)) {
      throw std::runtime_error("Missing MID360 calibration key: " + key);
    }
    const XmlRpc::XmlRpcValue& value = config[key];
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      return static_cast<int>(value);
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      return static_cast<double>(value);
    }
    throw std::runtime_error("MID360 calibration key is not numeric: " + key);
  }

  void loadMountTransform() {
    XmlRpc::XmlRpcValue mount;
    if (!nh_.getParam("/mid360_mount/base_link_to_lidar_link", mount)) {
      throw std::runtime_error(
          "Missing /mid360_mount/base_link_to_lidar_link; refusing to use an "
          "assumed ray origin");
    }
    const double x = getNumeric(mount, "x");
    const double y = getNumeric(mount, "y");
    const double z = getNumeric(mount, "z");
    const double degrees_to_radians = std::acos(-1.0) / 180.0;
    const double roll = getNumeric(mount, "roll_deg") * degrees_to_radians;
    const double pitch = getNumeric(mount, "pitch_deg") * degrees_to_radians;
    const double yaw = getNumeric(mount, "yaw_deg") * degrees_to_radians;
    const Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !rotation.coeffs().allFinite()) {
      throw std::runtime_error("MID360 calibration contains a non-finite value");
    }
    base_to_lidar_translation_ = Eigen::Vector3d(x, y, z);
    // Registered points are already expressed in odom. Mount rotation is
    // validated here, while only the translated sensor origin enters raycasts.
  }

  void loadParameters() {
    pnh_.param<std::string>("input_cloud", input_cloud_,
                            "/cloud_registered_odom");
    std::string legacy_input_odom;
    pnh_.param<std::string>("input_odom", legacy_input_odom, "/odom_nav");
    pnh_.param<std::string>("pose_odom_topic", pose_odom_topic_,
                            "/odom_robot");
    pnh_.param<std::string>("trajectory_odom_topic", trajectory_odom_topic_,
                            legacy_input_odom);
    pnh_.param<std::string>("static_scan_topic", static_scan_topic_,
                            "/go2_mapping/static_scan");
    pnh_.param<std::string>("static_map_topic", static_map_topic_,
                            "/go2_mapping/static_map_cloud");
    pnh_.param<std::string>("dynamic_topic", dynamic_topic_,
                            "/go2_mapping/dynamic_points");
    pnh_.param<std::string>("output_path", output_path_,
                            "/tmp/public_map.pcd");
    pnh_.param<std::string>("trajectory_output_path", trajectory_output_path_,
                            std::string());
    pnh_.param("min_range", min_range_, 0.5);
    pnh_.param("max_range", max_range_, 50.0);
    pnh_.param("scan_voxel_size", scan_voxel_size_, 0.05);
    pnh_.param("radius_filter/enable", radius_filter_enable_, true);
    pnh_.param("radius_filter/radius", radius_, 0.20);
    pnh_.param("radius_filter/min_neighbors", min_neighbors_, 2);
    pnh_.param("self_filter/enable", self_filter_enable_, false);
    pnh_.param("self_filter/min_x", self_min_x_, -0.55);
    pnh_.param("self_filter/max_x", self_max_x_, 0.55);
    pnh_.param("self_filter/min_y", self_min_y_, -0.45);
    pnh_.param("self_filter/max_y", self_max_y_, 0.45);
    pnh_.param("self_filter/min_z", self_min_z_, -0.40);
    pnh_.param("self_filter/max_z", self_max_z_, 0.25);
    pnh_.param("dynamic_filter/enable", dynamic_filter_enable_, true);
    pnh_.param("dynamic_filter/voxel_size", temporal_voxel_size_, 0.20);

    double hit_probability = 0.65;
    double miss_probability = 0.40;
    double occupied_probability = 0.75;
    double clearing_probability = 0.35;
    pnh_.param("dynamic_filter/hit_probability", hit_probability, 0.65);
    pnh_.param("dynamic_filter/miss_probability", miss_probability, 0.40);
    pnh_.param("dynamic_filter/occupied_probability", occupied_probability,
               0.75);
    pnh_.param("dynamic_filter/clearing_probability", clearing_probability,
               0.35);
    int min_hit_scans = 12;
    int fine_min_hit_scans = 6;
    double fine_min_observation_span = 0.50;
    double fine_min_hit_ratio = 0.50;
    int min_candidate_clear_miss_scans = 4;
    int min_clear_miss_scans = 12;
    pnh_.param("dynamic_filter/min_hit_scans", min_hit_scans, 12);
    pnh_.param("dynamic_filter/min_observation_span",
               dynamic_config_.min_observation_span, 2.0);
    pnh_.param("dynamic_filter/min_hit_ratio", dynamic_config_.min_hit_ratio,
               0.60);
    pnh_.param("dynamic_filter/fine_min_hit_scans", fine_min_hit_scans, 6);
    pnh_.param("dynamic_filter/fine_min_observation_span",
               fine_min_observation_span, 0.50);
    pnh_.param("dynamic_filter/fine_min_hit_ratio", fine_min_hit_ratio, 0.50);
    pnh_.param("dynamic_filter/min_candidate_clear_miss_scans",
               min_candidate_clear_miss_scans, 4);
    pnh_.param("dynamic_filter/min_candidate_clear_miss_span",
               dynamic_config_.min_candidate_clear_miss_span, 0.30);
    pnh_.param("dynamic_filter/min_clear_miss_scans", min_clear_miss_scans,
               12);
    pnh_.param("dynamic_filter/min_clear_miss_span",
               dynamic_config_.min_clear_miss_span, 1.20);
    pnh_.param("dynamic_filter/ray_stride", ray_stride_, 2);
    pnh_.param("dynamic_filter/max_clearing_range", max_clearing_range_,
               20.0);
    pnh_.param("dynamic_filter/ray_endpoint_margin", ray_endpoint_margin_,
               0.25);
    pnh_.param("dynamic_filter/candidate_timeout", candidate_timeout_, 6.0);
    pnh_.param("dynamic_filter/cleanup_period", cleanup_period_, 1.0);
    int max_voxels_param = 2000000;
    pnh_.param("dynamic_filter/max_voxels", max_voxels_param, 2000000);
    max_voxels_ = static_cast<std::size_t>(std::max(1, max_voxels_param));
    pnh_.param("map/voxel_size", map_voxel_size_, 0.05);
    int max_map_voxels_param = 5000000;
    pnh_.param("map/max_voxels", max_map_voxels_param, 5000000);
    max_map_voxels_ =
        static_cast<std::size_t>(std::max(1, max_map_voxels_param));
    int max_fine_candidate_voxels_param = 500000;
    pnh_.param("map/max_candidate_voxels",
               max_fine_candidate_voxels_param, 500000);
    max_fine_candidate_voxels_ = static_cast<std::size_t>(
        std::max(1, max_fine_candidate_voxels_param));
    pnh_.param("map/autosave_period", autosave_period_, 30.0);
    pnh_.param("map/status_period", snapshot_status_period_, 1.0);
    pnh_.param("map/save_on_shutdown", save_on_shutdown_, true);
    pnh_.param("map/trajectory_min_distance", trajectory_min_distance_,
               0.05);
    pnh_.param("map/publish_full_map", publish_full_map_, false);
    pnh_.param("map_publish_period", map_publish_period_, 2.0);
    pnh_.param("publish_dynamic_points", publish_dynamic_, false);
    pnh_.param("max_odom_age", max_odom_age_, 0.20);
    pnh_.param("odom_cache_duration", odom_cache_duration_, 2.0);
    int odom_cache_max_messages_param = 400;
    pnh_.param("odom_cache_max_messages", odom_cache_max_messages_param, 400);
    odom_cache_max_messages_ = static_cast<std::size_t>(
        std::max(2, odom_cache_max_messages_param));
    int cloud_queue_max_messages_param = 10;
    pnh_.param("cloud_queue_max_messages", cloud_queue_max_messages_param, 10);
    cloud_queue_max_messages_ = static_cast<std::size_t>(
        std::max(2, cloud_queue_max_messages_param));

    nh_.param<std::string>("/frames/odom", odom_frame_, "odom");
    nh_.param<std::string>("/frames/base_link", base_frame_, "base_link");
    loadMountTransform();

    if (trajectory_output_path_.empty()) {
      const boost::filesystem::path map_path(output_path_);
      trajectory_output_path_ =
          (map_path.parent_path() / "traversed_path_map.pcd").string();
    }
    if (boost::filesystem::absolute(output_path_) ==
        boost::filesystem::absolute(trajectory_output_path_)) {
      throw std::runtime_error(
          "Map and trajectory output paths must be different");
    }
    const boost::filesystem::path map_path =
        boost::filesystem::absolute(output_path_);
    const boost::filesystem::path trajectory_path =
        boost::filesystem::absolute(trajectory_output_path_);
    if (map_path.parent_path() != trajectory_path.parent_path() ||
        map_path.filename().string() != "public_map.pcd" ||
        trajectory_path.filename().string() != "traversed_path_map.pcd") {
      throw std::runtime_error(
          "Snapshot outputs must be sibling public_map.pcd and "
          "traversed_path_map.pcd files");
    }
    snapshot_manifest_path_ =
        (map_path.parent_path() / "mapping_snapshot.sha256").string();
    min_range_ = std::max(0.0, min_range_);
    max_range_ = std::max(min_range_, max_range_);
    scan_voxel_size_ = std::max(0.01, scan_voxel_size_);
    temporal_voxel_size_ = std::max(0.01, temporal_voxel_size_);
    map_voxel_size_ = std::max(0.01, map_voxel_size_);
    const double cells_per_axis = temporal_voxel_size_ / map_voxel_size_;
    const double rounded_cells_per_axis = std::round(cells_per_axis);
    if (dynamic_filter_enable_ &&
        (!std::isfinite(cells_per_axis) || cells_per_axis < 1.0 ||
         cells_per_axis > 4.0 ||
         std::fabs(cells_per_axis - rounded_cells_per_axis) > 1e-6)) {
      throw std::runtime_error(
          "dynamic_filter/voxel_size must be an integer 1..4 multiple of "
          "map/voxel_size so sparse fine-child masks fit in 64 bits");
    }
    fine_cells_per_coarse_ = dynamic_filter_enable_
                                 ? static_cast<int>(rounded_cells_per_axis)
                                 : 1;
    max_fine_candidate_voxels_ =
        std::min(max_fine_candidate_voxels_, max_map_voxels_);
    dynamic_config_.hit_log_odds =
        go2_mapping::probabilityToLogOdds(hit_probability);
    dynamic_config_.miss_log_odds =
        go2_mapping::probabilityToLogOdds(miss_probability);
    dynamic_config_.occupied_log_odds =
        go2_mapping::probabilityToLogOdds(occupied_probability);
    dynamic_config_.clearing_log_odds =
        go2_mapping::probabilityToLogOdds(clearing_probability);
    dynamic_config_.min_hit_scans =
        static_cast<uint32_t>(std::max(1, min_hit_scans));
    dynamic_config_.min_observation_span =
        std::max(0.0, dynamic_config_.min_observation_span);
    dynamic_config_.min_hit_ratio =
        std::max(0.0, std::min(1.0, dynamic_config_.min_hit_ratio));
    dynamic_config_.min_candidate_clear_miss_scans =
        static_cast<uint32_t>(std::max(1, min_candidate_clear_miss_scans));
    dynamic_config_.min_candidate_clear_miss_span =
        std::max(0.0, dynamic_config_.min_candidate_clear_miss_span);
    dynamic_config_.min_clear_miss_scans =
        static_cast<uint32_t>(std::max(1, min_clear_miss_scans));
    dynamic_config_.min_clear_miss_span =
        std::max(0.0, dynamic_config_.min_clear_miss_span);
    ray_stride_ = std::max(1, ray_stride_);
    max_clearing_range_ =
        std::max(temporal_voxel_size_, max_clearing_range_);
    ray_endpoint_margin_ =
        std::max(temporal_voxel_size_, ray_endpoint_margin_);
    candidate_timeout_ = std::max(0.1, candidate_timeout_);
    cleanup_period_ = std::max(0.1, cleanup_period_);
    trajectory_min_distance_ = std::max(0.001, trajectory_min_distance_);
    map_publish_period_ = std::max(0.1, map_publish_period_);
    autosave_period_ = std::max(0.0, autosave_period_);
    snapshot_status_period_ = std::max(0.2, snapshot_status_period_);
    max_odom_age_ = std::max(0.001, max_odom_age_);
    odom_cache_duration_ =
        std::max(2.0 * max_odom_age_, odom_cache_duration_);

    if (dynamic_config_.hit_log_odds <= 0.0 ||
        dynamic_config_.miss_log_odds >= 0.0 ||
        occupied_probability <= 0.5 || clearing_probability >= 0.5) {
      ROS_WARN("Invalid Bayesian probabilities; using 0.65/0.40/0.75/0.35");
      dynamic_config_.hit_log_odds =
          go2_mapping::probabilityToLogOdds(0.65);
      dynamic_config_.miss_log_odds =
          go2_mapping::probabilityToLogOdds(0.40);
      dynamic_config_.occupied_log_odds =
          go2_mapping::probabilityToLogOdds(0.75);
      dynamic_config_.clearing_log_odds =
          go2_mapping::probabilityToLogOdds(0.35);
    }

    // Coarse evidence remains deliberately conservative. Fine endpoints are
    // evaluated independently with a shorter gate so normal centimetre-scale
    // lidar jitter does not erase otherwise stable walls and floor returns.
    fine_dynamic_config_ = dynamic_config_;
    fine_dynamic_config_.min_hit_scans =
        static_cast<uint32_t>(std::max(1, fine_min_hit_scans));
    fine_dynamic_config_.min_observation_span =
        std::max(0.0, fine_min_observation_span);
    fine_dynamic_config_.min_hit_ratio =
        std::max(0.0, std::min(1.0, fine_min_hit_ratio));
    ROS_INFO_STREAM(
        "Dynamic-map retention gates: coarse="
        << dynamic_config_.min_hit_scans << " hits/"
        << dynamic_config_.min_observation_span << " s/"
        << dynamic_config_.min_hit_ratio << " ratio, fine="
        << fine_dynamic_config_.min_hit_scans << " hits/"
        << fine_dynamic_config_.min_observation_span << " s/"
        << fine_dynamic_config_.min_hit_ratio << " ratio, candidate clear="
        << dynamic_config_.min_candidate_clear_miss_scans << " misses/"
        << dynamic_config_.min_candidate_clear_miss_span
        << " s, confirmed clear=" << dynamic_config_.min_clear_miss_scans
        << " misses/" << dynamic_config_.min_clear_miss_span
        << " s, candidate timeout=" << candidate_timeout_
        << " s, endpoint margin=" << ray_endpoint_margin_ << " m");
  }

  static bool validPose(const nav_msgs::Odometry& odom) {
    const auto& p = odom.pose.pose.position;
    const auto& q = odom.pose.pose.orientation;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
           std::isfinite(q.w) &&
           (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 1e-12;
  }

  void poseOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    if (msg->header.stamp.isZero() || !validPose(*msg)) {
      ROS_WARN_THROTTLE(2.0, "Mapper discarded invalid 6DoF pose");
      return;
    }
    if (msg->header.frame_id != odom_frame_) {
      ROS_WARN_THROTTLE(2.0, "Mapper expected pose frame '%s', got '%s'",
                        odom_frame_.c_str(), msg->header.frame_id.c_str());
      return;
    }
    if (msg->child_frame_id != base_frame_) {
      ROS_WARN_THROTTLE(2.0, "Mapper expected pose child '%s', got '%s'",
                        base_frame_.c_str(), msg->child_frame_id.c_str());
      return;
    }

    PoseSample sample;
    sample.stamp = msg->header.stamp;
    sample.frame_id = msg->header.frame_id;
    sample.child_frame_id = msg->child_frame_id;
    sample.position = Eigen::Vector3d(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.position.z);
    sample.orientation = Eigen::Quaterniond(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    sample.orientation.normalize();

    const auto position = std::upper_bound(
        pose_odom_cache_.begin(), pose_odom_cache_.end(), sample.stamp,
        [](const ros::Time& stamp, const PoseSample& candidate) {
          return stamp < candidate.stamp;
        });
    pose_odom_cache_.insert(position, sample);
    const ros::Time newest_stamp = pose_odom_cache_.back().stamp;
    while (!pose_odom_cache_.empty() &&
           ((newest_stamp - pose_odom_cache_.front().stamp).toSec() >
                odom_cache_duration_ ||
            pose_odom_cache_.size() > odom_cache_max_messages_)) {
      pose_odom_cache_.pop_front();
    }
    processPendingClouds();
  }

  void trajectoryOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    if (msg->header.stamp.isZero() || !validPose(*msg) ||
        (!last_trajectory_stamp_.isZero() &&
         msg->header.stamp <= last_trajectory_stamp_)) {
      return;
    }
    if (msg->header.frame_id != odom_frame_) {
      ROS_WARN_THROTTLE(2.0, "Mapper expected trajectory frame '%s', got '%s'",
                        odom_frame_.c_str(), msg->header.frame_id.c_str());
      return;
    }
    const Eigen::Vector3d position(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
    if (!trajectory_.empty()) {
      const Point& previous = trajectory_.back();
      const Eigen::Vector3d previous_position(previous.x, previous.y,
                                               previous.z);
      if ((position - previous_position).norm() < trajectory_min_distance_) {
        last_trajectory_stamp_ = msg->header.stamp;
        return;
      }
    }
    if (trajectory_start_stamp_.isZero()) {
      trajectory_start_stamp_ = msg->header.stamp;
    }
    Point point;
    point.x = static_cast<float>(position.x());
    point.y = static_cast<float>(position.y());
    point.z = static_cast<float>(position.z());
    point.intensity = static_cast<float>(
        (msg->header.stamp - trajectory_start_stamp_).toSec());
    trajectory_.push_back(point);
    trajectory_frame_id_ = msg->header.frame_id;
    last_trajectory_stamp_ = msg->header.stamp;
    markDirty();
  }

  bool lookupRobotPose(const ros::Time& stamp, PoseSample* result) const {
    if (stamp.isZero() || pose_odom_cache_.empty()) {
      ROS_WARN_THROTTLE(2.0,
                        "Mapper is waiting for timestamped /odom_robot data");
      return false;
    }
    const auto upper = std::lower_bound(
        pose_odom_cache_.begin(), pose_odom_cache_.end(), stamp,
        [](const PoseSample& sample, const ros::Time& requested) {
          return sample.stamp < requested;
        });
    if (upper != pose_odom_cache_.end() && upper->stamp == stamp) {
      *result = *upper;
      return true;
    }

    if (upper != pose_odom_cache_.begin() && upper != pose_odom_cache_.end()) {
      const PoseSample& before = *(upper - 1);
      const PoseSample& after = *upper;
      if (before.frame_id == after.frame_id &&
          before.child_frame_id == after.child_frame_id &&
          go2_mapping::interpolatePose(
              before.stamp.toSec(), before.position, before.orientation,
              after.stamp.toSec(), after.position, after.orientation,
              stamp.toSec(), max_odom_age_, &result->position,
              &result->orientation)) {
        result->stamp = stamp;
        result->frame_id = before.frame_id;
        result->child_frame_id = before.child_frame_id;
        return true;
      }
    }

    const PoseSample* nearest = nullptr;
    if (upper == pose_odom_cache_.begin()) {
      nearest = &*upper;
    } else if (upper == pose_odom_cache_.end()) {
      nearest = &pose_odom_cache_.back();
    } else {
      const PoseSample& before = *(upper - 1);
      nearest = (stamp - before.stamp) <= (upper->stamp - stamp) ? &before
                                                                 : &*upper;
    }
    const double age = std::fabs((stamp - nearest->stamp).toSec());
    if (age > max_odom_age_) {
      ROS_WARN_THROTTLE(2.0,
                        "Mapper skipped a scan: nearest /odom_robot differs "
                        "by %.3f s",
                        age);
      return false;
    }
    *result = *nearest;
    return true;
  }

  VoxelKey pointVoxelKey(const Point& point, double voxel_size) const {
    return go2_mapping::voxelKey(
        Eigen::Vector3d(point.x, point.y, point.z), voxel_size);
  }

  void markDirty() {
    dirty_ = true;
    ++mutation_sequence_;
  }

  bool allowNewVoxel(std::size_t current_size, std::size_t maximum_size,
                     const char* collection) {
    const bool was_latched = capacity_gate_.latched();
    if (capacity_gate_.allowNewVoxel(current_size, maximum_size)) {
      return true;
    }
    if (!was_latched) {
      capacity_error_ = std::string(collection) + " reached its configured " +
                        "limit; rejecting all new voxels until reset_map";
      pnh_.setParam("capacity_ok", false);
      pnh_.setParam("capacity_error", capacity_error_);
      last_snapshot_status_ =
          "capacity fail-closed; waiting for snapshot writer";
      publishSnapshotStatus();
      waitForBackgroundAutosave("Capacity gate wait");
      std::string invalidation_error;
      if (!go2_mapping::invalidateSnapshotManifest(snapshot_manifest_path_,
                                                    &invalidation_error)) {
        capacity_error_ += "; snapshot invalidation failed: " +
                           invalidation_error;
        pnh_.setParam("capacity_error", capacity_error_);
        last_snapshot_status_ =
            "capacity fail-closed; snapshot invalidation failed: " +
            invalidation_error;
      } else {
        last_snapshot_status_ =
            "capacity fail-closed; committed snapshot invalidated";
      }
      publishSnapshotStatus();
    }
    ROS_ERROR_THROTTLE(
        5.0, "Mapper capacity fail-closed: %s. Existing voxels remain "
             "updateable, but snapshots are refused until reset_map.",
        capacity_error_.c_str());
    return false;
  }

  OccupancyVoxelState* getOrCreateOccupancy(const VoxelKey& key) {
    auto existing = temporal_voxels_.find(key);
    if (existing != temporal_voxels_.end()) {
      return &existing->second;
    }
    if (!allowNewVoxel(temporal_voxels_.size(), max_voxels_,
                       "dynamic_filter/max_voxels")) {
      return nullptr;
    }
    const auto inserted =
        temporal_voxels_.emplace(key, OccupancyVoxelState());
    inserted.first->second.generation = next_occupancy_generation_++;
    coarse_candidate_keys_.insert(key);
    return &inserted.first->second;
  }

  bool registerFineChild(const VoxelKey& parent, const VoxelKey& fine) {
    uint32_t child_index = 0;
    if (!go2_mapping::fineChildLinearIndex(
            parent, fine, fine_cells_per_coarse_, &child_index)) {
      ROS_ERROR_THROTTLE(
          2.0, "Mapper rejected a fine voxel outside its coarse parent");
      return false;
    }
    fine_child_masks_[parent] |= (uint64_t{1} << child_index);
    return true;
  }

  void unregisterFineChild(const VoxelKey& parent, const VoxelKey& fine) {
    uint32_t child_index = 0;
    if (!go2_mapping::fineChildLinearIndex(
            parent, fine, fine_cells_per_coarse_, &child_index)) {
      return;
    }
    auto mask_it = fine_child_masks_.find(parent);
    if (mask_it == fine_child_masks_.end()) {
      return;
    }
    mask_it->second &= ~(uint64_t{1} << child_index);
    if (mask_it->second == 0) {
      fine_child_masks_.erase(mask_it);
    }
  }

  MapVoxelTable::iterator eraseFineVoxel(MapVoxelTable::iterator it) {
    const auto parent_it = temporal_voxels_.find(it->second.occupancy_key);
    const OccupancyVoxelState* parent =
        parent_it == temporal_voxels_.end() ? nullptr : &parent_it->second;
    const bool was_public =
        !dynamic_filter_enable_
            ? it->second.sample_count > 0
            : go2_mapping::fineVoxelPublishable(
                  it->second.occupancy_generation, parent,
                  it->second.fine_evidence, it->second.sample_count);
    const bool was_candidate =
        dynamic_filter_enable_ && !it->second.fine_evidence.confirmed_static;
    unregisterFineChild(it->second.occupancy_key, it->first);
    if (was_candidate) {
      fine_candidate_keys_.erase(it->first);
    }
    const auto next = map_voxels_.erase(it);
    if (was_public) {
      ++fine_cleared_voxels_;
      markDirty();
    } else {
      ++fine_candidate_evictions_;
    }
    return next;
  }

  void eraseFineChildrenForParent(const VoxelKey& parent) {
    const auto mask_it = fine_child_masks_.find(parent);
    if (mask_it == fine_child_masks_.end()) {
      return;
    }
    const uint64_t children = mask_it->second;
    for (uint32_t child_index = 0; child_index < 64; ++child_index) {
      if ((children & (uint64_t{1} << child_index)) == 0) {
        continue;
      }
      VoxelKey fine;
      if (!go2_mapping::fineChildKey(parent, fine_cells_per_coarse_,
                                     child_index, &fine)) {
        continue;
      }
      const auto fine_it = map_voxels_.find(fine);
      if (fine_it != map_voxels_.end()) {
        eraseFineVoxel(fine_it);
      }
    }
    fine_child_masks_.erase(parent);
  }

  MapVoxelTable::iterator getOrCreateFineVoxel(
      const VoxelKey& fine, const VoxelKey& parent,
      uint64_t parent_generation) {
    auto existing = map_voxels_.find(fine);
    if (existing != map_voxels_.end() &&
        (existing->second.occupancy_key != parent ||
         existing->second.occupancy_generation != parent_generation)) {
      eraseFineVoxel(existing);
      existing = map_voxels_.end();
    }
    if (existing != map_voxels_.end()) {
      return existing;
    }

    uint32_t child_index = 0;
    if (!go2_mapping::fineChildLinearIndex(
            parent, fine, fine_cells_per_coarse_, &child_index)) {
      ROS_ERROR_THROTTLE(
          2.0, "Mapper rejected an inconsistent coarse/fine voxel pair");
      return map_voxels_.end();
    }
    if (!allowNewVoxel(map_voxels_.size(), max_map_voxels_,
                       "map/max_voxels")) {
      return map_voxels_.end();
    }
    if (!go2_mapping::fineCandidateCapacityAvailable(
            fine_candidate_keys_.size(), max_fine_candidate_voxels_)) {
      ++fine_candidate_rejections_;
      ROS_WARN_THROTTLE(
          2.0,
          "Mapper fine-candidate budget reached (%zu); rejecting transient "
          "0.05 m candidates while preserving confirmed map voxels",
          max_fine_candidate_voxels_);
      return map_voxels_.end();
    }

    MapVoxelState state;
    state.occupancy_key = parent;
    state.occupancy_generation = parent_generation;
    const auto inserted = map_voxels_.emplace(fine, std::move(state));
    if (!inserted.second) {
      return inserted.first;
    }
    if (!registerFineChild(parent, fine)) {
      map_voxels_.erase(inserted.first);
      return map_voxels_.end();
    }
    fine_candidate_keys_.insert(fine);
    return inserted.first;
  }

  void updateMiss(const VoxelKey& key, double stamp) {
    auto it = temporal_voxels_.find(key);
    if (it == temporal_voxels_.end()) {
      return;
    }
    const bool clear_confirmed = go2_mapping::recordMiss(
        &it->second, scan_sequence_, stamp, dynamic_config_);
    const bool clear_candidate = go2_mapping::unconfirmedEvidenceCleared(
        it->second, dynamic_config_);
    if (clear_confirmed || clear_candidate) {
      if (clear_candidate) {
        ++coarse_candidate_miss_evictions_;
      }
      coarse_candidate_keys_.erase(key);
      eraseFineChildrenForParent(key);
      temporal_voxels_.erase(it);
      ++bayesian_cleared_voxels_;
    }
  }

  void updateFineMiss(const VoxelKey& key, double stamp) {
    auto it = map_voxels_.find(key);
    if (it == map_voxels_.end()) {
      return;
    }
    MapVoxelState& state = it->second;
    const auto parent_it = temporal_voxels_.find(state.occupancy_key);
    const OccupancyVoxelState* parent =
        parent_it == temporal_voxels_.end() ? nullptr : &parent_it->second;
    const bool parent_valid =
        go2_mapping::generationMatches(state.occupancy_generation, parent);
    if (!parent_valid) {
      ++fine_parent_invalid_evictions_;
      eraseFineVoxel(it);
      return;
    }

    const bool clear_confirmed = go2_mapping::recordMiss(
        &state.fine_evidence, scan_sequence_, stamp, fine_dynamic_config_);
    const bool clear_candidate = go2_mapping::unconfirmedEvidenceCleared(
        state.fine_evidence, fine_dynamic_config_);
    if (clear_confirmed || clear_candidate) {
      if (clear_candidate) {
        ++fine_candidate_miss_evictions_;
      }
      eraseFineVoxel(it);
    }
  }

  void updateFreeSpace(
      const Eigen::Vector3d& sensor_position, const Cloud& cloud,
      const std::unordered_set<VoxelKey, VoxelKeyHash>& occupied_coarse,
      const std::unordered_set<VoxelKey, VoxelKeyHash>& occupied_fine,
      const ros::Time& stamp) {
    if (!dynamic_filter_enable_ || cloud.empty()) {
      return;
    }
    for (std::size_t index = 0; index < cloud.size();
         index += static_cast<std::size_t>(ray_stride_)) {
      const Point& endpoint_point = cloud.points[index];
      const Eigen::Vector3d endpoint(endpoint_point.x, endpoint_point.y,
                                     endpoint_point.z);
      const Eigen::Vector3d delta = endpoint - sensor_position;
      const double full_range = delta.norm();
      if (full_range <= ray_endpoint_margin_) {
        continue;
      }
      const double clear_until =
          std::min(max_clearing_range_, full_range - ray_endpoint_margin_);
      if (clear_until <= temporal_voxel_size_) {
        continue;
      }
      const Eigen::Vector3d clear_endpoint =
          sensor_position + delta * (clear_until / full_range);
      const double stamp_seconds = stamp.toSec();
      go2_mapping::visitRayDda(
          sensor_position, clear_endpoint, temporal_voxel_size_,
          [this, &occupied_coarse, &occupied_fine, &sensor_position,
           &clear_endpoint, stamp_seconds](const VoxelKey& key) {
            if (occupied_coarse.find(key) == occupied_coarse.end()) {
              updateMiss(key, stamp_seconds);
            }

            // A 20 m ray would visit about 400 cells at 0.05 m. Instead, the
            // coarse DDA consults a compact mask and tests only fine children
            // that have actually received endpoint evidence. Rays never grow
            // the fine map.
            const auto mask_it = fine_child_masks_.find(key);
            if (mask_it == fine_child_masks_.end()) {
              return;
            }
            const uint64_t children = mask_it->second;
            for (uint32_t child_index = 0; child_index < 64;
                 ++child_index) {
              if ((children & (uint64_t{1} << child_index)) == 0) {
                continue;
              }
              VoxelKey fine;
              if (!go2_mapping::fineChildKey(
                      key, fine_cells_per_coarse_, child_index, &fine) ||
                  occupied_fine.find(fine) != occupied_fine.end() ||
                  !go2_mapping::segmentIntersectsVoxelInterior(
                      sensor_position, clear_endpoint, fine,
                      map_voxel_size_)) {
                continue;
              }
              updateFineMiss(fine, stamp_seconds);
            }
          });
    }
  }

  Cloud::Ptr prefilter(const Cloud::ConstPtr& input,
                       const Eigen::Vector3d& base_position,
                       const Eigen::Quaterniond& base_orientation,
                       const Eigen::Vector3d& sensor_position) const {
    Cloud::Ptr valid(new Cloud);
    valid->reserve(input->size());
    const double min_range_sq = min_range_ * min_range_;
    const double max_range_sq = max_range_ * max_range_;
    const Eigen::Quaterniond odom_to_body = base_orientation.conjugate();

    for (const Point& point : input->points) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      const Eigen::Vector3d world_point(point.x, point.y, point.z);
      const double range_sq = (world_point - sensor_position).squaredNorm();
      if (range_sq < min_range_sq || range_sq > max_range_sq) {
        continue;
      }
      const Eigen::Vector3d body_point =
          odom_to_body * (world_point - base_position);
      if (self_filter_enable_ && body_point.x() >= self_min_x_ &&
          body_point.x() <= self_max_x_ && body_point.y() >= self_min_y_ &&
          body_point.y() <= self_max_y_ && body_point.z() >= self_min_z_ &&
          body_point.z() <= self_max_z_) {
        continue;
      }
      valid->push_back(point);
    }

    Cloud::Ptr downsampled(new Cloud);
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);
    voxel.setInputCloud(valid);
    voxel.filter(*downsampled);

    if (!radius_filter_enable_ || downsampled->empty()) {
      return downsampled;
    }
    Cloud::Ptr filtered(new Cloud);
    pcl::RadiusOutlierRemoval<Point> radius_filter;
    radius_filter.setInputCloud(downsampled);
    radius_filter.setRadiusSearch(radius_);
    radius_filter.setMinNeighborsInRadius(min_neighbors_);
    radius_filter.filter(*filtered);
    return filtered;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (msg->header.stamp.isZero()) {
      ROS_WARN_THROTTLE(2.0, "Mapper discarded point cloud with zero stamp");
      return;
    }
    if (msg->header.frame_id != odom_frame_) {
      ROS_WARN_THROTTLE(2.0, "Mapper expected cloud frame '%s', got '%s'",
                        odom_frame_.c_str(), msg->header.frame_id.c_str());
      return;
    }
    const auto position = std::upper_bound(
        pending_clouds_.begin(), pending_clouds_.end(), msg->header.stamp,
        [](const ros::Time& stamp,
           const sensor_msgs::PointCloud2ConstPtr& candidate) {
          return stamp < candidate->header.stamp;
        });
    pending_clouds_.insert(position, msg);
    if (pending_clouds_.size() > cloud_queue_max_messages_) {
      ROS_WARN_THROTTLE(
          2.0, "Mapper point-cloud synchronization queue overflow; dropping "
               "oldest scan");
      pending_clouds_.pop_front();
    }
    processPendingClouds();
  }

  void processPendingClouds() {
    while (!pending_clouds_.empty() && !pose_odom_cache_.empty()) {
      const sensor_msgs::PointCloud2ConstPtr msg = pending_clouds_.front();
      // Wait for the bracketing odometry sample instead of silently using the
      // previous pose when the cloud callback wins the scheduling race.
      if (pose_odom_cache_.back().stamp < msg->header.stamp) {
        return;
      }
      pending_clouds_.pop_front();
      PoseSample robot_pose;
      if (!lookupRobotPose(msg->header.stamp, &robot_pose)) {
        continue;
      }
      processCloud(msg, robot_pose);
    }
  }

  void processCloud(const sensor_msgs::PointCloud2ConstPtr& msg,
                    const PoseSample& robot_pose) {
    if (!robot_pose.frame_id.empty() && !msg->header.frame_id.empty() &&
        robot_pose.frame_id != msg->header.frame_id) {
      ROS_ERROR_THROTTLE(2.0,
                         "Mapper frame mismatch: cloud='%s', pose='%s'",
                         msg->header.frame_id.c_str(),
                         robot_pose.frame_id.c_str());
      return;
    }
    const Eigen::Vector3d sensor_position = go2_mapping::sensorOrigin(
        robot_pose.position, robot_pose.orientation,
        base_to_lidar_translation_);

    Cloud::Ptr input(new Cloud);
    pcl::fromROSMsg(*msg, *input);
    Cloud::Ptr filtered = prefilter(input, robot_pose.position,
                                    robot_pose.orientation, sensor_position);
    Cloud static_scan;
    Cloud dynamic_scan;
    static_scan.reserve(filtered->size());
    if (publish_dynamic_) {
      dynamic_scan.reserve(filtered->size());
    }
    ++scan_sequence_;

    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_coarse;
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_fine;
    occupied_coarse.reserve(filtered->size());
    occupied_fine.reserve(filtered->size());
    for (const Point& point : filtered->points) {
      occupied_coarse.insert(pointVoxelKey(point, temporal_voxel_size_));
      occupied_fine.insert(pointVoxelKey(point, map_voxel_size_));
    }

    bool mapping_state_updated = false;
    for (const Point& point : filtered->points) {
      const VoxelKey key = pointVoxelKey(point, temporal_voxel_size_);
      OccupancyVoxelState* occupancy = nullptr;
      if (dynamic_filter_enable_) {
        occupancy = getOrCreateOccupancy(key);
        if (occupancy != nullptr &&
            go2_mapping::recordHit(occupancy, scan_sequence_,
                                   msg->header.stamp.toSec(), dynamic_config_)) {
          ++promoted_static_voxels_;
          coarse_candidate_keys_.erase(key);
          // Every confirmed fine child with cached geometry becomes
          // publishable when its parent confirms, even if that child is not
          // hit again in this scan.
          mapping_state_updated = true;
        }
      }
      if (dynamic_filter_enable_ && occupancy == nullptr) {
        if (publish_dynamic_) {
          dynamic_scan.push_back(point);
        }
        continue;
      }

      const VoxelKey map_key = pointVoxelKey(point, map_voxel_size_);
      MapVoxelTable::iterator map_it;
      if (dynamic_filter_enable_) {
        // Fine evidence begins with the first endpoint, in parallel with its
        // coarse parent. Public-map eligibility still requires both levels to
        // pass independent gates. The coarse gate remains conservative while
        // the fine gate tolerates normal endpoint jitter between 5 cm cells.
        map_it = getOrCreateFineVoxel(map_key, key, occupancy->generation);
        if (map_it == map_voxels_.end()) {
          if (publish_dynamic_) {
            dynamic_scan.push_back(point);
          }
          continue;
        }
      } else {
        map_it = map_voxels_.find(map_key);
        if (map_it == map_voxels_.end()) {
          if (!allowNewVoxel(map_voxels_.size(), max_map_voxels_,
                             "map/max_voxels")) {
            continue;
          }
          map_it = map_voxels_.emplace(map_key, MapVoxelState()).first;
        }
      }
      MapVoxelState& map_state = map_it->second;

      bool fine_evidence_ready = true;
      bool fine_public = true;
      if (dynamic_filter_enable_) {
        if (go2_mapping::recordHit(&map_state.fine_evidence, scan_sequence_,
                                   msg->header.stamp.toSec(),
                                   fine_dynamic_config_)) {
          ++fine_promoted_voxels_;
          if (fine_candidate_keys_.erase(map_key) == 0) {
            ROS_ERROR_THROTTLE(
                2.0, "Mapper fine-candidate promotion accounting underflow "
                     "prevented");
          }
        }
        fine_evidence_ready =
            go2_mapping::fineVoxelReadyToCache(map_state.fine_evidence);
        fine_public = go2_mapping::fineVoxelConfirmedForParent(
            map_state.occupancy_generation, occupancy,
            map_state.fine_evidence);
      }
      if (!fine_evidence_ready) {
        if (publish_dynamic_) {
          dynamic_scan.push_back(point);
        }
        continue;
      }

      // Preserve representative geometry once the 5 cm endpoint itself is
      // stable. Public topics and snapshots remain guarded by the confirmed
      // coarse parent and its generation.
      map_state.sum += Eigen::Vector3d(point.x, point.y, point.z);
      map_state.intensity_sum += point.intensity;
      ++map_state.sample_count;
      mapping_state_updated = true;
      if (!fine_public) {
        if (publish_dynamic_) {
          dynamic_scan.push_back(point);
        }
        continue;
      }
      static_scan.push_back(point);
    }
    if (mapping_state_updated) {
      markDirty();
    }

    updateFreeSpace(sensor_position, *filtered, occupied_coarse, occupied_fine,
                    msg->header.stamp);

    frame_id_ = msg->header.frame_id;
    last_cloud_stamp_ = msg->header.stamp;
    publishCloud(static_scan, static_scan_pub_, msg->header);
    if (publish_dynamic_) {
      publishCloud(dynamic_scan, dynamic_pub_, msg->header);
    }
    maybeCleanup(msg->header.stamp);

    ROS_INFO_THROTTLE(
        5.0,
        "Mapper: input=%zu filtered=%zu static_scan=%zu map=%zu occupancy=%zu "
        "coarse_promoted=%llu coarse_cleared=%llu fine_promoted=%llu "
        "fine_cleared=%llu fine_candidates=%zu fine_candidates_evicted=%llu "
        "fine_candidates_rejected=%llu trajectory=%zu",
        input->size(), filtered->size(), static_scan.size(),
        map_voxels_.size(), temporal_voxels_.size(),
        static_cast<unsigned long long>(promoted_static_voxels_),
        static_cast<unsigned long long>(bayesian_cleared_voxels_),
        static_cast<unsigned long long>(fine_promoted_voxels_),
        static_cast<unsigned long long>(fine_cleared_voxels_),
        fine_candidate_keys_.size(),
        static_cast<unsigned long long>(fine_candidate_evictions_),
        static_cast<unsigned long long>(fine_candidate_rejections_),
        trajectory_.size());
    ROS_INFO_THROTTLE(
        5.0,
        "Mapper candidate evictions: coarse_miss=%llu coarse_timeout=%llu "
        "fine_miss=%llu fine_timeout=%llu fine_parent_invalid=%llu",
        static_cast<unsigned long long>(coarse_candidate_miss_evictions_),
        static_cast<unsigned long long>(coarse_candidate_timeout_evictions_),
        static_cast<unsigned long long>(fine_candidate_miss_evictions_),
        static_cast<unsigned long long>(fine_candidate_timeout_evictions_),
        static_cast<unsigned long long>(fine_parent_invalid_evictions_));
  }

  void publishCloud(const Cloud& cloud, const ros::Publisher& publisher,
                    const std_msgs::Header& header) const {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header = header;
    publisher.publish(output);
  }

  Cloud buildMapCloud() {
    Cloud map;
    map.reserve(map_voxels_.size());
    for (auto it = map_voxels_.begin(); it != map_voxels_.end();) {
      const MapVoxelState& state = it->second;
      if (dynamic_filter_enable_) {
        const auto occupancy_it = temporal_voxels_.find(state.occupancy_key);
        const OccupancyVoxelState* occupancy =
            occupancy_it == temporal_voxels_.end() ? nullptr
                                                    : &occupancy_it->second;
        if (!go2_mapping::generationMatches(state.occupancy_generation,
                                            occupancy)) {
          it = eraseFineVoxel(it);
          continue;
        }
        if (!go2_mapping::fineVoxelPublishable(
                state.occupancy_generation, occupancy,
                state.fine_evidence, state.sample_count)) {
          ++it;
          continue;
        }
      }
      if (!dynamic_filter_enable_ && state.sample_count == 0) {
        it = dynamic_filter_enable_ ? eraseFineVoxel(it)
                                    : map_voxels_.erase(it);
        continue;
      }
      Point point;
      const Eigen::Vector3d mean = state.sum / state.sample_count;
      point.x = static_cast<float>(mean.x());
      point.y = static_cast<float>(mean.y());
      point.z = static_cast<float>(mean.z());
      point.intensity =
          static_cast<float>(state.intensity_sum / state.sample_count);
      map.push_back(point);
      ++it;
    }
    map.width = static_cast<uint32_t>(map.size());
    map.height = 1;
    map.is_dense = true;
    return map;
  }

  void maybeCleanup(const ros::Time& now) {
    if (!last_cleanup_.isZero() &&
        (now - last_cleanup_).toSec() < cleanup_period_) {
      return;
    }
    last_cleanup_ = now;
    for (auto candidate_it = coarse_candidate_keys_.begin();
         candidate_it != coarse_candidate_keys_.end();) {
      const VoxelKey key = *candidate_it;
      const auto occupancy_it = temporal_voxels_.find(key);
      if (occupancy_it == temporal_voxels_.end()) {
        candidate_it = coarse_candidate_keys_.erase(candidate_it);
        eraseFineChildrenForParent(key);
        continue;
      }
      if (occupancy_it->second.confirmed_static) {
        candidate_it = coarse_candidate_keys_.erase(candidate_it);
        continue;
      }
      const bool expired =
          std::isfinite(occupancy_it->second.last_seen_time) &&
          now.toSec() - occupancy_it->second.last_seen_time >
              candidate_timeout_;
      if (expired) {
        ++coarse_candidate_timeout_evictions_;
        candidate_it = coarse_candidate_keys_.erase(candidate_it);
        eraseFineChildrenForParent(key);
        temporal_voxels_.erase(occupancy_it);
      } else {
        ++candidate_it;
      }
    }

    // Only candidates can expire by time. Keeping an explicit index avoids a
    // full scan of the multi-million-voxel confirmed map every second.
    for (auto candidate_it = fine_candidate_keys_.begin();
         candidate_it != fine_candidate_keys_.end();) {
      const VoxelKey key = *candidate_it;
      const auto map_it = map_voxels_.find(key);
      if (map_it == map_voxels_.end() ||
          map_it->second.fine_evidence.confirmed_static) {
        candidate_it = fine_candidate_keys_.erase(candidate_it);
        continue;
      }
      const MapVoxelState& state = map_it->second;
      const auto parent_it = temporal_voxels_.find(state.occupancy_key);
      const OccupancyVoxelState* parent =
          parent_it == temporal_voxels_.end() ? nullptr : &parent_it->second;
      const bool parent_valid = go2_mapping::generationMatches(
          state.occupancy_generation, parent);
      const bool candidate_expired = go2_mapping::evidenceCandidateExpired(
          state.fine_evidence, now.toSec(), candidate_timeout_);
      if (!parent_valid || candidate_expired) {
        if (!parent_valid) {
          ++fine_parent_invalid_evictions_;
        } else {
          ++fine_candidate_timeout_evictions_;
        }
        // Advance before eraseFineVoxel removes this key from the candidate
        // index. unordered_set erasure preserves all other iterators.
        ++candidate_it;
        eraseFineVoxel(map_it);
      } else {
        ++candidate_it;
      }
    }
    if (capacity_gate_.latched()) {
      ROS_ERROR_THROTTLE(
          5.0, "Mapper remains fail-closed for new voxels: %s",
          capacity_error_.c_str());
    }
  }

  void publishMapTimer(const ros::TimerEvent&) {
    if (frame_id_.empty()) {
      return;
    }
    const Cloud map = buildMapCloud();
    std_msgs::Header header;
    header.frame_id = frame_id_;
    header.stamp = last_cloud_stamp_;
    publishCloud(map, static_map_pub_, header);
  }

  bool saveMap(std_srvs::Trigger::Request&,
               std_srvs::Trigger::Response& response) {
    response.success = saveMapToDisk(&response.message, "Manual");
    if (response.success) {
      ROS_INFO_STREAM(response.message);
    }
    return true;
  }

  bool buildSnapshot(Cloud* map, Cloud* trajectory, std::string* message) {
    if (capacity_gate_.latched()) {
      *message = "Mapping snapshot refused after capacity fail-closed: " +
                 capacity_error_ + "; call reset_map and remap the area";
      return false;
    }
    *map = buildMapCloud();
    if (map->empty()) {
      *message = "No confirmed static map points are available";
      return false;
    }
    if (trajectory_.empty()) {
      *message = "No /odom_nav trajectory points are available";
      return false;
    }
    if (!frame_id_.empty() && !trajectory_frame_id_.empty() &&
        frame_id_ != trajectory_frame_id_) {
      *message = "Map/trajectory frame mismatch: map='" + frame_id_ +
                 "', trajectory='" + trajectory_frame_id_ + "'";
      return false;
    }
    *trajectory = trajectory_;
    trajectory->width = static_cast<uint32_t>(trajectory->size());
    trajectory->height = 1;
    trajectory->is_dense = true;
    return true;
  }

  void applySnapshotResult(const go2_mapping::SnapshotWriteResult& result,
                           const char* context) {
    if (!result.had_work) {
      return;
    }
    if (!result.success) {
      last_snapshot_status_ = std::string(context) + " snapshot failed: " +
                              result.message;
      publishSnapshotStatus();
      ROS_ERROR_STREAM(context << " mapping snapshot failed: "
                               << result.message);
      return;
    }
    if (result.mutation_sequence == mutation_sequence_) {
      dirty_ = false;
    }
    last_snapshot_status_ =
        std::string(context) + " snapshot committed at mutation " +
        std::to_string(result.mutation_sequence);
    publishSnapshotStatus();
    ROS_INFO_STREAM(context << " mapping snapshot completed at mutation "
                            << result.mutation_sequence << ": "
                            << result.message);
  }

  void reapBackgroundAutosave() {
    go2_mapping::SnapshotWriteResult result;
    if (snapshot_writer_.reapIfComplete(&result)) {
      applySnapshotResult(result, "Automatic");
    }
  }

  void waitForBackgroundAutosave(const char* context) {
    applySnapshotResult(snapshot_writer_.wait(), context);
  }

  void publishSnapshotStatus() {
    pnh_.setParam("snapshot_writer_busy", snapshot_writer_.busy());
    pnh_.setParam("snapshot_dirty", dirty_);
    pnh_.setParam("last_snapshot_status", last_snapshot_status_);
    pnh_.setParam("fine_candidate_voxels",
                  static_cast<int>(std::min<std::size_t>(
                      fine_candidate_keys_.size(),
                      static_cast<std::size_t>(
                          std::numeric_limits<int>::max()))));
    pnh_.setParam("fine_candidate_rejections",
                  static_cast<double>(fine_candidate_rejections_));
    pnh_.setParam("coarse_candidate_miss_evictions",
                  static_cast<double>(coarse_candidate_miss_evictions_));
    pnh_.setParam("coarse_candidate_timeout_evictions",
                  static_cast<double>(coarse_candidate_timeout_evictions_));
    pnh_.setParam("fine_candidate_miss_evictions",
                  static_cast<double>(fine_candidate_miss_evictions_));
    pnh_.setParam("fine_candidate_timeout_evictions",
                  static_cast<double>(fine_candidate_timeout_evictions_));
    pnh_.setParam("fine_parent_invalid_evictions",
                  static_cast<double>(fine_parent_invalid_evictions_));
  }

  void snapshotStatusTimer(const ros::TimerEvent&) {
    reapBackgroundAutosave();
    publishSnapshotStatus();
  }

  bool saveMapToDisk(std::string* message, const char* context) {
    last_snapshot_status_ =
        std::string(context) + " snapshot waiting for background writer";
    publishSnapshotStatus();
    const std::string wait_context = std::string(context) + " wait";
    waitForBackgroundAutosave(wait_context.c_str());
    Cloud map;
    Cloud trajectory;
    if (!buildSnapshot(&map, &trajectory, message)) {
      last_snapshot_status_ = std::string(context) +
                              " snapshot refused: " + *message;
      publishSnapshotStatus();
      return false;
    }
    const uint64_t snapshot_sequence = mutation_sequence_;
    last_snapshot_status_ =
        std::string(context) + " snapshot serializing mutation " +
        std::to_string(snapshot_sequence);
    publishSnapshotStatus();
    std::string storage_message;
    if (!go2_mapping::writeMappingSnapshot(
            output_path_, trajectory_output_path_, snapshot_manifest_path_,
            map, trajectory, &storage_message)) {
      *message = storage_message;
      last_snapshot_status_ = std::string(context) +
                              " snapshot failed: " + storage_message;
      publishSnapshotStatus();
      return false;
    }

    if (snapshot_sequence == mutation_sequence_) {
      dirty_ = false;
    }
    *message = "Saved " + std::to_string(map.size()) +
               " fine static-map points to " + output_path_ + " and " +
               std::to_string(trajectory.size()) + " trajectory points to " +
               trajectory_output_path_ + "; " + storage_message;
    last_snapshot_status_ =
        std::string(context) + " snapshot committed at mutation " +
        std::to_string(snapshot_sequence);
    publishSnapshotStatus();
    return true;
  }

  void autosaveTimer(const ros::TimerEvent&) {
    reapBackgroundAutosave();
    if (snapshot_writer_.busy()) {
      ROS_WARN_THROTTLE(5.0,
                        "Automatic mapping snapshot is still being written");
      return;
    }
    if (!dirty_) {
      return;
    }
    last_snapshot_status_ = "Automatic snapshot build started";
    publishSnapshotStatus();
    Cloud map;
    Cloud trajectory;
    std::string message;
    if (!buildSnapshot(&map, &trajectory, &message)) {
      last_snapshot_status_ = "Automatic snapshot skipped: " + message;
      publishSnapshotStatus();
      ROS_ERROR_STREAM_THROTTLE(5.0,
                                "Automatic snapshot skipped: " << message);
      return;
    }
    const uint64_t snapshot_sequence = mutation_sequence_;
    if (!snapshot_writer_.start(
            output_path_, trajectory_output_path_, snapshot_manifest_path_,
            std::move(map), std::move(trajectory), snapshot_sequence,
            &message)) {
      last_snapshot_status_ =
          "Automatic snapshot writer failed to start: " + message;
      publishSnapshotStatus();
      ROS_ERROR_STREAM_THROTTLE(
          5.0, "Could not start automatic snapshot writer: " << message);
      return;
    }
    last_snapshot_status_ =
        "Automatic snapshot writer started at mutation " +
        std::to_string(snapshot_sequence);
    publishSnapshotStatus();
    ROS_INFO_STREAM("Automatic mapping snapshot started at mutation "
                    << snapshot_sequence);
  }

  bool resetMap(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    last_snapshot_status_ = "Reset waiting for background snapshot writer";
    publishSnapshotStatus();
    waitForBackgroundAutosave("Reset wait");
    std::string invalidation_error;
    if (!go2_mapping::invalidateSnapshotManifest(snapshot_manifest_path_,
                                                  &invalidation_error)) {
      last_snapshot_status_ = "Reset refused: " + invalidation_error;
      publishSnapshotStatus();
      ROS_ERROR_STREAM("Mapper reset refused: " << invalidation_error);
      return false;
    }
    temporal_voxels_.clear();
    map_voxels_.clear();
    fine_child_masks_.clear();
    coarse_candidate_keys_.clear();
    fine_candidate_keys_.clear();
    trajectory_.clear();
    pending_clouds_.clear();
    frame_id_.clear();
    trajectory_frame_id_.clear();
    scan_sequence_ = 0;
    next_occupancy_generation_ = 1;
    promoted_static_voxels_ = 0;
    bayesian_cleared_voxels_ = 0;
    fine_promoted_voxels_ = 0;
    fine_cleared_voxels_ = 0;
    fine_candidate_evictions_ = 0;
    fine_candidate_rejections_ = 0;
    coarse_candidate_miss_evictions_ = 0;
    coarse_candidate_timeout_evictions_ = 0;
    fine_candidate_miss_evictions_ = 0;
    fine_candidate_timeout_evictions_ = 0;
    fine_parent_invalid_evictions_ = 0;
    last_cleanup_ = ros::Time();
    last_cloud_stamp_ = ros::Time();
    last_trajectory_stamp_ = ros::Time();
    trajectory_start_stamp_ = ros::Time();
    capacity_gate_.reset();
    capacity_error_.clear();
    pnh_.setParam("capacity_ok", true);
    pnh_.deleteParam("capacity_error");
    ++mutation_sequence_;
    dirty_ = false;
    last_snapshot_status_ =
        "Reset complete; committed mapping snapshot invalidated";
    publishSnapshotStatus();
    ROS_WARN("go2_pointcloud_mapper map and trajectory were reset");
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber pose_odom_sub_;
  ros::Subscriber trajectory_odom_sub_;
  ros::Publisher static_scan_pub_;
  ros::Publisher static_map_pub_;
  ros::Publisher dynamic_pub_;
  ros::ServiceServer save_service_;
  ros::ServiceServer reset_service_;
  ros::Timer publish_timer_;
  ros::Timer autosave_timer_;
  ros::Timer snapshot_status_timer_;

  std::string input_cloud_;
  std::string pose_odom_topic_;
  std::string trajectory_odom_topic_;
  std::string static_scan_topic_;
  std::string static_map_topic_;
  std::string dynamic_topic_;
  std::string output_path_;
  std::string trajectory_output_path_;
  std::string snapshot_manifest_path_;
  std::string frame_id_;
  std::string trajectory_frame_id_;
  std::string odom_frame_;
  std::string base_frame_;
  Eigen::Vector3d base_to_lidar_translation_ = Eigen::Vector3d::Zero();
  double min_range_ = 0.5;
  double max_range_ = 50.0;
  double scan_voxel_size_ = 0.05;
  bool radius_filter_enable_ = true;
  double radius_ = 0.20;
  int min_neighbors_ = 2;
  bool self_filter_enable_ = false;
  double self_min_x_ = -0.55;
  double self_max_x_ = 0.55;
  double self_min_y_ = -0.45;
  double self_max_y_ = 0.45;
  double self_min_z_ = -0.40;
  double self_max_z_ = 0.25;
  bool dynamic_filter_enable_ = true;
  double temporal_voxel_size_ = 0.20;
  go2_mapping::DynamicFilterConfig dynamic_config_;
  go2_mapping::DynamicFilterConfig fine_dynamic_config_;
  int ray_stride_ = 2;
  double max_clearing_range_ = 20.0;
  double ray_endpoint_margin_ = 0.25;
  double candidate_timeout_ = 6.0;
  double cleanup_period_ = 1.0;
  std::size_t max_voxels_ = 2000000;
  double map_voxel_size_ = 0.05;
  std::size_t max_map_voxels_ = 5000000;
  std::size_t max_fine_candidate_voxels_ = 500000;
  int fine_cells_per_coarse_ = 4;
  double autosave_period_ = 30.0;
  double snapshot_status_period_ = 1.0;
  bool save_on_shutdown_ = true;
  double trajectory_min_distance_ = 0.05;
  bool publish_full_map_ = false;
  double map_publish_period_ = 2.0;
  bool publish_dynamic_ = false;
  double max_odom_age_ = 0.20;
  double odom_cache_duration_ = 2.0;
  std::size_t odom_cache_max_messages_ = 400;
  std::size_t cloud_queue_max_messages_ = 10;

  std::deque<PoseSample> pose_odom_cache_;
  std::deque<sensor_msgs::PointCloud2ConstPtr> pending_clouds_;
  Cloud trajectory_;
  go2_mapping::AsyncSnapshotWriter<Point> snapshot_writer_;
  go2_mapping::VoxelCapacityGate capacity_gate_;
  std::string capacity_error_;
  uint64_t mutation_sequence_ = 0;
  uint64_t scan_sequence_ = 0;
  uint64_t next_occupancy_generation_ = 1;
  uint64_t promoted_static_voxels_ = 0;
  uint64_t bayesian_cleared_voxels_ = 0;
  uint64_t fine_promoted_voxels_ = 0;
  uint64_t fine_cleared_voxels_ = 0;
  uint64_t fine_candidate_evictions_ = 0;
  uint64_t fine_candidate_rejections_ = 0;
  uint64_t coarse_candidate_miss_evictions_ = 0;
  uint64_t coarse_candidate_timeout_evictions_ = 0;
  uint64_t fine_candidate_miss_evictions_ = 0;
  uint64_t fine_candidate_timeout_evictions_ = 0;
  uint64_t fine_parent_invalid_evictions_ = 0;
  ros::Time last_cleanup_;
  ros::Time last_cloud_stamp_;
  ros::Time last_trajectory_stamp_;
  ros::Time trajectory_start_stamp_;
  bool dirty_ = false;
  std::string last_snapshot_status_;
  std::unordered_map<VoxelKey, OccupancyVoxelState, VoxelKeyHash>
      temporal_voxels_;
  MapVoxelTable map_voxels_;
  std::unordered_map<VoxelKey, uint64_t, VoxelKeyHash> fine_child_masks_;
  std::unordered_set<VoxelKey, VoxelKeyHash> coarse_candidate_keys_;
  std::unordered_set<VoxelKey, VoxelKeyHash> fine_candidate_keys_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "go2_pointcloud_mapper");
  try {
    PointcloudMapper mapper;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("go2_pointcloud_mapper fatal: %s", error.what());
    return 2;
  }
  return 0;
}
