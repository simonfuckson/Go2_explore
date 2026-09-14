#include <algorithm>
#include <boost/bind.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/point_tests.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>

#include "go2_terrain/terrain_model.hpp"

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using SyncPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;

struct GroundCell {
  std::vector<float> heights;
  float ground_z = 0.0F;
  bool valid = false;
};

sensor_msgs::PointCloud2 toMessage(const Cloud& cloud,
                                  const std_msgs::Header& header) {
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header = header;
  return message;
}

diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                   const std::string& value) {
  diagnostic_msgs::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

template <typename Value>
std::string asString(const Value& value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

class TerrainGuard {
 public:
  TerrainGuard() : nh_(), pnh_("~") {
    loadParameters();
    validateParameters();

    ground_subscriber_.subscribe(nh_, "ground", 1);
    nonground_subscriber_.subscribe(nh_, "nonground", 1);
    synchronizer_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(1), ground_subscriber_, nonground_subscriber_));
    synchronizer_->registerCallback(
        boost::bind(&TerrainGuard::cloudCallback, this, _1, _2));

    obstacle_publisher_ =
        nh_.advertise<sensor_msgs::PointCloud2>("obstacles", 1, false);
    clearing_publisher_ =
        nh_.advertise<sensor_msgs::PointCloud2>("clearing", 1, false);
    if (publish_debug_clouds_) {
      ground_publisher_ =
          nh_.advertise<sensor_msgs::PointCloud2>("safe_ground", 1, false);
      unknown_publisher_ =
          nh_.advertise<sensor_msgs::PointCloud2>("unknown", 1, false);
    }
    diagnostic_publisher_ =
        nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1,
                                                        true);
    healthy_publisher_ = nh_.advertise<std_msgs::Bool>("healthy", 1, true);
    health_timer_ = nh_.createWallTimer(
        ros::WallDuration(0.20), &TerrainGuard::healthTimerCallback, this);

    start_wall_ = ros::WallTime::now();
    frame_reason_ = "waiting for synchronized Patchwork++ output";
    publishHealth();
    ROS_INFO(
        "Terrain guard ready: frame=%s relative obstacle height=%.2f..%.2f m",
        expected_frame_.c_str(), thresholds_.min_relative_height,
        thresholds_.max_relative_height);
  }

 private:
  void loadParameters() {
    pnh_.param<std::string>("expected_frame", expected_frame_,
                            "terrain_sensor");
    pnh_.param("sensor_height", sensor_height_, 0.51);
    pnh_.param("grid/resolution", resolution_, 0.15);
    pnh_.param("grid/min_x", min_x_, -5.0);
    pnh_.param("grid/max_x", max_x_, 5.0);
    pnh_.param("grid/min_y", min_y_, -4.0);
    pnh_.param("grid/max_y", max_y_, 4.0);
    pnh_.param("grid/reference_radius_cells", reference_radius_cells_, 3);
    pnh_.param("vehicle/min_horizontal_range", min_horizontal_range_, 0.35);
    pnh_.param("vehicle/max_horizontal_range", max_horizontal_range_, 5.0);
    pnh_.param("surface/min_points_per_cell", min_points_per_cell_, 1);
    pnh_.param("surface/ground_candidate_quantile",
               ground_candidate_quantile_, 0.10);
    pnh_.param("surface/ground_support_band", ground_support_band_, 0.15);
    pnh_.param("surface/anchor_radius", ground_connectivity_.anchor_radius,
               1.20);
    pnh_.param("surface/anchor_height_tolerance",
               ground_connectivity_.anchor_height_tolerance, 0.25);
    pnh_.param("surface/anchor_seed_height_band",
               ground_connectivity_.anchor_seed_height_band, 0.05);
    pnh_.param("surface/max_connected_slope_deg",
               ground_connectivity_.maximum_slope_deg, 35.0);
    pnh_.param("surface/continuity_height_margin",
               ground_connectivity_.height_margin, 0.01);
    pnh_.param("surface/steep_unknown_min_slope_deg",
               steep_surface_.minimum_slope_deg, 8.0);
    pnh_.param("surface/steep_unknown_max_slope_deg",
               steep_surface_.maximum_slope_deg, 70.0);
    pnh_.param("surface/steep_unknown_min_run_cells",
               steep_surface_.minimum_run_cells, 3);
    pnh_.param("surface/steep_unknown_min_points_per_cell",
               steep_surface_.minimum_points_per_cell, 2);
    pnh_.param("surface/steep_unknown_max_step_delta",
               steep_surface_.maximum_step_delta_m, 0.04);
    pnh_.param("surface/steep_unknown_plane_huber",
               steep_surface_.plane_huber_m, 0.02);
    pnh_.param("surface/steep_unknown_max_plane_rmse",
               steep_surface_.maximum_plane_rmse_m, 0.025);
    pnh_.param("surface/steep_unknown_max_point_residual",
               steep_surface_.maximum_point_residual_m, 0.05);
    pnh_.param("surface/obstacle_min_relative_height",
               thresholds_.min_relative_height, 0.05);
    pnh_.param("surface/obstacle_max_relative_height",
               thresholds_.max_relative_height, 1.50);
    pnh_.param("output/publish_debug_clouds", publish_debug_clouds_, false);
    pnh_.param("health/input_timeout_sec", input_timeout_sec_, 0.60);
    pnh_.param("health/min_input_points", min_input_points_, 20);
    pnh_.param("health/min_ground_points", min_ground_points_, 20);
    pnh_.param("health/minimum_connected_ground_area_m2",
               ground_health_.minimum_connected_area_m2, 0.60);
    pnh_.param("health/near_support_radius_m",
               ground_health_.near_support_radius_m, 1.00);
    pnh_.param("health/minimum_near_support_area_m2",
               ground_health_.minimum_near_support_area_m2, 0.18);
    pnh_.param("health/ground_sector_count", ground_health_.sector_count, 8);
    pnh_.param("health/minimum_ground_sectors",
               ground_health_.minimum_covered_sectors, 4);
    pnh_.param("health/minimum_cells_per_sector",
               ground_health_.minimum_cells_per_sector, 2);
    pnh_.param("health/ground_plane_min_connected_cells",
               ground_plane_fit_.minimum_connected_samples, 12);
    pnh_.param("health/ground_plane_maximum_radius_m",
               ground_plane_fit_.maximum_radius_m, 1.50);
    pnh_.param("health/ground_plane_min_planar_variance_m2",
               ground_plane_fit_.minimum_planar_variance_m2, 0.01);
    pnh_.param("health/ground_plane_huber_m",
               ground_plane_fit_.huber_delta_m, 0.03);
    pnh_.param("health/ground_plane_max_rmse_m",
               ground_plane_fit_.maximum_rmse_m, 0.04);
    pnh_.param("health/ground_plane_irls_iterations",
               ground_plane_fit_.irls_iterations, 5);
    pnh_.param("health/min_sensor_height_m",
               ground_plane_fit_.minimum_sensor_height_m, 0.43);
    pnh_.param("health/max_sensor_height_m",
               ground_plane_fit_.maximum_sensor_height_m, 0.59);
    pnh_.param("health/sensor_height_hysteresis_m",
               sensor_height_hysteresis_m_, 0.02);
    pnh_.param("health/open_after_consecutive_healthy_frames",
               health_hysteresis_parameters_.opening_healthy_frames, 5);
    pnh_.param("health/max_soft_geometry_failure_frames",
               health_hysteresis_parameters_.maximum_soft_failure_frames, 3);
    pnh_.param(
        "health/max_soft_geometry_failure_duration_sec",
        health_hysteresis_parameters_.maximum_soft_failure_duration_sec, 0.25);
    pnh_.param("health/max_processing_ms", max_processing_ms_, 100.0);
    pnh_.param("health/min_output_rate_hz", min_output_rate_hz_, 8.0);
    pnh_.param("health/min_rate_samples", min_rate_samples_, 3);
    pnh_.param("health/startup_grace_sec", startup_grace_sec_, 3.0);
    pnh_.param("health/low_rate_hold_sec", low_rate_hold_sec_, 1.0);

    thresholds_.no_ground_high_split_z =
        -sensor_height_ + thresholds_.max_relative_height;
  }

  void validateParameters() {
    if (expected_frame_.empty() || resolution_ <= 0.0 || min_x_ >= max_x_ ||
        min_y_ >= max_y_ || reference_radius_cells_ < 0 ||
        min_horizontal_range_ < 0.0 ||
        max_horizontal_range_ <= min_horizontal_range_ ||
        min_points_per_cell_ < 1 ||
        ground_candidate_quantile_ < 0.0 ||
        ground_candidate_quantile_ > 1.0 || ground_support_band_ <= 0.0 ||
        ground_connectivity_.anchor_radius <= min_horizontal_range_ ||
        ground_connectivity_.anchor_height_tolerance <= 0.0 ||
        ground_connectivity_.anchor_seed_height_band <= 0.0 ||
        ground_connectivity_.maximum_slope_deg <= 0.0 ||
        ground_connectivity_.maximum_slope_deg > 35.0 ||
        ground_connectivity_.height_margin < 0.0 ||
        steep_surface_.minimum_slope_deg <= 0.0 ||
        steep_surface_.minimum_slope_deg >
            ground_connectivity_.maximum_slope_deg ||
        steep_surface_.maximum_slope_deg <=
            steep_surface_.minimum_slope_deg ||
        steep_surface_.maximum_slope_deg >= 89.0 ||
        steep_surface_.minimum_run_cells < 2 ||
        steep_surface_.minimum_points_per_cell < 1 ||
        steep_surface_.maximum_step_delta_m < 0.0 ||
        steep_surface_.plane_huber_m <= 0.0 ||
        steep_surface_.maximum_plane_rmse_m <= 0.0 ||
        steep_surface_.maximum_point_residual_m <= 0.0 ||
        thresholds_.min_relative_height < 0.0 ||
        thresholds_.max_relative_height <= thresholds_.min_relative_height ||
        input_timeout_sec_ <= 0.0 || min_input_points_ < 1 ||
        min_ground_points_ < 1 || max_processing_ms_ <= 0.0 ||
        ground_health_.minimum_connected_area_m2 <= 0.0 ||
        ground_health_.near_support_radius_m <= min_horizontal_range_ ||
        ground_health_.minimum_near_support_area_m2 <= 0.0 ||
        ground_health_.sector_count < 1 ||
        ground_health_.minimum_covered_sectors < 1 ||
        ground_health_.minimum_covered_sectors > ground_health_.sector_count ||
        ground_health_.minimum_cells_per_sector < 1 ||
        ground_plane_fit_.minimum_connected_samples < 3 ||
        ground_plane_fit_.maximum_radius_m <= 0.0 ||
        ground_plane_fit_.minimum_planar_variance_m2 <= 0.0 ||
        ground_plane_fit_.huber_delta_m <= 0.0 ||
        ground_plane_fit_.maximum_rmse_m <= 0.0 ||
        ground_plane_fit_.irls_iterations < 1 ||
        ground_plane_fit_.minimum_sensor_height_m <= 0.0 ||
        ground_plane_fit_.maximum_sensor_height_m <
            ground_plane_fit_.minimum_sensor_height_m ||
        !std::isfinite(sensor_height_hysteresis_m_) ||
        sensor_height_hysteresis_m_ < 0.0 || sensor_height_hysteresis_m_ > 0.03 ||
        health_hysteresis_parameters_.opening_healthy_frames < 1 ||
        health_hysteresis_parameters_.maximum_soft_failure_frames < 0 ||
        health_hysteresis_parameters_.maximum_soft_failure_duration_sec <
            0.0 ||
        min_output_rate_hz_ <= 0.0 || min_rate_samples_ < 1 ||
        startup_grace_sec_ < 0.0 ||
        low_rate_hold_sec_ < 0.0) {
      throw std::invalid_argument("invalid terrain guard parameter");
    }
    if (!go2_terrain::validSensorHeight(sensor_height_, 0.43, 0.59)) {
      throw std::invalid_argument(
          "terrain guard sensor_height must be within 0.43-0.59 m");
    }
    width_ = static_cast<int>(std::ceil((max_x_ - min_x_) / resolution_));
    height_ = static_cast<int>(std::ceil((max_y_ - min_y_) / resolution_));
    ground_connectivity_.width = width_;
    ground_connectivity_.height = height_;
    ground_connectivity_.resolution = resolution_;
    ground_connectivity_.origin_x = min_x_;
    ground_connectivity_.origin_y = min_y_;
    ground_connectivity_.sensor_height = sensor_height_;
  }

  bool inWorkingArea(const Point& point) const {
    if (!pcl::isFinite(point)) {
      return false;
    }
    const double range = std::hypot(point.x, point.y);
    return range >= min_horizontal_range_ && range <= max_horizontal_range_ &&
           point.x >= min_x_ && point.x < max_x_ && point.y >= min_y_ &&
           point.y < max_y_;
  }

  int xIndex(double x) const {
    return static_cast<int>(std::floor((x - min_x_) / resolution_));
  }

  int yIndex(double y) const {
    return static_cast<int>(std::floor((y - min_y_) / resolution_));
  }

  int flatIndex(int x, int y) const { return y * width_ + x; }

  bool nearestGround(const std::vector<GroundCell>& cells, const Point& point,
                     double* ground_z) const {
    const int center_x = xIndex(point.x);
    const int center_y = yIndex(point.y);
    double best_distance_sq = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int dy = -reference_radius_cells_; dy <= reference_radius_cells_;
         ++dy) {
      for (int dx = -reference_radius_cells_; dx <= reference_radius_cells_;
           ++dx) {
        const int x = center_x + dx;
        const int y = center_y + dy;
        if (x < 0 || x >= width_ || y < 0 || y >= height_) {
          continue;
        }
        const GroundCell& cell = cells[flatIndex(x, y)];
        if (!cell.valid) {
          continue;
        }
        const double distance_sq = static_cast<double>(dx * dx + dy * dy);
        if (distance_sq < best_distance_sq) {
          best_distance_sq = distance_sq;
          *ground_z = cell.ground_z;
          found = true;
        }
      }
    }
    return found;
  }

  void updateFrameHealthGate(const ros::WallTime& now) {
    go2_terrain::updateTerrainHealthHysteresis(
        frame_health_class_, now.toSec(), health_hysteresis_parameters_,
        &health_hysteresis_state_);
    soft_failure_deadline_timer_.stop();
    if (frame_health_class_ !=
            go2_terrain::TerrainFrameHealthClass::kSoftGeometryFailure ||
        !health_hysteresis_state_.gate_open) {
      return;
    }
    const double age = go2_terrain::terrainSoftFailureAgeSec(
        health_hysteresis_state_, now.toSec());
    const double remaining =
        health_hysteresis_parameters_.maximum_soft_failure_duration_sec - age;
    if (remaining <= 0.0) {
      go2_terrain::enforceTerrainHealthHysteresisDeadline(
          now.toSec(), health_hysteresis_parameters_,
          &health_hysteresis_state_);
      return;
    }
    // The deadline is measured from the first consecutive soft failure. A
    // later soft frame reschedules only the remaining duration and can never
    // extend the allowance.
    soft_failure_deadline_timer_ = nh_.createWallTimer(
        ros::WallDuration(remaining),
        &TerrainGuard::softFailureDeadlineCallback, this, true, true);
  }

  void softFailureDeadlineCallback(const ros::WallTimerEvent&) {
    go2_terrain::enforceTerrainHealthHysteresisDeadline(
        ros::WallTime::now().toSec(), health_hysteresis_parameters_,
        &health_hysteresis_state_);
    publishHealth();
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& ground_message,
                     const sensor_msgs::PointCloud2ConstPtr& nonground_message) {
    const ros::WallTime start = ros::WallTime::now();
    if (!last_frame_wall_.isZero()) {
      const double interval = (start - last_frame_wall_).toSec();
      if (interval > input_timeout_sec_) {
        // A stale gap starts a new rate window. Counting the first callback
        // after a gap would turn a stopped 1 Hz stream into a plausible rate.
        output_rate_hz_ = 0.0;
        rate_samples_ = 0U;
        low_rate_since_ = ros::WallTime();
      } else if (interval > 1e-4) {
        const double instantaneous_rate = 1.0 / interval;
        output_rate_hz_ = rate_samples_ == 0U
                              ? instantaneous_rate
                              : 0.2 * instantaneous_rate +
                                    0.8 * output_rate_hz_;
        ++rate_samples_;
      }
    }
    last_frame_wall_ = start;
    if (rate_samples_ >= static_cast<std::size_t>(min_rate_samples_) &&
        output_rate_hz_ < min_output_rate_hz_) {
      if (low_rate_since_.isZero()) {
        low_rate_since_ = start;
      }
    } else {
      low_rate_since_ = ros::WallTime();
    }
    last_input_wall_ = start;
    if (ground_message->header.frame_id != expected_frame_ ||
        nonground_message->header.frame_id != expected_frame_ ||
        ground_message->header.stamp.isZero()) {
      frame_healthy_ = false;
      frame_health_class_ =
          go2_terrain::TerrainFrameHealthClass::kHardFailure;
      updateFrameHealthGate(start);
      frame_reason_ = "frame mismatch or zero timestamp";
      publishHealth();
      ROS_ERROR_THROTTLE(1.0,
                         "Terrain guard expected frame %s with nonzero stamp",
                         expected_frame_.c_str());
      return;
    }

    Cloud ground;
    Cloud nonground;
    pcl::fromROSMsg(*ground_message, ground);
    pcl::fromROSMsg(*nonground_message, nonground);
    last_input_points_ = 0U;
    last_ground_points_ = 0U;

    std::vector<GroundCell> cells(
        static_cast<std::size_t>(width_ * height_));
    for (const Point& point : ground.points) {
      if (!inWorkingArea(point)) {
        continue;
      }
      ++last_input_points_;
      cells[flatIndex(xIndex(point.x), yIndex(point.y))].heights.push_back(
          point.z);
    }

    std::vector<float> candidate_heights(
        cells.size(), std::numeric_limits<float>::quiet_NaN());
    for (std::size_t index = 0; index < cells.size(); ++index) {
      GroundCell& cell = cells[index];
      if (static_cast<int>(cell.heights.size()) < min_points_per_cell_) {
        continue;
      }
      cell.ground_z = go2_terrain::robustLowSupportHeight(
          cell.heights, ground_candidate_quantile_, ground_support_band_);
      candidate_heights[index] = cell.ground_z;
    }
    const std::vector<std::uint8_t> connected_ground_candidates =
        go2_terrain::connectedGroundMask(candidate_heights,
                                         ground_connectivity_);
    const go2_terrain::SelectedGroundComponent selected_ground =
        go2_terrain::selectGroundComponent(connected_ground_candidates,
                                           ground_connectivity_,
                                           ground_health_);
    const std::vector<std::uint8_t>& connected_ground = selected_ground.mask;
    last_ground_coverage_ = selected_ground.coverage;
    std::vector<std::vector<go2_terrain::SurfaceSample>> surface_samples(
        cells.size());
    auto append_surface_sample = [&](const Point& point) {
      const std::size_t index = static_cast<std::size_t>(
          flatIndex(xIndex(point.x), yIndex(point.y)));
      surface_samples[index].push_back({point.x, point.y, point.z});
    };
    for (const Point& point : nonground.points) {
      if (inWorkingArea(point)) {
        append_surface_sample(point);
      }
    }
    for (const Point& point : ground.points) {
      if (!inWorkingArea(point)) {
        continue;
      }
      const std::size_t index = static_cast<std::size_t>(
          flatIndex(xIndex(point.x), yIndex(point.y)));
      const bool is_connected_support =
          connected_ground[index] != 0U &&
          std::isfinite(candidate_heights[index]) &&
          std::fabs(static_cast<double>(point.z) - candidate_heights[index]) <=
              ground_support_band_;
      if (!is_connected_support) {
        append_surface_sample(point);
      }
    }
    std::vector<float> surface_candidate_heights(
        cells.size(), std::numeric_limits<float>::quiet_NaN());
    for (std::size_t index = 0; index < cells.size(); ++index) {
      if (static_cast<int>(surface_samples[index].size()) <
          min_points_per_cell_) {
        continue;
      }
      std::vector<go2_terrain::SurfaceSample> low_support =
          go2_terrain::lowSupportSurfaceSamples(
              surface_samples[index], ground_candidate_quantile_,
              ground_support_band_, &surface_candidate_heights[index]);
      surface_samples[index].swap(low_support);
    }
    const std::vector<std::uint8_t> continuous_steep_surface =
        go2_terrain::continuousSteepSurfaceMask(
            candidate_heights, connected_ground,
            surface_candidate_heights, surface_samples,
            ground_connectivity_, steep_surface_);
    active_ground_plane_fit_ = go2_terrain::groundPlaneFitForHealthGate(
        ground_plane_fit_, health_hysteresis_state_.gate_open,
        sensor_height_hysteresis_m_);
    last_ground_plane_ = go2_terrain::estimateConnectedGroundPlane(
        candidate_heights, connected_ground, ground_connectivity_,
        active_ground_plane_fit_);
    for (std::size_t index = 0; index < cells.size(); ++index) {
      cells[index].valid = connected_ground[index] != 0U;
    }

    Cloud clearing;
    Cloud debug_ground;
    Cloud unknown;
    Cloud obstacles;
    std::size_t high_returns = 0U;
    std::size_t low_returns = 0U;
    std::size_t steep_unknown_returns = 0U;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        GroundCell& cell = cells[flatIndex(x, y)];
        if (!cell.valid) {
          continue;
        }
        Point representative;
        representative.x =
            static_cast<float>(min_x_ + (x + 0.5) * resolution_);
        representative.y =
            static_cast<float>(min_y_ + (y + 0.5) * resolution_);
        representative.z = cell.ground_z;
        representative.intensity = 0.0F;
        clearing.push_back(representative);
        if (publish_debug_clouds_) {
          debug_ground.push_back(representative);
        }
      }
    }

    auto append_unknown = [&](const Point& point,
                              go2_terrain::NongroundClass classification) {
      if (classification == go2_terrain::NongroundClass::kUnknownHigh) {
        ++high_returns;
      } else {
        ++low_returns;
      }
      if (publish_debug_clouds_) {
        unknown.push_back(point);
      }
    };

    auto classify_nonfloor_return = [&](const Point& point) {
      double ground_z = 0.0;
      const bool has_ground = nearestGround(cells, point, &ground_z);
      const std::size_t index = static_cast<std::size_t>(
          flatIndex(xIndex(point.x), yIndex(point.y)));
      const bool is_continuous_steep_surface =
          continuous_steep_surface[index] != 0U;
      const go2_terrain::NongroundClass classification =
          go2_terrain::classifyNongroundForLocalCostmap(
              point.z, has_ground, ground_z, is_continuous_steep_surface,
              thresholds_);
      if (is_continuous_steep_surface) {
        ++steep_unknown_returns;
      }
      if (classification == go2_terrain::NongroundClass::kObstacle) {
        obstacles.push_back(point);
        return;
      }
      append_unknown(point, classification);
    };

    for (const Point& point : ground.points) {
      if (!inWorkingArea(point)) {
        continue;
      }
      const GroundCell& cell =
          cells[flatIndex(xIndex(point.x), yIndex(point.y))];
      if (cell.valid &&
          std::fabs(static_cast<double>(point.z - cell.ground_z)) <=
              ground_support_band_) {
        ++last_ground_points_;
        continue;
      }
      classify_nonfloor_return(point);
    }

    for (const Point& point : nonground.points) {
      if (!inWorkingArea(point)) {
        continue;
      }
      ++last_input_points_;
      classify_nonfloor_return(point);
    }
    ground_ratio_ = last_input_points_ == 0U
                        ? 0.0
                        : static_cast<double>(last_ground_points_) /
                              static_cast<double>(last_input_points_);

    clearing_publisher_.publish(toMessage(clearing, ground_message->header));
    obstacle_publisher_.publish(toMessage(obstacles, ground_message->header));
    if (publish_debug_clouds_) {
      ground_publisher_.publish(
          toMessage(debug_ground, ground_message->header));
      unknown_publisher_.publish(toMessage(unknown, ground_message->header));
    }

    last_obstacle_points_ = obstacles.size();
    last_clearing_points_ = clearing.size();
    last_high_returns_ = high_returns;
    last_low_returns_ = low_returns;
    last_steep_unknown_points_ = steep_unknown_returns;
    last_processing_ms_ = (ros::WallTime::now() - start).toSec() * 1000.0;
    const bool enough_input_points =
        static_cast<int>(last_input_points_) >= min_input_points_;
    const bool enough_ground_points =
        static_cast<int>(last_ground_points_) >= min_ground_points_;
    const bool enough_connected_area =
        last_ground_coverage_.connected_area_m2 >=
        ground_health_.minimum_connected_area_m2;
    const bool enough_near_support =
        last_ground_coverage_.near_support_area_m2 >=
        ground_health_.minimum_near_support_area_m2;
    const bool enough_sector_coverage =
        last_ground_coverage_.covered_sectors >=
        ground_health_.minimum_covered_sectors;
    const bool processing_within_deadline =
        last_processing_ms_ <= max_processing_ms_;
    frame_health_class_ = go2_terrain::classifyTerrainFrameHealth(
        enough_input_points, enough_ground_points, enough_connected_area,
        enough_near_support, enough_sector_coverage, last_ground_plane_,
        processing_within_deadline);
    frame_healthy_ =
        frame_health_class_ == go2_terrain::TerrainFrameHealthClass::kHealthy;
    updateFrameHealthGate(start);
    if (static_cast<int>(last_input_points_) < min_input_points_) {
      frame_reason_ = "too few input points";
    } else if (static_cast<int>(last_ground_points_) < min_ground_points_) {
      frame_reason_ = "too few ground points";
    } else if (last_ground_coverage_.connected_area_m2 <
               ground_health_.minimum_connected_area_m2) {
      frame_reason_ = "insufficient connected ground area";
    } else if (last_ground_coverage_.near_support_area_m2 <
               ground_health_.minimum_near_support_area_m2) {
      frame_reason_ = "insufficient near-field ground support";
    } else if (last_ground_coverage_.covered_sectors <
               ground_health_.minimum_covered_sectors) {
      frame_reason_ = "insufficient ground sector coverage";
    } else if (!last_ground_plane_.valid) {
      switch (last_ground_plane_.status) {
        case go2_terrain::GroundPlaneFitStatus::
            kInsufficientConnectedSamples:
          frame_reason_ =
              "insufficient connected ground samples for MID360 height";
          break;
        case go2_terrain::GroundPlaneFitStatus::kDegenerateGeometry:
          frame_reason_ =
              "connected ground geometry is degenerate for MID360 height";
          break;
        case go2_terrain::GroundPlaneFitStatus::kFitFailure:
          frame_reason_ = "connected ground plane fit failed";
          break;
        case go2_terrain::GroundPlaneFitStatus::kExcessiveResidual:
          frame_reason_ = "connected ground plane RMSE " +
                          asString(last_ground_plane_.rmse_m) +
                          " m exceeds maximum " +
                          asString(ground_plane_fit_.maximum_rmse_m) + " m";
          break;
        case go2_terrain::GroundPlaneFitStatus::
            kSensorHeightBelowMinimum:
          frame_reason_ = "estimated MID360 height " +
                          asString(last_ground_plane_.sensor_height_m) +
                          " m is below minimum " +
                          asString(active_ground_plane_fit_.minimum_sensor_height_m) +
                          " m";
          break;
        case go2_terrain::GroundPlaneFitStatus::
            kSensorHeightAboveMaximum:
          frame_reason_ = "estimated MID360 height " +
                          asString(last_ground_plane_.sensor_height_m) +
                          " m is above maximum " +
                          asString(active_ground_plane_fit_.maximum_sensor_height_m) +
                          " m";
          break;
        case go2_terrain::GroundPlaneFitStatus::kValid:
          frame_reason_ = "connected ground plane validity mismatch";
          break;
      }
    } else if (last_processing_ms_ > max_processing_ms_) {
      frame_reason_ = "processing deadline exceeded";
    } else {
      frame_reason_ = "healthy";
    }
    publishHealth();
    ROS_INFO_THROTTLE(
        5.0,
        "Terrain guard ground=%zu obstacles=%zu steep_unknown=%zu "
        "ceilings/high=%zu low=%zu processing=%.1f ms",
        last_ground_points_, last_obstacle_points_,
        last_steep_unknown_points_, last_high_returns_, last_low_returns_,
        last_processing_ms_);
  }

  void healthTimerCallback(const ros::WallTimerEvent&) { publishHealth(); }

  void publishHealth() {
    const ros::WallTime now = ros::WallTime::now();
    double input_age = std::numeric_limits<double>::infinity();
    if (!last_input_wall_.isZero()) {
      input_age = (now - last_input_wall_).toSec();
    }
    const bool in_startup_grace =
        !start_wall_.isZero() &&
        (now - start_wall_).toSec() <= startup_grace_sec_;
    const bool low_rate_sustained =
        !in_startup_grace && !low_rate_since_.isZero() &&
        (now - low_rate_since_).toSec() >= low_rate_hold_sec_;
    const bool stale = input_age > input_timeout_sec_;
    const bool rate_healthy = go2_terrain::healthyTerrainOutputRate(
        rate_samples_, output_rate_hz_, min_rate_samples_,
        min_output_rate_hz_);
    go2_terrain::enforceTerrainHealthHysteresisDeadline(
        now.toSec(), health_hysteresis_parameters_,
        &health_hysteresis_state_);
    if (stale || !rate_healthy) {
      // Missing input and insufficient output rate are hard failures. They
      // must not consume the geometry-only hold allowance.
      go2_terrain::forceTerrainHealthGateClosed(&health_hysteresis_state_);
    }
    // Startup grace may suppress noisy diagnostics, but it must never arm
    // navigation before the live terrain stream has proved its rate.
    const bool healthy =
        health_hysteresis_state_.gate_open && !stale && rate_healthy;
    std_msgs::Bool health_message;
    health_message.data = healthy;
    healthy_publisher_.publish(health_message);

    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "go2_terrain/live_ground_filter";
    status.hardware_id = "livox_mid360";
    const bool rate_warning =
        frame_health_class_ !=
            go2_terrain::TerrainFrameHealthClass::kHardFailure &&
        !stale && !rate_healthy &&
        (in_startup_grace || !low_rate_sustained);
    const bool soft_failure_held =
        healthy && frame_health_class_ ==
                       go2_terrain::TerrainFrameHealthClass::
                           kSoftGeometryFailure;
    const bool opening_gate =
        !healthy && !stale && rate_healthy && frame_healthy_ &&
        health_hysteresis_state_.consecutive_healthy_frames > 0;
    status.level = healthy && !soft_failure_held
                       ? diagnostic_msgs::DiagnosticStatus::OK
                       : ((rate_warning || soft_failure_held || opening_gate)
                              ? diagnostic_msgs::DiagnosticStatus::WARN
                              : diagnostic_msgs::DiagnosticStatus::ERROR);
    if (stale) {
      status.message = "terrain input stale";
    } else if (frame_health_class_ ==
               go2_terrain::TerrainFrameHealthClass::kHardFailure) {
      status.message = frame_reason_;
    } else if (rate_samples_ <
               static_cast<std::size_t>(min_rate_samples_)) {
      status.message = "collecting terrain output rate samples";
    } else if (!rate_healthy) {
      status.message = "Patchwork++ output below minimum rate";
    } else if (soft_failure_held) {
      status.message = "holding terrain health during transient: " +
                       frame_reason_;
    } else if (frame_health_class_ ==
               go2_terrain::TerrainFrameHealthClass::
                   kSoftGeometryFailure) {
      status.message = frame_reason_;
    } else if (!healthy) {
      status.message = "collecting consecutive healthy terrain frames";
    } else {
      status.message = frame_reason_;
    }
    status.values.push_back(keyValue("expected_frame", expected_frame_));
    status.values.push_back(keyValue("input_age_sec", asString(input_age)));
    status.values.push_back(
        keyValue("input_points", asString(last_input_points_)));
    status.values.push_back(
        keyValue("ground_points", asString(last_ground_points_)));
    status.values.push_back(keyValue("ground_ratio", asString(ground_ratio_)));
    status.values.push_back(keyValue(
        "connected_ground_area_m2",
        asString(last_ground_coverage_.connected_area_m2)));
    status.values.push_back(keyValue(
        "near_support_area_m2",
        asString(last_ground_coverage_.near_support_area_m2)));
    status.values.push_back(keyValue(
        "covered_ground_sectors",
        asString(last_ground_coverage_.covered_sectors)));
    status.values.push_back(keyValue(
        "ground_plane_fit_status",
        go2_terrain::groundPlaneFitStatusName(last_ground_plane_.status)));
    status.values.push_back(keyValue(
        "ground_plane_samples", asString(last_ground_plane_.sample_count)));
    status.values.push_back(keyValue(
        "ground_plane_maximum_radius_m",
        asString(ground_plane_fit_.maximum_radius_m)));
    status.values.push_back(keyValue(
        "estimated_sensor_height_m",
        asString(last_ground_plane_.sensor_height_m)));
    status.values.push_back(keyValue(
        "minimum_sensor_height_m",
        asString(ground_plane_fit_.minimum_sensor_height_m)));
    status.values.push_back(keyValue(
        "maximum_sensor_height_m",
        asString(ground_plane_fit_.maximum_sensor_height_m)));
    status.values.push_back(keyValue("sensor_height_hysteresis_m",
                                    asString(sensor_height_hysteresis_m_)));
    status.values.push_back(keyValue("active_minimum_sensor_height_m",
        asString(active_ground_plane_fit_.minimum_sensor_height_m)));
    status.values.push_back(keyValue("active_maximum_sensor_height_m",
        asString(active_ground_plane_fit_.maximum_sensor_height_m)));
    status.values.push_back(keyValue(
        "ground_plane_slope_deg", asString(last_ground_plane_.slope_deg)));
    status.values.push_back(keyValue(
        "ground_plane_rmse_m", asString(last_ground_plane_.rmse_m)));
    status.values.push_back(keyValue(
        "frame_health_class",
        go2_terrain::terrainFrameHealthClassName(frame_health_class_)));
    status.values.push_back(keyValue(
        "health_gate_open", health_hysteresis_state_.gate_open ? "true"
                                                                 : "false"));
    status.values.push_back(keyValue(
        "consecutive_healthy_frames",
        asString(health_hysteresis_state_.consecutive_healthy_frames)));
    status.values.push_back(keyValue(
        "consecutive_soft_geometry_failures",
        asString(
            health_hysteresis_state_.consecutive_soft_failure_frames)));
    status.values.push_back(keyValue(
        "soft_geometry_failure_age_sec",
        asString(go2_terrain::terrainSoftFailureAgeSec(
            health_hysteresis_state_, now.toSec()))));
    status.values.push_back(keyValue(
        "open_after_consecutive_healthy_frames",
        asString(health_hysteresis_parameters_.opening_healthy_frames)));
    status.values.push_back(keyValue(
        "max_soft_geometry_failure_frames",
        asString(
            health_hysteresis_parameters_.maximum_soft_failure_frames)));
    status.values.push_back(keyValue(
        "max_soft_geometry_failure_duration_sec",
        asString(health_hysteresis_parameters_
                     .maximum_soft_failure_duration_sec)));
    status.values.push_back(
        keyValue("output_rate_hz", asString(output_rate_hz_)));
    status.values.push_back(
        keyValue("rate_samples", asString(rate_samples_)));
    status.values.push_back(
        keyValue("minimum_rate_samples", asString(min_rate_samples_)));
    status.values.push_back(
        keyValue("obstacle_points", asString(last_obstacle_points_)));
    status.values.push_back(keyValue(
        "steep_ground_unknown_points",
        asString(last_steep_unknown_points_)));
    status.values.push_back(
        keyValue("clearing_cells", asString(last_clearing_points_)));
    status.values.push_back(
        keyValue("ceiling_or_high_points", asString(last_high_returns_)));
    status.values.push_back(
        keyValue("processing_ms", asString(last_processing_ms_)));
    array.status.push_back(status);
    diagnostic_publisher_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> ground_subscriber_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> nonground_subscriber_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  ros::Publisher obstacle_publisher_;
  ros::Publisher clearing_publisher_;
  ros::Publisher ground_publisher_;
  ros::Publisher unknown_publisher_;
  ros::Publisher diagnostic_publisher_;
  ros::Publisher healthy_publisher_;
  ros::WallTimer health_timer_;
  ros::WallTimer soft_failure_deadline_timer_;

  std::string expected_frame_;
  double sensor_height_ = 0.51;
  double resolution_ = 0.15;
  double min_x_ = -5.0;
  double max_x_ = 5.0;
  double min_y_ = -4.0;
  double max_y_ = 4.0;
  int reference_radius_cells_ = 3;
  double min_horizontal_range_ = 0.35;
  double max_horizontal_range_ = 5.0;
  int min_points_per_cell_ = 1;
  double ground_candidate_quantile_ = 0.10;
  double ground_support_band_ = 0.15;
  go2_terrain::GroundConnectivityParameters ground_connectivity_;
  go2_terrain::GroundHealthParameters ground_health_;
  go2_terrain::GroundPlaneFitParameters ground_plane_fit_;
  go2_terrain::GroundPlaneFitParameters active_ground_plane_fit_;
  double sensor_height_hysteresis_m_ = 0.02;
  go2_terrain::TerrainHealthHysteresisParameters
      health_hysteresis_parameters_;
  go2_terrain::TerrainHealthHysteresisState health_hysteresis_state_;
  go2_terrain::SteepSurfaceParameters steep_surface_;
  go2_terrain::GroundCoverageMetrics last_ground_coverage_;
  go2_terrain::GroundPlaneEstimate last_ground_plane_;
  go2_terrain::NongroundThresholds thresholds_;
  bool publish_debug_clouds_ = false;
  double input_timeout_sec_ = 0.60;
  int min_input_points_ = 20;
  int min_ground_points_ = 20;
  double max_processing_ms_ = 100.0;
  double min_output_rate_hz_ = 8.0;
  int min_rate_samples_ = 3;
  double startup_grace_sec_ = 3.0;
  double low_rate_hold_sec_ = 1.0;
  int width_ = 0;
  int height_ = 0;

  ros::WallTime last_input_wall_;
  ros::WallTime last_frame_wall_;
  ros::WallTime low_rate_since_;
  ros::WallTime start_wall_;
  bool frame_healthy_ = false;
  go2_terrain::TerrainFrameHealthClass frame_health_class_ =
      go2_terrain::TerrainFrameHealthClass::kHardFailure;
  std::string frame_reason_;
  std::size_t last_input_points_ = 0U;
  std::size_t last_ground_points_ = 0U;
  std::size_t last_obstacle_points_ = 0U;
  std::size_t last_steep_unknown_points_ = 0U;
  std::size_t last_clearing_points_ = 0U;
  std::size_t last_high_returns_ = 0U;
  std::size_t last_low_returns_ = 0U;
  double last_processing_ms_ = 0.0;
  double ground_ratio_ = 0.0;
  double output_rate_hz_ = 0.0;
  std::size_t rate_samples_ = 0U;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "go2_terrain_guard");
  try {
    TerrainGuard node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Cannot start GO2 terrain guard: %s", error.what());
    return 1;
  }
  return 0;
}
