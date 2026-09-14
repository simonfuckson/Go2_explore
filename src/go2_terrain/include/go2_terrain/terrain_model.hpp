#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

namespace go2_terrain {

enum class NongroundClass {
  kObstacle,
  kUnknownLow,
  kUnknownHigh,
};

struct NongroundThresholds {
  double min_relative_height = 0.05;
  double max_relative_height = 1.50;
  double no_ground_high_split_z = 0.99;
};

struct GroundConnectivityParameters {
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double sensor_height = 0.51;
  double anchor_radius = 1.20;
  double anchor_height_tolerance = 0.25;
  double anchor_seed_height_band = 0.05;
  double maximum_slope_deg = 35.0;
  double height_margin = 0.01;
};

struct GroundHealthParameters {
  double minimum_connected_area_m2 = 0.60;
  double near_support_radius_m = 1.00;
  double minimum_near_support_area_m2 = 0.18;
  int sector_count = 8;
  int minimum_covered_sectors = 4;
  int minimum_cells_per_sector = 2;
};

struct GroundCoverageMetrics {
  std::size_t connected_cells = 0U;
  std::size_t near_support_cells = 0U;
  int covered_sectors = 0;
  double connected_area_m2 = 0.0;
  double near_support_area_m2 = 0.0;
};

struct SelectedGroundComponent {
  std::vector<std::uint8_t> mask;
  GroundCoverageMetrics coverage;
};

enum class GroundPlaneFitStatus {
  kValid,
  kInsufficientConnectedSamples,
  kDegenerateGeometry,
  kFitFailure,
  kExcessiveResidual,
  kSensorHeightBelowMinimum,
  kSensorHeightAboveMaximum,
};

struct GroundPlaneFitParameters {
  int minimum_connected_samples = 12;
  double maximum_radius_m = 1.50;
  double minimum_planar_variance_m2 = 0.01;
  double huber_delta_m = 0.03;
  double maximum_rmse_m = 0.04;
  int irls_iterations = 5;
  double minimum_sensor_height_m = 0.43;
  double maximum_sensor_height_m = 0.59;
};

// Schmitt bounds: startup/recovery stays strict; a healthy running gate gets
// a small height margin for stance oscillation and ramp-transition fit bias.
// This changes no ground classification, plane-quality or stale-data checks.
inline GroundPlaneFitParameters groundPlaneFitForHealthGate(
    const GroundPlaneFitParameters& configured, bool gate_open,
    double height_hysteresis_m) {
  if (!std::isfinite(height_hysteresis_m) || height_hysteresis_m < 0.0 ||
      height_hysteresis_m > 0.03) {
    throw std::invalid_argument("sensor height hysteresis must be 0-0.03 m");
  }
  auto effective = configured;
  if (gate_open) {
    effective.minimum_sensor_height_m -= height_hysteresis_m;
    effective.maximum_sensor_height_m += height_hysteresis_m;
  }
  return effective;
}

struct GroundPlaneEstimate {
  bool valid = false;
  GroundPlaneFitStatus status =
      GroundPlaneFitStatus::kInsufficientConnectedSamples;
  double a = std::numeric_limits<double>::quiet_NaN();
  double b = std::numeric_limits<double>::quiet_NaN();
  double c = std::numeric_limits<double>::quiet_NaN();
  double sensor_height_m = std::numeric_limits<double>::quiet_NaN();
  double slope_deg = std::numeric_limits<double>::quiet_NaN();
  double rmse_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t sample_count = 0U;
};

inline const char* groundPlaneFitStatusName(GroundPlaneFitStatus status) {
  switch (status) {
    case GroundPlaneFitStatus::kValid:
      return "valid";
    case GroundPlaneFitStatus::kInsufficientConnectedSamples:
      return "insufficient_connected_samples";
    case GroundPlaneFitStatus::kDegenerateGeometry:
      return "degenerate_geometry";
    case GroundPlaneFitStatus::kFitFailure:
      return "fit_failure";
    case GroundPlaneFitStatus::kExcessiveResidual:
      return "excessive_residual";
    case GroundPlaneFitStatus::kSensorHeightBelowMinimum:
      return "sensor_height_below_minimum";
    case GroundPlaneFitStatus::kSensorHeightAboveMaximum:
      return "sensor_height_above_maximum";
  }
  return "unknown";
}

enum class TerrainFrameHealthClass {
  kHealthy,
  kSoftGeometryFailure,
  kHardFailure,
};

inline const char* terrainFrameHealthClassName(
    TerrainFrameHealthClass classification) {
  switch (classification) {
    case TerrainFrameHealthClass::kHealthy:
      return "healthy";
    case TerrainFrameHealthClass::kSoftGeometryFailure:
      return "soft_geometry_failure";
    case TerrainFrameHealthClass::kHardFailure:
      return "hard_failure";
  }
  return "unknown";
}

struct TerrainHealthHysteresisParameters {
  int opening_healthy_frames = 5;
  int maximum_soft_failure_frames = 3;
  double maximum_soft_failure_duration_sec = 0.25;
};

struct TerrainHealthHysteresisState {
  bool gate_open = false;
  int consecutive_healthy_frames = 0;
  int consecutive_soft_failure_frames = 0;
  double soft_failure_started_sec =
      std::numeric_limits<double>::quiet_NaN();
};

inline void forceTerrainHealthGateClosed(
    TerrainHealthHysteresisState* state) {
  if (state == nullptr) {
    throw std::invalid_argument("terrain health state must not be null");
  }
  state->gate_open = false;
  state->consecutive_healthy_frames = 0;
  state->consecutive_soft_failure_frames = 0;
  state->soft_failure_started_sec =
      std::numeric_limits<double>::quiet_NaN();
}

inline bool updateTerrainHealthHysteresis(
    TerrainFrameHealthClass classification, double now_sec,
    const TerrainHealthHysteresisParameters& parameters,
    TerrainHealthHysteresisState* state) {
  if (state == nullptr || !std::isfinite(now_sec) ||
      parameters.opening_healthy_frames < 1 ||
      parameters.maximum_soft_failure_frames < 0 ||
      parameters.maximum_soft_failure_duration_sec < 0.0) {
    throw std::invalid_argument("invalid terrain health hysteresis input");
  }
  if (classification == TerrainFrameHealthClass::kHardFailure) {
    forceTerrainHealthGateClosed(state);
    return false;
  }
  if (classification == TerrainFrameHealthClass::kHealthy) {
    state->consecutive_soft_failure_frames = 0;
    state->soft_failure_started_sec =
        std::numeric_limits<double>::quiet_NaN();
    if (!state->gate_open) {
      ++state->consecutive_healthy_frames;
      if (state->consecutive_healthy_frames >=
          parameters.opening_healthy_frames) {
        state->gate_open = true;
      }
    } else {
      state->consecutive_healthy_frames =
          parameters.opening_healthy_frames;
    }
    return state->gate_open;
  }

  state->consecutive_healthy_frames = 0;
  if (state->consecutive_soft_failure_frames == 0 ||
      !std::isfinite(state->soft_failure_started_sec) ||
      now_sec < state->soft_failure_started_sec) {
    state->soft_failure_started_sec = now_sec;
  }
  ++state->consecutive_soft_failure_frames;
  const double failure_duration = now_sec - state->soft_failure_started_sec;
  if (!state->gate_open ||
      state->consecutive_soft_failure_frames >
          parameters.maximum_soft_failure_frames ||
      failure_duration > parameters.maximum_soft_failure_duration_sec) {
    state->gate_open = false;
  }
  return state->gate_open;
}

inline double terrainSoftFailureAgeSec(
    const TerrainHealthHysteresisState& state, double now_sec) {
  if (!std::isfinite(now_sec) ||
      !std::isfinite(state.soft_failure_started_sec)) {
    return 0.0;
  }
  return std::max(0.0, now_sec - state.soft_failure_started_sec);
}

inline bool enforceTerrainHealthHysteresisDeadline(
    double now_sec, const TerrainHealthHysteresisParameters& parameters,
    TerrainHealthHysteresisState* state) {
  if (state == nullptr || !std::isfinite(now_sec) ||
      parameters.maximum_soft_failure_duration_sec < 0.0) {
    throw std::invalid_argument("invalid terrain health deadline input");
  }
  if (state->gate_open && state->consecutive_soft_failure_frames > 0 &&
      terrainSoftFailureAgeSec(*state, now_sec) >=
          parameters.maximum_soft_failure_duration_sec) {
    state->gate_open = false;
    state->consecutive_healthy_frames = 0;
  }
  return state->gate_open;
}

inline TerrainFrameHealthClass classifyTerrainFrameHealth(
    bool enough_input_points, bool enough_ground_points,
    bool enough_connected_area, bool enough_near_support,
    bool enough_sector_coverage, const GroundPlaneEstimate& plane,
    bool processing_within_deadline) {
  if (!enough_input_points || !processing_within_deadline) {
    return TerrainFrameHealthClass::kHardFailure;
  }
  if (!plane.valid) {
    switch (plane.status) {
      case GroundPlaneFitStatus::kFitFailure:
      case GroundPlaneFitStatus::kExcessiveResidual:
      case GroundPlaneFitStatus::kSensorHeightBelowMinimum:
      case GroundPlaneFitStatus::kSensorHeightAboveMaximum:
      case GroundPlaneFitStatus::kValid:
        return TerrainFrameHealthClass::kHardFailure;
      case GroundPlaneFitStatus::kInsufficientConnectedSamples:
      case GroundPlaneFitStatus::kDegenerateGeometry:
        break;
    }
  }
  if (!enough_ground_points || !enough_connected_area ||
      !enough_near_support || !enough_sector_coverage || !plane.valid) {
    return TerrainFrameHealthClass::kSoftGeometryFailure;
  }
  return TerrainFrameHealthClass::kHealthy;
}

struct SteepSurfaceParameters {
  // Patchwork++ may reject otherwise traversable ramps for reasons other than
  // its uprightness threshold. Cover the entire globally costed slope band so
  // those returns cannot become local marking obstacles.
  double minimum_slope_deg = 8.0;
  double maximum_slope_deg = 70.0;
  int minimum_run_cells = 3;
  int minimum_points_per_cell = 2;
  double maximum_step_delta_m = 0.04;
  double plane_huber_m = 0.02;
  double maximum_plane_rmse_m = 0.025;
  double maximum_point_residual_m = 0.05;
};

struct SurfaceSample {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline float robustLowSupportHeight(std::vector<float> heights,
                                    double candidate_quantile,
                                    double support_band) {
  heights.erase(
      std::remove_if(heights.begin(), heights.end(),
                     [](float value) { return !std::isfinite(value); }),
      heights.end());
  if (heights.empty() || candidate_quantile < 0.0 ||
      candidate_quantile > 1.0 || support_band <= 0.0) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  std::sort(heights.begin(), heights.end());
  const std::size_t candidate_index = static_cast<std::size_t>(std::floor(
      candidate_quantile * static_cast<double>(heights.size() - 1U)));
  const float support_limit =
      heights[candidate_index] + static_cast<float>(support_band);
  const auto support_end = std::upper_bound(heights.begin(), heights.end(),
                                             support_limit);
  const std::size_t support_count =
      static_cast<std::size_t>(support_end - heights.begin());
  const std::size_t middle = support_count / 2U;
  if (support_count % 2U != 0U) {
    return heights[middle];
  }
  return 0.5F * (heights[middle - 1U] + heights[middle]);
}

inline float finiteMedianHeight(std::vector<float> heights) {
  heights.erase(
      std::remove_if(heights.begin(), heights.end(),
                     [](float value) { return !std::isfinite(value); }),
      heights.end());
  if (heights.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const std::size_t middle = heights.size() / 2U;
  std::nth_element(heights.begin(), heights.begin() + middle, heights.end());
  if (heights.size() % 2U != 0U) {
    return heights[middle];
  }
  const float upper = heights[middle];
  std::nth_element(heights.begin(), heights.begin() + middle - 1U,
                   heights.begin() + middle);
  return 0.5F * (heights[middle - 1U] + upper);
}

inline std::vector<SurfaceSample> lowSupportSurfaceSamples(
    const std::vector<SurfaceSample>& samples, double candidate_quantile,
    double support_band, float* candidate_height) {
  if (candidate_height == nullptr || candidate_quantile < 0.0 ||
      candidate_quantile > 1.0 || support_band <= 0.0) {
    throw std::invalid_argument("invalid low-support surface parameters");
  }
  std::vector<float> heights;
  heights.reserve(samples.size());
  for (const SurfaceSample& sample : samples) {
    if (std::isfinite(sample.x) && std::isfinite(sample.y) &&
        std::isfinite(sample.z)) {
      heights.push_back(static_cast<float>(sample.z));
    }
  }
  *candidate_height =
      robustLowSupportHeight(heights, candidate_quantile, support_band);
  std::vector<SurfaceSample> result;
  if (!std::isfinite(*candidate_height)) {
    return result;
  }
  result.reserve(heights.size());
  for (const SurfaceSample& sample : samples) {
    if (std::isfinite(sample.x) && std::isfinite(sample.y) &&
        std::isfinite(sample.z) &&
        std::fabs(sample.z - static_cast<double>(*candidate_height)) <=
            support_band) {
      result.push_back(sample);
    }
  }
  return result;
}

inline std::vector<std::uint8_t> connectedGroundMask(
    const std::vector<float>& candidate_heights,
    const GroundConnectivityParameters& parameters) {
  if (parameters.width <= 0 || parameters.height <= 0 ||
      parameters.resolution <= 0.0 || parameters.sensor_height <= 0.0 ||
      parameters.anchor_radius <= 0.0 ||
      parameters.anchor_height_tolerance <= 0.0 ||
      parameters.anchor_seed_height_band <= 0.0 ||
      parameters.maximum_slope_deg <= 0.0 ||
      parameters.maximum_slope_deg > 35.0 || parameters.height_margin < 0.0) {
    throw std::invalid_argument("invalid connected-ground parameters");
  }
  const std::size_t expected_size =
      static_cast<std::size_t>(parameters.width * parameters.height);
  if (candidate_heights.size() != expected_size) {
    throw std::invalid_argument("connected-ground grid size mismatch");
  }

  std::vector<std::uint8_t> accepted(expected_size, 0U);
  std::queue<int> frontier;
  const double nominal_ground_z = -parameters.sensor_height;
  double best_anchor_error = std::numeric_limits<double>::infinity();
  for (int y = 0; y < parameters.height; ++y) {
    for (int x = 0; x < parameters.width; ++x) {
      const int index = y * parameters.width + x;
      const float height = candidate_heights[static_cast<std::size_t>(index)];
      if (!std::isfinite(height)) {
        continue;
      }
      const double center_x =
          parameters.origin_x + (static_cast<double>(x) + 0.5) *
                                    parameters.resolution;
      const double center_y =
          parameters.origin_y + (static_cast<double>(y) + 0.5) *
                                    parameters.resolution;
      if (std::hypot(center_x, center_y) <= parameters.anchor_radius &&
          std::fabs(static_cast<double>(height) - nominal_ground_z) <=
              parameters.anchor_height_tolerance) {
        best_anchor_error = std::min(
            best_anchor_error,
            std::fabs(static_cast<double>(height) - nominal_ground_z));
      }
    }
  }
  if (!std::isfinite(best_anchor_error)) {
    return accepted;
  }
  for (int y = 0; y < parameters.height; ++y) {
    for (int x = 0; x < parameters.width; ++x) {
      const int index = y * parameters.width + x;
      const float height = candidate_heights[static_cast<std::size_t>(index)];
      if (!std::isfinite(height)) {
        continue;
      }
      const double center_x =
          parameters.origin_x + (static_cast<double>(x) + 0.5) *
                                    parameters.resolution;
      const double center_y =
          parameters.origin_y + (static_cast<double>(y) + 0.5) *
                                    parameters.resolution;
      const double anchor_error =
          std::fabs(static_cast<double>(height) - nominal_ground_z);
      if (std::hypot(center_x, center_y) <= parameters.anchor_radius &&
          anchor_error <= parameters.anchor_height_tolerance &&
          anchor_error <= best_anchor_error +
                              parameters.anchor_seed_height_band) {
        accepted[static_cast<std::size_t>(index)] = 1U;
        frontier.push(index);
      }
    }
  }

  // Four-connectivity prevents a raised step corner from masquerading as a
  // longer diagonal run with an artificially lower grade.
  constexpr int kOffsets[4][2] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};
  const double maximum_grade =
      std::tan(parameters.maximum_slope_deg * 3.14159265358979323846 / 180.0);
  while (!frontier.empty()) {
    const int source_index = frontier.front();
    frontier.pop();
    const int source_x = source_index % parameters.width;
    const int source_y = source_index / parameters.width;
    const float source_height =
        candidate_heights[static_cast<std::size_t>(source_index)];
    for (const auto& offset : kOffsets) {
      const int target_x = source_x + offset[0];
      const int target_y = source_y + offset[1];
      if (target_x < 0 || target_x >= parameters.width || target_y < 0 ||
          target_y >= parameters.height) {
        continue;
      }
      const int target_index = target_y * parameters.width + target_x;
      if (accepted[static_cast<std::size_t>(target_index)] != 0U) {
        continue;
      }
      const float target_height =
          candidate_heights[static_cast<std::size_t>(target_index)];
      if (!std::isfinite(target_height)) {
        continue;
      }
      const double horizontal_distance =
          parameters.resolution * std::hypot(static_cast<double>(offset[0]),
                                              static_cast<double>(offset[1]));
      const double maximum_rise =
          maximum_grade * horizontal_distance + parameters.height_margin;
      if (std::fabs(static_cast<double>(target_height - source_height)) <=
          maximum_rise) {
        accepted[static_cast<std::size_t>(target_index)] = 1U;
        frontier.push(target_index);
      }
    }
  }
  return accepted;
}

inline SelectedGroundComponent selectGroundComponent(
    const std::vector<std::uint8_t>& accepted,
    const GroundConnectivityParameters& geometry,
    const GroundHealthParameters& parameters) {
  const std::size_t expected_size =
      static_cast<std::size_t>(geometry.width * geometry.height);
  if (geometry.width <= 0 || geometry.height <= 0 ||
      geometry.resolution <= 0.0 || accepted.size() != expected_size ||
      parameters.minimum_connected_area_m2 <= 0.0 ||
      parameters.near_support_radius_m <= 0.0 ||
      parameters.minimum_near_support_area_m2 <= 0.0 ||
      parameters.sector_count < 1 || parameters.sector_count > 360 ||
      parameters.minimum_covered_sectors < 1 ||
      parameters.minimum_covered_sectors > parameters.sector_count ||
      parameters.minimum_cells_per_sector < 1) {
    throw std::invalid_argument("invalid ground-health parameters");
  }

  std::vector<std::uint8_t> visited(expected_size, 0U);
  SelectedGroundComponent selected;
  selected.mask.assign(expected_size, 0U);
  constexpr int kOffsets[4][2] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};
  constexpr double kPi = 3.14159265358979323846;
  const double cell_area = geometry.resolution * geometry.resolution;

  for (int start = 0; start < geometry.width * geometry.height; ++start) {
    if (accepted[static_cast<std::size_t>(start)] == 0U ||
        visited[static_cast<std::size_t>(start)] != 0U) {
      continue;
    }
    std::queue<int> frontier;
    frontier.push(start);
    visited[static_cast<std::size_t>(start)] = 1U;
    GroundCoverageMetrics current;
    std::vector<int> component;
    std::vector<int> sector_cells(
        static_cast<std::size_t>(parameters.sector_count), 0);
    while (!frontier.empty()) {
      const int index = frontier.front();
      frontier.pop();
      component.push_back(index);
      ++current.connected_cells;
      const int x = index % geometry.width;
      const int y = index / geometry.width;
      const double center_x =
          geometry.origin_x + (static_cast<double>(x) + 0.5) *
                                  geometry.resolution;
      const double center_y =
          geometry.origin_y + (static_cast<double>(y) + 0.5) *
                                  geometry.resolution;
      const double range = std::hypot(center_x, center_y);
      if (range <= parameters.near_support_radius_m) {
        ++current.near_support_cells;
        if (range > geometry.resolution * 0.25) {
          const double angle = std::atan2(center_y, center_x) + kPi;
          int sector = static_cast<int>(std::floor(
              angle * parameters.sector_count / (2.0 * kPi)));
          sector = std::max(0, std::min(parameters.sector_count - 1, sector));
          ++sector_cells[static_cast<std::size_t>(sector)];
        }
      }

      for (const auto& offset : kOffsets) {
        const int neighbor_x = x + offset[0];
        const int neighbor_y = y + offset[1];
        if (neighbor_x < 0 || neighbor_x >= geometry.width ||
            neighbor_y < 0 || neighbor_y >= geometry.height) {
          continue;
        }
        const int neighbor = neighbor_y * geometry.width + neighbor_x;
        if (accepted[static_cast<std::size_t>(neighbor)] != 0U &&
            visited[static_cast<std::size_t>(neighbor)] == 0U) {
          visited[static_cast<std::size_t>(neighbor)] = 1U;
          frontier.push(neighbor);
        }
      }
    }
    current.connected_area_m2 = current.connected_cells * cell_area;
    current.near_support_area_m2 = current.near_support_cells * cell_area;
    current.covered_sectors = static_cast<int>(std::count_if(
        sector_cells.begin(), sector_cells.end(),
        [&parameters](int count) {
          return count >= parameters.minimum_cells_per_sector;
        }));
    // A component that never reaches the robot's near field is not permitted
    // to clear obstacles or provide a health estimate. Among robot-connected
    // candidates, prefer support closest to the robot, then angular diversity,
    // then total area. The final tie is deterministic because components are
    // visited in increasing flat-index order.
    const bool better =
        current.near_support_cells > selected.coverage.near_support_cells ||
        (current.near_support_cells ==
             selected.coverage.near_support_cells &&
         current.near_support_cells > 0U &&
         (current.covered_sectors > selected.coverage.covered_sectors ||
          (current.covered_sectors == selected.coverage.covered_sectors &&
           current.connected_cells > selected.coverage.connected_cells)));
    if (better) {
      std::fill(selected.mask.begin(), selected.mask.end(), 0U);
      for (const int index : component) {
        selected.mask[static_cast<std::size_t>(index)] = 1U;
      }
      selected.coverage = current;
    }
  }
  return selected;
}

inline GroundCoverageMetrics measureGroundCoverage(
    const std::vector<std::uint8_t>& accepted,
    const GroundConnectivityParameters& geometry,
    const GroundHealthParameters& parameters) {
  return selectGroundComponent(accepted, geometry, parameters).coverage;
}

inline bool healthyGroundCoverage(const GroundCoverageMetrics& metrics,
                                  const GroundHealthParameters& parameters) {
  return metrics.connected_area_m2 >= parameters.minimum_connected_area_m2 &&
         metrics.near_support_area_m2 >=
             parameters.minimum_near_support_area_m2 &&
         metrics.covered_sectors >= parameters.minimum_covered_sectors;
}

inline GroundPlaneEstimate estimateConnectedGroundPlane(
    const std::vector<float>& candidate_heights,
    const std::vector<std::uint8_t>& connected_ground,
    const GroundConnectivityParameters& geometry,
    const GroundPlaneFitParameters& parameters) {
  const std::size_t expected_size =
      static_cast<std::size_t>(geometry.width * geometry.height);
  if (geometry.width <= 0 || geometry.height <= 0 ||
      geometry.resolution <= 0.0 || candidate_heights.size() != expected_size ||
      connected_ground.size() != expected_size ||
      parameters.minimum_connected_samples < 3 ||
      parameters.maximum_radius_m <= 0.0 ||
      parameters.minimum_planar_variance_m2 <= 0.0 ||
      parameters.huber_delta_m <= 0.0 || parameters.maximum_rmse_m <= 0.0 ||
      parameters.irls_iterations < 1 ||
      parameters.minimum_sensor_height_m <= 0.0 ||
      parameters.maximum_sensor_height_m <
          parameters.minimum_sensor_height_m) {
    throw std::invalid_argument("invalid connected-ground plane parameters");
  }

  // The guard supplies its selected near-field component. Defensively retain
  // only the largest four-connected finite portion inside the fit radius so
  // a malformed caller cannot let a disconnected platform bias the height.
  std::vector<std::uint8_t> visited(expected_size, 0U);
  std::vector<int> largest_component;
  constexpr int kOffsets[4][2] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};
  auto inside_fit_radius = [&geometry, &parameters](int index) {
    const int x = index % geometry.width;
    const int y = index / geometry.width;
    const double center_x =
        geometry.origin_x +
        (static_cast<double>(x) + 0.5) * geometry.resolution;
    const double center_y =
        geometry.origin_y +
        (static_cast<double>(y) + 0.5) * geometry.resolution;
    return std::hypot(center_x, center_y) <= parameters.maximum_radius_m;
  };
  for (int start = 0; start < geometry.width * geometry.height; ++start) {
    const std::size_t start_index = static_cast<std::size_t>(start);
    if (connected_ground[start_index] == 0U || visited[start_index] != 0U ||
        !std::isfinite(candidate_heights[start_index]) ||
        !inside_fit_radius(start)) {
      continue;
    }
    std::vector<int> component;
    std::queue<int> frontier;
    frontier.push(start);
    visited[start_index] = 1U;
    while (!frontier.empty()) {
      const int index = frontier.front();
      frontier.pop();
      component.push_back(index);
      const int x = index % geometry.width;
      const int y = index / geometry.width;
      for (const auto& offset : kOffsets) {
        const int neighbor_x = x + offset[0];
        const int neighbor_y = y + offset[1];
        if (neighbor_x < 0 || neighbor_x >= geometry.width ||
            neighbor_y < 0 || neighbor_y >= geometry.height) {
          continue;
        }
        const int neighbor = neighbor_y * geometry.width + neighbor_x;
        const std::size_t neighbor_index =
            static_cast<std::size_t>(neighbor);
        if (connected_ground[neighbor_index] == 0U ||
            visited[neighbor_index] != 0U ||
            !std::isfinite(candidate_heights[neighbor_index]) ||
            !inside_fit_radius(neighbor)) {
          continue;
        }
        visited[neighbor_index] = 1U;
        frontier.push(neighbor);
      }
    }
    if (component.size() > largest_component.size()) {
      largest_component.swap(component);
    }
  }

  GroundPlaneEstimate estimate;
  estimate.sample_count = largest_component.size();
  if (largest_component.size() <
      static_cast<std::size_t>(parameters.minimum_connected_samples)) {
    estimate.status = GroundPlaneFitStatus::kInsufficientConnectedSamples;
    return estimate;
  }

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const int index : largest_component) {
    const int x = index % geometry.width;
    const int y = index / geometry.width;
    mean_x += geometry.origin_x +
              (static_cast<double>(x) + 0.5) * geometry.resolution;
    mean_y += geometry.origin_y +
              (static_cast<double>(y) + 0.5) * geometry.resolution;
  }
  mean_x /= static_cast<double>(largest_component.size());
  mean_y /= static_cast<double>(largest_component.size());

  double variance_x = 0.0;
  double variance_y = 0.0;
  double covariance_xy = 0.0;
  for (const int index : largest_component) {
    const int x = index % geometry.width;
    const int y = index / geometry.width;
    const double dx = geometry.origin_x +
                      (static_cast<double>(x) + 0.5) * geometry.resolution -
                      mean_x;
    const double dy = geometry.origin_y +
                      (static_cast<double>(y) + 0.5) * geometry.resolution -
                      mean_y;
    variance_x += dx * dx;
    variance_y += dy * dy;
    covariance_xy += dx * dy;
  }
  const double inverse_count =
      1.0 / static_cast<double>(largest_component.size());
  variance_x *= inverse_count;
  variance_y *= inverse_count;
  covariance_xy *= inverse_count;
  const double covariance_trace = variance_x + variance_y;
  const double covariance_discriminant = std::sqrt(std::max(
      0.0, (variance_x - variance_y) * (variance_x - variance_y) +
               4.0 * covariance_xy * covariance_xy));
  const double minimum_planar_variance =
      0.5 * (covariance_trace - covariance_discriminant);
  if (!std::isfinite(minimum_planar_variance) ||
      minimum_planar_variance < parameters.minimum_planar_variance_m2) {
    estimate.status = GroundPlaneFitStatus::kDegenerateGeometry;
    return estimate;
  }

  std::vector<double> weights(largest_component.size(), 1.0);
  Eigen::Vector3d centered_plane(0.0, 0.0, 0.0);
  for (int iteration = 0; iteration < parameters.irls_iterations;
       ++iteration) {
    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (std::size_t sample = 0; sample < largest_component.size();
         ++sample) {
      const int index = largest_component[sample];
      const int x = index % geometry.width;
      const int y = index / geometry.width;
      const double center_x =
          geometry.origin_x +
          (static_cast<double>(x) + 0.5) * geometry.resolution;
      const double center_y =
          geometry.origin_y +
          (static_cast<double>(y) + 0.5) * geometry.resolution;
      const Eigen::Vector3d basis(center_x - mean_x, center_y - mean_y, 1.0);
      normal.noalias() += weights[sample] * basis * basis.transpose();
      rhs.noalias() +=
          weights[sample] * basis * candidate_heights[static_cast<std::size_t>(index)];
    }
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(normal);
    if (decomposition.info() != Eigen::Success) {
      estimate.status = GroundPlaneFitStatus::kFitFailure;
      return estimate;
    }
    centered_plane = decomposition.solve(rhs);
    if (decomposition.info() != Eigen::Success ||
        !centered_plane.allFinite()) {
      estimate.status = GroundPlaneFitStatus::kFitFailure;
      return estimate;
    }
    for (std::size_t sample = 0; sample < largest_component.size();
         ++sample) {
      const int index = largest_component[sample];
      const int x = index % geometry.width;
      const int y = index / geometry.width;
      const double center_x =
          geometry.origin_x +
          (static_cast<double>(x) + 0.5) * geometry.resolution;
      const double center_y =
          geometry.origin_y +
          (static_cast<double>(y) + 0.5) * geometry.resolution;
      const double predicted =
          centered_plane.x() * (center_x - mean_x) +
          centered_plane.y() * (center_y - mean_y) + centered_plane.z();
      const double residual = std::fabs(
          static_cast<double>(candidate_heights[static_cast<std::size_t>(index)]) -
          predicted);
      weights[sample] = residual <= parameters.huber_delta_m
                            ? 1.0
                            : parameters.huber_delta_m / residual;
    }
  }

  estimate.a = centered_plane.x();
  estimate.b = centered_plane.y();
  estimate.c = centered_plane.z() - estimate.a * mean_x -
               estimate.b * mean_y;
  estimate.sensor_height_m = -estimate.c;
  constexpr double kPi = 3.14159265358979323846;
  estimate.slope_deg =
      std::atan(std::hypot(estimate.a, estimate.b)) * 180.0 / kPi;
  double weighted_squared_error = 0.0;
  double total_weight = 0.0;
  for (std::size_t sample = 0; sample < largest_component.size(); ++sample) {
    const int index = largest_component[sample];
    const int x = index % geometry.width;
    const int y = index / geometry.width;
    const double center_x =
        geometry.origin_x +
        (static_cast<double>(x) + 0.5) * geometry.resolution;
    const double center_y =
        geometry.origin_y +
        (static_cast<double>(y) + 0.5) * geometry.resolution;
    const double residual =
        static_cast<double>(candidate_heights[static_cast<std::size_t>(index)]) -
        (estimate.a * center_x + estimate.b * center_y + estimate.c);
    weighted_squared_error += weights[sample] * residual * residual;
    total_weight += weights[sample];
  }
  if (!std::isfinite(total_weight) || total_weight <= 0.0) {
    estimate.status = GroundPlaneFitStatus::kFitFailure;
    return estimate;
  }
  estimate.rmse_m = std::sqrt(weighted_squared_error / total_weight);
  if (!std::isfinite(estimate.sensor_height_m) ||
      !std::isfinite(estimate.slope_deg) || !std::isfinite(estimate.rmse_m)) {
    estimate.status = GroundPlaneFitStatus::kFitFailure;
    return estimate;
  }
  if (estimate.rmse_m > parameters.maximum_rmse_m) {
    estimate.status = GroundPlaneFitStatus::kExcessiveResidual;
    return estimate;
  }
  if (estimate.sensor_height_m < parameters.minimum_sensor_height_m) {
    estimate.status = GroundPlaneFitStatus::kSensorHeightBelowMinimum;
    return estimate;
  }
  if (estimate.sensor_height_m > parameters.maximum_sensor_height_m) {
    estimate.status = GroundPlaneFitStatus::kSensorHeightAboveMaximum;
    return estimate;
  }
  estimate.valid = true;
  estimate.status = GroundPlaneFitStatus::kValid;
  return estimate;
}

inline bool healthyTerrainOutputRate(std::size_t rate_samples,
                                     double output_rate_hz,
                                     int minimum_rate_samples,
                                     double minimum_output_rate_hz) {
  return minimum_rate_samples > 0 && minimum_output_rate_hz > 0.0 &&
         rate_samples >= static_cast<std::size_t>(minimum_rate_samples) &&
         std::isfinite(output_rate_hz) &&
         output_rate_hz >= minimum_output_rate_hz;
}

inline std::vector<std::uint8_t> continuousSteepSurfaceMask(
    const std::vector<float>& connected_ground_heights,
    const std::vector<std::uint8_t>& connected_ground,
    const std::vector<float>& nonground_candidate_heights,
    const std::vector<std::vector<SurfaceSample>>& nonground_samples,
    const GroundConnectivityParameters& geometry,
    const SteepSurfaceParameters& parameters) {
  const std::size_t expected_size =
      static_cast<std::size_t>(geometry.width * geometry.height);
  if (geometry.width <= 0 || geometry.height <= 0 ||
      geometry.resolution <= 0.0 ||
      connected_ground_heights.size() != expected_size ||
      connected_ground.size() != expected_size ||
      nonground_candidate_heights.size() != expected_size ||
      nonground_samples.size() != expected_size ||
      parameters.minimum_slope_deg <= 0.0 ||
      parameters.maximum_slope_deg <= parameters.minimum_slope_deg ||
      parameters.maximum_slope_deg >= 89.0 ||
      parameters.minimum_run_cells < 2 ||
      parameters.minimum_points_per_cell < 1 ||
      parameters.maximum_step_delta_m < 0.0 ||
      parameters.plane_huber_m <= 0.0 ||
      parameters.maximum_plane_rmse_m <= 0.0 ||
      parameters.maximum_point_residual_m <= 0.0) {
    throw std::invalid_argument("invalid steep-surface parameters");
  }

  std::vector<std::uint8_t> result(expected_size, 0U);
  constexpr int kDirections[8][2] = {
      {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
      {1, 0},   {-1, 1}, {0, 1},  {1, 1}};
  constexpr double kPi = 3.14159265358979323846;
  const double minimum_grade =
      std::tan(parameters.minimum_slope_deg * kPi / 180.0);
  const double maximum_grade =
      std::tan(parameters.maximum_slope_deg * kPi / 180.0);
  const double minimum_directional_grade =
      minimum_grade * std::cos(kPi / 8.0);
  constexpr double kTolerance = 1e-6;

  auto ground_like_fit = [&geometry, &connected_ground_heights,
                          &nonground_samples, &parameters, minimum_grade,
                          maximum_grade](int source,
                                         const std::vector<int>& run) {
    const int source_x = source % geometry.width;
    const int source_y = source / geometry.width;
    const double source_center_x =
        geometry.origin_x + (static_cast<double>(source_x) + 0.5) *
                                geometry.resolution;
    const double source_center_y =
        geometry.origin_y + (static_cast<double>(source_y) + 0.5) *
                                geometry.resolution;
    std::vector<double> local_x = {0.0};
    std::vector<double> local_y = {0.0};
    std::vector<double> height = {
        connected_ground_heights[static_cast<std::size_t>(source)]};
    for (const int index : run) {
      const auto& samples =
          nonground_samples[static_cast<std::size_t>(index)];
      if (samples.size() <
          static_cast<std::size_t>(parameters.minimum_points_per_cell)) {
        return false;
      }
      for (const SurfaceSample& sample : samples) {
        if (!std::isfinite(sample.x) || !std::isfinite(sample.y) ||
            !std::isfinite(sample.z)) {
          continue;
        }
        local_x.push_back(sample.x - source_center_x);
        local_y.push_back(sample.y - source_center_y);
        height.push_back(sample.z);
      }
    }
    if (height.size() <
        1U + run.size() *
                 static_cast<std::size_t>(parameters.minimum_points_per_cell)) {
      return false;
    }

    std::vector<double> weights(height.size(), 1.0);
    Eigen::Vector3d plane(0.0, 0.0, height.front());
    for (int iteration = 0; iteration < 4; ++iteration) {
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      for (std::size_t i = 0; i < height.size(); ++i) {
        const Eigen::Vector3d basis(local_x[i], local_y[i], 1.0);
        normal.noalias() += weights[i] * basis * basis.transpose();
        rhs.noalias() += weights[i] * basis * height[i];
      }
      const Eigen::LDLT<Eigen::Matrix3d> decomposition(normal);
      if (decomposition.info() != Eigen::Success) {
        return false;
      }
      plane = decomposition.solve(rhs);
      if (decomposition.info() != Eigen::Success || !plane.allFinite()) {
        return false;
      }
      for (std::size_t i = 0; i < height.size(); ++i) {
        const double residual =
            std::fabs(height[i] -
                      (plane.x() * local_x[i] + plane.y() * local_y[i] +
                       plane.z()));
        weights[i] = residual <= parameters.plane_huber_m
                         ? 1.0
                         : parameters.plane_huber_m / residual;
      }
    }
    const double fitted_grade = std::hypot(plane.x(), plane.y());
    if (fitted_grade + kTolerance < minimum_grade ||
        fitted_grade > maximum_grade + kTolerance) {
      return false;
    }
    double squared_error = 0.0;
    double maximum_residual = 0.0;
    for (std::size_t i = 0; i < height.size(); ++i) {
      const double residual =
          std::fabs(height[i] -
                    (plane.x() * local_x[i] + plane.y() * local_y[i] +
                     plane.z()));
      squared_error += residual * residual;
      maximum_residual = std::max(maximum_residual, residual);
    }
    return std::sqrt(squared_error / static_cast<double>(height.size())) <=
               parameters.maximum_plane_rmse_m &&
           maximum_residual <= parameters.maximum_point_residual_m;
  };

  for (int source = 0; source < geometry.width * geometry.height; ++source) {
    if (connected_ground[static_cast<std::size_t>(source)] == 0U ||
        !std::isfinite(
            connected_ground_heights[static_cast<std::size_t>(source)])) {
      continue;
    }
    const int source_x = source % geometry.width;
    const int source_y = source / geometry.width;
    for (const auto& direction : kDirections) {
      const double sample_spacing = geometry.resolution * std::hypot(
          static_cast<double>(direction[0]),
          static_cast<double>(direction[1]));
      const double minimum_rise = minimum_directional_grade * sample_spacing;
      const double maximum_rise = maximum_grade * sample_spacing;
      std::vector<int> run;
      double previous_height =
          connected_ground_heights[static_cast<std::size_t>(source)];
      double previous_rise = std::numeric_limits<double>::quiet_NaN();
      int rise_sign = 0;
      for (int step = 1;; ++step) {
        const int x = source_x + direction[0] * step;
        const int y = source_y + direction[1] * step;
        if (x < 0 || x >= geometry.width || y < 0 || y >= geometry.height) {
          break;
        }
        const int index = y * geometry.width + x;
        if (connected_ground[static_cast<std::size_t>(index)] != 0U) {
          break;
        }
        const float height =
            nonground_candidate_heights[static_cast<std::size_t>(index)];
        if (!std::isfinite(height) ||
            nonground_samples[static_cast<std::size_t>(index)].size() <
                static_cast<std::size_t>(parameters.minimum_points_per_cell)) {
          break;
        }
        const double signed_rise = static_cast<double>(height) - previous_height;
        const double rise = std::fabs(signed_rise);
        if (rise + kTolerance < minimum_rise ||
            rise > maximum_rise + kTolerance) {
          break;
        }
        const int current_sign = signed_rise > 0.0 ? 1 : -1;
        if ((rise_sign != 0 && current_sign != rise_sign) ||
            (std::isfinite(previous_rise) &&
             std::fabs(rise - previous_rise) >
                 parameters.maximum_step_delta_m + kTolerance)) {
          break;
        }
        rise_sign = current_sign;
        previous_rise = rise;
        previous_height = height;
        run.push_back(index);
      }
      if (run.size() >=
              static_cast<std::size_t>(parameters.minimum_run_cells) &&
          ground_like_fit(source, run)) {
        for (const int index : run) {
          result[static_cast<std::size_t>(index)] = 1U;
        }
      }
    }
  }
  return result;
}

inline bool validSensorHeight(double sensor_height,
                              double minimum_height,
                              double maximum_height) {
  return std::isfinite(sensor_height) &&
         std::isfinite(minimum_height) &&
         std::isfinite(maximum_height) &&
         minimum_height <= maximum_height &&
         sensor_height >= minimum_height &&
         sensor_height <= maximum_height;
}

inline NongroundClass classifyNonground(
    double point_z, bool has_ground, double ground_z,
    const NongroundThresholds& thresholds) {
  if (!has_ground) {
    return point_z > thresholds.no_ground_high_split_z
               ? NongroundClass::kUnknownHigh
               : NongroundClass::kUnknownLow;
  }
  const double relative_height = point_z - ground_z;
  // Decimal metre thresholds such as 0.05 are not represented exactly in a
  // binary double. Keep the documented inclusive envelope at both boundaries.
  constexpr double kBoundaryEpsilon = 1e-9;
  if (relative_height + kBoundaryEpsilon < thresholds.min_relative_height) {
    return NongroundClass::kUnknownLow;
  }
  if (relative_height - kBoundaryEpsilon > thresholds.max_relative_height) {
    return NongroundClass::kUnknownHigh;
  }
  return NongroundClass::kObstacle;
}

inline NongroundClass classifyIgnoredSteepSurface(
    double point_z, bool has_ground, double ground_z,
    const NongroundThresholds& thresholds) {
  const NongroundClass classification =
      classifyNonground(point_z, has_ground, ground_z, thresholds);
  // A surface is ignored only after the point-level continuity and plane-fit
  // checks identify it as ground-like. It must never become a local hard
  // obstacle, regardless of which Patchwork output stream supplied it.
  return classification == NongroundClass::kUnknownHigh
             ? NongroundClass::kUnknownHigh
             : NongroundClass::kUnknownLow;
}

inline NongroundClass classifyNongroundForLocalCostmap(
    double point_z, bool has_ground, double ground_z,
    bool continuous_steep_surface,
    const NongroundThresholds& thresholds) {
  return continuous_steep_surface
             ? classifyIgnoredSteepSurface(point_z, has_ground, ground_z,
                                           thresholds)
             : classifyNonground(point_z, has_ground, ground_z, thresholds);
}

inline Eigen::Quaterniond yawOnly(const Eigen::Quaterniond& orientation) {
  Eigen::Quaterniond normalized = orientation;
  if (normalized.norm() < 1e-9) {
    return Eigen::Quaterniond::Identity();
  }
  normalized.normalize();
  const double sin_yaw = 2.0 *
      (normalized.w() * normalized.z() +
       normalized.x() * normalized.y());
  const double cos_yaw = 1.0 - 2.0 *
      (normalized.y() * normalized.y() +
       normalized.z() * normalized.z());
  const double yaw = std::atan2(sin_yaw, cos_yaw);
  return Eigen::Quaterniond(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

}  // namespace go2_terrain
