#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "go2_terrain/terrain_model.hpp"

namespace gt = go2_terrain;

namespace {

gt::GroundConnectivityParameters coverageGeometry() {
  gt::GroundConnectivityParameters result;
  result.width = 21;
  result.height = 21;
  result.resolution = 0.15;
  result.origin_x = -1.575;
  result.origin_y = -1.575;
  return result;
}

std::size_t coverageIndex(int x, int y,
                          const gt::GroundConnectivityParameters& geometry) {
  return static_cast<std::size_t>(y * geometry.width + x);
}

std::vector<std::uint8_t> annularGroundMask(
    const gt::GroundConnectivityParameters& geometry) {
  std::vector<std::uint8_t> mask(
      static_cast<std::size_t>(geometry.width * geometry.height), 0U);
  for (int y = 0; y < geometry.height; ++y) {
    for (int x = 0; x < geometry.width; ++x) {
      const double center_x =
          geometry.origin_x + (static_cast<double>(x) + 0.5) *
                                  geometry.resolution;
      const double center_y =
          geometry.origin_y + (static_cast<double>(y) + 0.5) *
                                  geometry.resolution;
      const double range = std::hypot(center_x, center_y);
      if (range >= 0.35 && range <= 1.40) {
        mask[coverageIndex(x, y, geometry)] = 1U;
      }
    }
  }
  return mask;
}

std::vector<float> planarCandidateHeights(
    const gt::GroundConnectivityParameters& geometry,
    const std::vector<std::uint8_t>& mask, double sensor_height,
    double slope_deg, double azimuth_deg = 0.0) {
  const double pi = 3.14159265358979323846;
  const double grade = std::tan(slope_deg * pi / 180.0);
  const double azimuth = azimuth_deg * pi / 180.0;
  const double gradient_x = grade * std::cos(azimuth);
  const double gradient_y = grade * std::sin(azimuth);
  std::vector<float> candidates(
      mask.size(), std::numeric_limits<float>::quiet_NaN());
  for (int y = 0; y < geometry.height; ++y) {
    for (int x = 0; x < geometry.width; ++x) {
      const std::size_t index = coverageIndex(x, y, geometry);
      if (mask[index] == 0U) {
        continue;
      }
      const double center_x =
          geometry.origin_x +
          (static_cast<double>(x) + 0.5) * geometry.resolution;
      const double center_y =
          geometry.origin_y +
          (static_cast<double>(y) + 0.5) * geometry.resolution;
      candidates[index] = static_cast<float>(
          -sensor_height + gradient_x * center_x + gradient_y * center_y);
    }
  }
  return candidates;
}

void addPlanarCellSamples(
    const gt::GroundConnectivityParameters& geometry,
    int x, int y, double reference_x, double reference_y,
    double reference_z, double gradient_x, double gradient_y,
    std::vector<std::vector<gt::SurfaceSample>>* samples) {
  const double center_x =
      geometry.origin_x + (static_cast<double>(x) + 0.5) *
                              geometry.resolution;
  const double center_y =
      geometry.origin_y + (static_cast<double>(y) + 0.5) *
                              geometry.resolution;
  const double norm = std::hypot(gradient_x, gradient_y);
  const double unit_x = gradient_x / norm;
  const double unit_y = gradient_y / norm;
  const double lateral_x = -unit_y;
  const double lateral_y = unit_x;
  const double offsets[5][2] = {
      {0.0, 0.0},
      {0.05 * unit_x, 0.05 * unit_y},
      {-0.05 * unit_x, -0.05 * unit_y},
      {0.04 * lateral_x, 0.04 * lateral_y},
      {-0.04 * lateral_x, -0.04 * lateral_y}};
  auto& cell = (*samples)[coverageIndex(x, y, geometry)];
  for (const auto& offset : offsets) {
    const double point_x = center_x + offset[0];
    const double point_y = center_y + offset[1];
    cell.push_back({point_x, point_y,
                    reference_z + gradient_x * (point_x - reference_x) +
                        gradient_y * (point_y - reference_y)});
  }
}

void addFlatCellSamples(
    const gt::GroundConnectivityParameters& geometry,
    int x, int y, double z,
    std::vector<std::vector<gt::SurfaceSample>>* samples) {
  const double center_x =
      geometry.origin_x + (static_cast<double>(x) + 0.5) *
                              geometry.resolution;
  const double center_y =
      geometry.origin_y + (static_cast<double>(y) + 0.5) *
                              geometry.resolution;
  auto& cell = (*samples)[coverageIndex(x, y, geometry)];
  cell.push_back({center_x - 0.03, center_y, z});
  cell.push_back({center_x, center_y, z});
  cell.push_back({center_x + 0.03, center_y, z});
  cell.push_back({center_x, center_y - 0.03, z});
  cell.push_back({center_x, center_y + 0.03, z});
}

}  // namespace

TEST(TerrainModel, AcceptsObstacleRelativeToGround) {
  gt::NongroundThresholds thresholds;
  EXPECT_EQ(gt::NongroundClass::kObstacle,
            gt::classifyNonground(-0.20, true, -0.44, thresholds));
}

TEST(TerrainModel, RejectsCeilingWithGroundReference) {
  gt::NongroundThresholds thresholds;
  EXPECT_EQ(gt::NongroundClass::kUnknownHigh,
            gt::classifyNonground(1.10, true, -0.44, thresholds));
}

TEST(TerrainModel, UsesInclusiveObstacleHeightEnvelope) {
  gt::NongroundThresholds thresholds;
  EXPECT_EQ(gt::NongroundClass::kObstacle,
            gt::classifyNonground(-0.46, true, -0.51, thresholds));
  EXPECT_EQ(gt::NongroundClass::kUnknownLow,
            gt::classifyNonground(-0.461, true, -0.51, thresholds));
  EXPECT_EQ(gt::NongroundClass::kObstacle,
            gt::classifyNonground(0.99, true, -0.51, thresholds));
  EXPECT_EQ(gt::NongroundClass::kUnknownHigh,
            gt::classifyNonground(0.991, true, -0.51, thresholds));
}

TEST(TerrainModel, RejectsHighReturnWithoutGroundReference) {
  gt::NongroundThresholds thresholds;
  EXPECT_EQ(gt::NongroundClass::kUnknownHigh,
            gt::classifyNonground(1.80, false, 0.0, thresholds));
}

TEST(TerrainModel, NeverMarksSlopeWithoutGroundReference) {
  gt::NongroundThresholds thresholds;
  EXPECT_NE(gt::NongroundClass::kObstacle,
            gt::classifyNonground(-0.15, false, 0.0, thresholds));
  EXPECT_NE(gt::NongroundClass::kObstacle,
            gt::classifyNonground(0.25, false, 0.0, thresholds));
}

TEST(TerrainModel, ValidatesGo2SensorHeightEnvelope) {
  EXPECT_TRUE(gt::validSensorHeight(0.51, 0.43, 0.59));
  EXPECT_TRUE(gt::validSensorHeight(0.43, 0.43, 0.59));
  EXPECT_TRUE(gt::validSensorHeight(0.59, 0.43, 0.59));
  EXPECT_FALSE(gt::validSensorHeight(0.42, 0.43, 0.59));
  EXPECT_FALSE(gt::validSensorHeight(0.60, 0.43, 0.59));
}

TEST(TerrainModel, EstimatesNominalSensorHeightFromFlatConnectedGround) {
  const gt::GroundConnectivityParameters geometry = coverageGeometry();
  const std::vector<std::uint8_t> mask = annularGroundMask(geometry);
  const std::vector<float> candidates =
      planarCandidateHeights(geometry, mask, 0.51, 0.0);
  const gt::GroundPlaneEstimate estimate =
      gt::estimateConnectedGroundPlane(
          candidates, mask, geometry, gt::GroundPlaneFitParameters());
  ASSERT_TRUE(estimate.valid);
  EXPECT_EQ(gt::GroundPlaneFitStatus::kValid, estimate.status);
  EXPECT_NEAR(0.51, estimate.sensor_height_m, 1e-6);
  EXPECT_NEAR(0.0, estimate.slope_deg, 1e-6);
  EXPECT_NEAR(0.0, estimate.rmse_m, 1e-6);
  EXPECT_GE(estimate.sample_count, 12U);
}

TEST(TerrainModel, EstimatesNominalHeightOnTwentyAndThirtyDegreeRamps) {
  const gt::GroundConnectivityParameters geometry = coverageGeometry();
  const std::vector<std::uint8_t> mask = annularGroundMask(geometry);
  for (const double slope_deg : {20.0, 30.0}) {
    const std::vector<float> candidates =
        planarCandidateHeights(geometry, mask, 0.51, slope_deg, 27.0);
    const gt::GroundPlaneEstimate estimate =
        gt::estimateConnectedGroundPlane(
            candidates, mask, geometry, gt::GroundPlaneFitParameters());
    ASSERT_TRUE(estimate.valid) << "slope_deg=" << slope_deg;
    EXPECT_NEAR(0.51, estimate.sensor_height_m, 1e-5)
        << "slope_deg=" << slope_deg;
    EXPECT_NEAR(slope_deg, estimate.slope_deg, 1e-5)
        << "slope_deg=" << slope_deg;
    EXPECT_LT(estimate.rmse_m, 1e-5) << "slope_deg=" << slope_deg;
  }
}

TEST(TerrainModel, SensorHeightFitIgnoresConnectedTerrainBeyondLocalRadius) {
  const gt::GroundConnectivityParameters geometry = coverageGeometry();
  std::vector<std::uint8_t> mask(
      static_cast<std::size_t>(geometry.width * geometry.height), 1U);
  std::vector<float> candidates(mask.size(), -0.51F);
  std::size_t local_samples = 0U;
  for (int y = 0; y < geometry.height; ++y) {
    for (int x = 0; x < geometry.width; ++x) {
      const double center_x =
          geometry.origin_x +
          (static_cast<double>(x) + 0.5) * geometry.resolution;
      const double center_y =
          geometry.origin_y +
          (static_cast<double>(y) + 0.5) * geometry.resolution;
      const std::size_t index = coverageIndex(x, y, geometry);
      if (std::hypot(center_x, center_y) <= 1.50) {
        ++local_samples;
      } else {
        candidates[index] = 0.75F;
      }
    }
  }
  const gt::GroundPlaneEstimate estimate =
      gt::estimateConnectedGroundPlane(
          candidates, mask, geometry, gt::GroundPlaneFitParameters());
  ASSERT_TRUE(estimate.valid);
  EXPECT_EQ(local_samples, estimate.sample_count);
  EXPECT_NEAR(0.51, estimate.sensor_height_m, 1e-6);
  EXPECT_NEAR(0.0, estimate.slope_deg, 1e-6);
}

TEST(TerrainModel, RejectsLowStanceFromMeasuredSensorHeight) {
  const gt::GroundConnectivityParameters geometry = coverageGeometry();
  const std::vector<std::uint8_t> mask = annularGroundMask(geometry);
  const std::vector<float> candidates =
      planarCandidateHeights(geometry, mask, 0.31, 0.0);
  const gt::GroundPlaneEstimate estimate =
      gt::estimateConnectedGroundPlane(
          candidates, mask, geometry, gt::GroundPlaneFitParameters());
  EXPECT_FALSE(estimate.valid);
  EXPECT_EQ(gt::GroundPlaneFitStatus::kSensorHeightBelowMinimum,
            estimate.status);
  EXPECT_NEAR(0.31, estimate.sensor_height_m, 1e-6);
}

TEST(TerrainModel, LowStanceReachesPhysicalHeightGateEndToEnd) {
  gt::GroundConnectivityParameters geometry = coverageGeometry();
  geometry.sensor_height = 0.51;
  geometry.anchor_radius = 1.20;
  ASSERT_DOUBLE_EQ(0.25, geometry.anchor_height_tolerance);
  const std::vector<std::uint8_t> floor_support =
      annularGroundMask(geometry);
  const std::vector<float> candidates =
      planarCandidateHeights(geometry, floor_support, 0.31, 0.0);

  const std::vector<std::uint8_t> connected_candidates =
      gt::connectedGroundMask(candidates, geometry);
  const gt::GroundHealthParameters ground_health;
  const gt::SelectedGroundComponent selected =
      gt::selectGroundComponent(connected_candidates, geometry,
                                ground_health);
  ASSERT_GT(selected.coverage.connected_cells, 0U);
  ASSERT_TRUE(gt::healthyGroundCoverage(selected.coverage, ground_health));

  const gt::GroundPlaneEstimate estimate =
      gt::estimateConnectedGroundPlane(
          candidates, selected.mask, geometry,
          gt::GroundPlaneFitParameters());
  ASSERT_FALSE(estimate.valid);
  EXPECT_EQ(gt::GroundPlaneFitStatus::kSensorHeightBelowMinimum,
            estimate.status);
  EXPECT_NEAR(0.31, estimate.sensor_height_m, 1e-6);
  EXPECT_EQ(gt::TerrainFrameHealthClass::kHardFailure,
            gt::classifyTerrainFrameHealth(
                true, true, true, true, true, estimate, true));
}

TEST(TerrainModel, RejectsGroundSplitIntoUndersizedComponents) {
  gt::GroundConnectivityParameters geometry;
  geometry.width = 10;
  geometry.height = 4;
  geometry.resolution = 0.15;
  std::vector<std::uint8_t> mask(40U, 0U);
  std::vector<float> candidates(
      40U, std::numeric_limits<float>::quiet_NaN());
  for (int y = 1; y <= 2; ++y) {
    for (const int x : {0, 1, 2, 7, 8, 9}) {
      const std::size_t index = coverageIndex(x, y, geometry);
      mask[index] = 1U;
      candidates[index] = -0.51F;
    }
  }
  gt::GroundPlaneFitParameters parameters;
  parameters.minimum_connected_samples = 7;
  const gt::GroundPlaneEstimate estimate =
      gt::estimateConnectedGroundPlane(candidates, mask, geometry,
                                       parameters);
  EXPECT_FALSE(estimate.valid);
  EXPECT_EQ(gt::GroundPlaneFitStatus::kInsufficientConnectedSamples,
            estimate.status);
  EXPECT_EQ(6U, estimate.sample_count);
}

TEST(TerrainModel, RejectsTooFewConnectedGroundSamples) {
  gt::GroundConnectivityParameters geometry;
  geometry.width = 4;
  geometry.height = 4;
  geometry.resolution = 0.15;
  std::vector<std::uint8_t> mask(16U, 0U);
  std::vector<float> candidates(
      16U, std::numeric_limits<float>::quiet_NaN());
  for (int y = 1; y <= 2; ++y) {
    for (int x = 1; x <= 2; ++x) {
      const std::size_t index = coverageIndex(x, y, geometry);
      mask[index] = 1U;
      candidates[index] = -0.51F;
    }
  }
  const gt::GroundPlaneEstimate estimate =
      gt::estimateConnectedGroundPlane(
          candidates, mask, geometry, gt::GroundPlaneFitParameters());
  EXPECT_FALSE(estimate.valid);
  EXPECT_EQ(gt::GroundPlaneFitStatus::kInsufficientConnectedSamples,
            estimate.status);
  EXPECT_EQ(4U, estimate.sample_count);
}

TEST(TerrainModel, RecordedRampHeightsPassOnlyAfterHealthyStartup) {
  const auto geometry = coverageGeometry();
  const auto mask = annularGroundMask(geometry);
  const gt::GroundPlaneFitParameters configured;
  for (double height : {0.429955, 0.4216, 0.420934, 0.60}) {
    const auto cloud = planarCandidateHeights(geometry, mask, height, 6.1);
    EXPECT_FALSE(gt::estimateConnectedGroundPlane(cloud, mask, geometry,
        gt::groundPlaneFitForHealthGate(configured, false, 0.02)).valid);
    EXPECT_TRUE(gt::estimateConnectedGroundPlane(cloud, mask, geometry,
        gt::groundPlaneFitForHealthGate(configured, true, 0.02)).valid);
  }
}

TEST(TerrainModel, HeightHysteresisStillRejectsUnsafeStanceAndBadPlane) {
  const auto geometry = coverageGeometry();
  const auto mask = annularGroundMask(geometry);
  const auto fit = gt::groundPlaneFitForHealthGate(
      gt::GroundPlaneFitParameters(), true, 0.02);
  for (double height : {0.31, 0.39, 0.63}) {
    const auto cloud = planarCandidateHeights(geometry, mask, height, 6.1);
    const auto plane = gt::estimateConnectedGroundPlane(cloud, mask, geometry, fit);
    EXPECT_FALSE(plane.valid);
    EXPECT_EQ(gt::TerrainFrameHealthClass::kHardFailure,
        gt::classifyTerrainFrameHealth(true, true, true, true, true, plane, true));
  }
  auto cloud = planarCandidateHeights(geometry, mask, 0.45, 6.1);
  for (std::size_t i=0; i<cloud.size(); ++i)
    if (mask[i]) cloud[i] += (i%2 ? 0.09f : -0.09f);
  const auto plane = gt::estimateConnectedGroundPlane(cloud, mask, geometry, fit);
  EXPECT_FALSE(plane.valid);
  EXPECT_EQ(gt::GroundPlaneFitStatus::kExcessiveResidual, plane.status);
}

TEST(TerrainModel, HardFailureRequiresNominalHeightBeforeReopening) {
  const auto geometry = coverageGeometry();
  const auto mask = annularGroundMask(geometry);
  const gt::GroundPlaneFitParameters configured;
  const gt::TerrainHealthHysteresisParameters hysteresis;
  gt::TerrainHealthHysteresisState state;
  double time=0.0;
  const auto frame = [&](double height) {
    const auto plane = gt::estimateConnectedGroundPlane(
        planarCandidateHeights(geometry, mask, height, 6.1), mask, geometry,
        gt::groundPlaneFitForHealthGate(configured, state.gate_open, 0.02));
    time += 0.1;
    return gt::updateTerrainHealthHysteresis(
        gt::classifyTerrainFrameHealth(true, true, true, true, true, plane, true),
        time, hysteresis, &state);
  };
  for (int i=0;i<4;++i) EXPECT_FALSE(frame(0.51));
  EXPECT_TRUE(frame(0.51));
  for (int i=0;i<20;++i) EXPECT_TRUE(frame(0.4216));
  EXPECT_FALSE(frame(0.39));
  for (int i=0;i<10;++i) EXPECT_FALSE(frame(0.4216));
  for (int i=0;i<4;++i) EXPECT_FALSE(frame(0.51));
  EXPECT_TRUE(frame(0.51));
}

TEST(TerrainModel, InvalidHeightHysteresisIsRejected) {
  const gt::GroundPlaneFitParameters fit;
  for (double margin : {-0.01, 0.031, std::numeric_limits<double>::quiet_NaN()})
    EXPECT_THROW(gt::groundPlaneFitForHealthGate(fit, true, margin),
                 std::invalid_argument);
  const auto strict = gt::groundPlaneFitForHealthGate(fit, true, 0.0);
  EXPECT_DOUBLE_EQ(fit.minimum_sensor_height_m, strict.minimum_sensor_height_m);
}

TEST(TerrainModel, TerrainHealthGateNeedsFiveConsecutiveHealthyFrames) {
  gt::TerrainHealthHysteresisParameters parameters;
  gt::TerrainHealthHysteresisState state;
  for (int frame = 0; frame < 4; ++frame) {
    EXPECT_FALSE(gt::updateTerrainHealthHysteresis(
        gt::TerrainFrameHealthClass::kHealthy, frame * 0.05, parameters,
        &state));
  }
  EXPECT_EQ(4, state.consecutive_healthy_frames);
  EXPECT_TRUE(gt::updateTerrainHealthHysteresis(
      gt::TerrainFrameHealthClass::kHealthy, 0.20, parameters, &state));
  EXPECT_TRUE(state.gate_open);
}

TEST(TerrainModel, OpenGateHoldsOnlyThreeShortGeometryFailures) {
  gt::TerrainHealthHysteresisParameters parameters;
  gt::TerrainHealthHysteresisState state;
  for (int frame = 0; frame < parameters.opening_healthy_frames; ++frame) {
    gt::updateTerrainHealthHysteresis(
        gt::TerrainFrameHealthClass::kHealthy, frame * 0.05, parameters,
        &state);
  }
  ASSERT_TRUE(state.gate_open);
  EXPECT_TRUE(gt::updateTerrainHealthHysteresis(
      gt::TerrainFrameHealthClass::kSoftGeometryFailure, 1.00, parameters,
      &state));
  EXPECT_TRUE(gt::updateTerrainHealthHysteresis(
      gt::TerrainFrameHealthClass::kSoftGeometryFailure, 1.10, parameters,
      &state));
  EXPECT_TRUE(gt::updateTerrainHealthHysteresis(
      gt::TerrainFrameHealthClass::kSoftGeometryFailure, 1.20, parameters,
      &state));
  EXPECT_FALSE(gt::updateTerrainHealthHysteresis(
      gt::TerrainFrameHealthClass::kSoftGeometryFailure, 1.21, parameters,
      &state));
  EXPECT_EQ(4, state.consecutive_soft_failure_frames);
}

TEST(TerrainModel, SoftGeometryHoldAlsoHonorsQuarterSecondLimit) {
  gt::TerrainHealthHysteresisParameters parameters;
  gt::TerrainHealthHysteresisState state;
  for (int frame = 0; frame < parameters.opening_healthy_frames; ++frame) {
    gt::updateTerrainHealthHysteresis(
        gt::TerrainFrameHealthClass::kHealthy, frame * 0.05, parameters,
        &state);
  }
  ASSERT_TRUE(gt::updateTerrainHealthHysteresis(
      gt::TerrainFrameHealthClass::kSoftGeometryFailure, 1.00, parameters,
      &state));
  EXPECT_FALSE(gt::updateTerrainHealthHysteresis(
      gt::TerrainFrameHealthClass::kSoftGeometryFailure, 1.251, parameters,
      &state));
}

TEST(TerrainModel, SoftGeometryDeadlineExpiresWithoutAnotherFrame) {
  gt::TerrainHealthHysteresisParameters parameters;
  gt::TerrainHealthHysteresisState state;
  for (int frame = 0; frame < parameters.opening_healthy_frames; ++frame) {
    gt::updateTerrainHealthHysteresis(
        gt::TerrainFrameHealthClass::kHealthy, frame * 0.05, parameters,
        &state);
  }
  ASSERT_TRUE(gt::updateTerrainHealthHysteresis(
      gt::TerrainFrameHealthClass::kSoftGeometryFailure, 1.00, parameters,
      &state));
  EXPECT_TRUE(gt::enforceTerrainHealthHysteresisDeadline(
      1.249, parameters, &state));
  EXPECT_FALSE(gt::enforceTerrainHealthHysteresisDeadline(
      1.250, parameters, &state));
  EXPECT_EQ(1, state.consecutive_soft_failure_frames);
}

TEST(TerrainModel, HardFailureClosesImmediatelyAndRequiresFreshOpeningRun) {
  gt::TerrainHealthHysteresisParameters parameters;
  gt::TerrainHealthHysteresisState state;
  for (int frame = 0; frame < parameters.opening_healthy_frames; ++frame) {
    gt::updateTerrainHealthHysteresis(
        gt::TerrainFrameHealthClass::kHealthy, frame * 0.05, parameters,
        &state);
  }
  ASSERT_TRUE(state.gate_open);
  EXPECT_FALSE(gt::updateTerrainHealthHysteresis(
      gt::TerrainFrameHealthClass::kHardFailure, 1.00, parameters, &state));
  EXPECT_EQ(0, state.consecutive_healthy_frames);
  for (int frame = 0; frame < 4; ++frame) {
    EXPECT_FALSE(gt::updateTerrainHealthHysteresis(
        gt::TerrainFrameHealthClass::kHealthy, 1.05 + frame * 0.05,
        parameters, &state));
  }
  EXPECT_TRUE(gt::updateTerrainHealthHysteresis(
      gt::TerrainFrameHealthClass::kHealthy, 1.25, parameters, &state));
}

TEST(TerrainModel, FrameHealthClassificationNeverSoftensSafetyFailures) {
  gt::GroundPlaneEstimate plane;
  plane.valid = true;
  plane.status = gt::GroundPlaneFitStatus::kValid;
  EXPECT_EQ(gt::TerrainFrameHealthClass::kHealthy,
            gt::classifyTerrainFrameHealth(
                true, true, true, true, true, plane, true));
  EXPECT_EQ(gt::TerrainFrameHealthClass::kSoftGeometryFailure,
            gt::classifyTerrainFrameHealth(
                true, false, true, true, true, plane, true));

  plane.valid = false;
  for (const gt::GroundPlaneFitStatus soft_status : {
           gt::GroundPlaneFitStatus::kInsufficientConnectedSamples,
           gt::GroundPlaneFitStatus::kDegenerateGeometry}) {
    plane.status = soft_status;
    EXPECT_EQ(gt::TerrainFrameHealthClass::kSoftGeometryFailure,
              gt::classifyTerrainFrameHealth(
                  true, true, true, true, true, plane, true));
  }
  for (const gt::GroundPlaneFitStatus hard_status : {
           gt::GroundPlaneFitStatus::kFitFailure,
           gt::GroundPlaneFitStatus::kExcessiveResidual,
           gt::GroundPlaneFitStatus::kSensorHeightBelowMinimum,
           gt::GroundPlaneFitStatus::kSensorHeightAboveMaximum,
           gt::GroundPlaneFitStatus::kValid}) {
    plane.status = hard_status;
    EXPECT_EQ(gt::TerrainFrameHealthClass::kHardFailure,
              gt::classifyTerrainFrameHealth(
                  true, true, true, true, true, plane, true));
  }
  plane.valid = true;
  plane.status = gt::GroundPlaneFitStatus::kValid;
  EXPECT_EQ(gt::TerrainFrameHealthClass::kHardFailure,
            gt::classifyTerrainFrameHealth(
                false, true, true, true, true, plane, true));
  EXPECT_EQ(gt::TerrainFrameHealthClass::kHardFailure,
            gt::classifyTerrainFrameHealth(
                true, false, false, false, false, plane, false));
}

TEST(TerrainModel, GravityAlignmentPreservesYawOnly) {
  const Eigen::Quaterniond input =
      Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitX());
  const Eigen::Quaterniond output = gt::yawOnly(input);
  const Eigen::Vector3d up = output * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(0.0, up.x(), 1e-9);
  EXPECT_NEAR(0.0, up.y(), 1e-9);
  EXPECT_NEAR(1.0, up.z(), 1e-9);
  const Eigen::Vector3d forward = output * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(std::cos(0.7), forward.x(), 1e-9);
  EXPECT_NEAR(std::sin(0.7), forward.y(), 1e-9);
}

TEST(TerrainModel, LowSupportCandidateSurvivesSameCellCeilingReturn) {
  const float height = gt::robustLowSupportHeight(
      {-0.52F, -0.50F, 2.02F, 2.05F}, 0.10, 0.15);
  EXPECT_NEAR(-0.51, height, 1e-6);
}

TEST(TerrainModel, ConnectedFloorRejectsPatchworkCeilingCandidates) {
  gt::GroundConnectivityParameters parameters;
  parameters.width = 9;
  parameters.height = 3;
  parameters.resolution = 0.15;
  parameters.origin_x = -0.675;
  parameters.origin_y = -0.225;
  parameters.sensor_height = 0.51;
  parameters.anchor_radius = 0.80;
  parameters.anchor_height_tolerance = 0.18;
  parameters.maximum_slope_deg = 35.0;
  parameters.height_margin = 0.01;

  std::vector<float> candidates(
      static_cast<std::size_t>(parameters.width * parameters.height),
      std::numeric_limits<float>::quiet_NaN());
  for (int x = 0; x < parameters.width; ++x) {
    candidates[static_cast<std::size_t>(parameters.width + x)] = -0.51F;
    // Simulates a ceiling surface incorrectly emitted in Patchwork ground.
    candidates[static_cast<std::size_t>(2 * parameters.width + x)] = 2.04F;
  }

  const std::vector<std::uint8_t> accepted =
      gt::connectedGroundMask(candidates, parameters);
  for (int x = 0; x < parameters.width; ++x) {
    EXPECT_EQ(1U, accepted[static_cast<std::size_t>(parameters.width + x)]);
    EXPECT_EQ(0U,
              accepted[static_cast<std::size_t>(2 * parameters.width + x)]);
  }
}

TEST(TerrainModel, RejectedPatchworkTabletopRemainsAMarkingObstacle) {
  gt::GroundConnectivityParameters geometry;
  geometry.width = 9;
  geometry.height = 3;
  geometry.resolution = 0.15;
  geometry.origin_x = -0.675;
  geometry.origin_y = -0.225;
  std::vector<float> candidates(
      static_cast<std::size_t>(geometry.width * geometry.height),
      std::numeric_limits<float>::quiet_NaN());
  std::vector<float> rejected_surface(
      candidates.size(), std::numeric_limits<float>::quiet_NaN());
  std::vector<std::vector<gt::SurfaceSample>> samples(candidates.size());
  for (int x = 0; x < geometry.width; ++x) {
    candidates[static_cast<std::size_t>(geometry.width + x)] = -0.51F;
    const std::size_t tabletop =
        static_cast<std::size_t>(2 * geometry.width + x);
    candidates[tabletop] = -0.25F;
    rejected_surface[tabletop] = -0.25F;
    addFlatCellSamples(geometry, x, 2, -0.25, &samples);
  }

  const std::vector<std::uint8_t> accepted =
      gt::connectedGroundMask(candidates, geometry);
  const gt::SteepSurfaceParameters steep_parameters;
  const std::vector<std::uint8_t> steep =
      gt::continuousSteepSurfaceMask(
          candidates, accepted, rejected_surface, samples, geometry,
          steep_parameters);
  const std::size_t tabletop =
      static_cast<std::size_t>(2 * geometry.width + 4);
  EXPECT_EQ(0U, accepted[tabletop]);
  EXPECT_EQ(0U, steep[tabletop]);
  gt::NongroundThresholds thresholds;
  EXPECT_EQ(gt::NongroundClass::kObstacle,
            gt::classifyNongroundForLocalCostmap(
                -0.25, true, -0.51, steep[tabletop] != 0U, thresholds));
}

TEST(TerrainModel, PatchworkNongroundTabletopRemainsAnObstacle) {
  gt::NongroundThresholds thresholds;
  EXPECT_EQ(gt::NongroundClass::kObstacle,
            gt::classifyNonground(-0.25, true, -0.51, thresholds));
}

TEST(TerrainModel, RejectedSteepGroundDoesNotBecomeAnObstacle) {
  gt::GroundConnectivityParameters parameters;
  parameters.width = 3;
  parameters.height = 1;
  parameters.resolution = 0.15;
  parameters.origin_x = -0.225;
  parameters.origin_y = -0.075;
  parameters.anchor_radius = 0.08;
  const float steep_rise = static_cast<float>(
      std::tan(40.0 * 3.14159265358979323846 / 180.0) *
      parameters.resolution);
  const std::vector<float> candidates = {
      -0.51F, -0.51F, -0.51F + steep_rise};

  const std::vector<std::uint8_t> accepted =
      gt::connectedGroundMask(candidates, parameters);
  EXPECT_EQ(1U, accepted[1]);
  EXPECT_EQ(0U, accepted[2]);
  gt::NongroundThresholds thresholds;
  EXPECT_NE(gt::NongroundClass::kObstacle,
            gt::classifyIgnoredSteepSurface(
                candidates[2], true, candidates[1], thresholds));
}

TEST(TerrainModel, GrowsAContinuousThirtyFiveDegreeSurface) {
  gt::GroundConnectivityParameters parameters;
  parameters.width = 12;
  parameters.height = 1;
  parameters.resolution = 0.15;
  parameters.origin_x = -0.30;
  parameters.origin_y = -0.075;
  parameters.sensor_height = 0.51;
  parameters.anchor_radius = 0.80;
  parameters.anchor_height_tolerance = 0.18;
  parameters.maximum_slope_deg = 35.0;
  parameters.height_margin = 0.01;

  std::vector<float> candidates(static_cast<std::size_t>(parameters.width));
  const double rise = std::tan(35.0 * 3.14159265358979323846 / 180.0) *
                      parameters.resolution;
  for (int x = 0; x < parameters.width; ++x) {
    candidates[static_cast<std::size_t>(x)] =
        static_cast<float>(-parameters.sensor_height + (x - 1) * rise);
  }

  const std::vector<std::uint8_t> accepted =
      gt::connectedGroundMask(candidates, parameters);
  for (std::uint8_t value : accepted) {
    EXPECT_EQ(1U, value);
  }
}

TEST(TerrainModel, RaisedStepCannotBypassSlopeLimitDiagonally) {
  gt::GroundConnectivityParameters parameters;
  parameters.width = 8;
  parameters.height = 5;
  parameters.resolution = 0.15;
  parameters.origin_x = -0.60;
  parameters.origin_y = -0.375;
  parameters.sensor_height = 0.51;
  parameters.anchor_radius = 0.25;
  parameters.anchor_height_tolerance = 0.18;
  parameters.anchor_seed_height_band = 0.05;
  parameters.maximum_slope_deg = 35.0;
  parameters.height_margin = 0.01;

  for (const float step_height : {0.12F, 0.15F, 0.16F}) {
    std::vector<float> candidates(
        static_cast<std::size_t>(parameters.width * parameters.height),
        -0.51F);
    for (int y = 0; y < parameters.height; ++y) {
      for (int x = 4; x < parameters.width; ++x) {
        candidates[static_cast<std::size_t>(y * parameters.width + x)] +=
            step_height;
      }
    }

    const std::vector<std::uint8_t> accepted =
        gt::connectedGroundMask(candidates, parameters);
    for (int y = 0; y < parameters.height; ++y) {
      EXPECT_EQ(1U, accepted[static_cast<std::size_t>(y * parameters.width +
                                                     3)])
          << "step_height=" << step_height;
      EXPECT_EQ(0U, accepted[static_cast<std::size_t>(y * parameters.width +
                                                     4)])
          << "step_height=" << step_height;
    }
  }
}

TEST(TerrainModel, DoesNotReconnectAcrossMissingGround) {
  gt::GroundConnectivityParameters parameters;
  parameters.width = 7;
  parameters.height = 1;
  parameters.resolution = 0.15;
  parameters.origin_x = -0.30;
  parameters.origin_y = -0.075;
  parameters.anchor_radius = 0.25;
  std::vector<float> candidates = {
      -0.51F, -0.51F, -0.51F,
      std::numeric_limits<float>::quiet_NaN(),
      -0.51F, -0.51F, -0.51F};

  const std::vector<std::uint8_t> accepted =
      gt::connectedGroundMask(candidates, parameters);
  EXPECT_EQ(1U, accepted[0]);
  EXPECT_EQ(1U, accepted[2]);
  EXPECT_EQ(0U, accepted[3]);
  EXPECT_EQ(0U, accepted[4]);
  EXPECT_EQ(0U, accepted[6]);
}

TEST(TerrainModel, SparseFiveCellGroundCoverageIsUnhealthy) {
  const gt::GroundConnectivityParameters geometry = coverageGeometry();
  std::vector<std::uint8_t> mask(
      static_cast<std::size_t>(geometry.width * geometry.height), 0U);
  for (int x = 8; x <= 12; ++x) {
    mask[coverageIndex(x, 10, geometry)] = 1U;
  }
  const gt::GroundHealthParameters parameters;
  const gt::GroundCoverageMetrics metrics =
      gt::measureGroundCoverage(mask, geometry, parameters);
  EXPECT_EQ(5U, metrics.connected_cells);
  EXPECT_FALSE(gt::healthyGroundCoverage(metrics, parameters));
}

TEST(TerrainModel, GroundWithoutNearFieldReferenceIsUnhealthy) {
  const gt::GroundConnectivityParameters geometry = coverageGeometry();
  std::vector<std::uint8_t> mask(
      static_cast<std::size_t>(geometry.width * geometry.height), 0U);
  for (int y = 4; y <= 16; ++y) {
    for (int x = 18; x <= 20; ++x) {
      mask[coverageIndex(x, y, geometry)] = 1U;
    }
  }
  const gt::GroundHealthParameters parameters;
  const gt::GroundCoverageMetrics metrics =
      gt::measureGroundCoverage(mask, geometry, parameters);
  EXPECT_EQ(0U, metrics.near_support_cells);
  EXPECT_FALSE(gt::healthyGroundCoverage(metrics, parameters));
}

TEST(TerrainModel, SingleSectorGroundCoverageIsUnhealthy) {
  const gt::GroundConnectivityParameters geometry = coverageGeometry();
  std::vector<std::uint8_t> mask(
      static_cast<std::size_t>(geometry.width * geometry.height), 0U);
  for (int y = 9; y <= 12; ++y) {
    for (int x = 12; x <= 19; ++x) {
      mask[coverageIndex(x, y, geometry)] = 1U;
    }
  }
  const gt::GroundHealthParameters parameters;
  const gt::GroundCoverageMetrics metrics =
      gt::measureGroundCoverage(mask, geometry, parameters);
  EXPECT_GE(metrics.connected_area_m2, parameters.minimum_connected_area_m2);
  EXPECT_LT(metrics.covered_sectors, parameters.minimum_covered_sectors);
  EXPECT_FALSE(gt::healthyGroundCoverage(metrics, parameters));
}

TEST(TerrainModel, NormalFlatGroundCoverageIsHealthy) {
  const gt::GroundConnectivityParameters geometry = coverageGeometry();
  const std::vector<std::uint8_t> mask = annularGroundMask(geometry);
  const gt::GroundHealthParameters parameters;
  const gt::GroundCoverageMetrics metrics =
      gt::measureGroundCoverage(mask, geometry, parameters);
  EXPECT_GE(metrics.connected_area_m2, parameters.minimum_connected_area_m2);
  EXPECT_GE(metrics.near_support_area_m2,
            parameters.minimum_near_support_area_m2);
  EXPECT_GE(metrics.covered_sectors, parameters.minimum_covered_sectors);
  EXPECT_TRUE(gt::healthyGroundCoverage(metrics, parameters));
}

TEST(TerrainModel, HealthEvidenceCannotCombineTwoGroundComponents) {
  const gt::GroundConnectivityParameters geometry = coverageGeometry();
  const std::size_t count =
      static_cast<std::size_t>(geometry.width * geometry.height);
  std::vector<std::uint8_t> candidates_mask(count, 0U);
  std::vector<float> candidate_heights(
      count, std::numeric_limits<float>::quiet_NaN());
  std::size_t near_low_cells = 0U;
  std::size_t remote_nominal_cells = 0U;

  for (int y = 0; y < geometry.height; ++y) {
    for (int x = 0; x < geometry.width; ++x) {
      const double center_x =
          geometry.origin_x +
          (static_cast<double>(x) + 0.5) * geometry.resolution;
      const double center_y =
          geometry.origin_y +
          (static_cast<double>(y) + 0.5) * geometry.resolution;
      const double range = std::hypot(center_x, center_y);
      const std::size_t index = coverageIndex(x, y, geometry);
      if (range >= 0.35 && range <= 0.90 && center_x >= 0.0) {
        candidates_mask[index] = 1U;
        candidate_heights[index] = -0.34F;
        ++near_low_cells;
      } else if (range >= 1.10 && range <= 1.50) {
        candidates_mask[index] = 1U;
        candidate_heights[index] = -0.51F;
        ++remote_nominal_cells;
      }
    }
  }
  ASSERT_GT(remote_nominal_cells, near_low_cells);

  const gt::GroundHealthParameters health_parameters;
  const gt::SelectedGroundComponent selected =
      gt::selectGroundComponent(candidates_mask, geometry,
                                health_parameters);
  EXPECT_EQ(near_low_cells, selected.coverage.connected_cells);
  EXPECT_EQ(near_low_cells,
            static_cast<std::size_t>(std::count(selected.mask.begin(),
                                                selected.mask.end(), 1U)));
  EXPECT_TRUE(gt::healthyGroundCoverage(selected.coverage,
                                        health_parameters));

  const gt::GroundPlaneFitParameters fit_parameters;
  const gt::GroundPlaneEstimate selected_plane =
      gt::estimateConnectedGroundPlane(candidate_heights, selected.mask,
                                       geometry, fit_parameters);
  EXPECT_FALSE(selected_plane.valid);
  EXPECT_EQ(gt::GroundPlaneFitStatus::kSensorHeightBelowMinimum,
            selected_plane.status);
  EXPECT_NEAR(0.34, selected_plane.sensor_height_m, 1e-6);

  // Before component selection was shared, the independent height fitter chose
  // the larger remote component and could combine its nominal height with the
  // near component's healthy coverage.
  const gt::GroundPlaneEstimate independently_fitted_plane =
      gt::estimateConnectedGroundPlane(candidate_heights, candidates_mask,
                                       geometry, fit_parameters);
  ASSERT_TRUE(independently_fitted_plane.valid);
  EXPECT_NEAR(0.51, independently_fitted_plane.sensor_height_m, 1e-6);
  EXPECT_FALSE(gt::healthyGroundCoverage(selected.coverage,
                                         health_parameters) &&
               selected_plane.valid);
}

TEST(TerrainModel, RemoteGroundWithoutNearSupportIsNotSelectedForClearing) {
  const gt::GroundConnectivityParameters geometry = coverageGeometry();
  const std::size_t count =
      static_cast<std::size_t>(geometry.width * geometry.height);
  std::vector<std::uint8_t> mask(count, 0U);
  for (int y = 0; y < geometry.height; ++y) {
    for (int x = 0; x < geometry.width; ++x) {
      const double center_x =
          geometry.origin_x +
          (static_cast<double>(x) + 0.5) * geometry.resolution;
      const double center_y =
          geometry.origin_y +
          (static_cast<double>(y) + 0.5) * geometry.resolution;
      if (std::hypot(center_x, center_y) >= 1.10) {
        mask[coverageIndex(x, y, geometry)] = 1U;
      }
    }
  }

  const gt::SelectedGroundComponent selected =
      gt::selectGroundComponent(mask, geometry,
                                gt::GroundHealthParameters());
  EXPECT_EQ(0U, selected.coverage.connected_cells);
  EXPECT_EQ(0, std::count(selected.mask.begin(), selected.mask.end(), 1U));
}

TEST(TerrainModel, ContinuousTwentyDegreeSlopeCoverageIsHealthy) {
  gt::GroundConnectivityParameters geometry = coverageGeometry();
  std::vector<float> candidates(
      static_cast<std::size_t>(geometry.width * geometry.height),
      std::numeric_limits<float>::quiet_NaN());
  const double grade = std::tan(20.0 * 3.14159265358979323846 / 180.0);
  for (int y = 0; y < geometry.height; ++y) {
    for (int x = 0; x < geometry.width; ++x) {
      const double center_x =
          geometry.origin_x + (static_cast<double>(x) + 0.5) *
                                  geometry.resolution;
      const double center_y =
          geometry.origin_y + (static_cast<double>(y) + 0.5) *
                                  geometry.resolution;
      const double range = std::hypot(center_x, center_y);
      if (range >= 0.35 && range <= 1.40) {
        candidates[coverageIndex(x, y, geometry)] =
            static_cast<float>(-geometry.sensor_height + grade * center_x);
      }
    }
  }

  const std::vector<std::uint8_t> accepted =
      gt::connectedGroundMask(candidates, geometry);
  const gt::GroundHealthParameters parameters;
  const gt::GroundCoverageMetrics metrics =
      gt::measureGroundCoverage(accepted, geometry, parameters);
  EXPECT_TRUE(gt::healthyGroundCoverage(metrics, parameters));
}

TEST(TerrainModel, TerrainRateNeedsEnoughSamples) {
  EXPECT_FALSE(gt::healthyTerrainOutputRate(0U, 20.0, 3, 8.0));
  EXPECT_FALSE(gt::healthyTerrainOutputRate(1U, 20.0, 3, 8.0));
  EXPECT_FALSE(gt::healthyTerrainOutputRate(2U, 20.0, 3, 8.0));
}

TEST(TerrainModel, TerrainRateRejectsOneHertzAndAcceptsEightHertz) {
  EXPECT_FALSE(gt::healthyTerrainOutputRate(3U, 1.0, 3, 8.0));
  EXPECT_FALSE(gt::healthyTerrainOutputRate(3U, 7.999, 3, 8.0));
  EXPECT_TRUE(gt::healthyTerrainOutputRate(3U, 8.0, 3, 8.0));
  EXPECT_TRUE(gt::healthyTerrainOutputRate(3U, 12.0, 3, 8.0));
  EXPECT_FALSE(gt::healthyTerrainOutputRate(
      3U, std::numeric_limits<double>::quiet_NaN(), 3, 8.0));
}

TEST(TerrainModel, ContinuousSlopeNongroundIsNeverAMarkingObstacle) {
  gt::GroundConnectivityParameters geometry;
  geometry.width = 7;
  geometry.height = 3;
  geometry.resolution = 0.15;
  geometry.origin_x = 0.0;
  geometry.origin_y = -0.225;
  gt::SteepSurfaceParameters parameters;
  gt::NongroundThresholds thresholds;
  const float unknown = std::numeric_limits<float>::quiet_NaN();

  for (const double slope_deg : {8.0, 10.0, 20.0, 30.0, 35.0, 45.0, 70.0}) {
    std::vector<float> ground_heights(
        static_cast<std::size_t>(geometry.width * geometry.height), unknown);
    std::vector<std::uint8_t> connected_ground(ground_heights.size(), 0U);
    std::vector<float> nonground(ground_heights.size(), unknown);
    std::vector<std::vector<gt::SurfaceSample>> samples(
        ground_heights.size());
    const double gradient = std::tan(
        slope_deg * 3.14159265358979323846 / 180.0);
    for (int y = 0; y < geometry.height; ++y) {
      const std::size_t source =
          static_cast<std::size_t>(y * geometry.width);
      const double reference_x = geometry.origin_x + 0.5 * geometry.resolution;
      const double reference_y =
          geometry.origin_y + (static_cast<double>(y) + 0.5) *
                                  geometry.resolution;
      ground_heights[source] = -0.51F;
      connected_ground[source] = 1U;
      for (int x = 1; x < geometry.width; ++x) {
        const std::size_t index =
            static_cast<std::size_t>(y * geometry.width + x);
        const double center_x =
            geometry.origin_x + (static_cast<double>(x) + 0.5) *
                                    geometry.resolution;
        nonground[index] = static_cast<float>(
            -0.51 + gradient * (center_x - reference_x));
        addPlanarCellSamples(geometry, x, y, reference_x, reference_y,
                             -0.51, gradient, 0.0, &samples);
      }
    }

    const std::vector<std::uint8_t> steep =
        gt::continuousSteepSurfaceMask(
            ground_heights, connected_ground, nonground, samples,
            geometry, parameters);
    for (int y = 0; y < geometry.height; ++y) {
      for (int x = 1; x < geometry.width; ++x) {
        EXPECT_EQ(1U, steep[static_cast<std::size_t>(
                          y * geometry.width + x)])
            << "slope_deg=" << slope_deg;
      }
    }
    ASSERT_EQ(gt::NongroundClass::kObstacle,
              gt::classifyNonground(
                  nonground[3], true, ground_heights[0], thresholds))
        << "slope_deg=" << slope_deg;
    EXPECT_NE(gt::NongroundClass::kObstacle,
               gt::classifyNongroundForLocalCostmap(
                  nonground[3], true, ground_heights[0], true, thresholds))
        << "slope_deg=" << slope_deg;
  }
}

TEST(TerrainModel, DiagonalFortyFiveDegreeNongroundIsUnknown) {
  gt::GroundConnectivityParameters geometry;
  geometry.width = 6;
  geometry.height = 6;
  geometry.resolution = 0.15;
  const std::size_t count =
      static_cast<std::size_t>(geometry.width * geometry.height);
  const float unknown = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> ground_heights(count, unknown);
  std::vector<std::uint8_t> connected_ground(count, 0U);
  std::vector<float> nonground(count, unknown);
  std::vector<std::vector<gt::SurfaceSample>> samples(count);
  ground_heights[0] = -0.51F;
  connected_ground[0] = 1U;
  const double spacing = geometry.resolution * std::sqrt(2.0);
  const double reference_x = geometry.origin_x + 0.5 * geometry.resolution;
  const double reference_y = geometry.origin_y + 0.5 * geometry.resolution;
  const double gradient_component = 1.0 / std::sqrt(2.0);
  for (int step = 1; step < geometry.width; ++step) {
    const std::size_t index =
        static_cast<std::size_t>(step * geometry.width + step);
    nonground[index] = static_cast<float>(-0.51 + step * spacing);
    addPlanarCellSamples(geometry, step, step, reference_x, reference_y,
                         -0.51, gradient_component, gradient_component,
                         &samples);
  }

  const gt::SteepSurfaceParameters parameters;
  const std::vector<std::uint8_t> steep =
      gt::continuousSteepSurfaceMask(
          ground_heights, connected_ground, nonground, samples,
          geometry, parameters);
  for (int step = 1; step < geometry.width; ++step) {
    EXPECT_EQ(1U, steep[static_cast<std::size_t>(
                      step * geometry.width + step)]);
  }
}

TEST(TerrainModel, ObliqueThirtyFiveDegreeNongroundIsUnknown) {
  gt::GroundConnectivityParameters geometry;
  geometry.width = 7;
  geometry.height = 3;
  geometry.resolution = 0.15;
  geometry.origin_x = 0.0;
  geometry.origin_y = -0.225;
  const std::size_t count =
      static_cast<std::size_t>(geometry.width * geometry.height);
  const float unknown = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> ground_heights(count, unknown);
  std::vector<std::uint8_t> connected_ground(count, 0U);
  std::vector<float> nonground(count, unknown);
  std::vector<std::vector<gt::SurfaceSample>> samples(count);
  const int row = 1;
  const std::size_t source =
      static_cast<std::size_t>(row * geometry.width);
  ground_heights[source] = -0.51F;
  connected_ground[source] = 1U;
  const double reference_x = geometry.origin_x + 0.5 * geometry.resolution;
  const double reference_y = geometry.origin_y + 1.5 * geometry.resolution;
  const double grade = std::tan(35.0 * 3.14159265358979323846 / 180.0);
  const double azimuth = 22.5 * 3.14159265358979323846 / 180.0;
  const double gradient_x = grade * std::cos(azimuth);
  const double gradient_y = grade * std::sin(azimuth);
  for (int x = 1; x < geometry.width; ++x) {
    const std::size_t index =
        static_cast<std::size_t>(row * geometry.width + x);
    const double center_x =
        geometry.origin_x + (static_cast<double>(x) + 0.5) *
                                geometry.resolution;
    nonground[index] = static_cast<float>(
        -0.51 + gradient_x * (center_x - reference_x));
    addPlanarCellSamples(geometry, x, row, reference_x, reference_y,
                         -0.51, gradient_x, gradient_y, &samples);
  }
  const gt::SteepSurfaceParameters parameters;
  const std::vector<std::uint8_t> steep =
      gt::continuousSteepSurfaceMask(
          ground_heights, connected_ground, nonground, samples,
          geometry, parameters);
  for (int x = 1; x < geometry.width; ++x) {
    EXPECT_EQ(1U, steep[static_cast<std::size_t>(
                      row * geometry.width + x)]);
  }
}

TEST(TerrainModel, SameColumnCeilingDoesNotPoisonContinuousSlopeFit) {
  gt::GroundConnectivityParameters geometry;
  geometry.width = 7;
  geometry.height = 3;
  geometry.resolution = 0.15;
  geometry.origin_x = 0.0;
  geometry.origin_y = -0.225;
  const std::size_t count =
      static_cast<std::size_t>(geometry.width * geometry.height);
  const float unknown = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> ground_heights(count, unknown);
  std::vector<std::uint8_t> connected_ground(count, 0U);
  std::vector<float> surface_heights(count, unknown);
  std::vector<std::vector<gt::SurfaceSample>> samples(count);
  const int row = 1;
  const std::size_t source =
      static_cast<std::size_t>(row * geometry.width);
  ground_heights[source] = -0.51F;
  connected_ground[source] = 1U;
  const double reference_x = geometry.origin_x + 0.5 * geometry.resolution;
  const double reference_y = geometry.origin_y + 1.5 * geometry.resolution;
  for (int x = 1; x < geometry.width; ++x) {
    const std::size_t index =
        static_cast<std::size_t>(row * geometry.width + x);
    addPlanarCellSamples(geometry, x, row, reference_x, reference_y,
                         -0.51, 1.0, 0.0, &samples);
    samples[index].push_back(
        {geometry.origin_x + (static_cast<double>(x) + 0.5) *
                                 geometry.resolution,
         reference_y, 2.55});
    std::vector<gt::SurfaceSample> low_support =
        gt::lowSupportSurfaceSamples(samples[index], 0.10, 0.15,
                                     &surface_heights[index]);
    samples[index].swap(low_support);
    ASSERT_EQ(5U, samples[index].size());
    for (const gt::SurfaceSample& sample : samples[index]) {
      EXPECT_LT(sample.z, 1.0);
    }
  }

  const gt::SteepSurfaceParameters parameters;
  const std::vector<std::uint8_t> steep =
      gt::continuousSteepSurfaceMask(
          ground_heights, connected_ground, surface_heights, samples,
          geometry, parameters);
  for (int x = 1; x < geometry.width; ++x) {
    EXPECT_EQ(1U, steep[static_cast<std::size_t>(
                      row * geometry.width + x)]);
  }
  const gt::NongroundThresholds thresholds;
  EXPECT_EQ(gt::NongroundClass::kUnknownHigh,
            gt::classifyNonground(2.55, false, 0.0, thresholds));
}

TEST(TerrainModel, ThinWallRemainsAMarkingObstacle) {
  gt::GroundConnectivityParameters geometry;
  geometry.width = 6;
  geometry.height = 5;
  geometry.resolution = 0.15;
  const std::size_t count =
      static_cast<std::size_t>(geometry.width * geometry.height);
  const float unknown = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> ground_heights(count, unknown);
  std::vector<std::uint8_t> connected_ground(count, 0U);
  std::vector<float> nonground(count, unknown);
  std::vector<std::vector<gt::SurfaceSample>> samples(count);
  for (int y = 0; y < geometry.height; ++y) {
    for (int x = 0; x <= 1; ++x) {
      const std::size_t index =
          static_cast<std::size_t>(y * geometry.width + x);
      ground_heights[index] = -0.51F;
      connected_ground[index] = 1U;
    }
    const std::size_t wall =
        static_cast<std::size_t>(y * geometry.width + 2);
    nonground[wall] = 0.0F;
    const double center_x =
        geometry.origin_x + 2.5 * geometry.resolution;
    const double center_y =
        geometry.origin_y + (static_cast<double>(y) + 0.5) *
                                geometry.resolution;
    samples[wall] = {{center_x, center_y, -0.40},
                     {center_x, center_y, 0.0},
                     {center_x, center_y, 0.40}};
  }
  const gt::SteepSurfaceParameters parameters;
  const std::vector<std::uint8_t> steep =
      gt::continuousSteepSurfaceMask(
          ground_heights, connected_ground, nonground, samples,
          geometry, parameters);
  EXPECT_EQ(0, std::count(steep.begin(), steep.end(), 1U));
  gt::NongroundThresholds thresholds;
  EXPECT_EQ(gt::NongroundClass::kObstacle,
            gt::classifyNongroundForLocalCostmap(
                0.0, true, -0.51, false, thresholds));
}

TEST(TerrainModel, FlatToppedBoxRemainsAMarkingObstacle) {
  gt::GroundConnectivityParameters geometry;
  geometry.width = 8;
  geometry.height = 3;
  geometry.resolution = 0.15;
  const std::size_t count =
      static_cast<std::size_t>(geometry.width * geometry.height);
  const float unknown = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> ground_heights(count, unknown);
  std::vector<std::uint8_t> connected_ground(count, 0U);
  std::vector<float> nonground(count, unknown);
  std::vector<std::vector<gt::SurfaceSample>> samples(count);
  for (int y = 0; y < geometry.height; ++y) {
    const std::size_t source =
        static_cast<std::size_t>(y * geometry.width);
    ground_heights[source] = -0.51F;
    connected_ground[source] = 1U;
    for (int x = 1; x <= 5; ++x) {
      const std::size_t index =
          static_cast<std::size_t>(y * geometry.width + x);
      nonground[index] = -0.21F;
      addFlatCellSamples(geometry, x, y, -0.21, &samples);
    }
  }
  const gt::SteepSurfaceParameters parameters;
  const std::vector<std::uint8_t> steep =
      gt::continuousSteepSurfaceMask(
          ground_heights, connected_ground, nonground, samples,
          geometry, parameters);
  EXPECT_EQ(0, std::count(steep.begin(), steep.end(), 1U));
  gt::NongroundThresholds thresholds;
  EXPECT_EQ(gt::NongroundClass::kObstacle,
            gt::classifyNongroundForLocalCostmap(
                -0.21, true, -0.51, false, thresholds));
}

TEST(TerrainModel, RegularStaircaseRemainsAMarkingObstacle) {
  gt::GroundConnectivityParameters geometry;
  geometry.width = 8;
  geometry.height = 1;
  geometry.resolution = 0.15;
  const float unknown = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> ground_heights(8U, unknown);
  std::vector<std::uint8_t> connected_ground(8U, 0U);
  std::vector<float> nonground(8U, unknown);
  std::vector<std::vector<gt::SurfaceSample>> samples(8U);
  for (int x = 1; x <= 6; ++x) {
    const std::size_t index = static_cast<std::size_t>(x);
    nonground[index] = static_cast<float>(-0.51 + 0.12 * x);
    addFlatCellSamples(geometry, x, 0, nonground[index], &samples);
    const double center_x =
        geometry.origin_x + (static_cast<double>(x) + 0.5) *
                                geometry.resolution;
    const double center_y = geometry.origin_y + 0.5 * geometry.resolution;
    samples[index].push_back(
        {center_x, center_y, nonground[index] - 0.12});
  }
  ground_heights[0] = -0.51F;
  connected_ground[0] = 1U;
  const gt::SteepSurfaceParameters parameters;
  const std::vector<std::uint8_t> steep =
      gt::continuousSteepSurfaceMask(
          ground_heights, connected_ground, nonground, samples,
          geometry, parameters);
  EXPECT_EQ(0, std::count(steep.begin(), steep.end(), 1U));
  gt::NongroundThresholds thresholds;
  EXPECT_EQ(gt::NongroundClass::kObstacle,
            gt::classifyNongroundForLocalCostmap(
                -0.39, true, -0.51, false, thresholds));
}
