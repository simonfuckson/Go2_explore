#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <costmap_2d/cost_values.h>

#include "go2_terrain/static_layer_update.hpp"
#include "go2_terrain/terrain_algorithms.hpp"
#include "go2_terrain/terrain_grid.hpp"

namespace go2_terrain
{
namespace
{

GridGeometry geometry(std::size_t width, std::size_t height)
{
  GridGeometry result;
  result.width = width;
  result.height = height;
  result.resolution = 0.05;
  result.origin_x = -1.0;
  result.origin_y = -2.0;
  result.origin_yaw = 0.0;
  return result;
}

TEST(TerrainAlgorithms, MapsApprovedSoftSlopeRange)
{
  CostParameters parameters;
  EXPECT_FLOAT_EQ(0.0F, slopeToSoftCost(0.0, parameters));
  EXPECT_FLOAT_EQ(0.0F, slopeToSoftCost(8.0, parameters));
  EXPECT_NEAR(15.0F, slopeToSoftCost(8.000001, parameters), 1e-3F);
  EXPECT_NEAR(47.5F, slopeToSoftCost(19.0, parameters), 1e-4F);
  EXPECT_FLOAT_EQ(80.0F, slopeToSoftCost(30.0, parameters));
  EXPECT_FLOAT_EQ(80.0F, slopeToSoftCost(35.0, parameters));
}

TEST(TerrainAlgorithms, RequiresFourConnectedSteepCellsForLethalCost)
{
  const GridGeometry grid = geometry(6, 4);
  std::vector<float> slope(grid.cellCount(), 0.0F);
  slope[1 + 1 * grid.width] = 31.0F;
  slope[2 + 1 * grid.width] = 31.0F;
  slope[1 + 2 * grid.width] = 31.0F;

  CostParameters parameters;
  parameters.dilation_m = 0.0;
  std::vector<std::uint8_t> cost =
      buildSlopeCostLayer(slope, grid, parameters);
  EXPECT_EQ(80, cost[1 + 1 * grid.width]);
  EXPECT_EQ(80, cost[2 + 1 * grid.width]);
  EXPECT_EQ(80, cost[1 + 2 * grid.width]);

  slope[2 + 2 * grid.width] = 31.0F;
  cost = buildSlopeCostLayer(slope, grid, parameters);
  EXPECT_EQ(254, cost[1 + 1 * grid.width]);
  EXPECT_EQ(254, cost[2 + 1 * grid.width]);
  EXPECT_EQ(254, cost[1 + 2 * grid.width]);
  EXPECT_EQ(254, cost[2 + 2 * grid.width]);
}

TEST(TerrainAlgorithms, TenThroughThirtyDegreesStayTraversableAndThirtyFiveIsLethal)
{
  const GridGeometry grid = geometry(2, 2);
  CostParameters parameters;
  parameters.dilation_m = 0.0;
  const struct
  {
    float slope;
    int expected;
  } cases[] = {{10.0F, 21}, {20.0F, 50}, {30.0F, 80}, {35.0F, 254}};
  for (const auto& test : cases)
  {
    const std::vector<float> slope(grid.cellCount(), test.slope);
    const std::vector<std::uint8_t> cost =
        buildSlopeCostLayer(slope, grid, parameters);
    for (const std::uint8_t value : cost)
    {
      EXPECT_EQ(test.expected, value) << "slope=" << test.slope;
    }
  }
}

TEST(TerrainAlgorithms, CostDilationDoesNotInventKnownTerrain)
{
  const GridGeometry grid = geometry(5, 5);
  std::vector<std::uint8_t> cost(grid.cellCount(), 255);
  for (int y = 1; y <= 3; ++y)
  {
    for (int x = 1; x <= 3; ++x)
    {
      cost[static_cast<std::size_t>(y) * grid.width + x] = 0;
    }
  }
  cost[2 + 2 * grid.width] = 80;
  dilateKnownCosts(&cost, grid, 0.05);
  EXPECT_EQ(80, cost[2 + 1 * grid.width]);
  EXPECT_EQ(80, cost[1 + 2 * grid.width]);
  EXPECT_EQ(255, cost[0]);
}

TEST(TerrainAlgorithms, ThreeCentimeterMaskDilationMatchesLegacyOneCellRounding)
{
  const GridGeometry grid = geometry(5, 5);
  std::vector<std::uint8_t> source(grid.cellCount(), 0U);
  source[2 + 2 * grid.width] = 1U;
  std::vector<std::uint8_t> dilated;

  dilateBinaryMaskMetric(source, &dilated, grid, 0.03);

  EXPECT_EQ(1U, dilated[2 + 2 * grid.width]);
  EXPECT_EQ(1U, dilated[2 + 1 * grid.width]);
  EXPECT_EQ(1U, dilated[1 + 2 * grid.width]);
  EXPECT_EQ(1U, dilated[3 + 2 * grid.width]);
  EXPECT_EQ(1U, dilated[2 + 3 * grid.width]);
  EXPECT_EQ(0U, dilated[1 + 1 * grid.width]);
  EXPECT_EQ(5, std::count(dilated.begin(), dilated.end(), 1U));
}

TEST(TerrainAlgorithms, FiveCentimeterMaskDilationIncludesCardinalCellCenters)
{
  const GridGeometry grid = geometry(5, 5);
  std::vector<std::uint8_t> source(grid.cellCount(), 0U);
  source[2 + 2 * grid.width] = 1U;
  std::vector<std::uint8_t> dilated;

  dilateBinaryMaskMetric(source, &dilated, grid, 0.05);

  EXPECT_EQ(1U, dilated[2 + 1 * grid.width]);
  EXPECT_EQ(1U, dilated[1 + 2 * grid.width]);
  EXPECT_EQ(0U, dilated[1 + 1 * grid.width]);
}

TEST(TerrainAlgorithms, TerrainMergePreservesStaticUnknownAndLethalCells)
{
  EXPECT_EQ(costmap_2d::NO_INFORMATION,
            mergeTerrainCost(costmap_2d::NO_INFORMATION, 80));
  EXPECT_EQ(costmap_2d::LETHAL_OBSTACLE,
            mergeTerrainCost(costmap_2d::LETHAL_OBSTACLE, 15));
  EXPECT_EQ(80, mergeTerrainCost(costmap_2d::FREE_SPACE, 80));
  EXPECT_EQ(42, mergeTerrainCost(42, 15));
  EXPECT_EQ(42, mergeTerrainCost(42, costmap_2d::NO_INFORMATION));
  EXPECT_EQ(42, mergeTerrainCost(42, costmap_2d::FREE_SPACE));
  EXPECT_EQ(costmap_2d::LETHAL_OBSTACLE,
            mergeTerrainCost(42, costmap_2d::LETHAL_OBSTACLE));
}

TEST(StaticLayerUpdateState, RequiresCompleteWindowBeforeClearingRequest)
{
  StaticLayerUpdateState state(true);
  EXPECT_TRUE(state.fullUpdateRequired());
  EXPECT_FALSE(state.acknowledgeWindow(0, 0, 99, 80, 100, 80));
  EXPECT_TRUE(state.fullUpdateRequired());
  EXPECT_FALSE(state.acknowledgeWindow(1, 0, 100, 80, 100, 80));
  EXPECT_TRUE(state.fullUpdateRequired());
  EXPECT_TRUE(state.acknowledgeWindow(0, 0, 100, 80, 100, 80));
  EXPECT_FALSE(state.fullUpdateRequired());
}

TEST(StaticLayerUpdateState, ResetOrResizeCanRequestFullReapplication)
{
  StaticLayerUpdateState state;
  EXPECT_FALSE(state.fullUpdateRequired());
  state.requestFullUpdate();
  EXPECT_TRUE(state.fullUpdateRequired());
  EXPECT_TRUE(state.acknowledgeWindow(-2, -3, 102, 83, 100, 80));
  EXPECT_FALSE(state.fullUpdateRequired());
  state.requestFullUpdate();
  state.clearFullUpdate();
  EXPECT_FALSE(state.fullUpdateRequired());
}

TEST(TerrainAlgorithms, CeilingReturnsDoNotRejectFloorColumn)
{
  std::vector<float> samples;
  for (int i = 0; i < 20; ++i)
  {
    samples.push_back(-0.350F + static_cast<float>(i % 3 - 1) * 0.004F);
    samples.push_back(2.550F + static_cast<float>(i % 3 - 1) * 0.004F);
  }

  const GroundColumnSummary summary =
      summarizeGroundColumn(samples, 0.05, 0.12);
  EXPECT_TRUE(isKnown(summary.candidate));
  EXPECT_NEAR(-0.354F, summary.candidate, 0.015F);
  EXPECT_LT(summary.support_span, 0.03F);
  EXPECT_GE(summary.support_count, 20U);
  EXPECT_FALSE(isObstacleHeight(2.55 - summary.candidate, 0.05, 1.50));
  EXPECT_TRUE(isObstacleHeight(0.30, 0.05, 1.50));
}

TEST(TerrainAlgorithms, TracesTwentyDegreeRampWithFlattenedOdomTrajectory)
{
  GridGeometry grid;
  grid.width = 41;
  grid.height = 5;
  grid.resolution = 0.05;
  grid.origin_x = 0.0;
  grid.origin_y = -0.125;
  grid.origin_yaw = 0.0;
  std::vector<float> candidate(grid.cellCount(),
                               std::numeric_limits<float>::quiet_NaN());
  std::vector<float> span(grid.cellCount(),
                          std::numeric_limits<float>::quiet_NaN());
  std::vector<std::uint8_t> admissible(grid.cellCount(), 1U);
  std::vector<PlanarPoint> trajectory;
  const double tangent = std::tan(20.0 * 3.14159265358979323846 / 180.0);
  for (std::size_t x = 0; x < grid.width; ++x)
  {
    const double world_x = (static_cast<double>(x) + 0.5) * grid.resolution;
    trajectory.push_back({world_x, 0.0});  // There is deliberately no Z field.
    for (std::size_t y = 0; y < grid.height; ++y)
    {
      const std::size_t index = y * grid.width + x;
      candidate[index] = static_cast<float>(-0.35 + tangent * world_x);
      span[index] = 0.01F;
    }
  }

  const std::vector<float> traced = traceGroundAlongTrajectory(
      trajectory,
      candidate,
      span,
      admissible,
      grid,
      0.10,
      0.25,
      -0.35,
      0.05,
      35.0,
      0.005,
      0.65,
      0.50);
  ASSERT_EQ(trajectory.size(), traced.size());
  for (const float value : traced)
  {
    EXPECT_TRUE(isKnown(value));
  }
  EXPECT_NEAR(-0.35 + tangent * trajectory.back().x,
              traced.back(),
              0.04);
}

TEST(TerrainAlgorithms, TrajectorySeedUsesTrueMetricDisk)
{
  GridGeometry grid;
  grid.width = 15;
  grid.height = 15;
  grid.resolution = 0.05;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.origin_yaw = 0.0;
  const std::size_t count = grid.cellCount();
  std::vector<float> candidate(count, 0.0F);
  std::vector<float> span(count, 0.01F);
  const std::vector<PlanarPoint> trajectory = {{0.375, 0.375}};
  const std::vector<float> ground_z = {0.0F};

  const std::vector<std::uint8_t> seed = buildTrajectorySeedMask(
      trajectory, ground_z, candidate, span, grid,
      0.25, 0.25, 0.04);

  const std::size_t center = 7U + 7U * grid.width;
  const std::size_t cardinal_boundary = 12U + 7U * grid.width;
  const std::size_t diagonal_gap = 12U + 12U * grid.width;
  EXPECT_EQ(1U, seed[center]);
  EXPECT_EQ(1U, seed[cardinal_boundary]);
  EXPECT_EQ(0U, seed[diagonal_gap]);
}

TEST(TerrainAlgorithms, DoesNotReconnectAcrossUnboundedMissingGround)
{
  GridGeometry grid = geometry(30, 1);
  std::vector<float> candidate(grid.cellCount(), 0.0F);
  std::vector<float> span(grid.cellCount(), 0.01F);
  std::vector<std::uint8_t> admissible(grid.cellCount(), 0U);
  admissible[0] = 1U;
  for (std::size_t x = 3; x < 20; ++x)
  {
    candidate[x] = std::numeric_limits<float>::quiet_NaN();
    span[x] = std::numeric_limits<float>::quiet_NaN();
  }
  std::vector<PlanarPoint> trajectory;
  for (std::size_t x = 0; x < grid.width; ++x)
  {
    trajectory.push_back(
        {grid.origin_x + (x + 0.5) * grid.resolution,
         grid.origin_y + 0.5 * grid.resolution});
  }
  const std::vector<float> traced = traceGroundAlongTrajectory(
      trajectory, candidate, span, admissible, grid, 0.02, 0.25, 0.0,
      0.05, 35.0, 0.02, 0.65, 0.50);
  EXPECT_TRUE(isKnown(traced[2]));
  EXPECT_FALSE(isKnown(traced[20]));
  EXPECT_FALSE(isKnown(traced.back()));
}

TEST(TerrainAlgorithms, ReanchorsAfterBoundedGapOnlyWithAdmissibleSeed)
{
  GridGeometry grid = geometry(30, 1);
  std::vector<float> candidate(grid.cellCount(), 0.0F);
  std::vector<float> span(grid.cellCount(), 0.01F);
  std::vector<std::uint8_t> admissible(grid.cellCount(), 0U);
  for (std::size_t x = 3; x < 20; ++x)
  {
    candidate[x] = std::numeric_limits<float>::quiet_NaN();
    span[x] = std::numeric_limits<float>::quiet_NaN();
  }
  admissible[20] = 1U;
  std::vector<PlanarPoint> trajectory;
  for (std::size_t x = 0; x < grid.width; ++x)
  {
    trajectory.push_back(
        {grid.origin_x + (x + 0.5) * grid.resolution,
         grid.origin_y + 0.5 * grid.resolution});
  }
  const std::vector<float> traced = traceGroundAlongTrajectory(
      trajectory, candidate, span, admissible, grid, 0.02, 0.25, 0.0,
      0.05, 35.0, 0.02, 0.65, 0.50);
  EXPECT_FALSE(isKnown(traced[19]));
  EXPECT_TRUE(isKnown(traced[20]));
  EXPECT_TRUE(isKnown(traced.back()));
}

TEST(TerrainAlgorithms, ReanchorsLongRisingAndFallingRampFromInitialHeightBand)
{
  const GridGeometry grid = geometry(100, 1);
  const double tangent = std::tan(20.0 * 3.14159265358979323846 / 180.0);
  std::vector<PlanarPoint> trajectory;
  for (std::size_t x = 0; x < grid.width; ++x)
  {
    trajectory.push_back(
        {grid.origin_x + (x + 0.5) * grid.resolution,
         grid.origin_y + 0.5 * grid.resolution});
  }

  for (const double direction : {-1.0, 1.0})
  {
    std::vector<float> candidate(grid.cellCount(), 0.0F);
    std::vector<float> span(grid.cellCount(), 0.01F);
    std::vector<std::uint8_t> admissible(grid.cellCount(), 0U);
    for (std::size_t x = 0; x < grid.width; ++x)
    {
      candidate[x] = static_cast<float>(
          direction * tangent * (trajectory[x].x - trajectory.front().x));
    }
    for (std::size_t x = 5; x < 35; ++x)
    {
      candidate[x] = std::numeric_limits<float>::quiet_NaN();
      span[x] = std::numeric_limits<float>::quiet_NaN();
    }
    admissible[35] = 1U;

    const std::vector<float> traced = traceGroundAlongTrajectory(
        trajectory, candidate, span, admissible, grid, 0.02, 0.25, 0.0,
        0.05, 35.0, 0.02, 0.65, 0.50);
    EXPECT_FALSE(isKnown(traced[34]));
    EXPECT_TRUE(isKnown(traced[35]));
    EXPECT_TRUE(isKnown(traced.back()));
    EXPECT_NEAR(direction * tangent *
                    (trajectory.back().x - trajectory.front().x),
                traced.back(), 0.02);
  }
}

TEST(TerrainAlgorithms, ReanchorRejectsAdmissibleButImplausibleCeiling)
{
  GridGeometry grid = geometry(30, 1);
  std::vector<float> candidate(grid.cellCount(), 0.0F);
  std::vector<float> span(grid.cellCount(), 0.01F);
  std::vector<std::uint8_t> admissible(grid.cellCount(), 0U);
  for (std::size_t x = 3; x < 20; ++x)
  {
    candidate[x] = std::numeric_limits<float>::quiet_NaN();
    span[x] = std::numeric_limits<float>::quiet_NaN();
  }
  for (std::size_t x = 20; x < grid.width; ++x)
  {
    candidate[x] = 2.55F;
    admissible[x] = 1U;
  }
  std::vector<PlanarPoint> trajectory;
  for (std::size_t x = 0; x < grid.width; ++x)
  {
    trajectory.push_back(
        {grid.origin_x + (x + 0.5) * grid.resolution,
         grid.origin_y + 0.5 * grid.resolution});
  }
  const std::vector<float> traced = traceGroundAlongTrajectory(
      trajectory, candidate, span, admissible, grid, 0.02, 0.25, 0.0,
      0.05, 35.0, 0.02, 0.65, 0.50);
  EXPECT_FALSE(isKnown(traced[20]));
  EXPECT_FALSE(isKnown(traced.back()));
}

TEST(TerrainAlgorithms, ReanchorRejectsSuspendedPlatformAtPointEightMeters)
{
  GridGeometry grid = geometry(30, 1);
  std::vector<float> candidate(grid.cellCount(), 0.0F);
  std::vector<float> span(grid.cellCount(), 0.01F);
  std::vector<std::uint8_t> admissible(grid.cellCount(), 0U);
  for (std::size_t x = 3; x < 20; ++x)
  {
    candidate[x] = std::numeric_limits<float>::quiet_NaN();
    span[x] = std::numeric_limits<float>::quiet_NaN();
  }
  for (std::size_t x = 20; x < grid.width; ++x)
  {
    candidate[x] = 0.80F;
    admissible[x] = 1U;
  }
  std::vector<PlanarPoint> trajectory;
  for (std::size_t x = 0; x < grid.width; ++x)
  {
    trajectory.push_back(
        {grid.origin_x + (x + 0.5) * grid.resolution,
         grid.origin_y + 0.5 * grid.resolution});
  }
  const std::vector<float> traced = traceGroundAlongTrajectory(
      trajectory, candidate, span, admissible, grid, 0.02, 0.25, 0.0,
      0.05, 35.0, 0.02, 0.65, 0.50);
  EXPECT_FALSE(isKnown(traced[20]));
  EXPECT_FALSE(isKnown(traced.back()));
}

TEST(TerrainAlgorithms, RecordedTrajectoryFillsUnknownEvidenceMask)
{
  const std::vector<std::uint8_t> recorded_trajectory = {0, 1, 1, 0};
  std::vector<std::uint8_t> free = {1, 0, 0, 1};
  applyTrajectoryFreeEvidence(recorded_trajectory, &free);
  EXPECT_EQ(1, free[0]);
  EXPECT_EQ(1, free[1]);
  EXPECT_EQ(1, free[2]);
  EXPECT_EQ(1, free[3]);
}

class RampOccupancyMergeTest : public ::testing::TestWithParam<double>
{
};

TEST_P(RampOccupancyMergeTest, ContinuousGroundClearsLegacyAbsoluteZObstacles)
{
  constexpr std::size_t kCells = 80U;
  constexpr double kResolution = 0.05;
  constexpr std::uint8_t kOccupied = 0U;
  constexpr std::uint8_t kUnknown = 205U;
  constexpr std::uint8_t kFree = 254U;
  const double tangent = std::tan(GetParam() *
                                  3.14159265358979323846 / 180.0);
  std::vector<std::uint8_t> baseline(kCells, kFree);
  std::vector<std::uint8_t> verified_ground(kCells, 1U);
  std::vector<std::uint8_t> trajectory_only(kCells, 0U);
  std::vector<std::uint8_t> obstacles(kCells, 0U);
  for (std::size_t i = 0; i < kCells; ++i)
  {
    const double z = -0.35 + tangent * (i + 0.5) * kResolution;
    if (z >= 0.05)
    {
      baseline[i] = kOccupied;
    }
  }

  const std::vector<std::uint8_t> merged = mergeOccupancyEvidence(
      baseline, verified_ground, trajectory_only, obstacles,
      kUnknown, kFree, kOccupied);
  EXPECT_TRUE(std::all_of(merged.begin(), merged.end(),
                          [](std::uint8_t value) { return value == kFree; }));
}

INSTANTIATE_TEST_SUITE_P(TraversableSlopeProfiles,
                         RampOccupancyMergeTest,
                         ::testing::Values(10.0, 20.0, 30.0));

TEST(TerrainAlgorithms, OccupancyMergeNeverClearsWallOrRejectedStep)
{
  constexpr std::uint8_t kOccupied = 0U;
  constexpr std::uint8_t kUnknown = 205U;
  constexpr std::uint8_t kFree = 254U;
  const std::vector<std::uint8_t> baseline = {
      kFree, kOccupied, kOccupied, kUnknown, kUnknown};
  const std::vector<std::uint8_t> verified_ground = {1U, 1U, 0U, 0U, 0U};
  const std::vector<std::uint8_t> trajectory_only = {0U, 1U, 1U, 1U, 1U};
  const std::vector<std::uint8_t> obstacles = {0U, 1U, 0U, 1U, 0U};
  const std::vector<std::uint8_t> merged = mergeOccupancyEvidence(
      baseline, verified_ground, trajectory_only, obstacles,
      kUnknown, kFree, kOccupied);
  EXPECT_EQ(kFree, merged[0]);
  EXPECT_EQ(kOccupied, merged[1]);  // Measured wall overrides ground.
  EXPECT_EQ(kOccupied, merged[2]);  // Rejected step stays occupied.
  EXPECT_EQ(kOccupied, merged[3]);  // New obstacle overrides unknown/free.
  EXPECT_EQ(kFree, merged[4]);      // Trajectory may fill unknown only.
}

TEST(TerrainAlgorithms, MeasuresCoverageAndLargestEightConnectedComponent)
{
  const GridGeometry grid = geometry(4, 3);
  const std::vector<std::uint8_t> mask = {
      1, 1, 0, 1,
      1, 0, 0, 0,
      0, 0, 1, 1};
  const BinaryMaskMetrics metrics = measureBinaryMaskQuality(mask, grid);
  EXPECT_EQ(6U, metrics.active_cells);
  EXPECT_EQ(3U, metrics.largest_component_cells);
  EXPECT_DOUBLE_EQ(0.5, metrics.coverage_ratio);
  EXPECT_DOUBLE_EQ(0.5, metrics.largest_component_ratio);
}

TEST(TerrainAlgorithms, QualityMetricConnectsDiagonalLidarSamplesWithoutDilation)
{
  const GridGeometry grid = geometry(4, 4);
  const std::vector<std::uint8_t> diagonal = {
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1};
  const BinaryMaskMetrics metrics = measureBinaryMaskQuality(diagonal, grid);
  EXPECT_EQ(4U, metrics.active_cells);
  EXPECT_EQ(4U, metrics.largest_component_cells);
  EXPECT_DOUBLE_EQ(1.0, metrics.largest_component_ratio);
  EXPECT_EQ(0U, diagonal[1]);  // Measurement did not fill a neighboring cell.
}

TEST(TerrainAlgorithms, EightConnectedQualityDoesNotRelaxGroundGrowth)
{
  const GridGeometry grid = geometry(2, 2);
  const float unknown = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> candidate = {0.0F, unknown, unknown, 0.0F};
  std::vector<float> span = {0.01F, unknown, unknown, 0.01F};
  std::vector<std::uint8_t> seed = {1U, 0U, 0U, 0U};
  const std::vector<std::uint8_t> accepted = growConnectedGround(
      candidate, span, seed, grid, 0.25, 35.0, 0.005, 0);
  EXPECT_EQ(1U, accepted[0]);
  EXPECT_EQ(0U, accepted[3]);

  const std::vector<std::uint8_t> diagonal_quality = {1U, 0U, 0U, 1U};
  const BinaryMaskMetrics metrics =
      measureBinaryMaskQuality(diagonal_quality, grid);
  EXPECT_EQ(2U, metrics.largest_component_cells);
}

TEST(TerrainAlgorithms, RejectsWallStepAndDisconnectedCeilingDuringGrowth)
{
  const GridGeometry grid = geometry(8, 3);
  const float unknown = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> candidate(grid.cellCount(), unknown);
  std::vector<float> span(grid.cellCount(), unknown);
  std::vector<std::uint8_t> seed(grid.cellCount(), 0);
  for (int x = 0; x < 4; ++x)
  {
    candidate[1 * grid.width + x] = 0.0F;
    span[1 * grid.width + x] = 0.01F;
  }
  for (int x = 4; x < 8; ++x)
  {
    candidate[1 * grid.width + x] = 0.30F;  // Discontinuous step.
    span[1 * grid.width + x] = 0.01F;
  }
  candidate[2 * grid.width + 2] = 0.0F;  // Same XY support as a wall.
  span[2 * grid.width + 2] = 1.20F;
  candidate[0 * grid.width + 6] = 2.55F;  // Disconnected ceiling patch.
  candidate[0 * grid.width + 7] = 2.55F;
  span[0 * grid.width + 6] = 0.01F;
  span[0 * grid.width + 7] = 0.01F;
  seed[1 * grid.width] = 1;

  const std::vector<std::uint8_t> accepted = growConnectedGround(
      candidate, span, seed, grid, 0.25, 35.0, 0.01, 0);
  EXPECT_EQ(1, accepted[1 * grid.width + 3]);
  EXPECT_EQ(0, accepted[1 * grid.width + 4]);
  EXPECT_EQ(0, accepted[2 * grid.width + 2]);
  EXPECT_EQ(0, accepted[0 * grid.width + 6]);
  EXPECT_EQ(0, accepted[0 * grid.width + 7]);
}

TEST(TerrainAlgorithms, PreservesContinuousThirtyFiveDegreeSurface)
{
  const GridGeometry grid = geometry(10, 1);
  std::vector<float> candidate(grid.cellCount(), 0.0F);
  std::vector<float> span(grid.cellCount(), 0.01F);
  std::vector<std::uint8_t> seed(grid.cellCount(), 0);
  const double rise = std::tan(35.0 * 3.14159265358979323846 / 180.0) *
                      grid.resolution;
  for (std::size_t x = 0; x < grid.width; ++x)
  {
    candidate[x] = static_cast<float>(x * rise);
  }
  seed[0] = 1;
  const std::vector<std::uint8_t> accepted = growConnectedGround(
      candidate, span, seed, grid, 0.25, 35.0, 1e-6, 0);
  for (const std::uint8_t value : accepted)
  {
    EXPECT_EQ(1, value);
  }
}

class LowStepGrowthTest : public ::testing::TestWithParam<float>
{
};

TEST_P(LowStepGrowthTest, DoesNotAbsorbStepChainOrIsolatedStep)
{
  const GridGeometry grid = geometry(12, 3);
  const float step_height = GetParam();
  std::vector<float> candidate(grid.cellCount(), 0.0F);
  std::vector<float> span(grid.cellCount(), 0.01F);
  std::vector<std::uint8_t> seed(grid.cellCount(), 0);

  // A full raised chain begins at x=6. The isolated raised cell at (3,1)
  // becomes surrounded as flat ground grows, exercising the fill pass too.
  for (int y = 0; y < static_cast<int>(grid.height); ++y)
  {
    for (int x = 6; x < static_cast<int>(grid.width); ++x)
    {
      candidate[static_cast<std::size_t>(y) * grid.width + x] = step_height;
    }
  }
  candidate[1 * grid.width + 3] = step_height;
  seed[1 * grid.width] = 1;

  const std::vector<std::uint8_t> accepted = growConnectedGround(
      candidate, span, seed, grid, 0.25, 35.0, 0.005, 3);
  EXPECT_EQ(1, accepted[1 * grid.width + 2]);
  EXPECT_EQ(0, accepted[1 * grid.width + 3]);
  EXPECT_EQ(1, accepted[1 * grid.width + 5]);
  for (int y = 0; y < static_cast<int>(grid.height); ++y)
  {
    for (int x = 6; x < static_cast<int>(grid.width); ++x)
    {
      EXPECT_EQ(0, accepted[static_cast<std::size_t>(y) * grid.width + x])
          << "step_height=" << step_height << " x=" << x << " y=" << y;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(ApprovedLowStepHeights,
                         LowStepGrowthTest,
                         ::testing::Values(0.05F, 0.07F, 0.09F));

TEST(TerrainGrid, RejectsGeometryMismatch)
{
  const GridGeometry lhs = geometry(10, 20);
  GridGeometry rhs = lhs;
  rhs.origin_x += 0.05;
  std::string error;
  EXPECT_FALSE(geometryMatches(lhs, rhs, 1e-6, &error));
  EXPECT_NE(std::string::npos, error.find("origin differs"));
}

}  // namespace
}  // namespace go2_terrain
