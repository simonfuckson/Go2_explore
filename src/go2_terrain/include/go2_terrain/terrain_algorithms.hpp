#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "go2_terrain/terrain_grid.hpp"

namespace go2_terrain
{

struct CostParameters
{
  double flat_slope_deg = 8.0;
  double lethal_slope_deg = 30.0;
  float minimum_cost = 15.0F;
  float maximum_soft_cost = 80.0F;
  std::size_t minimum_lethal_cluster_cells = 4;
  double dilation_m = 0.20;
};

struct GroundColumnSummary
{
  float candidate = kUnknownTerrain;
  float support_span = kUnknownTerrain;
  std::size_t support_count = 0;
};

struct PlanarPoint
{
  double x = 0.0;
  double y = 0.0;
};

struct BinaryMaskMetrics
{
  std::size_t active_cells = 0U;
  std::size_t largest_component_cells = 0U;
  double coverage_ratio = 0.0;
  double largest_component_ratio = 0.0;
};

GroundColumnSummary summarizeGroundColumn(const std::vector<float>& samples,
                                          double candidate_quantile,
                                          double support_band_m);
bool isObstacleHeight(double relative_height,
                      double minimum_height,
                      double maximum_height);

// Tracks a continuous ground profile using only trajectory XY. This API
// intentionally has no trajectory-Z input because /odom_nav.z is flattened.
std::vector<float> traceGroundAlongTrajectory(
    const std::vector<PlanarPoint>& trajectory_xy,
    const std::vector<float>& candidate_elevation,
    const std::vector<float>& candidate_support_span,
    const std::vector<std::uint8_t>& admissible_reanchor,
    const GridGeometry& geometry,
    double search_radius_m,
    double maximum_support_span_m,
    double initial_ground_z,
    double initial_tolerance_m,
    double maximum_slope_deg,
    double continuity_margin_m,
    double maximum_reanchor_height_from_initial_m,
    double maximum_gap_m);

// Builds ground seeds inside a true metric disk around each traced trajectory
// sample. The traced height is already bounded by the measured base height,
// local slope continuity, and low-column support. Requiring an additional PMF
// overlap here can eliminate every seed in indoor point clouds where PMF sees
// floor away from the robot path. PMF remains authoritative for long-gap
// re-anchoring and export quality measurement.
std::vector<std::uint8_t> buildTrajectorySeedMask(
    const std::vector<PlanarPoint>& trajectory_xy,
    const std::vector<float>& trajectory_ground_z,
    const std::vector<float>& candidate_elevation,
    const std::vector<float>& candidate_support_span,
    const GridGeometry& geometry,
    double radius_m,
    double maximum_support_span_m,
    double height_tolerance_m);

std::vector<std::uint8_t> growConnectedGround(
    const std::vector<float>& candidate_elevation,
    const std::vector<float>& candidate_support_span,
    const std::vector<std::uint8_t>& initial_seed,
    const GridGeometry& geometry,
    double maximum_support_span_m,
    double maximum_slope_deg,
    double height_margin_m,
    int fill_iterations);

// Adds free evidence from the recorded, gap-bounded robot trajectory. The
// occupancy merge limits this evidence to unknown cells, and obstacles must
// be applied afterward so measured structure remains authoritative.
void applyTrajectoryFreeEvidence(
    const std::vector<std::uint8_t>& recorded_trajectory_cells,
    std::vector<std::uint8_t>* free_cells);

// Measures mask coverage and the largest eight-connected component. Primary
// ground propagation remains four-connected and the existing height-bounded
// hole fill is unchanged; the quality metric alone accepts diagonal sampling
// adjacency without dilating or inventing cells.
BinaryMaskMetrics measureBinaryMaskQuality(
    const std::vector<std::uint8_t>& mask,
    const GridGeometry& geometry);

// Combines a legacy occupancy image with strong terrain evidence. Verified
// continuous ground may reclassify a legacy absolute-Z false obstacle.
// Trajectory-only evidence may fill unknown but cannot clear an occupied
// baseline cell. Measured terrain obstacles are always applied last.
std::vector<std::uint8_t> mergeOccupancyEvidence(
    const std::vector<std::uint8_t>& baseline,
    const std::vector<std::uint8_t>& verified_ground,
    const std::vector<std::uint8_t>& verified_trajectory_free,
    const std::vector<std::uint8_t>& obstacles,
    std::uint8_t unknown_value,
    std::uint8_t free_value,
    std::uint8_t occupied_value);

float slopeToSoftCost(double slope_deg, const CostParameters& parameters);

// Converts a slope grid to ROS cost values. Unknown input stays unknown. Cells
// above lethal_slope_deg become lethal only when their 8-connected component
// meets minimum_lethal_cluster_cells; smaller components remain soft-cost cells.
std::vector<std::uint8_t> buildSlopeCostLayer(
    const std::vector<float>& slope_deg,
    const GridGeometry& geometry,
    const CostParameters& parameters);

// Preserves the validated legacy occupancy-export behavior: radius_m is
// rounded to cells and expansion uses four-connectivity. At 0.03 m on the
// locked 0.05 m grid this intentionally expands one cardinal cell.
void dilateBinaryMaskMetric(const std::vector<std::uint8_t>& source,
                            std::vector<std::uint8_t>* destination,
                            const GridGeometry& geometry,
                            double radius_m);

// Expands the maximum nonzero cost by a metric disk. Unknown source cells do
// not create costs, and unknown destination cells remain unknown.
void dilateKnownCosts(std::vector<std::uint8_t>* costs,
                      const GridGeometry& geometry,
                      double radius_m);

// Merges one terrain cell using ROS costmap values. Static-map unknown and
// lethal cells are authoritative; terrain may only raise an already-known
// traversable cell.
std::uint8_t mergeTerrainCost(std::uint8_t master_cost,
                              std::uint8_t terrain_cost);

}  // namespace go2_terrain
