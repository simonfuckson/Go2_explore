#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include "go2_terrain/terrain_grid.hpp"

namespace go2_terrain {
// PDF Route 1 defaults, with Go2's measured origin floor and 35 degree model.
struct SurfaceParameters {
  double cell_size = 0.05;
  double candidate_percentile = 0.05;
  double seed_x = 0.0;
  double seed_y = 0.0;
  double seed_ground_z = -0.15;
  double seed_radius_m = 1.0;
  double seed_height_tolerance_m = 0.12;
  double trusted_seed_height_tolerance_m = 0.08;
  int pmf_max_window_size = 101;
  double pmf_slope = 0.40;
  double pmf_initial_distance_m = 0.04;
  double pmf_max_distance_m = 0.18;
  double pmf_base = 2.0;
  bool pmf_exponential = true;
  double validation_radius_m = 0.30;
  int validation_min_candidates = 8;
  double plane_inlier_tolerance_m = 0.05;
  double plane_min_inlier_ratio = 0.55;
  double plane_min_spread_m = 0.04;
  double candidate_plane_tolerance_m = 0.05;
  double plane_max_rmse_m = 0.035;
  double max_ground_slope_deg = 35.0;
  double connect_radius_m = 0.25;
  double connection_plane_tolerance_m = 0.05;
  double connection_max_normal_delta_deg = 20.0;
  double max_ground_step_m = 0.015;
  double surface_fit_radius_m = 0.80;
  double surface_observation_radius_m = 0.15;
  double surface_gap_fill_radius_m = 0.30;
  int surface_min_candidates = 4;
  int surface_min_component_cells = 100;
  double surface_min_component_fraction = 0.03;
  double min_obstacle_relative_height_m = 0.05;
  double max_obstacle_relative_height_m = 1.50;
  int min_connected_ground_cells = 100;

  double wall_search_radius_m = 1.0;
  double wall_max_nearest_m = 0.8;
  double wall_min_inlier_ratio = 0.75;
  int wall_min_ground_cells = 8;
};
struct SurfaceResult {
  TerrainGrid terrain;
  pcl::PointCloud<pcl::PointXYZI> ground, obstacles;
  std::vector<unsigned> obstacle_count;
  std::vector<std::uint8_t> measured_step, unresolved_vertical;
  std::size_t recovered_wall_cells = 0;
  std::size_t unresolved_vertical_cells = 0;
};
// References inferred at a wall are used only to classify measured obstacles;
// they must never become ground, free-space, or terrain cost evidence.
bool hasVerticalSupport(const std::vector<float>& heights);
SurfaceResult reconstructSurface(const pcl::PointCloud<pcl::PointXYZ>& input,
                                 const GridGeometry& geometry,
                                 const SurfaceParameters& parameters);
}  // namespace go2_terrain
