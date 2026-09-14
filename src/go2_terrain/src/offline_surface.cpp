// Adapted from WheelTech terrain_reclassify.cpp, commit ad23ac64.
// See THIRD_PARTY.md for source provenance and licensing status.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include "go2_terrain/offline_surface.hpp"

namespace go2_terrain {
namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

struct Plane {
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  double rmse = std::numeric_limits<double>::infinity();
  int inliers = 0;
};

struct Cell {
  std::vector<float> samples;
  float candidate = std::numeric_limits<float>::quiet_NaN();
  bool trusted_seed = false;
  bool valid_candidate = false;
  bool connected = false;
  Plane plane;
  Plane surface_plane;
  float surface_z = std::numeric_limits<float>::quiet_NaN();
};

struct Observation {
  Eigen::Vector3d row;
  double z = 0.0;
  double base_weight = 1.0;
};

bool finite(const Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

double percentile(std::vector<float>* values, double fraction) {
  if (values->empty()) return std::numeric_limits<double>::quiet_NaN();
  fraction = std::max(0.0, std::min(1.0, fraction));
  const std::size_t selected = static_cast<std::size_t>(std::round(
      fraction * static_cast<double>(values->size() - 1)));
  std::nth_element(values->begin(), values->begin() + selected, values->end());
  return (*values)[selected];
}

std::vector<std::pair<int, int>> diskOffsets(int radius) {
  std::vector<std::pair<int, int>> offsets;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      if (dx * dx + dy * dy <= radius * radius) {
        offsets.emplace_back(dx, dy);
      }
    }
  }
  return offsets;
}

}  // namespace

class TerrainReclassifier {
 public:
  TerrainReclassifier(const pcl::PointCloud<pcl::PointXYZ>& input,
                      const GridGeometry& geometry, const SurfaceParameters& p) {
    cell_size_ = p.cell_size;
    candidate_percentile_ = p.candidate_percentile;
    seed_x_ = p.seed_x;
    seed_y_ = p.seed_y;
    seed_ground_z_ = p.seed_ground_z;
    seed_radius_m_ = p.seed_radius_m;
    seed_height_tolerance_m_ = p.seed_height_tolerance_m;
    trusted_seed_height_tolerance_m_ = p.trusted_seed_height_tolerance_m;
    pmf_max_window_size_ = p.pmf_max_window_size;
    pmf_slope_ = p.pmf_slope;
    pmf_initial_distance_m_ = p.pmf_initial_distance_m;
    pmf_max_distance_m_ = p.pmf_max_distance_m;
    pmf_base_ = p.pmf_base;
    pmf_exponential_ = p.pmf_exponential;
    validation_radius_m_ = p.validation_radius_m;
    validation_min_candidates_ = p.validation_min_candidates;
    plane_inlier_tolerance_m_ = p.plane_inlier_tolerance_m;
    plane_min_inlier_ratio_ = p.plane_min_inlier_ratio;
    plane_min_spread_m_ = p.plane_min_spread_m;
    candidate_plane_tolerance_m_ = p.candidate_plane_tolerance_m;
    plane_max_rmse_m_ = p.plane_max_rmse_m;
    max_ground_slope_deg_ = p.max_ground_slope_deg;
    connect_radius_m_ = p.connect_radius_m;
    connection_plane_tolerance_m_ = p.connection_plane_tolerance_m;
    connection_max_normal_delta_deg_ = p.connection_max_normal_delta_deg;
    max_ground_step_m_ = p.max_ground_step_m;
    surface_fit_radius_m_ = p.surface_fit_radius_m;
    surface_observation_radius_m_ = p.surface_observation_radius_m;
    surface_gap_fill_radius_m_ = p.surface_gap_fill_radius_m;
    surface_min_candidates_ = p.surface_min_candidates;
    surface_min_component_cells_ = p.surface_min_component_cells;
    surface_min_component_fraction_ = p.surface_min_component_fraction;
    min_obstacle_relative_height_m_ = p.min_obstacle_relative_height_m;
    max_obstacle_relative_height_m_ = p.max_obstacle_relative_height_m;
    min_connected_ground_cells_ = p.min_connected_ground_cells;
    parameters_ = p;
    geometry_ = geometry;
    if (!geometry.valid() || geometry.cellCount() > 4000000 ||
        std::abs(geometry.resolution - cell_size_) > 1e-9 ||
        std::abs(geometry.origin_yaw) > 1e-9)
      throw std::runtime_error("Invalid or excessively large terrain geometry");
    input_.reset(new Cloud);
    input_->reserve(input.size());
    for (const auto& value : input) {
      Point point; point.x=value.x; point.y=value.y; point.z=value.z; point.intensity=0;
      if (finite(point)) {
        if (point.x<geometry.origin_x || point.y<geometry.origin_y ||
            point.x>=geometry.origin_x+geometry.width*geometry.resolution ||
            point.y>=geometry.origin_y+geometry.height*geometry.resolution)
          throw std::runtime_error("Source point lies outside declared map geometry");
        input_->push_back(point);
      }
    }
    if (input_->empty()) throw std::runtime_error("No finite map points");
    initializeOffsets();
    buildTrustedSeeds();
    buildCandidateGrid();
    validateCandidates();
    connectSurface();
    buildSurface();
    filterSurfaceComponents();
    retainOriginConnectedSurface();
    detectMeasuredSteps();
    classifyObstacles();
  }

  SurfaceResult result() const {
    SurfaceResult output;
    output.terrain.resize(geometry_);
    output.obstacle_count.assign(cells_.size(), 0);
    output.measured_step=measured_step_;
    output.unresolved_vertical=unresolved_vertical_;
    output.ground = ground_;
    output.obstacles = obstacles_;
    output.recovered_wall_cells = recovered_wall_cells_;
    output.unresolved_vertical_cells = unresolved_vertical_cells_;
    for (std::size_t i=0; i<cells_.size(); ++i) {
      const auto& cell=cells_[i];
      if (!std::isfinite(cell.surface_z)) continue;
      output.terrain.elevation[i]=cell.surface_z;
      output.terrain.slope_deg[i]=std::atan(std::hypot(cell.surface_plane.a,
                                   cell.surface_plane.b))*180.0/M_PI;
      output.terrain.roughness[i]=cell.surface_plane.rmse;
      output.terrain.step[i]=0;
      output.terrain.confidence[i]=std::min(100, 50+cell.surface_plane.inliers);
      const int x=i%width_, y=i/width_;
      // Step is the residual after subtracting the fitted slope, not ramp rise.
      for (int dy=-1;dy<=1;++dy) for (int dx=-1;dx<=1;++dx) {
        if (!inside(x+dx,y+dy)) continue;
        const auto& other=cells_[index(x+dx,y+dy)];
        if (!std::isfinite(other.surface_z)) continue;
        output.terrain.step[i]=std::max(output.terrain.step[i],
          static_cast<float>(std::abs(other.surface_z-cell.surface_z-
            cell.surface_plane.a*dx*cell_size_-cell.surface_plane.b*dy*cell_size_)));
      }
    }
    for (const auto& point: obstacles_) {
      int x,y; if (pointCell(point,&x,&y)) ++output.obstacle_count[index(x,y)];
    }
    return output;
  }

 private:
  void initializeOffsets() {
    for (double value: {cell_size_, candidate_percentile_, seed_x_, seed_y_, seed_ground_z_, seed_radius_m_, seed_height_tolerance_m_, trusted_seed_height_tolerance_m_, pmf_slope_, pmf_initial_distance_m_, pmf_max_distance_m_, pmf_base_, validation_radius_m_, plane_inlier_tolerance_m_, plane_min_inlier_ratio_, plane_min_spread_m_, candidate_plane_tolerance_m_, plane_max_rmse_m_, max_ground_slope_deg_, connect_radius_m_, connection_plane_tolerance_m_, connection_max_normal_delta_deg_, max_ground_step_m_, surface_fit_radius_m_, surface_observation_radius_m_, surface_gap_fill_radius_m_, surface_min_component_fraction_, min_obstacle_relative_height_m_, max_obstacle_relative_height_m_, parameters_.wall_search_radius_m,
                        parameters_.wall_max_nearest_m, parameters_.wall_min_inlier_ratio})
      if (!std::isfinite(value)) throw std::runtime_error("Nonfinite terrain parameter");
    if (pmf_max_window_size_>201 || pmf_base_<=1.0 || surface_fit_radius_m_>2.0 ||
        connect_radius_m_>0.50 || validation_radius_m_>1.0 || surface_gap_fill_radius_m_>0.50)
      throw std::runtime_error("Surface support radii exceed bounded offline profile");
    if (cell_size_ <= 0.0 || candidate_percentile_ < 0.0 ||
        candidate_percentile_ > 1.0 || seed_radius_m_ < cell_size_ ||
        trusted_seed_height_tolerance_m_ <= 0.0 ||
        pmf_max_window_size_ < 3 || pmf_slope_ < 0.0 ||
        pmf_initial_distance_m_ < 0.0 ||
        pmf_max_distance_m_ < pmf_initial_distance_m_ || pmf_base_ < 1.0 ||
        validation_radius_m_ < cell_size_ || validation_min_candidates_ < 3 ||
        plane_inlier_tolerance_m_ <= 0.0 || plane_min_inlier_ratio_ <= 0.0 ||
        plane_min_inlier_ratio_ > 1.0 || plane_min_spread_m_ <= 0.0 ||
        candidate_plane_tolerance_m_ <= 0.0 || plane_max_rmse_m_ <= 0.0 ||
        max_ground_slope_deg_ <= 0.0 || max_ground_slope_deg_ >= 89.0 ||
        connect_radius_m_ < cell_size_ || connection_plane_tolerance_m_ <= 0.0 ||
        connection_max_normal_delta_deg_ <= 0.0 ||
        connection_max_normal_delta_deg_ >= 90.0 || max_ground_step_m_ < 0.0 ||
        surface_fit_radius_m_ < cell_size_ ||
        surface_observation_radius_m_ < 0.0 ||
        surface_gap_fill_radius_m_ < surface_observation_radius_m_ ||
        surface_min_candidates_ < 3 || surface_min_component_cells_ < 1 ||
        surface_min_component_fraction_ < 0.0 ||
        surface_min_component_fraction_ > 1.0 ||
        min_obstacle_relative_height_m_ < 0.0 ||
        max_obstacle_relative_height_m_ <= min_obstacle_relative_height_m_) {
      throw std::runtime_error("Invalid terrain surface parameters");
    }
    if (!std::isfinite(parameters_.wall_search_radius_m) || parameters_.wall_search_radius_m<=0 ||
        parameters_.wall_search_radius_m>1.0 || parameters_.wall_max_nearest_m<=0 ||
        parameters_.wall_max_nearest_m>0.8 || parameters_.wall_min_inlier_ratio<0.75 ||
        parameters_.wall_min_inlier_ratio>1 || parameters_.wall_min_ground_cells<8)
      throw std::runtime_error("Invalid wall-reference limits");
    validation_offsets_ = diskOffsets(std::max(
        1, static_cast<int>(std::ceil(validation_radius_m_ / cell_size_))));
    connect_offsets_ = diskOffsets(std::max(
        1, static_cast<int>(std::ceil(connect_radius_m_ / cell_size_))));
    surface_offsets_ = diskOffsets(std::max(
        1, static_cast<int>(std::ceil(surface_fit_radius_m_ / cell_size_))));
    observation_offsets_ = diskOffsets(std::max(
        0, static_cast<int>(std::ceil(
               surface_observation_radius_m_ / cell_size_))));
    gap_fill_offsets_ = diskOffsets(std::max(
        1, static_cast<int>(
               std::ceil(surface_gap_fill_radius_m_ / cell_size_))));
  }

  void buildTrustedSeeds() {
    if (!trusted_seeds_.empty()) return;
    pcl::ApproximateProgressiveMorphologicalFilter<Point> pmf;
    pmf.setInputCloud(input_);
    pmf.setCellSize(static_cast<float>(cell_size_));
    pmf.setMaxWindowSize(pmf_max_window_size_);
    pmf.setSlope(static_cast<float>(pmf_slope_));
    pmf.setInitialDistance(static_cast<float>(pmf_initial_distance_m_));
    pmf.setMaxDistance(static_cast<float>(pmf_max_distance_m_));
    pmf.setBase(static_cast<float>(pmf_base_));
    pmf.setExponential(pmf_exponential_);
    std::vector<int> indices;
    pmf.extract(indices);
    trusted_seeds_.reserve(indices.size());
    for (const int point_index : indices) {
      if (point_index < 0 ||
          static_cast<std::size_t>(point_index) >= input_->size()) {
        continue;
      }
      const Point& point = input_->points[point_index];
      if (finite(point)) trusted_seeds_.push_back(point);
    }
    if (trusted_seeds_.empty()) {
      throw std::runtime_error("PMF returned no conservative terrain seeds");
    }
  }

  int index(int x, int y) const { return y * width_ + x; }

  bool inside(int x, int y) const {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
  }

  bool pointCell(const Point& point, int* x, int* y) const {
    *x = static_cast<int>(std::floor((point.x - min_x_) / cell_size_));
    *y = static_cast<int>(std::floor((point.y - min_y_) / cell_size_));
    return inside(*x, *y);
  }

  void buildCandidateGrid() {
    min_x_=geometry_.origin_x; min_y_=geometry_.origin_y;
    width_=geometry_.width; height_=geometry_.height;
    cells_.resize(static_cast<std::size_t>(width_) * height_);

    for (const Point& point : input_->points) {
      if (!finite(point)) continue;
      int x = 0;
      int y = 0;
      if (!pointCell(point, &x, &y)) continue;
      cells_[index(x, y)].samples.push_back(point.z);
    }
    for (Cell& cell : cells_) {
      if (cell.samples.empty()) continue;
      cell.candidate = static_cast<float>(
          percentile(&cell.samples, candidate_percentile_));
      // Retain measured columns for evidence-based wall recovery.
      ++candidate_cells_;
    }
  }

  bool fitPlane(int center_x, int center_y,
                const std::vector<std::pair<int, int>>& offsets,
                bool connected_only, int minimum_candidates,
                bool require_center, Plane* plane, bool wall_reference=false) const {
    std::vector<Observation> observations;
    observations.reserve(offsets.size());
    for (const auto& offset : offsets) {
      const int x = center_x + offset.first;
      const int y = center_y + offset.second;
      if (!inside(x, y)) continue;
      const Cell& cell = cells_[index(x, y)];
      const float height=wall_reference ? cell.surface_z : cell.candidate;
      if (!std::isfinite(height) || (!wall_reference && connected_only && !cell.connected)) {
        continue;
      }
      const double dx = offset.first * cell_size_;
      const double dy = offset.second * cell_size_;
      observations.push_back({Eigen::Vector3d(dx, dy, 1.0), height,
                              1.0 / (0.05 + std::hypot(dx, dy))});
    }
    if (static_cast<int>(observations.size()) < minimum_candidates) return false;

    std::vector<double> robust_weight(observations.size(), 1.0);
    Eigen::Vector3d coefficients = Eigen::Vector3d::Zero();
    for (int iteration = 0; iteration < 4; ++iteration) {
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      for (std::size_t i = 0; i < observations.size(); ++i) {
        const double weight = observations[i].base_weight * robust_weight[i];
        normal.noalias() += weight * observations[i].row *
                            observations[i].row.transpose();
        rhs.noalias() += weight * observations[i].row * observations[i].z;
      }
      const Eigen::LDLT<Eigen::Matrix3d> decomposition(normal);
      if (decomposition.info() != Eigen::Success) return false;
      coefficients = decomposition.solve(rhs);
      if (!coefficients.allFinite()) return false;
      for (std::size_t i = 0; i < observations.size(); ++i) {
        const double residual = std::abs(
            observations[i].row.dot(coefficients) - observations[i].z);
        robust_weight[i] = residual <= plane_inlier_tolerance_m_
                               ? 1.0
                               : plane_inlier_tolerance_m_ / residual;
      }
    }

    std::vector<std::size_t> inliers;
    inliers.reserve(observations.size());
    for (std::size_t i = 0; i < observations.size(); ++i) {
      if (std::abs(observations[i].row.dot(coefficients) - observations[i].z) <=
          plane_inlier_tolerance_m_) {
        inliers.push_back(i);
      }
    }
    if (static_cast<int>(inliers.size()) < minimum_candidates ||
        static_cast<double>(inliers.size()) / observations.size() <
            (wall_reference ? parameters_.wall_min_inlier_ratio : plane_min_inlier_ratio_)) {
      return false;
    }

    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    double total_weight = 0.0;
    for (const std::size_t i : inliers) {
      const double weight = observations[i].base_weight;
      normal.noalias() += weight * observations[i].row *
                          observations[i].row.transpose();
      rhs.noalias() += weight * observations[i].row * observations[i].z;
      mean += weight * observations[i].row.head<2>();
      total_weight += weight;
    }
    if (total_weight <= 0.0) return false;
    mean /= total_weight;
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const std::size_t i : inliers) {
      const Eigen::Vector2d centered = observations[i].row.head<2>() - mean;
      covariance.noalias() += observations[i].base_weight * centered *
                              centered.transpose();
    }
    covariance /= total_weight;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> spread_solver(covariance);
    if (spread_solver.info() != Eigen::Success ||
        spread_solver.eigenvalues().minCoeff() <
            plane_min_spread_m_ * plane_min_spread_m_) {
      return false;
    }

    const Eigen::LDLT<Eigen::Matrix3d> decomposition(normal);
    if (decomposition.info() != Eigen::Success) return false;
    coefficients = decomposition.solve(rhs);
    if (!coefficients.allFinite()) return false;

    const double slope = std::hypot(coefficients.x(), coefficients.y());
    if (slope > std::tan(max_ground_slope_deg_ * M_PI / 180.0) + 1e-6) return false;
    double squared_error = 0.0;
    for (const std::size_t i : inliers) {
      const double residual =
          observations[i].row.dot(coefficients) - observations[i].z;
      squared_error += observations[i].base_weight * residual * residual;
    }
    const double rmse = std::sqrt(squared_error / total_weight);
    if (rmse > plane_max_rmse_m_) return false;

    if (require_center) {
      const Cell& center = cells_[index(center_x, center_y)];
      if (!std::isfinite(center.candidate) ||
          std::abs(center.candidate - coefficients.z()) >
              candidate_plane_tolerance_m_) {
        return false;
      }
    }
    plane->a = coefficients.x();
    plane->b = coefficients.y();
    plane->c = coefficients.z();
    plane->rmse = rmse;
    plane->inliers = static_cast<int>(inliers.size());
    return true;
  }

  void validateCandidates() {
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        Cell& cell = cells_[index(x, y)];
        if (!std::isfinite(cell.candidate)) continue;
        Plane plane;
        if (!fitPlane(x, y, validation_offsets_, false,
                      validation_min_candidates_, true, &plane)) {
          continue;
        }
        cell.valid_candidate = true;
        cell.plane = plane;
        ++valid_candidate_cells_;
      }
    }
  }

  bool compatible(const Cell& current, const Cell& neighbor,
                  int dx, int dy) const {
    if (!current.valid_candidate || !neighbor.valid_candidate) return false;
    const double world_dx = dx * cell_size_;
    const double world_dy = dy * cell_size_;
    const double distance = std::hypot(world_dx, world_dy);
    const double allowed_step = max_ground_step_m_ +
        std::tan(max_ground_slope_deg_ * M_PI / 180.0) * distance;
    if (std::abs(neighbor.candidate - current.candidate) > allowed_step) {
      return false;
    }
    const double expected_neighbor = current.plane.a * world_dx +
                                     current.plane.b * world_dy +
                                     current.plane.c;
    const double expected_current = neighbor.plane.a * -world_dx +
                                    neighbor.plane.b * -world_dy +
                                    neighbor.plane.c;
    if (std::abs(neighbor.candidate - expected_neighbor) >
            connection_plane_tolerance_m_ ||
        std::abs(current.candidate - expected_current) >
            connection_plane_tolerance_m_) {
      return false;
    }
    Eigen::Vector3d current_normal(-current.plane.a, -current.plane.b, 1.0);
    Eigen::Vector3d neighbor_normal(-neighbor.plane.a, -neighbor.plane.b, 1.0);
    current_normal.normalize();
    neighbor_normal.normalize();
    const double cosine = std::max(-1.0, std::min(
        1.0, current_normal.dot(neighbor_normal)));
    const double angle = std::acos(cosine) * 180.0 / M_PI;
    return angle <= connection_max_normal_delta_deg_;
  }

  void connectSurface() {
    std::deque<int> queue;
    for (const Point& point : trusted_seeds_.points) {
      if (!finite(point)) continue;
      int x = 0;
      int y = 0;
      if (!pointCell(point, &x, &y)) continue;
      Cell& cell = cells_[index(x, y)];
      if (!std::isfinite(cell.candidate) ||
          std::abs(cell.candidate - point.z) >
              trusted_seed_height_tolerance_m_) {
        continue;
      }
      if (!cell.trusted_seed) {
        cell.trusted_seed = true;
        ++trusted_seed_cells_;
      }
    }
    // The origin seed is a useful frame sanity check when local observations
    // are planar enough.  It must not be the only growth source: a saved PCD
    // contains no sensor trajectory, and a ramp edge or a single sparse scan
    // gap can make the origin cell fail strict local-plane validation.
    const double seed_radius_sq = seed_radius_m_ * seed_radius_m_;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        Cell& cell = cells_[index(x, y)];
        if (!cell.valid_candidate) continue;
        const double world_x = min_x_ + (x + 0.5) * cell_size_;
        const double world_y = min_y_ + (y + 0.5) * cell_size_;
        const double distance_sq = (world_x - seed_x_) * (world_x - seed_x_) +
                                   (world_y - seed_y_) * (world_y - seed_y_);
        if (distance_sq <= seed_radius_sq &&
            std::abs(cell.candidate - seed_ground_z_) <=
                seed_height_tolerance_m_) {
          if (!cell.connected) {
            cell.connected = true;
            queue.push_back(index(x, y));
            ++origin_seed_cells_;
          }
        }
      }
    }
    // PMF selects the locally lowest morphological surface.  All of its cells
    // provide distributed support for the later robust plane fit even when
    // their absolute Z differs greatly from the map origin.  Only locally
    // validated PMF cells are allowed to grow into neighboring candidates.
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        Cell& cell = cells_[index(x, y)];
        if (!cell.trusted_seed) continue;
        ++distributed_seed_cells_;
        if (cell.connected) continue;
        cell.connected = true;
        if (cell.valid_candidate) queue.push_back(index(x, y));
      }
    }
    if (origin_seed_cells_ == 0) {
      throw std::runtime_error("No measured planar origin floor seed; check base height and map frame");
    }
    if (queue.empty()) {
      throw std::runtime_error("No valid distributed terrain seed was found");
    }

    while (!queue.empty()) {
      const int current_index = queue.front();
      queue.pop_front();
      const int current_x = current_index % width_;
      const int current_y = current_index / width_;
      const Cell& current = cells_[current_index];
      for (const auto& offset : connect_offsets_) {
        if (offset.first == 0 && offset.second == 0) continue;
        const int x = current_x + offset.first;
        const int y = current_y + offset.second;
        if (!inside(x, y)) continue;
        Cell& neighbor = cells_[index(x, y)];
        if (neighbor.connected || !neighbor.valid_candidate) continue;
        if (!compatible(current, neighbor, offset.first, offset.second)) continue;
        neighbor.connected = true;
        queue.push_back(index(x, y));
      }
    }

    for (const Cell& cell : cells_) {
      connected_cells_ += cell.connected ? 1 : 0;
      connected_trusted_seed_cells_ +=
          cell.connected && cell.trusted_seed ? 1 : 0;
    }
    if (connected_cells_ < static_cast<std::size_t>(min_connected_ground_cells_)) {
      throw std::runtime_error(
          "Connected terrain is too small; check the seed and plane parameters");
    }
  }

  bool hasOpposingConnectedSupport(int center_x, int center_y) const {
    uint8_t sector_mask = 0;
    for (const auto& offset : gap_fill_offsets_) {
      if (offset.first == 0 && offset.second == 0) continue;
      const int x = center_x + offset.first;
      const int y = center_y + offset.second;
      if (!inside(x, y)) continue;
      const Cell& cell = cells_[index(x, y)];
      if (!cell.connected || !std::isfinite(cell.candidate)) continue;
      const double angle = std::atan2(
          static_cast<double>(offset.second),
          static_cast<double>(offset.first));
      int sector = static_cast<int>(std::floor(
          (angle + M_PI + M_PI / 8.0) / (M_PI / 4.0)));
      sector %= 8;
      if (sector < 0) sector += 8;
      sector_mask |= static_cast<uint8_t>(1u << sector);
    }
    for (int first = 0; first < 8; ++first) {
      if ((sector_mask & (1u << first)) == 0) continue;
      for (int second = first + 1; second < 8; ++second) {
        if ((sector_mask & (1u << second)) == 0) continue;
        const int separation = std::min(second - first, 8 - (second - first));
        if (separation >= 3) return true;
      }
    }
    return false;
  }

  void buildSurface() {
    const auto transition_offsets=diskOffsets(std::max(2,static_cast<int>(
        std::ceil(validation_radius_m_/cell_size_))));
    const auto detail_offsets=diskOffsets(std::max(2,static_cast<int>(
        std::ceil(0.5*validation_radius_m_/cell_size_))));
    ground_.reserve(connected_cells_ * 2);
    candidates_.reserve(connected_cells_);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        Cell& cell = cells_[index(x, y)];
        if (cell.connected) {
          Point candidate;
          candidate.x = static_cast<float>(min_x_ + (x + 0.5) * cell_size_);
          candidate.y = static_cast<float>(min_y_ + (y + 0.5) * cell_size_);
          candidate.z = cell.candidate;
          candidate.intensity = 0.0f;
          candidates_.push_back(candidate);
        }

        bool observed_nearby = false;
        for (const auto& offset : observation_offsets_) {
          const int nx = x + offset.first;
          const int ny = y + offset.second;
          if (inside(nx, ny) &&
              std::isfinite(cells_[index(nx, ny)].candidate)) {
            observed_nearby = true;
            break;
          }
        }
        const bool internal_gap =
            !observed_nearby && hasOpposingConnectedSupport(x, y);
        if (!observed_nearby && !internal_gap) continue;

        Plane surface;
        bool fitted=fitPlane(x,y,surface_offsets_,true,surface_min_candidates_,false,&surface);
        // A single broad plane is inappropriate at a ramp crest or foot.
        // Refine support locally instead of loosening the residual limits or
        // erasing the crease. A wall candidate is not a connected floor anchor.
        auto consistent=[&]() {
          return !cell.connected || !std::isfinite(cell.candidate) ||
              std::abs(surface.c-cell.candidate)<=candidate_plane_tolerance_m_;
        };
        if (!fitted || !consistent())
          fitted=fitPlane(x,y,transition_offsets,true,surface_min_candidates_,false,&surface);
        if (!fitted || !consistent())
          fitted=fitPlane(x,y,detail_offsets,true,surface_min_candidates_,false,&surface);
        if (!fitted || !consistent()) continue;
        cell.surface_z = static_cast<float>(surface.c);
        cell.surface_plane = surface;
        Point point;
        point.x = static_cast<float>(min_x_ + (x + 0.5) * cell_size_);
        point.y = static_cast<float>(min_y_ + (y + 0.5) * cell_size_);
        point.z = cell.surface_z;
        point.intensity = 0.0f;
        ground_.push_back(point);
        if (internal_gap) ++gap_filled_surface_cells_;
      }
    }
  }

  void filterSurfaceComponents() {
    std::vector<int> labels(cells_.size(), -1);
    std::vector<std::vector<int>> components;
    const int neighbor_dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int neighbor_dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const int start = index(x, y);
        if (!std::isfinite(cells_[start].surface_z) || labels[start] >= 0) {
          continue;
        }
        const int component_id = static_cast<int>(components.size());
        components.emplace_back();
        std::deque<int> queue;
        queue.push_back(start);
        labels[start] = component_id;
        while (!queue.empty()) {
          const int current = queue.front();
          queue.pop_front();
          components.back().push_back(current);
          const int current_x = current % width_;
          const int current_y = current / width_;
          for (int direction = 0; direction < 8; ++direction) {
            const int neighbor_x = current_x + neighbor_dx[direction];
            const int neighbor_y = current_y + neighbor_dy[direction];
            if (!inside(neighbor_x, neighbor_y)) continue;
            const int neighbor = index(neighbor_x, neighbor_y);
            if (labels[neighbor] >= 0 ||
                !std::isfinite(cells_[neighbor].surface_z)) {
              continue;
            }
            labels[neighbor] = component_id;
            queue.push_back(neighbor);
          }
        }
      }
    }

    surface_component_count_ = components.size();
    if (components.empty()) {
      throw std::runtime_error("No terrain surface survived local-plane fitting");
    }
    std::size_t largest_component_cells = 0;
    for (const auto& component : components) {
      largest_component_cells = std::max(
          largest_component_cells, component.size());
    }
    const std::size_t relative_minimum = static_cast<std::size_t>(std::ceil(
        surface_min_component_fraction_ * largest_component_cells));
    const std::size_t minimum_cells = std::max(
        static_cast<std::size_t>(surface_min_component_cells_),
        relative_minimum);

    for (const auto& component : components) {
      if (component.size() >= minimum_cells) {
        ++kept_surface_component_count_;
        continue;
      }
      removed_surface_cells_ += component.size();
      for (const int cell_index : component) {
        cells_[cell_index].surface_z =
            std::numeric_limits<float>::quiet_NaN();
      }
    }

    ground_.clear();
    ground_.reserve(cells_.size() - removed_surface_cells_);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const Cell& cell = cells_[index(x, y)];
        if (!std::isfinite(cell.surface_z)) continue;
        Point point;
        point.x = static_cast<float>(min_x_ + (x + 0.5) * cell_size_);
        point.y = static_cast<float>(min_y_ + (y + 0.5) * cell_size_);
        point.z = cell.surface_z;
        point.intensity = 0.0f;
        ground_.push_back(point);
      }
    }
  }

  void retainOriginConnectedSurface() {
    // PMF supplies distributed candidates, not permission to call an isolated
    // roof a floor. Follow the reconstructed height surface from the measured
    // start floor, allowing only bounded sampling gaps and compatible planes.
    std::vector<std::uint8_t> accepted(cells_.size(),0);
    std::deque<int> queue;
    for (int y=0;y<height_;++y) for (int x=0;x<width_;++x) {
      const auto& c=cells_[index(x,y)];
      if (std::isfinite(c.surface_z) &&
          std::hypot(min_x_+(x+.5)*cell_size_-seed_x_,
                     min_y_+(y+.5)*cell_size_-seed_y_)<=seed_radius_m_ &&
          std::abs(c.surface_z-seed_ground_z_)<=seed_height_tolerance_m_) {
        accepted[index(x,y)]=1;queue.push_back(index(x,y));
      }
    }
    const auto offsets=diskOffsets(static_cast<int>(std::floor(surface_gap_fill_radius_m_/cell_size_+1e-9)));
    while (!queue.empty()) {
      const int i=queue.front();queue.pop_front();
      const int x=i%width_, y=i/width_; const auto& a=cells_[i];
      for (const auto& d:offsets) {
        const int nx=x+d.first,ny=y+d.second;
        if (!inside(nx,ny)) continue;
        const int j=index(nx,ny);const auto& b=cells_[j];
        if (accepted[j] || !std::isfinite(b.surface_z)) continue;
        const double dx=d.first*cell_size_,dy=d.second*cell_size_;
        if (std::abs(b.surface_z-a.surface_z)>max_ground_step_m_+
            std::tan(max_ground_slope_deg_*M_PI/180)*std::hypot(dx,dy)+1e-6) continue;
        if (std::abs(b.surface_z-a.surface_z-a.surface_plane.a*dx-a.surface_plane.b*dy)>
            connection_plane_tolerance_m_ ||
            std::abs(b.surface_z-a.surface_z-b.surface_plane.a*dx-b.surface_plane.b*dy)>
            connection_plane_tolerance_m_) continue;
        accepted[j]=1;queue.push_back(j);
      }
    }
    ground_.clear();
    for (int y=0;y<height_;++y) for (int x=0;x<width_;++x) {
      auto& c=cells_[index(x,y)];
      if (!accepted[index(x,y)]) { c.surface_z=std::numeric_limits<float>::quiet_NaN();continue; }
      Point p;p.x=min_x_+(x+.5)*cell_size_;p.y=min_y_+(y+.5)*cell_size_;
      p.z=c.surface_z;p.intensity=0;ground_.push_back(p);
    }
    if (ground_.size()<static_cast<std::size_t>(min_connected_ground_cells_))
      throw std::runtime_error("Too little floor connected to the measured origin; rescan disconnected regions");
  }

  void detectMeasuredSteps() {
    // Robust fitting can smooth a regular staircase into an apparent ramp.
    // Detect a measured jump relative to the two same-side gradients, and
    // require three spatially adjacent edge cells to suppress single outliers.
    std::vector<std::uint8_t> edges(cells_.size(),0);
    const int dxs[]={-1,1,0,0}, dys[]={0,0,-1,1};
    const double slope_rise=std::tan(max_ground_slope_deg_*M_PI/180)*cell_size_;
    for (int y=2;y<height_-2;++y) for (int x=2;x<width_-2;++x) {
      const float high=cells_[index(x,y)].candidate;
      if (!std::isfinite(high)) continue;
      for (int direction=0;direction<4;++direction) {
        const int dx=dxs[direction],dy=dys[direction];
        const float low=cells_[index(x+dx,y+dy)].candidate;
        const float beyond_low=cells_[index(x+2*dx,y+2*dy)].candidate;
        const float beyond_high=cells_[index(x-dx,y-dy)].candidate;
        if (!std::isfinite(low)||!std::isfinite(beyond_low)||!std::isfinite(beyond_high)) continue;
        const double jump=high-low, before=low-beyond_low, after=beyond_high-high;
        if (jump<0.05-1e-6 || jump>max_obstacle_relative_height_m_ ||
            std::abs(before)>slope_rise || std::abs(after)>slope_rise ||
            jump-.5*(before+after)<0.045) continue;
        // At least one side must have a supported floor reference. A floating
        // roof's jagged edge alone is not a near-ground navigation obstacle.
        if (!std::isfinite(cells_[index(x,y)].surface_z) &&
            !std::isfinite(cells_[index(x+dx,y+dy)].surface_z)) continue;
        edges[index(x,y)]=1;
      }
    }
    measured_step_.assign(cells_.size(),0);
    for (int y=1;y<height_-1;++y) for (int x=1;x<width_-1;++x) if (edges[index(x,y)]) {
      int count=0;
      for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) count+=edges[index(x+dx,y+dy)]!=0;
      measured_step_[index(x,y)]=count>=3;
    }
  }

  void classifyObstacles() {
    unresolved_vertical_.assign(cells_.size(),0);
    std::vector<float> references(cells_.size(), std::numeric_limits<float>::quiet_NaN());
    const auto offsets=diskOffsets(static_cast<int>(std::floor(
        parameters_.wall_search_radius_m/cell_size_+1e-9)));
    for (int y=0;y<height_;++y) for (int x=0;x<width_;++x) {
      const auto& cell=cells_[index(x,y)];
      if (std::isfinite(cell.surface_z)) { references[index(x,y)]=cell.surface_z; continue; }
      if (!hasVerticalSupport(cell.samples)) continue;
      ++unresolved_vertical_cells_;
      unresolved_vertical_[index(x,y)]=1;
      double nearest=std::numeric_limits<double>::infinity();
      for (const auto& d:offsets) if (inside(x+d.first,y+d.second) &&
          std::isfinite(cells_[index(x+d.first,y+d.second)].surface_z))
        nearest=std::min(nearest, std::hypot(d.first,d.second)*cell_size_);
      if (nearest>parameters_.wall_max_nearest_m) continue;
      Plane plane;
      if (!fitPlane(x,y,offsets,false,parameters_.wall_min_ground_cells,false,&plane,true)) continue;
      std::vector<float> band;
      for (const float z:cell.samples) if (z-plane.c>=min_obstacle_relative_height_m_ &&
                                           z-plane.c<=max_obstacle_relative_height_m_)
        band.push_back(z);
      if (!hasVerticalSupport(band)) continue;
      references[index(x,y)]=plane.c;
      ++recovered_wall_cells_; --unresolved_vertical_cells_;
      unresolved_vertical_[index(x,y)]=0;
    }
    obstacles_.reserve(input_->size()/3);
    for (const auto& point: *input_) {
      int x,y; if (!pointCell(point,&x,&y)) continue;
      if (measured_step_[index(x,y)] &&
          std::abs(point.z-cells_[index(x,y)].candidate)<=0.12) {
        obstacles_.push_back(point); continue;
      }
      const float ground_z=references[index(x,y)];
      if (!std::isfinite(ground_z)) { ++points_without_surface_; continue; }
      const double relative=point.z-ground_z;
      if (relative>=min_obstacle_relative_height_m_ &&
          relative<=max_obstacle_relative_height_m_) obstacles_.push_back(point);
    }
  }

  SurfaceParameters parameters_;
  std::vector<std::uint8_t> measured_step_, unresolved_vertical_;
  GridGeometry geometry_;
  std::size_t recovered_wall_cells_=0, unresolved_vertical_cells_=0;
  double cell_size_ = 0.05;
  double candidate_percentile_ = 0.05;
  double seed_x_ = 0.0;
  double seed_y_ = 0.0;
  double seed_ground_z_ = -0.15;
  double seed_radius_m_ = 1.0;
  double seed_height_tolerance_m_ = 0.12;
  double trusted_seed_height_tolerance_m_ = 0.08;
  int pmf_max_window_size_ = 101;
  double pmf_slope_ = 0.40;
  double pmf_initial_distance_m_ = 0.04;
  double pmf_max_distance_m_ = 0.18;
  double pmf_base_ = 2.0;
  bool pmf_exponential_ = true;
  double validation_radius_m_ = 0.30;
  int validation_min_candidates_ = 8;
  double plane_inlier_tolerance_m_ = 0.05;
  double plane_min_inlier_ratio_ = 0.55;
  double plane_min_spread_m_ = 0.04;
  double candidate_plane_tolerance_m_ = 0.05;
  double plane_max_rmse_m_ = 0.035;
  double max_ground_slope_deg_ = 35.0;
  double connect_radius_m_ = 0.25;
  double connection_plane_tolerance_m_ = 0.05;
  double connection_max_normal_delta_deg_ = 20.0;
  double max_ground_step_m_ = 0.015;
  double surface_fit_radius_m_ = 0.80;
  double surface_observation_radius_m_ = 0.15;
  double surface_gap_fill_radius_m_ = 0.30;
  int surface_min_candidates_ = 4;
  int surface_min_component_cells_ = 100;
  double surface_min_component_fraction_ = 0.03;
  double min_obstacle_relative_height_m_ = 0.04;
  double max_obstacle_relative_height_m_ = 1.50;
  int min_connected_ground_cells_ = 100;

  Cloud::Ptr input_;
  Cloud trusted_seeds_;
  Cloud candidates_;
  Cloud ground_;
  Cloud obstacles_;
  std::vector<Cell> cells_;
  std::vector<std::pair<int, int>> validation_offsets_;
  std::vector<std::pair<int, int>> connect_offsets_;
  std::vector<std::pair<int, int>> surface_offsets_;
  std::vector<std::pair<int, int>> observation_offsets_;
  std::vector<std::pair<int, int>> gap_fill_offsets_;
  int width_ = 0;
  int height_ = 0;
  double min_x_ = 0.0;
  double min_y_ = 0.0;
  std::size_t candidate_cells_ = 0;
  std::size_t valid_candidate_cells_ = 0;
  std::size_t origin_seed_cells_ = 0;
  std::size_t distributed_seed_cells_ = 0;
  std::size_t trusted_seed_cells_ = 0;
  std::size_t connected_trusted_seed_cells_ = 0;
  std::size_t connected_cells_ = 0;
  std::size_t gap_filled_surface_cells_ = 0;
  std::size_t surface_component_count_ = 0;
  std::size_t kept_surface_component_count_ = 0;
  std::size_t removed_surface_cells_ = 0;
  std::size_t points_without_surface_ = 0;
};

bool hasVerticalSupport(const std::vector<float>& heights) {
  std::vector<int> bins; float low=INFINITY, high=-INFINITY;
  for (const float z:heights) if (std::isfinite(z)) {
    bins.push_back(static_cast<int>(std::floor(z/0.10)));
    low=std::min(low,z); high=std::max(high,z);
  }
  if (bins.size()<4 || high-low<0.30-1e-6) return false;
  std::sort(bins.begin(),bins.end());
  bins.erase(std::unique(bins.begin(),bins.end()),bins.end());
  int run=1;
  for (std::size_t i=1;i<bins.size();++i) {
    run=bins[i]==bins[i-1]+1 ? run+1 : 1;
    if (run>=4) return true;
  }
  return false;
}
SurfaceResult reconstructSurface(const pcl::PointCloud<pcl::PointXYZ>& input,
                                 const GridGeometry& geometry,
                                 const SurfaceParameters& parameters) {
  return TerrainReclassifier(input,geometry,parameters).result();
}
}  // namespace go2_terrain
