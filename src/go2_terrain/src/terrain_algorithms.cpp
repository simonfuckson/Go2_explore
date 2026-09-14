#include "go2_terrain/terrain_algorithms.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

#include <costmap_2d/cost_values.h>

namespace go2_terrain
{
namespace
{

std::size_t indexOf(std::size_t x,
                    std::size_t y,
                    const GridGeometry& geometry)
{
  return y * geometry.width + x;
}

}  // namespace

GroundColumnSummary summarizeGroundColumn(const std::vector<float>& samples,
                                          double candidate_quantile,
                                          double support_band_m)
{
  if (samples.empty() || candidate_quantile < 0.0 || candidate_quantile > 1.0 ||
      support_band_m <= 0.0)
  {
    return GroundColumnSummary();
  }
  std::vector<float> finite;
  finite.reserve(samples.size());
  for (const float value : samples)
  {
    if (std::isfinite(value))
    {
      finite.push_back(value);
    }
  }
  if (finite.empty())
  {
    return GroundColumnSummary();
  }
  std::sort(finite.begin(), finite.end());
  const double location = candidate_quantile *
                          static_cast<double>(finite.size() - 1);
  const std::size_t low = static_cast<std::size_t>(std::floor(location));
  const std::size_t high = static_cast<std::size_t>(std::ceil(location));
  const double alpha = location - static_cast<double>(low);

  GroundColumnSummary result;
  result.candidate = static_cast<float>((1.0 - alpha) * finite[low] +
                                        alpha * finite[high]);
  const auto support_end = std::upper_bound(
      finite.begin(), finite.end(), result.candidate + support_band_m);
  result.support_count =
      static_cast<std::size_t>(std::distance(finite.begin(), support_end));
  if (result.support_count > 0)
  {
    result.support_span = *(support_end - 1) - finite.front();
  }
  return result;
}

bool isObstacleHeight(double relative_height,
                      double minimum_height,
                      double maximum_height)
{
  return std::isfinite(relative_height) &&
         relative_height >= minimum_height &&
         relative_height <= maximum_height;
}

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
    double maximum_gap_m)
{
  if (!geometry.valid() ||
      candidate_elevation.size() != geometry.cellCount() ||
      candidate_support_span.size() != geometry.cellCount() ||
      admissible_reanchor.size() != geometry.cellCount() ||
      search_radius_m <= 0.0 || maximum_support_span_m <= 0.0 ||
      initial_tolerance_m <= 0.0 || maximum_slope_deg <= 0.0 ||
      maximum_slope_deg > 89.0 || continuity_margin_m < 0.0 ||
      maximum_reanchor_height_from_initial_m <= 0.0 ||
      maximum_gap_m <= 0.0)
  {
    throw std::invalid_argument("Invalid trajectory-ground tracing input");
  }

  const float unknown = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> result(trajectory_xy.size(), unknown);
  const int radius_cells =
      static_cast<int>(std::ceil(search_radius_m / geometry.resolution));
  const double tangent =
      std::tan(maximum_slope_deg * 3.14159265358979323846 / 180.0);
  std::size_t previous = trajectory_xy.size();
  double path_gap = 0.0;

  for (std::size_t i = 0; i < trajectory_xy.size(); ++i)
  {
    if (i > 0)
    {
      path_gap += std::hypot(trajectory_xy[i].x - trajectory_xy[i - 1].x,
                             trajectory_xy[i].y - trajectory_xy[i - 1].y);
    }
    const int center_x = static_cast<int>(std::floor(
        (trajectory_xy[i].x - geometry.origin_x) / geometry.resolution));
    const int center_y = static_cast<int>(std::floor(
        (trajectory_xy[i].y - geometry.origin_y) / geometry.resolution));
    if (center_x < 0 || center_y < 0 ||
        center_x >= static_cast<int>(geometry.width) ||
        center_y >= static_cast<int>(geometry.height))
    {
      continue;
    }

    const bool has_previous = previous != trajectory_xy.size();
    const bool requires_reanchor = has_previous && path_gap > maximum_gap_m;
    const double distance_from_previous = has_previous
        ? std::hypot(trajectory_xy[i].x - trajectory_xy[previous].x,
                     trajectory_xy[i].y - trajectory_xy[previous].y)
        : 0.0;
    const double expected = has_previous ? result[previous] : initial_ground_z;
    // Normal tracking remains locally slope-bounded. After a long data gap,
    // reacquisition is instead referenced to the initial floor and a broad,
    // explicit terrain-height envelope. This permits accumulated rise/fall on
    // a long ramp without allowing the tolerance to grow without bound.
    const double comparison_height = requires_reanchor
        ? initial_ground_z
        : expected;
    const double allowance = !has_previous
        ? initial_tolerance_m
        : requires_reanchor
              ? maximum_reanchor_height_from_initial_m
              : tangent * distance_from_previous + continuity_margin_m;
    std::vector<float> plausible;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy)
    {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx)
      {
        if (std::hypot(dx, dy) * geometry.resolution > search_radius_m)
        {
          continue;
        }
        const int x = center_x + dx;
        const int y = center_y + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(geometry.width) ||
            y >= static_cast<int>(geometry.height))
        {
          continue;
        }
        const std::size_t cell = indexOf(
            static_cast<std::size_t>(x), static_cast<std::size_t>(y), geometry);
        if (!isKnown(candidate_elevation[cell]) ||
            !isKnown(candidate_support_span[cell]) ||
            candidate_support_span[cell] > maximum_support_span_m ||
            std::fabs(candidate_elevation[cell] - comparison_height) >
                allowance ||
            (requires_reanchor && admissible_reanchor[cell] == 0U))
        {
          continue;
        }
        plausible.push_back(candidate_elevation[cell]);
      }
    }
    if (plausible.empty())
    {
      continue;
    }
    const std::size_t middle = plausible.size() / 2;
    std::nth_element(plausible.begin(), plausible.begin() + middle, plausible.end());
    result[i] = plausible[middle];
    previous = i;
    path_gap = 0.0;
  }
  return result;
}

std::vector<std::uint8_t> buildTrajectorySeedMask(
    const std::vector<PlanarPoint>& trajectory_xy,
    const std::vector<float>& trajectory_ground_z,
    const std::vector<float>& candidate_elevation,
    const std::vector<float>& candidate_support_span,
    const GridGeometry& geometry,
    double radius_m,
    double maximum_support_span_m,
    double height_tolerance_m)
{
  if (!geometry.valid() ||
      trajectory_xy.size() != trajectory_ground_z.size() ||
      candidate_elevation.size() != geometry.cellCount() ||
      candidate_support_span.size() != geometry.cellCount() ||
      radius_m <= 0.0 || maximum_support_span_m <= 0.0 ||
      height_tolerance_m < 0.0)
  {
    throw std::invalid_argument("Invalid trajectory seed input");
  }

  std::vector<std::uint8_t> result(geometry.cellCount(), 0U);
  const int radius_cells =
      static_cast<int>(std::ceil(radius_m / geometry.resolution));
  constexpr double kDistanceTolerance = 1e-9;
  for (std::size_t path_index = 0; path_index < trajectory_xy.size();
       ++path_index)
  {
    const PlanarPoint& path = trajectory_xy[path_index];
    const float expected_ground = trajectory_ground_z[path_index];
    if (!std::isfinite(path.x) || !std::isfinite(path.y) ||
        !isKnown(expected_ground))
    {
      continue;
    }
    const int center_x = static_cast<int>(std::floor(
        (path.x - geometry.origin_x) / geometry.resolution));
    const int center_y = static_cast<int>(std::floor(
        (path.y - geometry.origin_y) / geometry.resolution));
    if (center_x < 0 || center_y < 0 ||
        center_x >= static_cast<int>(geometry.width) ||
        center_y >= static_cast<int>(geometry.height))
    {
      continue;
    }
    for (int dy = -radius_cells; dy <= radius_cells; ++dy)
    {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx)
      {
        const int x = center_x + dx;
        const int y = center_y + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(geometry.width) ||
            y >= static_cast<int>(geometry.height))
        {
          continue;
        }
        const double cell_center_x =
            geometry.origin_x + (static_cast<double>(x) + 0.5) *
                                    geometry.resolution;
        const double cell_center_y =
            geometry.origin_y + (static_cast<double>(y) + 0.5) *
                                    geometry.resolution;
        if (std::hypot(cell_center_x - path.x, cell_center_y - path.y) >
            radius_m + kDistanceTolerance)
        {
          continue;
        }
        const std::size_t index = indexOf(
            static_cast<std::size_t>(x), static_cast<std::size_t>(y),
            geometry);
        if (!isKnown(candidate_elevation[index]) ||
            !isKnown(candidate_support_span[index]) ||
            candidate_support_span[index] > maximum_support_span_m ||
            std::fabs(candidate_elevation[index] - expected_ground) >
                height_tolerance_m)
        {
          continue;
        }
        result[index] = 1U;
      }
    }
  }
  return result;
}

std::vector<std::uint8_t> growConnectedGround(
    const std::vector<float>& candidate_elevation,
    const std::vector<float>& candidate_support_span,
    const std::vector<std::uint8_t>& initial_seed,
    const GridGeometry& geometry,
    double maximum_support_span_m,
    double maximum_slope_deg,
    double height_margin_m,
    int fill_iterations)
{
  if (!geometry.valid() ||
      candidate_elevation.size() != geometry.cellCount() ||
      candidate_support_span.size() != geometry.cellCount() ||
      initial_seed.size() != geometry.cellCount() ||
      maximum_support_span_m <= 0.0 || maximum_slope_deg <= 0.0 ||
      maximum_slope_deg > 89.0 || height_margin_m < 0.0 || fill_iterations < 0)
  {
    throw std::invalid_argument("Invalid connected-ground growth input");
  }

  std::vector<std::uint8_t> accepted(geometry.cellCount(), 0);
  std::queue<std::size_t> pending;
  for (std::size_t i = 0; i < initial_seed.size(); ++i)
  {
    if (initial_seed[i] && isKnown(candidate_elevation[i]) &&
        isKnown(candidate_support_span[i]) &&
        candidate_support_span[i] <= maximum_support_span_m)
    {
      accepted[i] = 1;
      pending.push(i);
    }
  }

  const double tangent =
      std::tan(maximum_slope_deg * 3.14159265358979323846 / 180.0);
  auto can_grow = [&](std::size_t from, int next_x, int next_y) {
    const std::size_t next = indexOf(
        static_cast<std::size_t>(next_x),
        static_cast<std::size_t>(next_y),
        geometry);
    if (accepted[next] || !isKnown(candidate_elevation[next]) ||
        !isKnown(candidate_support_span[next]) ||
        candidate_support_span[next] > maximum_support_span_m)
    {
      return false;
    }
    const int from_x = static_cast<int>(from % geometry.width);
    const int from_y = static_cast<int>(from / geometry.width);
    const double distance = geometry.resolution *
                            std::hypot(next_x - from_x, next_y - from_y);
    return std::fabs(candidate_elevation[next] - candidate_elevation[from]) <=
           tangent * distance + height_margin_m;
  };

  while (!pending.empty())
  {
    const std::size_t current = pending.front();
    pending.pop();
    const int x = static_cast<int>(current % geometry.width);
    const int y = static_cast<int>(current / geometry.width);
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dx = -1; dx <= 1; ++dx)
      {
        // Four-connectivity prevents a discrete step edge from being crossed
        // diagonally even when its diagonal rise/run is below 35 degrees.
        if (std::abs(dx) + std::abs(dy) != 1)
        {
          continue;
        }
        const int next_x = x + dx;
        const int next_y = y + dy;
        if (next_x < 0 || next_y < 0 ||
            next_x >= static_cast<int>(geometry.width) ||
            next_y >= static_cast<int>(geometry.height) ||
            !can_grow(current, next_x, next_y))
        {
          continue;
        }
        const std::size_t next = indexOf(
            static_cast<std::size_t>(next_x),
            static_cast<std::size_t>(next_y),
            geometry);
        accepted[next] = 1;
        pending.push(next);
      }
    }
  }

  for (int iteration = 0; iteration < fill_iterations; ++iteration)
  {
    std::vector<std::size_t> additions;
    for (int y = 0; y < static_cast<int>(geometry.height); ++y)
    {
      for (int x = 0; x < static_cast<int>(geometry.width); ++x)
      {
        const std::size_t cell = indexOf(
            static_cast<std::size_t>(x), static_cast<std::size_t>(y), geometry);
        if (accepted[cell] || !isKnown(candidate_elevation[cell]) ||
            !isKnown(candidate_support_span[cell]) ||
            candidate_support_span[cell] > maximum_support_span_m)
        {
          continue;
        }
        std::vector<float> neighbor_heights;
        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dx = -1; dx <= 1; ++dx)
          {
            if (dx == 0 && dy == 0)
            {
              continue;
            }
            const int next_x = x + dx;
            const int next_y = y + dy;
            if (next_x < 0 || next_y < 0 ||
                next_x >= static_cast<int>(geometry.width) ||
                next_y >= static_cast<int>(geometry.height))
            {
              continue;
            }
            const std::size_t neighbor = indexOf(
                static_cast<std::size_t>(next_x),
                static_cast<std::size_t>(next_y),
                geometry);
            if (accepted[neighbor])
            {
              neighbor_heights.push_back(candidate_elevation[neighbor]);
            }
          }
        }
        if (neighbor_heights.size() < 3)
        {
          continue;
        }
        const std::size_t middle = neighbor_heights.size() / 2;
        std::nth_element(neighbor_heights.begin(),
                         neighbor_heights.begin() + middle,
                         neighbor_heights.end());
        const double neighbor_median = neighbor_heights[middle];
        // Hole filling must obey the same single-cell rise as primary growth;
        // otherwise three diagonal neighbors can absorb a low isolated step.
        if (std::fabs(candidate_elevation[cell] - neighbor_median) <=
            tangent * geometry.resolution + height_margin_m)
        {
          additions.push_back(cell);
        }
      }
    }
    if (additions.empty())
    {
      break;
    }
    for (const std::size_t cell : additions)
    {
      accepted[cell] = 1;
    }
  }
  return accepted;
}

void applyTrajectoryFreeEvidence(
    const std::vector<std::uint8_t>& recorded_trajectory_cells,
    std::vector<std::uint8_t>* free_cells)
{
  if (!free_cells || recorded_trajectory_cells.size() != free_cells->size())
  {
    throw std::invalid_argument("Trajectory free-evidence grids must match");
  }
  for (std::size_t i = 0; i < recorded_trajectory_cells.size(); ++i)
  {
    if (recorded_trajectory_cells[i])
    {
      (*free_cells)[i] = 1;
    }
  }
}

BinaryMaskMetrics measureBinaryMaskQuality(
    const std::vector<std::uint8_t>& mask,
    const GridGeometry& geometry)
{
  if (!geometry.valid() || mask.size() != geometry.cellCount())
  {
    throw std::invalid_argument("Invalid binary mask quality input");
  }

  BinaryMaskMetrics metrics;
  for (const std::uint8_t value : mask)
  {
    metrics.active_cells += value != 0U ? 1U : 0U;
  }
  metrics.coverage_ratio = static_cast<double>(metrics.active_cells) /
                           static_cast<double>(mask.size());
  if (metrics.active_cells == 0U)
  {
    return metrics;
  }

  std::vector<std::uint8_t> visited(mask.size(), 0U);
  std::queue<std::size_t> pending;
  constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  for (std::size_t start = 0; start < mask.size(); ++start)
  {
    if (mask[start] == 0U || visited[start] != 0U)
    {
      continue;
    }
    std::size_t component = 0U;
    visited[start] = 1U;
    pending.push(start);
    while (!pending.empty())
    {
      const std::size_t current = pending.front();
      pending.pop();
      ++component;
      const int x = static_cast<int>(current % geometry.width);
      const int y = static_cast<int>(current / geometry.width);
      for (int direction = 0; direction < 8; ++direction)
      {
        const int next_x = x + kDx[direction];
        const int next_y = y + kDy[direction];
        if (next_x < 0 || next_y < 0 ||
            next_x >= static_cast<int>(geometry.width) ||
            next_y >= static_cast<int>(geometry.height))
        {
          continue;
        }
        const std::size_t next = indexOf(
            static_cast<std::size_t>(next_x),
            static_cast<std::size_t>(next_y), geometry);
        if (mask[next] != 0U && visited[next] == 0U)
        {
          visited[next] = 1U;
          pending.push(next);
        }
      }
    }
    metrics.largest_component_cells =
        std::max(metrics.largest_component_cells, component);
  }
  metrics.largest_component_ratio =
      static_cast<double>(metrics.largest_component_cells) /
      static_cast<double>(metrics.active_cells);
  return metrics;
}

std::vector<std::uint8_t> mergeOccupancyEvidence(
    const std::vector<std::uint8_t>& baseline,
    const std::vector<std::uint8_t>& verified_ground,
    const std::vector<std::uint8_t>& verified_trajectory_free,
    const std::vector<std::uint8_t>& obstacles,
    std::uint8_t unknown_value,
    std::uint8_t free_value,
    std::uint8_t occupied_value)
{
  if (verified_ground.size() != obstacles.size() ||
      verified_trajectory_free.size() != obstacles.size() ||
      (!baseline.empty() && baseline.size() != verified_ground.size()) ||
      unknown_value == free_value || unknown_value == occupied_value ||
      free_value == occupied_value)
  {
    throw std::invalid_argument("Invalid occupancy evidence input");
  }
  std::vector<std::uint8_t> result = baseline.empty()
      ? std::vector<std::uint8_t>(verified_ground.size(), unknown_value)
      : baseline;
  for (std::size_t i = 0; i < result.size(); ++i)
  {
    if (verified_ground[i] != 0U && obstacles[i] == 0U)
    {
      result[i] = free_value;
    }
    else if (verified_trajectory_free[i] != 0U &&
             result[i] == unknown_value && obstacles[i] == 0U)
    {
      result[i] = free_value;
    }
    if (obstacles[i] != 0U)
    {
      result[i] = occupied_value;
    }
  }
  return result;
}

float slopeToSoftCost(double slope_deg, const CostParameters& parameters)
{
  if (!std::isfinite(slope_deg) || slope_deg <= parameters.flat_slope_deg)
  {
    return 0.0F;
  }
  if (parameters.lethal_slope_deg <= parameters.flat_slope_deg)
  {
    throw std::invalid_argument("lethal_slope_deg must exceed flat_slope_deg");
  }

  const double clamped = std::min(slope_deg, parameters.lethal_slope_deg);
  const double ratio = (clamped - parameters.flat_slope_deg) /
                       (parameters.lethal_slope_deg - parameters.flat_slope_deg);
  return static_cast<float>(parameters.minimum_cost +
                            ratio * (parameters.maximum_soft_cost -
                                     parameters.minimum_cost));
}

std::vector<std::uint8_t> buildSlopeCostLayer(
    const std::vector<float>& slope_deg,
    const GridGeometry& geometry,
    const CostParameters& parameters)
{
  if (!geometry.valid() || slope_deg.size() != geometry.cellCount())
  {
    throw std::invalid_argument("Slope grid does not match terrain geometry");
  }
  if (parameters.minimum_lethal_cluster_cells == 0)
  {
    throw std::invalid_argument("minimum_lethal_cluster_cells must be positive");
  }

  std::vector<std::uint8_t> result(geometry.cellCount(), 255);
  std::vector<std::uint8_t> steep(geometry.cellCount(), 0);
  std::vector<std::uint8_t> visited(geometry.cellCount(), 0);

  for (std::size_t i = 0; i < slope_deg.size(); ++i)
  {
    if (!isKnown(slope_deg[i]))
    {
      continue;
    }
    result[i] = static_cast<std::uint8_t>(
        std::lround(slopeToSoftCost(slope_deg[i], parameters)));
    if (slope_deg[i] > parameters.lethal_slope_deg)
    {
      steep[i] = 1;
    }
  }

  for (std::size_t start = 0; start < steep.size(); ++start)
  {
    if (!steep[start] || visited[start])
    {
      continue;
    }

    std::vector<std::size_t> component;
    std::queue<std::size_t> pending;
    pending.push(start);
    visited[start] = 1;

    while (!pending.empty())
    {
      const std::size_t current = pending.front();
      pending.pop();
      component.push_back(current);
      const int x = static_cast<int>(current % geometry.width);
      const int y = static_cast<int>(current / geometry.width);

      for (int dy = -1; dy <= 1; ++dy)
      {
        for (int dx = -1; dx <= 1; ++dx)
        {
          if (dx == 0 && dy == 0)
          {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 ||
              nx >= static_cast<int>(geometry.width) ||
              ny >= static_cast<int>(geometry.height))
          {
            continue;
          }
          const std::size_t neighbor = indexOf(
              static_cast<std::size_t>(nx), static_cast<std::size_t>(ny), geometry);
          if (steep[neighbor] && !visited[neighbor])
          {
            visited[neighbor] = 1;
            pending.push(neighbor);
          }
        }
      }
    }

    if (component.size() >= parameters.minimum_lethal_cluster_cells)
    {
      for (const std::size_t cell : component)
      {
        result[cell] = 254;
      }
    }
  }

  dilateKnownCosts(&result, geometry, parameters.dilation_m);
  return result;
}

void dilateBinaryMaskMetric(const std::vector<std::uint8_t>& source,
                            std::vector<std::uint8_t>* destination,
                            const GridGeometry& geometry,
                            double radius_m)
{
  if (!destination || source.size() != geometry.cellCount() ||
      !geometry.valid() || !std::isfinite(radius_m))
  {
    throw std::invalid_argument("Binary mask does not match terrain geometry");
  }
  *destination = source;
  if (radius_m <= 0.0)
  {
    return;
  }

  // Preserve the proven legacy occupancy-exporter rasterization. At the
  // locked 0.05 m resolution, round(0.03 / 0.05) is one cardinal cell.
  const int radius_cells =
      static_cast<int>(std::round(radius_m / geometry.resolution));
  if (radius_cells <= 0)
  {
    return;
  }
  const int radius_squared = radius_cells * radius_cells;
  for (int y = 0; y < static_cast<int>(geometry.height); ++y)
  {
    for (int x = 0; x < static_cast<int>(geometry.width); ++x)
    {
      if (source[indexOf(static_cast<std::size_t>(x),
                         static_cast<std::size_t>(y), geometry)] == 0U)
      {
        continue;
      }
      for (int dy = -radius_cells; dy <= radius_cells; ++dy)
      {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx)
        {
          if (dx * dx + dy * dy > radius_squared)
          {
            continue;
          }
          const int neighbor_x = x + dx;
          const int neighbor_y = y + dy;
          if (neighbor_x < 0 || neighbor_y < 0 ||
              neighbor_x >= static_cast<int>(geometry.width) ||
              neighbor_y >= static_cast<int>(geometry.height))
          {
            continue;
          }
          (*destination)[indexOf(static_cast<std::size_t>(neighbor_x),
                                 static_cast<std::size_t>(neighbor_y),
                                 geometry)] = 1U;
        }
      }
    }
  }
}

void dilateKnownCosts(std::vector<std::uint8_t>* costs,
                      const GridGeometry& geometry,
                      double radius_m)
{
  if (!costs || costs->size() != geometry.cellCount() || !geometry.valid())
  {
    throw std::invalid_argument("Cost grid does not match terrain geometry");
  }
  if (radius_m <= 0.0)
  {
    return;
  }

  const int radius_cells =
      static_cast<int>(std::ceil(radius_m / geometry.resolution));
  const double radius_squared =
      (radius_m / geometry.resolution) * (radius_m / geometry.resolution) + 1e-9;
  const std::vector<std::uint8_t> source = *costs;

  for (int y = 0; y < static_cast<int>(geometry.height); ++y)
  {
    for (int x = 0; x < static_cast<int>(geometry.width); ++x)
    {
      const std::size_t destination = indexOf(
          static_cast<std::size_t>(x), static_cast<std::size_t>(y), geometry);
      if (source[destination] == 255)
      {
        continue;
      }

      std::uint8_t maximum = source[destination];
      for (int dy = -radius_cells; dy <= radius_cells; ++dy)
      {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx)
        {
          if (static_cast<double>(dx * dx + dy * dy) > radius_squared)
          {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 ||
              nx >= static_cast<int>(geometry.width) ||
              ny >= static_cast<int>(geometry.height))
          {
            continue;
          }
          const std::uint8_t value = source[indexOf(
              static_cast<std::size_t>(nx), static_cast<std::size_t>(ny), geometry)];
          if (value != 255)
          {
            maximum = std::max(maximum, value);
          }
        }
      }
      (*costs)[destination] = maximum;
    }
  }
}

std::uint8_t mergeTerrainCost(std::uint8_t master_cost,
                              std::uint8_t terrain_cost)
{
  if (master_cost == costmap_2d::NO_INFORMATION ||
      master_cost >= costmap_2d::LETHAL_OBSTACLE ||
      terrain_cost == costmap_2d::NO_INFORMATION ||
      terrain_cost == costmap_2d::FREE_SPACE)
  {
    return master_cost;
  }
  return std::max(master_cost, terrain_cost);
}

}  // namespace go2_terrain
