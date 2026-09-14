#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace go2_mapping {

struct VoxelKey {
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;

  bool operator==(const VoxelKey& rhs) const {
    return x == rhs.x && y == rhs.y && z == rhs.z;
  }

  bool operator!=(const VoxelKey& rhs) const { return !(*this == rhs); }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    std::size_t seed = std::hash<int64_t>()(key.x);
    seed ^= std::hash<int64_t>()(key.y) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    seed ^= std::hash<int64_t>()(key.z) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    return seed;
  }
};

inline VoxelKey voxelKey(const Eigen::Vector3d& point, double voxel_size) {
  return {static_cast<int64_t>(std::floor(point.x() / voxel_size)),
          static_cast<int64_t>(std::floor(point.y() / voxel_size)),
          static_cast<int64_t>(std::floor(point.z() / voxel_size))};
}

// Returns a stable bit index for a fine voxel inside its coarse parent. The
// caller can therefore maintain one compact child mask per coarse voxel and
// avoid a second high-resolution DDA for every lidar ray.
inline bool fineChildLinearIndex(const VoxelKey& coarse,
                                 const VoxelKey& fine,
                                 int cells_per_axis, uint32_t* index) {
  if (index == nullptr || cells_per_axis < 1 || cells_per_axis > 4) {
    return false;
  }
  const int64_t local_x =
      fine.x - coarse.x * static_cast<int64_t>(cells_per_axis);
  const int64_t local_y =
      fine.y - coarse.y * static_cast<int64_t>(cells_per_axis);
  const int64_t local_z =
      fine.z - coarse.z * static_cast<int64_t>(cells_per_axis);
  if (local_x < 0 || local_y < 0 || local_z < 0 ||
      local_x >= cells_per_axis || local_y >= cells_per_axis ||
      local_z >= cells_per_axis) {
    return false;
  }
  *index = static_cast<uint32_t>(
      local_x + cells_per_axis * (local_y + cells_per_axis * local_z));
  return true;
}

inline bool fineChildKey(const VoxelKey& coarse, int cells_per_axis,
                         uint32_t index, VoxelKey* fine) {
  if (fine == nullptr || cells_per_axis < 1 || cells_per_axis > 4) {
    return false;
  }
  const uint32_t child_count = static_cast<uint32_t>(
      cells_per_axis * cells_per_axis * cells_per_axis);
  if (index >= child_count) {
    return false;
  }
  const int64_t local_x = index % cells_per_axis;
  index /= static_cast<uint32_t>(cells_per_axis);
  const int64_t local_y = index % cells_per_axis;
  const int64_t local_z = index / static_cast<uint32_t>(cells_per_axis);
  fine->x = coarse.x * static_cast<int64_t>(cells_per_axis) + local_x;
  fine->y = coarse.y * static_cast<int64_t>(cells_per_axis) + local_y;
  fine->z = coarse.z * static_cast<int64_t>(cells_per_axis) + local_z;
  return true;
}

// Tests whether the segment crosses the voxel interior. Shrinking the AABB by
// a tiny numerical tolerance prevents a ray that only touches a face, edge, or
// corner from supplying false free-space evidence to the adjacent voxel.
inline bool segmentIntersectsVoxelInterior(const Eigen::Vector3d& start,
                                           const Eigen::Vector3d& end,
                                           const VoxelKey& voxel,
                                           double voxel_size) {
  if (!(voxel_size > 0.0) || !start.allFinite() || !end.allFinite()) {
    return false;
  }
  const Eigen::Vector3d delta = end - start;
  const double epsilon = 1e-9 * std::max(1.0, voxel_size);
  double minimum_t = 0.0;
  double maximum_t = 1.0;
  const int64_t indices[3] = {voxel.x, voxel.y, voxel.z};
  for (int axis = 0; axis < 3; ++axis) {
    const double minimum =
        static_cast<double>(indices[axis]) * voxel_size + epsilon;
    const double maximum = minimum + voxel_size - 2.0 * epsilon;
    if (minimum >= maximum) {
      return false;
    }
    if (std::fabs(delta[axis]) < 1e-12) {
      if (start[axis] <= minimum || start[axis] >= maximum) {
        return false;
      }
      continue;
    }
    double entry = (minimum - start[axis]) / delta[axis];
    double exit = (maximum - start[axis]) / delta[axis];
    if (entry > exit) {
      std::swap(entry, exit);
    }
    minimum_t = std::max(minimum_t, entry);
    maximum_t = std::min(maximum_t, exit);
    if (minimum_t > maximum_t) {
      return false;
    }
  }
  return maximum_t >= 0.0 && minimum_t <= 1.0;
}

inline bool fineCandidateCapacityAvailable(std::size_t candidate_count,
                                           std::size_t maximum_candidates) {
  return candidate_count < maximum_candidates;
}

// Amanatides-Woo traversal. The sensor-origin voxel is intentionally omitted.
// Advancing tied axes together avoids clearing cells touched at only an edge or
// corner. The visitor form avoids one allocation per lidar ray.
template <typename Visitor>
inline uint64_t visitRayDda(const Eigen::Vector3d& start,
                            const Eigen::Vector3d& end, double voxel_size,
                            Visitor visitor) {
  if (!(voxel_size > 0.0) || !start.allFinite() || !end.allFinite()) {
    return 0;
  }

  VoxelKey current = voxelKey(start, voxel_size);
  const VoxelKey finish = voxelKey(end, voxel_size);
  if (current == finish) {
    return 0;
  }

  const Eigen::Vector3d delta = end - start;
  const double infinity = std::numeric_limits<double>::infinity();
  int step[3] = {0, 0, 0};
  double t_max[3] = {infinity, infinity, infinity};
  double t_delta[3] = {infinity, infinity, infinity};
  int64_t current_index[3] = {current.x, current.y, current.z};

  for (int axis = 0; axis < 3; ++axis) {
    const double component = delta[axis];
    if (std::fabs(component) < 1e-12) {
      continue;
    }
    step[axis] = component > 0.0 ? 1 : -1;
    const double boundary =
        (component > 0.0 ? static_cast<double>(current_index[axis] + 1)
                         : static_cast<double>(current_index[axis])) *
        voxel_size;
    t_max[axis] = (boundary - start[axis]) / component;
    t_delta[axis] = voxel_size / std::fabs(component);
  }

  const uint64_t maximum_steps =
      static_cast<uint64_t>(std::llabs(finish.x - current.x) +
                            std::llabs(finish.y - current.y) +
                            std::llabs(finish.z - current.z) + 3);
  uint64_t visited = 0;
  for (uint64_t iteration = 0; iteration < maximum_steps && current != finish;
       ++iteration) {
    const double next_t = std::min(t_max[0], std::min(t_max[1], t_max[2]));
    if (!std::isfinite(next_t)) {
      break;
    }
    const double tie_epsilon = 1e-12 * std::max(1.0, std::fabs(next_t));
    for (int axis = 0; axis < 3; ++axis) {
      if (t_max[axis] <= next_t + tie_epsilon) {
        current_index[axis] += step[axis];
        t_max[axis] += t_delta[axis];
      }
    }
    current = {current_index[0], current_index[1], current_index[2]};
    visitor(current);
    ++visited;
  }
  return visited;
}

inline std::vector<VoxelKey> traceRayDda(const Eigen::Vector3d& start,
                                         const Eigen::Vector3d& end,
                                         double voxel_size) {
  std::vector<VoxelKey> keys;
  visitRayDda(start, end, voxel_size,
              [&keys](const VoxelKey& key) { keys.push_back(key); });
  return keys;
}

inline Eigen::Vector3d sensorOrigin(
    const Eigen::Vector3d& base_position,
    const Eigen::Quaterniond& base_orientation,
    const Eigen::Vector3d& base_to_sensor_translation) {
  return base_position + base_orientation * base_to_sensor_translation;
}

inline bool interpolatePose(
    double before_stamp, const Eigen::Vector3d& before_position,
    const Eigen::Quaterniond& before_orientation, double after_stamp,
    const Eigen::Vector3d& after_position,
    const Eigen::Quaterniond& after_orientation, double requested_stamp,
    double max_sample_age, Eigen::Vector3d* position,
    Eigen::Quaterniond* orientation) {
  const double interval = after_stamp - before_stamp;
  const double before_age = requested_stamp - before_stamp;
  const double after_age = after_stamp - requested_stamp;
  if (!(interval > 1e-9) || before_age < 0.0 || after_age < 0.0 ||
      before_age > max_sample_age || after_age > max_sample_age ||
      !before_position.allFinite() || !after_position.allFinite() ||
      !before_orientation.coeffs().allFinite() ||
      !after_orientation.coeffs().allFinite() ||
      before_orientation.squaredNorm() < 1e-12 ||
      after_orientation.squaredNorm() < 1e-12) {
    return false;
  }
  const double ratio = before_age / interval;
  *position = before_position + ratio * (after_position - before_position);
  *orientation = before_orientation.normalized().slerp(
      ratio, after_orientation.normalized());
  orientation->normalize();
  return true;
}

inline double clampProbability(double probability) {
  return std::max(0.001, std::min(0.999, probability));
}

inline double probabilityToLogOdds(double probability) {
  const double clamped = clampProbability(probability);
  return std::log(clamped / (1.0 - clamped));
}

struct DynamicFilterConfig {
  double hit_log_odds = probabilityToLogOdds(0.65);
  double miss_log_odds = probabilityToLogOdds(0.40);
  double occupied_log_odds = probabilityToLogOdds(0.75);
  double clearing_log_odds = probabilityToLogOdds(0.35);
  double min_log_odds = -4.0;
  double max_log_odds = 4.0;
  uint32_t min_hit_scans = 12;
  double min_observation_span = 2.0;
  double min_hit_ratio = 0.60;
  uint32_t min_candidate_clear_miss_scans = 4;
  double min_candidate_clear_miss_span = 0.30;
  uint32_t min_clear_miss_scans = 12;
  double min_clear_miss_span = 1.20;
};

struct OccupancyVoxelState {
  double log_odds = 0.0;
  uint32_t hit_scans = 0;
  uint32_t miss_scans = 0;
  uint32_t consecutive_clear_misses = 0;
  uint64_t last_hit_scan = std::numeric_limits<uint64_t>::max();
  uint64_t last_miss_scan = std::numeric_limits<uint64_t>::max();
  double first_hit_time = std::numeric_limits<double>::quiet_NaN();
  double last_hit_time = std::numeric_limits<double>::quiet_NaN();
  double first_clear_miss_time = std::numeric_limits<double>::quiet_NaN();
  double last_clear_miss_time = std::numeric_limits<double>::quiet_NaN();
  double last_seen_time = std::numeric_limits<double>::quiet_NaN();
  bool confirmed_static = false;
  uint64_t generation = 0;
};

inline bool generationMatches(uint64_t map_generation,
                              const OccupancyVoxelState* occupancy) {
  return occupancy != nullptr && occupancy->generation == map_generation;
}

// A confirmed coarse parent is only a spatial/generation guard. Every fine
// voxel must independently satisfy its configured temporal Bayesian evidence
// before it is eligible for the public map.
inline bool fineVoxelConfirmedForParent(
    uint64_t parent_generation, const OccupancyVoxelState* parent,
    const OccupancyVoxelState& fine_evidence) {
  return generationMatches(parent_generation, parent) &&
         parent->confirmed_static && fine_evidence.confirmed_static;
}

// Geometry may be cached as soon as the fine voxel confirms, but it is only
// publishable after its coarse parent confirms in the same generation.
inline bool fineVoxelReadyToCache(
    const OccupancyVoxelState& fine_evidence) {
  return fine_evidence.confirmed_static;
}

inline bool fineVoxelPublishable(
    uint64_t parent_generation, const OccupancyVoxelState* parent,
    const OccupancyVoxelState& fine_evidence, uint32_t sample_count) {
  return sample_count > 0 &&
         fineVoxelConfirmedForParent(parent_generation, parent, fine_evidence);
}

inline bool evidenceCandidateExpired(const OccupancyVoxelState& evidence,
                                     double now, double timeout) {
  return !evidence.confirmed_static &&
         std::isfinite(evidence.last_hit_time) &&
         now - evidence.last_hit_time > timeout;
}

inline bool unconfirmedEvidenceCleared(
    const OccupancyVoxelState& evidence,
    const DynamicFilterConfig& config) {
  return !evidence.confirmed_static &&
         evidence.log_odds <= config.clearing_log_odds &&
         evidence.consecutive_clear_misses >=
             config.min_candidate_clear_miss_scans &&
         std::isfinite(evidence.first_clear_miss_time) &&
         std::isfinite(evidence.last_clear_miss_time) &&
         evidence.last_clear_miss_time - evidence.first_clear_miss_time >=
             config.min_candidate_clear_miss_span;
}

// A capacity violation latches until reset. Callers check for an existing key
// before asking permission, so established voxels remain updateable and
// saveable while all memory-growing insertions fail closed.
class VoxelCapacityGate {
 public:
  bool allowNewVoxel(std::size_t current_size, std::size_t maximum_size) {
    if (latched_ || current_size >= maximum_size) {
      latched_ = true;
      return false;
    }
    return true;
  }

  bool latched() const { return latched_; }
  void reset() { latched_ = false; }

 private:
  bool latched_ = false;
};

inline double hitRatio(const OccupancyVoxelState& state) {
  const uint64_t observations = static_cast<uint64_t>(state.hit_scans) +
                                static_cast<uint64_t>(state.miss_scans);
  return observations == 0
             ? 0.0
             : static_cast<double>(state.hit_scans) /
                   static_cast<double>(observations);
}

inline bool recordHit(OccupancyVoxelState* state, uint64_t scan_sequence,
                      double stamp, const DynamicFilterConfig& config) {
  state->last_seen_time = stamp;
  if (state->last_hit_scan == scan_sequence) {
    return false;
  }
  state->last_hit_scan = scan_sequence;
  state->log_odds =
      std::min(config.max_log_odds, state->log_odds + config.hit_log_odds);
  ++state->hit_scans;
  if (!std::isfinite(state->first_hit_time)) {
    state->first_hit_time = stamp;
  }
  state->last_hit_time = stamp;

  // Any occupied endpoint breaks a run of free-space evidence.
  state->consecutive_clear_misses = 0;
  state->first_clear_miss_time = std::numeric_limits<double>::quiet_NaN();
  state->last_clear_miss_time = std::numeric_limits<double>::quiet_NaN();

  if (!state->confirmed_static &&
      state->hit_scans >= config.min_hit_scans &&
      state->log_odds >= config.occupied_log_odds &&
      stamp - state->first_hit_time >= config.min_observation_span &&
      hitRatio(*state) >= config.min_hit_ratio) {
    state->confirmed_static = true;
    return true;
  }
  return false;
}

// Returns true only when a previously confirmed static voxel has accumulated
// enough consecutive free-space evidence to be removed.
inline bool recordMiss(OccupancyVoxelState* state, uint64_t scan_sequence,
                       double stamp, const DynamicFilterConfig& config) {
  if (state->last_miss_scan == scan_sequence) {
    return false;
  }
  state->last_miss_scan = scan_sequence;
  state->last_seen_time = stamp;
  state->log_odds =
      std::max(config.min_log_odds, state->log_odds + config.miss_log_odds);
  ++state->miss_scans;
  if (state->consecutive_clear_misses == 0) {
    state->first_clear_miss_time = stamp;
  }
  ++state->consecutive_clear_misses;
  state->last_clear_miss_time = stamp;

  return state->confirmed_static &&
         state->log_odds <= config.clearing_log_odds &&
         state->consecutive_clear_misses >= config.min_clear_miss_scans &&
         stamp - state->first_clear_miss_time >= config.min_clear_miss_span;
}

}  // namespace go2_mapping
