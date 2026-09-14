#include <cmath>
#include <fstream>
#include <iterator>
#include <regex>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "go2_mapping/dynamic_mapping.hpp"
#include "go2_mapping/map_storage.hpp"

namespace gm = go2_mapping;

namespace {

std::string readTextFile(const std::string& path) {
  std::ifstream input(path, std::ios::in | std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(input),
                     std::istreambuf_iterator<char>());
}

gm::DynamicFilterConfig fineFilterConfig() {
  gm::DynamicFilterConfig config;
  config.min_hit_scans = 6;
  config.min_observation_span = 0.50;
  config.min_hit_ratio = 0.50;
  return config;
}

}  // namespace

TEST(DdaRay, TraversesEveryAxisAlignedVoxelOnce) {
  const std::vector<gm::VoxelKey> keys = gm::traceRayDda(
      Eigen::Vector3d(0.05, 0.05, 0.05),
      Eigen::Vector3d(0.85, 0.05, 0.05), 0.20);
  ASSERT_EQ(4u, keys.size());
  EXPECT_EQ((gm::VoxelKey{1, 0, 0}), keys[0]);
  EXPECT_EQ((gm::VoxelKey{2, 0, 0}), keys[1]);
  EXPECT_EQ((gm::VoxelKey{3, 0, 0}), keys[2]);
  EXPECT_EQ((gm::VoxelKey{4, 0, 0}), keys[3]);
}

TEST(DdaRay, HandlesNegativeCoordinatesAndCornerTies) {
  const std::vector<gm::VoxelKey> keys = gm::traceRayDda(
      Eigen::Vector3d(-0.05, -0.05, -0.05),
      Eigen::Vector3d(-0.65, -0.65, -0.65), 0.20);
  ASSERT_EQ(3u, keys.size());
  EXPECT_EQ((gm::VoxelKey{-2, -2, -2}), keys[0]);
  EXPECT_EQ((gm::VoxelKey{-3, -3, -3}), keys[1]);
  EXPECT_EQ((gm::VoxelKey{-4, -4, -4}), keys[2]);
}

TEST(FineChildMask, RoundTripsAllChildrenAtNegativeCoordinates) {
  const gm::VoxelKey parent{-2, 3, -1};
  for (uint32_t expected = 0; expected < 64; ++expected) {
    gm::VoxelKey fine;
    ASSERT_TRUE(gm::fineChildKey(parent, 4, expected, &fine));
    uint32_t actual = 64;
    ASSERT_TRUE(gm::fineChildLinearIndex(parent, fine, 4, &actual));
    EXPECT_EQ(expected, actual);
  }
  uint32_t invalid_index = 0;
  EXPECT_FALSE(gm::fineChildLinearIndex(
      parent, gm::VoxelKey{-4, 12, -4}, 4, &invalid_index));
}

TEST(FineChildMask, RayMustCrossVoxelInterior) {
  const gm::VoxelKey voxel{0, 0, 0};
  EXPECT_TRUE(gm::segmentIntersectsVoxelInterior(
      Eigen::Vector3d(-0.10, 0.025, 0.025),
      Eigen::Vector3d(0.30, 0.025, 0.025), voxel, 0.05));
  EXPECT_FALSE(gm::segmentIntersectsVoxelInterior(
      Eigen::Vector3d(-0.10, 0.050, 0.025),
      Eigen::Vector3d(0.30, 0.050, 0.025), voxel, 0.05));
  EXPECT_FALSE(gm::segmentIntersectsVoxelInterior(
      Eigen::Vector3d(-0.10, 0.075, 0.025),
      Eigen::Vector3d(0.30, 0.075, 0.025), voxel, 0.05));
}

TEST(Extrinsics, RotatesBaseToLidarTranslationIntoOdom) {
  const double half_pi = std::acos(-1.0) / 2.0;
  const Eigen::Quaterniond yaw_90(
      Eigen::AngleAxisd(half_pi, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d origin = gm::sensorOrigin(
      Eigen::Vector3d(1.0, 2.0, 3.0), yaw_90,
      Eigen::Vector3d(0.187, 0.0, 0.16));
  EXPECT_NEAR(1.0, origin.x(), 1e-9);
  EXPECT_NEAR(2.187, origin.y(), 1e-9);
  EXPECT_NEAR(3.16, origin.z(), 1e-9);
}

TEST(OdomInterpolation, InterpolatesTranslationAndOrientationAtCloudStamp) {
  const double half_pi = std::acos(-1.0) / 2.0;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  ASSERT_TRUE(gm::interpolatePose(
      10.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity(),
      10.2, Eigen::Vector3d(2.0, 0.0, 1.0),
      Eigen::Quaterniond(Eigen::AngleAxisd(half_pi,
                                           Eigen::Vector3d::UnitZ())),
      10.1, 0.20, &position, &orientation));
  EXPECT_NEAR(1.0, position.x(), 1e-9);
  EXPECT_NEAR(0.5, position.z(), 1e-9);
  const Eigen::Vector3d rotated = orientation * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(std::sqrt(0.5), rotated.x(), 1e-9);
  EXPECT_NEAR(std::sqrt(0.5), rotated.y(), 1e-9);
}

TEST(OdomInterpolation, RejectsSamplesOutsideMaximumAge) {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  EXPECT_FALSE(gm::interpolatePose(
      10.0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), 11.0,
      Eigen::Vector3d::Ones(), Eigen::Quaterniond::Identity(), 10.5, 0.20,
      &position, &orientation));
}

TEST(BayesianVoxel, PromotionRequiresHitsSpanProbabilityAndRatio) {
  gm::DynamicFilterConfig config;
  gm::OccupancyVoxelState state;
  for (uint64_t scan = 1; scan <= 10; ++scan) {
    EXPECT_FALSE(gm::recordMiss(&state, scan, scan * 0.1, config));
  }
  for (uint64_t scan = 11; scan <= 22; ++scan) {
    gm::recordHit(&state, scan, (scan - 11) * 0.2, config);
  }
  EXPECT_FALSE(state.confirmed_static);
  for (uint64_t scan = 23; scan <= 26; ++scan) {
    gm::recordHit(&state, scan, 2.2 + (scan - 22) * 0.2, config);
  }
  EXPECT_TRUE(state.confirmed_static);
  EXPECT_GE(gm::hitRatio(state), config.min_hit_ratio);
}

TEST(BayesianVoxel, ConfirmedVoxelNeedsConsecutiveMissHysteresis) {
  gm::DynamicFilterConfig config;
  gm::OccupancyVoxelState state;
  for (uint64_t scan = 1; scan <= 12; ++scan) {
    gm::recordHit(&state, scan, (scan - 1) * 0.2, config);
  }
  ASSERT_TRUE(state.confirmed_static);

  for (uint64_t scan = 13; scan <= 23; ++scan) {
    EXPECT_FALSE(gm::recordMiss(&state, scan, 3.0 + (scan - 13) * 0.12,
                                config));
  }
  EXPECT_TRUE(gm::recordMiss(&state, 24, 4.32, config));
}

TEST(BayesianVoxel, HitResetsConsecutiveClearingEvidence) {
  gm::DynamicFilterConfig config;
  gm::OccupancyVoxelState state;
  for (uint64_t scan = 1; scan <= 12; ++scan) {
    gm::recordHit(&state, scan, (scan - 1) * 0.2, config);
  }
  for (uint64_t scan = 13; scan <= 22; ++scan) {
    gm::recordMiss(&state, scan, 3.0 + (scan - 13) * 0.2, config);
  }
  gm::recordHit(&state, 23, 5.0, config);
  EXPECT_EQ(0u, state.consecutive_clear_misses);
  for (uint64_t scan = 24; scan <= 34; ++scan) {
    EXPECT_FALSE(gm::recordMiss(&state, scan, 5.1 + (scan - 24) * 0.2,
                                config));
  }
}

TEST(BayesianVoxel, MovingPointClusterNeverPromotesToStatic) {
  gm::DynamicFilterConfig config;
  std::vector<gm::OccupancyVoxelState> visited_voxels(10);
  uint64_t scan = 1;
  for (std::size_t voxel = 0; voxel < visited_voxels.size(); ++voxel) {
    gm::recordHit(&visited_voxels[voxel], scan++, voxel * 0.20, config);
    gm::recordHit(&visited_voxels[voxel], scan++, voxel * 0.20 + 0.10,
                  config);
  }
  for (const gm::OccupancyVoxelState& state : visited_voxels) {
    EXPECT_FALSE(state.confirmed_static);
    EXPECT_LT(state.hit_scans, config.min_hit_scans);
  }
}

TEST(FineVoxelEvidence, GroundContactClusterDoesNotInheritStaticFloor) {
  const gm::DynamicFilterConfig config = fineFilterConfig();
  gm::OccupancyVoxelState confirmed_floor_parent;
  confirmed_floor_parent.confirmed_static = true;
  confirmed_floor_parent.generation = 71;

  // Sixteen 5 cm cells fit in the horizontal face of one 20 cm parent. A
  // moving foot can repeatedly occupy that confirmed floor parent without any
  // individual fine cell accumulating static evidence.
  std::vector<gm::OccupancyVoxelState> foot_cells(16);
  for (uint64_t scan = 1; scan <= 48; ++scan) {
    gm::OccupancyVoxelState& cell = foot_cells[(scan - 1) % foot_cells.size()];
    gm::recordHit(&cell, scan, static_cast<double>(scan) * 0.10, config);
  }
  for (const gm::OccupancyVoxelState& cell : foot_cells) {
    EXPECT_FALSE(cell.confirmed_static);
    EXPECT_FALSE(gm::fineVoxelConfirmedForParent(
        confirmed_floor_parent.generation, &confirmed_floor_parent, cell));
  }
}

TEST(FineVoxelEvidence, WallAdjacentMovingClusterNeedsIndependentEvidence) {
  const gm::DynamicFilterConfig config = fineFilterConfig();
  gm::OccupancyVoxelState confirmed_wall_parent;
  confirmed_wall_parent.confirmed_static = true;
  confirmed_wall_parent.generation = 19;

  gm::OccupancyVoxelState fixed_wall_cell;
  for (uint64_t scan = 1; scan <= 12; ++scan) {
    gm::recordHit(&fixed_wall_cell, scan, (scan - 1) * 0.20, config);
  }
  ASSERT_TRUE(gm::fineVoxelConfirmedForParent(
      confirmed_wall_parent.generation, &confirmed_wall_parent,
      fixed_wall_cell));

  std::vector<gm::OccupancyVoxelState> moving_cells(4);
  for (uint64_t scan = 13; scan <= 36; ++scan) {
    const std::size_t active = (scan - 13) % moving_cells.size();
    gm::recordHit(&moving_cells[active], scan, scan * 0.20, config);
    for (std::size_t index = 0; index < moving_cells.size(); ++index) {
      if (index != active) {
        gm::recordMiss(&moving_cells[index], scan, scan * 0.20, config);
      }
    }
  }
  for (const gm::OccupancyVoxelState& cell : moving_cells) {
    EXPECT_FALSE(cell.confirmed_static);
    EXPECT_LT(gm::hitRatio(cell), config.min_hit_ratio);
    EXPECT_FALSE(gm::fineVoxelConfirmedForParent(
        confirmed_wall_parent.generation, &confirmed_wall_parent, cell));
  }
}

TEST(FineVoxelEvidence, ParentAndChildConfirmInParallelNotSerially) {
  gm::DynamicFilterConfig coarse_config;
  const gm::DynamicFilterConfig fine_config = fineFilterConfig();
  gm::OccupancyVoxelState parent;
  parent.generation = 23;
  gm::OccupancyVoxelState child;
  for (uint64_t scan = 1; scan <= 12; ++scan) {
    const double stamp = (scan - 1) * 0.20;
    gm::recordHit(&parent, scan, stamp, coarse_config);
    gm::recordHit(&child, scan, stamp, fine_config);
    if (scan == 6) {
      EXPECT_TRUE(child.confirmed_static);
      EXPECT_FALSE(parent.confirmed_static);
      EXPECT_FALSE(gm::fineVoxelConfirmedForParent(parent.generation, &parent,
                                                   child));
    }
  }
  ASSERT_TRUE(parent.confirmed_static);
  ASSERT_TRUE(child.confirmed_static);
  EXPECT_TRUE(gm::fineVoxelConfirmedForParent(parent.generation, &parent,
                                              child));
  EXPECT_DOUBLE_EQ(parent.first_hit_time, child.first_hit_time);
  EXPECT_DOUBLE_EQ(parent.last_hit_time, child.last_hit_time);
}

TEST(FineVoxelEvidence, CachedPointPublishesAfterParentConfirmsWithoutNewHit) {
  gm::DynamicFilterConfig coarse_config;
  const gm::DynamicFilterConfig fine_config = fineFilterConfig();
  gm::OccupancyVoxelState parent;
  parent.generation = 37;
  gm::OccupancyVoxelState child;
  uint32_t sample_count = 0;

  // The fine endpoint stabilizes during the first six scans, then scan jitter
  // moves all later endpoints to sibling cells in the same coarse parent.
  for (uint64_t scan = 1; scan <= 6; ++scan) {
    const double stamp = (scan - 1) * 0.20;
    gm::recordHit(&parent, scan, stamp, coarse_config);
    gm::recordHit(&child, scan, stamp, fine_config);
    if (gm::fineVoxelReadyToCache(child)) {
      ++sample_count;
    }
  }
  ASSERT_TRUE(child.confirmed_static);
  ASSERT_EQ(1u, sample_count);
  EXPECT_FALSE(gm::fineVoxelPublishable(parent.generation, &parent, child,
                                        sample_count));

  for (uint64_t scan = 7; scan <= 12; ++scan) {
    gm::recordHit(&parent, scan, (scan - 1) * 0.20, coarse_config);
  }
  ASSERT_TRUE(parent.confirmed_static);
  EXPECT_TRUE(gm::fineVoxelPublishable(parent.generation, &parent, child,
                                       sample_count));
  EXPECT_FALSE(gm::fineVoxelPublishable(parent.generation + 1, &parent, child,
                                        sample_count));
  EXPECT_FALSE(
      gm::fineVoxelPublishable(parent.generation, &parent, child, 0));
}

TEST(FineVoxelEvidence, StaticWallJitterPromotesAdjacentFiveCentimeterCells) {
  const gm::DynamicFilterConfig fine_config = fineFilterConfig();
  gm::OccupancyVoxelState confirmed_parent;
  confirmed_parent.confirmed_static = true;
  confirmed_parent.generation = 29;
  gm::OccupancyVoxelState fine_cells[2];

  const gm::VoxelKey lower = gm::voxelKey(
      Eigen::Vector3d(1.0 - 0.025, 0.125, 0.825), 0.05);
  const gm::VoxelKey upper = gm::voxelKey(
      Eigen::Vector3d(1.0 + 0.025, 0.125, 0.825), 0.05);
  ASSERT_NE(lower, upper);

  // A fixed wall return oscillates by +/-2.5 cm across one fine-cell boundary.
  // Neither cell sees more than one consecutive miss, so both should become
  // independently public once six temporally distributed hits accumulate.
  for (uint64_t scan = 1; scan <= 12; ++scan) {
    const std::size_t active = (scan - 1) % 2;
    const std::size_t inactive = 1 - active;
    gm::recordHit(&fine_cells[active], scan, (scan - 1) * 0.10,
                  fine_config);
    gm::recordMiss(&fine_cells[inactive], scan, (scan - 1) * 0.10,
                   fine_config);
    EXPECT_FALSE(
        gm::unconfirmedEvidenceCleared(fine_cells[inactive], fine_config));
  }

  for (const gm::OccupancyVoxelState& cell : fine_cells) {
    EXPECT_TRUE(cell.confirmed_static);
    EXPECT_GE(gm::hitRatio(cell), fine_config.min_hit_ratio);
    EXPECT_TRUE(gm::fineVoxelConfirmedForParent(
        confirmed_parent.generation, &confirmed_parent, cell));
  }
}

TEST(FineVoxelEvidence, RepeatedMovingClusterNeverPassesFineGate) {
  const gm::DynamicFilterConfig fine_config = fineFilterConfig();
  gm::OccupancyVoxelState moving_cells[4];
  for (uint64_t scan = 1; scan <= 40; ++scan) {
    const std::size_t active = (scan - 1) % 4;
    for (std::size_t index = 0; index < 4; ++index) {
      if (index == active) {
        gm::recordHit(&moving_cells[index], scan, (scan - 1) * 0.10,
                      fine_config);
      } else {
        gm::recordMiss(&moving_cells[index], scan, (scan - 1) * 0.10,
                       fine_config);
      }
    }
  }
  for (const gm::OccupancyVoxelState& cell : moving_cells) {
    EXPECT_FALSE(cell.confirmed_static);
    EXPECT_LT(gm::hitRatio(cell), fine_config.min_hit_ratio);
  }
}

TEST(FineVoxelEvidence, CandidateRequiresFourMissesAndMinimumSpanToEvict) {
  gm::DynamicFilterConfig config;
  gm::OccupancyVoxelState candidate;
  gm::recordHit(&candidate, 1, 1.0, config);
  EXPECT_FALSE(gm::evidenceCandidateExpired(candidate, 7.0, 6.0));
  EXPECT_TRUE(gm::evidenceCandidateExpired(candidate, 7.01, 6.0));
  gm::recordMiss(&candidate, 2, 1.1, config);
  EXPECT_FALSE(gm::unconfirmedEvidenceCleared(candidate, config));
  gm::recordMiss(&candidate, 3, 1.21, config);
  EXPECT_FALSE(gm::unconfirmedEvidenceCleared(candidate, config));
  gm::recordMiss(&candidate, 4, 1.32, config);
  EXPECT_FALSE(gm::unconfirmedEvidenceCleared(candidate, config));
  gm::recordMiss(&candidate, 5, 1.44, config);
  EXPECT_TRUE(gm::unconfirmedEvidenceCleared(candidate, config));
}

TEST(FineVoxelEvidence, CandidateHitResetsShortMissRun) {
  gm::DynamicFilterConfig config;
  gm::OccupancyVoxelState candidate;
  gm::recordHit(&candidate, 1, 1.0, config);
  for (uint64_t scan = 2; scan <= 4; ++scan) {
    gm::recordMiss(&candidate, scan, 1.0 + (scan - 1) * 0.15, config);
    EXPECT_FALSE(gm::unconfirmedEvidenceCleared(candidate, config));
  }
  gm::recordHit(&candidate, 5, 1.50, config);
  EXPECT_EQ(0u, candidate.consecutive_clear_misses);
  for (uint64_t scan = 6; scan <= 8; ++scan) {
    gm::recordMiss(&candidate, scan, 1.50 + (scan - 5) * 0.15, config);
    EXPECT_FALSE(gm::unconfirmedEvidenceCleared(candidate, config));
  }
}

TEST(MapGeneration, RejectsFineVoxelAfterCoarseVoxelRecreation) {
  gm::OccupancyVoxelState original;
  original.generation = 41;
  const uint64_t fine_voxel_generation = original.generation;
  EXPECT_TRUE(gm::generationMatches(fine_voxel_generation, &original));

  // Clearing erases the coarse state. A later observation at the same key gets
  // a new generation, so samples accumulated by the old object cannot return.
  EXPECT_FALSE(gm::generationMatches(fine_voxel_generation, nullptr));
  gm::OccupancyVoxelState recreated;
  recreated.generation = 42;
  EXPECT_FALSE(gm::generationMatches(fine_voxel_generation, &recreated));
}

TEST(VoxelCapacity, LatchesFailClosedUntilExplicitReset) {
  gm::VoxelCapacityGate gate;
  EXPECT_TRUE(gate.allowNewVoxel(0, 2));
  EXPECT_TRUE(gate.allowNewVoxel(1, 2));
  EXPECT_FALSE(gate.allowNewVoxel(2, 2));
  EXPECT_TRUE(gate.latched());
  EXPECT_FALSE(gate.allowNewVoxel(0, 2));
  gate.reset();
  EXPECT_FALSE(gate.latched());
  EXPECT_TRUE(gate.allowNewVoxel(0, 2));
}

TEST(FineCandidateCapacity, RejectsTransientGrowthAtIndependentHardLimit) {
  EXPECT_TRUE(gm::fineCandidateCapacityAvailable(0, 2));
  EXPECT_TRUE(gm::fineCandidateCapacityAvailable(1, 2));
  EXPECT_FALSE(gm::fineCandidateCapacityAvailable(2, 2));
  EXPECT_FALSE(gm::fineCandidateCapacityAvailable(3, 2));
}

TEST(AtomicPcdStorage, ShutdownStyleSaveReplacesOnlyACompleteFile) {
  using StoragePoint = pcl::PointXYZI;
  using StorageCloud = pcl::PointCloud<StoragePoint>;
  const boost::filesystem::path directory =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("go2_mapping_test_%%%%-%%%%-%%%%");
  ASSERT_TRUE(boost::filesystem::create_directories(directory));
  const std::string output = (directory / "public_map.pcd").string();
  std::string error;

  StorageCloud first;
  StoragePoint first_point;
  first_point.x = 1.0F;
  first.push_back(first_point);
  ASSERT_TRUE(gm::prepareBinaryPcd(output, first, &error)) << error;
  ASSERT_TRUE(gm::commitPreparedPcd(output, &error)) << error;

  StorageCloud replacement;
  StoragePoint second_point;
  second_point.x = 2.0F;
  replacement.push_back(second_point);
  second_point.x = 3.0F;
  replacement.push_back(second_point);
  ASSERT_TRUE(gm::prepareBinaryPcd(output, replacement, &error)) << error;
  ASSERT_TRUE(boost::filesystem::exists(output));
  ASSERT_TRUE(gm::commitPreparedPcd(output, &error)) << error;
  EXPECT_FALSE(boost::filesystem::exists(gm::temporaryPcdPath(output)));

  StorageCloud loaded;
  ASSERT_EQ(0, pcl::io::loadPCDFile(output, loaded));
  ASSERT_EQ(2u, loaded.size());
  EXPECT_FLOAT_EQ(2.0F, loaded[0].x);
  EXPECT_FLOAT_EQ(3.0F, loaded[1].x);

  const std::string trajectory_output =
      (directory / "traversed_path_map.pcd").string();
  StorageCloud trajectory;
  StoragePoint trajectory_point;
  trajectory_point.x = 0.5F;
  trajectory.push_back(trajectory_point);
  ASSERT_TRUE(gm::prepareBinaryPcd(trajectory_output, trajectory, &error))
      << error;
  ASSERT_TRUE(gm::commitPreparedPcd(trajectory_output, &error)) << error;

  const std::string abc_path = (directory / "abc.dat").string();
  {
    std::ofstream abc(abc_path, std::ios::out | std::ios::binary);
    abc << "abc";
  }
  std::string abc_digest;
  ASSERT_TRUE(gm::sha256File(abc_path, &abc_digest, &error)) << error;
  EXPECT_EQ("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
            abc_digest);

  std::string manifest_contents;
  ASSERT_TRUE(gm::buildSnapshotManifest(output, trajectory_output,
                                        &manifest_contents, &error))
      << error;
  const std::regex strict_gnu_format(
      "[0-9a-f]{64}  public_map\\.pcd\\n"
      "[0-9a-f]{64}  traversed_path_map\\.pcd\\n");
  EXPECT_TRUE(std::regex_match(manifest_contents, strict_gnu_format));

  const std::string manifest_output =
      (directory / "mapping_snapshot.sha256").string();
  {
    std::ofstream old_manifest(manifest_output,
                               std::ios::out | std::ios::binary);
    old_manifest << "stale snapshot\n";
  }
  gm::discardSnapshotManifest(manifest_output);
  EXPECT_FALSE(boost::filesystem::exists(manifest_output));
  ASSERT_TRUE(gm::prepareSnapshotManifest(
      manifest_output, manifest_contents, &error)) << error;
  EXPECT_FALSE(boost::filesystem::exists(manifest_output));
  EXPECT_TRUE(boost::filesystem::exists(
      gm::temporaryPcdPath(manifest_output)));
  ASSERT_TRUE(gm::commitPreparedSnapshotManifest(manifest_output, &error))
      << error;
  EXPECT_EQ(manifest_contents, readTextFile(manifest_output));
  EXPECT_FALSE(boost::filesystem::exists(
      gm::temporaryPcdPath(manifest_output)));
  boost::filesystem::remove_all(directory);
}

TEST(AsyncSnapshotWriter, AllowsOnlyOneWriterAndCommitsManifestLast) {
  using StoragePoint = pcl::PointXYZI;
  using StorageCloud = pcl::PointCloud<StoragePoint>;
  const boost::filesystem::path directory =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("go2_async_writer_%%%%-%%%%-%%%%");
  ASSERT_TRUE(boost::filesystem::create_directories(directory));
  const std::string map_path = (directory / "public_map.pcd").string();
  const std::string trajectory_path =
      (directory / "traversed_path_map.pcd").string();
  const std::string manifest_path =
      (directory / "mapping_snapshot.sha256").string();

  StoragePoint point;
  point.x = 1.0F;
  StorageCloud map;
  map.push_back(point);
  StorageCloud trajectory;
  trajectory.push_back(point);

  gm::AsyncSnapshotWriter<StoragePoint> writer;
  std::string error;
  ASSERT_TRUE(writer.start(map_path, trajectory_path, manifest_path, map,
                           trajectory, 17, &error)) << error;
  EXPECT_TRUE(writer.busy());
  EXPECT_FALSE(writer.start(map_path, trajectory_path, manifest_path, map,
                            trajectory, 18, &error));
  const gm::SnapshotWriteResult result = writer.wait();
  EXPECT_TRUE(result.had_work);
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(17u, result.mutation_sequence);
  EXPECT_FALSE(writer.busy());
  EXPECT_TRUE(boost::filesystem::exists(map_path));
  EXPECT_TRUE(boost::filesystem::exists(trajectory_path));
  EXPECT_TRUE(boost::filesystem::exists(manifest_path));
  EXPECT_FALSE(boost::filesystem::exists(
      gm::temporaryPcdPath(manifest_path)));

  std::string expected_manifest;
  ASSERT_TRUE(gm::buildSnapshotManifest(map_path, trajectory_path,
                                        &expected_manifest, &error)) << error;
  EXPECT_EQ(expected_manifest, readTextFile(manifest_path));
  boost::filesystem::remove_all(directory);
}

TEST(SnapshotManifest, ResetInvalidatesCommittedAndPreparedSnapshots) {
  const boost::filesystem::path directory =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("go2_reset_manifest_%%%%-%%%%-%%%%");
  ASSERT_TRUE(boost::filesystem::create_directories(directory));
  const std::string manifest_path =
      (directory / "mapping_snapshot.sha256").string();
  const std::string temporary_path = gm::temporaryPcdPath(manifest_path);
  {
    std::ofstream committed(manifest_path,
                            std::ios::out | std::ios::binary);
    committed << "old committed snapshot\n";
    std::ofstream temporary(temporary_path,
                            std::ios::out | std::ios::binary);
    temporary << "interrupted replacement\n";
  }
  ASSERT_TRUE(boost::filesystem::exists(manifest_path));
  ASSERT_TRUE(boost::filesystem::exists(temporary_path));

  std::string error;
  ASSERT_TRUE(gm::invalidateSnapshotManifest(manifest_path, &error)) << error;
  EXPECT_FALSE(boost::filesystem::exists(manifest_path));
  EXPECT_FALSE(boost::filesystem::exists(temporary_path));
  EXPECT_TRUE(gm::invalidateSnapshotManifest(manifest_path, &error)) << error;
  boost::filesystem::remove_all(directory);
}
