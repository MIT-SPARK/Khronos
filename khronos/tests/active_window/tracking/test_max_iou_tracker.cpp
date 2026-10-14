#include <initializer_list>
#include <memory>

#include <config_utilities/validation.h>
#include <gtest/gtest.h>
#include <hydra/input/camera.h>
#include <khronos/active_window/tracking/max_iou_tracker.h>

namespace khronos {
namespace {

using Association = MaxIoUTracker::Config::SemanticAssociation;

struct Detection {
  int id;
  float x;
  int category = 1;
};

void appendObservations(std::initializer_list<Detection> detections,
                        cv::Mat& vertex_map,
                        MeasurementClusters& clusters,
                        int& column,
                        bool is_dynamic) {
  for (const auto& detection : detections) {
    MeasurementCluster cluster;
    cluster.id = detection.id;
    cluster.pixels = {{column, 0}, {column, 1}};
    vertex_map.at<cv::Vec3f>(0, column) = {detection.x, 0.0f, 1.0f};
    vertex_map.at<cv::Vec3f>(1, column) = {detection.x + 1.0f, 1.0f, 2.0f};
    if (!is_dynamic) {
      cluster.semantics = SemanticClusterInfo(detection.category);
    }

    clusters.push_back(cluster);
    ++column;
  }
}

//! Voxels of the axis-aligned voxel box [min, max] (inclusive).
GlobalIndexSet voxelBox(const GlobalIndex& min, const GlobalIndex& max) {
  GlobalIndexSet voxels;
  for (int x = min.x(); x <= max.x(); ++x) {
    for (int y = min.y(); y <= max.y(); ++y) {
      for (int z = min.z(); z <= max.z(); ++z) {
        voxels.emplace(x, y, z);
      }
    }
  }
  return voxels;
}

//! Static track updated in tracker pass 0 whose voxels were each observed `hits` times.
Track makeTrack(int id, TimeStamp first_seen, const GlobalIndexSet& voxels, int hits, int category = 1) {
  Track track;
  track.id = id;
  track.first_seen = first_seen;
  track.last_seen = first_seen + hits - 1;
  track.semantics = SemanticClusterInfo(category);
  track.sequence_number = 0;
  for (int i = 0; i < hits; ++i) {
    track.observations.emplace_back(first_seen + i, id, -1, "camera");
    track.accumulateVoxels(voxels, 0.1f, first_seen + i, 2);
  }
  return track;
}

}  // namespace

class MaxIoUTrackerTest : public testing::TestWithParam<Association> {
 protected:
  MaxIoUTrackerTest() {
    hydra::Camera::Config camera;
    camera.width = 32;
    camera.height = 2;
    camera.cx = 16.0f;
    camera.cy = 1.0f;
    camera.fx = 10.0f;
    camera.fy = 10.0f;
    camera.min_range = 0.1f;
    camera.max_range = 100.0f;
    camera.extrinsics = hydra::ParamSensorExtrinsics::Config();
    sensor = std::make_shared<hydra::Camera>(camera, "camera");
    config.track_by = MaxIoUTracker::Config::TrackBy::kBoundingBox;
  }

  FrameData frame(TimeStamp stamp,
                  std::initializer_list<Detection> semantic,
                  std::initializer_list<Detection> dynamic = {}) const {
    InputData input(sensor);
    input.timestamp_ns = stamp;
    input.vertex_map = cv::Mat::zeros(2, 32, CV_32FC3);
    FrameData data(input);
    int column = 0;
    appendObservations(semantic, input.vertex_map, data.semantic_clusters, column, false);
    appendObservations(dynamic, input.vertex_map, data.dynamic_clusters, column, true);
    return data;
  }

  MaxIoUTracker::Config config;
  std::shared_ptr<hydra::Camera> sensor;
};

TEST_P(MaxIoUTrackerTest, SemanticAssociation) {
  config.semantic_association = GetParam();
  MaxIoUTracker tracker(config);
  Tracks tracks;
  auto first = frame(1, {{10, 0.0f}, {20, 10.0f}});
  tracker.processInput(first, tracks);
  ASSERT_EQ(tracks.size(), 2u);
  const auto first_id = tracks[0].id;
  const auto second_id = tracks[1].id;

  auto next = frame(2, {{11, 0.0f}, {12, 0.0f}, {21, 10.0f, 2}, {30, 20.0f}});
  tracker.processInput(next, tracks);
  ASSERT_EQ(tracks.size(), 5u);
  EXPECT_EQ(tracks[0].id, first_id);
  ASSERT_EQ(tracks[0].observations.size(), 2u);
  EXPECT_EQ(tracks[0].observations.back().semantic_cluster_id, 11);
  EXPECT_EQ(tracks[0].first_seen, 1u);
  EXPECT_EQ(tracks[0].last_seen, 2u);
  EXPECT_EQ(tracks[1].id, second_id);
  EXPECT_EQ(tracks[1].observations.size(), 1u);
  EXPECT_EQ(tracks[2].observations.back().semantic_cluster_id, 12);
  EXPECT_EQ(tracks[3].observations.back().semantic_cluster_id, 21);
  EXPECT_EQ(tracks[4].observations.back().semantic_cluster_id, 30);
}

TEST_P(MaxIoUTrackerTest, IdMatchingTakesPriority) {
  config.semantic_association = GetParam();
  for (const bool enabled : {false, true}) {
    SCOPED_TRACE(enabled);
    config.preassociate_by_id = enabled;
    MaxIoUTracker tracker(config);
    Tracks tracks;
    auto first = frame(1, {{40, 0.0f}, {0, 10.0f}});
    tracker.processInput(first, tracks);
    auto next = frame(2, {{0, 0.0f}, {40, 10.0f}});
    tracker.processInput(next, tracks);
    ASSERT_EQ(tracks.size(), 2u);
    EXPECT_EQ(tracks[0].observations.back().semantic_cluster_id, enabled ? 40 : 0);
    EXPECT_EQ(tracks[1].observations.back().semantic_cluster_id, enabled ? 0 : 40);
    EXPECT_EQ(tracks[0].observations.size(), 2u);
    EXPECT_EQ(tracks[1].observations.size(), 2u);
  }
}

TEST_P(MaxIoUTrackerTest, UnmatchedIdsFallBack) {
  config.semantic_association = GetParam();
  config.preassociate_by_id = true;
  MaxIoUTracker tracker(config);
  Tracks tracks;
  auto first = frame(1, {{10, 0.0f}, {20, 10.0f}});
  tracker.processInput(first, tracks);
  // The same timestamp still represents a separate tracker pass.
  auto next = frame(1, {{10, 30.0f}, {21, 10.0f}, {30, 20.0f}});
  tracker.processInput(next, tracks);
  ASSERT_EQ(tracks.size(), 3u);
  EXPECT_EQ(tracks[0].observations.back().semantic_cluster_id, 10);
  EXPECT_EQ(tracks[1].observations.back().semantic_cluster_id, 21);
  EXPECT_EQ(tracks[2].observations.back().semantic_cluster_id, 30);
  EXPECT_EQ(tracks[0].observations.size(), 2u);
  EXPECT_EQ(tracks[1].observations.size(), 2u);

  auto last = frame(2, {{21, 50.0f}});
  tracker.processInput(last, tracks);
  ASSERT_EQ(tracks.size(), 3u);
  EXPECT_EQ(tracks[1].observations.size(), 3u);
  EXPECT_EQ(tracks[1].last_bounding_box.world_P_center.x(), 50.5f);
  EXPECT_EQ(tracks[0].observations.size(), 2u);
}

TEST(TrackAccumulation, BoundingBoxGrowsWithConfirmedVoxels) {
  Track track;
  const float voxel_size = 0.5f;
  // Two hits needed: a voxel seen once is not in the bounding box.
  track.accumulateVoxels({GlobalIndex(0, 0, 0), GlobalIndex(4, 0, 0)}, voxel_size, 1, 2);
  EXPECT_EQ(track.accumulated_voxels.size(), 2u);
  EXPECT_FALSE(track.accumulated_bounding_box.isValid());
  EXPECT_TRUE(track.confirmedVoxels().empty());

  // Second hit of (0,0,0) confirms it: the box is exactly its cube.
  track.accumulateVoxels({GlobalIndex(0, 0, 0), GlobalIndex(0, 2, 0)}, voxel_size, 2, 2);
  EXPECT_EQ(track.accumulated_voxels.at(GlobalIndex(0, 0, 0)).hits, 2);
  EXPECT_EQ(track.accumulated_voxels.at(GlobalIndex(0, 0, 0)).last_seen, 2u);
  EXPECT_EQ(track.accumulated_voxels.at(GlobalIndex(4, 0, 0)).last_seen, 1u);
  ASSERT_TRUE(track.accumulated_bounding_box.isValid());
  EXPECT_EQ(track.confirmedVoxels(), GlobalIndexSet({GlobalIndex(0, 0, 0)}));
  EXPECT_TRUE(track.accumulated_bounding_box.dimensions.isApprox(Eigen::Vector3f::Constant(0.5f)));
  EXPECT_TRUE(track.accumulated_bounding_box.world_P_center.isApprox(
      Eigen::Vector3f::Constant(0.25f)));

  // Confirming (4,0,0) grows the box to enclose both cubes; more hits don't change it.
  track.accumulateVoxels({GlobalIndex(4, 0, 0)}, voxel_size, 3, 2);
  track.accumulateVoxels({GlobalIndex(0, 0, 0), GlobalIndex(4, 0, 0)}, voxel_size, 4, 2);
  EXPECT_EQ(track.confirmedVoxels().size(), 2u);
  EXPECT_TRUE(track.accumulated_bounding_box.dimensions.isApprox(Eigen::Vector3f(2.5f, 0.5f, 0.5f)));
  EXPECT_TRUE(track.accumulated_bounding_box.world_P_center.isApprox(
      Eigen::Vector3f(1.25f, 0.25f, 0.25f)));
}

TEST(TrackAccumulation, SingleHitIsUnion) {
  Track track;
  track.accumulateVoxels({GlobalIndex(0, 0, 0)}, 1.0f, 1, 1);
  track.accumulateVoxels({GlobalIndex(1, 1, 1)}, 1.0f, 2, 1);
  EXPECT_EQ(track.confirmedVoxels().size(), 2u);
  EXPECT_TRUE(track.accumulated_bounding_box.dimensions.isApprox(Eigen::Vector3f::Constant(2.0f)));
}

INSTANTIATE_TEST_SUITE_P(AssociationModes,
                         MaxIoUTrackerTest,
                         testing::Values(Association::kAssignCluster, Association::kAssignTrack));

TEST_F(MaxIoUTrackerTest, DynamicAndCrossAssociation) {
  MaxIoUTracker tracker(config);
  Tracks tracks;
  auto first = frame(1, {{40, 0.0f}}, {{5, 0.0f}});
  tracker.processInput(first, tracks);
  ASSERT_EQ(tracks.size(), 1u);
  const auto id = tracks[0].id;
  auto next = frame(2, {{41, 0.25f}}, {{6, 0.25f}, {7, 10.0f}});
  tracker.processInput(next, tracks);
  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0].id, id);
  EXPECT_TRUE(tracks[0].is_dynamic);
  ASSERT_EQ(tracks[0].observations.size(), 2u);
  EXPECT_EQ(tracks[0].observations.back().semantic_cluster_id, 41);
  EXPECT_EQ(tracks[0].observations.back().dynamic_cluster_id, 6);
  EXPECT_EQ(tracks[0].observations.back().sensor, "camera");
  EXPECT_FLOAT_EQ(tracks[0].confidence, 1.0f / config.min_num_observations);
  EXPECT_FLOAT_EQ(tracks[0].last_centroid.x(), 0.75f);
  EXPECT_TRUE(tracks[1].is_dynamic);
  EXPECT_EQ(tracks[1].observations.size(), 1u);
  EXPECT_EQ(tracks[1].observations.back().dynamic_cluster_id, 7);
}

TEST_F(MaxIoUTrackerTest, DynamicIdPreassociation) {
  config.preassociate_by_id = true;
  for (const bool enabled : {false, true}) {
    for (const bool has_dynamic : {false, true}) {
      SCOPED_TRACE(testing::Message() << "enabled=" << enabled << ", dynamic=" << has_dynamic);
      config.preassociate_to_dynamic = enabled;
      MaxIoUTracker tracker(config);
      Tracks tracks;
      auto first = frame(1, {{40, 0.0f}}, {{5, 0.0f}});
      tracker.processInput(first, tracks);
      auto gap = frame(2, {}, {{6, 0.0f}});
      tracker.processInput(gap, tracks);
      ASSERT_EQ(tracks.size(), 1u);
      ASSERT_EQ(tracks[0].observations.size(), 2u);

      // The ID match is far away. A competing cluster must not overwrite it.
      const auto competitor_x = enabled && !has_dynamic ? 10.0f : 0.0f;
      auto next = frame(3,
                        {{40, 10.0f}, {50, competitor_x}},
                        has_dynamic ? std::initializer_list<Detection>{{7, 0.0f}}
                                    : std::initializer_list<Detection>{});
      tracker.processInput(next, tracks);
      ASSERT_EQ(tracks.size(), 2u);
      ASSERT_EQ(tracks[0].observations.size(), 3u);
      const auto& observation = tracks[0].observations.back();
      EXPECT_EQ(observation.semantic_cluster_id, enabled ? 40 : 50);
      EXPECT_EQ(observation.dynamic_cluster_id, has_dynamic ? 7 : -1);
      EXPECT_FLOAT_EQ(tracks[0].confidence, 3.0f / (2.0f * config.min_num_observations));
      EXPECT_EQ(tracks[1].observations.back().semantic_cluster_id, enabled ? 50 : 40);
      EXPECT_FALSE(tracks[1].is_dynamic);
      if (!has_dynamic) {
        ASSERT_TRUE(tracks[0].semantics);
        EXPECT_EQ(tracks[0].semantics->category_id, 1);
      }
    }
  }
}

TEST_F(MaxIoUTrackerTest, AccumulatesStaticObservations) {
  config.min_voxel_hits = 2;
  MaxIoUTracker tracker(config);
  Tracks tracks;
  // Each detection has two points, (x, 0, 1) and (x + 1, 1, 2), in different voxels.
  auto first = frame(1, {{10, 0.0f}}, {{5, 20.0f}});
  tracker.processInput(first, tracks);
  ASSERT_EQ(tracks.size(), 2u);
  const Track& dynamic = tracks[0].is_dynamic ? tracks[0] : tracks[1];
  const Track& track = tracks[0].is_dynamic ? tracks[1] : tracks[0];
  EXPECT_TRUE(dynamic.accumulated_voxels.empty());
  EXPECT_EQ(track.accumulated_voxels.size(), 2u);
  EXPECT_FLOAT_EQ(track.accumulated_voxel_size, config.voxel_size);
  EXPECT_EQ(track.accumulated_min_hits, 2);
  EXPECT_FALSE(track.accumulated_bounding_box.isValid());

  // The same object again: both voxels confirmed, last_bounding_box is unaffected.
  auto second = frame(2, {{10, 0.0f}});
  tracker.processInput(second, tracks);
  const Track& updated = tracks[0].is_dynamic ? tracks[1] : tracks[0];
  ASSERT_EQ(updated.observations.size(), 2u);
  EXPECT_EQ(updated.confirmedVoxels().size(), 2u);
  ASSERT_TRUE(updated.accumulated_bounding_box.isValid());
  // Two voxel cubes around (0, 0, 1) and (1, 1, 2): about 1.1 m on each side.
  const auto& bbox = updated.accumulated_bounding_box;
  EXPECT_LT((bbox.dimensions - Eigen::Vector3f::Constant(1.1f)).cwiseAbs().maxCoeff(), 1e-3f);
  EXPECT_LT((bbox.world_P_center - Eigen::Vector3f(0.5f, 0.5f, 1.5f)).cwiseAbs().maxCoeff(),
            config.voxel_size);
  EXPECT_EQ(updated.last_bounding_box, second.semantic_clusters[0].bounding_box);
}

TEST_F(MaxIoUTrackerTest, AccumulationDoesNotChangeAssociation) {
  Tracks with;
  Tracks without;
  for (const bool accumulate : {true, false}) {
    config.accumulate_voxels = accumulate;
    MaxIoUTracker tracker(config);
    auto& tracks = accumulate ? with : without;
    auto first = frame(1, {{10, 0.0f}, {20, 10.0f}});
    tracker.processInput(first, tracks);
    auto next = frame(2, {{11, 0.0f}, {12, 0.0f}, {21, 10.0f, 2}, {30, 20.0f}});
    tracker.processInput(next, tracks);
  }

  ASSERT_EQ(with.size(), without.size());
  for (size_t i = 0; i < with.size(); ++i) {
    EXPECT_EQ(with[i].id, without[i].id);
    EXPECT_EQ(with[i].observations.size(), without[i].observations.size());
    EXPECT_EQ(with[i].last_bounding_box, without[i].last_bounding_box);
    EXPECT_FALSE(with[i].accumulated_voxels.empty());
    EXPECT_TRUE(without[i].accumulated_voxels.empty());
  }
}

TEST(TrackAbsorb, CombinesTracks) {
  Track older = makeTrack(1, 10, voxelBox({0, 0, 0}, {1, 0, 0}), 1);
  Track newer = makeTrack(2, 20, voxelBox({1, 0, 0}, {2, 0, 0}), 1);
  newer.absorbed_track_ids = {5};
  older.absorb(std::move(newer));

  EXPECT_EQ(older.id, 1);
  EXPECT_EQ(older.absorbed_track_ids, std::set<int>({2, 5}));
  EXPECT_EQ(older.absorbed_instance_ids, std::set<int>({1, 2}));
  EXPECT_EQ(older.first_seen, 10u);
  EXPECT_EQ(older.last_seen, 20u);
  ASSERT_EQ(older.observations.size(), 2u);
  EXPECT_EQ(older.observations.back().stamp, 20u);
  EXPECT_EQ(older.latestInstanceId(), 2);
  // Hits add up: the shared voxel got confirmed, so the bbox is exactly its cube.
  EXPECT_EQ(older.accumulated_voxels.at(GlobalIndex(1, 0, 0)).hits, 2);
  EXPECT_EQ(older.confirmedVoxels(), GlobalIndexSet({GlobalIndex(1, 0, 0)}));
  ASSERT_TRUE(older.accumulated_bounding_box.isValid());
  EXPECT_TRUE(older.accumulated_bounding_box.dimensions.isApprox(Eigen::Vector3f::Constant(0.1f)));
}

TEST_F(MaxIoUTrackerTest, MergesContainedFragmentIntoOlderTrack) {
  config.merge_tracks = true;
  const MaxIoUTracker tracker(config);
  for (const bool newer_is_container : {false, true}) {
    SCOPED_TRACE(newer_is_container);
    const auto big = voxelBox({0, 0, 0}, {3, 3, 3});
    const auto small = voxelBox({1, 1, 1}, {2, 2, 2});
    Tracks tracks{makeTrack(1, 10, newer_is_container ? small : big, 2),
                  makeTrack(2, 20, newer_is_container ? big : small, 2)};
    tracker.mergeTracks(tracks);
    ASSERT_EQ(tracks.size(), 1u);
    EXPECT_EQ(tracks[0].id, 1);
    EXPECT_EQ(tracks[0].absorbed_track_ids, std::set<int>({2}));
    EXPECT_EQ(tracks[0].accumulated_voxels.size(), big.size());
    EXPECT_EQ(tracks[0].accumulated_voxels.at(GlobalIndex(1, 1, 1)).hits, 4);
  }
}

TEST_F(MaxIoUTrackerTest, MergeGates) {
  config.merge_tracks = true;
  const auto big = voxelBox({0, 0, 0}, {3, 3, 3});
  const auto small = voxelBox({1, 1, 1}, {2, 2, 2});
  {
    SCOPED_TRACE("unconfirmed voxels don't count");
    Tracks tracks{makeTrack(1, 10, big, 2), makeTrack(2, 20, small, 1)};
    MaxIoUTracker(config).mergeTracks(tracks);
    EXPECT_EQ(tracks.size(), 2u);
  }
  {
    SCOPED_TRACE("semantics must match");
    Tracks tracks{makeTrack(1, 10, big, 2), makeTrack(2, 20, small, 2, 2)};
    MaxIoUTracker(config).mergeTracks(tracks);
    EXPECT_EQ(tracks.size(), 2u);
  }
  {
    SCOPED_TRACE("inactive tracks don't merge");
    Tracks tracks{makeTrack(1, 10, big, 2), makeTrack(2, 20, small, 2)};
    tracks[0].is_active = false;
    MaxIoUTracker(config).mergeTracks(tracks);
    EXPECT_EQ(tracks.size(), 2u);
  }
  {
    SCOPED_TRACE("distance gate");
    // Centers 0.1 m apart; the big track's radius is ~0.35 m.
    auto distance_config = config;
    distance_config.merge_distance_factor = 0.1f;
    Tracks tracks{makeTrack(1, 10, big, 2), makeTrack(2, 20, voxelBox({0, 0, 0}, {1, 1, 1}), 2)};
    MaxIoUTracker(distance_config).mergeTracks(tracks);
    EXPECT_EQ(tracks.size(), 2u);
  }
  {
    SCOPED_TRACE("containment threshold");
    // Two equal tracks sharing half of their voxels.
    const auto left = voxelBox({0, 0, 0}, {1, 1, 1});
    const auto right = voxelBox({1, 0, 0}, {2, 1, 1});
    for (const float threshold : {0.6f, 0.5f}) {
      auto threshold_config = config;
      threshold_config.merge_containment_threshold = threshold;
      Tracks tracks{makeTrack(1, 10, left, 2), makeTrack(2, 20, right, 2)};
      MaxIoUTracker(threshold_config).mergeTracks(tracks);
      EXPECT_EQ(tracks.size(), threshold > 0.5f ? 2u : 1u);
    }
  }
}

TEST_F(MaxIoUTrackerTest, MergeRequiresAccumulation) {
  config.merge_tracks = true;
  EXPECT_TRUE(config::isValid(config));
  config.accumulate_voxels = false;
  EXPECT_FALSE(config::isValid(config));
}

TEST_F(MaxIoUTrackerTest, MergedTrackClaimsAbsorbedIds) {
  config.preassociate_by_id = true;
  for (const bool merge : {false, true}) {
    SCOPED_TRACE(merge);
    config.merge_tracks = merge;
    MaxIoUTracker tracker(config);
    Tracks tracks;
    // Track for upstream id 10, then a second track for id 20 on the same object.
    auto first = frame(1, {{10, 0.0f}});
    tracker.processInput(first, tracks);
    auto second = frame(2, {{10, 0.0f}});
    tracker.processInput(second, tracks);
    auto third = frame(3, {{10, 0.0f}, {20, 0.0f}});
    tracker.processInput(third, tracks);
    auto fourth = frame(4, {{10, 0.0f}, {20, 0.0f}});
    tracker.processInput(fourth, tracks);
    ASSERT_EQ(tracks.size(), merge ? 1u : 2u);
    if (!merge) {
      continue;
    }
    EXPECT_EQ(tracks[0].absorbed_track_ids.size(), 1u);
    EXPECT_EQ(tracks[0].absorbed_instance_ids, std::set<int>({10, 20}));

    // Id 10 shows up again somewhere without geometric overlap: it still goes to the survivor.
    auto fifth = frame(5, {{10, 30.0f}});
    tracker.processInput(fifth, tracks);
    ASSERT_EQ(tracks.size(), 1u);
    EXPECT_EQ(tracks[0].observations.back().semantic_cluster_id, 10);
    EXPECT_EQ(tracks[0].last_seen, 5u);
  }
}

}  // namespace khronos
