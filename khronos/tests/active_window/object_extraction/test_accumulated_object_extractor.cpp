#include <config_utilities/parsing/yaml.h>
#include <gtest/gtest.h>
#include <khronos/active_window/object_extraction/accumulated_object_extractor.h>
#include <spatial_hash/grid.h>

namespace khronos {
namespace {

// Static track whose voxels (at 0.5 m) are all confirmed after two hits.
Track makeStaticTrack(const GlobalIndexSet& voxels) {
  Track track;
  track.id = 3;
  track.confidence = 1.0f;
  track.first_seen = 10;
  track.last_seen = 20;
  track.semantics = SemanticClusterInfo(7);
  track.accumulateVoxels(voxels, 0.5f, 10, 2);
  track.accumulateVoxels(voxels, 0.5f, 20, 2);
  return track;
}

AccumulatedObjectExtractor makeExtractor(float min_volume = 0.0f, float max_volume = 4.0f) {
  AccumulatedObjectExtractor::Config config;
  config.min_object_volume = min_volume;
  config.max_object_volume = max_volume;
  return AccumulatedObjectExtractor(config);
}

}  // namespace

TEST(AccumulatedObjectExtractor, CopiesConfirmedVoxelsAsPoints) {
  const GlobalIndexSet voxels{GlobalIndex(0, 0, 0), GlobalIndex(1, 0, 0), GlobalIndex(1, 1, 0)};
  Track track = makeStaticTrack(voxels);
  // A voxel hit once is not confirmed and must not be extracted.
  track.accumulateVoxels({GlobalIndex(5, 5, 5)}, 0.5f, 20, 2);
  auto extractor = makeExtractor();

  const auto object = extractor.extractObject(track, FrameDataBuffer({}));
  ASSERT_TRUE(object);
  EXPECT_TRUE(object->bounding_box.dimensions.isApprox(track.accumulated_bounding_box.dimensions));
  EXPECT_TRUE(object->bounding_box.world_P_center.isApprox(
      track.accumulated_bounding_box.world_P_center));
  EXPECT_TRUE(object->position.isApprox(object->bounding_box.world_P_center.cast<double>()));
  EXPECT_EQ(object->mesh.numVertices(), voxels.size());
  EXPECT_EQ(object->mesh.numFaces(), 0u);

  // Vertices are the voxel centers, relative to the bounding box center.
  for (size_t i = 0; i < object->mesh.numVertices(); ++i) {
    const Point world = object->mesh.pos(i) + object->bounding_box.world_P_center;
    const auto index = spatial_hash::indexFromPoint<GlobalIndex>(world, 1.0f / 0.5f);
    EXPECT_TRUE(voxels.count(index));
  }

  EXPECT_EQ(object->semantic_label, 7u);
  EXPECT_EQ(object->first_observed_ns, std::vector<uint64_t>{10});
  EXPECT_EQ(object->last_observed_ns, std::vector<uint64_t>{20});
}

TEST(AccumulatedObjectExtractor, DropsInvalidTracks) {
  const GlobalIndexSet voxels{GlobalIndex(0, 0, 0), GlobalIndex(1, 0, 0)};

  // No confirmed voxels.
  Track unconfirmed;
  unconfirmed.confidence = 1.0f;
  unconfirmed.accumulateVoxels(voxels, 0.5f, 10, 2);
  EXPECT_FALSE(makeExtractor().extractObject(unconfirmed, FrameDataBuffer({})));

  // Low confidence.
  Track low_confidence = makeStaticTrack(voxels);
  low_confidence.confidence = 0.2f;
  EXPECT_FALSE(makeExtractor().extractObject(low_confidence, FrameDataBuffer({})));

  // Volume outside the range: the box is 1.0 x 0.5 x 0.5 = 0.25 m3.
  const Track track = makeStaticTrack(voxels);
  EXPECT_TRUE(makeExtractor(0.2f, 0.3f).extractObject(track, FrameDataBuffer({})));
  EXPECT_FALSE(makeExtractor(0.3f, 4.0f).extractObject(track, FrameDataBuffer({})));
  EXPECT_FALSE(makeExtractor(0.0f, 0.2f).extractObject(track, FrameDataBuffer({})));
}

TEST(AccumulatedObjectExtractor, DynamicTrackKeepsLastState) {
  Track track;
  track.id = 4;
  track.confidence = 1.0f;
  track.is_dynamic = true;
  track.last_seen = 30;
  track.last_centroid = Point(1.0f, 2.0f, 3.0f);
  track.last_bounding_box = BoundingBox(Point::Constant(0.5f), track.last_centroid);

  const auto object = makeExtractor().extractObject(track, FrameDataBuffer({}));
  ASSERT_TRUE(object);
  ASSERT_EQ(object->trajectory_positions.size(), 1u);
  EXPECT_TRUE(object->trajectory_positions[0].isApprox(track.last_centroid));
  EXPECT_EQ(object->trajectory_timestamps, std::vector<uint64_t>{30});
  EXPECT_TRUE(object->bounding_box.world_P_center.isApprox(track.last_centroid));
  EXPECT_EQ(object->mesh.numVertices(), 0u);
}

TEST(AccumulatedObjectExtractor, CreatesFromYaml) {
  const auto node = YAML::Load(
      "{type: AccumulatedObjectExtractor, min_object_allocation_confidence: 0.3, "
      "max_object_volume: 20.0}");
  const auto extractor = config::createFromYaml<ObjectExtractor>(node);
  const auto* derived = dynamic_cast<AccumulatedObjectExtractor*>(extractor.get());
  ASSERT_TRUE(derived);
  EXPECT_FLOAT_EQ(derived->config.min_object_allocation_confidence, 0.3f);
  EXPECT_FLOAT_EQ(derived->config.max_object_volume, 20.0f);
}

}  // namespace khronos
