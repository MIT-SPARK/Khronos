/** -----------------------------------------------------------------------------
 * Copyright (c) 2024 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:      Lukas Schmid <lschmid@mit.edu>, Marcus Abate <mabate@mit.edu>,
 *               Yun Chang <yunchang@mit.edu>, Luca Carlone <lcarlone@mit.edu>
 * AFFILIATION:  MIT SPARK Lab, Massachusetts Institute of Technology
 * YEAR:         2024
 * SOURCE:       https://github.com/MIT-SPARK/Khronos
 * LICENSE:      BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

// Round-trip check for Track::save / Track::load (khronos/active_window/data/track.h). Builds a
// Track with representative values for every field the tracker consumes (see MaxIoUTracker in
// max_iou_tracker.cpp), writes it to a temp file, reloads it, and compares field-by-field. This
// is the minimal validation for Step 1 of the tracker-debugging pipeline (see tracker_debug.md)
// and the exact input format Step 2's replay harness consumes.

#include <gtest/gtest.h>

#include <filesystem>

#include "khronos/active_window/data/track.h"

namespace khronos {
namespace {

Track makeSampleTrack() {
  Track track;
  track.id = 42;
  track.first_seen = 1000;
  track.last_seen = 1066;
  track.is_dynamic = false;
  track.confidence = 0.75f;
  track.num_features = 3;

  track.observations = {Observation(1000, 5, -1, "camera_left"),
                        Observation(1033, 6, -1, "camera_left"),
                        Observation(1066, 7, -1, "camera_right")};

  track.last_points = {Point(1.0f, 2.0f, 3.0f), Point(4.0f, 5.0f, 6.0f), Point(-1.5f, 0.0f, 2.25f)};

  track.last_voxels = {GlobalIndex(1, 2, 3), GlobalIndex(-4, 5, -6), GlobalIndex(0, 0, 0)};
  track.last_voxel_size = 0.1f;

  track.last_centroid = Point(1.1f, 2.2f, 3.3f);

  track.last_bounding_box =
      spark_dsg::BoundingBox(Eigen::Vector3f(2.0f, 3.0f, 1.0f), Eigen::Vector3f(0.5f, 0.5f, 0.5f));

  FeatureVector feature(4);
  feature << 0.1f, 0.2f, 0.3f, 0.4f;
  track.semantics = SemanticClusterInfo(7, feature);

  return track;
}

}  // namespace

TEST(TrackSerialization, RoundTripPreservesAllFields) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "khronos_track_roundtrip_test.json";

  const Track original = makeSampleTrack();
  original.save(tmp_path.string());
  const Track reloaded = Track::load(tmp_path.string());
  std::filesystem::remove(tmp_path);

  EXPECT_EQ(original.id, reloaded.id);
  EXPECT_EQ(original.first_seen, reloaded.first_seen);
  EXPECT_EQ(original.last_seen, reloaded.last_seen);
  EXPECT_EQ(original.is_dynamic, reloaded.is_dynamic);
  EXPECT_FLOAT_EQ(original.confidence, reloaded.confidence);
  EXPECT_EQ(original.num_features, reloaded.num_features);

  ASSERT_EQ(original.observations.size(), reloaded.observations.size());
  for (size_t i = 0; i < original.observations.size(); ++i) {
    SCOPED_TRACE("observations[" + std::to_string(i) + "]");
    const auto& lhs = original.observations[i];
    const auto& rhs = reloaded.observations[i];
    EXPECT_EQ(lhs.stamp, rhs.stamp);
    EXPECT_EQ(lhs.semantic_cluster_id, rhs.semantic_cluster_id);
    EXPECT_EQ(lhs.dynamic_cluster_id, rhs.dynamic_cluster_id);
    EXPECT_EQ(lhs.sensor, rhs.sensor);
  }

  ASSERT_EQ(original.last_points.size(), reloaded.last_points.size());
  for (size_t i = 0; i < original.last_points.size(); ++i) {
    SCOPED_TRACE("last_points[" + std::to_string(i) + "]");
    EXPECT_LT((original.last_points[i] - reloaded.last_points[i]).norm(), 1e-5f);
  }

  EXPECT_EQ(original.last_voxels, reloaded.last_voxels);
  EXPECT_FLOAT_EQ(original.last_voxel_size, reloaded.last_voxel_size);
  EXPECT_LT((original.last_centroid - reloaded.last_centroid).norm(), 1e-5f);
  EXPECT_EQ(original.last_bounding_box, reloaded.last_bounding_box);

  ASSERT_EQ(original.semantics.has_value(), reloaded.semantics.has_value());
  ASSERT_TRUE(reloaded.semantics.has_value());
  EXPECT_EQ(original.semantics->category_id, reloaded.semantics->category_id);
  EXPECT_LT((original.semantics->feature - reloaded.semantics->feature).norm(), 1e-5f);
}

}  // namespace khronos
