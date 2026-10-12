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

#include "khronos/active_window/data/track.h"

#include <algorithm>

#include <spatial_hash/grid.h>

namespace khronos {

void Track::accumulateVoxels(const GlobalIndexSet& voxels,
                             float voxel_size,
                             TimeStamp stamp,
                             int min_hits) {
  accumulated_voxel_size = voxel_size;
  accumulated_min_hits = min_hits;
  const Eigen::Vector3f voxel_dims = Eigen::Vector3f::Constant(voxel_size);
  for (const GlobalIndex& index : voxels) {
    auto& voxel = accumulated_voxels[index];
    ++voxel.hits;
    voxel.last_seen = stamp;
    if (voxel.hits == min_hits) {
      // The voxel just got confirmed: grow the bounding box by its cube.
      accumulated_bounding_box.merge(
          BoundingBox(voxel_dims, spatial_hash::centerPointFromIndex(index, voxel_size)));
    }
  }
}

GlobalIndexSet Track::confirmedVoxels() const {
  GlobalIndexSet confirmed;
  for (const auto& [index, voxel] : accumulated_voxels) {
    if (voxel.hits >= accumulated_min_hits) {
      confirmed.insert(index);
    }
  }
  return confirmed;
}

void Track::recomputeAccumulatedBoundingBox() {
  accumulated_bounding_box = BoundingBox();
  const Eigen::Vector3f voxel_dims = Eigen::Vector3f::Constant(accumulated_voxel_size);
  for (const auto& [index, voxel] : accumulated_voxels) {
    if (voxel.hits >= accumulated_min_hits) {
      accumulated_bounding_box.merge(BoundingBox(
          voxel_dims, spatial_hash::centerPointFromIndex(index, accumulated_voxel_size)));
    }
  }
}

std::optional<int> Track::latestInstanceId() const {
  for (auto it = observations.rbegin(); it != observations.rend(); ++it) {
    if (it->semantic_cluster_id >= 0) {
      return it->semantic_cluster_id;
    }
  }
  return std::nullopt;
}

void Track::absorb(Track&& other) {
  // Provenance and upstream ids of both tracks: after merging the observations, the latest
  // observation of this track may be the other's, which would shadow this track's own latest id.
  absorbed_track_ids.insert(other.id);
  absorbed_track_ids.insert(other.absorbed_track_ids.begin(), other.absorbed_track_ids.end());
  for (const auto& instance_id : {latestInstanceId(), other.latestInstanceId()}) {
    if (instance_id) {
      absorbed_instance_ids.insert(*instance_id);
    }
  }
  absorbed_instance_ids.insert(other.absorbed_instance_ids.begin(),
                               other.absorbed_instance_ids.end());

  // The latest-observation state comes from whichever track was seen last.
  if (other.last_seen > last_seen) {
    last_bounding_box = std::move(other.last_bounding_box);
    last_voxels = std::move(other.last_voxels);
    last_points = std::move(other.last_points);
    last_voxel_size = other.last_voxel_size;
    last_centroid = other.last_centroid;
  }
  first_seen = std::min(first_seen, other.first_seen);
  last_seen = std::max(last_seen, other.last_seen);

  observations.insert(observations.end(), other.observations.begin(), other.observations.end());
  std::stable_sort(observations.begin(),
                   observations.end(),
                   [](const Observation& a, const Observation& b) { return a.stamp < b.stamp; });

  // Both tracks observed the object at different times, so their hits add up.
  if (accumulated_voxels.empty()) {
    accumulated_voxel_size = other.accumulated_voxel_size;
    accumulated_min_hits = other.accumulated_min_hits;
  }
  for (const auto& [index, other_voxel] : other.accumulated_voxels) {
    auto& voxel = accumulated_voxels[index];
    voxel.hits += other_voxel.hits;
    voxel.last_seen = std::max(voxel.last_seen, other_voxel.last_seen);
  }
  recomputeAccumulatedBoundingBox();

  updateSemantics(other.semantics);
  // Confidence counts observations, so it adds up too.
  confidence = std::min(confidence + other.confidence, 1.0f);
  if (other.sequence_number &&
      (!sequence_number || *other.sequence_number > *sequence_number)) {
    sequence_number = other.sequence_number;
  }
}

void Track::updateSemantics(const std::optional<SemanticClusterInfo>& other) {
  if (!other) {
    return;
  }

  if (!semantics) {
    semantics = other;
    return;
  }

  const bool other_has_feature = other->feature.size() > 0;
  const bool has_feature = semantics->feature.size() > 0;
  if (has_feature && !other_has_feature) {
    return;  // prefer to keep feature
  }

  if (other_has_feature && !has_feature) {
    semantics->feature = other->feature;
    ++num_features;
    return;
  }

  // incremental mean
  const auto total_features = static_cast<float>(num_features + 1);
  const auto prev_weight = static_cast<float>(num_features) / total_features;
  const auto new_weight = 1.0f / total_features;
  semantics->feature = prev_weight * semantics->feature + new_weight * other->feature;
  ++num_features;
}

}  // namespace khronos
