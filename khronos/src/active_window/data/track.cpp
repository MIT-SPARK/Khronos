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
