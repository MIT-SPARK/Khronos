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

#pragma once

#include <map>
#include <vector>

#include "khronos/active_window/data/measurement_clusters.h"
#include "khronos/common/common_types.h"

namespace khronos {

/**
 * @brief Data structure for a single observation of an object.
 */
struct Observation {
  // Construction.
  Observation() = default;
  explicit Observation(const TimeStamp& stamp) : stamp(stamp) {}
  Observation(const TimeStamp& stamp, int semantic_cluster_id, int dynamic_cluster_id)
      : stamp(stamp),
        semantic_cluster_id(semantic_cluster_id),
        dynamic_cluster_id(dynamic_cluster_id) {}

  // Timestamp of the observation.
  TimeStamp stamp;

  // ID of the corresponding SemanticCluster, -1 indicates none.
  int semantic_cluster_id = -1;

  // ID of the corresponding DynamicCluster, -1 indicates none.
  int dynamic_cluster_id = -1;
};
using Observations = std::vector<Observation>;

/**
 * @brief Data structure to track objects and associations throughout the active window.
 */
struct Track {
  // ID of the track.
  int id;

  // Last observation timestamp.
  TimeStamp last_seen;

  // First observation timestamp.
  TimeStamp first_seen;

  // All timestamps of frames in which this object was observed and the corresponding
  // cluster ID in the respective image.
  Observations observations;

  // Most recent data stored for tracking. These will be adaptively set by the tracker as needed.
  BoundingBox last_bounding_box;
  GlobalIndexSet last_voxels;
  Points last_points;
  float last_voxel_size;

  // Centroid of the observation, used for dynamic tracking.
  Point last_centroid;

  //! Semantic information associated with the track
  std::optional<SemanticClusterInfo> semantics;
  size_t num_features = 0;

  // Whether this object is dynamic.
  bool is_dynamic = false;

  // Probability estimate [0-1] the object exists.
  float confidence;

  // Whether this object is still active.
  bool is_active = true;

  void updateSemantics(const std::optional<SemanticClusterInfo>& other_semantics);
};
using Tracks = std::vector<Track>;

}  // namespace khronos
