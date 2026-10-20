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

#include <optional>
#include <set>
#include <string>
#include <vector>

#include <spatial_hash/hash.h>

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
  Observation(const TimeStamp& stamp,
              int semantic_cluster_id,
              int dynamic_cluster_id,
              const std::string& sensor)
      : stamp(stamp),
        semantic_cluster_id(semantic_cluster_id),
        dynamic_cluster_id(dynamic_cluster_id),
        sensor(sensor) {}

  //! Timestamp of the observation.
  TimeStamp stamp;
  //! ID of the corresponding SemanticCluster, -1 indicates none.
  int semantic_cluster_id = -1;
  //! ID of the corresponding DynamicCluster, -1 indicates none.
  int dynamic_cluster_id = -1;
  //! Sensor ID associated with observation
  std::string sensor;
};

using Observations = std::vector<Observation>;

/**
 * @brief Accumulated evidence of a single voxel of a track.
 */
struct TrackVoxel {
  //! Number of observations whose points fell into this voxel (at most one hit per observation).
  int hits = 0;
  //! Timestamp of the latest observation that hit this voxel.
  TimeStamp last_seen = 0;
};

using TrackVoxels = spatial_hash::IndexHashMap<TrackVoxel>;

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

  // Geometry accumulated over all static observations, at accumulated_voxel_size. Unlike the last_*
  // fields these are never overwritten. Empty if the tracker does not accumulate.
  TrackVoxels accumulated_voxels;
  float accumulated_voxel_size = 0.0f;
  // Hits an accumulated voxel needs to be confirmed, i.e. counted in the bounding box.
  int accumulated_min_hits = 1;
  // AABB over the confirmed accumulated voxels (see accumulateVoxels). Invalid until the first
  // voxel is confirmed.
  BoundingBox accumulated_bounding_box;

  // Ids of all tracks merged into this one (see absorb), transitively.
  std::set<int> absorbed_track_ids;
  // Upstream instance ids (semantic_cluster_id, e.g. BoxMOT ids) the merged tracks were last
  // observed with (this track's included), so later detections with those ids can still associate.
  std::set<int> absorbed_instance_ids;

  //! Semantic information associated with the track
  std::optional<SemanticClusterInfo> semantics;
  size_t num_features = 0;

  // Whether this object is dynamic.
  bool is_dynamic = false;

  // Probability estimate [0-1] the object exists.
  float confidence = 0.0f;

  // Whether this object is still active.
  bool is_active = true;

  //! Tracker pass the track was last updated in
  std::optional<uint32_t> sequence_number;

  void updateSemantics(const std::optional<SemanticClusterInfo>& other_semantics);

  /**
   * @brief Add one observation's unique voxels to the accumulated geometry, one hit each. The
   * bounding box grows by each voxel once its hits reach min_hits, so it is exactly the AABB of
   * the confirmed voxels. voxel_size and min_hits must be the same for every call.
   */
  void accumulateVoxels(const GlobalIndexSet& voxels,
                        float voxel_size,
                        TimeStamp stamp,
                        int min_hits);

  /**
   * @brief Accumulated voxels with at least accumulated_min_hits hits (what the bounding box covers).
   */
  GlobalIndexSet confirmedVoxels() const;

  /**
   * @brief Recompute accumulated_bounding_box from the confirmed accumulated voxels.
   */
  void recomputeAccumulatedBoundingBox();

  /**
   * @brief Merge another track of the same object into this one, keeping this id and recording the
   * other's ids in absorbed_*. Observations and voxel hits are combined; last_* come from whichever
   * track was seen last. `other` is left in a valid but unspecified state.
   */
  void absorb(Track&& other);

  /**
   * @brief Upstream instance id (semantic_cluster_id) of the most recent observation that has one.
   */
  std::optional<int> latestInstanceId() const;
};

using Tracks = std::vector<Track>;

}  // namespace khronos
