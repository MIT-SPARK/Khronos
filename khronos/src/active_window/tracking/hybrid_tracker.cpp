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

#include "khronos/active_window/tracking/hybrid_tracker.h"

#include <unordered_map>

#include <config_utilities/config.h>
#include <config_utilities/factory.h>

namespace khronos {

namespace {
// Config is a plain alias for MaxIoUTracker::Config (see the header), so its declare_config
// overload is reused as-is -- no new one needed here.
static const auto registration =
    config::RegistrationWithConfig<Tracker, HybridTracker, HybridTracker::Config>("HybridTracker");
}  // namespace

HybridTracker::HybridTracker(const Config& config) : MaxIoUTracker(config) {}

void HybridTracker::processInput(FrameData& data, Tracks& tracks) {
  processing_stamp_ = data.input.timestamp_ns;
  Timer timer("tracking/all", processing_stamp_);

  // Compute the entities the objects are going to be tracked by (pixels, voxels, or bounding
  // box) -- same as MaxIoUTracker::processInput, needed regardless of which clusters end up
  // trust-resolved.
  setupTrackMeasurements(data);

  // Trust pass: exact BoxMOT-id match, for every track (dynamic and static alike).
  std::unordered_set<int> resolved_dynamic_cluster_ids;
  std::unordered_set<int> resolved_semantic_cluster_ids;
  trustPass(data, tracks, resolved_dynamic_cluster_ids, resolved_semantic_cluster_ids);

  // Geometric fallback (inherited, seeded): dynamic tracks then static tracks, skipping
  // clusters the trust pass already claimed and tracks it already updated this frame.
  associateTracks(data, tracks, resolved_dynamic_cluster_ids, resolved_semantic_cluster_ids);
}

void HybridTracker::trustPass(const FrameData& data,
                              Tracks& tracks,
                              std::unordered_set<int>& resolved_dynamic_cluster_ids,
                              std::unordered_set<int>& resolved_semantic_cluster_ids) {
  std::unordered_map<int, const MeasurementCluster*> semantic_by_id;
  for (const auto& cluster : data.semantic_clusters) {
    semantic_by_id[cluster.id] = &cluster;
  }
  std::unordered_map<int, const MeasurementCluster*> dynamic_by_id;
  for (const auto& cluster : data.dynamic_clusters) {
    dynamic_by_id[cluster.id] = &cluster;
  }

  for (Track& track : tracks) {
    if (track.observations.empty()) {
      continue;
    }

    const Observation& last_obs = track.observations.back();
    const int last_id = track.is_dynamic ? last_obs.dynamic_cluster_id : last_obs.semantic_cluster_id;
    if (last_id < 0) {
      // No BoxMOT id recorded last time (shouldn't happen once a track has been updated at
      // least once, but guard defensively).
      continue;
    }

    const auto& clusters_by_id = track.is_dynamic ? dynamic_by_id : semantic_by_id;
    const auto it = clusters_by_id.find(last_id);
    if (it == clusters_by_id.end()) {
      // BoxMOT lost this id this frame -- leave for the geometric fallback pass.
      continue;
    }

    if (track.is_dynamic) {
      resolved_dynamic_cluster_ids.insert(last_id);
    } else {
      resolved_semantic_cluster_ids.insert(last_id);
    }
    updateTrack(data, *it->second, track, track.is_dynamic);
    if (track.is_dynamic) {
      // Mirrors MaxIoUTracker::associateDynamicTracks, which also refreshes last_centroid on
      // every dynamic-track update (used for the next frame's nearest-centroid fallback).
      track.last_centroid = computeCentroid(data, *it->second);
    }
  }
}

}  // namespace khronos
