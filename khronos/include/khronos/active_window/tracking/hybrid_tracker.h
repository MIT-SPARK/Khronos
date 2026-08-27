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

#include <unordered_set>

#include "khronos/active_window/tracking/max_iou_tracker.h"

namespace khronos {

/**
 * @brief Tracker that trusts an upstream external tracker's (BoxMOT's) instance ids first, and
 * only falls back to MaxIoUTracker-style geometric association for tracks/clusters the trust
 * pass couldn't resolve this frame.
 *
 * Motivation: ExternalTracker (exact-id-match only) has no way to reconnect a track when BoxMOT
 * loses its id (occlusion, brief leaving-frame, a 2D-tracker association failure) and re-detects
 * the object later under a new id -- it just mints a new track, fragmenting one physical object
 * into several. MaxIoUTracker already solves this re-entry problem via geometric
 * re-association, but on its own is pose-drift-sensitive as the *primary* signal (see
 * boxmot_integration_plan.md). HybridTracker combines both: BoxMOT's id while it holds (immune
 * to pose drift), geometry (default: voxel overlap, see the deployed config) only as the
 * fallback for re-entry.
 *
 * Built as a MaxIoUTracker subclass (rather than owning a separate MaxIoUTracker instance)
 * because every method the trust pass needs to hand off to (updateTrack, associateTracks,
 * setupTrackMeasurements) is already public on MaxIoUTracker, and MaxIoUTracker::updateTrack
 * already populates Track::last_bounding_box/last_points/last_voxels correctly per
 * config.track_by -- unlike ExternalTracker, which needed ExternalTrackerWithBox as a bolt-on
 * fix for exactly that. Reusing MaxIoUTracker's own current_track_id_ counter (via its
 * inherited addNewTrack) also means there is only ever one id-minting authority -- no collision
 * risk between a trust-resolved and a geometry-created track. One consequence: khronos track
 * ids under HybridTracker are NOT raw BoxMOT ids (unlike ExternalTracker/ExternalTrackerWithBox,
 * which set track.id = observation.id directly).
 *
 * Trust applies to dynamic tracks too, not just static ones (unlike ExternalTracker, which is
 * documented as not handling dynamic tracks at all): khronos' is_dynamic is the motion
 * detector's classification of a cluster, orthogonal to whether BoxMOT can hold a stable id on
 * it -- BoxMOT ids are just as trustworthy for a moving object as for a static one.
 */
class HybridTracker : public MaxIoUTracker {
 public:
  // No new fields: the trust pass reuses MaxIoUTracker::updateTrack's existing confidence
  // formula and min_num_observations, and the geometric fallback is the inherited MaxIoUTracker
  // association logic verbatim, so every existing knob (track_by, voxel_size, min_semantic_iou,
  // min_cross_iou, semantic_association, bbox_type) already means the right thing.
  using Config = MaxIoUTracker::Config;

  explicit HybridTracker(const Config& config);
  virtual ~HybridTracker() = default;

  // Inputs.
  void processInput(FrameData& data, Tracks& tracks) override;

 protected:
  /**
   * @brief Exact-id trust pass: for each track with a recorded last-known BoxMOT id (static
   * tracks via Observation::semantic_cluster_id, dynamic tracks via
   * Observation::dynamic_cluster_id), look for a cluster with that same id this frame (in
   * data.semantic_clusters or data.dynamic_clusters respectively) and update the track directly
   * if found -- immune to the pose-drift issues of reprojection-based IoU while the id holds.
   * Tracks/clusters left unresolved here fall through to the inherited, seeded
   * MaxIoUTracker::associateTracks() geometric fallback pass called right after this in
   * processInput().
   * @param resolved_dynamic_cluster_ids[out] Dynamic-cluster ids claimed by this pass.
   * @param resolved_semantic_cluster_ids[out] Semantic-cluster ids claimed by this pass.
   * Kept as two separate sets (not one shared set): MaxIoUTracker::associateDynamicTracks/
   * associateSemanticTracks each assume every id in their seed is actually a member of the
   * cluster list they iterate -- mixing the two silently corrupts that bookkeeping (see the
   * associateTracks() seeded-overload comment in max_iou_tracker.h).
   */
  void trustPass(const FrameData& data,
                Tracks& tracks,
                std::unordered_set<int>& resolved_dynamic_cluster_ids,
                std::unordered_set<int>& resolved_semantic_cluster_ids);
};

}  // namespace khronos
