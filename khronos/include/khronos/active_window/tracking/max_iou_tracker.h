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

#include <memory>
#include <string>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>
#include <spatial_hash/grid.h>

#include "khronos/active_window/data/frame_data.h"
#include "khronos/active_window/data/track.h"
#include "khronos/active_window/tracking/tracker.h"
#include "khronos/common/common_types.h"

namespace khronos {

/**
 * @brief Simple tracker that performs frame to frame tracking byassociating objects to
 * the highest IoU between bounding boxes if a minimum IoU is met.
 */
// TODO(lschmid): Double check the tracking by pixels works as intended after the changes, might be
// something funky there.
class MaxIoUTracker : public Tracker {
 public:
  // Config.
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // Semantic association method
    enum class SemanticAssociation {
      kAssignCluster,
      kAssignTrack
    } semantic_association = SemanticAssociation::kAssignCluster;

    // Minimum IoU to consider two semantic detections a match.
    float min_semantic_iou = 0.5f;

    // Minimum cosine similarity of the semantic features
    float min_cosine_sim = 0.0f;

    // Minimum IoU to consider a semantic and a dynamic detections a match.
    float min_cross_iou = 0.5f;

    // Allows the dynamic object to move at mostthis distance [m] between frames.
    float max_dynamic_distance = 1.f;

    // Duration [s] until tracks become deactivated, leaving the active window.
    float temporal_window = 3.f;

    // Number of times a track has to be observed to be considered existent.
    int min_num_observations = 20;

    // Which representation to track between frames [pixels, voxels, bounding_box].
    enum class TrackBy { kPixels, kVoxels, kBouningBox } track_by = TrackBy::kPixels;

    // Voxel size in meters used for tracking. Only used if 'track_by' is 'voxels'.
    float voxel_size = 0.1f;
  } const config;

  // Construction.
  explicit MaxIoUTracker(const Config& config);
  virtual ~MaxIoUTracker() = default;

  // Inputs.
  void processInput(FrameData& data) override;

  // Processing.
  void setup();
  void setupTrackMeasurements(FrameData& data) const;
  void associateTracks(const FrameData& data);
  void associateSemanticTracks(const FrameData& data);
  void associateDynamicTracks(const FrameData& data);
  void assignStaticTracksToCluster(const FrameData& data,
                                   std::unordered_set<int>& associated_objects);
  void assignClustersToStaticTrack(const FrameData& data,
                                   std::unordered_set<int>& associated_objects);
  void updateTrackingDuration();
  Track& addNewTrack(const FrameData& data, const MeasurementCluster& observation, bool is_dynamic);
  void updateTrack(const FrameData& data,
                   const MeasurementCluster& observation,
                   Track& track,
                   bool is_observation_dynamic) const;

  // Track by type specific functions.
  // Function pointers that call the right function for the track by mode.
  std::function<void(const FrameData&, MeasurementCluster&)> setupTrackMeasurement;
  std::function<float(const FrameData&, const MeasurementCluster&, const Track&)> computeIoU;
  void setupTrackMeasurementVoxels(const FrameData& data, MeasurementCluster& cluster) const;
  float computeIoUVoxels(const FrameData& data,
                         const MeasurementCluster& cluster,
                         const Track& track) const;
  float computeIoUPixels(const FrameData& data,
                         const MeasurementCluster& cluster,
                         const Track& track) const;
  float computeIoUBoundingBox(const FrameData& data,
                              const MeasurementCluster& cluster,
                              const Track& track) const;
  Point computeCentroid(const FrameData& data, const MeasurementCluster& cluster) const;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<Tracker, MaxIoUTracker, MaxIoUTracker::Config>(
          "MaxIouTracker");

  // Members.
  const spatial_hash::Grid<GlobalIndex> grid_;

  // Variables.
  TimeStamp processing_stamp_;
  int current_track_id_ = 0;  // TODO(lschmid): at some point reuse IDs.
};

void declare_config(MaxIoUTracker::Config& config);

}  // namespace khronos
