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

#include "khronos/active_window/tracking/external_tracker.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>

#include "khronos/utils/geometry_utils.h"

namespace khronos {

namespace {
static const auto registration =
    config::RegistrationWithConfig<Tracker, ExternalTracker, ExternalTracker::Config>(
        "ExternalTracker");

}  // namespace

void declare_config(ExternalTracker::Config& config) {
  using namespace config;
  name("ExternalTracker");
  field(config.verbosity, "verbosity");
  field(config.temporal_window, "temporal_window", "s");
  field(config.min_num_observations, "min_num_observations", "frames");
  check(config.temporal_window, GT, 0.f, "temporal_window");
}

ExternalTracker::ExternalTracker(const Config& config) : config(config::checkValid(config)) {}

void ExternalTracker::processInput(FrameData& data) {
  processing_stamp_ = data.input.timestamp_ns;
  Timer timer("tracking/all", processing_stamp_);

  // Compute the bounding boxes for all clusters for visualization and extent.
  for (auto& cluster : data.semantic_clusters) {
    cluster.bounding_box =
        BoundingBox(utils::VertexMapAdaptor(cluster.pixels, data.input.vertex_map));
  }

  // Associate current objects to existing tracks and create new tracks for
  // unassociated objects.
  associateTracks(data);

  // Update which tracks are still active. Tracks labeled inactive will be removed by
  // the active window.
  updateTrackingDuration();
}

void ExternalTracker::associateTracks(const FrameData& data) {
  // Associate tracks exactly if the IDs are the same.
  std::unordered_set<int> associated_objects;
  for (Track& track : tracks_) {
    if (track.is_dynamic) {
      continue;
    }

    for (const auto& cluster : data.semantic_clusters) {
      if (associated_objects.find(cluster.id) != associated_objects.end()) {
        // Duplicate cluster id. This should not happen for 1-to-1 tracking, we'll skip for now.
        continue;
      }

      if (track.id == cluster.id) {
        // Track and cluster IDs match, associate them.
        associated_objects.insert(cluster.id);
        updateTrack(cluster, track);
        break;
      }
    }
  }

  // Create new tracks for unassociated objects.
  for (const auto& cluster : data.semantic_clusters) {
    if (associated_objects.find(cluster.id) == associated_objects.end()) {
      addNewTrack(cluster);
    }
  }
}

void ExternalTracker::addNewTrack(const MeasurementCluster& observation) {
  auto& track = tracks_.emplace_back();
  track.is_dynamic = false;
  track.id = observation.id;
  track.first_seen = processing_stamp_;
  updateTrack(observation, track);
}

void ExternalTracker::updateTrack(const MeasurementCluster& observation, Track& track) const {
  // Simple existence probability estimate: count number of observations. We multiply by
  // two so that the minimum observations yield 50% confidence.
  track.updateSemantics(observation.semantics);

  track.last_seen = processing_stamp_;
  track.observations.emplace_back(processing_stamp_, observation.id, -1);
  track.confidence = std::min(
      static_cast<float>(track.observations.size()) / (config.min_num_observations * 2), 1.f);
}

void ExternalTracker::updateTrackingDuration() {
  // Label tracks that exit the temporal window as inactive.
  const TimeStamp min_time = processing_stamp_ - fromSeconds(config.temporal_window);
  for (Track& track : tracks_) {
    track.is_active = track.last_seen >= min_time;
  }
}

}  // namespace khronos
