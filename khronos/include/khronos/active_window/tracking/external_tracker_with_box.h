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

#include "khronos/active_window/tracking/external_tracker.h"

namespace khronos {

/**
 * @brief ExternalTracker variant that additionally populates the track geometry fields
 * (Track::last_bounding_box, Track::last_points) that ExternalTracker itself leaves
 * untouched. Two downstream consumers depend on these being current, and silently misbehave
 * without them:
 * - ActiveWindow::updateTrackingStatus uses last_bounding_box.world_P_center to decide whether
 *   a track has left the spatial map_window. A default-constructed BoundingBox (what
 *   last_bounding_box stays as under plain ExternalTracker) reports world_P_center = (0,0,0),
 *   so the check silently measures the robot's distance from the world origin instead of from
 *   the object -- once the robot is more than map_window.max_radius_m from the origin, every
 *   currently-live track is marked inactive on (almost) every frame regardless of the real
 *   object's position, and gets submitted for extraction (then erased, whether or not
 *   extraction succeeds) after only a handful of observations. Confirmed against a real bag:
 *   every saved track showed last_bounding_box.world_P_center == [0,0,0] and confidence capped
 *   at a handful of observations, with the same numeric (BoxMOT-stable) track id being reborn
 *   and killed repeatedly over the run instead of accumulating one continuous history. See
 *   boxmot_integration_plan.md for the full investigation.
 * - ActiveWindowChangeDetector::computeContainmentRatios skips any track with empty
 *   last_points outright, so AWCD's newly-added-object detection produces zero measurements
 *   for every ExternalTracker track.
 *
 * Kept as a separate registered tracker type (rather than changing ExternalTracker in place)
 * so both behaviors stay independently selectable via active_window.tracker.type -- this is
 * the variant used for A/B testing the fix.
 */
class ExternalTrackerWithBox : public ExternalTracker {
 public:
  // No new config fields: same knobs as ExternalTracker, just registered under a distinct
  // factory name so it can be selected independently.
  using Config = ExternalTracker::Config;

  explicit ExternalTrackerWithBox(const Config& config);
  virtual ~ExternalTrackerWithBox() = default;

 protected:
  void updateTrack(const FrameData& data,
                   const MeasurementCluster& observation,
                   Track& track) const override;
};

}  // namespace khronos
