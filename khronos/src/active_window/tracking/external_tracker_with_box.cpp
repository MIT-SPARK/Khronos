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

#include "khronos/active_window/tracking/external_tracker_with_box.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>

namespace khronos {

namespace {
// Config is a plain alias for ExternalTracker::Config (see the header), so its declare_config
// overload is reused as-is -- no new one needed here.
static const auto registration =
    config::RegistrationWithConfig<Tracker, ExternalTrackerWithBox, ExternalTrackerWithBox::Config>(
        "ExternalTrackerWithBox");
}  // namespace

ExternalTrackerWithBox::ExternalTrackerWithBox(const Config& config) : ExternalTracker(config) {}

void ExternalTrackerWithBox::updateTrack(const FrameData& data,
                                         const MeasurementCluster& observation,
                                         Track& track) const {
  // Do everything ExternalTracker::updateTrack already does (semantics, observations,
  // confidence)...
  ExternalTracker::updateTrack(data, observation, track);

  // ...then fill in the geometry fields it leaves untouched. Mirrors
  // MaxIoUTracker::updateTrack's kPixels branch (max_iou_tracker.cpp).
  track.last_bounding_box = observation.bounding_box;
  track.last_points.clear();
  track.last_points.reserve(observation.pixels.size());
  for (const Pixel& pixel : observation.pixels) {
    const auto& point = data.input.vertex_map.at<InputData::VertexType>(pixel.v, pixel.u);
    track.last_points.emplace_back(point[0], point[1], point[2]);
  }
}

}  // namespace khronos
