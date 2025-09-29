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

#include <hydra/common/global_info.h>

#include "khronos/active_window/data/frame_data.h"
#include "khronos/active_window/tracking/tracker.h"

namespace khronos {

/**
 * @brief Tracker if tracking is performed externally, i.e. all input IDs are already conssitent
 * instance IDs.
 * @note This currently does not handle any dynamic tracks, only semantic ones.
 * @note
 */
class ExternalTracker : public Tracker {
 public:
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // Duration [s] until tracks become deactivated, leaving the active window.
    float temporal_window = 3.f;

    // Number of times a track has to be observed to be considered existent.
    int min_num_observations = 20;
  } const config;

  // Construction.
  explicit ExternalTracker(const Config& config);
  virtual ~ExternalTracker() = default;

  // Inputs.
  void processInput(FrameData& data) override;

 protected:
  // Processing.
  void associateTracks(const FrameData& data);
  void updateTrackingDuration();
  void addNewTrack(const MeasurementCluster& observation);
  void updateTrack(const MeasurementCluster& observation, Track& track) const;

 private:
  TimeStamp processing_stamp_;
};

void declare_config(ExternalTracker::Config& config);

}  // namespace khronos
