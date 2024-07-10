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

#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>

#include "khronos/backend/change_detection/ray_verificator.h"
#include "khronos/backend/change_state.h"
#include "khronos/common/common_types.h"

namespace khronos {

/**
 * @brief This module takes a set of presence and absence observations obtained from the ray detctor
 * and filters and assesses them to determine the final change state.
 */

class RayChangeDetector {
 public:
  // Types.
  using Ptr = std::shared_ptr<RayChangeDetector>;
  using ConstPtr = std::shared_ptr<const RayChangeDetector>;

  // Config.
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // Granularity of change detection in seconds.
    float temporal_resolution = 1.f;

    // Number of 'temporal_resolution's to consider when assessing presence/absence.
    size_t window_size = 5;

    // True: Compute confidences as fraction of checked rays. False: Use a the first strong count as
    // signal.
    bool use_relative_confidence = true;

    // Used for majority voting.
    // Percentage [0, 1] or count [1,] of checked rays required to mark a point as absent.
    float absence_confidence = 0.5f;

    // Percentage [0, 1] or count [1,] of checked rays required to mark a point as present.
    float presence_confidence = 0.5f;
  } const config;

  // Construction.
  explicit RayChangeDetector(const Config& config);
  virtual ~RayChangeDetector() = default;

  // Processing.
  struct ChangeResult {
    // The closest timestamp to the current one that has been observed to be absent, if it exists.
    std::optional<uint64_t> closest_absent;

    // The furthest timestamp to the current one that has been observed to be present, if it exists.
    std::optional<uint64_t> furthest_persistent;
  };

  /**
   * @brief Computes the closest time a reliable absence observation has been made, and the furthest
   * before that a reliable presence observation has been made, if they exist.
   * @param check All individual presence and absence observations.
   * @param forward If true, find the closest observations in the future, otherwise in the past.
   * @returns The resulting change times.
   */
  ChangeResult detectChanges(const RayVerificator::CheckResult& check,
                             const bool forward = true) const;

 private:
  // Cached data.
  const uint64_t resolution_ns_;
};

void declare_config(RayChangeDetector::Config& config);

}  // namespace khronos
