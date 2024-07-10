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

#include "khronos/backend/change_detection/ray_change_detector.h"

namespace khronos {

void declare_config(RayChangeDetector::Config& config) {
  using namespace config;
  name("RayChangeDetector");
  field(config.verbosity, "verbosity");
  field(config.temporal_resolution, "temporal_resolution", "s");
  field(config.window_size, "window_size");
  field(config.use_relative_confidence, "use_relative_confidence");
  field(config.absence_confidence, "absence_confidence");
  field(config.presence_confidence, "presence_confidence");

  check(config.temporal_resolution, GT, 0.f, "temporal_resolution");
  check(config.window_size, GT, 0, "window_size");
  if (config.use_relative_confidence) {
    checkInRange(config.absence_confidence, 0.f, 1.f, "absence_confidence");
    checkInRange(config.presence_confidence, 0.f, 1.f, "presence_confidence");
  } else {
    check(config.absence_confidence, GT, 0.f, "absence_confidence");
    check(config.presence_confidence, GT, 0.f, "presence_confidence");
  }
}

RayChangeDetector::RayChangeDetector(const Config& config)
    : config(config::checkValid(config)), resolution_ns_(config.temporal_resolution * 1e9) {}

RayChangeDetector::ChangeResult RayChangeDetector::detectChanges(
    const RayVerificator::CheckResult& check,
    const bool forward) const {
  // Compute a discretized time series of presence/absence observations. Only consider changes as
  // meaningful if a meaningful majority within the window_size points toward the same result.

  // Build time series of observations. series[time_index] = {num_present, num_absent}
  std::unordered_map<size_t, std::pair<uint, uint>> time_series;
  for (uint64_t timestamp : check.present) {
    const size_t time_index = timestamp / resolution_ns_;
    time_series[time_index].first++;
  }
  for (uint64_t timestamp : check.absent) {
    const size_t time_index = timestamp / resolution_ns_;
    time_series[time_index].second++;
  }

  // Build a directional list of indices to iterate through.
  std::vector<size_t> time_indices;
  time_indices.reserve(time_series.size());
  for (const auto& [time_index, _] : time_series) {
    time_indices.push_back(time_index);
  }
  if (forward) {
    std::sort(time_indices.begin(), time_indices.end());
  } else {
    std::sort(time_indices.begin(), time_indices.end(), std::greater<size_t>());
  }

  // Iterate through the time series and find the closest absent and furthest persistent
  // observations.
  ChangeResult result;
  for (const size_t time_index : time_indices) {
    uint num_present = 0;
    uint num_absent = 0;
    for (size_t i = 0; i < config.window_size; ++i) {
      const auto it = time_series.find(time_index + i);
      if (it != time_series.end()) {
        num_present += it->second.first;
        num_absent += it->second.second;
      }
    }

    // Compute confidence and take decision.
    if (config.use_relative_confidence) {
      const float absence_confidence = num_absent / static_cast<float>(num_present + num_absent);
      if (absence_confidence > config.absence_confidence) {
        result.closest_absent = time_index * resolution_ns_;
        // Once an absence is found, we can stop looking for earlier persistent observations.
        return result;
      }
      if (1.f - absence_confidence > config.presence_confidence) {
        result.furthest_persistent = time_index * resolution_ns_;
      }
    } else {
      if (num_absent > config.absence_confidence) {
        result.closest_absent = time_index * resolution_ns_;
        // Once an absence is found, we can stop looking for earlier persistent observations.
        return result;
      }
      if (num_present > config.presence_confidence) {
        result.furthest_persistent = time_index * resolution_ns_;
      }
    }
  }

  return result;
}

}  // namespace khronos
