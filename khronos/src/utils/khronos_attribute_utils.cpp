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

#include "khronos/utils/khronos_attribute_utils.h"

#include "khronos/utils/geometry_utils.h"

namespace khronos {

Point computeSurfaceCentroid(const KhronosObjectAttributes& attrs) {
  return utils::computeCentroid(attrs.mesh.points);
}

std::optional<TimeStamp> lastAppearedBefore(const KhronosObjectAttributes& attrs,
                                            const TimeStamp query_time) {
  TimeStamp last_appeared = 0;
  bool found = false;
  for (const auto& time : attrs.first_observed_ns) {
    if (time > query_time) {
      break;
    }
    last_appeared = time;
    found = true;
  }
  if (found) {
    return last_appeared;
  } else {
    return std::nullopt;
  }
}

std::optional<TimeStamp> lastDisappearedBefore(const KhronosObjectAttributes& attrs,
                                               const TimeStamp query_time) {
  TimeStamp last_disappeared = 0;
  bool found = false;
  for (const auto& time : attrs.last_observed_ns) {
    if (time > query_time) {
      break;
    }
    last_disappeared = time;
    found = true;
  }
  if (found) {
    return last_disappeared;
  } else {
    return std::nullopt;
  }
}

bool isPresent(const KhronosObjectAttributes& attrs, const TimeStamp query_time) {
  const auto last_appeared = lastAppearedBefore(attrs, query_time);
  if (!last_appeared) {
    return false;
  }
  const auto last_disappeared = lastDisappearedBefore(attrs, query_time);
  if (!last_disappeared) {
    return true;
  }
  return last_appeared.value() > last_disappeared.value();
}

bool hasAppeared(const KhronosObjectAttributes& attrs, const TimeStamp query_time) {
  const auto last_appeared = lastAppearedBefore(attrs, query_time);
  if (!last_appeared) {
    return false;
  }
  const auto last_disappeared = lastDisappearedBefore(attrs, query_time);
  if (!last_disappeared) {
    return last_appeared.value() > 0;
  }
  return last_appeared.value() > 0 && last_appeared.value() > last_disappeared.value();
}

bool hasDisappeared(const KhronosObjectAttributes& attrs, const TimeStamp query_time) {
  const auto last_disappeared = lastDisappearedBefore(attrs, query_time);
  if (!last_disappeared) {
    return false;
  }
  const auto last_appeared = lastAppearedBefore(attrs, query_time);
  if (!last_appeared) {
    return true;
  }
  return last_disappeared.value() > last_appeared.value();
}

void addPresenceDuration(KhronosObjectAttributes& attrs,
                         const TimeStamp t_start,
                         const TimeStamp t_end) {
  // Remove all completely engulfed observations.
  size_t i = 0;
  while (i < attrs.first_observed_ns.size()) {
    if (attrs.first_observed_ns[i] >= t_start && attrs.last_observed_ns[i] <= t_end) {
      attrs.first_observed_ns.erase(attrs.first_observed_ns.begin() + i);
      attrs.last_observed_ns.erase(attrs.last_observed_ns.begin() + i);
    } else {
      i++;
    }
  }

  // Check if t_start intersects an existing observation. This assumes that the observations are
  // valid (no duplicates etc) and sorted.
  int t_start_intersects_index = -1;
  int t_end_intersects_index = -1;
  for (size_t i = 0; i < attrs.first_observed_ns.size(); ++i) {
    if (attrs.first_observed_ns[i] <= t_start && attrs.last_observed_ns[i] >= t_start) {
      t_start_intersects_index = i;
    }
    if (attrs.first_observed_ns[i] <= t_end && attrs.last_observed_ns[i] >= t_end) {
      t_end_intersects_index = i;
    }
  }

  // Apply the needed modifications.
  if (t_end_intersects_index == t_start_intersects_index && t_end_intersects_index != -1) {
    // Fits into a single existing observation, so no changes needed.
    return;
  }

  if (t_start_intersects_index == -1) {
    // No intersection, add new start time.
    attrs.first_observed_ns.insert(
        std::upper_bound(attrs.first_observed_ns.begin(), attrs.first_observed_ns.end(), t_start),
        t_start);
  } else {
    // Intersection, removing the previous end but keep the start.
    attrs.last_observed_ns.erase(attrs.last_observed_ns.begin() + t_start_intersects_index);
  }

  if (t_end_intersects_index == -1) {
    // No intersection, add new end time.
    attrs.last_observed_ns.insert(
        std::upper_bound(attrs.last_observed_ns.begin(), attrs.last_observed_ns.end(), t_end),
        t_end);
  } else {
    // Intersection, removing the previous start but keep the end.
    attrs.first_observed_ns.erase(attrs.first_observed_ns.begin() + t_end_intersects_index);
  }
}

}  // namespace khronos
