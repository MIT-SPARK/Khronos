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

#include <optional>

#include <khronos/common/common_types.h>

namespace khronos {

Point computeSurfaceCentroid(const KhronosObjectAttributes& attrs);

/**
 * @brief Get the latest time the object appeared before query time, if such a time exists.
 * Assumes that first/last seen stamps are sorted.
 * @param attrs Object attributes to check.
 * @param query_time Time to check for.
 * @return The latest time the object appeared before query time, if such a time exists.
 */
std::optional<TimeStamp> lastAppearedBefore(const KhronosObjectAttributes& attrs,
                                            const TimeStamp query_time);

/**
 * @brief Get the latest time the object disappeared before query time, if such a time exists.
 * Assumes that first/last seen stamps are sorted.
 * @param attrs Object attributes to check.
 * @param query_time Time to check for.
 * @return The latest time the object disappeared before query time, if such a time exists.
 */
std::optional<TimeStamp> lastDisappearedBefore(const KhronosObjectAttributes& attrs,
                                               const TimeStamp query_time);

/**
 * @brief Check if object is considered present at query time. Assumes that first/last seen stamps
 * are sorted.
 * @param attrs Object attributes to check.
 * @param query_time Time to check for.
 * @return True if object is present at query time.
 */
bool isPresent(const KhronosObjectAttributes& attrs, const TimeStamp query_time);

/**
 * @brief Check if object has appeared at query time. Assumes that first/last seen stamps
 * are sorted. A first seen value of 0 assumes the object was always present and does not count as
 * appeared.
 * @param attrs Object attributes to check.
 * @param query_time Time to check for.
 * @return True if object has appeared before query time.
 */
bool hasAppeared(const KhronosObjectAttributes& attrs, const TimeStamp query_time);

/**
 * @brief Check if object has disappeared at query time. Assumes that first/last seen stamps
 * are sorted.
 * @param attrs Object attributes to check.
 * @param query_time Time to check for.
 * @return True if object has disappeared before query time.
 */
bool hasDisappeared(const KhronosObjectAttributes& attrs, const TimeStamp query_time);

/**
 * @brief Add a presence duration from t_start to t_end to the object attributes' presence time,
 * represented by the last/first seen stamps. This assumes the time stamps in attrs are sorted and
 * the resulting new time stamps will also be sorted.
 * @param attrs Object attributes to add presence duration to.
 * @param t_start Start of presence duration.
 * @param t_end End of presence duration.
 */
void addPresenceDuration(KhronosObjectAttributes& attrs,
                         const TimeStamp t_start,
                         const TimeStamp t_end);
}  // namespace khronos
