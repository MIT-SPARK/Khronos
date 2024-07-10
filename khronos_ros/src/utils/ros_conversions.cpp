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

#include "khronos_ros/utils/ros_conversions.h"

namespace khronos {

ObjectChange fromMsg(const khronos_msgs::ObjectChange& msg) {
  ObjectChange result;
  result.node_id = msg.node_id;
  result.merged_id = msg.merged_id;
  result.first_absent = msg.first_absent;
  result.last_absent = msg.last_absent;
  result.first_persistent = msg.first_persistent;
  result.last_persistent = msg.last_persistent;
  return result;
}

khronos_msgs::ObjectChange toMsg(const ObjectChange& object_change) {
  khronos_msgs::ObjectChange msg;
  msg.node_id = object_change.node_id;
  msg.merged_id = object_change.merged_id;
  msg.first_absent = object_change.first_absent;
  msg.last_absent = object_change.last_absent;
  msg.first_persistent = object_change.first_persistent;
  return msg;
}

Changes fromMsg(const khronos_msgs::Changes& msg) {
  Changes changes;
  changes.object_changes.reserve(msg.objects.size());
  for (const auto& object_change_msg : msg.objects) {
    changes.object_changes.emplace_back(fromMsg(object_change_msg));
  }
  changes.background_changes.reserve(msg.background.size());
  for (const auto& background_change_msg : msg.background) {
    changes.background_changes.emplace_back(static_cast<ChangeState>(background_change_msg));
  }
  return changes;
}

khronos_msgs::Changes toMsg(const Changes& changes) {
  khronos_msgs::Changes msg;
  msg.objects.reserve(changes.object_changes.size());
  for (const auto& object_change : changes.object_changes) {
    msg.objects.emplace_back(toMsg(object_change));
  }
  msg.background.reserve(changes.background_changes.size());
  for (const auto& background_change : changes.background_changes) {
    msg.background.emplace_back(static_cast<uint8_t>(background_change));
  }
  return msg;
}

}  // namespace khronos
