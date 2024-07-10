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

#include <string>
#include <utility>
#include <vector>

#include <spark_dsg/dynamic_scene_graph.h>

#include "khronos/common/common_types.h"

namespace khronos {

// TODO(lschmid): This probably needs to go somewhere else eventually but cleanest to have here for
// now I think.

// Data structure to take in proposed merges from the Robust Pose Graph Optimizer.
struct RPGOMerge {
  RPGOMerge() = default;
  RPGOMerge(NodeId from_node, NodeId to_node, bool is_valid)
      : from_node(from_node), to_node(to_node), is_valid(is_valid) {}

  NodeId from_node;
  NodeId to_node;
  bool is_valid;
};

struct RPGOMerges : public std::vector<RPGOMerge> {
  bool save(const std::string& filename) const;
  bool load(const std::string& filename);
  static RPGOMerges fromFile(const std::string& filename);

 private:
  inline static const std::vector<std::string> kHeaderNames = {"from_node", "to_node", "is_valid"};
};

/**
 * @brief Struct that stores information about the change of a single object.
 */
struct ObjectChange {
  // ID of the object being checked.
  NodeId node_id;

  // Key timestamps of the object where observed. History of an object is:
  // first_absent -> first_persistent -> obj.first_seen -> obj.last_seen -> last_persistent ->
  // last_absent.

  // This id is set to the object that this object is to be merged with. If the object is not
  // merged, this is set to the 0.
  NodeId merged_id = 0ul;

  // Time stamp when there was last evidence of absence before this object was first seen. If the
  // object was never observed absent before, this is set to 0.
  uint64_t first_absent = 0ul;

  // Time stamp when there was first evidence of absence after this object was last seen. If the
  // object was never observed absent after, this is set to 0.
  uint64_t last_absent = 0ul;

  // Time stamp when there was first evidence of presence before this object was first seen. If the
  // object was never observed present before, this is set to 0.
  uint64_t first_persistent = 0ul;

  // Time stamp when there was last evidence of presence after this object was last seen. If the
  // object was never observed present after, this is set to 0.
  uint64_t last_persistent = 0ul;
};

struct ObjectChanges : public std::vector<ObjectChange> {
  bool save(const std::string& filename) const;
  bool load(const std::string& filename);
  static ObjectChanges fromFile(const std::string& filename);
  std::vector<ObjectChange>::iterator find(NodeId node_id);
  std::vector<ObjectChange>::const_iterator find(NodeId node_id) const;

 private:
  inline static const std::vector<std::string> kHeaderNames = {"node_id",
                                                               "merged_id",
                                                               "first_absent",
                                                               "last_absent",
                                                               "first_persistent",
                                                               "last_persistent"};
};

/**
 * @brief Main kinds of changes that can be detected.
 */
enum class ChangeState { kUnobserved, kPersistent, kAbsent };

/**
 * @brief Change state for each vertex in equal order as in the DSG background mesh.
 */
struct BackgroundChanges : public std::vector<ChangeState> {
  bool save(const std::string& filename) const;
  bool load(const std::string& filename);
  static BackgroundChanges fromFile(const std::string& filename);
};

/**
 * @brief Struct that stores the current best estimated configuration of the DSG after change
 * detection.
 */
struct Changes {
  // All changes detected in objects.
  ObjectChanges object_changes;

  // All changes detected in the background.
  BackgroundChanges background_changes;
};

}  // namespace khronos
