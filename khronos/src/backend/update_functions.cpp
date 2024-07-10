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

#include "khronos/backend/update_functions.h"

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <khronos/common/common_types.h>

namespace khronos {

using hydra::MergeList;

void declare_config(UpdateObjectsFunctor::Config& config) {
  using namespace config;
  name("UpdateObjectsFunctor");
  field(config.merge_require_same_label, "merge_require_same_label");
  field(config.merge_require_no_co_visibility, "merge_require_no_co_visibility");
  field(config.merge_min_iou, "merge_min_iou");
  checkInRange(config.merge_min_iou, 0.f, 1.f, "merge_min_iou");
}

UpdateObjectsFunctor::Hooks UpdateObjectsFunctor::hooks() const {
  auto my_hooks = UpdateFunctor::hooks();
  return my_hooks;
}

MergeList UpdateObjectsFunctor::call(const DynamicSceneGraph& /* unmerged */,
                                     SharedDsgInfo& dsg,
                                     const UpdateInfo::ConstPtr& info) const {
  Timer spin_timer("backend/update_objects", info->timestamp_ns);
  // NOTE(lschmid): This flag is high-jacked and indicates whether the graph is ready to
  // propose merges or not.
  if (!info->loop_closure_detected) {
    return {};
  }
  std::unique_lock<std::mutex> lock(dsg.mutex);
  const auto& graph = *dsg.graph;
  // TODO(Yun): Refactor
  if (!graph.hasLayer(DsgLayers::OBJECTS) || !info->pgmo_values) {
    return {};
  }

  const auto& layer = graph.getLayer(DsgLayers::OBJECTS);
  const auto& objects_values = *info->pgmo_values;

  std::map<NodeId, NodeId> nodes_to_merge;
  std::set<NodeId> merge_targets;
  for (auto it = layer.nodes().begin(); it != layer.nodes().end(); ++it) {
    const NodeId id = it->first;
    auto& attrs = it->second->attributes<KhronosObjectAttributes>();
    // Update all objects with their new poses.
    // TODO(marcus): track down why this is now necessary
    if (!objects_values.exists(id)) {
      LOG(WARNING) << "Failure to update object " << attrs.name << ": PGMO value not found.";
      continue;
    }
    updateObject(objects_values, id, attrs);

    // Allow many to one merges but make sure each node is only merged once.
    if (merge_targets.count(id) || nodes_to_merge.count(id)) {
      continue;
    }

    // Find all candidate merges for this object.
    for (auto it2 = it; it2 != layer.nodes().end(); ++it2) {
      const NodeId candidate_id = it2->first;
      if (candidate_id == id || nodes_to_merge.count(candidate_id)) {
        continue;
      }
      const auto& candidate_attrs = it2->second->attributes<KhronosObjectAttributes>();
      if (shouldMerge(attrs, candidate_attrs)) {
        nodes_to_merge[candidate_id] = id;
        merge_targets.insert(id);
      }
    }
  }

  for (const auto& [from, to] : nodes_to_merge) {
    new_proposed_merges_.push_back({from, to});
  }

  // The goal of this functor is to compute candidate merges and store them in
  // 'new_proposed_merges_'. Don't return anything so the candidate objects don't get merged.
  // TODO(lschmid): This can also be cleaned up once the new hydra unmerged graph architecture is
  // in.
  return {};
}

void UpdateObjectsFunctor::updateObject(const gtsam::Values& objects_values,
                                        NodeId node_id,
                                        KhronosObjectAttributes& attrs) const {
  if (!objects_values.exists(node_id)) {
    throw std::runtime_error("object does not exist in deformation graph.");
  }
  // Move the object and its bbox to the new position.
  // NOTE(lschmid): Currently only translates AABB bounding boxes.
  attrs.position = objects_values.at<gtsam::Pose3>(node_id).translation();
  attrs.bounding_box.world_P_center = attrs.position.cast<float>();
}

bool UpdateObjectsFunctor::shouldMerge(const KhronosObjectAttributes& from_attrs,
                                       const KhronosObjectAttributes& to_attrs) const {
  // Must be of same semantic label.
  if (config.merge_require_same_label && from_attrs.semantic_label != to_attrs.semantic_label) {
    return false;
  }

  // May not have co-visibility.
  if (config.merge_require_no_co_visibility) {
    // Note (Yun): Implicit assumption here: since we do not actually reconcile the back-end scene
    // graph, the first_observed_ns and last_observed_ns vectors should only have a single element.
    if (from_attrs.first_observed_ns.front() < to_attrs.first_observed_ns.front() &&
        from_attrs.last_observed_ns.front() > to_attrs.first_observed_ns.front()) {
      return false;
    }

    if (from_attrs.last_observed_ns.front() > to_attrs.first_observed_ns.front() &&
        from_attrs.first_observed_ns.front() < to_attrs.last_observed_ns.front()) {
      return false;
    }
  }

  // Must always have overlap.
  if (!to_attrs.bounding_box.intersects(from_attrs.bounding_box)) {
    return false;
  }

  // Check required IoU.
  return config.merge_min_iou == 0 ||
         to_attrs.bounding_box.computeIoU(from_attrs.bounding_box) >= config.merge_min_iou;
}

}  // namespace khronos
