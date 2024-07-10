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

#include "khronos/backend/change_detection/objects/ray_object_change_detector.h"

#include <numeric>

#include <config_utilities/config_utilities.h>

namespace khronos {

void declare_config(RayObjectChangeDetector::Config& config) {
  using namespace config;
  name("RayObjectChangeDetector");
  field(config.time_filtering_threshold, "time_filtering_threshold", "s");
  field(config.query_subsampling, "query_subsampling");
  check(config.time_filtering_threshold, GE, 0.f, "time_filtering_threshold");
}

RayObjectChangeDetector::RayObjectChangeDetector(const Config& config,
                                                 RayVerificator::ConstPtr ray_verificator,
                                                 RayChangeDetector::ConstPtr ray_change_detector)
    : config(config::checkValid(config)),
      ray_verificator_(std::move(ray_verificator)),
      ray_change_detector_(std::move(ray_change_detector)),
      time_filtering_threshold_ns_(config.time_filtering_threshold * 1e9) {}

void RayObjectChangeDetector::detectChanges(const DynamicSceneGraph& dsg,
                                            const RPGOMerges& rpgo_merges,
                                            ObjectChanges& changes) {
  // Compute changes only for new and re-observed objects.
  for (const NodeId node_id : ray_verificator_->getReobservedObjects()) {
    auto it = changes.find(node_id);
    if (it != changes.end()) {
      // Removing change states of re-observed objects so they are recomputed.
      changes.erase(it);
    }
  }
  std::unordered_set<NodeId> already_existing_objects;
  for (const ObjectChange& change : changes) {
    already_existing_objects.insert(change.node_id);
  }

  // Iterate over all objects and chompute their current change state.
  auto& object_layer = dsg.getLayer(DsgLayers::OBJECTS);
  for (const auto& object : object_layer.nodes()) {
    // Ignore existing objects.
    if (already_existing_objects.count(object.first)) {
      continue;
    }

    // Ignore dynamic objects.
    auto& attrs = object.second->attributes<KhronosObjectAttributes>();
    if (!attrs.trajectory_positions.empty()) {
      continue;
    }

    // Setup the object change.
    ObjectChange& change = changes.emplace_back();
    change.node_id = object.first;

    // Check for merges.
    checkObjectMerge(rpgo_merges, change);

    // Check for volumetric changes (presence/absence).
    checkObjectObservation(attrs, change);
  }
}

void RayObjectChangeDetector::checkObjectMerge(const RPGOMerges& rpgo_merges,
                                               ObjectChange& change) const {
  // NOTE(lschmid): This does currently not use invalid merge proposals as evidence of absence.
  // Could think more about.
  const auto it =
      std::find_if(rpgo_merges.begin(), rpgo_merges.end(), [change](const RPGOMerge& merge) {
        return merge.from_node == change.node_id;
      });
  if (it != rpgo_merges.end() && it->is_valid) {
    change.merged_id = it->to_node;
  }
}

void RayObjectChangeDetector::checkObjectObservation(KhronosObjectAttributes& attrs,
                                                     ObjectChange& change) const {
  if (attrs.mesh.points.empty() || attrs.first_observed_ns.empty() ||
      attrs.last_observed_ns.empty()) {
    return;
  }

  // Compute all ray presence/absence stamps of all vertices before and after the object was
  // observed.
  RayVerificator::CheckResult before_data, after_data;
  for (size_t i = 0; i < attrs.mesh.numVertices(); i += config.query_subsampling) {
    const Point point = attrs.mesh.pos(i) + attrs.bounding_box.world_P_center;

    // TODO(lschmid): This double query could be simplified into a single double-ended query.
    const auto before_check = ray_verificator_->check(
        point, 0ul, attrs.first_observed_ns.front() - time_filtering_threshold_ns_);
    const auto after_check = ray_verificator_->check(
        point, attrs.last_observed_ns.back() + time_filtering_threshold_ns_);
    before_data.merge(before_check);
    after_data.merge(after_check);
  }

  // Perform change detection.
  const auto before_result = ray_change_detector_->detectChanges(before_data, false);
  const auto after_result = ray_change_detector_->detectChanges(after_data, true);
  change.first_absent = before_result.closest_absent.value_or(0ul);
  change.last_absent = after_result.closest_absent.value_or(0ul);
  change.first_persistent = before_result.furthest_persistent.value_or(0ul);
  change.last_persistent = after_result.furthest_persistent.value_or(0ul);
}

}  // namespace khronos
