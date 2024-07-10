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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include "khronos/active_window/active_window.h"
#include "khronos/backend/change_state.h"
#include "khronos/backend/reconciliation/mesh/mesh_merger.h"
#include "khronos/common/common_types.h"
#include "khronos/spatio_temporal_map/spatio_temporal_map.h"

namespace khronos {

/**
 * @brief Class to reconcile temporal differences in an optimized DSG into a single
 * spatio-temporal map.
 */
class Reconciler {
 public:
  using ChangeMap = std::unordered_map<NodeId, ObjectChange>;

  // Config.
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // If true only estimate times when there's evidence. If false extend open ended times to
    // infinity.
    bool time_estimates_conservative = false;

    // If true allow estimated times to exceed the previous/next observation. If false, clamp to
    // previous/next observation during fusion
    bool allow_overestimation = false;

    // If true, add meshes of objects that are merged. If false, keep the largest one.
    bool merge_object_meshes = false;

    // Which mesh merger to use.
    config::VirtualConfig<MeshMerger> mesh_merger;
  } const config;

  // Construction.
  explicit Reconciler(const Config& config);
  virtual ~Reconciler() = default;

  // Processing.
  /**
   * @brief Reconcile temporal differences in an optimized DSG. The DSG will be updated in place.
   * @param dsg The optimized (backend) DSG to reconcile.
   * @param changes The current changes of the DSG computed by change detection to reconcile.
   * @param stamp The current time stamp.
   * @returns The merged changes for each object.
   */
  void reconcile(DynamicSceneGraph& dsg, const Changes& changes, TimeStamp stamp);

 protected:
  // Utility.
  void reconcileObjects(const Changes& changes, DynamicSceneGraph& dsg);
  // Merge node from into node into in the dsg.
  void mergeObjects(const ObjectChange& change, DynamicSceneGraph& dsg) const;
  void mergeObjectAttributes(const KhronosObjectAttributes& from,
                             KhronosObjectAttributes& into) const;
  void mergeObjectMeshes(const KhronosObjectAttributes& from, KhronosObjectAttributes& into) const;
  void mergeObjectChanges(const ObjectChange& from,
                          ObjectChange& into,
                          const uint64_t first_seen,
                          const uint64_t last_seen) const;

  // Estimate the presence time of an object by combining the first/last seen data in the attributes
  // with the first/last evidence of presence/absence in the change. Call this on the initial
  // observation of the object. Will update the presence times in the object attributes.
  void estimatePresenceTimes(const ObjectChange& change, KhronosObjectAttributes& attrs) const;

  void clampOverestimation(const KhronosObjectAttributes& reference,
                           KhronosObjectAttributes& to_clamp) const;

 private:
  // Members for processing.
  std::unique_ptr<MeshMerger> mesh_merger_;
};

void declare_config(Reconciler::Config& config);

}  // namespace khronos
