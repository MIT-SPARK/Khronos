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

#include <map>
#include <memory>
#include <set>

#include <config_utilities/config_utilities.h>
#include <gtsam/nonlinear/Values.h>
#include <hydra/backend/update_functions.h>
#include <khronos/common/common_types.h>

namespace khronos {

using hydra::SharedDsgInfo;
using hydra::UpdateInfo;
using spark_dsg::SemanticLabel;
using spark_dsg::SemanticNodeAttributes;

/**
 * @brief Specific object updater for Khronos.
 */
struct UpdateObjectsFunctor : public hydra::UpdateFunctor {
  // Config.
  struct Config {
    bool merge_require_same_label = true;
    bool merge_require_no_co_visibility = true;
    float merge_min_iou = 0.5;
  } const config;

  // Constructor.
  UpdateObjectsFunctor(const Config& config, hydra::MergeList& new_proposed_merges)
      : config(config::checkValid(config)), new_proposed_merges_(new_proposed_merges) {}

  // Implement Hydra interfaces.
  Hooks hooks() const override;
  hydra::MergeList call(const DynamicSceneGraph& unmerged,
                        SharedDsgInfo& dsg,
                        const UpdateInfo::ConstPtr& info) const override;

  // Functionality.
  void updateObject(const gtsam::Values& objects_values,
                    NodeId node_id,
                    KhronosObjectAttributes& attrs) const;

  bool shouldMerge(const KhronosObjectAttributes& from_attrs,
                   const KhronosObjectAttributes& to_attrs) const;

  // Reference to the output variable where this object functor stores proposed merges.
  // TODO(lschmid): This needs clean up, not the most elegant solution.
  hydra::MergeList& new_proposed_merges_;
};

void declare_config(UpdateObjectsFunctor::Config& config);

}  // namespace khronos
