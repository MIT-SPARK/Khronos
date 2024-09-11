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

#include "khronos/active_window/integration/projective_integrator.h"

namespace khronos {

ProjectiveIntegrator::ProjectiveIntegrator(const hydra::ProjectiveIntegratorConfig& config)
    : hydra::ProjectiveIntegrator(config) {}

void ProjectiveIntegrator::updateBackgroundMap(const FrameData& data, VolumetricMap& map) const {
  current_data_ = &data;
  hydra::ProjectiveIntegrator::updateMap(data.input, map, true);
}

void ProjectiveIntegrator::updateObjectMap(const FrameData& data,
                                           VolumetricMap& map,
                                           int object_id) const {
  if (!semantic_integrator_) {
    LOG(WARNING) << "Can not perform object reconstruction without semantic integrator.";
    return;
  }
  current_data_ = &data;
  // NOTE(lschmid): We abuse 'canIntegrate' which is not used in hydra.
  semantic_integrator_->canIntegrate(object_id);

  // NOTE(nathan) we side-step finding "in-view" blocks and just integrate all of the allocated blocks
  const auto indices = map.getTsdfLayer().allocatedBlockIndices();
  hydra::ProjectiveIntegrator::updateBlocks(indices, data.input, map);
}

bool ProjectiveIntegrator::computeLabel(const InputData& /* data */,
                                        const float truncation_distance,
                                        VoxelMeasurement& measurement) const {
  // Don't integrate surface voxels of dynamic measurements.
  const bool is_dynamic = interpolator_->interpolateID(current_data_->dynamic_image,
                                                       measurement.interpolation_weights) != 0u;
  if (is_dynamic && measurement.sdf < truncation_distance) {
    return false;
  }

  // If the semantic integrator is set, we assume this is used for object extraction. We thus set
  // the label to the object id.
  if (semantic_integrator_) {
    measurement.label = interpolator_->interpolateID(current_data_->object_image,
                                                     measurement.interpolation_weights);
  }
  return true;
}

}  // namespace khronos
