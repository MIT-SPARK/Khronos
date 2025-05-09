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

#include "khronos/active_window/integration/object_integrator.h"

namespace khronos {

using IntegratorConfig = hydra::ProjectiveIntegrator::Config;

IntegratorConfig forceBinaryIntegrator(const IntegratorConfig& config) {
  auto new_config = config;
  new_config.semantic_integrator = hydra::BinarySemanticIntegrator::Config();
  return new_config;
}

ObjectIntegrator::ObjectIntegrator(const IntegratorConfig& config)
    : hydra::ProjectiveIntegrator(forceBinaryIntegrator(config)) {}

void ObjectIntegrator::setFrameData(FrameData* frame_data, int target_object_id) {
  current_data_ = frame_data;
  current_object_id_ = target_object_id;
}

bool ObjectIntegrator::computeLabel(const VolumetricMap::Config& map_config,
                                    const InputData& /* data */,
                                    const cv::Mat& integration_mask,
                                    VoxelMeasurement& measurement) const {
  if (std::abs(measurement.sdf) >= map_config.truncation_distance) {
    // If SDF value is beyond the truncation band, we don't need to
    // compute a label and the point is always valid for integration
    return true;
  }

  // the formatting here is a little ugly, but if an integration mask is supplied and it
  // is non-zero for the best pixel, we skip integrating the current voxel
  if (!integration_mask.empty() &&
      interpolator_->interpolateID(integration_mask, measurement.interpolation_weights)) {
    return false;
  }

  // If the semantic integrator is set, we assume this is used for object extraction. We thus set
  // the label to the object id.
  measurement.label =
      interpolator_->interpolateID(current_data_->object_image,
                                   measurement.interpolation_weights) == current_object_id_;
  return true;
}

}  // namespace khronos
