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

#include <hydra/reconstruction/projective_integrator.h>

#include "khronos/active_window/data/frame_data.h"
#include "khronos/active_window/data/reconstruction_types.h"

namespace khronos {

/**
 * @brief Retain khronos-specific data and interfaces for the projective integrator after it was
 * moved to hydra.
 */
class ObjectIntegrator : public hydra::ProjectiveIntegrator {
 public:
  // Construction.
  explicit ObjectIntegrator(const hydra::ProjectiveIntegrator::Config& config);
  ~ObjectIntegrator() = default;

  /**
   * @brief Set the data and target object ID for the integrator.
   * @param frame_data The frame data to use for integration.
   * @param target_object_id The ID of the target object to integrate as belonging to the object.
   */
  void setFrameData(FrameData* frame_data, int target_object_id);

 private:
  // Specialize computeLabel to select the object image insstead of semantics.
  bool computeLabel(const VolumetricMap::Config& map_config,
                    const InputData& /* data */,
                    const cv::Mat& integration_mask,
                    VoxelMeasurement& measurement) const override;

  // The current frame data to use for integration.
  FrameData* current_data_ = nullptr;
  int current_object_id_ = -1;
};

}  // namespace khronos
