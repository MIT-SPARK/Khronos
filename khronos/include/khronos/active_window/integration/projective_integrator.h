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
class ProjectiveIntegrator : protected hydra::ProjectiveIntegrator {
 public:
  // Construction.
  explicit ProjectiveIntegrator(const hydra::ProjectiveIntegratorConfig& config);
  ~ProjectiveIntegrator() = default;

  // Interfaces to perform integration of a frame.
  /**
   * @brief Update all blocks in the background map with the given data in
   * parallel.
   * @param data Input data to use for the update.
   * @param map Map to update.
   */
  void updateBackgroundMap(const FrameData& data, VolumetricMap& map) const;

  /**
   * @brief Update all blocks in the object map with the given data in parallel.
   * @param data Input data to use for the update.
   * @param map Map to update.
   * @param object_id ID of the object in the object image to perform confidence
   * estimation.
   */
  void updateObjectMap(const FrameData& data, VolumetricMap& map, int object_id) const;

 private:
  bool computeLabel(const InputData& data,
                    const float truncation_distance,
                    VoxelMeasurement& measurement) const override;

  mutable const FrameData* current_data_ = nullptr;
};

}  // namespace khronos
