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

#include <vector>

#include <config_utilities/config_utilities.h>

#include "khronos/backend/reconciliation/mesh/mesh_merger.h"

namespace khronos {

/**
 * @brief Simple mesh merger that overwrites the mesh based on the most recent time stamp. Applies a
 * voxel filter to find nearby faces and keeps faces that are within 'time_threshold' seconds of the
 * most recent measurement.
 */
class OverwriteMesh : public MeshMerger {
 public:
  // Config.
  struct Config : public MeshMerger::Config {
    // Distance [m] within which meshes are considered to be the same.
    float voxel_size = 0.1f;

    // Time [s] after which meshes start being overwritten.
    float time_threshold = 1.f;

    // Maximum extent exected in the map used for indexing in meters.
    float max_extent = 1000.f;
  } const config;

  void merge(DynamicSceneGraph& dsg, const BackgroundChanges& /* changes */) override;

  // Construction.
  explicit OverwriteMesh(const Config& config);
  virtual ~OverwriteMesh() = default;

 private:
  inline static const auto registration =
      config::RegistrationWithConfig<MeshMerger, OverwriteMesh, OverwriteMesh::Config>("overwrite");

  // Utility functions.
  // Note(lschmid): Voxel filtering based on the GlobalIndexHashMap does not seem to work,
  // so we implement a simple indxing ourselves. Returns a unique index for each voxel all its
  // dimensions are within [-max_extent, max_extent].
  size_t computeIndex(const Point& point) const;
};

void declare_config(OverwriteMesh::Config& config);

}  // namespace khronos
