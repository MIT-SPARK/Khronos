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

#include "khronos/common/common_types.h"

namespace khronos {

/**
 * @brief Common data structurefor all detected measurement clusters.
 */
struct MeasurementCluster {
  // All pixels associated with this cluster in the object_image.
  Pixels pixels;

  // 3D axis aligned bounding box of this cluster in world frame.
  BoundingBox bounding_box;

  // Center points of all voxels associated with this cluster in world frame.
  GlobalIndexSet voxels;

  // ID of this cluster (=value of pixels in the corresponding image).
  int id;
};

/**
 * @brief Data structure that collects all detected semantic clusters.
 */
struct SemanticCluster : public MeasurementCluster {
  // ID of this cluster refers to value of pixels in the object image. A value of 0
  // indicates background points.

  // Semantic class ID of this cluster.
  int semantic_id;
};
using SemanticClusters = std::vector<SemanticCluster>;

/**
 * @brief Data structure that collects all detected dynamic clusters.
 */
struct DynamicCluster : public MeasurementCluster {
  // ID of this cluster corresponds to value of pixels in the dynamic_image. A value of
  // 0 indicates static points.
};
using DynamicClusters = std::vector<DynamicCluster>;

}  // namespace khronos
