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

#include "khronos/utils/geometry_utils.h"

#include <hydra/input/input_data.h>

namespace khronos::utils {

Point computeCentroid(const Points& points) {
  Point centroid(0, 0, 0);
  for (const auto& vertex : points) {
    centroid += vertex;
  }
  return centroid / points.size();
}

VertexMapAdaptor::VertexMapAdaptor(const Pixels& pixels, const cv::Mat& vertex_map)
    : pixels(pixels), vertex_map(vertex_map) {}

Eigen::Vector3f VertexMapAdaptor::get(size_t index) const {
  const auto& pixel = pixels[index];
  const auto& vertex = vertex_map.at<hydra::InputData::VertexType>(pixel.v, pixel.u);
  return Eigen::Vector3f(vertex[0], vertex[1], vertex[2]);
}

Mesh combineMeshLayer(const MeshLayer& mesh_layer) {
  Mesh output;
  size_t vertex_index_offset = 0;
  for (const auto& block : mesh_layer) {
    // Copy all vertex attributes.
    output.points.insert(output.points.end(), block.points.begin(), block.points.end());
    output.colors.insert(output.colors.end(), block.colors.begin(), block.colors.end());
    output.labels.insert(output.labels.end(), block.labels.begin(), block.labels.end());
    output.first_seen_stamps.insert(output.first_seen_stamps.end(),
                                    block.first_seen_stamps.begin(),
                                    block.first_seen_stamps.end());
    output.stamps.insert(output.stamps.end(), block.stamps.begin(), block.stamps.end());

    // Copy all faces and adjust their vertex indices.
    output.faces.reserve(output.faces.size() + block.faces.size());
    for (const auto& face : block.faces) {
      Mesh::Face new_face;
      new_face[0] = face[0] + vertex_index_offset;
      new_face[1] = face[1] + vertex_index_offset;
      new_face[2] = face[2] + vertex_index_offset;
      output.faces.emplace_back(std::move(new_face));
    }
    vertex_index_offset += block.points.size();
  }
  return output;
}

}  // namespace khronos::utils
