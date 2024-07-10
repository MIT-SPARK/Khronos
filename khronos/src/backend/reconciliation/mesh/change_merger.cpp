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

#include "khronos/backend/reconciliation/mesh/change_merger.h"

#include <config_utilities/config_utilities.h>
#include <hydra/utils/nearest_neighbor_utilities.h>

namespace khronos {

void declare_config(ChangeMerger::Config& config) {
  using namespace config;
  name("ChangeMerger");
  base<MeshMerger::Config>(config);
}

ChangeMerger::ChangeMerger(const Config& config)
    : MeshMerger(config), config(config::checkValid(config)) {}

void ChangeMerger::merge(DynamicSceneGraph& dsg, const BackgroundChanges& changes) {
  Timer timer("merge_mesh/all", 0);
  const auto& vertices = dsg.mesh()->points;
  const size_t num_prev_vertices = vertices.size();
  const size_t num_prev_faces = dsg.mesh()->numFaces();

  // Setup object surface search if required.
  Points object_points;
  if (config.remove_objects_from_background) {
    const auto& nodes = dsg.getLayer(DsgLayers::OBJECTS).nodes();
    for (const auto& [id, node] : nodes) {
      const auto& attrs = node->attributes<KhronosObjectAttributes>();
      object_points.reserve(object_points.size() + attrs.mesh.numVertices());
      for (const auto& vertex : attrs.mesh.points) {
        object_points.emplace_back(attrs.bounding_box.pointToWorldFrame(vertex));
      }
    }
  }
  const hydra::PointNeighborSearch search(object_points);
  const float distance_threshold =
      config.object_proximity_threshold * config.object_proximity_threshold;

  // Remove all absent and persistent (=duplicated) vertices and corresponding faces.
  std::unordered_set<uint64_t> vertices_to_delete;
  for (size_t i = 0; i < vertices.size(); ++i) {
    if (changes.size() > i && changes[i] != ChangeState::kUnobserved) {
      vertices_to_delete.insert(i);
      continue;
    }

    // Check if close to an object.
    if (!config.remove_objects_from_background) {
      continue;
    }
    float distance_sqaured = std::numeric_limits<float>::max();
    size_t index;
    search.search(vertices[i], distance_sqaured, index);
    if (distance_sqaured < distance_threshold) {
      vertices_to_delete.insert(i);
    }
  }

  // Write the new mesh.
  dsg.mesh()->eraseVertices(vertices_to_delete);
  CLOG(3) << "[ChangeMerger] Removed " << num_prev_faces - dsg.mesh()->numFaces() << " faces and "
          << num_prev_vertices - dsg.mesh()->numVertices() << " vertices.";
}

}  // namespace khronos
