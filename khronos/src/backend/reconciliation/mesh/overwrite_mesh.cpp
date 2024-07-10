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

#include "khronos/backend/reconciliation/mesh/overwrite_mesh.h"

#include "config_utilities/config_utilities.h"

namespace khronos {

void declare_config(OverwriteMesh::Config& config) {
  using namespace config;
  name("OverwriteMesh");
  base<MeshMerger::Config>(config);
  field(config.voxel_size, "voxel_size");
  field(config.time_threshold, "time_threshold", "s");
  field(config.max_extent, "max_extent", "m");

  check(config.voxel_size, GT, 0, "voxel_size");
  check(config.time_threshold, GT, 0, "time_threshold");
  check(config.max_extent, GT, 0, "max_extent");
}

OverwriteMesh::OverwriteMesh(const Config& config)
    : MeshMerger(config), config(config::checkValid(config)) {}

void OverwriteMesh::merge(DynamicSceneGraph& dsg, const BackgroundChanges& /* changes */) {
  Timer timer("merge_mesh/all", 0);
  //  Simple version: Check for each face if it is within the voxel size of another face.
  const auto& faces = dsg.mesh()->faces;
  const auto& vertices = dsg.mesh()->points;
  const auto& colors = dsg.mesh()->colors;
  const std::vector<uint64_t>& time_stamps = dsg.mesh()->stamps;
  if (time_stamps.size() != vertices.size()) {
    LOG(ERROR) << "[OverwriteMesh] Time stamps and vertices do not match (" << time_stamps.size()
               << " vs " << vertices.size() << ").";
    return;
  }

  // Apply voxel filter to faces and compare their time stamps.
  std::unordered_map<size_t, float> most_recent_measrements;
  std::vector<size_t> indices;
  std::vector<float> times;
  indices.reserve(faces.size());
  times.reserve(faces.size());

  for (size_t i = 0; i < faces.size(); ++i) {
    // Compute the voxel index for the center of the face.
    Point center = Point::Zero();
    uint64_t most_recent_stamp = 0;
    for (const size_t vertex_id : faces[i]) {
      center += vertices[vertex_id];
      most_recent_stamp = std::max(most_recent_stamp, time_stamps[vertex_id]);
    }
    center /= faces[i].size();
    const float time = most_recent_stamp / 1e9;
    const size_t index = computeIndex(center);
    indices.push_back(index);
    times.push_back(time);

    auto it = most_recent_measrements.find(index);
    if (it == most_recent_measrements.end()) {
      most_recent_measrements[index] = time;
    } else {
      it->second = std::max(it->second, time);
    }
  }

  // Create a new mesh based on the remaining faces.
  std::unordered_map<uint32_t, uint32_t> vertex_id_map;
  auto new_mesh = std::make_shared<spark_dsg::Mesh>();
  size_t new_vertex_id = 0;
  for (size_t i = 0; i < faces.size(); ++i) {
    if (most_recent_measrements[indices[i]] > times[i] + config.time_threshold) {
      // The face is outdated and thus not added to the new mesh.
      continue;
    }

    // Copy the face, if needed the vertices, and adjust the vertex ids.
    auto& new_face = new_mesh->faces.emplace_back(faces[i]);
    for (size_t& vertex_id : new_face) {
      auto it = vertex_id_map.find(vertex_id);
      if (it == vertex_id_map.end()) {
        // Vertex added for the first time.
        vertex_id_map[vertex_id] = new_vertex_id;
        new_mesh->points.push_back(vertices[vertex_id]);
        new_mesh->colors.push_back(colors[vertex_id]);
        new_mesh->first_seen_stamps.push_back(dsg.mesh()->firstSeenTimestamp(vertex_id));
        vertex_id = new_vertex_id;
        ++new_vertex_id;
      } else {
        // Existing vertex.
        vertex_id = it->second;
      }
    }
  }

  // Write mesh.
  dsg.setMesh(new_mesh);
  CLOG(3) << "[OverwriteMesh] Removed " << faces.size() - new_mesh->numFaces() << " faces and "
          << vertices.size() - new_mesh->numVertices() << " vertices.";

  removeObjectsFromBackground(dsg);

  // Some additional verbose printing for debugging / tuning.
  if (config.verbosity >= 4) {
    LOG(INFO) << "[OverwriteMesh] Min time: "
              << static_cast<float>(*std::min_element(time_stamps.begin(), time_stamps.end()) /
                                    1e9);
    LOG(INFO) << "[OverwriteMesh] Max time: "
              << static_cast<float>(*std::max_element(time_stamps.begin(), time_stamps.end()) /
                                    1e9);
    LOG(INFO) << "[OverwriteMesh] Num voxels: " << most_recent_measrements.size();
  }
}

size_t OverwriteMesh::computeIndex(const Point& point) const {
  const size_t vps = 2 * config.max_extent / config.voxel_size;
  size_t index = 0;
  for (size_t i = 0; i < 3; ++i) {
    const size_t dim_index = std::floor((point[i] + config.max_extent) / config.voxel_size);
    index += dim_index * std::pow(vps, i);
  }
  return index;
}

}  // namespace khronos
