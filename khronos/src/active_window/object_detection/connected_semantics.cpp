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

#include "khronos/active_window/object_detection/connected_semantics.h"

#include <string>
#include <vector>

namespace khronos {

void declare_config(ConnectedSemantics::Config& config) {
  using namespace config;
  name("ConnectedSemantics");
  field(config.verbosity, "verbosity");
  field(config.use_full_connectivity, "use_full_connectivity");
  field(config.min_cluster_size, "min_cluster_size");
  field(config.max_cluster_size, "max_cluster_size");
  field(config.max_range, "max_range", "m");
  field(config.use_3d, "use_3d");
}

ConnectedSemantics::ConnectedSemantics(const Config& config)
    : config(config::checkValid(config)), neighbor_search_(config.use_full_connectivity ? 26 : 6) {}

void ConnectedSemantics::processInput(const VolumetricMap& /* map */, FrameData& data) {
  processing_stamp_ = data.input.timestamp_ns;
  Timer timer("object_detection/all", processing_stamp_);
  if (config.use_3d) {
    semanticClustering3D(data, config.grid_size);
  } else {
    semanticClustering2D(data);
    filterClusters(data);
  }
}

void ConnectedSemantics::semanticClustering3D(FrameData& data, const float grid_size) {
  Timer timer("object_detection/clustering", processing_stamp_);

  // Detect all candidate points and order them by semantic id.
  SemanticVoxelPixelMaps semantic_pixelmaps = computeCandidateVoxels(data, 1.f / grid_size);

  // For each semantic id, cluster the candidate voxels.
  for (auto& semantic_pixelmap : semantic_pixelmaps) {
    const int semantic_id = semantic_pixelmap.first;
    VoxelPixelMap& voxel_to_pixels = semantic_pixelmap.second;

    // Cluster the voxels.
    while (!voxel_to_pixels.empty()) {
      MeasurementCluster cluster;
      GlobalIndices stack;
      auto item = voxel_to_pixels.begin();
      stack.emplace_back(item->first);
      cluster.pixels.insert(cluster.pixels.end(), item->second.begin(), item->second.end());
      voxel_to_pixels.erase(item);

      // Region growing in voxel index space.
      while (!stack.empty()) {
        const GlobalIndex voxel_index = stack.back();
        stack.pop_back();
        for (const GlobalIndex& neighbor : neighbor_search_.neighborIndices(voxel_index)) {
          auto it = voxel_to_pixels.find(neighbor);
          if (it == voxel_to_pixels.end()) {
            continue;
          }
          stack.emplace_back(it->first);
          cluster.pixels.insert(cluster.pixels.end(), it->second.begin(), it->second.end());
          voxel_to_pixels.erase(it);
        }
      }

      // Add the cluster if it is inside the bounds
      const auto curr_cluster_size = static_cast<int>(cluster.pixels.size());
      if (curr_cluster_size < config.min_cluster_size ||
          (config.max_cluster_size > 0 && curr_cluster_size > config.max_cluster_size)) {
        continue;
      }
      cluster.id = data.semantic_clusters.size() + 1;
      cluster.semantics = SemanticClusterInfo(semantic_id);
      for (const Pixel& pixel : cluster.pixels) {
        data.object_image.at<FrameData::ObjectImageType>(pixel.v, pixel.u) = cluster.id;
      }
      data.semantic_clusters.emplace_back(std::move(cluster));
    }
  }
}

ConnectedSemantics::SemanticVoxelPixelMaps ConnectedSemantics::computeCandidateVoxels(
    const FrameData& data,
    const float voxel_size_inv) const {
  SemanticVoxelPixelMaps semantic_pixelmaps;
  for (int u = 0; u < data.input.label_image.cols; u++) {
    for (int v = 0; v < data.input.label_image.rows; v++) {
      if (config.max_range > 0.f) {
        const float range = data.input.range_image.at<InputData::RangeType>(v, u);
        if (range > config.max_range) {
          continue;
        }
      }
      const int semantic_id = data.input.label_image.at<InputData::LabelType>(v, u);
      if (!hydra::GlobalInfo::instance().getLabelSpaceConfig().isObject(semantic_id)) {
        continue;
      }
      const auto& point = data.input.vertex_map.at<InputData::VertexType>(v, u);
      const auto voxel_index = spatial_hash::indexFromPoint<GlobalIndex>(
          Point(point[0], point[1], point[2]), voxel_size_inv);
      semantic_pixelmaps[semantic_id][voxel_index].emplace_back(u, v);
    }
  }
  return semantic_pixelmaps;
}

void ConnectedSemantics::semanticClustering2D(FrameData& data) {
  // NOTE(lschmid): Naive implementation as cv::connectedComponents does not seem to do
  // this well.
  Timer timer("object_detection/clustering", processing_stamp_);
  for (int u = 0; u < data.input.label_image.cols; u++) {
    for (int v = 0; v < data.input.label_image.rows; v++) {
      auto& id = data.object_image.at<FrameData::ObjectImageType>(v, u);
      if (id != 0) {
        continue;
      }
      const int semantic_id = data.input.label_image.at<InputData::LabelType>(v, u);
      if (!hydra::GlobalInfo::instance().getLabelSpaceConfig().isObject(semantic_id)) {
        continue;
      }
      growCluster2D(u, v, data);
    }
  }
}

void ConnectedSemantics::growCluster2D(const int u, const int v, FrameData& data) const {
  Pixels stack{{u, v}};
  auto& cluster = data.semantic_clusters.emplace_back();
  cluster.id = data.semantic_clusters.size();
  const int semantic_id = data.input.label_image.at<InputData::LabelType>(v, u);
  cluster.semantics = SemanticClusterInfo(semantic_id);

  // Add the pixel to the cluster.
  cluster.pixels.emplace_back(u, v);
  data.object_image.at<FrameData::ObjectImageType>(v, u) = cluster.id;
  while (!stack.empty()) {
    Pixel pixel = stack.back();
    stack.pop_back();

    // Check the neighbors.
    for (const Pixel& neighbor :
         config.use_full_connectivity ? pixel.neighbors8() : pixel.neighbors4()) {
      if (!neighbor.isInImage(data.input.label_image)) {
        continue;
      }
      auto& neighbor_id = data.object_image.at<FrameData::ObjectImageType>(neighbor.v, neighbor.u);
      if (neighbor_id != 0) {
        continue;
      }
      const int neighbor_semantic_id =
          data.input.label_image.at<InputData::LabelType>(neighbor.v, neighbor.u);
      if (neighbor_semantic_id == semantic_id) {
        cluster.pixels.emplace_back(neighbor);
        neighbor_id = cluster.id;
        stack.push_back(neighbor);
      }
    }
  }
}

void ConnectedSemantics::filterClusters(FrameData& data) const {
  Timer timer("object_detection_filtering", processing_stamp_);

  auto it = data.semantic_clusters.begin();
  size_t num_erased_clusters = 0;
  while (it != data.semantic_clusters.end()) {
    if (static_cast<int>(it->pixels.size()) < config.min_cluster_size) {
      data.object_image.setTo(0, data.object_image == it->id);
      it = data.semantic_clusters.erase(it);
      num_erased_clusters++;
    } else {
      it++;
    }
  }
  CLOG(4) << "[ConnectedSemantics] Identified "
          << data.semantic_clusters.size() + num_erased_clusters << " clusters, erased "
          << num_erased_clusters << " (" << data.semantic_clusters.size() << " total).";
}

}  // namespace khronos
