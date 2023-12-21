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

#include "khronos/active_window/motion_detection/free_space_motion_detector.h"

#include <future>
#include <string>
#include <vector>

#include <hydra/common/global_info.h>
#include <hydra/reconstruction/index_getter.h>

#include "khronos/utils/geometry_utils.h"

namespace khronos {

void declare_config(FreeSpaceMotionDetector::Config& config) {
  using namespace config;
  name("FreeSpaceMotionDetector");
  field(config.verbosity, "verbosity");
  field(config.neighbor_connectivity, "neighbor_connectivity");
  field(config.min_cluster_size, "min_cluster_size");
  field(config.max_cluster_size, "max_cluster_size");
  field(config.min_separation_distance, "min_separation_distance", "voxels");
  field(config.max_range, "max_range", "m");
  field<ThreadNumConversion>(config.num_threads, "num_threads");

  check(config.num_threads, GT, 0, "num_threads");
  checkIsOneOf(config.neighbor_connectivity, {6, 18, 26}, "neighbor_connectivity");
  checkCondition(config.max_cluster_size >= config.min_cluster_size,
                 "param 'max_cluster_size' must be >= 'min_cluster_size'");
  check(config.max_range, GT, 0, "max_range");
}

FreeSpaceMotionDetector::FreeSpaceMotionDetector(const Config& config)
    : config(config::checkValid(config)) {}

void FreeSpaceMotionDetector::processInput(const VolumetricMap& map, FrameData& data) {
  processing_stamp_ = data.input.timestamp_ns;
  Timer timer("motion_detection/all", processing_stamp_);

  // Detect all moving object seeds.
  GlobalIndexSet voxel_seeds;
  BlockToPointsMap point_map;
  setUpPointMap(data, *map.getTrackingLayer(), point_map, voxel_seeds);

  // Cluster them through voxels.
  auto clusters = clusterDynamicVoxels(point_map, voxel_seeds, map.config.voxels_per_side);
  const size_t num_initial_clusters = clusters.size();

  // Merge nearby clusters.
  mergeClusters(clusters);
  const size_t num_merged_clusters = clusters.size();

  // Filter clusters.
  applyClusterLevelFilters(clusters);

  // Write to data.
  writeClustersToData(clusters, data);

  // Logging some info if requested.
  CLOG(4) << "[FreeSpaceMotionDetector] Found " << voxel_seeds.size()
          << " voxel seeds, clustered them into " << num_initial_clusters
          << " initial clusters, merged " << num_initial_clusters - num_merged_clusters
          << ", filtered out " << num_merged_clusters - clusters.size() << ", resulting in "
          << clusters.size() << " final clusters.";
}

void FreeSpaceMotionDetector::setUpPointMap(const FrameData& data,
                                            const TrackingLayer& tracking_layer,
                                            BlockToPointsMap& point_map,
                                            GlobalIndexSet& voxel_seeds) const {
  Timer timer("motion_detection/compute_point_map", processing_stamp_);

  // Multi-threaded implementation to split the image into parts.
  std::vector<std::future<void>> threads;
  std::mutex aggregate_results_mutex;
  int u_step = data.input.vertex_map.cols / config.num_threads;
  if (u_step * config.num_threads < data.input.vertex_map.cols) {
    u_step++;
  }
  for (int i = 0; i < config.num_threads; ++i) {
    threads.emplace_back(std::async(
        std::launch::async,
        [this,
         &data,
         &tracking_layer,
         &u_step,
         &aggregate_results_mutex,
         &voxel_seeds,
         &point_map,
         i]() {
          // FrameData to store results local to each thread.
          GlobalIndexSet local_seeds;
          BlockToPointsMap local_point_map;

          // Process a part of the image.
          setUpPointMapPart(data,
                            tracking_layer,
                            local_point_map,
                            local_seeds,
                            u_step * i,
                            std::min(u_step * (i + 1), data.input.vertex_map.cols));

          // After processing is done add data to the output map.
          std::lock_guard<std::mutex> lock(aggregate_results_mutex);
          voxel_seeds.merge(local_seeds);
          for (auto& block_idx_map_pair : local_point_map) {
            for (auto& voxel_idx_map_pair : block_idx_map_pair.second) {
              Pixels& pixels = point_map[block_idx_map_pair.first][voxel_idx_map_pair.first];
              pixels.insert(
                  pixels.end(), voxel_idx_map_pair.second.begin(), voxel_idx_map_pair.second.end());
            }
          }
        }));
  }
  for (auto& thread : threads) {
    thread.get();
  }
}

void FreeSpaceMotionDetector::setUpPointMapPart(const FrameData& data,
                                                const TrackingLayer& tracking_layer,
                                                BlockToPointsMap& point_map,
                                                GlobalIndexSet& voxel_seeds,
                                                int u_start,
                                                int u_stop) const {
  // Identifies for any point the block it falls in and constructs the
  // hash-map block_map mapping each block to the points that
  // fall into the block.
  for (int v = 0; v < data.input.vertex_map.rows; v++) {
    for (int u = u_start; u < u_stop; u++) {
      const float range = data.input.range_image.at<InputData::RangeType>(v, u);
      if (range <= 0.f || range > config.max_range) {
        continue;
      }

      const auto& vertex = data.input.vertex_map.at<InputData::VertexType>(v, u);
      const Point p_W(vertex[0], vertex[1], vertex[2]);
      const auto block = tracking_layer.getBlockPtr(p_W);
      if (!block) {
        continue;
      }
      const VoxelIndex voxel_index = block->getVoxelIndex(p_W);

      // Add the pixel to the list (allocating block and voxel maps if needed).
      VoxelToPointsMap& voxel_map = point_map[block->index];
      Pixels& pixels = voxel_map[voxel_index];
      pixels.emplace_back(u, v);

      // If the voxel is ever-free it is used as a dynamic object seed.
      if (!block->isValidVoxelIndex(voxel_index)) {
        // NOTE(lschmid): This can rarely happen due to floating point precision if the point is on
        // the border of the block.
        continue;
      }
      const TrackingVoxel& voxel = block->getVoxel(voxel_index);
      if (voxel.ever_free) {
        voxel_seeds.emplace(block->getGlobalVoxelIndex(voxel_index));
      }
    }
  }
}

MeasurementClusters FreeSpaceMotionDetector::clusterDynamicVoxels(
    const BlockToPointsMap& point_map,
    const GlobalIndexSet& voxel_seeds,
    const size_t voxels_per_side) const {
  Timer timer("motion_detection/clustering", processing_stamp_);

  MeasurementClusters result;
  GlobalIndexSet closed_set;
  const spatial_hash::NeighborSearch search(config.neighbor_connectivity);

  for (const GlobalIndex& seed : voxel_seeds) {
    if (closed_set.find(seed) != closed_set.end()) {
      continue;
    }
    GlobalIndices stack = {seed};
    MeasurementCluster cluster;

    while (!stack.empty()) {
      // Get the voxel.
      const GlobalIndex global_index = stack.back();
      stack.pop_back();

      // Check if it is already processed.
      if (closed_set.find(global_index) != closed_set.end()) {
        continue;
      }
      closed_set.insert(global_index);

      // Compute block and voxel index.
      const VoxelKey key = spatial_hash::keyFromGlobalIndex(global_index, voxels_per_side);

      // Add voxel to cluster.
      auto it = point_map.find(key.first);  // Block index.
      if (it == point_map.end()) {
        continue;
      }
      auto it2 = it->second.find(key.second);  // Voxel index.
      if (it2 == it->second.end()) {
        continue;
      }
      cluster.pixels.insert(cluster.pixels.end(), it2->second.begin(), it2->second.end());
      cluster.voxels.emplace(global_index);

      // Extend cluster to neighbor voxels.
      for (const auto& neighbor_index : search.neighborIndices(global_index)) {
        const VoxelKey neighbor_key =
            spatial_hash::keyFromGlobalIndex(neighbor_index, voxels_per_side);
        if (voxel_seeds.find(neighbor_index) != voxel_seeds.end()) {
          // The voxel is also a ever-free so continue clustering.
          stack.push_back(neighbor_index);
        } else {
          // If neighbor voxel is occupied add it ot the cluster.
          auto it = point_map.find(neighbor_key.first);
          if (it != point_map.end()) {
            auto it2 = it->second.find(neighbor_key.second);
            if (it2 != it->second.end()) {
              cluster.pixels.insert(cluster.pixels.end(), it2->second.begin(), it2->second.end());
              cluster.voxels.emplace(neighbor_index);
              closed_set.insert(neighbor_index);
            }
          }
        }
      }
    }
    result.emplace_back(cluster);
  }
  return result;
}

void FreeSpaceMotionDetector::mergeClusters(MeasurementClusters& clusters) const {
  Timer timer("motion_detection/merge_clusters", processing_stamp_);

  // Comput exhaustive overlap to merge multiple overlapping clusters correctly.
  const size_t num_clusters = clusters.size();
  Eigen::MatrixXi overlap(num_clusters, num_clusters);
  overlap.setZero();
  for (size_t i = 0; i < num_clusters; ++i) {
    for (size_t j = i + 1; j < num_clusters; ++j) {
      if (checkClusterOverlap(clusters[i], clusters[j])) {
        overlap(i, j) = 1;
        overlap(j, i) = 1;
      }
    }
  }

  // Merge clusters that have overlap.
  std::vector<bool> merged(num_clusters, false);
  std::vector<bool> clusters_to_keep(num_clusters, false);
  size_t current = 0;

  while (true) {
    if (current >= num_clusters) {
      break;
    }
    if (merged[current]) {
      current++;
      continue;
    }
    // Find all connected clusters.
    std::unordered_set<int> indices = getConnectedClusters(overlap, merged, current);
    for (int i : indices) {
      mergeClusters(clusters[i], clusters[current]);
    }
    clusters_to_keep[current] = true;
    current++;
  }

  // Erase merged clusters.
  current = 0;
  for (auto it = clusters.begin(); it != clusters.end();) {
    if (clusters_to_keep[current]) {
      ++it;
    } else {
      it = clusters.erase(it);
    }
    current++;
  }
}

std::unordered_set<int> FreeSpaceMotionDetector::getConnectedClusters(
    const Eigen::MatrixXi& overlap,
    std::vector<bool>& merged,
    size_t cluster_index) const {
  std::unordered_set<int> result;
  for (int i = 0; i < overlap.rows(); ++i) {
    // Ignore clusters that are already merged.
    if (merged[i]) {
      continue;
    }
    if (overlap(cluster_index, i) == 1) {
      // For clusters that have overlap add them and all of their overlapping clusters.
      merged[i] = true;
      result.insert(i);
      result.merge(getConnectedClusters(overlap, merged, i));
    }
  }
  result.erase(cluster_index);
  return result;
}

bool FreeSpaceMotionDetector::checkClusterOverlap(const MeasurementCluster& cluster1,
                                                  const MeasurementCluster& cluster2) const {
  for (const GlobalIndex& p1 : cluster1.voxels) {
    for (const GlobalIndex& p2 : cluster2.voxels) {
      if ((p1 - p2).norm() < config.min_separation_distance) {
        return true;
      }
    }
  }
  return false;
}

void FreeSpaceMotionDetector::mergeClusters(const MeasurementCluster& cluster1,
                                            MeasurementCluster& cluster2) const {
  // NOTE(lschmid): The sets of pixels and voxels are disjoint by construction so we can
  // just insert.
  cluster2.pixels.insert(cluster2.pixels.end(), cluster1.pixels.begin(), cluster1.pixels.end());
  cluster2.voxels.insert(cluster1.voxels.begin(), cluster1.voxels.end());
}

void FreeSpaceMotionDetector::applyClusterLevelFilters(MeasurementClusters& candidates) const {
  candidates.erase(std::remove_if(candidates.begin(),
                                  candidates.end(),
                                  [this](const auto& cluster) { return filterCluster(cluster); }),
                   candidates.end());
}

bool FreeSpaceMotionDetector::filterCluster(const MeasurementCluster& cluster) const {
  // Check point count.
  const int cluster_size = static_cast<int>(cluster.pixels.size());
  if (cluster_size < config.min_cluster_size || cluster_size > config.max_cluster_size) {
    return true;
  }
  return false;
}

void FreeSpaceMotionDetector::writeClustersToData(const MeasurementClusters& clusters,
                                                  FrameData& data) const {
  data.dynamic_clusters = clusters;
  int id = 1;  // 0 is reserved for static parts.
  for (auto& cluster : data.dynamic_clusters) {
    cluster.id = id;
    for (const Pixel& pixel : cluster.pixels) {
      data.dynamic_image.at<FrameData::DynamicImageType>(pixel.v, pixel.u) = id;
    }
    if (id == 255) {
      LOG(WARNING) << "Maximum number (255) of dynamic clusters reached.";
    }
    if (id < 255) {
      id++;
    }
    cluster.bounding_box =
        BoundingBox(utils::VertexMapAdaptor(cluster.pixels, data.input.vertex_map));
  }
}

}  // namespace khronos
