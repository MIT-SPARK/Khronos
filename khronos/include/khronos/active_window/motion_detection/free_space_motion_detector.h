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

/**
 * DISCLAIMER: The free space motion detector is taken and adapted from Dynablox:
 *
 * SOURCE: https://github.com/ethz-asl/dynablox
 *
 * PAPER: Lukas Schmid, Olov Andersson, Aurelio Sulser, Patrick Pfreundschuh, and Roland
 * Siegwart, "Dynablox: Real-time Detection of Diverse Dynamic Objects in Complex
 * Environments", in ArXiv Preprint, 2023.
 */
#pragma once

#include <memory>
#include <thread>
#include <unordered_set>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>
#include <spatial_hash/neighbor_utils.h>

#include "khronos/active_window/data/frame_data.h"
#include "khronos/active_window/motion_detection/motion_detector.h"
#include "khronos/common/common_types.h"

namespace khronos {

/**
 * @brief Geoemtric motion detector that uses the clue if voxels are free with high
 * confidence, points that fall into them must be moving. Performs additional clustering
 * and filtering to extract object detections.
 */
class FreeSpaceMotionDetector : public MotionDetector {
 public:
  // Config.
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // Number of neighbors to consider for clustering of dynamic voxels.
    int neighbor_connectivity = 26;

    // Discard clusters with fewer points than this.
    int min_cluster_size = 0;

    // Discard clusters with more points than this.
    int max_cluster_size = 1000000;

    // Merge clusters whose voxels are less than the separation distance apart in
    // voxels.
    float min_separation_distance = 1.f;

    // Maximum range within which points are considered in meters.
    float max_range = 10000.f;

    // Number of threads to use.
    int num_threads = hydra::GlobalInfo::instance().getConfig().default_num_threads;
  } const config;

  // FrameData structures processed by this module.
  // Map of voxel indices to point indices in the image.
  using VoxelToPointsMap = VoxelIndexMap<Pixels>;

  // Map of block indices to voxel indices to point indices in the image.
  using BlockToPointsMap = BlockIndexMap<VoxelToPointsMap>;

  // Construction.
  explicit FreeSpaceMotionDetector(const Config& config);
  virtual ~FreeSpaceMotionDetector() = default;

  // Inputs.
  /**
   * @brief Performs motion detection by detecting and clustering points that fall into
   * ever-free space and writes the results to the dynamic clusters and dynamic image in
   * the data.
   * @param map Map to perform voxelization and lookup.
   * @param data FrameData to write results to.
   */
  void processInput(const VolumetricMap& map, FrameData& data) override;

  // Processing functions.

  /**
   * @brief Create a mapping of all blocks and voxels that contain points to these
   * points.
   * @param data FrameData that stores the vertex map and transform to look up blocks.
   * @param tracking_layer The tracking layer to compute indices and check for dynamic
   * seeds.
   * @param point_map The output point map.
   * @param voxel_seeds The output voxel seeds.
   */
  void setUpPointMap(const FrameData& data,
                     const TrackingLayer& tracking_layer,
                     BlockToPointsMap& point_map,
                     GlobalIndexSet& voxel_seeds) const;

  void setUpPointMapPart(const FrameData& data,
                         const TrackingLayer& tracking_layer,
                         BlockToPointsMap& point_map,
                         GlobalIndexSet& voxel_seeds,
                         int u_start,
                         int u_stop) const;

  MeasurementClusters clusterDynamicVoxels(const BlockToPointsMap& point_map,
                                           const GlobalIndexSet& voxel_seeds,
                                           const size_t voxels_per_side) const;

  void mergeClusters(MeasurementClusters& clusters) const;

  /**
   * @brief Check if two clusters overlap by checking if any of their voxels are within
   * the minimum separation distance.
   * @param cluster1 First cluster.
   * @param cluster2 Second cluster.
   * @return true If the clusters overlap.
   */
  bool checkClusterOverlap(const MeasurementCluster& cluster1,
                           const MeasurementCluster& cluster2) const;

  /**
   * @brief Merge two clusters by adding the points and voxels of the first cluster to
   * the second one, retaining the first ID.
   * @param cluster1 Cluster to be merged.
   * @param cluster2 Cluster to be merged into.
   */
  void mergeClusters(const MeasurementCluster& cluster1, MeasurementCluster& cluster2) const;

  /**
   * @brief Get all yet unmerged connected clusters from a matrix of overlap between
   * clusters.
   * @param overlap Matrix of overlap between clusters.
   * @param merged Vector of booleans indicating if a cluster has been merged. Only
   * unmerged clusters are considered and bits will be set to true for clusters included
   * in the result.
   * @param cluster_index Index of the cluster to start the search from.
   * @returns Set of cluster previously unmerged indices that are connected to the
   * cluster at cluster_index.
   */
  std::unordered_set<int> getConnectedClusters(const Eigen::MatrixXi& overlap,
                                               std::vector<bool>& merged,
                                               size_t cluster_index) const;

  /**
   * @brief Apply cluster level filters to discard clusters that are too small or too
   * large.
   * @param candidates Clusters to be filtered.
   */
  void applyClusterLevelFilters(MeasurementClusters& candidates) const;

  bool filterCluster(const MeasurementCluster& cluster) const;

  void writeClustersToData(const MeasurementClusters& clusters, FrameData& data) const;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<MotionDetector,
                                     FreeSpaceMotionDetector,
                                     FreeSpaceMotionDetector::Config>("FreeSpaceMotionDetector");

  // Variables.
  TimeStamp processing_stamp_;
};

void declare_config(FreeSpaceMotionDetector::Config& config);

}  // namespace khronos
