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

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>
#include <hydra/reconstruction/volumetric_map.h>
#include <spatial_hash/neighbor_utils.h>

#include "khronos/active_window/data/frame_data.h"
#include "khronos/active_window/object_detection/object_detector.h"
#include "khronos/common/common_types.h"

namespace khronos {

/**
 * @brief Simple semantic object detector that computes connected semantic components in
 * the image or 3D voxel space as object detection clusters.
 */
class ConnectedSemantics : public ObjectDetector {
 public:
  // Config.
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // Number of neighbors to consider for region growing. In image space this reflects
    // 8 (if true) and 4 (if false) neighbors. In 3D voxel space this reflects 26 (if
    // true) and 6 (if false) neighbors.
    bool use_full_connectivity = true;

    // Discard clusters with fewer pixels than this.
    int min_cluster_size = 0;

    // Discard clusters with more pixels than this (<= 0 disables this check).
    int max_cluster_size = -1;

    // True: Cluster pixels in voxel space, false: cluster pixels in image space.
    bool use_3d = true;

    // Grid size in meters used for 3D voxel-based clustering.
    float grid_size = 0.1f;

    // Maximum depth values to consider for object extraction in meters. Use 0 for infinity.
    float max_range = 0.f;
  } const config;

  // Types.
  using VoxelPixelMap = GlobalIndexMap<Pixels>;
  using SemanticVoxelPixelMaps = std::map<int, VoxelPixelMap>;

  // Construction.
  explicit ConnectedSemantics(const Config& config);
  virtual ~ConnectedSemantics() = default;

  // Inputs.

  /**
   * @brief Performs semantic object extraction by grouping the semantic input image
   * into connected components.
   * @param map Volumetric map used for voxel-based clustering.
   * @param data Frame data to read input images and write results to.
   */
  void processInput(const VolumetricMap& /* map */, FrameData& data) override;

  // Processing.

  /**
   * @brief Performs semantic object extraction by grouping the semantic input image
   * into connected components in the 3d voxel space.
   * @param data Frame data to read input images and write results to.
   * @param grid_size Size of the voxel grid used for voxel-based clustering.
   */
  void semanticClustering3D(FrameData& data, const float grid_size);

  /**
   * @brief Extract all pixels that have a sematic label of an object class and group
   * the minto voxels.
   * @param data Frame data to read semantic and vertex images from.
   * @param voxel_size_inv Inverse of the voxel size of the clustering grid.
   * @returns Map from semantic id to maps of voxel index to a list of pixels in that
   * voxel.
   */
  SemanticVoxelPixelMaps computeCandidateVoxels(const FrameData& data,
                                                const float voxel_size_inv) const;

  /**
   * @brief Performs semantic object extraction by grouping the semantic input image
   * into connected components in the image space.
   * @param data Frame data to read input images and write results to.
   */
  void semanticClustering2D(FrameData& data);

  /**
   * @brief Performs region growing starting on the pixel at (u,v) in the semantic
   * image. Adds the resulting cluster to the clusters in the data and labels the object
   * image accordingly.
   * @param u Column of the pixel to start growing from.
   * @param v Row of the pixel to start growing from.
   * @param data FrameData to write results to.
   */
  void growCluster2D(const int u, const int v, FrameData& data) const;

  /**
   * Erase all clusters with fewer pixels than min_cluster_size.
   * @param data Frame data to filter clusters in.
   */
  void filterClusters(FrameData& data) const;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<ObjectDetector,
                                     ConnectedSemantics,
                                     ConnectedSemantics::Config>("ConnectedSemantics");

  const spatial_hash::NeighborSearch neighbor_search_;

  // Variables.
  TimeStamp processing_stamp_;
};

void declare_config(ConnectedSemantics::Config& config);

}  // namespace khronos
