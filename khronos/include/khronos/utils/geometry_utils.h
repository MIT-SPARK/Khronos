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

#include <spark_dsg/bounding_box_extraction.h>

#include "khronos/common/common_types.h"

namespace khronos::utils {

Point computeCentroid(const Points& points);

// Label for a point not assigned to any density-connected cluster.
inline constexpr int kDbscanNoise = -1;

/**
 * @brief Density-based clustering (DBSCAN) over 3D points. Two points are neighbors if their
 * Euclidean distance is <= eps; a point is a core point if it has >= min_points neighbors
 * (including itself). Clusters are formed by density-connecting core points and their neighbors.
 * @param points Input points.
 * @param eps Neighbor radius in meters.
 * @param min_points Minimum neighbors (including self) for a point to be a core point.
 * @return Per-point cluster label, same size/order as `points`. Labels are 0-indexed cluster ids;
 * points not assigned to any cluster get `kDbscanNoise`.
 */
std::vector<int> dbscan(const Points& points, float eps, int min_points);

/**
 * @brief Number of points in each cluster, indexed by 0-based cluster id. The returned vector's
 * size equals the number of clusters found; `kDbscanNoise` points are not counted.
 */
std::vector<size_t> dbscanClusterSizes(const std::vector<int>& labels);

/**
 * @brief Label of the largest cluster in a dbscan() label vector, ties broken by lowest cluster
 * id. Returns `kDbscanNoise` if no cluster is present (e.g. all points are noise).
 */
int largestDbscanClusterLabel(const std::vector<int>& labels);

/**
 * @brief Indices of every point carrying `label` in a dbscan() label vector, in ascending order.
 */
std::vector<size_t> dbscanClusterIndices(const std::vector<int>& labels, int label);

/**
 * @brief Runs dbscan() and returns the indices belonging to the single largest cluster (ties
 * broken by lowest cluster id). Returns an empty vector if no cluster is found (e.g. all points
 * are noise). Convenience composition of dbscan() + largestDbscanClusterLabel() +
 * dbscanClusterIndices().
 */
std::vector<size_t> largestDbscanCluster(const Points& points, float eps, int min_points);

/**
 * @brief Adaptor to create bounding boxes from pixels in a vertex map. Note that this assumes all
 * pixels are valid, i.e., are within the bounds of the vertex map.
 */
struct VertexMapAdaptor : public BoundingBox::PointAdaptor {
  VertexMapAdaptor(const Pixels& pixels, const cv::Mat& vertex_map);
  virtual ~VertexMapAdaptor() = default;

  // Lookup interfaces.
  size_t size() const override { return pixels.size(); }
  Eigen::Vector3f get(size_t index) const override;

  // Data references.
  const Pixels& pixels;
  const cv::Mat& vertex_map;
};

/**
 * @brief Combine all blocks in the mesh layer into a single mesh.
 */
Mesh combineMeshLayer(const MeshLayer& mesh_layer);

}  // namespace khronos::utils
