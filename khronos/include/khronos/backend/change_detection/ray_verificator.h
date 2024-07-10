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

#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>
#include <hydra/common/robot_prefix_config.h>
#include <spatial_hash/grid.h>

#include "khronos/common/common_types.h"

namespace khronos {

/**
 * @brief Utility class that checks if a point has been observed to be absent by comparing it to a
 * set of deformable rays stored in the DSG.
 */
class RayVerificator {
 public:
  // Types.
  using Ptr = std::shared_ptr<RayVerificator>;
  using ConstPtr = std::shared_ptr<const RayVerificator>;
  using IndexSet = std::unordered_set<size_t>;
  using SpatialHash = BlockIndexMap<IndexSet>;

  // Config.
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // Size of the blocks used for background hashing in meters. This is independent of the mesh
    // block size.
    float block_size = 1.f;

    // Maximum distance to the ray to cound as occlusion in meters.
    float radial_tolerance = 0.1f;

    // Maximum depth difference within which points are considered to be the same in meters.
    float depth_tolerance = 0.1f;

    // Time stamps to raycast for verification.
    // NOTE(lschmid): Could add uniform, random, all (that'd be expensive though).
    enum class RayPolicy {
      kFirst,
      kLast,
      kFirstAndLast,
      kMiddle,
      kAll,
      kRandom,
      kRandom3,
    } ray_policy = RayPolicy::kMiddle;

    // Time to subtract from the latest time stamp to compensat the Active Window in s.
    float active_window_duration = 0.f;

    // Robot prefix to use for the ray verificator. TODO(lschmid): This should probably be moved to
    // the query.
    hydra::RobotPrefixConfig prefix;
  } const config;

  // Construction.
  explicit RayVerificator(const Config& config);
  virtual ~RayVerificator() = default;

  // Processing.
  struct CheckResult {
    // Timestamps of all rays that marked the point as absent of present.
    std::vector<uint64_t> absent;
    std::vector<uint64_t> present;

    // Merge other check results into this one.
    void merge(const CheckResult& other) {
      absent.insert(absent.end(), other.absent.begin(), other.absent.end());
      present.insert(present.end(), other.present.begin(), other.present.end());
    }
  };

  // Extra details about the check that can be optionally requested for visualization.
  struct CheckDetails {
    Points start;
    Points end;
    std::vector<float> range;
    enum class Result { kNoOverlap, kOccludded, kAbsent, kMatch };
    std::vector<Result> result;
  };

  /**
   * @brief Checks if a point is observed to be present or absent in the given time window.
   * @param point The 3D position in world frame to check.
   * @param earliest Only consder measurments that are at least this old, as nanosecond stamp.
   * @param latest Only consder measurments that are at most this old, as nanosecond stamp.
   * @param details Optional pointer to a CheckDetails struct to fill with details about the check.
   * @return The result of the check.
   */
  CheckResult check(const Point& point,
                    const uint64_t earliest = 0ul,
                    const uint64_t latest = std::numeric_limits<uint64_t>::max(),
                    CheckDetails* details = nullptr) const;

  /**
   * @brief Setup the ray verificator to check absence on a given DynamicSceneGraph. This
   * precomputes all required properties to perform 'check()' globally, overwriting all previous
   * data in the ray verificator.
   * @param dsg The DynamicSceneGraph to use for ray verification.
   */
  void setDsg(std::shared_ptr<const DynamicSceneGraph> dsg);

  /**
   * @brief Incrementally add new measurements of a single growing dynamic scene graph to the ray
   * verificator. This assumes that the initially set DSG only adds new poses and vertices. Past
   * poses and vertices may change but not be removed. Use 'setDsg()' to change the underlying DSG.
   */
  void updateDsg();

  /**
   * @brief Recompute the spatial hashing function used to query points. This may not be required
   * for small deformations of the DSG.
   */
  void recomputeHash();

  // Accessors.
  const Config& getConfig() const { return config; }

  /**
   * @brief Get the set of vertices that were re-observed during the last update.
   */
  const IndexSet& getReobservedVertices() const { return reobserved_vertices_; }

  /**
   * @brief Get the set of objects that were re-observed during the last update.
   */
  const IndexSet& getReobservedObjects() const { return reobserved_objects_; }

 private:
  struct Ray {
    Ray() = default;
    Ray(const uint64_t timestamp, const size_t source_index, const size_t target_index)
        : timestamp(timestamp), source_index(source_index), target_index(target_index) {}

    // Time stamp of the measurement.
    uint64_t timestamp;

    // NOTE(lschmid): Because of how DSGs and PCL store the points, these need to be an indices.
    // Index to the position of the sensor at the measurement (ray source point).
    size_t source_index;

    // Index of the vertex associated with this measurement (ray target points).
    size_t target_index;
  };

  // Data to lookup ray queries.
  std::vector<Ray> rays_;

  // Time stamps and pointers to all source points. These are sorted by timestamp.
  std::vector<uint64_t> timestamps_;

  // Spatial Hash.
  spatial_hash::Grid<BlockIndex> grid_;
  SpatialHash block_seen_by_rays_;  // Map which blocks were seen by which rays. Stores the index to
                                    // the ray.
  SpatialHash vertices_in_block_;   // Map which vertices are contained in which blocks.
  SpatialHash objects_in_block_;    // Map which objects are contained in which blocks.

  // Cached data.
  std::shared_ptr<const DynamicSceneGraph> dsg_;
  unsigned int seed_;

  // Variables.
  // The index of the last incrementally added measurement to add all new indices.
  size_t previous_node_index_ = 0;
  size_t previous_vertex_index_ = 0;
  size_t previous_object_index_ = 0;

  // Newly (during last update) re-observed vertices and objects for change detection. These are set
  // during 'updateDsg()'.
  IndexSet reobserved_vertices_;
  IndexSet reobserved_objects_;

  // Helper functions.
  // Allocate all new nodes as potential sources for rays.
  void addPoseNodes();

  // Add all new vertices as rays. Requires that addPoseNodes is called first. Returns the observed
  // blocks.
  BlockIndexSet addVertices();

  // Compute for a single vertex which sources it is associated with.
  std::unordered_set<size_t> computeVertexSources(const size_t first_seen, const size_t last_seen);

  // Add a single ray to the hash. Returns the observed blocks.
  BlockIndexSet addRayToHash(const size_t ray_index);

  void addObjectsToHash();

  // Struct to lookup rays for a given dsg somewhat efficiently.
  struct RayLookup {
    RayLookup(const DynamicSceneGraph& dsg, const Config& config)
        : vertices_(dsg.mesh()->points),
          nodes_(dsg.getLayer(DsgLayers::AGENTS, config.prefix.key).nodes()) {}

    Point getSource(const Ray& ray) const {
      return nodes_.at(ray.source_index)
          ->attributes<spark_dsg::NodeAttributes>()
          .position.cast<float>();
    }

    Point getTarget(const Ray& ray) const { return vertices_.at(ray.target_index); }

   private:
    const std::vector<spark_dsg::Mesh::Pos>& vertices_;
    const spark_dsg::DynamicSceneGraphLayer::Nodes& nodes_;
  };
};

void declare_config(RayVerificator::Config& config);

}  // namespace khronos
