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

#include <filesystem>
#include <vector>

#include "khronos/active_window/active_window.h"

namespace khronos {

class ActiveWindowChangeDetector : public ActiveWindow::KhronosSink {
 public:
  // Config.
  struct Config {
    //! Verbosity level.
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    //! Path to prior map to use for change detection.
    std::filesystem::path prior_map_path;

    //! Ratio of free prior map points of an object's vertices to consider it removed.
    float removal_vertex_free_ratio_threshold = 0.8f;
  } const config;

  // Construction.
  explicit ActiveWindowChangeDetector(const Config& config);
  virtual ~ActiveWindowChangeDetector() = default;

  /**
   * @brief TODO(multy): documentation
   * @param map The current volumetric map that the active window is building.
   * @param data The current data after processing.
   * @param tracks The current tracks in the active window.
   */
  void call(const FrameData& data, const VolumetricMap& map, const Tracks& tracks) const override;

  void loadPriorMap();

  bool isPriorPointFree(const Point& point_in_map, const VolumetricMap& map) const;

  /**
   * @brief Check if a point is within allocated map bounds (has an allocated block).
   * @param point The point to check in world frame.
   * @param map The volumetric map to check against.
   * @return True if the point is within an allocated block of the map.
   */
  bool isPointInMapBounds(const Point& point, const VolumetricMap& map) const;

  /**
   * @brief Find all prior object nodes whose centroid is within the current volumetric map bounds.
   * @param map The current volumetric map.
   * @return Vector of node IDs for objects within map bounds.
   */
  std::vector<spark_dsg::NodeId> findPriorObjectsInMapBounds(const VolumetricMap& map) const;

  /**
   * @brief Set the transform from prior map frame to current map frame.
   * @param current_T_prior Transform that converts points from prior map frame to current frame.
   */
  void setCurrentToPriorTransform(const Eigen::Isometry3d& current_T_prior);

  /**
   * @brief Transform a point from prior map frame to current map frame.
   * @param point_in_prior Point in the prior map's world frame.
   * @return Point transformed to the current map's world frame.
   */
  Point transformPriorToCurrentFrame(const Eigen::Vector3d& point_in_prior) const;

 private:
  //! Prior map as a 3D scene graph.
  DynamicSceneGraph::Ptr prior_graph_;

  //! Transform from prior map frame to current map frame.
  //! current_point = current_T_prior_ * prior_point
  Eigen::Isometry3d current_T_prior_ = Eigen::Isometry3d::Identity();
};

void declare_config(ActiveWindowChangeDetector::Config& config);

}  // namespace khronos
