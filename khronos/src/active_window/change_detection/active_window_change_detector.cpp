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

#include "khronos/active_window/change_detection/active_window_change_detector.h"

#include <config_utilities/types/path.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph_types.h>

namespace khronos {
namespace {

static const auto registration = config::RegistrationWithConfig<ActiveWindow::KhronosSink,
                                                                ActiveWindowChangeDetector,
                                                                ActiveWindowChangeDetector::Config>(
    "ActiveWindowChangeDetector");
}

void declare_config(ActiveWindowChangeDetector::Config& config) {
  using namespace config;
  name("ActiveWindowChangeDetector");
  field(config.verbosity, "verbosity");
  field(config.removal_vertex_free_ratio_threshold, "removal_vertex_free_ratio_threshold");
  field<Path::Absolute>(config.prior_map_path, "prior_map_path");
  field(config.awcd_sinks, "awcd_sinks");
  check<Path::Exists>(config.prior_map_path, "prior_map_path");
  //   check<Path::Extension>(config.prior_map_path, "prior_map_path", ".spark_dsg"); // Add the
  //   check back sometimes.
}

ActiveWindowChangeDetector::ActiveWindowChangeDetector(const Config& config)
    : config(config::checkValid(config)),
      sinks_(ActiveWindowCDSink::instantiate(config.awcd_sinks)) {
  MLOG(1) << "[Khronos Active Window Change Detector] Initialized with prior map path: "
          << config.prior_map_path;
  loadPriorMap();
  MLOG(1) << "[Khronos Active Window Change Detector] Loaded prior scene graph with "
          << (prior_graph_ ? std::to_string(prior_graph_->numNodes()) + " nodes." : "0 nodes.");
}

void ActiveWindowChangeDetector::addKhronosSink(const ActiveWindowCDSink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

void ActiveWindowChangeDetector::call(const FrameData& data,
                                      const VolumetricMap& map,
                                      const Tracks& tracks) const {
  // 1. Find all object nodes in the prior graph within current volumetric map bounds
  const auto objects_id_in_bounds = findPriorObjectsInMapBounds(map);

  std::vector<spark_dsg::NodeId> removed_object_ids;

  // 2. For each object node, get mesh vertices and transform to current frame
  for (const auto object_id : objects_id_in_bounds) {
    const auto& object_node = prior_graph_->getNode(object_id);

    // Cast to KhronosObjectAttributes to access mesh
    const auto* khronos_attrs = object_node.tryAttributes<KhronosObjectAttributes>();
    if (!khronos_attrs) {
      MLOG(2) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(object_id).str()
              << " does not have KhronosObjectAttributes, skipping";
      continue;
    }

    const auto& mesh = khronos_attrs->mesh;
    const auto& bbox = khronos_attrs->bounding_box;

    if (mesh.numVertices() == 0) {
      continue;
    }

    int num_vertices_in_free_space = 0;

    // 3. For each vertex, check if it's in free space
    for (size_t i = 0; i < mesh.numVertices(); ++i) {
      // Transform: local → prior_world → current_world
      const Eigen::Vector3f vertex_local = mesh.pos(i);
      const Eigen::Vector3f vertex_prior_world = bbox.pointToWorldFrame(vertex_local);
      const Point vertex_current = transformPriorToCurrentFrame(vertex_prior_world.cast<double>());

      if (isPriorPointFree(vertex_current, map)) {
        ++num_vertices_in_free_space;
      }
    }

    // 4. If more than threshold % of vertices are in free space, mark as removed
    const float free_ratio =
        static_cast<float>(num_vertices_in_free_space) / static_cast<float>(mesh.numVertices());

    if (free_ratio >= config.removal_vertex_free_ratio_threshold) {
      removed_object_ids.push_back(object_id);
      MLOG(2) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(object_id).str()
              << " detected as REMOVED (free ratio: " << free_ratio << ")";
    } else {
      MLOG(2) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(object_id).str()
              << " still present (free ratio: " << free_ratio << ")";
    }
  }

  // 5. TODO (multy): need ways to report the problem or even visualize it.
  MLOG(1) << "[ActiveWindowChangeDetector] Detected " << removed_object_ids.size()
          << " removed objects out of " << objects_id_in_bounds.size() << " checked";

  // 6. Call all sinks with the removed objects
  ActiveWindowCDSink::callAll(sinks_, prior_graph_, removed_object_ids);
}

void ActiveWindowChangeDetector::loadPriorMap() {
  // NOTE(multy): required to load the full path to the DSG file.
  // TODO(multy): in documentation might require to set a prior map path that's different from the
  // DCIST env variable.
  prior_graph_ = DynamicSceneGraph::load(config.prior_map_path);
}

bool ActiveWindowChangeDetector::isPriorPointFree(const Point& point_in_map,
                                                  const VolumetricMap& map) const {
  const auto* voxel = map.getTrackingLayer()->getVoxelPtr(point_in_map);
  return voxel && voxel->ever_free;
}

bool ActiveWindowChangeDetector::isPointInMapBounds(const Point& point,
                                                    const VolumetricMap& map) const {
  // A point is "in bounds" if the map has an allocated block at that location
  const auto& tsdf_layer = map.getTsdfLayer();
  return tsdf_layer.hasBlock(point.cast<float>());
}

std::vector<spark_dsg::NodeId> ActiveWindowChangeDetector::findPriorObjectsInMapBounds(
    const VolumetricMap& map) const {
  std::vector<spark_dsg::NodeId> objects_in_bounds;

  if (!prior_graph_ || !prior_graph_->hasLayer(DsgLayers::OBJECTS)) {
    LOG(WARNING) << "[ActiveWindowChangeDetector] Prior graph has no OBJECTS layer";
    return objects_in_bounds;
  }

  const auto& objects_layer = prior_graph_->getLayer(DsgLayers::OBJECTS);

  for (const auto& [node_id, node] : objects_layer.nodes()) {
    const auto& attrs = node->attributes();
    // Transform position from prior map frame to current map frame
    const Point position_in_current = transformPriorToCurrentFrame(attrs.position);

    if (isPointInMapBounds(position_in_current, map)) {
      objects_in_bounds.push_back(node_id);
      MLOG(3) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(node_id).str()
              << " is within map bounds at position (current frame): "
              << position_in_current.transpose();
    }
  }

  MLOG(1) << "[ActiveWindowChangeDetector] Found " << objects_in_bounds.size()
          << " prior objects within current map bounds";

  return objects_in_bounds;
}

void ActiveWindowChangeDetector::setCurrentToPriorTransform(
    const Eigen::Isometry3d& current_T_prior) {
  current_T_prior_ = current_T_prior;
  MLOG(1) << "[ActiveWindowChangeDetector] Updated current_T_prior transform:\n"
          << "  Translation: " << current_T_prior_.translation().transpose() << "\n"
          << "  Rotation (quaternion wxyz): "
          << Eigen::Quaterniond(current_T_prior_.rotation()).coeffs().transpose();
}

Point ActiveWindowChangeDetector::transformPriorToCurrentFrame(
    const Eigen::Vector3d& point_in_prior) const {
  // TODO(multy): In the future, consider transforming the prior map once to the current map frame
  // instead of transforming each point.
  // Transform: current_point = current_T_prior * prior_point
  const Eigen::Vector3d point_in_current = current_T_prior_ * point_in_prior;
  return point_in_current.cast<float>();
}

}  // namespace khronos
