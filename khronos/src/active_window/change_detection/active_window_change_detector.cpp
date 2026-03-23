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

#include "khronos/utils/icp_registration_utils.h"

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
  field(config.transformation_getter, "transformation_getter");
  field(config.enable_icp_refinement, "enable_icp_refinement");
  field(config.invert_roman_lc_transform, "invert_roman_lc_transform");
  field(config.icp_crop_radius, "icp_crop_radius");
  field(config.icp_num_threads, "icp_num_threads");
  field(config.icp_downsampling_resolution, "icp_downsampling_resolution");
  field(config.icp_max_correspondence_distance, "icp_max_correspondence_distance");
  field(config.icp_min_inliers, "icp_min_inliers");
  check<Path::Exists>(config.prior_map_path, "prior_map_path");
  //   check<Path::Extension>(config.prior_map_path, "prior_map_path", ".spark_dsg"); // Add the
  //   check back sometimes.
}

ActiveWindowChangeDetector::ActiveWindowChangeDetector(const Config& config)
    : config(config::checkValid(config)),
      transformation_getter_(config.transformation_getter.create()),
      sinks_(ActiveWindowCDSink::instantiate(config.awcd_sinks)) {
  MLOG(1) << "[Khronos Active Window Change Detector] Initialized with prior map path: "
          << config.prior_map_path;
  loadPriorMap();
  MLOG(1) << "[Khronos Active Window Change Detector] Loaded prior scene graph with "
          << (prior_graph_ ? std::to_string(prior_graph_->numNodes()) + " nodes." : "0 nodes.");
  // print out the config
  MLOG(1) << "[Khronos Active Window Change Detector] Config: " << config;
}

void ActiveWindowChangeDetector::addKhronosSink(const ActiveWindowCDSink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

void ActiveWindowChangeDetector::call(const FrameData& data,
                                      const VolumetricMap& map,
                                      const Tracks& tracks) const {
  // Poll the transformation getter and update current_T_prior_ if a new transform is available.
  const auto tf = transformation_getter_->getTransformation();
  if (tf.has_value()) {
    if (config.enable_icp_refinement) {
      runIcpRefinement(data, map, tf.value());
    } else {
      setCurrentToPriorTransform(tf.value());
    }
  }

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
      MLOG(2) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(object_id).str()
              << " has empty mesh, skipping";
      continue;
    }

    int num_vertices_in_free_space = 0;
    int num_vertices_in_bound_and_known = 0;

    // 3. For each vertex in bound and not unknown, check if it's in free space
    for (size_t i = 0; i < mesh.numVertices(); ++i) {
      // Transform: local → prior_world → current_world
      const Eigen::Vector3f vertex_local = mesh.pos(i);
      const Eigen::Vector3f vertex_prior_world = bbox.pointToWorldFrame(vertex_local);
      const Point vertex_current = transformPriorToCurrentFrame(vertex_prior_world.cast<double>());

      if (!isPointKnown(vertex_current, map)) {
        continue;  // Skip unknown points
      }

      ++num_vertices_in_bound_and_known;

      if (isPriorPointFree(vertex_current, map)) {
        ++num_vertices_in_free_space;
      }
    }

    // 4. Compute ratio of vertices in bound and known (?) in free space
    const float free_ratio = static_cast<float>(num_vertices_in_free_space) /
                             static_cast<float>(num_vertices_in_bound_and_known);

    // 5. If more than threshold % of vertices are in free space, mark as removed
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
  MLOG(2) << "[ActiveWindowChangeDetector] Detected " << removed_object_ids.size()
          << " removed objects out of " << objects_id_in_bounds.size() << " checked";

  // 6. Call all sinks with the removed objects
  ActiveWindowCDSink::callAll(sinks_, prior_graph_, removed_object_ids, current_T_prior_);
}

void ActiveWindowChangeDetector::loadPriorMap() {
  // NOTE(multy): required to load the full path to the DSG file.
  // TODO(multy): in documentation might require to set a prior map path that's different from the
  // DCIST env variable.
  prior_graph_ = DynamicSceneGraph::load(config.prior_map_path);
  MLOG(1) << "[ActiveWindowChangeDetector] Loaded prior graph from " << config.prior_map_path
          << " with "
          << (prior_graph_ ? std::to_string(prior_graph_->numNodes()) + " nodes." : "0 nodes.");
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

bool ActiveWindowChangeDetector::isPointKnown(const Point& point_in_map,
                                              const VolumetricMap& map) const {
  const auto* voxel = map.getTrackingLayer()->getVoxelPtr(point_in_map);
  return voxel && voxel->last_observed != 0u;
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

  MLOG(2) << "[ActiveWindowChangeDetector] Found " << objects_in_bounds.size()
          << " prior objects within current map bounds";

  return objects_in_bounds;
}

void ActiveWindowChangeDetector::setCurrentToPriorTransform(
    const Eigen::Isometry3d& current_T_prior) const {
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

void ActiveWindowChangeDetector::runIcpRefinement(const FrameData& data,
                                                  const VolumetricMap& map,
                                                  const Eigen::Isometry3d& initial) const {
  if (!prior_graph_ || !prior_graph_->mesh() || prior_graph_->mesh()->points.empty()) {
    LOG(WARNING) << "[ActiveWindowChangeDetector] No prior background mesh for ICP.";
    return;
  }

  const Eigen::Vector3f robot_cur = data.input.world_T_body.translation().cast<float>();
  const Eigen::Isometry3f prior_T_current = initial.inverse().cast<float>();
  const Eigen::Vector3f robot_prior = prior_T_current * robot_cur;
  const float r = config.icp_crop_radius;

  // Collect current mesh points and pre-transform to prior frame.
  std::vector<Eigen::Vector3f> source;
  for (const auto& block : map.getMeshLayer()) {
    for (const auto& pt : block.points) {
      if ((pt - robot_cur).norm() <= r) {
        source.push_back(prior_T_current * pt);
      }
    }
  }

  // Collect prior background mesh points near robot in prior frame.
  std::vector<Eigen::Vector3f> target;
  for (const auto& pt : prior_graph_->mesh()->points) {
    if ((pt - robot_prior).norm() <= r) {
      target.push_back(pt);
    }
  }

  if (source.empty() || target.empty()) {
    LOG(WARNING) << "[ActiveWindowChangeDetector] Insufficient mesh points for ICP.";
    return;
  }
  MLOG(1) << "[ActiveWindowChangeDetector] ICP: " << source.size() << " src, " << target.size()
          << " tgt pts.";

  const auto res =
      ICPRegistrationUtils::registerPointClouds(source,
                                                target,
                                                config.icp_num_threads,
                                                config.icp_downsampling_resolution,
                                                config.icp_max_correspondence_distance);

  if (!res.converged || res.num_inliers < config.icp_min_inliers) {
    LOG(WARNING) << "[ActiveWindowChangeDetector] ICP failed: converged=" << res.converged
                 << " inliers=" << res.num_inliers;
    return;
  }

  setCurrentToPriorTransform(initial * res.T_target_source.inverse());
  MLOG(1) << "[ActiveWindowChangeDetector] ICP refined transform. Inliers: " << res.num_inliers
          << ", translation delta: "
          << (current_T_prior_.translation() - initial.translation()).norm() << " m.";

  // print initial guess and refined transform for debugging in x,y,z and euler angles x,y,z
  const Eigen::Vector3d initial_trans = initial.translation();
  const Eigen::Vector3d initial_euler = initial.rotation().eulerAngles(0, 1, 2);
  const Eigen::Vector3d refined_trans = current_T_prior_.translation();
  const Eigen::Vector3d refined_euler = current_T_prior_.rotation().eulerAngles(0, 1, 2);

  MLOG(1) << "[ActiveWindowChangeDetector] Initial guess - Translation (x,y,z): "
          << initial_trans.transpose() << ", Euler angles (x,y,z): " << initial_euler.transpose();
  MLOG(1) << "[ActiveWindowChangeDetector] Refined transform - Translation (x,y,z): "
          << refined_trans.transpose() << ", Euler angles (x,y,z): " << refined_euler.transpose();
}

}  // namespace khronos
