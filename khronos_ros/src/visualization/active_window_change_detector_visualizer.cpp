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

#include "khronos_ros/visualization/active_window_change_detector_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <iomanip>
#include <sstream>

#include "khronos_ros/visualization/visualization_utils.h"

namespace khronos {
namespace {

static const auto registration =
    config::RegistrationWithConfig<ActiveWindowChangeDetector::ActiveWindowCDSink,
                                   ActiveWindowChangeDetectorVisualizer,
                                   ActiveWindowChangeDetectorVisualizer::Config>(
        "ActiveWindowChangeDetectorVisualizer");

}

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void declare_config(ActiveWindowChangeDetectorVisualizer::Config& config) {
  using namespace config;
  name("ActiveWindowChangeDetectorVisualizer");
  field(config.verbosity, "verbosity");
  field(config.global_frame_name, "global_frame_name");
  field(config.renderer, "renderer");
  field(config.mesh, "mesh");
  field(config.queue_size, "queue_size");
  field(config.bounding_box_line_width, "bounding_box_line_width");
  field(config.min_draw_period_s, "min_draw_period_s");
  field(config.draw_graph, "draw_graph");
  field(config.draw_mesh, "draw_mesh");
  field(config.draw_status, "draw_status");
  field(config.draw_status_voxels, "draw_status_voxels");
  field(config.label_text_scale, "label_text_scale", "m");
  field(config.voxel_alpha, "voxel_alpha");
  checkInRange(config.voxel_alpha, 0.0f, 1.0f, "voxel_alpha");
  // TODO(multy): add checks for the fields.
}

ActiveWindowChangeDetectorVisualizer::ActiveWindowChangeDetectorVisualizer(
    const Config& config,
    const ianvs::NodeHandle* nh)
    : config(config::checkValid(config)),
      nh_(nh ? *nh / "change_detector_visualizer"
             : ianvs::NodeHandle::this_node("change_detector_visualizer")) {
  // Initialize renderer and mesh plugin
  if (config.draw_graph) {
    renderer_ = std::make_shared<hydra::SceneGraphRenderer>(config.renderer, nh_);
  }
  if (config.draw_mesh) {
    mesh_plugin_ = std::make_shared<hydra::MeshPlugin>(config.mesh, nh_, "prior_mesh");
  }
  has_drawn_ = false;
  last_draw_time_ = std::nullopt;

  // Initialize publishers
  object_bbox_pub_ =
      nh_.create_publisher<MarkerArray>("changed_object_bounding_boxes", config.queue_size);
  status_pub_ = nh_.create_publisher<MarkerArray>("change_detection/status", config.queue_size);
  status_voxels_pub_ =
      nh_.create_publisher<MarkerArray>("change_detection/voxels", config.queue_size);
  MLOG(1) << "[ActiveWindowChangeDetectorVisualizer] Initialized.";
}

void ActiveWindowChangeDetectorVisualizer::call(
    const DynamicSceneGraph::Ptr& dsg,
    const std::vector<ActiveWindowChangeDetector::RemovedObject>& removed_objects,
    const std::vector<ActiveWindowChangeDetector::AddedObject>& newly_added_objects,
    const Eigen::Isometry3d& current_T_prior,
    const ActiveWindowChangeDetector::ChangeDetectionStatus& status) const {
  // Throttle visualization redraws to reduce flicker from high-frequency calls.
  if (config.min_draw_period_s > 0.0) {
    const rclcpp::Time now = nh_.now();
    if (last_draw_time_.has_value() && (now - *last_draw_time_).seconds() < config.min_draw_period_s) {
      return;
    }
    last_draw_time_ = now;
  }

  drawPriorGraph(dsg);

  // print out removed object ids
  MLOG(3) << "[ActiveWindowChangeDetectorVisualizer] Object ";
  for (const auto& obj : removed_objects) {
    MLOG(3) << spark_dsg::NodeSymbol(obj.id).str() << ", ";
  }
  MLOG(3) << " are removed";
  // set stamps for all visualizations
  stamp_ = rclcpp::Time(0);
  stamp_is_set_ = true;

  // Visualize removed objects
  visualizeChangedObjects(dsg, removed_objects);
  visualizeAddedObjects(newly_added_objects, current_T_prior);
  if (config.draw_status) {
    visualizeRemovedStatus(dsg, status);
  }
  if (config.draw_status_voxels) {
    visualizeRemovedVoxels(status, current_T_prior);
  }
  stamp_is_set_ = false;
}

void ActiveWindowChangeDetectorVisualizer::drawPriorGraph(const DynamicSceneGraph::Ptr& dsg) const {
  if (!dsg) {
    MLOG(2) << "[ChangeDetectorVisualizer] No prior graph provided, skipping draw";
    return;
  }
  if (!renderer_ && !mesh_plugin_) {
    return;
  }

  // Only redraw if renderer has changes or we haven't drawn yet. The prior graph is static, so
  // without a renderer it is drawn once.
  if (has_drawn_ && (!renderer_ || !renderer_->hasChange())) {
    MLOG(3) << "[ChangeDetectorVisualizer] Prior graph already drawn and no changes detected, "
               "skipping draw";
    return;
  }

  MLOG(2) << "[ChangeDetectorVisualizer] Drawing prior graph";

  std_msgs::msg::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = rclcpp::Time(0);

  if (renderer_) {
    renderer_->draw(header, *dsg);
    renderer_->clearChangeFlag();
  }
  if (mesh_plugin_) {
    mesh_plugin_->draw(header, *dsg);
  }
  has_drawn_ = true;
}

void ActiveWindowChangeDetectorVisualizer::visualizeChangedObjects(
    const DynamicSceneGraph::Ptr& dsg,
    const std::vector<ActiveWindowChangeDetector::RemovedObject>& removed_objects) const {
  if (object_bbox_pub_->get_subscription_count() == 0u) {
    return;
  }

  // draw red bounding boxes for removed objects
  MarkerArray new_markers;
  new_markers.markers.reserve(removed_objects.size());

  std_msgs::msg::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = getStamp();
  for (const auto& obj : removed_objects) {
    const auto& object_node = dsg->getNode(obj.id);
    const auto* khronos_attrs = object_node.tryAttributes<KhronosObjectAttributes>();
    if (!khronos_attrs || !khronos_attrs->bounding_box.isValid()) {
      continue;
    }
    auto& marker = new_markers.markers.emplace_back(setBoundingBox(
        khronos_attrs->bounding_box, Color(255, 0, 0, 255), header, config.bounding_box_line_width));
    // Use stable identity-based id (lower 32 bits of NodeId) so the same object
    // always gets the same marker id across frames. Namespace "removed_objects"
    // avoids collision with green added-object markers on the same publisher.
    marker.ns = "removed_objects";
    marker.id = static_cast<int>(obj.id & 0xffffffff);
  }

  MarkerArray msg;
  object_bbox_tracker_.add(new_markers, msg);
  object_bbox_tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    object_bbox_pub_->publish(msg);
  }
}

void ActiveWindowChangeDetectorVisualizer::visualizeAddedObjects(
    const std::vector<ActiveWindowChangeDetector::AddedObject>& newly_added_objects,
    const Eigen::Isometry3d& current_T_prior) const {
  if (object_bbox_pub_->get_subscription_count() == 0u) {
    return;
  }

  // Transform object bounding boxes (in current/odom frame) into map frame for RViz.
  const Eigen::Isometry3d prior_T_current = current_T_prior.inverse();

  MarkerArray new_markers;
  new_markers.markers.reserve(newly_added_objects.size());

  std_msgs::msg::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = getStamp();
  for (const auto& obj : newly_added_objects) {
    MLOG(2) << "[ActiveWindowChangeDetectorVisualizer] Visualizing newly added object with id "
            << obj.id << " (" << obj.member_track_ids.size() << " merged track(s))";
    BoundingBox bbox = obj.bounding_box;
    if (!bbox.isValid()) {
      continue;
    }
    bbox.transform(prior_T_current);
    auto& marker = new_markers.markers.emplace_back(
        setBoundingBox(bbox, Color(0, 255, 0, 255), header, config.bounding_box_line_width));
    // Use the stable, detector-owned object id (independent of any Track::id) as marker id.
    // Namespace "added_objects" avoids collision with red removed-object markers on the same publisher.
    marker.ns = "added_objects";
    marker.id = obj.id;
  }

  MarkerArray msg;
  added_object_bbox_tracker_.add(new_markers, msg);
  added_object_bbox_tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    object_bbox_pub_->publish(msg);
  }
}

namespace {

using RemovedCandidateStatus = ActiveWindowChangeDetector::RemovedCandidateStatus;

const Color kRemovedColor(255, 0, 0, 255);
//! Muted amber: stands out against the gray floor without being as bright as pure yellow.
const Color kCandidateColor(230, 180, 40, 255);
const Color kFreeColor(0, 255, 0, 255);
const Color kOccupiedColor(255, 0, 0, 255);
const Color kUnknownColor(128, 128, 128, 255);
//! Alpha of candidates that failed the in-bounds gate.
const uint8_t kOutOfBoundsAlpha = 80;

int markerId(spark_dsg::NodeId id) { return static_cast<int>(id & 0xffffffff); }

std::string removedLabel(const RemovedCandidateStatus& candidate) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << spark_dsg::NodeSymbol(candidate.id).str() << (candidate.removed ? " REMOVED" : "")
     << " p=" << candidate.free_probability << " n=" << candidate.num_frames_observed;
  if (!candidate.has_geometry) {
    ss << "\nno mesh (centroid in bounds)";
    return ss.str();
  }

  const float in_bounds_fraction =
      candidate.num_voxels > 0
          ? static_cast<float>(candidate.num_voxels_in_bounds) / candidate.num_voxels
          : 0.0f;
  ss << "\ninb " << candidate.num_voxels_in_bounds << "/" << candidate.num_voxels << " ("
     << in_bounds_fraction << ")";
  if (!candidate.in_bounds) {
    ss << " out of bounds";
  } else if (!candidate.measured) {
    ss << " no known samples";
  } else {
    ss << " raw=" << candidate.raw_ratio << " free/known " << candidate.num_free << "/"
       << candidate.num_known;
  }
  return ss.str();
}

}  // namespace

void ActiveWindowChangeDetectorVisualizer::visualizeRemovedStatus(
    const DynamicSceneGraph::Ptr& dsg,
    const ActiveWindowChangeDetector::ChangeDetectionStatus& status) const {
  if (!dsg || status_pub_->get_subscription_count() == 0u) {
    return;
  }

  std_msgs::msg::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = getStamp();

  MarkerArray new_markers;
  for (const auto& candidate : status.removed_candidates) {
    const auto* node = dsg->findNode(candidate.id);
    const auto* attrs = node ? node->tryAttributes<spark_dsg::SemanticNodeAttributes>() : nullptr;
    if (!attrs || !attrs->bounding_box.isValid()) {
      continue;
    }
    const auto& bbox = attrs->bounding_box;

    Color color = candidate.removed ? kRemovedColor : kCandidateColor;
    if (!candidate.in_bounds) {
      color.a = kOutOfBoundsAlpha;
    }

    // Prior boxes are in the prior (map) frame, like the red removed boxes.
    auto& box = new_markers.markers.emplace_back(
        setBoundingBox(bbox, color, header, config.bounding_box_line_width));
    box.ns = "removed_status";
    box.id = markerId(candidate.id);

    auto& label = new_markers.markers.emplace_back();
    label.header = header;
    label.type = Marker::TEXT_VIEW_FACING;
    label.action = Marker::ADD;
    label.ns = "removed_labels";
    label.id = markerId(candidate.id);
    label.scale.z = config.label_text_scale;
    label.color = setColor(color);
    label.pose.orientation.w = 1.0;
    label.pose.position = setPoint(
        bbox.world_P_center +
        Point(0, 0, bbox.dimensions.z() / 2.0f + config.label_text_scale * 1.5f));
    label.text = removedLabel(candidate);
  }

  MarkerArray msg;
  status_tracker_.add(new_markers, msg);
  status_tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    status_pub_->publish(msg);
  }
}

void ActiveWindowChangeDetectorVisualizer::visualizeRemovedVoxels(
    const ActiveWindowChangeDetector::ChangeDetectionStatus& status,
    const Eigen::Isometry3d& current_T_prior) const {
  if (status_voxels_pub_->get_subscription_count() == 0u) {
    return;
  }

  std_msgs::msg::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = getStamp();

  // Voxels are map voxels in the current frame. Placing the marker at prior_T_current draws them
  // exactly as the detector looked them up, rotated into the prior (map) frame.
  const Eigen::Isometry3d prior_T_current = current_T_prior.inverse();
  const Eigen::Quaterniond prior_R_current(prior_T_current.linear());
  const uint8_t alpha = static_cast<uint8_t>(config.voxel_alpha * 255.0f);

  MarkerArray new_markers;
  for (const auto& candidate : status.removed_candidates) {
    auto& marker = new_markers.markers.emplace_back();
    marker.header = header;
    marker.type = Marker::CUBE_LIST;
    marker.action = Marker::ADD;
    marker.ns = "removed_voxels";
    marker.id = markerId(candidate.id);
    marker.scale = setScale(status.voxel_size);
    marker.color = setColor(kUnknownColor, alpha);
    marker.pose.position.x = prior_T_current.translation().x();
    marker.pose.position.y = prior_T_current.translation().y();
    marker.pose.position.z = prior_T_current.translation().z();
    marker.pose.orientation.w = prior_R_current.w();
    marker.pose.orientation.x = prior_R_current.x();
    marker.pose.orientation.y = prior_R_current.y();
    marker.pose.orientation.z = prior_R_current.z();

    const auto add_voxels = [&](const std::vector<GlobalIndex>& voxels, const Color& color) {
      const auto voxel_color = setColor(color, alpha);
      for (const auto& voxel : voxels) {
        marker.points.push_back(
            setPoint(spatial_hash::centerPointFromIndex(voxel, status.voxel_size)));
        marker.colors.push_back(voxel_color);
      }
    };
    add_voxels(candidate.free_voxels, kFreeColor);
    add_voxels(candidate.occupied_voxels, kOccupiedColor);
    add_voxels(candidate.unknown_voxels, kUnknownColor);
  }

  // Markers without points (e.g. collect_debug_voxels off) are deleted by the tracker.
  MarkerArray msg;
  status_voxels_tracker_.add(new_markers, msg);
  status_voxels_tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    status_voxels_pub_->publish(msg);
  }
}

}  // namespace khronos
