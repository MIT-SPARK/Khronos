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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <glog/logging.h>

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
  // TODO(multy): add checks for the fields.
}

ActiveWindowChangeDetectorVisualizer::ActiveWindowChangeDetectorVisualizer(
    const Config& config,
    const ianvs::NodeHandle* nh)
    : config(config::checkValid(config)),
      nh_(nh ? *nh / "change_detector_visualizer"
             : ianvs::NodeHandle::this_node("change_detector_visualizer")) {
  // Initialize renderer and mesh plugin
  renderer_ = std::make_shared<hydra::SceneGraphRenderer>(config.renderer, nh_);
  mesh_plugin_ = std::make_shared<hydra::MeshPlugin>(config.mesh, nh_, "prior_mesh");
  has_drawn_ = false;
  last_draw_time_ = std::nullopt;

  // Initialize publishers
  object_bbox_pub_ =
      nh_.create_publisher<MarkerArray>("changed_object_bounding_boxes", config.queue_size);
  tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(nh_.node());
  MLOG(1) << "[ActiveWindowChangeDetectorVisualizer] Initialized.";
}

void ActiveWindowChangeDetectorVisualizer::call(
    const DynamicSceneGraph::Ptr& dsg,
    const std::vector<spark_dsg::NodeId>& removed_object_ids,
    const std::vector<Track>& newly_added_tracks,
    const Eigen::Isometry3d& current_T_prior) const {
  // Broadcast: {robot}/odom → prior_map_frame (config.global_frame_name)
  // current_T_prior = odom_T_prior_map, so parent=odom, child=prior_map_frame.
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = nh_.now();
  tf_msg.header.frame_id = hydra::GlobalInfo::instance().getFrames().odom;
  tf_msg.child_frame_id = config.global_frame_name;
  const auto& t = current_T_prior.translation();
  tf_msg.transform.translation.x = t.x();
  tf_msg.transform.translation.y = t.y();
  tf_msg.transform.translation.z = t.z();
  const Eigen::Quaterniond q(current_T_prior.rotation());
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(tf_msg);

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
  for (const auto& id : removed_object_ids) {
    MLOG(3) << spark_dsg::NodeSymbol(id).str() << ", ";
  }
  MLOG(3) << " are removed";
  // set stamps for all visualizations
  stamp_ = rclcpp::Time(0);
  stamp_is_set_ = true;

  // Visualize removed objects
  visualizeChangedObjects(dsg, removed_object_ids);
  visualizeAddedObjects(newly_added_tracks, current_T_prior);
  stamp_is_set_ = false;
}

void ActiveWindowChangeDetectorVisualizer::drawPriorGraph(const DynamicSceneGraph::Ptr& dsg) const {
  if (!dsg) {
    MLOG(2) << "[ChangeDetectorVisualizer] No prior graph provided, skipping draw";
    return;
  }

  // Only redraw if renderer has changes or we haven't drawn yet.
  if (has_drawn_ && !renderer_->hasChange()) {
    MLOG(3) << "[ChangeDetectorVisualizer] Prior graph already drawn and no changes detected, "
               "skipping draw";
    return;
  }

  MLOG(2) << "[ChangeDetectorVisualizer] Drawing prior graph";

  std_msgs::msg::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = rclcpp::Time(0);

  renderer_->draw(header, *dsg);
  mesh_plugin_->draw(header, *dsg);
  renderer_->clearChangeFlag();
  has_drawn_ = true;
}

void ActiveWindowChangeDetectorVisualizer::visualizeChangedObjects(
    const DynamicSceneGraph::Ptr& dsg,
    const std::vector<spark_dsg::NodeId>& removed_object_ids) const {
  if (object_bbox_pub_->get_subscription_count() == 0u) {
    return;
  }

  // draw red bounding boxes for removed objects
  MarkerArray new_markers;
  new_markers.markers.reserve(removed_object_ids.size());

  std_msgs::msg::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = getStamp();
  for (size_t i = 0u; i < removed_object_ids.size(); ++i) {
    const auto& id = removed_object_ids[i];
    const auto& object_node = dsg->getNode(id);
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
    marker.id = static_cast<int>(id & 0xffffffff);
  }

  MarkerArray msg;
  object_bbox_tracker_.add(new_markers, msg);
  object_bbox_tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    object_bbox_pub_->publish(msg);
  }
}

void ActiveWindowChangeDetectorVisualizer::visualizeAddedObjects(
    const std::vector<Track>& newly_added_tracks,
    const Eigen::Isometry3d& current_T_prior) const {
  if (object_bbox_pub_->get_subscription_count() == 0u) {
    return;
  }

  // Transform track bounding boxes (in current/odom frame) into prior_map_frame for RViz.
  const Eigen::Isometry3d prior_T_current = current_T_prior.inverse();

  MarkerArray new_markers;
  new_markers.markers.reserve(newly_added_tracks.size());

  std_msgs::msg::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = getStamp();
  for (const Track& track : newly_added_tracks) {
    MLOG(2) << "[ActiveWindowChangeDetectorVisualizer] Visualizing newly added track with id " << track.id;
    BoundingBox bbox = track.last_bounding_box;
    if (!bbox.isValid()) {
      continue;
    }
    bbox.transform(prior_T_current);
    auto& marker = new_markers.markers.emplace_back(
        setBoundingBox(bbox, Color(0, 255, 0, 255), header, config.bounding_box_line_width));
    // Use stable track.id as marker id (monotonically increasing, never reused by MaxIoUTracker).
    // Namespace "added_objects" avoids collision with red removed-object markers on the same publisher.
    marker.ns = "added_objects";
    marker.id = static_cast<int>(track.id);
  }

  MarkerArray msg;
  added_object_bbox_tracker_.add(new_markers, msg);
  added_object_bbox_tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    object_bbox_pub_->publish(msg);
  }
}

}  // namespace khronos
