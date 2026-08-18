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

#include "khronos_ros/active_window/active_window_change_detector_publisher.h"

#include <config_utilities/config.h>
#include <glog/logging.h>
#include <rclcpp/time.hpp>

namespace khronos {
namespace {

static const auto registration =
    config::RegistrationWithConfig<ActiveWindowChangeDetector::ActiveWindowCDSink,
                                   ActiveWindowChangeDetectorPublisher,
                                   ActiveWindowChangeDetectorPublisher::Config>(
        "ActiveWindowChangeDetectorPublisher");

}

void declare_config(ActiveWindowChangeDetectorPublisher::Config& config) {
  using namespace config;
  name("ActiveWindowChangeDetectorPublisher");
  field(config.verbosity, "verbosity");
  field(config.global_frame_name, "global_frame_name");
  field(config.robot_name, "robot_name");
  field(config.topic, "topic");
  field(config.queue_size, "queue_size");
}

ActiveWindowChangeDetectorPublisher::ActiveWindowChangeDetectorPublisher(
    const Config& config,
    const ianvs::NodeHandle* nh)
    : config(config::checkValid(config)),
      nh_(nh ? *nh / "change_detector_publisher"
             : ianvs::NodeHandle::this_node("change_detector_publisher")) {
  changes_pub_ = nh_.create_publisher<khronos_msgs::msg::AwcdChanges>(config.topic, config.queue_size);
  MLOG(1) << "[ActiveWindowChangeDetectorPublisher] Initialized.";
}

void ActiveWindowChangeDetectorPublisher::call(
    const DynamicSceneGraph::Ptr& /* dsg */,
    const std::vector<ActiveWindowChangeDetector::RemovedObject>& removed_objects,
    const std::vector<ActiveWindowChangeDetector::AddedObject>& newly_added_objects,
    const Eigen::Isometry3d& current_T_prior) const {
  // Gate publishing on set-membership change: only publish if the set of removed and/or added
  // object ids differs from the last message actually published (not just from last frame's
  // detector output), so an unchanged scene does not keep re-publishing every frame.
  std::set<int64_t> cur_removed_ids;
  for (const auto& obj : removed_objects) {
    cur_removed_ids.insert(static_cast<int64_t>(obj.id));
  }
  std::set<int64_t> cur_added_ids;
  for (const auto& obj : newly_added_objects) {
    cur_added_ids.insert(static_cast<int64_t>(obj.id));
  }

  if (last_removed_ids_.has_value() && last_added_ids_.has_value() &&
      *last_removed_ids_ == cur_removed_ids && *last_added_ids_ == cur_added_ids) {
    return;
  }

  khronos_msgs::msg::AwcdChanges msg;
  msg.header.stamp = nh_.now();
  msg.header.frame_id = config.global_frame_name;
  msg.robot_name = config.robot_name;

  msg.removed_objects.reserve(removed_objects.size());
  for (const auto& obj : removed_objects) {
    msg.removed_objects.push_back(makeRemovedInfo(obj));
  }

  // Object bounding boxes / centroids are stored in the current (odom) frame; transform
  // them into the prior/report frame before publishing, mirroring the visualizer.
  const Eigen::Isometry3d prior_T_current = current_T_prior.inverse();
  msg.added_objects.reserve(newly_added_objects.size());
  for (const auto& obj : newly_added_objects) {
    msg.added_objects.push_back(makeAddedInfo(obj, prior_T_current));
  }

  changes_pub_->publish(msg);
  last_removed_ids_ = std::move(cur_removed_ids);
  last_added_ids_ = std::move(cur_added_ids);
}

khronos_msgs::msg::ChangedObjectInfo ActiveWindowChangeDetectorPublisher::makeRemovedInfo(
    const ActiveWindowChangeDetector::RemovedObject& obj) const {
  khronos_msgs::msg::ChangedObjectInfo info;
  info.id = static_cast<int64_t>(obj.id);
  // Latched at the detector: constant across every message that reports this object as removed.
  info.stamp = static_cast<builtin_interfaces::msg::Time>(
      rclcpp::Time(static_cast<int64_t>(obj.first_removed_ns)));
  info.change_confidence = obj.change_confidence;
  info.num_frames_observed = obj.num_frames_observed;
  return info;
}

khronos_msgs::msg::ChangedObjectInfo ActiveWindowChangeDetectorPublisher::makeAddedInfo(
    const ActiveWindowChangeDetector::AddedObject& obj,
    const Eigen::Isometry3d& prior_T_current) const {
  khronos_msgs::msg::ChangedObjectInfo info;
  info.id = static_cast<int64_t>(obj.id);
  info.stamp = static_cast<builtin_interfaces::msg::Time>(
      rclcpp::Time(static_cast<int64_t>(obj.first_seen)));

  const Eigen::Vector3d centroid = prior_T_current * obj.centroid.cast<double>();
  info.centroid.x = centroid.x();
  info.centroid.y = centroid.y();
  info.centroid.z = centroid.z();

  BoundingBox bbox = obj.bounding_box;
  if (bbox.isValid()) {
    bbox.transform(prior_T_current);
    info.bbox_center.x = bbox.world_P_center.x();
    info.bbox_center.y = bbox.world_P_center.y();
    info.bbox_center.z = bbox.world_P_center.z();
    info.bbox_dimensions.x = bbox.dimensions.x();
    info.bbox_dimensions.y = bbox.dimensions.y();
    info.bbox_dimensions.z = bbox.dimensions.z();
    const Eigen::Quaternionf q(bbox.world_R_center);
    info.bbox_orientation.w = q.w();
    info.bbox_orientation.x = q.x();
    info.bbox_orientation.y = q.y();
    info.bbox_orientation.z = q.z();
  }

  if (obj.semantics.has_value()) {
    info.semantic_label = obj.semantics->category_id;
  } else {
    info.semantic_label = -1;
  }
  // NOTE(multy): AddedObject only carries a numeric category_id; no label-space name lookup is
  // wired in here, so `name` is left empty. The base station can resolve it against the
  // same label space config (instance_seg_label_space.yaml) used to produce category_id.
  info.confidence = obj.confidence;
  info.change_confidence = obj.change_confidence;
  info.num_frames_observed = obj.num_frames_observed;

  return info;
}

}  // namespace khronos
