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

  // Initialize publishers
  object_bbox_pub_ =
      nh_.create_publisher<MarkerArray>("changed_object_bounding_boxes", config.queue_size);
  MLOG(1) << "[ActiveWindowChangeDetectorVisualizer] Initialized.";
}

void ActiveWindowChangeDetectorVisualizer::call(
    const DynamicSceneGraph::Ptr& dsg,
    const std::vector<spark_dsg::NodeId>& removed_object_ids) const {
  drawPriorGraph(dsg);

  // print out removed object ids
  MLOG(1) << "[ActiveWindowChangeDetectorVisualizer] Object ";
  for (const auto& id : removed_object_ids) {
    MLOG(1) << spark_dsg::NodeSymbol(id).str() << ", ";
  }
  MLOG(1) << " are removed";

  // set stamps for all visualizations
  stamp_ = nh_.now();
  stamp_is_set_ = true;

  // Visualize removed objects
  visualizeChangedObjects(dsg, removed_object_ids);
  stamp_is_set_ = false;
}

void ActiveWindowChangeDetectorVisualizer::drawPriorGraph(const DynamicSceneGraph::Ptr& dsg) const {
  if (!dsg) {
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
  header.stamp = nh_.now();

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

  // Get all removed object bounding boxes form the scene graph attributes
  std::vector<BoundingBox> removed_object_bboxes;
  for (const auto& id : removed_object_ids) {
    const auto& object_node = dsg->getNode(id);
    const auto* khronos_attrs = object_node.tryAttributes<KhronosObjectAttributes>();
    if (khronos_attrs) {
      removed_object_bboxes.push_back(khronos_attrs->bounding_box);
    } else {
      MLOG(2) << "[ActiveWindowChangeDetectorVisualizer] Could not find KhronosObjectAttributes "
                 "for removed object "
              << spark_dsg::NodeSymbol(id).str();
    }
  }

  // draw red bounding boxes for removed objects
  MarkerArray new_markers;
  new_markers.markers.reserve(removed_object_bboxes.size());

  size_t id = 0u;
  std_msgs::msg::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = getStamp();
  for (const auto& bbox : removed_object_bboxes) {
    if (bbox.isValid()) {
      auto& marker = new_markers.markers.emplace_back(
          setBoundingBox(bbox, Color(255, 0, 0, 255), header, config.bounding_box_line_width));
      marker.id = id++;
    }
  }

  MarkerArray msg;
  object_bbox_tracker_.add(new_markers, msg);
  object_bbox_tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    object_bbox_pub_->publish(msg);
  }
}

}  // namespace khronos
