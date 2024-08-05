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

#include "khronos_ros/visualization/dsg_visualizer_plugins/khronos_object_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <spark_dsg/colormaps.h>

#include "khronos_ros/visualization/visualization_utils.h"

namespace khronos {

namespace colormaps = spark_dsg::colormaps;

void declare_config(KhronosObjectPlugin::Config& config) {
  using namespace config;
  name("KhronosObjectPlugin");
  field(config.queue_size, "queue_size");
  enum_field(config.color_mode, "color_mode", {"ID", "SEMANTIC", "ATTRIBUTE"});
  field(config.id_color_revolutions, "id_color_revolutions");
  check(config.queue_size, GE, 0, "queue_size");
  check(config.id_color_revolutions, GT, 0, "id_color_revolutions");
}

KhronosObjectPlugin::KhronosObjectPlugin(const Config& config,
                                         const ros::NodeHandle& nh,
                                         const std::string& name)
    : VisualizerPlugin(nh, name), config(config::checkValid(config)) {
  traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory", config.queue_size, true);
}

void KhronosObjectPlugin::draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) {
  if (traj_pub_.getNumSubscribers() == 0) {
    return;
  }

  // For dynamic objects show the start and end bbox with a line for the trajectory.
  const auto& objects = graph.getLayer(DsgLayers::OBJECTS);
  visualization_msgs::MarkerArray msg;
  std::unordered_set<uint64_t> present_objects;

  for (const auto& [node_id, node] : objects.nodes()) {
    const uint64_t id = spark_dsg::NodeSymbol(node_id).categoryId();
    const auto& attrs = node->attributes<KhronosObjectAttributes>();
    if (attrs.trajectory_positions.empty()) {
      continue;
    }

    present_objects.emplace(id);

    auto it = previous_objects_.find(id);
    if (it != previous_objects_.end() && it->second.first == attrs.trajectory_positions.front() &&
        it->second.second == attrs.trajectory_positions.back()) {
      // The object has not changed, no need to update.
      continue;
    }

    // Publish the trajectory. This overwrites previous messages.
    addObjectMarkers(header, attrs, id, msg);

    // Track start and end positions.
    if (it != previous_objects_.end()) {
      it->second.first = attrs.trajectory_positions.front();
      it->second.second = attrs.trajectory_positions.back();
    } else {
      previous_objects_.emplace(
          id,
          std::make_pair(attrs.trajectory_positions.front(), attrs.trajectory_positions.back()));
    }
  }

  // Clear objects that are no longer present.
  for (auto it = previous_objects_.begin(); it != previous_objects_.end();) {
    if (present_objects.find(it->first) == present_objects.end()) {
      addClearingMarkers(header, it->first, msg);
      it = previous_objects_.erase(it);
    } else {
      ++it;
    }
  }
  traj_pub_.publish(msg);
}

void KhronosObjectPlugin::reset(const std_msgs::Header& header) {
  // Clear all objects.
  visualization_msgs::MarkerArray msg;
  for (const auto& id_pair : previous_objects_) {
    addClearingMarkers(header, id_pair.first, msg);
  }
  traj_pub_.publish(msg);
  previous_objects_.clear();

  // TODO(lschmid): Should draw all objects here.
}

void KhronosObjectPlugin::addObjectMarkers(const std_msgs::Header& header,
                                           const KhronosObjectAttributes& attrs,
                                           const uint64_t id,
                                           visualization_msgs::MarkerArray& msg) const {
  BoundingBox bbox = attrs.bounding_box;
  const Color color = getColor(attrs, id);
  bbox.world_P_center = attrs.trajectory_positions.front();
  auto& marker = msg.markers.emplace_back(setBoundingBox(bbox, color, header));
  marker.id = id;
  marker.ns = "start_bbox";

  bbox.world_P_center = attrs.trajectory_positions.back();
  auto& marker2 = msg.markers.emplace_back(setBoundingBox(bbox, color, header));
  marker2.id = id;
  marker2.ns = "end_bbox";

  // Add the trajectory.
  visualization_msgs::Marker& line = msg.markers.emplace_back();
  line.action = visualization_msgs::Marker::ADD;
  line.color = marker.color;
  line.header = header;
  line.id = id;
  line.ns = "trajectory";
  line.type = visualization_msgs::Marker::LINE_LIST;
  line.points.reserve(attrs.trajectory_positions.size());
  line.scale.x = 0.05;
  line.pose.orientation.w = 1.f;
  for (size_t i = 1; i < attrs.trajectory_positions.size(); ++i) {
    line.points.emplace_back(setPoint(attrs.trajectory_positions[i - 1]));
    line.points.emplace_back(setPoint(attrs.trajectory_positions[i]));
  }
}

void KhronosObjectPlugin::addClearingMarkers(const std_msgs::Header& header,
                                             const uint64_t id,
                                             visualization_msgs::MarkerArray& msg) const {
  // Add a delete marker for each namespace.
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.id = id;
  marker.ns = "trajectory";
  marker.action = visualization_msgs::Marker::DELETE;
  msg.markers.emplace_back(marker);
  marker.ns = "start_bbox";
  msg.markers.emplace_back(marker);
  marker.ns = "end_bbox";
  msg.markers.emplace_back(marker);
}

Color KhronosObjectPlugin::getColor(const KhronosObjectAttributes& attrs, const uint64_t id) const {
  switch (config.color_mode) {
    case Config::ColorMode::ID:
      return colormaps::rainbowId(id, config.id_color_revolutions);
    case Config::ColorMode::SEMANTIC:
      return colormaps::rainbowId(attrs.semantic_label, config.id_color_revolutions);
    case Config::ColorMode::ATTRIBUTE:
      return attrs.color;
    default:
      return Color::gray();
  }
}

}  // namespace khronos
