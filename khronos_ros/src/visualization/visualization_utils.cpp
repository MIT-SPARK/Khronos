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

#include "khronos_ros/visualization/visualization_utils.h"

namespace khronos {

visualization_msgs::Marker setBoundingBox(const BoundingBox& bb,
                                          const Color& color,
                                          const std_msgs::Header& header,
                                          const float scale) {
  visualization_msgs::Marker msg;
  msg.header = header;
  msg.type = visualization_msgs::Marker::LINE_LIST;
  msg.scale.x = scale;
  msg.pose.orientation.w = 1.f;
  msg.pose.position.x = bb.world_P_center.x();
  msg.pose.position.y = bb.world_P_center.y();
  msg.pose.position.z = bb.world_P_center.z();
  msg.color = setColor(color);
  const Point base = -bb.dimensions / 2.0;
  const Point dx = Point(bb.dimensions.x(), 0, 0);
  const Point dy = Point(0, bb.dimensions.y(), 0);
  const Point dz = Point(0, 0, bb.dimensions.z());

  // All points of the box.
  msg.points.push_back(setPoint(base));
  msg.points.push_back(setPoint(base + dx));
  msg.points.push_back(setPoint(base));
  msg.points.push_back(setPoint(base + dy));
  msg.points.push_back(setPoint(base + dx));
  msg.points.push_back(setPoint(base + dx + dy));
  msg.points.push_back(setPoint(base + dy));
  msg.points.push_back(setPoint(base + dx + dy));

  msg.points.push_back(setPoint(base + dz));
  msg.points.push_back(setPoint(base + dx + dz));
  msg.points.push_back(setPoint(base + dz));
  msg.points.push_back(setPoint(base + dy + dz));
  msg.points.push_back(setPoint(base + dx + dz));
  msg.points.push_back(setPoint(base + dx + dy + dz));
  msg.points.push_back(setPoint(base + dy + dz));
  msg.points.push_back(setPoint(base + dx + dy + dz));

  msg.points.push_back(setPoint(base));
  msg.points.push_back(setPoint(base + dz));
  msg.points.push_back(setPoint(base + dx));
  msg.points.push_back(setPoint(base + dx + dz));
  msg.points.push_back(setPoint(base + dy));
  msg.points.push_back(setPoint(base + dy + dz));
  msg.points.push_back(setPoint(base + dx + dy));
  msg.points.push_back(setPoint(base + dx + dy + dz));

  return msg;
}

geometry_msgs::Vector3 setScale(const float scale) {
  geometry_msgs::Vector3 msg;
  msg.x = scale;
  msg.y = scale;
  msg.z = scale;
  return msg;
}

geometry_msgs::Point setPoint(const Point& point) {
  geometry_msgs::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

std_msgs::ColorRGBA setColor(const std::vector<float>& color) {
  std_msgs::ColorRGBA msg;
  msg.r = color[0];
  msg.g = color[1];
  msg.b = color[2];
  msg.a = color[3];
  return msg;
}

std_msgs::ColorRGBA setColor(const Color& color) {
  std_msgs::ColorRGBA msg;
  msg.r = static_cast<float>(color.r) / 255.f;
  msg.g = static_cast<float>(color.g) / 255.f;
  msg.b = static_cast<float>(color.b) / 255.f;
  msg.a = static_cast<float>(color.a) / 255.f;
  return msg;
}

cv::Vec3b colorToCv(const Color& color) { return cv::Vec3b(color.r, color.g, color.b); }

Color cvToColor(const cv::Vec3b& color) { return Color(color[0], color[1], color[2]); }

void applyColor(const Color& color, cv::Vec3b& pixel, float alpha) {
  pixel = colorToCv(cvToColor(pixel).blend(color, alpha));
}

void MarkerArrayTracker::updateMarkerArray(visualization_msgs::MarkerArray& marker) {
  std::unordered_map<std::string, std::unordered_set<int>> current_ids;
  for (const auto& marker : marker.markers) {
    current_ids[marker.ns].emplace(marker.id);
  }
  for (const auto& [prev_ns, prev_ids] : previous_ids_) {
    if (current_ids.count(prev_ns) == 0) {
      continue;
    }
    const auto& curr_ids = current_ids.at(prev_ns);
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    delete_marker.ns = prev_ns;
    for (const int id : prev_ids) {
      if (curr_ids.count(id)) {
        continue;
      }
      delete_marker.id = id;
      marker.markers.push_back(delete_marker);
    }

    // Reset the previous ids.
    previous_ids_[prev_ns] = curr_ids;
  }
}

visualization_msgs::MarkerArray MarkerArrayTracker::createResetMarker(
    std::vector<std::string> namespaces) {
  visualization_msgs::MarkerArray reset_markers;
  for (const auto& ns : namespaces) {
    visualization_msgs::Marker reset_marker;
    reset_marker.action = visualization_msgs::Marker::DELETEALL;
    reset_marker.ns = ns;
    reset_markers.markers.push_back(reset_marker);
    previous_ids_.erase(ns);
  }
  return reset_markers;
}

}  // namespace khronos
