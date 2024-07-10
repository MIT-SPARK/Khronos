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

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <khronos/common/common_types.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace khronos {

// Conversion utils.
geometry_msgs::Vector3 setScale(const float scale);
geometry_msgs::Point setPoint(const Point& point);
std_msgs::ColorRGBA setColor(const std::vector<float>& color);
std_msgs::ColorRGBA setColor(const Color& color);
cv::Vec3b colorToCv(const Color& color);
Color cvToColor(const cv::Vec3b& color);
void applyColor(const Color& color, cv::Vec3b& pixel, float alpha = 1.f);

// Visualization utils.
visualization_msgs::Marker setBoundingBox(const BoundingBox& bb,
                                          const Color& color,
                                          const std_msgs::Header& header,
                                          const float scale = 0.03);

// Filling annoying rviz markers with empty and reset markers.
class MarkerArrayTracker {
  MarkerArrayTracker() = default;
  virtual ~MarkerArrayTracker() = default;

  /**
   * @brief Update the marker array such that only current markers are disaplyed. This adds delete
   * markers for all ids that are not in the current marker array for all namespaces that occur in
   * the current marker array.
   */
  void updateMarkerArray(visualization_msgs::MarkerArray& marker);
  visualization_msgs::MarkerArray createResetMarker(std::vector<std::string> namespaces);

  std::unordered_map<std::string, std::unordered_set<int>> previous_ids_;
};

}  // namespace khronos
