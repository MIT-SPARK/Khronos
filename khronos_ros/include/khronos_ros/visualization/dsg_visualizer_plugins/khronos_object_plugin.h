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

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra_visualizer/plugins/visualizer_plugin.h>
#include <khronos/common/common_types.h>
#include <visualization_msgs/MarkerArray.h>

namespace khronos {

class KhronosObjectPlugin : public hydra::VisualizerPlugin {
 public:
  // Config.
  struct Config {
    // Queue size for publishing.
    int queue_size = 100;

    // How to color the mesh of objects that are being visualized.
    enum class ColorMode { ID, SEMANTIC, ATTRIBUTE } color_mode = ColorMode::ID;

    // Number of color revolutions used to recolor integer IDs.
    int id_color_revolutions = 10;
  } const config;

  // Construction.
  KhronosObjectPlugin(const Config& config, const ros::NodeHandle& nh, const std::string& name);

  // Implement visualization interfaces.
  void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;
  void reset(const std_msgs::Header& header) override;

 protected:
  // Helper functions.
  // Add start and end bounding box and trajectory line to the msg.
  void addObjectMarkers(const std_msgs::Header& header,
                        const KhronosObjectAttributes& attrs,
                        const uint64_t id,
                        visualization_msgs::MarkerArray& msg) const;
  void addClearingMarkers(const std_msgs::Header& header,
                          const uint64_t id,
                          visualization_msgs::MarkerArray& msg) const;
  Color getColor(const KhronosObjectAttributes& attrs, const uint64_t id) const;

 private:
  // ROS.
  ros::Publisher traj_pub_;
  ros::Publisher bbox_pub_;

  // Variables.
  // Keep track of changed objects. id -> (num_vertices, num_faces)
  std::unordered_map<uint64_t, std::pair<Point, Point>> previous_objects_;

  // Registration.
  inline static const auto registration_ =
      config::RegistrationWithConfig<hydra::VisualizerPlugin,
                                     KhronosObjectPlugin,
                                     KhronosObjectPlugin::Config,
                                     ros::NodeHandle,
                                     std::string>("KhronosObjectPlugin");
};

void declare_config(KhronosObjectPlugin::Config& config);

}  // namespace khronos
