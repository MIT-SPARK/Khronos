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

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra_visualizer/plugins/visualizer_plugin.h>
#include <hydra_visualizer/utils/config_wrapper.h>
#include <khronos/backend/change_state.h>
#include <khronos/common/common_types.h>
#include <khronos_msgs/Changes.h>
#include <khronos_msgs/KhronosMeshPluginConfig.h>

#include "khronos_ros/visualization/khronos_mesh_visualizer.h"

namespace khronos {

class KhronosMeshPlugin : public hydra::VisualizerPlugin {
 public:
  // Config that is const and loaded.
  struct Config {
    int verbosity = 4;

    // TODO(lschmid): Move all of this to the scene graph.
    // Topic to subscribe to for incremental changes.
    std::string changes_topic = "/khronos_node/changes";

    // Optional: Files to load for visualization of static (saved) DSGs.
    std::string background_changes_file;
    std::string object_changes_file;

    // Config of the mesh visualizer.
    KhronosMeshVisualizer::Config mesh_visualizer;
  } const config;

  using DynamicConfig = hydra::visualizer::ConfigWrapper<khronos_msgs::KhronosMeshPluginConfig>;

  // Construction.
  KhronosMeshPlugin(const Config& config, const ros::NodeHandle& nh, const std::string& name);

  // Implement visualization interfaces.
  void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;
  void reset(const std_msgs::Header& header) override;
  bool hasChange() const override { return has_changes_; }
  void clearChangeFlag() override { has_changes_ = false; }

  // ROS.
  void changesCallback(const khronos_msgs::Changes& msg);

 protected:
  // Helper functions.
  void computeTimingData(const DynamicSceneGraph& dsg);
  void publishBackgroundMesh(const DynamicSceneGraph& dsg);
  void publishObjectMesh(const DynamicSceneGraph& dsg);
  hydra::MeshColoring::Ptr getBackgroundColoring(const DynamicSceneGraph& dsg) const;

 private:
  DynamicConfig dynamic_config_;
  KhronosMeshVisualizer mesh_visualizer_;

  // Variables.
  bool has_changes_ =
      false;  // Whether the plugin needs redrawing, not related to khronos change detection.

  // Time Tracking for viualization.
  TimeStamp time_start_;
  TimeStamp time_end_;
  TimeStamp current_query_time_;

  // The khronos change state.
  Changes changes_;

  // ROS.
  ros::Subscriber changes_sub_;

  // Registration.
  inline static const auto registration_ =
      config::RegistrationWithConfig<hydra::VisualizerPlugin,
                                     KhronosMeshPlugin,
                                     KhronosMeshPlugin::Config,
                                     ros::NodeHandle,
                                     std::string>("KhronosMeshPlugin");
};

void declare_config(KhronosMeshPlugin::Config& config);

Color colorFromChangeState(ChangeState state);

struct ChangeMeshColoring : public hydra::MeshColoring {
  explicit ChangeMeshColoring(const BackgroundChanges& changes) : changes_(changes) {}
  virtual ~ChangeMeshColoring() = default;

  Color getVertexColor(const Mesh&, size_t i) const override;

 private:
  const BackgroundChanges& changes_;
};

}  // namespace khronos
