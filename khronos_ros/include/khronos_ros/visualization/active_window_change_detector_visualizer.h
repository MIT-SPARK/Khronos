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

#include <filesystem>

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>
#include <hydra/utils/logging.h>
#include <hydra_visualizer/plugins/mesh_plugin.h>
#include <hydra_visualizer/scene_graph_renderer.h>
#include <hydra_visualizer/utils/marker_tracker.h>
#include <ianvs/node_handle.h>
#include <rclcpp/time.hpp>
#include <spark_dsg/dynamic_scene_graph.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "khronos/active_window/change_detection/active_window_change_detector.h"


namespace khronos {

class ActiveWindowChangeDetectorVisualizer : public ActiveWindowChangeDetector::ActiveWindowCDSink {
 public:
  struct Config : hydra::VerbosityConfig {
    // TODO(multy): after hydra is updated, should change it to also include a prefix like: "[Active
    // Window Change Detector Visualizer] "
    // int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;
    Config()
        : hydra::VerbosityConfig{hydra::GlobalInfo::instance().getConfig().default_verbosity} {}

    //! Frame in which to publish visualizations.
    std::string global_frame_name = hydra::GlobalInfo::instance().getFrames().map;

    //! Scene graph renderer config.
    hydra::SceneGraphRenderer::Config renderer;

    //! Mesh plugin config (optional - if coloring is not set, uses mesh colors).
    hydra::MeshPlugin::Config mesh;

    //! Publisher queue sizes.
    int queue_size = 10;

    //! Width in meters of lines indicating bounding boxes.
    float bounding_box_line_width = 0.1f;
  } const config;

  explicit ActiveWindowChangeDetectorVisualizer(const Config& config,
                                                const ianvs::NodeHandle* nh = nullptr);
  virtual ~ActiveWindowChangeDetectorVisualizer() = default;

  // KhronosSink callback - called each frame.
  void call(const DynamicSceneGraph::Ptr& dsg,
            const std::vector<spark_dsg::NodeId>& removed_object_ids) const override;

 private:
  void drawPriorGraph(const DynamicSceneGraph::Ptr& dsg) const;

  void visualizeChangedObjects(const DynamicSceneGraph::Ptr& dsg,
                               const std::vector<spark_dsg::NodeId>& removed_object_ids) const;

  // ROS
  ianvs::NodeHandle nh_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_bbox_pub_;

  // Renderer and plugins
  std::shared_ptr<hydra::SceneGraphRenderer> renderer_;
  std::shared_ptr<hydra::MeshPlugin> mesh_plugin_;

  // Variables
  mutable bool has_drawn_ = false;
  mutable rclcpp::Time stamp_;
  mutable bool stamp_is_set_ = false;
  mutable hydra::MarkerTracker object_bbox_tracker_;

  // Time stamp caching for synchronization of multiple visualizations.
  rclcpp::Time getStamp() const { return stamp_is_set_ ? stamp_ : nh_.now(); }
};

void declare_config(ActiveWindowChangeDetectorVisualizer::Config& config);

}  // namespace khronos
