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

#include <atomic>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/common/robot_prefix_config.h>
#include <hydra_visualizer/scene_graph_renderer.h>
#include <hydra_visualizer/utils/config_wrapper.h>
#include <khronos/common/common_types.h>
#include <khronos/spatio_temporal_map/spatio_temporal_map.h>
#include <khronos_msgs/KhronosSpatioTemporalVisConfig.h>
#include <khronos_msgs/SpatioTemporalVisualizerSetState.h>
#include <khronos_msgs/SpatioTemporalVisualizerSetTimeMode.h>
#include <khronos_msgs/SpatioTemporalVisualizerSetup.h>
#include <ros/node_handle.h>
#include <std_srvs/SetBool.h>
#include <visualization_msgs/MarkerArray.h>

#include "khronos_ros/visualization/khronos_mesh_visualizer.h"

namespace khronos {

/**
 * @brief Visualization of a 4D khronos scene belief extracted from an experiment directory via ROS.
 */
class SpatioTemporalVisualizer {
 public:
  using Coloring = std::optional<KhronosMeshVisualizer::Coloring>;
  using DynamicConfig =
      hydra::visualizer::ConfigWrapper<khronos_msgs::KhronosSpatioTemporalVisConfig>;

  // Config.
  struct Config {
    int verbosity = 0;

    // Which directory to load for visualization.
    std::string map_file = "";

    // Reference frame of all published meshes.
    std::string global_frame_name = "world";

    // Specify an optional maximum frame rate for visualization. Set <= 0 to disable.
    float max_frame_rate = 0;

    // Robot prefix for the agent to visualize.
    hydra::RobotPrefixConfig robot_prefix;

    // Initial config of the dynamic config server.
    khronos_msgs::KhronosSpatioTemporalVisConfig dynamic_config;

    // Config of the mesh visualizer.
    KhronosMeshVisualizer::Config mesh_visualizer;

    // Initial settings of the dsg visualizer.
    float initial_robot_time = 1.0f;  // Percentage of time to show on initialization.
    float initial_query_time = 1.0f;  // Percentage of time to show on initialization.
    enum class TimeMode { ROBOT, QUERY, ONLINE } initial_time_mode = TimeMode::ONLINE;
  } const config;

  explicit SpatioTemporalVisualizer(const ros::NodeHandle& nh);
  virtual ~SpatioTemporalVisualizer() = default;

  void spin();
  void draw();
  void reset();

  // Service callbacks.
  bool playCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& /* res */);
  bool setPlayForwardCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& /* res */);
  bool isSetupCb(khronos_msgs::SpatioTemporalVisualizerSetup::Request& /* req */,
                 khronos_msgs::SpatioTemporalVisualizerSetup::Response& res);
  bool setTimeModeCb(khronos_msgs::SpatioTemporalVisualizerSetTimeMode::Request& req,
                     khronos_msgs::SpatioTemporalVisualizerSetTimeMode::Response& res);
  bool setStateCb(khronos_msgs::SpatioTemporalVisualizerSetState::Request& req,
                  khronos_msgs::SpatioTemporalVisualizerSetState::Response& /* res*/);

 private:
  DynamicConfig dynamic_config_;

  // ROS.
  ros::NodeHandle nh_;
  ros::Publisher state_pub_;
  ros::Publisher dynamic_obj_pub_;
  ros::Publisher static_obj_pub_;
  ros::Publisher agent_pub_;
  ros::ServiceServer play_srv_;
  ros::ServiceServer set_play_forward_srv_;
  ros::ServiceServer set_time_mode_srv_;
  ros::ServiceServer is_setup_srv_;
  ros::ServiceServer set_state_srv_;

  // Stored data.
  const DynamicSceneGraph::LayerIds layer_ids_;
  const double min_time_delta_;  // Minimum time delta between two frames [s].

  // Threading.
  std::atomic<bool> should_shutdown_ = false;
  std::atomic<bool> playing_ = false;
  std::atomic<bool> play_forward_ = true;
  std::atomic<bool> needs_redraw_ = true;
  std::mutex mutex_;  // Any access to the visualizer state should be locked.

  // Visualizer state.
  size_t robot_time_;  // Timestamp of the current robot time.
  size_t query_time_;  // Timestamp of the current query time.
  Config::TimeMode time_mode_;

  // Visualizer state tracking for incremental updates.
  size_t prev_robot_time_ = 0;
  size_t prev_query_time_ = 0;
  double previous_wall_time_ = 0.0;  // Time in s for play increments.
  size_t num_prev_dynamic_objects_ = 0;
  size_t num_prev_static_objects_ = 0;

  // Visuals state.
  DynamicSceneGraph::Ptr current_dsg_;

  // Members.
  KhronosMeshVisualizer mesh_visualizer_;
  hydra::SceneGraphRenderer dsg_renderer_;

  // Loaded data.
  SpatioTemporalMap map_;

  // Helper functions.
  std::string printTime(TimeStamp time) const {
    std::stringstream ss;
    ss << time << " (" << std::setprecision(1) << std::fixed << (time - map_.earliest()) / 1e9
       << "s)";
    return ss.str();
  }

  // Setup.
  void loadMap();
  void setupRos();
  void initializeDynamicConfig();
  // Spinning.
  void updatePlayTime();
  // Update the dsg to be visualized according to the current robot and query time.
  void updateDsgTime();

  // Actions.
  void onConfigUpdate();

  // Visualization.
  void drawDynamicObjects();
  void visualizeDynamicObject(const std_msgs::Header& header,
                              const KhronosObjectAttributes& attrs,
                              const size_t id,
                              visualization_msgs::MarkerArray& msg) const;
  void resetStaticObjects();
  void resetDynamicObjects();
  void drawAgent();
  void resetAgent();

  // Coloring functions.
  KhronosMeshVisualizer::ObjectColors getObjectMeshColors() const;
  hydra::MeshColoring::Ptr getBackgroundMeshColoring() const;
  void recolorObjectDsgBoundingBoxes();

  void visualizeStaticObject(const std_msgs::Header& header,
                             const KhronosObjectAttributes& attrs,
                             const NodeId id,
                             const Color& color,
                             visualization_msgs::MarkerArray& msg) const;

  // Tools.
  float stampToSec(const uint64_t stamp) const;
};

void declare_config(SpatioTemporalVisualizer::Config& config);

}  // namespace khronos

namespace khronos_msgs {
void declare_config(KhronosSpatioTemporalVisConfig& config);
}  // namespace khronos_msgs
