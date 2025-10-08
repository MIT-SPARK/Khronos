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
#include <config_utilities/dynamic_config.h>
#include <hydra/common/robot_prefix_config.h>
#include <hydra_visualizer/plugins/mesh_plugin.h>
#include <hydra_visualizer/scene_graph_renderer.h>
#include <khronos/common/common_types.h>
#include <khronos/spatio_temporal_map/spatio_temporal_map.h>
#include <khronos_msgs/msg/spatio_temporal_visualizer_state.hpp>
#include <khronos_msgs/srv/spatio_temporal_visualizer_set_state.hpp>
#include <khronos_msgs/srv/spatio_temporal_visualizer_set_time_mode.hpp>
#include <khronos_msgs/srv/spatio_temporal_visualizer_setup.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace khronos {

struct DynamicVisualizerConfig {
  enum class ObjectColorMode : int {
    Semantic = 0,
    Instance = 1,
    Presence = 2
  } object_bbox_color = ObjectColorMode::Semantic;
  enum class DynamicColorMode : int {
    ID = 0,
    Red = 1,
    Mixed = 2
  } dynamic_object_color = DynamicColorMode::ID;
  double play_rate = 1.0;
  double fade_duration = 0.0;
};

void declare_config(DynamicVisualizerConfig& config);

/**
 * @brief Visualization of a 4D khronos scene belief extracted from an experiment directory via ROS.
 */
class SpatioTemporalVisualizer {
 public:
  using DynamicConfig = config::DynamicConfig<DynamicVisualizerConfig>;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Marker = visualization_msgs::msg::Marker;
  using State = khronos_msgs::msg::SpatioTemporalVisualizerState;
  using SetBool = std_srvs::srv::SetBool;
  using SetTimeMode = khronos_msgs::srv::SpatioTemporalVisualizerSetTimeMode;
  using Setup = khronos_msgs::srv::SpatioTemporalVisualizerSetup;
  using SetState = khronos_msgs::srv::SpatioTemporalVisualizerSetState;

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
    DynamicVisualizerConfig dynamic_config;

    // Config of the mesh visualizer.
    hydra::MeshPlugin::Config mesh;

    //! Config for underlying scene graph visualizer
    hydra::SceneGraphRenderer::Config scene_graph;

    // Initial settings of the dsg visualizer.
    float initial_robot_time = 1.0f;  // Percentage of time to show on initialization.
    float initial_query_time = 1.0f;  // Percentage of time to show on initialization.
    enum class TimeMode { ROBOT, QUERY, ONLINE } initial_time_mode = TimeMode::ONLINE;
  } const config;

  SpatioTemporalVisualizer(const Config& config, ianvs::NodeHandle nh);
  void draw();
  void reset();

  // Service callbacks.
  void playCb(SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res);
  void setPlayForwardCb(SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res);
  void isSetupCb(Setup::Request::SharedPtr req, Setup::Response::SharedPtr res);
  void setTimeModeCb(SetTimeMode::Request::SharedPtr req, SetTimeMode::Response::SharedPtr res);
  void setStateCb(SetState::Request::SharedPtr req, SetState::Response::SharedPtr res);

 private:
  void spinOnce();

  DynamicConfig dynamic_config_;

  // ROS.
  ianvs::NodeHandle nh_;
  ianvs::NodeHandle::Timer timer_;
  rclcpp::Publisher<State>::SharedPtr state_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr dynamic_obj_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr static_obj_pub_;
  rclcpp::Publisher<Marker>::SharedPtr agent_pub_;
  rclcpp::Service<SetBool>::SharedPtr play_srv_;
  rclcpp::Service<SetBool>::SharedPtr set_play_forward_srv_;
  rclcpp::Service<SetTimeMode>::SharedPtr set_time_mode_srv_;
  rclcpp::Service<Setup>::SharedPtr is_setup_srv_;
  rclcpp::Service<SetState>::SharedPtr set_state_srv_;

  // Stored data.
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
  std::chrono::time_point<std::chrono::steady_clock>
      previous_wall_time_;  // Time in s for play increments.
  size_t num_prev_dynamic_objects_ = 0;
  size_t num_prev_static_objects_ = 0;

  // Visuals state.
  DynamicSceneGraph::Ptr current_dsg_;

  // Members.
  hydra::MeshPlugin mesh_visualizer_;
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
  void visualizeDynamicObject(const std_msgs::msg::Header& header,
                              const KhronosObjectAttributes& attrs,
                              const size_t id,
                              MarkerArray& msg) const;
  void resetStaticObjects();
  void resetDynamicObjects();
  void drawAgent();
  void resetAgent();

  // Coloring functions.
  void recolorObjectDsgBoundingBoxes();

  void visualizeStaticObject(const std_msgs::msg::Header& header,
                             const KhronosObjectAttributes& attrs,
                             const NodeId id,
                             const Color& color,
                             MarkerArray& msg) const;

  // Tools.
  float stampToSec(const uint64_t stamp) const;
};

void declare_config(SpatioTemporalVisualizer::Config& config);

}  // namespace khronos
