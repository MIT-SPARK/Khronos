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
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <config_utilities/config_utilities.h>
#include <hydra/common/message_queue.h>
#include <hydra/common/shared_module_state.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <hydra_ros/backend/ros_backend_publisher.h>
#include <hydra_ros/frontend/ros_frontend_publisher.h>
#include <hydra_ros/input/ros_input_module.h>
#include <hydra_ros/utils/bow_subscriber.h>
#include <khronos/active_window/active_window.h>
#include <khronos/backend/backend.h>
#include <khronos/common/common_types.h>
#include <ros/node_handle.h>
#include <std_srvs/Empty.h>

#include "khronos/spatio_temporal_map/spatio_temporal_map.h"
#include "khronos_ros/visualization/active_window_visualizer.h"

namespace khronos {

using hydra::BackendInput;
using hydra::BowSubscriber;
using hydra::GraphBuilder;
using hydra::LoopClosureModule;
using hydra::RosBackendPublisher;
using hydra::RosInputModule;
using hydra::SharedDsgInfo;
using hydra::SharedModuleState;

// TODO(lschmid): Check how much this can be interfaced with the new design of the hydra pipeline.
class KhronosPipeline {
 public:
  // Config.
  struct Config : public hydra::PipelineConfig {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // If true process all remaining data in the input queue when shutting down.
    bool finish_processing_on_shutdown = false;

    // If true extract objects from the active window before saving the backend dsg.
    bool save_active_window_objects = false;

    // Module configs.
    hydra::RosInputModule::Config input;
    ActiveWindow::Config active_window;
    GraphBuilder::Config frontend;
    Backend::Config backend;
  } const config;

  // Types.
  using ActiveWindowEvaluationCallback =
      std::function<void(const VolumetricMap&, const FrameData&, const Tracks&)>;
  using BackendEvaluationCallback = std::function<
      void(TimeStamp, const DynamicSceneGraph&, const kimera_pgmo::DeformationGraph&)>;

  // Construction.
  explicit KhronosPipeline(const ros::NodeHandle& nh);
  ~KhronosPipeline() = default;

  // Interaction with the pipeline.
  void start();
  void stop();

  /**
   * @brief Saves the current state of the pipeline to the given path.
   * @param log_setup Configuration specifying where to save the different outputs to.
   * @param save_full_state If true, save the full state of the pipeline. Otherwise only save the
   * backend DSG.
   * @returns True if the save was successful, false otherwise.
   */
  bool save(const hydra::LogSetup& log_setup, bool save_full_state = true);

  // TODO(lschmid): Unify this with the above save call. These currently store much too much stuff
  // anyways.
  /**
   * @brief Save the the current spatio-temporal map to the given path.
   * @param path The path to save the map to.
   */
  bool save4DMap(const std::string& path) { return backend_->save4DMap(path); }

  /**
   * @brief Extracts all data currently in the active window and adds it to the
   * DSG.
   */
  void finishMapping();

  /**
   * @brief Optionally set an evaluation callback that is called after every
   * frame processed by the active window.
   * @param callback_function The function to be called for evaluation.
   */
  void setActiveWindowEvaluationCallback(const ActiveWindowEvaluationCallback& callback_function);

  /**
   * @brief Optionally set an evaluation callback that is called after every
   * frame processed by the backend.
   * @param callback_function The function to be called for evaluation.
   */
  void setBackendEvaluationCallback(const BackendEvaluationCallback& callback_function);

  /**
   * @brief Get the configuration of the pipeline as a string.
   */
  std::string getConfigInfo() const;

  // ROS Callbacks.
  bool finishMappingCallback(std_srvs::Empty::Request& /* req */,
                             std_srvs::Empty::Response& /* res */);

 private:
  // ROS.
  ros::NodeHandle nh_;
  ros::Publisher mesh_graph_pub_;
  ros::Publisher mesh_update_pub_;
  ros::Publisher changes_pub_;
  ros::ServiceServer finish_mapping_srv_;

  // Members.
  // Processing.
  hydra::RosInputModule::Ptr input_module_;
  std::shared_ptr<ActiveWindow> active_window_;
  std::shared_ptr<GraphBuilder> frontend_;
  std::shared_ptr<Backend> backend_;
  std::shared_ptr<LoopClosureModule> lcd_;
  std::unique_ptr<BowSubscriber> bow_subscriber_;

  // Scene Graph.
  SharedDsgInfo::Ptr frontend_dsg_;  // DSG the frontend does work on.
  SharedDsgInfo::Ptr backend_dsg_;   // DSG the backend does work on.
  SharedModuleState::Ptr shared_state_;

  // Visualization.
  ActiveWindowVisualizer map_visualizer_;

  // Callbacks.
  bool evaluate_aw_ = false;
  bool evaluate_backend_ = false;
  ActiveWindowEvaluationCallback aw_evaluation_callback_;
  BackendEvaluationCallback backend_evaluation_callback_;

  // Setup.
  void setupDsgs();
  void setupMembers();
  void setupRos();

  // Publishing.
  void sendChanges(uint64_t timestamp_ns,
                   const DynamicSceneGraph&,
                   const kimera_pgmo::DeformationGraph&) const;
};

void declare_config(KhronosPipeline::Config& config);

/**
 * @brief Initialize the configuration of the pipeline from the given node handle and verify
 * dependent paramters.
 */
KhronosPipeline::Config initializeConfig(const ros::NodeHandle& nh);

}  // namespace khronos
