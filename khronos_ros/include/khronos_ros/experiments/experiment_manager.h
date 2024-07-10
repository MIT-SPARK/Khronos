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
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <khronos/active_window/data/frame_data.h>
#include <khronos/active_window/data/reconstruction_types.h>

#include "khronos/utils/data_directory.h"
#include "khronos_ros/experiments/experiment_logger.h"
#include "khronos_ros/khronos_pipeline.h"

namespace khronos {

class ExperimentManager {
 public:
  // Config.
  struct Config {
    int verbosity = 4;

    // Directory to save the output. Leave empty to not save.
    std::string output_dir = "";

    // True: replace the existing output directory if it exists. False: Create a time
    // stamped sub-directory.
    bool overwrite = false;

    // If true save the config of all modules.
    bool log_config = true;

    // If true save the timing.
    bool log_timing = true;

    // If true save the elapsed times of all timers.
    bool log_timing_details = true;

    // Save the current map state every N frames. 0 to disable. The 4D map is always saved at the
    // end.
    int save_every_n_frames = 0;

    // If true save the full state of Khronos every N frames. Otherwise only save the backend DSG.
    bool save_full_state = false;
  } const config;

  // Types.
  using ActiveWindowEvaluationCallback = KhronosPipeline::ActiveWindowEvaluationCallback;
  using BackendEvaluationCallback = KhronosPipeline::BackendEvaluationCallback;

  // Construction.
  ExperimentManager(const ros::NodeHandle& nh, std::shared_ptr<KhronosPipeline> khronos);
  virtual ~ExperimentManager();

  // Run the experiment.
  void run();

  // Service for clean termination.
  bool finishMappingAndSaveCallback(std_srvs::Empty::Request& /* req */,
                                    std_srvs::Empty::Response& /* res */);

  /**
   * @brief To be called after an experiment to write all collected data to disk.
   */
  void logData();

  /**
   * @brief Stores all rosparams and validated config_utilities configs to disk. Resets the
   * config_utilities global buffer.
   */
  void logConfigs();

  // Perform evaluations every frame.
  void evaluateFrontend(const VolumetricMap& map, const FrameData& data, const Tracks& tracks);
  void evaluateBackend(TimeStamp timestamp_ns,
                       const DynamicSceneGraph& dsg,
                       const kimera_pgmo::DeformationGraph& deformation_graph);

  // Add a callback to be executed every N frames.
  void addFrontendCallback(size_t every_n_frames, ActiveWindowEvaluationCallback callback);
  void addBackendCallback(size_t every_n_frames, BackendEvaluationCallback callback);

  // Access.
  std::string getOutputDir() const { return data_dir_; }
  bool evaluationIsOn() const { return data_dir_; }

  // Evaluation callbacks.
  void saveMap(size_t timestamp);

  // Utility tools.
  // Execute a shell command and return the output.
  bool exec(const char* cmd, std::string& output);

 private:
  // Manages the allocation of the output directory.
  const DataDirectory data_dir_;

  // Pointer to the khronos pipeline to manage the experiment for.
  const std::shared_ptr<KhronosPipeline> khronos_;

  // ROS members.
  ros::NodeHandle nh_;
  ros::ServiceServer finish_mapping_and_save_srv_;

  // Experiment logger to write experiment progress to disk.
  ExperimentLogger::Ptr logger_;

  // Execute these evaluation callbacks every N frames.
  std::vector<std::pair<int, ActiveWindowEvaluationCallback>> frontend_callbacks_;
  std::vector<int> frontend_callback_counters_;
  std::vector<std::pair<int, BackendEvaluationCallback>> backend_callbacks_;
  std::vector<int> backend_callback_counters_;

  // Variables.
  int num_saved_maps_ = 0;
  uint64_t last_frontend_time_stamp_ = 0;
  uint64_t last_backend_time_stamp_ = 0;
  bool final_map_saved_ = false;
};

void declare_config(ExperimentManager::Config& config);

}  // namespace khronos
