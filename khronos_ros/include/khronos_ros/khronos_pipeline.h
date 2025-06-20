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

#include <hydra_ros/hydra_ros_pipeline.h>
#include <ianvs/node_handle.h>
#include <khronos/active_window/active_window.h>
#include <khronos/backend/backend.h>
#include <khronos/common/common_types.h>
#include <khronos_msgs/msg/changes.hpp>

namespace khronos {

class KhronosPipeline : public hydra::HydraRosPipeline {
 public:
  struct Config : public hydra::HydraRosPipeline::Config {
    //! Overrides defaults in Hydra for modules and other settings
    Config();
  } const config;

  // Types.
  using ActiveWindowEvaluationCallback =
      std::function<void(const VolumetricMap&, const FrameData&, const Tracks&)>;
  using BackendEvaluationCallback = std::function<
      void(TimeStamp, const DynamicSceneGraph&, const kimera_pgmo::DeformationGraph&)>;

  explicit KhronosPipeline(ianvs::NodeHandle nh);
  ~KhronosPipeline() = default;

  void init() override;

  /**
   * @brief Saves the current state of the pipeline to the given path.
   * @param log_setup Configuration specifying where to save the different outputs to.
   * @param save_full_state If true, save the full state of the pipeline. Otherwise only save the
   * backend DSG.
   * @returns True if the save was successful, false otherwise.
   */
  bool save(const hydra::DataDirectory& log_setup, bool save_full_state = true);

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

 private:
  // ROS.
  ianvs::NodeHandle nh_;
  rclcpp::Publisher<khronos_msgs::msg::Changes>::SharedPtr changes_pub_;

  Backend* khronos_backend_;
  ActiveWindow* khronos_active_window_;

  // Callbacks.
  ActiveWindowEvaluationCallback aw_evaluation_callback_;
  BackendEvaluationCallback backend_evaluation_callback_;
};

void declare_config(KhronosPipeline::Config& config);

}  // namespace khronos
