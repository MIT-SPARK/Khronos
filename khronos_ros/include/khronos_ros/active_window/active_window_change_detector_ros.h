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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "khronos/active_window/change_detection/active_window_change_detector.h"

namespace khronos {

/**
 * @brief ROS-aware subclass of ActiveWindowChangeDetector that uses TF2 to obtain
 * the current_T_prior initial guess automatically after hydra_multi processes a
 * ROMAN loop closure and publishes the updated map→{robot}/odom transform.
 *
 * Loaded as a plugin into hydra_ros_node via `type: ActiveWindowChangeDetectorRos`
 * in the khronos_sinks list.  No modifications to existing pipeline code required.
 */
class ActiveWindowChangeDetectorRos : public ActiveWindowChangeDetector {
 public:
  struct Config : ActiveWindowChangeDetector::Config {
    //! Frame ID of the prior map (hydra_multi world_frame, typically "map").
    std::string prior_frame_id = "map";
    //! Robot odometry frame published by hydra_multi.
    //! If empty, falls back to hydra::GlobalInfo::instance().getFrames().odom.
    std::string robot_frame_id = "";
    //! Minimum translation change [m] before re-triggering ICP.
    double tf_change_threshold_m = 0.05;
  } const config;

  explicit ActiveWindowChangeDetectorRos(const Config& config);
  ~ActiveWindowChangeDetectorRos() override = default;

  /**
   * @brief Looks up the latest map→robot_frame TF, checks for significant change,
   * calls notifyLoopClosure() if needed, then invokes the parent call().
   */
  void call(const FrameData& data,
            const VolumetricMap& map,
            const Tracks& tracks) const override;

 private:
  //! Returns robot_frame_id from config if set, otherwise GlobalInfo odom frame.
  std::string getRobotFrame() const;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable Eigen::Isometry3d last_tf_guess_ = Eigen::Isometry3d::Identity();
  mutable bool has_last_tf_ = false;
};

void declare_config(ActiveWindowChangeDetectorRos::Config& config);

}  // namespace khronos
