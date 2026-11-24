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

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include <hydra/utils/logging.h>
#include <hydra_ros/utils/tf_lookup.h>

#include "khronos/active_window/change_detection/transformation_getter.h"

namespace khronos {

/**
 * @brief TransformationGetter that obtains odom_T_prior by querying TF2.
 *
 * Polls `prior_frame_id` -> `robot_frame_id` (odom frame) in the background.
 * getTransformation() returns the cached inverse when its translation or rotation
 * changes beyond the configured thresholds.
 */
class TFTransformationGetter : public TransformationGetter {
 public:
  struct Config : hydra::VerbosityConfig {
    Config() : VerbosityConfig("tf_transform") {}

    //! Frame ID of the prior map (hydra_multi world_frame, typically "map").
    std::string prior_frame_id = "map";
    //! Robot odometry frame (empty uses the robot odom frame).
    std::string robot_frame_id = "";
    //! Minimum translation change [m] before reporting a new transform.
    double tf_change_threshold_m = 0.05;
    //! Minimum rotation change [rad] before reporting a new transform.
    double tf_change_threshold_rad = 0.05;
    /**
     * TF buffer settings. wait_duration_s is the polling interval; max_tries is
     * ignored because each poll makes one nonblocking attempt.
     */
    hydra::TFLookup::Config tf_lookup;
  } const config;

  explicit TFTransformationGetter(const Config& config, const hydra::RobotConfig& robot);
  ~TFTransformationGetter();

  /**
   * @brief Read cached TF and return odom_T_map = current_T_prior.
   * Returns nullopt on TF failure or if change is below threshold.
   */
  std::optional<Eigen::Isometry3d> getTransformation() const override;

 private:
  //! Returns robot_frame_id from config if set, otherwise the robot odom frame.
  std::string getRobotFrame() const;
  void spin();

  mutable std::mutex mutex_;
  std::condition_variable lookup_cv_;
  std::thread lookup_thread_;
  bool should_shutdown_ = false;

  hydra::PoseStatus last_status_;
  hydra::TFLookup tf_lookup_;

  mutable Eigen::Isometry3d last_reported_ = Eigen::Isometry3d::Identity();
  mutable bool has_last_ = false;
};

void declare_config(TFTransformationGetter::Config& config);

}  // namespace khronos
