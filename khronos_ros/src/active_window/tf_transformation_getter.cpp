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

#include "khronos_ros/active_window/tf_transformation_getter.h"

#include <chrono>

#include <Eigen/Geometry>
#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <hydra/utils/logging.h>

namespace khronos {
namespace {

static const auto registration =
    config::RegistrationWithConfig<TransformationGetter,
                                   TFTransformationGetter,
                                   TFTransformationGetter::Config,
                                   hydra::RobotConfig>("TFTransformationGetter");

}  // namespace

void declare_config(TFTransformationGetter::Config& config) {
  config::base<hydra::VerbosityConfig>(config);
  using namespace config;
  name("TFTransformationGetter");
  field(config.prior_frame_id, "prior_frame_id");
  field(config.robot_frame_id, "robot_frame_id");
  field(config.tf_change_threshold_m, "tf_change_threshold_m");
  field(config.tf_change_threshold_rad, "tf_change_threshold_rad", "rad");
  field(config.tf_lookup, "tf_lookup");
  check(config.tf_lookup.wait_duration_s, GT, 0.0, "tf_lookup.wait_duration_s");
}

TFTransformationGetter::TFTransformationGetter(const Config& cfg, const hydra::RobotConfig& robot)
    : config(config::checkValid(cfg)), tf_lookup_(config.tf_lookup, robot.frames) {
  MLOG(1, "TF listener created ({} -> {})", config.prior_frame_id, getRobotFrame());

  lookup_thread_ = std::thread(&TFTransformationGetter::spin, this);
}

TFTransformationGetter::~TFTransformationGetter() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    should_shutdown_ = true;
  }

  lookup_cv_.notify_all();
  lookup_thread_.join();
}

std::optional<Eigen::Isometry3d> TFTransformationGetter::getTransformation() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_status_) {
    HYDRA_LOG_EVERY_N(WARNING, 10, "No transformation to prior map found");
    return std::nullopt;
  }

  // status.target_T_source() is map_T_{robot}/odom; current_T_prior is its inverse
  // (odom frame is "current").
  const Eigen::Isometry3d current_T_prior = last_status_.target_T_source().inverse();

  const Eigen::Isometry3d delta = last_reported_.inverse() * current_T_prior;
  const double dt = delta.translation().norm();
  const double dr = Eigen::AngleAxisd(delta.linear()).angle();
  if (has_last_ && dt <= config.tf_change_threshold_m && dr <= config.tf_change_threshold_rad) {
    return std::nullopt;
  }

  MLOG(1, "TF changed by {} m, {} rad, reporting new transform.", dt, dr);
  last_reported_ = current_T_prior;
  has_last_ = true;
  return current_T_prior;
}

void TFTransformationGetter::spin() {
  const auto period = std::chrono::duration<double>(config.tf_lookup.wait_duration_s);
  const auto robot_frame = getRobotFrame();
  while (true) {
    hydra::PoseStatus status;
    try {
      // No timeout or retry loop: destruction must not wait for TF to become available.
      const auto transform =
          tf_lookup_.buffer.lookupTransform(config.prior_frame_id, robot_frame, rclcpp::Time());
      const auto& p = transform.transform.translation;
      const auto& q = transform.transform.rotation;
      status.target_p_source = Eigen::Vector3d(p.x, p.y, p.z);
      status.target_R_source = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
      status.is_valid = true;
    } catch (const tf2::TransformException&) {
      // Missing TF is expected before relocalization. Invalidate the previous result.
    }

    std::unique_lock<std::mutex> lock(mutex_);
    last_status_ = status;
    if (lookup_cv_.wait_for(lock, period, [this] { return should_shutdown_; })) {
      return;
    }
  }
}

std::string TFTransformationGetter::getRobotFrame() const {
  if (!config.robot_frame_id.empty()) {
    return config.robot_frame_id;
  }

  return tf_lookup_.frames.odom;
}

}  // namespace khronos
