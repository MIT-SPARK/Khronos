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

#include <Eigen/Geometry>
#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <hydra/utils/logging.h>

namespace khronos {
namespace {

static const auto registration =
    config::RegistrationWithConfig<TransformationGetter,
                                   TFTransformationGetter,
                                   TFTransformationGetter::Config>("TFTransformationGetter");

}  // namespace

void declare_config(TFTransformationGetter::Config& config) {
  using namespace config;
  name("TFTransformationGetter");
  field(config.verbosity, "verbosity");
  field(config.prior_frame_id, "prior_frame_id");
  field(config.robot_frame_id, "robot_frame_id");
  field(config.tf_change_threshold_m, "tf_change_threshold_m");
  field(config.tf_change_threshold_rad, "tf_change_threshold_rad", "rad");
  field(config.tf_lookup, "tf_lookup");
}

TFTransformationGetter::TFTransformationGetter(const Config& cfg)
    : config(config::checkValid(cfg)), tf_lookup_(config.tf_lookup) {
  MLOG(1) << "[TFTransformationGetter] TF listener created (" << config.prior_frame_id << " -> "
          << getRobotFrame() << ")";
}

std::optional<Eigen::Isometry3d> TFTransformationGetter::getTransformation() const {
  std::string err;
  const auto status = hydra::lookupTransform(tf_lookup_.buffer,
                                             std::nullopt,
                                             config.prior_frame_id,
                                             getRobotFrame(),
                                             config.tf_lookup.max_tries,
                                             config.tf_lookup.wait_duration_s,
                                             config.tf_lookup.verbosity,
                                             &err);
  if (!status) {
    MLOG(3) << "[TFTransformationGetter] TF lookup failed: " << err;
    return std::nullopt;
  }

  // status.target_T_source() is map_T_{robot}/odom; current_T_prior is its inverse
  // (odom frame is "current").
  const Eigen::Isometry3d current_T_prior = status.target_T_source().inverse();

  const Eigen::Isometry3d delta = last_reported_.inverse() * current_T_prior;
  const double dt = delta.translation().norm();
  const double dr = Eigen::AngleAxisd(delta.linear()).angle();
  if (has_last_ && dt <= config.tf_change_threshold_m && dr <= config.tf_change_threshold_rad) {
    return std::nullopt;
  }

  MLOG(1) << "[TFTransformationGetter] TF changed by " << dt << " m, " << dr
          << " rad, reporting new transform.";
  last_reported_ = current_T_prior;
  has_last_ = true;
  return current_T_prior;
}

std::string TFTransformationGetter::getRobotFrame() const {
  if (!config.robot_frame_id.empty()) {
    return config.robot_frame_id;
  }
  return hydra::GlobalInfo::instance().getFrames().odom;
}

}  // namespace khronos
