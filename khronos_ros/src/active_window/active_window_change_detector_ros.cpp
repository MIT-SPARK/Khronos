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

#include "khronos_ros/active_window/active_window_change_detector_ros.h"

#include <config_utilities/config.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>
#include <ianvs/node_handle.h>
#include <tf2/exceptions.h>

namespace khronos {
namespace {

static const auto registration_ros =
    config::RegistrationWithConfig<ActiveWindow::KhronosSink,
                                   ActiveWindowChangeDetectorRos,
                                   ActiveWindowChangeDetectorRos::Config>(
        "ActiveWindowChangeDetectorRos");

}  // namespace

void declare_config(ActiveWindowChangeDetectorRos::Config& config) {
  using namespace config;
  name("ActiveWindowChangeDetectorRos");
  // Declare all parent fields first.
  base<ActiveWindowChangeDetector::Config>(config);
  // New fields for this subclass.
  field(config.prior_frame_id, "prior_frame_id");
  field(config.robot_frame_id, "robot_frame_id");
  field(config.tf_change_threshold_m, "tf_change_threshold_m");
}

ActiveWindowChangeDetectorRos::ActiveWindowChangeDetectorRos(const Config& cfg)
    : ActiveWindowChangeDetector(cfg), config(cfg) {
  auto nh = ianvs::NodeHandle::this_node();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh.clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  MLOG(1) << "[ActiveWindowChangeDetectorRos] TF listener created ("
          << config.prior_frame_id << " -> " << getRobotFrame() << ")";
}

void ActiveWindowChangeDetectorRos::call(const FrameData& data,
                                         const VolumetricMap& map,
                                         const Tracks& tracks) const {
  try {
    const auto tf =
        tf_buffer_->lookupTransform(config.prior_frame_id, getRobotFrame(), tf2::TimePointZero);

    // Convert geometry_msgs::TransformStamped to Eigen: this is map_T_{robot}/odom.
    const auto& t = tf.transform.translation;
    const auto& r = tf.transform.rotation;
    Eigen::Isometry3d map_T_odom = Eigen::Isometry3d::Identity();
    map_T_odom.translation() << t.x, t.y, t.z;
    map_T_odom.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();

    // current_T_prior = inverse of map_T_odom (odom frame is treated as "current").
    const Eigen::Isometry3d current_T_prior = map_T_odom.inverse();

    const double delta =
        (current_T_prior.translation() - last_tf_guess_.translation()).norm();
    if (!has_last_tf_ || delta > config.tf_change_threshold_m) {
      MLOG(1) << "[ActiveWindowChangeDetectorRos] TF changed by " << delta
              << " m, triggering ICP.";
      notifyLoopClosure(current_T_prior);
      last_tf_guess_ = current_T_prior;
      has_last_tf_ = true;
    }
  } catch (const tf2::TransformException& e) {
    MLOG(3) << "[ActiveWindowChangeDetectorRos] TF lookup failed: " << e.what();
  }

  ActiveWindowChangeDetector::call(data, map, tracks);
}

// function that give map -> odom, in our case it's just tf lookup. 

std::string ActiveWindowChangeDetectorRos::getRobotFrame() const {
  if (!config.robot_frame_id.empty()) {
    return config.robot_frame_id;
  }
  return hydra::GlobalInfo::instance().getFrames().odom;
}

}  // namespace khronos
