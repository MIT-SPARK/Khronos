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

#include "khronos_ros/active_window/tf_icp_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <glog/logging.h>
#include <hydra/utils/logging.h>

namespace khronos {
namespace {

static const auto registration =
    config::RegistrationWithConfig<ActiveWindowChangeDetector::ActiveWindowCDSink,
                                   TfIcpPublisher,
                                   TfIcpPublisher::Config>("TfIcpPublisher");

}  // namespace

void declare_config(TfIcpPublisher::Config& config) {
  using namespace config;
  name("TfIcpPublisher");
  field(config.verbosity, "verbosity");
  field(config.pre_icp_odom_frame, "pre_icp_odom_frame");
  field(config.odom_frame, "odom_frame");
  field(config.tf_lookup, "tf_lookup");
}

TfIcpPublisher::TfIcpPublisher(const Config& cfg, const ianvs::NodeHandle* nh)
    : config(config::checkValid(cfg)),
      nh_(nh ? *nh / "tf_icp_publisher" : ianvs::NodeHandle::this_node("tf_icp_publisher")),
      tf_lookup_(config.tf_lookup) {
  tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(nh_.node());

  // Seed identity so map → odom TF chain is resolvable before the first ICP result.
  broadcastTransform(Eigen::Isometry3d::Identity());
  MLOG(1) << "[TfIcpPublisher] Initialized (" << config.pre_icp_odom_frame << " -> "
          << getOdomFrame() << "). Seeded identity transform.";
}

void TfIcpPublisher::call(const DynamicSceneGraph::Ptr& /*dsg*/,
                          const std::vector<ActiveWindowChangeDetector::RemovedObject>& /*removed_objects*/,
                          const std::vector<ActiveWindowChangeDetector::AddedObject>& /*newly_added_objects*/,
                          const Eigen::Isometry3d& current_T_prior) const {
  // Look up map -> pre_icp_odom (the ROMAN-only result, published upstream).
  std::string err;
  const auto status = hydra::lookupTransform(tf_lookup_.buffer,
                                             std::nullopt,
                                             "map",
                                             config.pre_icp_odom_frame,
                                             config.tf_lookup.max_tries,
                                             config.tf_lookup.wait_duration_s,
                                             config.tf_lookup.verbosity,
                                             &err);
  if (!status) {
    MLOG(3) << "[TfIcpPublisher] TF lookup map -> " << config.pre_icp_odom_frame
            << " failed: " << err << ". Broadcasting identity.";
    broadcastTransform(Eigen::Isometry3d::Identity());
    return;
  }
  const Eigen::Isometry3d map_T_pre_icp_odom = status.target_T_source();

  // Split map_T_odom = map_T_pre_icp_odom * pre_icp_odom_T_odom and solve for our half:
  //   pre_icp_odom_T_odom = map_T_pre_icp_odom^-1 * map_T_odom
  // current_T_prior is odom_T_map (full ROMAN + ICP result from AWCD), so map_T_odom is its
  // inverse. With ICP disabled the two factors cancel and this is identity, as documented above.
  const Eigen::Isometry3d pre_icp_odom_T_odom =
      map_T_pre_icp_odom.inverse() * current_T_prior.inverse();
  broadcastTransform(pre_icp_odom_T_odom);
}

std::string TfIcpPublisher::getOdomFrame() const {
  if (!config.odom_frame.empty()) {
    return config.odom_frame;
  }
  return hydra::GlobalInfo::instance().getFrames().odom;
}

void TfIcpPublisher::broadcastTransform(const Eigen::Isometry3d& pre_icp_odom_T_odom) const {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = nh_.now();
  tf_msg.header.frame_id = config.pre_icp_odom_frame;
  tf_msg.child_frame_id = getOdomFrame();

  const auto& t = pre_icp_odom_T_odom.translation();
  tf_msg.transform.translation.x = t.x();
  tf_msg.transform.translation.y = t.y();
  tf_msg.transform.translation.z = t.z();
  const Eigen::Quaterniond q(pre_icp_odom_T_odom.rotation());
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace khronos
