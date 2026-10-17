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

#include "khronos_ros/active_window/ros_change_detection_controller.h"

#include <glog/logging.h>

namespace khronos {
namespace {

static const auto registration =
    config::RegistrationWithConfig<ChangeDetectionController,
                                   RosChangeDetectionController,
                                   RosChangeDetectionController::Config>(
        "RosChangeDetectionController");

}  // namespace

void declare_config(RosChangeDetectionController::Config& config) {
  using namespace config;
  name("RosChangeDetectionController");
  field(config.ns, "ns");
}

RosChangeDetectionController::RosChangeDetectionController(const Config& config)
    : ChangeDetectionController(ChangeDetectionController::Config{}),
      config(config::checkValid(config)),
      nh_(ianvs::NodeHandle::this_node(config.ns)) {
  state_pub_ = nh_.create_publisher<std_msgs::msg::Bool>(
      "enabled", rclcpp::QoS(1).transient_local().reliable());

  // Requests only flip atomics; the detector applies them at its next frame.
  start_srv_ = nh_.create_service<Trigger>(
      "start", [this](const Trigger::Request::SharedPtr&, Trigger::Response::SharedPtr res) {
        res->success = true;
        res->message = setEnabled(true) ? "started" : "already running";
        LOG(INFO) << "[RosChangeDetectionController] Start: " << res->message;
      });
  stop_srv_ = nh_.create_service<Trigger>(
      "stop", [this](const Trigger::Request::SharedPtr&, Trigger::Response::SharedPtr res) {
        res->success = true;
        res->message = setEnabled(false) ? "stopped" : "already stopped";
        LOG(INFO) << "[RosChangeDetectionController] Stop: " << res->message;
      });
  reset_srv_ = nh_.create_service<Trigger>(
      "reset", [this](const Trigger::Request::SharedPtr&, Trigger::Response::SharedPtr res) {
        requestReset();
        res->success = true;
        res->message = "reset requested";
        LOG(INFO) << "[RosChangeDetectionController] Reset requested.";
      });
}

void RosChangeDetectionController::onStateChanged(bool enabled) {
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  state_pub_->publish(msg);
}

}  // namespace khronos
