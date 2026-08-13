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

#include <optional>
#include <set>

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>
#include <hydra/utils/logging.h>
#include <ianvs/node_handle.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <khronos_msgs/msg/awcd_changes.hpp>

#include "khronos/active_window/change_detection/active_window_change_detector.h"
#include "khronos/active_window/data/track.h"

namespace khronos {

/**
 * @brief ActiveWindowCDSink that packs the detector's per-frame removed/added object
 * results into a khronos_msgs/AwcdChanges message and publishes it, so that other
 * processes (e.g. a base station) can consume the change-detection output.
 */
class ActiveWindowChangeDetectorPublisher : public ActiveWindowChangeDetector::ActiveWindowCDSink {
 public:
  struct Config : hydra::VerbosityConfig {
    Config()
        : hydra::VerbosityConfig{hydra::GlobalInfo::instance().getConfig().default_verbosity} {}

    //! Frame in which reported object attributes (centroid, bbox) are expressed.
    std::string global_frame_name = hydra::GlobalInfo::instance().getFrames().map;

    //! Name of the robot reporting the changes, forwarded into AwcdChanges::robot_name.
    std::string robot_name;

    //! Topic to publish AwcdChanges messages on. Leading '/' makes this a global (non-namespaced)
    //! topic so all robots' change detectors publish to the same topic; the base station
    //! distinguishes robots via AwcdChanges::robot_name.
    std::string topic = "/awcd_changes";

    //! Publisher queue size.
    int queue_size = 10;
  } const config;

  explicit ActiveWindowChangeDetectorPublisher(const Config& config,
                                               const ianvs::NodeHandle* nh = nullptr);
  virtual ~ActiveWindowChangeDetectorPublisher() = default;

  // KhronosSink callback - called each frame. Only publishes when the set of removed and/or
  // added object ids differs from the last published sets (see last_removed_ids_/last_added_ids_).
  void call(const DynamicSceneGraph::Ptr& dsg,
            const std::vector<ActiveWindowChangeDetector::RemovedObject>& removed_objects,
            const std::vector<Track>& newly_added_tracks,
            const Eigen::Isometry3d& current_T_prior) const override;

 private:
  //! Fills in a ChangedObjectInfo entry for a removed object (id + latched first-removed time).
  khronos_msgs::msg::ChangedObjectInfo makeRemovedInfo(
      const ActiveWindowChangeDetector::RemovedObject& obj) const;

  //! Fills in a ChangedObjectInfo entry (full attributes) for a newly-added track.
  khronos_msgs::msg::ChangedObjectInfo makeAddedInfo(const Track& track,
                                                     const Eigen::Isometry3d& prior_T_current) const;

  // ROS
  ianvs::NodeHandle nh_;
  rclcpp::Publisher<khronos_msgs::msg::AwcdChanges>::SharedPtr changes_pub_;

  // Set-membership of the last *published* removed/added ids, used to gate publishing to only
  // when the reported set of changes actually changed (rather than every frame).
  mutable std::optional<std::set<int64_t>> last_removed_ids_;
  mutable std::optional<std::set<int64_t>> last_added_ids_;
};

void declare_config(ActiveWindowChangeDetectorPublisher::Config& config);

}  // namespace khronos
