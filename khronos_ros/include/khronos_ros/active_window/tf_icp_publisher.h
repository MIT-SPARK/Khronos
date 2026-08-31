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

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>
#include <hydra/utils/logging.h>
#include <hydra_ros/utils/tf_lookup.h>
#include <ianvs/node_handle.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "khronos/active_window/change_detection/active_window_change_detector.h"
#include "khronos/active_window/data/track.h"

namespace khronos {

/**
 * @brief ActiveWindowCDSink that publishes the ICP refinement delta as a TF.
 *
 * Publishes pre_icp_odom -> odom (StaticTF), splitting the full odom_T_map
 * transform (received as current_T_prior from AWCD) into:
 *   - map -> pre_icp_odom: published externally by upper level pose estimation modules (ROMAN, etc.)
 *   - pre_icp_odom -> odom: published here (ICP delta on top of estimated pose)
 *  
 * On startup, seeds identity so the TF chain map -> odom is immediately resolvable.
 * When enable_icp_refinement=false in AWCD, current_T_prior equals the estimated pose
 * and the ICP delta is identity.
 */
class TfIcpPublisher : public ActiveWindowChangeDetector::ActiveWindowCDSink {
 public:
  struct Config : hydra::VerbosityConfig {
    Config()
        : hydra::VerbosityConfig{hydra::GlobalInfo::instance().getConfig().default_verbosity} {}

    //! Frame published by upper stream pose estimation modules (usually child of map).
    std::string pre_icp_odom_frame = "";
    //! Robot odometry frame (child of pre_icp_odom_frame). Empty = GlobalInfo odom frame.
    std::string odom_frame = "";
    //! Underlying TF lookup config. max_tries defaults to 1 (non-blocking): TF being
    //! unavailable is the expected steady state before relocalization fires, and the
    //! default TFLookup retry loop would otherwise stall the AWCD call() path.
    struct NonBlockingTfLookupConfig : hydra::TFLookup::Config {
      NonBlockingTfLookupConfig() { max_tries = 1; }
    };
    hydra::TFLookup::Config tf_lookup = NonBlockingTfLookupConfig();
  } const config;

  explicit TfIcpPublisher(const Config& config, const ianvs::NodeHandle* nh = nullptr);
  virtual ~TfIcpPublisher() = default;

  void call(const DynamicSceneGraph::Ptr& dsg,
            const std::vector<ActiveWindowChangeDetector::RemovedObject>& removed_objects,
            const std::vector<ActiveWindowChangeDetector::AddedObject>& newly_added_objects,
            const Eigen::Isometry3d& current_T_prior) const override;

 private:
  std::string getOdomFrame() const;
  void broadcastTransform(const Eigen::Isometry3d& pre_icp_odom_T_odom) const;

  ianvs::NodeHandle nh_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  hydra::TFLookup tf_lookup_;
};

void declare_config(TfIcpPublisher::Config& config);

}  // namespace khronos
