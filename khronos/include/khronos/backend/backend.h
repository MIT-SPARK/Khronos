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

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <hydra/backend/backend_module.h>
#include <hydra/common/shared_module_state.h>

#include "khronos/backend/change_detection/sequential_change_detector.h"
#include "khronos/backend/change_state.h"
#include "khronos/backend/reconciliation/reconciler.h"
#include "khronos/backend/update_functions.h"
#include "khronos/common/common_types.h"
#include "khronos/spatio_temporal_map/spatio_temporal_map.h"

namespace khronos {

class Backend : public hydra::BackendModule {
 public:
  // Config.
  struct Config : public hydra::BackendModule::Config {
    int verbosity;

    double max_dt_merge_proposal = 3.0;
    bool optimize_on_new_merge = true;
    bool add_merge_factor = true;

    double pose_object_covariance = 0;
    double object_merge_covariance = 0;
    double pose_object_consistency_threshold = 0;

    double fix_input_pose_variance = 1e-2;
    bool fix_input_poses = false;

    // How often to run change detection in frames. A value of 0 runs only on loop closures. A value
    // of -1 or lower disables change detection. Note that this is also the frequency at which the
    // change representation (ray verificator) is updated.
    // TODO(lschmid): Refactor this together with asynchronous 4D-map updates.
    int run_change_detection_every_n_frames = 0;

    // Member configs.
    UpdateObjectsFunctor::Config update_objects;
    SpatioTemporalMap::Config spatio_temporal_map;
    SequentialChangeDetector::Config change_detection;
    Reconciler::Config reconciler;
  } const config;

  // Types.
  using Ptr = std::shared_ptr<Backend>;
  using ConstPtr = std::shared_ptr<const Backend>;

  // Construction.
  Backend(const Config& config,
          const hydra::SharedDsgInfo::Ptr& dsg,
          const hydra::SharedModuleState::Ptr& state);
  ~Backend();

  // Save data.
  void save(const hydra::LogSetup& log_setup) override;
  bool saveProposedMerges(const hydra::LogSetup& log_setup);
  bool save4DMap(const std::string& path);

  // Spinning.
  void start() override;
  void spin();
  void spinCallback(const hydra::BackendInput& input);

  // Interaction.
  /**
   * @brief Run a final optimization after all input data has been received.
   */
  void finishProcessing();

  // Accessors.
  const RPGOMerges& getProposedMerges() const { return proposed_merges_; }
  const Changes& getChanges() const { return change_detector_->getChanges(); }
  const DynamicSceneGraph& getDsg() const { return *private_dsg_->graph; }

 protected:
  using hydra::BackendModule::optimize;  // disables hidden virtual warning
  void optimize(size_t timestamp_ns, bool force_find_merge_proposals = false);

  void copyMeshDelta(const hydra::BackendInput& input) override;

  size_t findClosestNode(size_t timestamp_ns);

  void fixInputPoses(const hydra::BackendInput& input);

  void addObjectsToDeformationGraph(size_t timestamp_ns);

  void addProposedMergeToDeformationGraph(size_t timestamp_ns);

  void extractProposedMergeResults(size_t timestamp_ns);

  void runChangeDetection();

  void runChangeDetectionThread(DynamicSceneGraph::Ptr dsg,
                                RPGOMerges rpgo_merges,
                                TimeStamp stamp);

 protected:
  // Members.
  SpatioTemporalMap map_;
  std::unique_ptr<SequentialChangeDetector> change_detector_;
  std::unique_ptr<Reconciler> reconciler_;

  // Threading of detached change detection.
  std::mutex map_mutex_;

  // Variables.
  std::set<NodeId> objects_added_;
  // Mutex when accessing 'proposed_merges_'.
  std::mutex proposed_merges_mutex_;
  // All merge proposals generated and to be used in the optimization.
  hydra::MergeList new_proposed_merges_;
  // Validated result of 'new_proposed_merges_' after optimization, ready to be used in change
  // detection.
  RPGOMerges proposed_merges_;
  uint64_t last_merge_proposal_t_ = 0;
  uint64_t last_timestamp_received_ = 0;
  int num_frames_since_last_change_detection_ = 0;
};

void declare_config(Backend::Config& config);

}  // namespace khronos
