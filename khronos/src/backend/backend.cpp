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

#include "khronos/backend/backend.h"

#include <filesystem>
#include <iomanip>
#include <sstream>

#include <config_utilities/types/path.h>
#include <glog/logging.h>
#include <hydra/backend/mst_factors.h>
#include <hydra/common/global_info.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <kimera_pgmo/utils/mesh_io.h>

#include "khronos/backend/change_state.h"
#include "khronos/common/common_types.h"

namespace khronos {

using hydra::UpdateInfo;
using spark_dsg::ObjectNodeAttributes;
using spark_dsg::PlaceNodeAttributes;
using spark_dsg::SemanticNodeAttributes;

void declare_config(Backend::Config& config) {
  using namespace config;
  name("Backend");
  base<hydra::BackendModule::Config>(config);
  field(config.verbosity, "verbosity");
  field(config.max_dt_merge_proposal, "max_dt_merge_proposal");
  field(config.optimize_on_new_merge, "optimize_on_new_merge");
  field(config.add_merge_factor, "add_merge_factor");
  field(config.pose_object_covariance, "pose_object_covariance");
  field(config.object_merge_covariance, "object_merge_covariance");
  field(config.pose_object_consistency_threshold, "pose_object_consistency_threshold");
  field(config.fix_input_pose_variance, "fix_input_pose_variance");
  field(config.fix_input_poses, "fix_input_poses");

  {
    NameSpace ns("change_detection");
    field(config.run_change_detection_every_n_frames, "run_every_n_frames");
  }

  field(config.update_objects, "update_objects");
  field(config.spatio_temporal_map, "spatio_temporal_map");
  field(config.reconciler, "reconciler");
  field(config.change_detection, "change_detection");

  check(config.pose_object_covariance, GT, 0, "pose_object_covariance");
  check(config.object_merge_covariance, GT, 0, "object_merge_covariance");
  check(config.pose_object_consistency_threshold, GE, 0, "pose_object_consistency_threshold");
}

Backend::Backend(const Config& config,
                 const hydra::SharedDsgInfo::Ptr& dsg,
                 const hydra::SharedModuleState::Ptr& state)
    : hydra::BackendModule(config::checkValid(config), dsg, state),
      config(config),
      map_(config.spatio_temporal_map) {
  change_detector_ = std::make_unique<SequentialChangeDetector>(config.change_detection);
  change_detector_->setDsg(unmerged_graph_);
  reconciler_ = std::make_unique<Reconciler>(config.reconciler);
}

Backend::~Backend() {}

void Backend::start() { spin_thread_.reset(new std::thread(&Backend::spin, this)); }

void Backend::spin() {
  should_shutdown_ = false;
  bool should_shutdown = false;
  while (!should_shutdown) {
    auto& queue = hydra::PipelineQueues::instance().backend_queue;
    bool has_data = queue.poll();
    if (hydra::GlobalInfo::instance().force_shutdown() || !has_data) {
      // copy over shutdown request
      should_shutdown = should_shutdown_;
    }

    if (!has_data) {
      continue;
    }

    spinCallback(*queue.pop());
  }
}

void Backend::spinCallback(const hydra::BackendInput& input) {
  status_log_.emplace_back(hydra::BackendModuleStatus{});
  std::lock_guard<std::mutex> lock(mutex_);
  const uint64_t timestamp_ns = input.timestamp_ns;
  last_timestamp_received_ = timestamp_ns;
  last_sequence_number_ = input.sequence_number;
  if (config.run_change_detection_every_n_frames > 0) {
    num_frames_since_last_change_detection_++;
  }

  Timer timer("backend/update", last_timestamp_received_);
  updateFactorGraph(input);
  if (config.fix_input_poses) {
    // TODO(lschmid): BROKEN Fix this, since the new hydra update we get a warning "adding node
    // measurement to a node aXXX not previously seen before"
    // fixInputPoses(input);
  }
  copyMeshDelta(input);
  updateFromLcdQueue();
  status_log_.back().total_loop_closures = num_loop_closures_;

  // TODO(lschmid): Update the hydra logic to not just drop packages in the counting.
  if (!updatePrivateDsg(timestamp_ns, false)) {
    CLOG(5) << "[Backend] skipping input @" << timestamp_ns << ".";
    // we only read from the frontend dsg if we've processed all the
    // factor graph update packets (as long as force_update is false)
    // we still log the status for each received frontend packet
    logStatus();
    return;
  }

  timer.reset("backend/spin");
  // Copy have_new_LC since this will be reset after optimization.
  const bool force_check_merge_proposals =
      last_timestamp_received_ >= last_merge_proposal_t_ + config.max_dt_merge_proposal * 1e9;

  if ((config.optimize_on_lc && have_new_loopclosures_) ||
      (config.optimize_on_new_merge && !new_proposed_merges_.empty()) ||
      force_check_merge_proposals) {
    optimize(timestamp_ns, force_check_merge_proposals);
  } else {
    updateDsgMesh(timestamp_ns);
    UpdateInfo::ConstPtr info(new UpdateInfo{timestamp_ns});
    dsg_updater_->callUpdateFunctions(timestamp_ns, info);
  }

  CLOG(3) << "Proposed " << new_proposed_merges_.size() << " node merges.";

  // Run incremental change detection if desired. -1: disabled, 0: on LC only, >0: every n frames
  if (config.run_change_detection_every_n_frames >= 0) {
    if (have_new_loopclosures_ ||
        num_frames_since_last_change_detection_ > config.run_change_detection_every_n_frames) {
      num_frames_since_last_change_detection_ -= config.run_change_detection_every_n_frames;
      runChangeDetection();
    }
  }

  logStatus();

  timer.reset("backend/sinks");
  Sink::callAll(sinks_, timestamp_ns, *private_dsg_->graph, *deformation_graph_);
  have_new_loopclosures_ = false;
}

void Backend::runChangeDetection() {
  // Clone all relevant data structures for change detection.
  auto dsg = unmerged_graph_->clone();
  std::thread(
      &Backend::runChangeDetectionThread, this, dsg, proposed_merges_, last_timestamp_received_)
      .detach();
}

void Backend::runChangeDetectionThread(DynamicSceneGraph::Ptr dsg,
                                       RPGOMerges rpgo_merges,
                                       TimeStamp stamp) {
  // Currently just lock the entire map mutex to avoid threading headaches. Ideally 4D map updates
  // are not run at a fequency where this is blocking anyways.
  std::lock_guard<std::mutex> lock(map_mutex_);

  // TODO(lschmid): Decouple this for higher frequencey CD updates in the incremental visualization
  // version.
  // TODO(lschmid): Currently always reset the change detection to avoid rare (but possible) hiccups
  // from deleted active vertices. Fix this by only resetting the active window mesh.
  change_detector_->setDsg(dsg);
  const auto& changes =
      change_detector_->detectChanges(rpgo_merges, last_timestamp_received_, true);
  reconciler_->reconcile(*dsg, changes, stamp);
  map_.update(dsg, stamp);

  ChangeSink::callAll(change_sinks_, last_timestamp_received_, changes);
  CLOG(3) << "Change detection completed: " << map_.numTimeSteps() << " time steps in 4D map.";
}

void Backend::finishProcessing() {
  std::lock_guard<std::mutex> lock(mutex_);
  updatePrivateDsg(last_timestamp_received_, true);
  have_new_loopclosures_ = true;
  optimize(last_timestamp_received_, true);
  if (config.run_change_detection_every_n_frames >= 0) {
    runChangeDetection();
  }
}

void Backend::addChangeSink(const ChangeSink::Ptr& sink) {
  if (sink) {
    change_sinks_.push_back(sink);
  }
}

void Backend::optimize(size_t timestamp_ns, bool force_find_merge_proposals) {
  hydra::BackendModule::optimize(timestamp_ns, force_find_merge_proposals);
  if (have_new_loopclosures_ || force_find_merge_proposals) {
    last_merge_proposal_t_ = timestamp_ns;
  }
}

size_t Backend::findClosestNode(size_t timestamp_ns) {
  auto it = std::lower_bound(timestamps_.begin(), timestamps_.end(), timestamp_ns);
  if (it == timestamps_.end()) {
    return timestamps_.size() - 1;
  }
  return it - timestamps_.begin();
}

bool Backend::saveProposedMerges(const hydra::DataDirectory& log_setup) {
  const auto backend_path = log_setup.path("backend");
  std::lock_guard<std::mutex> lock(proposed_merges_mutex_);
  return proposed_merges_.save(backend_path / "proposed_merge.csv");
}

void Backend::save(const hydra::DataDirectory& log_setup) {
  const auto path = log_setup.path();
  unmerged_graph_->save(path / "shared_dsg.json", false);
  private_dsg_->graph->save(path / "dsg.json", false);
  const auto mesh = private_dsg_->graph->mesh();
  if (mesh && !mesh->empty()) {
    // mesh implements vertex and face traits
    kimera_pgmo::WriteMesh(path / "mesh.ply", *mesh, *mesh);
  }
  deformation_graph_->save(path / "deformation_graph.dgrf");

  // Save the proposed merges.
  saveProposedMerges(log_setup);

  // Save the detected changes.
  const Changes& changes = change_detector_->getChanges();
  if (!changes.object_changes.empty()) {
    changes.object_changes.save(path / "object_changes.csv");
  }
  if (!changes.background_changes.empty()) {
    changes.background_changes.save(path / "background_changes.csv");
  }

  // Ensure the 4D map has at least the final state
  if (map_.numTimeSteps() == 0 || config.run_change_detection_every_n_frames < 0) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_.update(private_dsg_->graph->clone(), last_timestamp_received_);
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (map_.save(path / "final.4dmap")) {
      CLOG(1) << "Saved 4D map with " << map_.numTimeSteps() << " time steps to '" << path << "'.";
    }

    // Save individual DSGs from the 4D map
    const auto maps_path = path / "maps";
    if (!std::filesystem::exists(maps_path)) {
      std::filesystem::create_directories(maps_path);
    }

    const auto& timestamps = map_.stamps();
    for (size_t i = 0; i < map_.numTimeSteps(); ++i) {
      // Get the DSG for this timestamp
      auto dsg = map_.getDsgPtr(timestamps[i]);
      if (dsg) {
        // Save with timestamp as filename
        std::stringstream filename;
        filename << "dsg_" << std::setw(5) << std::setfill('0') << i
                 << "_" << timestamps[i] << ".json";
        dsg->save(maps_path / filename.str(), false);
      }
    }

    if (map_.numTimeSteps() > 0) {
      CLOG(1) << "Saved " << map_.numTimeSteps() << " individual DSGs to '" << maps_path << "'.";
    }
  }
}

void Backend::fixInputPoses(const hydra::BackendInput& input) {
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> prior_measurements;
  for (const auto& msg : input.agent_updates.pose_graphs) {
    status_log_.back().new_factors += msg.edges.size();

    for (const auto& node : msg.nodes) {
      if (node.key == 0) {
        continue;  // This should already be set by 'add_initial_prior'
      }

      prior_measurements.push_back(
          {gtsam::Symbol(kimera_pgmo::GetRobotPrefix(node.robot_id), node.key),
           gtsam::Pose3(node.pose.matrix())});
    }
  }
  deformation_graph_->processNodeMeasurements(prior_measurements, config.fix_input_pose_variance);
}

}  // namespace khronos
