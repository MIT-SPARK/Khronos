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

#include <config_utilities/types/path.h>
#include <glog/logging.h>
#include <hydra/backend/mst_factors.h>
#include <hydra/common/global_info.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra/utils/pgmo_mesh_traits.h>

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
  update_functors_.push_back(
      std::make_shared<UpdateObjectsFunctor>(config.update_objects, new_proposed_merges_));
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

    if (has_data) {
      spinCallback(*queue.pop());
    }
  }
}

void Backend::spinCallback(const hydra::BackendInput& input) {
  status_.reset();
  std::lock_guard<std::mutex> lock(mutex_);
  last_timestamp_received_ = input.timestamp_ns;
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
  status_.total_loop_closures = num_loop_closures_;

  // TODO(lschmid): Update the hydra logic to not just drop packages in the counting.
  if (!updatePrivateDsg(input.timestamp_ns, false)) {
    CLOG(5) << "[Backend] skipping input @" << input.timestamp_ns << ".";
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
    optimize(last_timestamp_received_, force_check_merge_proposals);
  } else {
    updateDsgMesh(last_timestamp_received_);
    callUpdateFunctions(last_timestamp_received_);
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

  if (logs_) {
    logStatus();
  }

  timer.reset("backend/sinks");
  Sink::callAll(sinks_, last_timestamp_received_, *private_dsg_->graph, *deformation_graph_);
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

void Backend::optimize(size_t timestamp_ns, bool force_find_merge_proposals) {
  if (config.add_places_to_deformation_graph) {
    const auto vertex_key = hydra::GlobalInfo::instance().getRobotPrefix().vertex_key;
    hydra::addPlacesToDeformationGraph(*unmerged_graph_,
                                       timestamp_ns,
                                       *deformation_graph_,
                                       config.pgmo.place_edge_variance,
                                       config.pgmo.place_mesh_variance,
                                       [vertex_key](auto) { return vertex_key; });
  }

  addObjectsToDeformationGraph(timestamp_ns);
  addProposedMergeToDeformationGraph(timestamp_ns);

  Timer timer("backend/optimization", timestamp_ns, true, 0, false);
  deformation_graph_->optimize();
  timer.stop();

  extractProposedMergeResults(timestamp_ns);
  updateDsgMesh(timestamp_ns, true);

  const bool process_loopclosures = have_new_loopclosures_ || force_find_merge_proposals;
  callUpdateFunctions(timestamp_ns,
                      deformation_graph_->getGtsamTempValues(),
                      deformation_graph_->getGtsamValues(),
                      process_loopclosures);
  if (process_loopclosures) {
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

void Backend::addProposedMergeToDeformationGraph(size_t timestamp_ns) {
  Timer timer("backend/add_proposed_merges", timestamp_ns);
  if (!config.add_merge_factor) {
    return;
  }

  pose_graph_tools::PoseGraph merge_edges;
  for (const auto& merge : new_proposed_merges_) {
    if (spark_dsg::NodeSymbol(merge.from).category() != 'O' ||
        spark_dsg::NodeSymbol(merge.to).category() != 'O') {
      continue;
    }

    pose_graph_tools::PoseGraphEdge merge_edge;
    merge_edge.key_from = merge.from;
    merge_edge.key_to = merge.to;
    merge_edge.pose.setIdentity();
    merge_edges.edges.push_back(merge_edge);
  }

  CLOG(4) << "Adding " << merge_edges.edges.size() << " object merges to deformation graph.";
  deformation_graph_->addNewTempEdges(merge_edges, config.object_merge_covariance, false);
}

void Backend::addObjectsToDeformationGraph(size_t timestamp_ns) {
  Timer timer("backend/add_objects", timestamp_ns);

  pose_graph_tools::PoseGraph place_object_edges;
  const auto& objects = private_dsg_->graph->getLayer(DsgLayers::OBJECTS);
  for (const auto& id_node_pair : objects.nodes()) {
    if (objects_added_.count(id_node_pair.first)) {
      continue;
    }

    if (!private_dsg_->graph->hasNode(id_node_pair.first)) {
      continue;
    }

    const auto& attrs =
        private_dsg_->graph->getNode(id_node_pair.first).attributes<KhronosObjectAttributes>();

    if (attrs.is_active) {
      continue;
    }

    gtsam::Pose3 obj_pose(gtsam::Rot3(), attrs.position);

    // Implicit assumption that graph builder output only has a single observed time segment
    size_t first_obs_idx = findClosestNode(attrs.first_observed_ns.at(0));
    gtsam::Point3 first_t_obj = attrs.position - trajectory_.at(first_obs_idx).translation();

    deformation_graph_->addNewNode(id_node_pair.first, obj_pose, false);
    objects_added_.insert(id_node_pair.first);

    // TODO(Yun): For now both pose-object edges are added as inliers with a consistency check
    // added as to not cause any bad trajectory deformations. In the future better to use GNC to
    // decide this, but temporarily cannot resolve numerical issues with no having rotation on
    // objects.
    const auto& prefix = hydra::GlobalInfo::instance().getRobotPrefix();
    deformation_graph_->addNodeValenceEdge(gtsam::Symbol(prefix.key, first_obs_idx),
                                           id_node_pair.first,
                                           trajectory_.at(first_obs_idx),
                                           attrs.position,
                                           config.pose_object_covariance);

    size_t last_obs_idx = findClosestNode(attrs.last_observed_ns.at(0));
    gtsam::Point3 last_t_obj = attrs.position - trajectory_.at(last_obs_idx).translation();

    if (config.pose_object_consistency_threshold > 0) {
      gtsam::Pose3 first_T_last =
          trajectory_.at(first_obs_idx).between(trajectory_.at(last_obs_idx));

      if ((first_t_obj - first_T_last.transformFrom(last_t_obj)).norm() /
              (last_obs_idx - first_obs_idx) >
          config.pose_object_consistency_threshold) {
        continue;
      }
    }

    deformation_graph_->addNodeValenceEdge(gtsam::Symbol(prefix.key, last_obs_idx),
                                           id_node_pair.first,
                                           trajectory_.at(last_obs_idx),
                                           attrs.position,
                                           config.pose_object_covariance);
  }
}

void Backend::extractProposedMergeResults(size_t timestamp_ns) {
  Timer timer("backend/extract_merge_results", timestamp_ns);
  gtsam::Vector gnc_weights = Eigen::VectorXd::Zero(new_proposed_merges_.size());
  if (config.add_merge_factor) {
    auto new_weights = deformation_graph_->getTempFactorGncWeights();
    CHECK_EQ(gnc_weights.size(), new_proposed_merges_.size());
  }

  {  // begin critical section
    std::lock_guard<std::mutex> lock(proposed_merges_mutex_);
    proposed_merges_.resize(new_proposed_merges_.size());
    int64_t index = 0;
    for (const auto& proposal : new_proposed_merges_) {
      RPGOMerge& merge = proposed_merges_.at(index);
      merge.from_node = proposal.from;
      merge.to_node = proposal.to;
      merge.is_valid = !config.add_merge_factor || gnc_weights(index) > 0.5;
      index++;
    }
  }  // end critical section

  // Clear the proposed merges
  deformation_graph_->clearTemporaryStructures();
  new_proposed_merges_.clear();
}

void Backend::copyMeshDelta(const hydra::BackendInput& input) {
  Timer timer("backend/copy_mesh_delta", input.timestamp_ns);
  if (!input.mesh_update) {
    LOG(WARNING) << "[Backend] invalid mesh update!";
    return;
  }

  // TODO(lschmid): Adds maintenance of khronos first_seen_stamps. This should
  // definitively go into pgmo or similar, but I reallt couldn't force myself to read up
  // on and change all pgmo interfaces...
  auto& mesh = *private_dsg_->graph->mesh();
  const std::vector<uint64_t> prev_dsg_first_seen_stamps = mesh.first_seen_stamps;
  input.mesh_update->updateMesh(*private_dsg_->graph->mesh());
  kimera_pgmo::StampedCloud<pcl::PointXYZ> cloud_out{*original_vertices_, vertex_stamps_};
  input.mesh_update->updateVertices(cloud_out);
  std::vector<uint64_t>& dsg_first_seen_stamps = mesh.first_seen_stamps;

  // Fill new values.
  dsg_first_seen_stamps.resize(mesh.points.size(), 0);
  for (size_t i : input.mesh_update->new_indices) {
    dsg_first_seen_stamps[i] = input.timestamp_ns;
  }

  // Adjust the stamps of the vertices that were updated.
  for (const auto& [prev, curr] : input.mesh_update->prev_to_curr) {
    dsg_first_seen_stamps[curr] = prev_dsg_first_seen_stamps[prev];
  }

  // we use this to make sure that deformation only happens for vertices that are
  // still active
  num_archived_vertices_ = input.mesh_update->getTotalArchivedVertices();
  have_new_mesh_ = true;
}

bool Backend::saveProposedMerges(const hydra::LogSetup& log_setup) {
  const auto backend_path = log_setup.getLogDir("backend");
  const std::string output_csv = backend_path + "/proposed_merge.csv";
  std::lock_guard<std::mutex> lock(proposed_merges_mutex_);
  return proposed_merges_.save(output_csv);
}

bool Backend::save4DMap(const std::string& path) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (map_.save(path)) {
    CLOG(1) << "Saved 4D map with " << map_.numTimeSteps() << " time steps to '" << path << "'.";
    return true;
  }
  return false;
}

void Backend::save(const hydra::LogSetup& log_setup) {
  const auto backend_path = log_setup.getLogDir("backend");
  private_dsg_->graph->save(backend_path + "/dsg_with_mesh");

  // Save the proposed merges.
  saveProposedMerges(log_setup);

  // Save the detected changes.
  const Changes& changes = change_detector_->getChanges();
  if (!changes.object_changes.empty()) {
    const std::string save_file = backend_path + "/object_changes.csv";
    changes.object_changes.save(save_file);
  }
  if (!changes.background_changes.empty()) {
    const std::string save_file = backend_path + "/background_changes.csv";
    changes.background_changes.save(save_file);
  }
}

void Backend::fixInputPoses(const hydra::BackendInput& input) {
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> prior_measurements;
  for (const auto& msg : input.agent_updates.pose_graphs) {
    status_.new_factors += msg.edges.size();

    for (const auto& node : msg.nodes) {
      if (node.key == 0) {
        continue;  // This should already be set by 'add_initial_prior'
      }

      prior_measurements.push_back(
          {gtsam::Symbol(kimera_pgmo::GetRobotPrefix(node.robot_id), node.key),
           gtsam::Pose3(node.pose.matrix())});
    }
  }
  deformation_graph_->addNodeMeasurements(prior_measurements, config.fix_input_pose_variance);
}

}  // namespace khronos
