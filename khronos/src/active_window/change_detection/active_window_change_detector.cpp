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

#include "khronos/active_window/change_detection/active_window_change_detector.h"

#include <algorithm>
#include <limits>

#include <config_utilities/types/enum.h>
#include <config_utilities/types/path.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph_types.h>
#include <spatial_hash/grid.h>

#include "khronos/active_window/tracking/semantic_matching.h"
#include "khronos/utils/icp_registration_utils.h"

namespace khronos {
namespace {

static const auto registration = config::RegistrationWithConfig<ActiveWindow::KhronosSink,
                                                                ActiveWindowChangeDetector,
                                                                ActiveWindowChangeDetector::Config>(
    "ActiveWindowChangeDetector");
}

void declare_config(ActiveWindowChangeDetector::Config& config) {
  using namespace config;
  name("ActiveWindowChangeDetector");
  field(config.verbosity, "verbosity");
  field(config.removal_vertex_free_ratio_threshold, "removal_vertex_free_ratio_threshold");
  field(config.removal_ema_alpha, "removal_ema_alpha");
  field(config.removal_probability_threshold, "removal_probability_threshold");
  field(config.removal_min_frames_observed, "removal_min_frames_observed");
  field(config.added_object_containment_threshold, "added_object_containment_threshold");
  field(config.added_ema_alpha, "added_ema_alpha");
  field(config.added_probability_threshold, "added_probability_threshold");
  field(config.added_min_frames_observed, "added_min_frames_observed");
  field(config.added_prune_after_frames, "added_prune_after_frames");
  field(config.enable_added_object_merging, "enable_added_object_merging");
  field(config.merge_bbox_iou_threshold, "merge_bbox_iou_threshold");
  field(config.merge_centroid_distance_threshold, "merge_centroid_distance_threshold", "m");
  field(config.merge_min_semantic_cosine_sim, "merge_min_semantic_cosine_sim");
  field(config.reassociate_every_frame, "reassociate_every_frame");
  enum_field(config.bbox_merge_type,
             "bbox_merge_type",
             std::vector<std::string>{"union", "weighted_average"});
  field<Path::Absolute>(config.prior_map_path, "prior_map_path");
  field(config.awcd_sinks, "awcd_sinks");
  field(config.transformation_getter, "transformation_getter");
  field(config.enable_icp_refinement, "enable_icp_refinement");
  field(config.icp_crop_radius, "icp_crop_radius");
  field(config.icp_num_threads, "icp_num_threads");
  field(config.icp_downsampling_resolution, "icp_downsampling_resolution");
  field(config.icp_max_correspondence_distance, "icp_max_correspondence_distance");
  field(config.icp_max_iterations, "icp_max_iterations");
  field(config.icp_min_inliers, "icp_min_inliers");
  checkInRange(
      config.merge_min_semantic_cosine_sim, -1.0f, 1.0f, "merge_min_semantic_cosine_sim");
  check<Path::Exists>(config.prior_map_path, "prior_map_path");
}

ActiveWindowChangeDetector::ActiveWindowChangeDetector(const Config& config)
    : config(config::checkValid(config)),
      transformation_getter_(config.transformation_getter.create()),
      sinks_(ActiveWindowCDSink::instantiate(config.awcd_sinks)) {
  MLOG(1) << "[Khronos Active Window Change Detector] Initialized with prior map path: "
          << config.prior_map_path;
  loadPriorMap();
  MLOG(1) << "[Khronos Active Window Change Detector] Loaded prior scene graph with "
          << (prior_graph_ ? std::to_string(prior_graph_->numNodes()) + " nodes." : "0 nodes.");
  // print out the config
  MLOG(1) << "[Khronos Active Window Change Detector] Config: " << config;
}

void ActiveWindowChangeDetector::addKhronosSink(const ActiveWindowCDSink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

std::unordered_map<spark_dsg::NodeId, float> ActiveWindowChangeDetector::computeFreeRatios(
    const std::vector<spark_dsg::NodeId>& objects_id_in_bounds,
    const VolumetricMap& map) const {
  std::unordered_map<spark_dsg::NodeId, float> measurements;

  for (const auto object_id : objects_id_in_bounds) {
    const auto& object_node = prior_graph_->getNode(object_id);

    const auto* khronos_attrs = object_node.tryAttributes<KhronosObjectAttributes>();
    if (!khronos_attrs) {
      MLOG(5) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(object_id).str()
              << " does not have KhronosObjectAttributes, skipping";
      continue;
    }

    const auto& mesh = khronos_attrs->mesh;
    const auto& bbox = khronos_attrs->bounding_box;

    if (mesh.numVertices() == 0) {
      MLOG(5) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(object_id).str()
              << " has empty mesh, skipping";
      continue;
    }

    int num_vertices_in_free_space = 0;
    int num_vertices_in_bound_and_known = 0;

    for (size_t i = 0; i < mesh.numVertices(); ++i) {
      // Transform: local → prior_world → current_world
      const Eigen::Vector3f vertex_local = mesh.pos(i);
      const Eigen::Vector3f vertex_prior_world = bbox.pointToWorldFrame(vertex_local);
      const Point vertex_current = transformPriorToCurrentFrame(vertex_prior_world.cast<double>());

      if (!isPointKnown(vertex_current, map)) {
        continue;  // Skip unknown points — no valid measurement at this vertex.
      }

      ++num_vertices_in_bound_and_known;

      if (isPriorPointFree(vertex_current, map)) {
        ++num_vertices_in_free_space;
      }
    }

    // Guard division: if no known vertices observed this frame, there is no valid measurement.
    // Skip to avoid feeding NaN/garbage into the EMA filter.
    if (num_vertices_in_bound_and_known == 0) {
      MLOG(5) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(object_id).str()
              << " has no known vertices this frame, skipping measurement";
      continue;
    }

    const float free_ratio = static_cast<float>(num_vertices_in_free_space) /
                             static_cast<float>(num_vertices_in_bound_and_known);

    measurements[object_id] = free_ratio;
    MLOG(4) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(object_id).str()
            << " raw free ratio: " << free_ratio;
  }

  return measurements;
}

std::vector<ActiveWindowChangeDetector::RemovedObject> ActiveWindowChangeDetector::updateRemovedFilter(
    const std::unordered_map<spark_dsg::NodeId, float>& measurements, TimeStamp stamp) const {
  // Update EMA state for each object that has a measurement this frame.
  // Objects absent from measurements are left untouched (freeze-last policy).
  for (const auto& [object_id, free_ratio] : measurements) {
    auto& state = removed_object_states_[object_id];

    if (state.num_frames_observed == 0) {
      // First observation: initialize the filter to the raw measurement.
      state.free_probability = free_ratio;
    } else {
      // EMA update: p = alpha * free_ratio + (1 - alpha) * p
      state.free_probability =
          config.removal_ema_alpha * free_ratio + (1.0 - config.removal_ema_alpha) * state.free_probability;
    }
    ++state.num_frames_observed;
    state.last_free_ratio = free_ratio;

    MLOG(4) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(object_id).str()
            << " EMA update: raw=" << free_ratio
            << " smoothed=" << state.free_probability
            << " frames=" << state.num_frames_observed;
  }

  // Threshold the full state map (not just this frame's measurements) to build the removed set.
  // This means objects that leave the map temporarily retain their last smoothed probability.
  std::vector<RemovedObject> removed_objects;
  for (auto& [object_id, state] : removed_object_states_) {
    const bool above_prob = state.free_probability >= config.removal_probability_threshold;
    const bool enough_frames = state.num_frames_observed >= config.removal_min_frames_observed;
    if (above_prob && enough_frames) {
      // Latch the first sensor time this object was declared removed. Never reset, even if
      // the object later drops out of the removed set and returns.
      if (state.first_removed_ns == 0) {
        state.first_removed_ns = stamp;
      }
      removed_objects.push_back(RemovedObject{object_id,
                                              state.first_removed_ns,
                                              static_cast<float>(state.free_probability),
                                              state.num_frames_observed});
      MLOG(3) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(object_id).str()
              << " REMOVED (p=" << state.free_probability
              << ", frames=" << state.num_frames_observed << ")";
    } else {
      MLOG(4) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(object_id).str()
              << " not removed (p=" << state.free_probability
              << ", frames=" << state.num_frames_observed << ")";
    }
  }

  MLOG(2) << "[ActiveWindowChangeDetector] Removed-object filter: "
          << removed_objects.size() << " removed out of "
          << removed_object_states_.size() << " tracked candidates";

  return removed_objects;
}

GlobalIndex ActiveWindowChangeDetector::to2DIndex(const Point& point, float voxel_size_inv) {
  GlobalIndex idx = spatial_hash::indexFromPoint<GlobalIndex>(point, voxel_size_inv);
  idx.z() = 0;
  return idx;
}

GlobalIndexSet ActiveWindowChangeDetector::getPriorFreeFootprint2D(
    const VolumetricMap& map) const {
  GlobalIndexSet free_footprint;

  if (!prior_graph_ || !prior_graph_->hasLayer(DsgLayers::MESH_PLACES)) {
    LOG(WARNING) << "[ActiveWindowChangeDetector] Prior graph has no MESH_PLACES layer; "
                    "newly-added object detection requires traversability places.";
    return free_footprint;
  }

  const auto& places_layer = prior_graph_->getLayer(DsgLayers::MESH_PLACES);
  const float voxel_size = map.config.voxel_size;
  const float voxel_size_inv = 1.0f / voxel_size;
  // Step at half voxel size in prior frame to avoid coverage gaps after transform.
  const float step = voxel_size * 0.5f;

  int num_trav_nodes = 0;
  for (const auto& [node_id, node] : places_layer.nodes()) {
    const auto* attrs = node->tryAttributes<spark_dsg::TravNodeAttributes>();
    if (!attrs) {
      continue;
    }
    ++num_trav_nodes;

    // Filter to places whose center falls inside the current active-window map.
    const Point center_current =
        transformPriorToCurrentFrame(attrs->position);
    if (!isPointInMapBounds(center_current, map)) {
      continue;
    }

    // Rasterize the place's footprint in prior-map frame, then index in current frame.
    const double max_r = attrs->max_radius;
    const double cx = attrs->position.x();
    const double cy = attrs->position.y();
    const double cz = attrs->position.z();

    for (double x = cx - max_r; x <= cx + max_r; x += step) {
      for (double y = cy - max_r; y <= cy + max_r; y += step) {
        const Eigen::Vector3d candidate_prior(x, y, cz);
        if (!attrs->contains(candidate_prior)) {
          continue;
        }
        const Point pt_current = transformPriorToCurrentFrame(candidate_prior);
        free_footprint.insert(to2DIndex(pt_current, voxel_size_inv));
      }
    }
  }

  if (num_trav_nodes == 0) {
    LOG(WARNING) << "[ActiveWindowChangeDetector] MESH_PLACES layer exists but contains no "
                    "TravNodeAttributes nodes. Prior map may have been built without traversability "
                    "places; newly-added object detection will be a no-op.";
  }

  MLOG(3) << "[ActiveWindowChangeDetector] Prior free 2D footprint: " << free_footprint.size()
          << " voxels from " << num_trav_nodes << " traversability place nodes.";
  return free_footprint;
}

GlobalIndexSet ActiveWindowChangeDetector::getTrackFootprint2D(const Track& track,
                                                               float voxel_size) const {
  GlobalIndexSet footprint;
  const float voxel_size_inv = 1.0f / voxel_size;
  for (const Point& pt : track.last_points) {
    footprint.insert(to2DIndex(pt, voxel_size_inv));
  }
  return footprint;
}

std::unordered_map<int, float> ActiveWindowChangeDetector::computeContainmentRatios(
    const Tracks& tracks,
    const VolumetricMap& map) const {
  std::unordered_map<int, float> measurements;

  // Build the 2D free-space footprint from prior traversability places.
  const GlobalIndexSet prior_free = getPriorFreeFootprint2D(map);
  if (prior_free.empty()) {
    return measurements;
  }

  const float voxel_size = map.config.voxel_size;

  for (const Track& track : tracks) {
    // Skip dynamic objects and tracks with no point observations.
    if (track.is_dynamic || track.last_points.empty()) {
      continue;
    }

    const GlobalIndexSet track2D = getTrackFootprint2D(track, voxel_size);
    if (track2D.empty()) {
      continue;
    }

    // Containment ratio: fraction of the track's 2D footprint that falls in prior free space.
    int intersection = 0;
    for (const GlobalIndex& idx : track2D) {
      if (prior_free.count(idx)) {
        ++intersection;
      }
    }
    const float containment = static_cast<float>(intersection) / static_cast<float>(track2D.size());

    measurements[track.id] = containment;
    MLOG(4) << "[ActiveWindowChangeDetector] Track " << track.id
            << " raw containment: " << containment;
  }

  return measurements;
}

int ActiveWindowChangeDetector::findBestObjectMatch(
    const BoundingBox& bbox,
    const std::optional<SemanticClusterInfo>& semantics,
    int exclude_id) const {
  if (!config.enable_added_object_merging || !bbox.isValid()) {
    return -1;
  }
  const Point centroid = bbox.world_P_center;

  int best_id = -1;
  float best_iou = -1.0f;
  float best_dist = std::numeric_limits<float>::max();

  for (const auto& [object_id, state] : added_object_states_) {
    if (object_id == exclude_id || !state.bounding_box.isValid()) {
      continue;
    }

    // Semantics must match (category_id + feature cosine sim); geometry is an OR of bbox IoU
    // and centroid distance (either disabled by setting its threshold <= 0).
    const auto semantic_result =
        semanticsMatch(semantics, state.semantics, config.merge_min_semantic_cosine_sim);
    if (!semantic_result) {
      continue;
    }

    const float iou = state.bounding_box.computeIoU(bbox);
    const float dist = (state.bounding_box.world_P_center - centroid).norm();

    bool geometric_match = config.merge_bbox_iou_threshold > 0.0f &&
                           iou >= config.merge_bbox_iou_threshold;
    if (!geometric_match && config.merge_centroid_distance_threshold > 0.0f) {
      geometric_match = dist < config.merge_centroid_distance_threshold;
    }
    if (!geometric_match) {
      continue;
    }

    // Prefer the highest-IoU candidate; break ties with the smaller centroid distance.
    if (iou > best_iou || (iou == best_iou && dist < best_dist)) {
      best_iou = iou;
      best_dist = dist;
      best_id = object_id;
    }
  }

  return best_id;
}

void ActiveWindowChangeDetector::foldTrackIntoObject(AddedObjectState& state,
                                                      const Track& track,
                                                      const TrackAddedState& track_state) const {
  const bool first_member = state.member_track_ids.empty();

  if (first_member) {
    // Nothing to blend with yet -- initialize the object directly from this track.
    state.bounding_box = track.last_bounding_box;
    state.first_seen = track.first_seen;
    state.last_seen = track.last_seen;
    state.semantics = track.semantics;
    state.confidence = track.confidence;
  } else {
    // Fold the track's bounding box into the object's per Config::bbox_merge_type.
    const float w_obj = state.confidence;
    const float w_trk = track.confidence;
    const float total = w_obj + w_trk;
    if (config.bbox_merge_type == Config::BboxMergeType::kUnion ||
        !state.bounding_box.isValid() || !track.last_bounding_box.isValid() || total <= 0.0f) {
      state.bounding_box.merge(track.last_bounding_box);
    } else if (state.bounding_box.type == BoundingBox::Type::AABB &&
               track.last_bounding_box.type == BoundingBox::Type::AABB) {
      // AABB: average the min/max corner points component-wise, then re-derive center/dims.
      // (minCorner()/maxCorner() are protected on BoundingBox, so derive them from
      // world_P_center +/- dimensions/2, valid since AABB has no rotation.)
      const Eigen::Vector3f obj_min =
          state.bounding_box.world_P_center - state.bounding_box.dimensions * 0.5f;
      const Eigen::Vector3f obj_max =
          state.bounding_box.world_P_center + state.bounding_box.dimensions * 0.5f;
      const Eigen::Vector3f trk_min = track.last_bounding_box.world_P_center -
                                      track.last_bounding_box.dimensions * 0.5f;
      const Eigen::Vector3f trk_max = track.last_bounding_box.world_P_center +
                                      track.last_bounding_box.dimensions * 0.5f;
      const Eigen::Vector3f new_min = (w_obj * obj_min + w_trk * trk_min) / total;
      const Eigen::Vector3f new_max = (w_obj * obj_max + w_trk * trk_max) / total;
      state.bounding_box = BoundingBox(new_max - new_min, (new_min + new_max) * 0.5f);
    } else {
      // OBB/RAABB: average centroid and extent directly. Orientation is inherited from whichever
      // side has the larger weight (proper rotation averaging, e.g. SLERP, is not implemented).
      const Eigen::Vector3f new_center = (w_obj * state.bounding_box.world_P_center +
                                          w_trk * track.last_bounding_box.world_P_center) /
                                         total;
      const Eigen::Vector3f new_dims = (w_obj * state.bounding_box.dimensions +
                                        w_trk * track.last_bounding_box.dimensions) /
                                       total;
      const Eigen::Matrix3f& new_rotation = w_trk > w_obj ? track.last_bounding_box.world_R_center
                                                           : state.bounding_box.world_R_center;
      state.bounding_box =
          BoundingBox(state.bounding_box.type, new_dims, new_center, new_rotation);
    }

    state.first_seen = std::min(state.first_seen, track.first_seen);
    state.last_seen = std::max(state.last_seen, track.last_seen);
    state.confidence = std::max(state.confidence, track.confidence);

    // Fuse semantics similarly to Track::updateSemantics: prefer to keep/acquire openset
    // features, and average them (unweighted) when both sides carry one.
    if (track.semantics) {
      if (!state.semantics) {
        state.semantics = track.semantics;
      } else {
        const bool obj_has_feature = state.semantics->feature.size() != 1;
        const bool track_has_feature = track.semantics->feature.size() != 1;
        if (track_has_feature && !obj_has_feature) {
          state.semantics->feature = track.semantics->feature;
        } else if (track_has_feature && obj_has_feature) {
          state.semantics->feature =
              0.5f * state.semantics->feature + 0.5f * track.semantics->feature;
        }
      }
    }
  }

  state.change_confidence = std::max(state.change_confidence, track_state.change_confidence);
  state.num_frames_observed = std::max(state.num_frames_observed, track_state.num_frames_observed);
  state.member_track_ids.insert(track.id);
  state.last_updated_frame = frame_index_;
}

std::vector<ActiveWindowChangeDetector::AddedObject> ActiveWindowChangeDetector::updateAddedFilter(
    const std::unordered_map<int, float>& measurements,
    const Tracks& tracks) const {
  ++frame_index_;

  // Build a quick lookup from track ID to Track pointer for this frame's live info.
  std::unordered_map<int, const Track*> track_lookup;
  track_lookup.reserve(tracks.size());
  for (const Track& t : tracks) {
    track_lookup[t.id] = &t;
  }

  for (const auto& [track_id, containment] : measurements) {
    // 1. Per-track EMA update, independent of any object association.
    auto& track_state = track_added_states_[track_id];
    if (track_state.num_frames_observed == 0) {
      track_state.change_confidence = containment;
    } else {
      track_state.change_confidence = config.added_ema_alpha * containment +
                                      (1.0 - config.added_ema_alpha) * track_state.change_confidence;
    }
    ++track_state.num_frames_observed;
    track_state.last_containment = containment;
    track_state.last_updated_frame = frame_index_;

    MLOG(4) << "[ActiveWindowChangeDetector] Track " << track_id
            << " EMA update: raw=" << containment
            << " smoothed=" << track_state.change_confidence
            << " frames=" << track_state.num_frames_observed;

    const auto lookup_it = track_lookup.find(track_id);
    if (lookup_it == track_lookup.end()) {
      continue;  // Should not happen (measurement implies the track is in `tracks`); guard anyway.
    }
    const Track& track = *lookup_it->second;

    // 2. Association: already-associated tracks fold into their object (optionally re-checking
    // for a better match first); unassociated tracks search for a match or create a new object.
    if (track_state.object_id != -1) {
      if (config.reassociate_every_frame) {
        const int best_id = findBestObjectMatch(
            track.last_bounding_box, track.semantics, track_state.object_id);
        if (best_id != -1 && best_id != track_state.object_id) {
          MLOG(3) << "[ActiveWindowChangeDetector] Track " << track_id
                  << " re-associated from object " << track_state.object_id << " to " << best_id;
          track_state.object_id = best_id;
        }
      }
      foldTrackIntoObject(added_object_states_[track_state.object_id], track, track_state);
    } else {
      const int best_id = findBestObjectMatch(track.last_bounding_box, track.semantics);
      const int object_id = best_id != -1 ? best_id : next_object_id_++;
      MLOG(4) << "[ActiveWindowChangeDetector] Track " << track_id
              << (best_id != -1 ? " associated to existing object " : " created new object ")
              << object_id;
      track_state.object_id = object_id;
      foldTrackIntoObject(added_object_states_[object_id], track, track_state);
    }
  }

  // 3. Gate & prune.
  std::vector<AddedObject> newly_added_objects;
  for (auto it = added_object_states_.begin(); it != added_object_states_.end();) {
    const int object_id = it->first;
    auto& state = it->second;

    const bool above_prob = state.change_confidence >= config.added_probability_threshold;
    const bool enough_frames = state.num_frames_observed >= config.added_min_frames_observed;

    if (above_prob && enough_frames) {
      state.ever_added = true;

      AddedObject obj;
      obj.id = object_id;
      obj.member_track_ids = state.member_track_ids;
      obj.first_seen = state.first_seen;
      obj.last_seen = state.last_seen;
      obj.bounding_box = state.bounding_box;
      obj.centroid = state.bounding_box.isValid() ? state.bounding_box.world_P_center : Point::Zero();
      obj.semantics = state.semantics;
      obj.confidence = state.confidence;
      obj.change_confidence = static_cast<float>(state.change_confidence);
      obj.num_frames_observed = state.num_frames_observed;
      newly_added_objects.push_back(std::move(obj));

      MLOG(3) << "[ActiveWindowChangeDetector] Object " << object_id
              << " ADDED (confidence=" << state.confidence
              << ", change_confidence=" << state.change_confidence
              << ", frames=" << state.num_frames_observed
              << ", members=" << state.member_track_ids.size() << ")";
      ++it;
    } else if (!state.ever_added &&
               (frame_index_ - state.last_updated_frame) > config.added_prune_after_frames) {
      // Stale non-added record — prune it, freeing its member tracks to re-associate elsewhere.
      MLOG(4) << "[ActiveWindowChangeDetector] Pruning stale non-added object " << object_id;
      for (const int member_track_id : state.member_track_ids) {
        auto ts_it = track_added_states_.find(member_track_id);
        if (ts_it != track_added_states_.end() && ts_it->second.object_id == object_id) {
          ts_it->second.object_id = -1;
        }
      }
      it = added_object_states_.erase(it);
    } else {
      MLOG(4) << "[ActiveWindowChangeDetector] Object " << object_id
              << " not added (change_confidence=" << state.change_confidence
              << ", frames=" << state.num_frames_observed << ")";
      ++it;
    }
  }

  MLOG(2) << "[ActiveWindowChangeDetector] Added-object filter: "
          << newly_added_objects.size() << " added out of "
          << added_object_states_.size() << " tracked candidates";

  return newly_added_objects;
}

void ActiveWindowChangeDetector::call(const FrameData& data,
                                      const VolumetricMap& map,
                                      const Tracks& tracks) const {
  // Poll the transformation getter and update current_T_prior_ if a new transform is available.
  const auto tf = transformation_getter_->getTransformation();
  if (tf.has_value()) {
    // Loop residual: the getter reads map->odom, which traverses the pre_icp_odom->odom edge that
    // TfIcpPublisher derives from current_T_prior_. So this delta is what the world moved since we
    // last looked: last frame's ICP correction + any ROMAN update. It should shrink toward zero as
    // ICP converges; a large or oscillating value means the TF chain disagrees with current_T_prior_.
    const Eigen::Isometry3d loop_residual = current_T_prior_.inverse() * tf.value();
    MLOG(2) << "[ActiveWindowChangeDetector] TF readback loop residual: "
            << loop_residual.translation().norm() << " m, "
            << Eigen::AngleAxisd(loop_residual.linear()).angle() << " rad.";
    if (config.enable_icp_refinement) {
      runIcpRefinement(data, map, tf.value());
    } else {
      setCurrentToPriorTransform(tf.value());
    }
  }

  // 1. Find all object nodes in the prior graph within current volumetric map bounds.
  const auto objects_id_in_bounds = findPriorObjectsInMapBounds(map);

  // 2. Compute per-frame free ratios for objects with valid measurements.
  const auto free_ratio_measurements = computeFreeRatios(objects_id_in_bounds, map);

  // 3. Update EMA filter and threshold to obtain the temporally-filtered removed set.
  const auto removed_objects = updateRemovedFilter(free_ratio_measurements, data.input.timestamp_ns);

  // 4. Compute per-frame containment ratios for newly-added-object detection.
  // Known limitation: objects on tables cannot be detected this way (footprint-on-ground heuristic).
  const auto containment_measurements = computeContainmentRatios(tracks, map);

  // 5. Update EMA filter, merge fragmented tracks, prune stale records, and return fused objects.
  const auto newly_added_objects = updateAddedFilter(containment_measurements, tracks);

  MLOG(2) << "[ActiveWindowChangeDetector] Detected " << removed_objects.size()
          << " removed objects and " << newly_added_objects.size() << " newly added objects.";

  // Call all sinks with removed objects, newly-added objects, and the prior-to-current transform.
  ActiveWindowCDSink::callAll(
      sinks_, prior_graph_, removed_objects, newly_added_objects, current_T_prior_);
}

void ActiveWindowChangeDetector::loadPriorMap() {
  // NOTE(multy): required to load the full path to the DSG file.
  // TODO(multy): in documentation might require to set a prior map path that's different from the
  // DCIST env variable.
  prior_graph_ = DynamicSceneGraph::load(config.prior_map_path);
  MLOG(1) << "[ActiveWindowChangeDetector] Loaded prior graph from " << config.prior_map_path
          << " with "
          << (prior_graph_ ? std::to_string(prior_graph_->numNodes()) + " nodes." : "0 nodes.");
}

bool ActiveWindowChangeDetector::isPriorPointFree(const Point& point_in_map,
                                                  const VolumetricMap& map) const {
  const auto* voxel = map.getTrackingLayer()->getVoxelPtr(point_in_map);
  return voxel && voxel->ever_free;
}

bool ActiveWindowChangeDetector::isPointInMapBounds(const Point& point,
                                                    const VolumetricMap& map) const {
  // A point is "in bounds" if the map has an allocated block at that location
  const auto& tsdf_layer = map.getTsdfLayer();
  return tsdf_layer.hasBlock(point.cast<float>());
}

bool ActiveWindowChangeDetector::isPointKnown(const Point& point_in_map,
                                              const VolumetricMap& map) const {
  const auto* voxel = map.getTrackingLayer()->getVoxelPtr(point_in_map);
  return voxel && voxel->last_observed != 0u;
}

std::vector<spark_dsg::NodeId> ActiveWindowChangeDetector::findPriorObjectsInMapBounds(
    const VolumetricMap& map) const {
  std::vector<spark_dsg::NodeId> objects_in_bounds;

  if (!prior_graph_ || !prior_graph_->hasLayer(DsgLayers::OBJECTS)) {
    LOG(WARNING) << "[ActiveWindowChangeDetector] Prior graph has no OBJECTS layer";
    return objects_in_bounds;
  }

  const auto& objects_layer = prior_graph_->getLayer(DsgLayers::OBJECTS);

  for (const auto& [node_id, node] : objects_layer.nodes()) {
    const auto& attrs = node->attributes();
    // Transform position from prior map frame to current map frame
    const Point position_in_current = transformPriorToCurrentFrame(attrs.position);

    if (isPointInMapBounds(position_in_current, map)) {
      objects_in_bounds.push_back(node_id);
      MLOG(4) << "[ActiveWindowChangeDetector] Object " << spark_dsg::NodeSymbol(node_id).str()
              << " is within map bounds at position (current frame): "
              << position_in_current.transpose();
    }
  }

  MLOG(4) << "[ActiveWindowChangeDetector] Found " << objects_in_bounds.size()
          << " prior objects within current map bounds";

  return objects_in_bounds;
}

void ActiveWindowChangeDetector::setCurrentToPriorTransform(
    const Eigen::Isometry3d& current_T_prior) const {
  current_T_prior_ = current_T_prior;
  MLOG(1) << "[ActiveWindowChangeDetector] Updated current_T_prior transform:\n"
          << "  Translation: " << current_T_prior_.translation().transpose() << "\n"
          << "  Rotation (quaternion wxyz): "
          << Eigen::Quaterniond(current_T_prior_.rotation()).coeffs().transpose();
}

Point ActiveWindowChangeDetector::transformPriorToCurrentFrame(
    const Eigen::Vector3d& point_in_prior) const {
  // TODO(multy): In the future, consider transforming the prior map once to the current map frame
  // instead of transforming each point.
  // Transform: current_point = current_T_prior * prior_point
  const Eigen::Vector3d point_in_current = current_T_prior_ * point_in_prior;
  return point_in_current.cast<float>();
}

void ActiveWindowChangeDetector::runIcpRefinement(const FrameData& data,
                                                  const VolumetricMap& map,
                                                  const Eigen::Isometry3d& initial) const {
  if (!prior_graph_ || !prior_graph_->mesh() || prior_graph_->mesh()->points.empty()) {
    LOG(WARNING) << "[ActiveWindowChangeDetector] No prior background mesh for ICP.";
    return;
  }

  const Eigen::Vector3f robot_cur = data.input.world_T_body.translation().cast<float>();
  const Eigen::Isometry3f prior_T_current = initial.inverse().cast<float>();
  const Eigen::Vector3f robot_prior = prior_T_current * robot_cur;
  const float r = config.icp_crop_radius;

  // Collect current mesh points and pre-transform to prior frame.
  std::vector<Eigen::Vector3f> source;
  for (const auto& block : map.getMeshLayer()) {
    for (const auto& pt : block.points) {
      if ((pt - robot_cur).norm() <= r) {
        source.push_back(prior_T_current * pt);
      }
    }
  }

  // Collect prior background mesh points near robot in prior frame.
  std::vector<Eigen::Vector3f> target;
  for (const auto& pt : prior_graph_->mesh()->points) {
    if ((pt - robot_prior).norm() <= r) {
      target.push_back(pt);
    }
  }

  if (source.empty() || target.empty()) {
    LOG(WARNING) << "[ActiveWindowChangeDetector] Insufficient mesh points for ICP.";
    return;
  }
  MLOG(1) << "[ActiveWindowChangeDetector] ICP: " << source.size() << " src, " << target.size()
          << " tgt pts.";

  const auto res =
      ICPRegistrationUtils::registerPointClouds(source,
                                                target,
                                                config.icp_num_threads,
                                                config.icp_downsampling_resolution,
                                                config.icp_max_correspondence_distance,
                                                config.icp_max_iterations);

  if (!res.converged || res.num_inliers < config.icp_min_inliers) {
    LOG(WARNING) << "[ActiveWindowChangeDetector] ICP failed: converged=" << res.converged
                 << " inliers=" << res.num_inliers;
    return;
  }

  setCurrentToPriorTransform(initial * res.T_target_source.inverse());
  MLOG(1) << "[ActiveWindowChangeDetector] ICP refined transform. Inliers: " << res.num_inliers
          << ", translation delta: "
          << (current_T_prior_.translation() - initial.translation()).norm() << " m.";

  // print initial guess and refined transform for debugging in x,y,z and euler angles x,y,z
  const Eigen::Vector3d initial_trans = initial.translation();
  const Eigen::Vector3d initial_euler = initial.rotation().eulerAngles(0, 1, 2);
  const Eigen::Vector3d refined_trans = current_T_prior_.translation();
  const Eigen::Vector3d refined_euler = current_T_prior_.rotation().eulerAngles(0, 1, 2);

  MLOG(1) << "[ActiveWindowChangeDetector] Initial guess - Translation (x,y,z): "
          << initial_trans.transpose() << ", Euler angles (x,y,z): " << initial_euler.transpose();
  MLOG(1) << "[ActiveWindowChangeDetector] Refined transform - Translation (x,y,z): "
          << refined_trans.transpose() << ", Euler angles (x,y,z): " << refined_euler.transpose();
}

}  // namespace khronos
