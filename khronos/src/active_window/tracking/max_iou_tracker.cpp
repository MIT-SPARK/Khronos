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

#include "khronos/active_window/tracking/max_iou_tracker.h"

#include <algorithm>
#include <string>
#include <vector>

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <hydra/openset/embedding_distances.h>

#include "khronos/active_window/data/reconstruction_types.h"
#include "khronos/active_window/tracking/semantic_matching.h"
#include "khronos/utils/geometry_utils.h"

namespace khronos {
namespace {

static const auto registration =
    config::RegistrationWithConfig<Tracker, MaxIoUTracker, MaxIoUTracker::Config>("MaxIouTracker");

std::optional<uint32_t> getSemanticId(const Track& track) {
  if (track.observations.empty()) {
    return std::nullopt;
  }

  // NOTE(nathan) this has the potential to be really inefficient (linear traversal of observations
  // for every track in the worst case) when we allow dynamic tracks. we could fix this by adding a
  // summary semantic id to the track, but we need to clean up how track representative statistics
  // work

  // get the most recent semantic ID in a track by iterating in reverse
  for (auto iter = track.observations.rbegin(); iter != track.observations.rend(); ++iter) {
    const auto& obs = *iter;
    if (obs.semantic_cluster_id >= 0) {
      return obs.semantic_cluster_id;
    }
  }

  return std::nullopt;
}

}  // namespace

void declare_config(MaxIoUTracker::Config& config) {
  using namespace config;
  name("MaxIoUTracker");
  base<hydra::VerbosityConfig>(config);
  enum_field(config.track_by, "track_by", {"pixels", "voxels", "bounding_box"});
  enum_field(config.semantic_association,
             "semantic_association",
             std::vector<std::string>{"assign_cluster", "assign_track"});
  enum_field(config.bbox_type, "bbox_type", std::vector<std::string>{"aabb", "raabb"});
  field(config.min_semantic_iou, "min_semantic_iou");
  field(config.min_cosine_sim, "min_cosine_sim");
  field(config.min_cross_iou, "min_cross_iou");
  field(config.max_dynamic_distance, "max_dynamic_distance", "m");
  field(config.min_num_observations, "min_num_observations", "frames");
  field(config.voxel_size, "voxel_size", "m");
  field(config.preassociate_by_id, "preassociate_by_id");
  field(config.preassociate_to_dynamic, "preassociate_to_dynamic");
  field(config.accumulate_voxels, "accumulate_voxels");
  field(config.min_voxel_hits, "min_voxel_hits");
  field(config.merge_tracks, "merge_tracks");
  field(config.merge_containment_threshold, "merge_containment_threshold");
  field(config.merge_distance_factor, "merge_distance_factor");
  field(config.merge_min_cosine_sim, "merge_min_cosine_sim");

  checkInRange(config.min_cross_iou, 0.0f, 1.0f, "min_cross_iou");
  checkInRange(config.min_semantic_iou, 0.0f, 1.0f, "min_semantic_iou");
  checkInRange(config.min_cosine_sim, -1.0f, 1.0f, "min_cosine_sim");
  check(config.voxel_size, GT, 0.f, "voxel_size");
  check(config.min_voxel_hits, GE, 1, "min_voxel_hits");
  checkCondition(!config.merge_tracks || config.accumulate_voxels,
                 "merge_tracks requires accumulate_voxels");
  checkInRange(config.merge_containment_threshold, 0.0f, 1.0f, "merge_containment_threshold");
  check(config.merge_distance_factor, GT, 0.f, "merge_distance_factor");
  checkInRange(config.merge_min_cosine_sim, -1.0f, 1.0f, "merge_min_cosine_sim");
}

MaxIoUTracker::Config::Config()
    : hydra::VerbosityConfig(hydra::VerbosityConfig::default_verbosity("IoU Tracker")) {}

MaxIoUTracker::MaxIoUTracker(const Config& config)
    : config(config::checkValid(config)), grid(config.voxel_size) {}

float MaxIoUTracker::computeIoU(const FrameData& data,
                                const MeasurementCluster& cluster,
                                const Track& track) const {
  // Compute IoU on the corresponding measurement for track_by setting
  switch (config.track_by) {
    case Config::TrackBy::kPixels:
      return computeIoUPixels(data, cluster, track);
    case Config::TrackBy::kVoxels:
      return computeIoUVoxels(data, cluster, track);
    case Config::TrackBy::kBoundingBox:
    default:
      return computeIoUBoundingBox(data, cluster, track);
      break;
  }
}

void MaxIoUTracker::processInput(FrameData& data, Tracks& tracks) {
  Timer timer("tracking/all", data.input.timestamp_ns);

  // Compute values the objects are going to be tracked by (pixels, voxels, or bounding box).
  setupTrackMeasurements(data);

  // Initialize assignment tracking for input clusters
  const auto prev_num_tracks = tracks.size();
  dynamic_assigned_ = std::vector<bool>(data.dynamic_clusters.size(), false);
  semantic_assigned_ = std::vector<bool>(data.semantic_clusters.size(), false);

  // Associate current objects to tracks and create new tracks for unassociated objects.
  // TODO(lschmid): Handle objects splitting or merging explicitly at some point.
  associateTracks(data, tracks);

  // Merge fragment tracks of the same object.
  if (config.merge_tracks) {
    mergeTracks(tracks);
  }

  MLOG(3) << "Previous tracks: " << prev_num_tracks << " Current tracks: " << tracks.size();
  ++sequence_number_;
}

void MaxIoUTracker::associateTracks(const FrameData& data, Tracks& tracks) {
  Timer timer("tracking/associate", data.input.timestamp_ns);

  // Associate dynamic clusters first, allocating new dynamic tracks if no match is found.
  associateDynamicTracks(data, tracks);

  // Optionally assign clusters to tracks by matching ID (for external tracking)
  if (config.preassociate_by_id) {
    preassociateSemanticTracks(data, tracks);
  }

  // TODO(nathan) this needs to be revamped to combine a static track with a dynamic track for
  // objects that were static but start moving when observed

  // Associate semantic clusters to dynamic tracks
  crossAssociateTracks(data, tracks);

  // Then associate semantic clusters to all other tracks
  associateSemanticTracks(data, tracks);
}

// Assign dynamic tracks to closest dynamic clusters that moved less than the maximum.
void MaxIoUTracker::associateDynamicTracks(const FrameData& data, Tracks& tracks) {
  MLOG(4) << "Associating " << data.dynamic_clusters.size() << " dynamic detections to tracks";

  size_t num_assoc = 0;
  for (auto& track : tracks) {
    if (!track.is_dynamic) {
      continue;
    }

    if (track.sequence_number == sequence_number_) {
      continue;  // skip track updated this pass
    }

    std::optional<size_t> best_cluster;
    float best_distance = config.max_dynamic_distance;
    Point best_centroid = Point::Zero();
    for (size_t i = 0; i < data.dynamic_clusters.size(); ++i) {
      const auto& cluster = data.dynamic_clusters[i];
      if (dynamic_assigned_[i]) {
        continue;
      }

      // Compute the centroid distances.
      const auto centroid = computeCentroid(data, cluster);
      const auto distance = (centroid - track.last_centroid).norm();
      if (distance < best_distance) {
        best_cluster = i;
        best_distance = distance;
        best_centroid = centroid;
      }
    }

    // Associate object if IoU is high enough.
    if (best_cluster) {
      ++num_assoc;
      dynamic_assigned_[*best_cluster] = true;
      updateTrack(data, data.dynamic_clusters[*best_cluster], track, true);
      track.last_centroid = best_centroid;
    }
  }

  // Create new tracks for unassociated objects.
  size_t num_new = 0;
  for (size_t i = 0; i < data.dynamic_clusters.size(); ++i) {
    if (dynamic_assigned_[i]) {
      continue;
    }

    ++num_new;
    const auto& cluster = data.dynamic_clusters[i];
    auto& track = addNewTrack(data, cluster, tracks, true);
    track.last_centroid = computeCentroid(data, cluster);
  }

  MLOG(4) << "Associated " << num_assoc << " dynamic detections with " << num_new << " new tracks";
}

void MaxIoUTracker::preassociateSemanticTracks(const FrameData& data, Tracks& tracks) {
  std::unordered_map<int, size_t> id_to_cluster;
  for (size_t i = 0; i < data.semantic_clusters.size(); ++i) {
    id_to_cluster[data.semantic_clusters[i].id] = i;
  }

  for (auto& track : tracks) {
    if (track.is_dynamic && !config.preassociate_to_dynamic) {
      // NOTE(nathan) this probably shouldn't be turned on until we think through cross-association
      // more fully
      continue;
    }

    const auto last_track_id = getSemanticId(track);
    if (!last_track_id) {
      continue;
    }

    const auto iter = id_to_cluster.find(*last_track_id);
    if (iter == id_to_cluster.end()) {
      continue;  // track has no matching cluster
    }

    const auto& [id, cluster_idx] = *iter;
    if (semantic_assigned_[cluster_idx]) {
      continue;  // we've already associated this cluster to a different track
    }

    semantic_assigned_[cluster_idx] = true;
    const auto& cluster = data.semantic_clusters[cluster_idx];
    if (track.sequence_number == sequence_number_) {
      track.observations.back().semantic_cluster_id = cluster.id;
    } else {
      updateTrack(data, cluster, track, false);
    }
  }

  if (!config.merge_tracks) {
    return;
  }

  // Merged tracks also claim the upstream ids of the tracks they absorbed. This runs after all
  // latest ids are matched, so a track currently using an id keeps it.
  for (auto& track : tracks) {
    if (track.is_dynamic || track.sequence_number == sequence_number_) {
      continue;
    }

    for (const int absorbed_id : track.absorbed_instance_ids) {
      const auto iter = id_to_cluster.find(absorbed_id);
      if (iter == id_to_cluster.end() || semantic_assigned_[iter->second]) {
        continue;
      }

      semantic_assigned_[iter->second] = true;
      updateTrack(data, data.semantic_clusters[iter->second], track, false);
      break;
    }
  }
}

void MaxIoUTracker::crossAssociateTracks(const FrameData& data, Tracks& tracks) {
  MLOG(4) << "Cross associating " << data.semantic_clusters.size() << " semantic clusters";

  // First assign all semantic tracks to dynamic tracks where possible to avoid
  // allocating many semantic tracks for moving objects.
  size_t num_associated = 0;
  for (auto& track : tracks) {
    if (!track.is_dynamic) {
      continue;
    }

    if (track.sequence_number == sequence_number_ &&
        track.observations.back().semantic_cluster_id >= 0) {
      continue;
    }

    std::optional<size_t> best_cluster;
    float best_iou = config.min_cross_iou;
    for (size_t i = 0; i < data.semantic_clusters.size(); ++i) {
      const auto& cluster = data.semantic_clusters[i];
      if (semantic_assigned_[i]) {
        continue;
      }

      const auto iou = computeIoU(data, cluster, track);
      if (iou > best_iou) {
        best_cluster = i;
        best_iou = iou;
      }
    }

    // Associate object if IoU is high enough.
    if (best_cluster) {
      ++num_associated;
      semantic_assigned_[*best_cluster] = true;
      const auto& cluster = data.semantic_clusters[*best_cluster];
      if (track.sequence_number == sequence_number_) {
        // Tracks have already been updated by the dynamic tracking.
        track.observations.back().semantic_cluster_id = cluster.id;
        // TODO(lschmid): Do we want to merge/override the detections? Probably ok to
        // keep the dynamic ones as they will anyways be close hopefully.
      } else {
        // Update the track if it has not been seen this frame.
        updateTrack(data, cluster, track, false);
      }
    }
  }

  MLOG(4) << "Associated " << num_associated << " static detections to dynamic tracks";
}

void MaxIoUTracker::associateSemanticTracks(const FrameData& data, Tracks& tracks) {
  switch (config.semantic_association) {
    case Config::SemanticAssociation::kAssignCluster:
      assignClustersToStaticTrack(data, tracks);
      break;
    case Config::SemanticAssociation::kAssignTrack:
      assignStaticTracksToCluster(data, tracks);
      break;
  }
}

// Greedily associate semantic tracks to highest IoU objects.
void MaxIoUTracker::assignClustersToStaticTrack(const FrameData& data, Tracks& tracks) {
  size_t num_associated = 0;
  for (auto& track : tracks) {
    if (track.is_dynamic) {
      continue;
    }

    if (track.sequence_number == sequence_number_) {
      continue;  // skip track updated this pass
    }

    std::optional<size_t> best_cluster;
    float best_iou = config.min_semantic_iou;
    for (size_t i = 0; i < data.semantic_clusters.size(); ++i) {
      if (semantic_assigned_[i]) {
        continue;
      }

      const auto& cluster = data.semantic_clusters[i];
      const auto ret = semanticsMatch(cluster.semantics, track.semantics, config.min_cosine_sim);
      if (!ret) {
        MLOG(6) << "Rejected object " << cluster.id << " for " << track.id << ": " << toString(ret);
        continue;
      }

      const float iou = computeIoU(data, cluster, track);
      if (iou > best_iou) {
        best_cluster = i;
        best_iou = iou;
      } else {
        MLOG(6) << "Rejected object " << cluster.id << " for " << track.id << ": low IoU (" << iou
                << " < " << best_iou << ")";
      }
    }

    // Associate object if IoU is high enough.
    if (best_cluster) {
      ++num_associated;
      semantic_assigned_[*best_cluster] = true;
      updateTrack(data, data.semantic_clusters[*best_cluster], track, false);
    }
  }

  // Create new tracks for unassociated objects.
  size_t num_new = 0;
  for (size_t i = 0; i < data.semantic_clusters.size(); ++i) {
    if (semantic_assigned_[i]) {
      continue;
    }

    ++num_new;
    addNewTrack(data, data.semantic_clusters[i], tracks, false);
  }

  MLOG(4) << "Associated " << num_associated << " static detections to tracks and creating "
          << num_new << " new tracks";
}

void MaxIoUTracker::assignStaticTracksToCluster(const FrameData& data, Tracks& tracks) {
  size_t num_new = 0;
  size_t num_associated = 0;
  for (size_t i = 0; i < data.semantic_clusters.size(); ++i) {
    if (semantic_assigned_[i]) {
      continue;  // skip if cluster is previously associated
    }

    const auto& cluster = data.semantic_clusters[i];
    bool assigned = false;
    for (auto& track : tracks) {
      if (track.is_dynamic) {
        continue;
      }

      // TODO(nathan) consider multi-association
      if (track.sequence_number == sequence_number_) {
        continue;  // skip track updated this pass
      }

      const auto ret = semanticsMatch(cluster.semantics, track.semantics, config.min_cosine_sim);
      if (!ret) {
        MLOG(6) << "rejected " << track.id << " for object " << cluster.id << ": " << toString(ret);
        continue;
      }

      const float iou = computeIoU(data, cluster, track);
      if (iou < config.min_semantic_iou) {
        MLOG(6) << "rejected " << track.id << " for object " << cluster.id << ": low IoU (" << iou
                << " < " << config.min_semantic_iou << ")";
        continue;
      }

      MLOG(6) << "accepted track " << track.id << " for cluster " << cluster.id << ": IoU=" << iou
              << ", Sim=" << ret.similiarity.value_or(std::numeric_limits<float>::quiet_NaN());

      assigned = true;
      ++num_associated;
      semantic_assigned_[i] = true;
      updateTrack(data, cluster, track, false);
      break;
    }

    if (!assigned) {
      ++num_new;
      addNewTrack(data, cluster, tracks, false);
    }
  }

  MLOG(4) << "Associated " << num_associated << " static detections to tracks and creating "
          << num_new << " new tracks";
}

void MaxIoUTracker::setupTrackMeasurements(FrameData& data) const {
  const auto bbox_type = (config.bbox_type == Config::BBoxType::kRAABB) ? BoundingBox::Type::RAABB
                                                                        : BoundingBox::Type::AABB;
  for (auto& cluster : data.semantic_clusters) {
    const utils::VertexMapAdaptor adaptor(cluster.pixels, data.input.vertex_map);
    cluster.bounding_box = BoundingBox(adaptor, bbox_type);
    if (config.track_by == Config::TrackBy::kVoxels) {
      setupTrackMeasurementVoxels(data, cluster);
    }
  }

  for (auto& cluster : data.dynamic_clusters) {
    const utils::VertexMapAdaptor adaptor(cluster.pixels, data.input.vertex_map);
    cluster.bounding_box = BoundingBox(adaptor, bbox_type);
    if (config.track_by == Config::TrackBy::kVoxels) {
      setupTrackMeasurementVoxels(data, cluster);
    }
  }
}

void MaxIoUTracker::setupTrackMeasurementVoxels(const FrameData& data,
                                                MeasurementCluster& cluster) const {
  // Recompute the voxels based on the used tracking voxel size.
  cluster.voxels.clear();
  cluster.voxels.reserve(cluster.pixels.size());  // Worst case size.
  for (const auto& pixel : cluster.pixels) {
    const auto point = data.input.vertex_map.at<InputData::VertexType>(pixel.v, pixel.u);
    cluster.voxels.insert(grid.toIndex(Point(point[0], point[1], point[2])));
  }
}

Track& MaxIoUTracker::addNewTrack(const FrameData& data,
                                  const MeasurementCluster& observation,
                                  Tracks& tracks,
                                  bool is_dynamic) {
  auto& track = tracks.emplace_back();
  track.is_dynamic = is_dynamic;
  track.id = current_track_id_++;
  track.first_seen = data.input.timestamp_ns;
  // whether or not the new track is dynamic is the same as whether the observation is dynamic
  updateTrack(data, observation, track, is_dynamic);
  return track;
}

void MaxIoUTracker::updateTrack(const FrameData& data,
                                const MeasurementCluster& observation,
                                Track& track,
                                bool is_observation_dynamic) const {
  // Update the cluster estimate used for tracking by retaining last measurement.
  track.last_points.clear();
  track.last_points.reserve(observation.pixels.size());
  for (const auto& pixel : observation.pixels) {
    const auto& point = data.input.vertex_map.at<InputData::VertexType>(pixel.v, pixel.u);
    track.last_points.emplace_back(point[0], point[1], point[2]);
  }

  if (config.track_by == Config::TrackBy::kVoxels) {
    track.last_voxels = observation.voxels;
    track.last_voxel_size = config.voxel_size;
  }

  // Accumulate the geometry of static observations; dynamic objects move between observations.
  if (config.accumulate_voxels && !is_observation_dynamic && !track.is_dynamic) {
    GlobalIndexSet voxels;
    voxels.reserve(track.last_points.size());
    for (const auto& point : track.last_points) {
      voxels.insert(grid.toIndex(point));
    }
    track.accumulateVoxels(voxels, config.voxel_size, data.input.timestamp_ns, config.min_voxel_hits);
  }

  track.last_bounding_box = observation.bounding_box;
  if (!is_observation_dynamic) {
    // NOTE(nathan) we want to avoid overwriting any track semantic information with dynamic
    // observations
    if (!track.semantics) {
      track.semantics = observation.semantics;
    } else {
      track.updateSemantics(observation.semantics);
    }
  }
  // TODO(nathan) add new feature and aggregate semantics

  // Update tracking values.
  // NOTE(lschmid): This needs to happen after the bbox and confidence update as the
  // size of the previous observations is used.
  track.sequence_number = sequence_number_;
  track.last_seen = data.input.timestamp_ns;
  track.observations.emplace_back(data.input.timestamp_ns,
                                  !is_observation_dynamic ? observation.id : -1,
                                  is_observation_dynamic ? observation.id : -1,
                                  data.input.getSensor().name);

  // NOTE(nathan) sensors may have different confidence weights, so we add confidence for each obs
  // Simple existence probability estimate: count number of observations. We multiply by
  // two so that the minimum observations yield 50% confidence.
  track.confidence += 1.0f / (config.min_num_observations * 2.0f);
  track.confidence = std::min(track.confidence, 1.0f);
}

float MaxIoUTracker::computeContainment(const Track& a, const Track& b) {
  const auto count_confirmed = [](const Track& track) {
    size_t num = 0;
    for (const auto& [index, voxel] : track.accumulated_voxels) {
      num += voxel.hits >= track.accumulated_min_hits ? 1 : 0;
    }
    return num;
  };
  const size_t num_a = count_confirmed(a);
  const size_t num_b = count_confirmed(b);
  if (num_a == 0 || num_b == 0) {
    return 0.0f;
  }

  // Fraction of the smaller track's confirmed voxels that are confirmed in the larger one.
  const Track& small = num_a <= num_b ? a : b;
  const Track& large = num_a <= num_b ? b : a;
  size_t intersection = 0;
  for (const auto& [index, voxel] : small.accumulated_voxels) {
    if (voxel.hits < small.accumulated_min_hits) {
      continue;
    }
    const auto it = large.accumulated_voxels.find(index);
    if (it != large.accumulated_voxels.end() && it->second.hits >= large.accumulated_min_hits) {
      ++intersection;
    }
  }
  return static_cast<float>(intersection) / std::min(num_a, num_b);
}

void MaxIoUTracker::mergeTracks(Tracks& tracks) const {
  const auto bounding_box = [](const Track& track) -> const BoundingBox& {
    return track.accumulated_bounding_box.isValid() ? track.accumulated_bounding_box
                                                    : track.last_bounding_box;
  };
  const auto can_merge = [](const Track& track) {
    return !track.is_dynamic && track.is_active && !track.accumulated_voxels.empty();
  };

  std::vector<bool> absorbed(tracks.size(), false);
  size_t num_merged = 0;
  bool merged = true;
  // Merging grows the survivor, which can enable further merges. Bounded to be safe.
  for (size_t iteration = 0; merged && iteration < tracks.size(); ++iteration) {
    merged = false;
    for (size_t i = 0; i < tracks.size(); ++i) {
      if (absorbed[i] || tracks[i].sequence_number != sequence_number_ || !can_merge(tracks[i])) {
        continue;
      }

      const Track& track = tracks[i];
      const BoundingBox& bbox = bounding_box(track);
      if (!bbox.isValid()) {
        continue;
      }

      std::optional<size_t> best;
      float best_containment = config.merge_containment_threshold;
      for (size_t j = 0; j < tracks.size(); ++j) {
        if (j == i || absorbed[j] || !can_merge(tracks[j])) {
          continue;
        }

        const Track& other = tracks[j];
        const BoundingBox& other_bbox = bounding_box(other);
        if (!other_bbox.isValid()) {
          continue;
        }

        if (!semanticsMatch(track.semantics, other.semantics, config.merge_min_cosine_sim)) {
          continue;
        }

        // Distance gate adaptive to the larger of the two tracks (radius = half bbox diagonal).
        const float radius = 0.5f * std::max(bbox.dimensions.norm(), other_bbox.dimensions.norm());
        const float distance = (bbox.world_P_center - other_bbox.world_P_center).norm();
        if (distance > config.merge_distance_factor * radius) {
          continue;
        }

        const float containment = computeContainment(track, other);
        if (containment >= best_containment) {
          best = j;
          best_containment = containment;
        }
      }

      if (!best) {
        continue;
      }

      // The older track keeps its id so ids already reported downstream stay stable.
      const bool keep_i = std::make_pair(tracks[i].first_seen, tracks[i].id) <
                          std::make_pair(tracks[*best].first_seen, tracks[*best].id);
      const size_t survivor = keep_i ? i : *best;
      const size_t victim = keep_i ? *best : i;
      MLOG(3) << "Merging track " << tracks[victim].id << " into track " << tracks[survivor].id
              << " (containment " << best_containment << ")";
      tracks[survivor].absorb(std::move(tracks[victim]));
      absorbed[victim] = true;
      merged = true;
      ++num_merged;
    }
  }

  if (num_merged == 0) {
    return;
  }

  // Erase absorbed tracks, keeping the order of the remaining ones.
  size_t num_kept = 0;
  for (size_t i = 0; i < tracks.size(); ++i) {
    if (!absorbed[i]) {
      if (num_kept != i) {
        tracks[num_kept] = std::move(tracks[i]);
      }
      ++num_kept;
    }
  }
  tracks.erase(tracks.begin() + num_kept, tracks.end());
  MLOG(3) << "Merged " << num_merged << " tracks, " << tracks.size() << " remaining";
}

float MaxIoUTracker::computeIoUVoxels(const FrameData& /* data */,
                                      const MeasurementCluster& cluster,
                                      const Track& track) const {
  // The std::set_intersection does not work without more definitions on LongIndices.
  float intersection = 0.f;
  for (const GlobalIndex& voxel : cluster.voxels) {
    if (track.last_voxels.count(voxel)) {
      intersection += 1.f;
    }
  }
  return intersection / (cluster.voxels.size() + track.last_voxels.size() - intersection);
}

float MaxIoUTracker::computeIoUPixels(const FrameData& data,
                                      const MeasurementCluster& cluster,
                                      const Track& track) const {
  // Project every pixel of cluster 1 into the frame of cluster 2.
  const Transform sensor_T_world = data.input.getSensorPose().inverse();
  const Sensor& sensor = data.input.getSensor();
  std::set<Pixel> reprojected_pixels;
  for (const Point& point : track.last_points) {
    int u, v;
    const auto p_sensor = sensor_T_world * Eigen::Vector3d(point[0], point[1], point[2]);
    if (sensor.projectPointToImagePlane(p_sensor.cast<float>(), u, v)) {
      reprojected_pixels.emplace(u, v);
    }
  }

  // Compute IoU of the reprojected pixels and cluster 2.
  float intersection = 0.f;
  for (const Pixel& pixel : cluster.pixels) {
    if (reprojected_pixels.count(pixel)) {
      intersection += 1.f;
    }
  }
  return intersection / (cluster.pixels.size() + track.last_points.size() - intersection);
}

float MaxIoUTracker::computeIoUBoundingBox(const FrameData& /*data */,
                                           const MeasurementCluster& cluster,
                                           const Track& track) const {
  return track.last_bounding_box.computeIoU(cluster.bounding_box);
}

Point MaxIoUTracker::computeCentroid(const FrameData& data,
                                     const MeasurementCluster& cluster) const {
  switch (config.track_by) {
    case Config::TrackBy::kPixels:
      return centroidFromPixels(data, cluster);
    case Config::TrackBy::kVoxels:
      return centroidFromVoxels(cluster);
    case Config::TrackBy::kBoundingBox:
    default:
      return cluster.bounding_box.world_P_center;
  }
}

Point MaxIoUTracker::centroidFromPixels(const FrameData& data,
                                        const MeasurementCluster& cluster) const {
  Point centroid = Point::Zero();
  for (const auto& pixel : cluster.pixels) {
    const auto point = data.input.vertex_map.at<InputData::VertexType>(pixel.v, pixel.u);
    centroid += Point(point[0], point[1], point[2]);
  }

  return centroid / cluster.pixels.size();
}

Point MaxIoUTracker::centroidFromVoxels(const MeasurementCluster& cluster) const {
  Point centroid = Point::Zero();
  for (const auto& voxel : cluster.voxels) {
    centroid += grid.toPoint(voxel);
  }

  return centroid / cluster.voxels.size();
}

}  // namespace khronos
