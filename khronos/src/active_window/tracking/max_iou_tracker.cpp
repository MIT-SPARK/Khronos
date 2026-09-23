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

  checkInRange(config.min_cross_iou, 0.0f, 1.0f, "min_cross_iou");
  checkInRange(config.min_semantic_iou, 0.0f, 1.0f, "min_semantic_iou");
  checkInRange(config.min_cosine_sim, -1.0f, 1.0f, "min_cosine_sim");
  check(config.voxel_size, GT, 0.f, "voxel_size");
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

  MLOG(3) << "Previous tracks: " << prev_num_tracks << " Current tracks: " << tracks.size();
  ++sequence_number_;
}

void MaxIoUTracker::preassociateSemanticTracks(const FrameData& data, Tracks& tracks) {
  std::unordered_map<int, size_t> id_to_cluster;
  for (size_t i = 0; i < data.semantic_clusters.size(); ++i) {
    id_to_cluster[data.semantic_clusters[i].id] = i;
  }

  for (auto& track : tracks) {
    if (track.observations.empty()) {
      continue;
    }

    const auto& last_obs = track.observations.back();
    const int last_id =
        track.is_dynamic ? last_obs.dynamic_cluster_id : last_obs.semantic_cluster_id;
    if (last_id < 0) {
      continue;
    }

    const auto iter = id_to_cluster.find(last_id);
    if (iter == id_to_cluster.end()) {
      continue;  // track has no matching cluster
    }

    const auto& [id, cluster_idx] = *iter;
    semantic_assigned_[cluster_idx] = true;
    updateTrack(data, data.semantic_clusters[cluster_idx], track, track.is_dynamic);
  }
}

void MaxIoUTracker::associateTracks(const FrameData& data, Tracks& tracks) {
  Timer timer("tracking/associate", data.input.timestamp_ns);

  // Associate dynamic clusters first, allocating new dynamic tracks if no match is found.
  associateDynamicTracks(data, tracks);

  // Optionally assign clusters to tracks by matching ID (for external tracking)
  if (config.preassociate_by_id) {
    preassociateSemanticTracks(data, tracks);
  }

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

void MaxIoUTracker::crossAssociateTracks(const FrameData& data, Tracks& tracks) {
  MLOG(4) << "Cross associating " << data.semantic_clusters.size() << " semantic clusters";

  // First assign all semantic tracks to dynamic tracks where possible to avoid
  // allocating many semantic tracks for moving objects.
  size_t num_associated = 0;
  for (auto& track : tracks) {
    if (!track.is_dynamic) {
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
