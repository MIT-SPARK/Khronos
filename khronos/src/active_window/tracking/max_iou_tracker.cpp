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

#include "khronos/active_window/data/reconstruction_types.h"
#include "khronos/utils/geometry_utils.h"

namespace khronos {

void declare_config(MaxIoUTracker::Config& config) {
  using namespace config;
  name("MaxIoUTracker");
  field(config.verbosity, "verbosity");
  enum_field(config.track_by, "track_by", {"pixels", "voxels", "bounding_box"});
  field(config.min_semantic_iou, "min_semantic_iou");
  field(config.min_cross_iou, "min_cross_iou");
  field(config.max_dynamic_distance, "max_dynamic_distance", "m");
  field(config.temporal_window, "temporal_window", "s");
  field(config.min_num_observations, "min_num_observations", "frames");
  field(config.voxel_size, "voxel_size", "m");

  checkInRange(config.min_cross_iou, 0.f, 1.f, "min_cross_iou");
  checkInRange(config.min_semantic_iou, 0.f, 1.f, "min_semantic_iou");
  check(config.temporal_window, GT, 0.f, "temporal_window");
  check(config.voxel_size, GT, 0.f, "voxel_size");
}

MaxIoUTracker::MaxIoUTracker(const Config& config)
    : config(config::checkValid(config)), grid_(config.voxel_size) {
  // Define the right functionality for the track_by mode.
  setup();
}

void MaxIoUTracker::setup() {
  // Define the right functionality for the track_by mode.
  switch (config.track_by) {
    case Config::TrackBy::kPixels:
      // Pixels are always given in a cluster.
      setupTrackMeasurement = [](const FrameData&, MeasurementCluster&) {};
      computeIoU = std::bind(&MaxIoUTracker::computeIoUPixels,
                             this,
                             std::placeholders::_1,
                             std::placeholders::_2,
                             std::placeholders::_3);
      break;
    case Config::TrackBy::kVoxels: {
      setupTrackMeasurement = std::bind(&MaxIoUTracker::setupTrackMeasurementVoxels,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2);
      computeIoU = std::bind(&MaxIoUTracker::computeIoUVoxels,
                             this,
                             std::placeholders::_1,
                             std::placeholders::_2,
                             std::placeholders::_3);
      break;
    }
    case Config::TrackBy::kBouningBox: {
      setupTrackMeasurement = [](const FrameData& data, MeasurementCluster& cluster) {
        cluster.bounding_box =
            BoundingBox(utils::VertexMapAdaptor(cluster.pixels, data.input.vertex_map));
      };
      computeIoU = std::bind(&MaxIoUTracker::computeIoUBoundingBox,
                             this,
                             std::placeholders::_1,
                             std::placeholders::_2,
                             std::placeholders::_3);
    }; break;
  }
}

void MaxIoUTracker::processInput(FrameData& data) {
  processing_stamp_ = data.input.timestamp_ns;
  Timer timer("tracking/all", processing_stamp_);

  // Compute the entities the objects are going to be tracked by (pixels, voxels, or
  // bounding box).
  setupTrackMeasurements(data);

  // Associate current objects to existing tracks and create new tracks for
  // unassociated objects.
  // TODO(lschmid): Handle objects splitting or merging explicitly at some point.
  associateTracks(data);

  // Update which tracks are still active. Tracks labeled inactive will be removed by
  // the active window.
  updateTrackingDuration();
}

void MaxIoUTracker::associateTracks(const FrameData& data) {
  Timer timer("tracking/associate", processing_stamp_);

  // Associate dynamic tracks first, allocating new dynamic tracks if no match is found.
  associateDynamicTracks(data);

  // Then associate semantic tracks to all other tracks
  associateSemanticTracks(data);
}

void MaxIoUTracker::associateDynamicTracks(const FrameData& data) {
  // Assign dynamic tracks to the closest dynamic clusters that moved less than the
  // allowed maximum.
  std::unordered_set<int> associated_objects;
  for (Track& track : tracks_) {
    if (!track.is_dynamic) {
      continue;
    }
    float best_distance = config.max_dynamic_distance;
    const DynamicCluster* best_cluster = nullptr;
    Point best_centroid;

    for (const DynamicCluster& cluster : data.dynamic_clusters) {
      if (associated_objects.find(cluster.id) != associated_objects.end()) {
        continue;
      }

      // Compute the centroid distances.
      const Point centroid = computeCentroid(data, cluster);
      const float distance = (centroid - track.last_centroid).norm();
      if (distance < best_distance) {
        best_cluster = &cluster;
        best_distance = distance;
        best_centroid = centroid;
      }
    }

    // Associate object if IoU is high enough.
    if (best_cluster != nullptr) {
      associated_objects.insert(best_cluster->id);
      updateTrack(data, *best_cluster, track, -1, best_cluster->id);
      track.last_centroid = best_centroid;
    }
  }

  // Create new tracks for unassociated objects.
  for (const DynamicCluster& cluster : data.dynamic_clusters) {
    if (associated_objects.find(cluster.id) != associated_objects.end()) {
      continue;
    }
    Track& track = tracks_.emplace_back();
    track.is_dynamic = true;
    track.id = current_track_id_++;
    track.first_seen = processing_stamp_;
    updateTrack(data, cluster, track, -1, cluster.id);
    track.last_centroid = computeCentroid(data, cluster);
  }
}

void MaxIoUTracker::associateSemanticTracks(const FrameData& data) {
  // First assign all semantic tracks to dynamic tracks where possible to avoid
  // allocating many semantic tracks for moving objects.
  std::unordered_set<int> associated_objects;
  for (Track& track : tracks_) {
    if (!track.is_dynamic) {
      continue;
    }
    float best_iou = config.min_cross_iou;
    const SemanticCluster* best_cluster = nullptr;
    for (const SemanticCluster& cluster : data.semantic_clusters) {
      if (associated_objects.find(cluster.id) != associated_objects.end()) {
        continue;
      }
      const float iou = computeIoU(data, cluster, track);
      if (iou > best_iou) {
        best_cluster = &cluster;
        best_iou = iou;
      }
    }

    // Associate object if IoU is high enough.
    if (best_cluster != nullptr) {
      associated_objects.insert(best_cluster->id);
      if (track.last_seen < processing_stamp_) {
        // Update the track if it has not been seen this frame.
        updateTrack(data, *best_cluster, track, best_cluster->id, -1);
      } else {
        // Tracks have already been updated by the dynamic tracking.
        track.observations.back().semantic_cluster_id = best_cluster->id;
        // TODO(lschmid): Do we want to merge/override the detections? Probably ok to
        // keep the dynamic ones as they will anyways be close hopefully.
      }
      track.semantic_id = best_cluster->semantic_id;
    }
  }

  // Greedily associate semantic tracks to highest IoU objects.
  for (Track& track : tracks_) {
    if (track.is_dynamic) {
      continue;
    }
    float best_iou = config.min_semantic_iou;
    const SemanticCluster* best_cluster = nullptr;
    for (const SemanticCluster& cluster : data.semantic_clusters) {
      if (cluster.semantic_id != track.semantic_id) {
        continue;
      }
      if (associated_objects.find(cluster.id) != associated_objects.end()) {
        continue;
      }
      const float iou = computeIoU(data, cluster, track);
      if (iou > best_iou) {
        best_cluster = &cluster;
        best_iou = iou;
      }
    }

    // Associate object if IoU is high enough.
    if (best_cluster != nullptr) {
      associated_objects.insert(best_cluster->id);
      updateTrack(data, *best_cluster, track, best_cluster->id, -1);
    }
  }

  // Create new tracks for unassociated objects.
  for (const SemanticCluster& cluster : data.semantic_clusters) {
    if (associated_objects.find(cluster.id) != associated_objects.end()) {
      continue;
    }
    Track& track = tracks_.emplace_back();
    track.semantic_id = cluster.semantic_id;
    track.id = current_track_id_++;
    track.first_seen = processing_stamp_;
    updateTrack(data, cluster, track, cluster.semantic_id, cluster.id);
  }
}

void MaxIoUTracker::setupTrackMeasurements(FrameData& data) const {
  for (MeasurementCluster& cluster : data.semantic_clusters) {
    setupTrackMeasurement(data, cluster);
    // NOTE(lschmid): Compute the bounding boxes for all clusters for visualization and extent
    // computation in the future.
    cluster.bounding_box =
        BoundingBox(utils::VertexMapAdaptor(cluster.pixels, data.input.vertex_map));
  }
  for (MeasurementCluster& cluster : data.dynamic_clusters) {
    setupTrackMeasurement(data, cluster);
    cluster.bounding_box =
        BoundingBox(utils::VertexMapAdaptor(cluster.pixels, data.input.vertex_map));
  }
}

void MaxIoUTracker::setupTrackMeasurementVoxels(const FrameData& data,
                                                MeasurementCluster& cluster) const {
  // Recompute the voxels based on the used tracking voxel size.
  cluster.voxels.clear();
  cluster.voxels.reserve(cluster.pixels.size());  // Worst case size.
  for (const Pixel& pixel : cluster.pixels) {
    const auto& point = data.input.vertex_map.at<InputData::VertexType>(pixel.v, pixel.u);
    cluster.voxels.insert(grid_.toIndex(Point(point[0], point[1], point[2])));
  }
}

void MaxIoUTracker::updateTrack(const FrameData& data,
                                const MeasurementCluster& observation,
                                Track& track,
                                int semantic_cluster_id,
                                int dynamic_cluster_id) const {
  // Update the cluster estimate used for tracking by simply retaining the last
  // last_measurement.
  if (config.track_by == Config::TrackBy::kPixels) {
    track.last_points.clear();
    track.last_points.reserve(observation.pixels.size());
    for (const Pixel& pixel : observation.pixels) {
      const auto& point = data.input.vertex_map.at<InputData::VertexType>(pixel.v, pixel.u);
      track.last_points.emplace_back(point[0], point[1], point[2]);
    }
  } else if (config.track_by == Config::TrackBy::kVoxels) {
    track.last_voxels = observation.voxels;
    track.last_voxel_size = config.voxel_size;
  }
  track.last_bounding_box = observation.bounding_box;

  // Simple existence probability estimate: count number of observations. We multiply by
  // two so that the minimum observations yield 50% confidence.
  track.confidence = std::min(
      static_cast<float>(track.observations.size()) / (config.min_num_observations * 2), 1.f);

  // Update tracking values.
  // NOTE(lschmid): This needs to happen after the bbox and confidence update as the
  // size of the previous observations is used.
  track.last_seen = processing_stamp_;
  track.observations.emplace_back(processing_stamp_, semantic_cluster_id, dynamic_cluster_id);
}

void MaxIoUTracker::updateTrackingDuration() {
  // Label tracks that exit the temporal window as inactive.
  const TimeStamp min_time = processing_stamp_ - fromSeconds(config.temporal_window);
  for (Track& track : tracks_) {
    track.is_active = track.last_seen >= min_time;
  }
}

Point MaxIoUTracker::computeCentroid(const FrameData& data,
                                     const MeasurementCluster& cluster) const {
  Point centroid = Point::Zero();
  switch (config.track_by) {
    case Config::TrackBy::kPixels: {
      for (const Pixel& pixel : cluster.pixels) {
        const auto& point = data.input.vertex_map.at<InputData::VertexType>(pixel.v, pixel.u);
        centroid += Point(point[0], point[1], point[2]);
      }
      return centroid / cluster.pixels.size();
    }

    case Config::TrackBy::kVoxels: {
      for (const GlobalIndex& voxel : cluster.voxels) {
        centroid += grid_.toPoint(voxel);
      }
      return centroid / cluster.voxels.size();
    }

    case Config::TrackBy::kBouningBox:
      return cluster.bounding_box.world_P_center;
  }
  return centroid;
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
  const Transform sensor_T_world = data.input.getSensorPose();
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

}  // namespace khronos
