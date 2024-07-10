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

#include "khronos/active_window/object_extraction/mesh_object_extractor.h"

#include <sstream>

#include "khronos/active_window/data/reconstruction_types.h"
#include "khronos/utils/geometry_utils.h"

namespace khronos {

void declare_config(MeshObjectExtractor::Config& config) {
  using namespace config;
  name("MeshObjectExtractor::Config");
  field(config.verbosity, "verbosity");
  field(config.min_object_allocation_confidence, "min_object_allocation_confidence");
  field(config.min_object_volume, "min_object_volume");
  field(config.max_object_volume, "max_object_volume");
  field(config.only_extract_reconstructed_objects, "only_extract_reconstructed_objects");
  field(config.min_dynamic_displacement, "min_dynamic_displacement");
  field(config.min_object_reconstruction_confidence, "min_object_reconstruction_confidence");
  field(config.min_object_reconstruction_observations, "min_object_reconstruction_observations");
  field(config.object_reconstruction_resolution, "object_reconstruction_resolution");
  field(config.visualize_classification, "visualize_classification");
  field(config.projective_integrator, "projective_integrator");
  field(config.mesh_integrator, "mesh_integrator");

  checkInRange(
      config.min_object_allocation_confidence, 0.0f, 1.0f, "min_object_allocation_confidence");
  checkInRange(config.min_object_reconstruction_confidence,
               0.0f,
               1.0f,
               "min_object_reconstruction_confidence");
  check(config.min_object_volume, GE, 0, "min_object_volume");
  check(config.max_object_volume, GE, config.min_object_volume, "max_object_volume");
  check(config.min_dynamic_displacement, GE, 0, "min_dynamic_displacement");
}

MeshObjectExtractor::MeshObjectExtractor(const Config& config)
    : config(config),
      integrator_(config.projective_integrator),
      mesh_integrator_(config.mesh_integrator) {}

KhronosObjectAttributes::Ptr MeshObjectExtractor::extractObject(const Track& track,
                                                                const FrameDataBuffer& frame_data) {
  // Check the track is valid for extraction.
  if (!trackIsValid(track)) {
    return nullptr;
  }

  // Extract reconstructions of the object.
  auto object = track.is_dynamic ? extractDynamicObject(track, frame_data)
                                 : extractStaticObject(track, frame_data);
  if (!object) {
    return nullptr;
  }

  // General information.
  object->semantic_label = track.semantic_id;
  object->first_observed_ns = {track.first_seen};
  object->last_observed_ns = {track.last_seen};
  return object;
}

KhronosObjectAttributes::Ptr MeshObjectExtractor::extractDynamicObject(
    const Track& track,
    const FrameDataBuffer& frame_data) const {
  auto object = std::make_unique<KhronosObjectAttributes>();
  const bool store_visualization_details =
      hydra::GlobalInfo::instance().getConfig().store_visualization_details;
  Point bbox_extent = Point::Zero();  // Accumulated size to average.
  float max_displacement = 0;
  for (const Observation& observation : track.observations) {
    if (observation.dynamic_cluster_id == -1) {
      continue;
    }

    // Get the data for this observation.
    const auto frame = frame_data.getData(observation.stamp);
    if (!frame) {
      continue;
    }
    const auto it2 = std::find_if(frame->dynamic_clusters.begin(),
                                  frame->dynamic_clusters.end(),
                                  [&observation](const DynamicCluster& cluster) {
                                    return cluster.id == observation.dynamic_cluster_id;
                                  });
    if (it2 == frame->dynamic_clusters.end()) {
      continue;
    }
    const DynamicCluster& cluster = *it2;
    const auto& vertex_map = frame->input.vertex_map;

    // Compute the dynamic points, centroid, and mean bounding box.
    Points points;
    points.reserve(cluster.pixels.size());
    for (const Pixel& pixel : cluster.pixels) {
      if (pixel.u < 0 || pixel.u >= vertex_map.cols || pixel.v < 0 || pixel.v >= vertex_map.rows) {
        continue;
      }
      const auto& vertex = vertex_map.at<InputData::VertexType>(pixel.v, pixel.u);
      points.emplace_back(vertex[0], vertex[1], vertex[2]);
    }
    auto current_pos = utils::computeCentroid(points);
    object->trajectory_positions.emplace_back(current_pos);
    object->trajectory_timestamps.emplace_back(observation.stamp);
    bbox_extent += BoundingBox(points).dimensions;
    max_displacement =
        std::max(max_displacement, (current_pos - object->trajectory_positions.front()).norm());

    if (store_visualization_details) {
      object->dynamic_object_points.emplace_back(std::move(points));
    }
  }

  // Aggregate all results.
  if (object->trajectory_positions.empty()) {
    CLOG(5) << "[MeshObjectExtractor] Dropping dynamic " << getTrackName(track)
            << ": no obesrvations.";
    return nullptr;
  }
  if (max_displacement < config.min_dynamic_displacement) {
    CLOG(5) << "[MeshObjectExtractor] Dropping dynamic " << getTrackName(track)
            << ": low displacement (" << max_displacement << " < "
            << config.min_dynamic_displacement << ").";
    return nullptr;
  }
  object->bounding_box = BoundingBox(bbox_extent / object->trajectory_positions.size(),
                                     object->trajectory_positions.front());
  return object;
}

KhronosObjectAttributes::Ptr MeshObjectExtractor::extractStaticObject(
    const Track& track,
    const FrameDataBuffer& frame_data) const {
  if (config.object_reconstruction_resolution == 0.f) {
    return nullptr;
  }

  const auto frames = collectSemanticFrames(track, frame_data);
  if (frames.empty()) {
    CLOG(5) << "[MeshObjectExtractor] Dropping " << getTrackName(track)
            << ": no semantic observations.";
    return nullptr;
  }

  // Compute the maximal extent and resolution.
  BoundingBox extent = computeExtent(frames);
  if (extent.volume() < config.min_object_volume) {
    CLOG(5) << "[MeshObjectExtractor] Dropping " << getTrackName(track)
            << ": small maximal volume (" << extent.volume() << " < " << config.min_object_volume
            << ").";
    return nullptr;
  }

  // Setup a volumetric map to reconstruct this object.
  VolumetricMap::Config map_config;
  if (config.object_reconstruction_resolution < 0.f) {
    map_config.voxel_size = extent.dimensions.maxCoeff() * -config.object_reconstruction_resolution;
  } else {
    map_config.voxel_size = config.object_reconstruction_resolution;
  }
  map_config.voxels_per_side = 8;
  map_config.truncation_distance = map_config.voxel_size * 2;
  if (!config::isValid(map_config)) {
    return nullptr;
  }
  VolumetricMap map(map_config, true, false);

  // Allocate all blocks.
  TsdfLayer& tsdf_layer = map.getTsdfLayer();
  auto& confidence_layer = *map.getSemanticLayer();
  const auto min_block_index = tsdf_layer.getBlockIndex(extent.world_P_center - extent.dimensions);
  const auto max_block_index = tsdf_layer.getBlockIndex(extent.world_P_center + extent.dimensions);
  for (int x = min_block_index.x(); x <= max_block_index.x(); ++x) {
    for (int y = min_block_index.y(); y <= max_block_index.y(); ++y) {
      for (int z = min_block_index.z(); z <= max_block_index.z(); ++z) {
        map.allocateBlock(BlockIndex(x, y, z));
      }
    }
  }

  // Perform 3D reconstruction using projective updates over all frames.
  for (const auto& data_id_pair : frames) {
    integrator_.updateObjectMap(*data_id_pair.first, map, data_id_pair.second);
  }

  // Erase low_confidence voxels to extract the object of interest.
  for (TsdfBlock& tsdf_block : tsdf_layer) {
    const auto& confidence_block = confidence_layer.getBlock(tsdf_block.index);
    for (size_t i = 0; i < tsdf_block.numVoxels(); ++i) {
      TsdfVoxel& tsdf_voxel = tsdf_block.getVoxel(i);
      if (tsdf_voxel.distance > 0.f) {
        continue;
      }
      const hydra::SemanticVoxel& confidence_voxel = confidence_block.getVoxel(i);
      const float confidence = computeConfidence(tsdf_voxel, confidence_voxel);
      if (config.visualize_classification) {
        // Do not prune away low confidence voxels but instead visualize them.
        // NOTE(lschmid): High jack confidence -1 to idnicate not enough observations.
        tsdf_voxel.color = confidence >= 0 ? Color::quality(confidence) : Color::gray();
      } else if (confidence < config.min_object_reconstruction_confidence) {
        tsdf_voxel.distance = map_config.truncation_distance;
      }
    }
  }

  // Extract the mesh that belongs to this object.
  mesh_integrator_.generateMesh(map, true, false);
  auto object = std::make_unique<KhronosObjectAttributes>();
  object->mesh = utils::combineMeshLayer(map.getMeshLayer());

  if (object->mesh.points.empty() && config.only_extract_reconstructed_objects) {
    CLOG(5) << "[MeshObjectExtractor] Dropping " << getTrackName(track)
            << ": no reconstructed mesh.";
    return nullptr;
  }

  // Compute the bounding box.
  if (object->mesh.points.empty()) {
    object->bounding_box = extent;
  } else {
    object->bounding_box = BoundingBox(object->mesh.points);
  }
  if (object->bounding_box.volume() > config.max_object_volume) {
    CLOG(5) << "[MeshObjectExtractor] Dropping " << getTrackName(track) << ": large volume ("
            << object->bounding_box.volume() << " > " << config.max_object_volume << ").";
    return nullptr;
  }
  if (object->bounding_box.volume() < config.min_object_volume) {
    CLOG(5) << "[MeshObjectExtractor] Dropping " << getTrackName(track) << ": small volume ("
            << object->bounding_box.volume() << " < " << config.min_object_volume << ").";
    return nullptr;
  }

  CLOG(5) << "[MeshObjectExtractor] Extracted " << getTrackName(track) << " with volume "
          << object->bounding_box.volume() << ", confidence " << track.confidence << ", mesh size "
          << object->mesh.points.size() << ".";

  // Move the object mesh to bbox frame.
  const Point offset = object->bounding_box.world_P_center;
  for (Point& point : object->mesh.points) {
    point -= offset;
  }
  return object;
}

std::vector<std::pair<FrameData::Ptr, int>> MeshObjectExtractor::collectSemanticFrames(
    const Track& track,
    const FrameDataBuffer& frame_data) {
  std::vector<std::pair<FrameData::Ptr, int>> result;
  for (const Observation& observation : track.observations) {
    if (observation.semantic_cluster_id == -1) {
      continue;
    }
    const auto frame = frame_data.getData(observation.stamp);
    if (!frame) {
      continue;
    }
    result.emplace_back(frame, observation.semantic_cluster_id);
  }
  return result;
}

BoundingBox MeshObjectExtractor::computeExtent(
    const std::vector<std::pair<FrameData::Ptr, int>>& frames) const {
  BoundingBox extent;
  for (const auto& [frame, id] : frames) {
    const auto it = std::find_if(frame->semantic_clusters.begin(),
                                 frame->semantic_clusters.end(),
                                 [id](const SemanticCluster& cluster) { return cluster.id == id; });
    if (it == frame->semantic_clusters.end()) {
      continue;
    }
    extent.merge(it->bounding_box);
  }
  return extent;
}

float MeshObjectExtractor::computeConfidence(const TsdfVoxel& /* tsdf_voxel */,
                                             const hydra::SemanticVoxel& confidence_voxel) const {
  // NOTE(lschmid): This exploits the semantic_likelihoods to store the counts of correct and
  // incorrect observations.
  if (confidence_voxel.empty) {
    return 0.0f;
  }

  const float total_observations =
      confidence_voxel.semantic_likelihoods(0) + confidence_voxel.semantic_likelihoods(1);

  if (total_observations < config.min_object_reconstruction_observations) {
    return -1.f;
  }

  return confidence_voxel.semantic_likelihoods(0) / total_observations;
}

bool MeshObjectExtractor::trackIsValid(const Track& track) const {
  // Check the track is valid for extraction.
  if (track.confidence <= config.min_object_allocation_confidence) {
    CLOG(5) << "[MeshObjectExtractor] Dropping " << getTrackName(track) << ": low confidence ("
            << track.confidence << " < " << config.min_object_allocation_confidence << ").";
    return false;
  }
  return true;
}

std::string MeshObjectExtractor::getTrackName(const Track& track) {
  std::stringstream ss;
  ss << "track " << track.id << " ("
     << hydra::GlobalInfo::instance().getLabelToNameMap().at(track.semantic_id) << ")";
  return ss.str();
}

}  // namespace khronos
