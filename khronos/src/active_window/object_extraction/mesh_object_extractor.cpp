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

#include <algorithm>
#include <filesystem>
#include <sstream>

#include <config_utilities/types/path.h>
#include <glog/logging.h>
#include <hydra/frontend/keyframe_writer.h>
#include <hydra/input/camera.h>
#include <hydra/utils/image_folder.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <spark_dsg/colormaps.h>

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
  field(config.min_reconstruction_resolution, "min_reconstruction_resolution");
  field(config.visualize_classification, "visualize_classification");
  field<Path>(config.object_image_output_path, "object_image_output_path");
  enum_field(config.image_selection_criteria,
             "image_selection_criteria",
             std::vector<std::string>{"largest_bbox", "segment_size", "all"});
  field(config.projective_integrator, "projective_integrator");
  field(config.mesh_integrator, "mesh_integrator");
  enum_field(config.bbox_type, "bbox_type", std::vector<std::string>{"aabb", "raabb"});

  checkInRange(
      config.min_object_allocation_confidence, 0.0f, 1.0f, "min_object_allocation_confidence");
  checkInRange(config.min_object_reconstruction_confidence,
               0.0f,
               1.0f,
               "min_object_reconstruction_confidence");
  check(config.min_object_volume, GE, 0, "min_object_volume");
  check(config.max_object_volume, GE, config.min_object_volume, "max_object_volume");
  check(config.min_dynamic_displacement, GE, 0, "min_dynamic_displacement");
  check(config.min_reconstruction_resolution, GE, 0.0f, "min_reconstruction_resolution");
}

MeshObjectExtractor::MeshObjectExtractor(const Config& config)
    : config(config), mesh_integrator_(config.mesh_integrator) {}

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
  if (track.semantics) {
    object->semantic_label = track.semantics->category_id;
    object->semantic_feature = track.semantics->feature;
  }
  object->first_observed_ns = {track.first_seen};
  object->last_observed_ns = {track.last_seen};
  object->position = object->bounding_box.world_P_center.cast<double>();

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
    const auto frame = frame_data.getData(observation.stamp, observation.sensor);
    if (!frame) {
      continue;
    }
    const auto it2 = std::find_if(frame->dynamic_clusters.begin(),
                                  frame->dynamic_clusters.end(),
                                  [&observation](const auto& cluster) {
                                    return cluster.id == observation.dynamic_cluster_id;
                                  });
    if (it2 == frame->dynamic_clusters.end()) {
      continue;
    }
    const auto& cluster = *it2;
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

  CLOG(5) << "[MeshObjectExtractor] Using extents of " << extent << " for " << getTrackName(track)
          << " during extraction";

  // Setup a volumetric map to reconstruct this object.
  VolumetricMap::Config map_config;
  if (config.object_reconstruction_resolution < 0.f) {
    map_config.voxel_size = extent.dimensions.maxCoeff() * -config.object_reconstruction_resolution;
    map_config.voxel_size = std::max(map_config.voxel_size, config.min_reconstruction_resolution);
  } else {
    map_config.voxel_size = config.object_reconstruction_resolution;
  }
  map_config.voxels_per_side = 8;
  map_config.truncation_distance = map_config.voxel_size * 2;
  map_config.with_semantics = true;
  map_config.with_tracking = false;
  if (!config::isValid(map_config)) {
    return nullptr;
  }
  VolumetricMap map(map_config);

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

  // Print info if desired.
  const Eigen::IOFormat fmt(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");
  CLOG(5) << "[MeshObjectExtractor] Allocated TSDF layer for " << getTrackName(track) << " with "
          << tsdf_layer.numBlocks() << " blocks (min_index=" << min_block_index.format(fmt)
          << ", max_index=" << max_block_index.format(fmt) << ") with voxel size "
          << map_config.voxel_size << " [m]";

  // Perform 3D reconstruction using projective updates over all frames.
  hydra::SensorMap<ObjectIntegrator> integrator_map(config.projective_integrator);
  for (const auto& [frame_data, segment_id] : frames) {
    const auto sensor_name = frame_data->input.getSensor().name;
    const auto integrator = integrator_map.get(sensor_name);
    if (!integrator) {
      CLOG(2) << "Skipping Unknown sensor '" << sensor_name << "'";
      continue;
    }

    integrator->setFrameData(frame_data.get(), segment_id);
    integrator->updateMap(frame_data->input, map, false);
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
        tsdf_voxel.color =
            confidence >= 0 ? spark_dsg::colormaps::quality(confidence) : Color::gray();
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
    const BoundingBox::Type bbox_type = config.bbox_type == Config::BBoxType::kRAABB
                                            ? BoundingBox::Type::RAABB
                                            : BoundingBox::Type::AABB;
    object->bounding_box = BoundingBox(object->mesh.points, bbox_type);
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

  if (!config.object_image_output_path.empty()) {
    // Extraction runs on a worker thread, so failures must not escape.
    try {
      saveObjectImages(track, frames, *object);
    } catch (const std::exception& e) {
      LOG(WARNING) << "[MeshObjectExtractor] Failed to save images of " << getTrackName(track)
                   << ": " << e.what();
    }
  }

  return object;
}

void MeshObjectExtractor::saveObjectImages(
    const Track& track,
    const std::vector<std::pair<FrameData::Ptr, int>>& frames,
    KhronosObjectAttributes& object) const {
  // Observations of this track.
  std::vector<std::pair<const FrameData*, const MeasurementCluster*>> observations;
  for (const auto& [frame, id] : frames) {
    const auto it = std::find_if(frame->semantic_clusters.begin(),
                                 frame->semantic_clusters.end(),
                                 [id = id](const auto& cluster) { return cluster.id == id; });
    if (it != frame->semantic_clusters.end()) {
      observations.emplace_back(frame.get(), &*it);
    }
  }

  if (observations.empty()) {
    return;
  }

  // Select the observations to save.
  using Selection = Config::ImageSelection;
  if (config.image_selection_criteria != Selection::kAll) {
    const auto value = [this](const MeasurementCluster& cluster) {
      return config.image_selection_criteria == Selection::kSegmentSize
                 ? static_cast<float>(cluster.pixels.size())
                 : cluster.bounding_box.volume();
    };
    const auto best = std::max_element(
        observations.begin(), observations.end(), [&value](const auto& lhs, const auto& rhs) {
          return value(*lhs.second) < value(*rhs.second);
        });
    observations = {*best};
  }

  // The frontend writes to a temporary folder per track that the backend moves to a stable
  // folder per object node once the object is in the scene graph.
  const auto& root = config.object_image_output_path;
  const auto object_dir = root / hydra::utils::kTempImageFolder / ("O_" + std::to_string(track.id));
  // The calibration is shared by all objects and lives in the image root, one level above the
  // final object folders.
  const hydra::KeyframeWriter writer(
      object_dir,
      hydra::utils::kObjectImagePrefix,
      (std::filesystem::path("..") / hydra::utils::kCameraCalibFile).string());

  size_t num_written = 0;
  for (const auto& [frame, cluster] : observations) {
    const auto& input = frame->input;
    if (!calib_written_) {
      const auto camera = dynamic_cast<const hydra::Camera*>(&input.getSensor());
      if (camera && hydra::writeCameraCalib(root, hydra::CameraCalib::fromCamera(*camera))) {
        calib_written_ = true;
      }
    }

    const auto& reference = input.color_image.empty() ? input.depth_image : input.color_image;
    if (reference.empty()) {
      continue;
    }

    // Per-frame metadata in addition to the shared camera calibration.
    nlohmann::json meta{{"depth_encoding", hydra::kKeyframeDepthEncoding},
                        {"world_T_body", hydra::isometryToJson(input.world_T_body)}};

    // Axis-aligned extent of the 3D bounding box (works for AABB and RAABB).
    const auto corners = cluster->bounding_box.corners();
    Eigen::Vector3f min_c = corners[0];
    Eigen::Vector3f max_c = corners[0];
    for (const auto& c : corners) {
      min_c = min_c.cwiseMin(c);
      max_c = max_c.cwiseMax(c);
    }

    meta["bbox_min"] = {min_c.x(), min_c.y(), min_c.z()};
    meta["bbox_max"] = {max_c.x(), max_c.y(), max_c.z()};

    // Segmentation mask of the object and its 2D bounding box (inclusive pixel bounds).
    cv::Mat mask = cv::Mat::zeros(reference.size(), CV_8UC1);
    for (const auto& pixel : cluster->pixels) {
      if (pixel.u >= 0 && pixel.u < mask.cols && pixel.v >= 0 && pixel.v < mask.rows) {
        mask.at<uint8_t>(pixel.v, pixel.u) = 255;
      }
    }

    const auto timestamp_ns = input.timestamp_ns;
    const cv::Rect bbox_2d = cv::boundingRect(mask);
    if (bbox_2d.area() > 0) {
      meta["bbox_2d"] = {{"min_x", bbox_2d.x},
                         {"min_y", bbox_2d.y},
                         {"max_x", bbox_2d.x + bbox_2d.width - 1},
                         {"max_y", bbox_2d.y + bbox_2d.height - 1}};
      const auto mask_path = writer.prefix(timestamp_ns).string() + "_mask.png";
      if (hydra::writeImage(mask_path, mask)) {
        meta["mask_file"] = std::filesystem::path(mask_path).filename().string();
      }
    }

    const auto color =
        input.color_image.empty() ? cv::Mat() : hydra::colorToKeyframe(input.color_image);
    if (writer.write(timestamp_ns, color, hydra::depthToKeyframe(input.depth_image), meta)) {
      ++num_written;
    }
  }

  if (num_written > 0) {
    object.image_folder = hydra::utils::relativeImageFolder(root, object_dir);
  }
}

std::vector<std::pair<FrameData::Ptr, int>> MeshObjectExtractor::collectSemanticFrames(
    const Track& track,
    const FrameDataBuffer& frame_data) {
  std::vector<std::pair<FrameData::Ptr, int>> result;
  for (const Observation& observation : track.observations) {
    if (observation.semantic_cluster_id == -1) {
      // TODO(nathan) this might need to be relaxed
      continue;
    }
    const auto frame = frame_data.getData(observation.stamp, observation.sensor);
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
    // explicit variable required for lambda capture by clang for some reason
    const auto object_id = id;
    const auto it =
        std::find_if(frame->semantic_clusters.begin(),
                     frame->semantic_clusters.end(),
                     [object_id](const auto& cluster) { return cluster.id == object_id; });
    if (it == frame->semantic_clusters.end()) {
      continue;
    }
    extent.merge(it->bounding_box);
  }
  return extent;
}

float MeshObjectExtractor::computeConfidence(const TsdfVoxel& /* tsdf_voxel */,
                                             const hydra::SemanticVoxel& confidence_voxel) const {
  if (confidence_voxel.empty) {
    return 0.0f;
  }

  const float total_observations =
      confidence_voxel.semantic_likelihoods(0) + confidence_voxel.semantic_likelihoods(1);

  if (total_observations < config.min_object_reconstruction_observations) {
    return -1.f;
  }

  return confidence_voxel.semantic_likelihoods(1) / total_observations;
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
  std::string label = "unknown";
  if (track.semantics) {
    const auto& label_names = hydra::GlobalInfo::instance().labelspace().label_names;
    auto iter = label_names.find(track.semantics->category_id);
    if (iter != label_names.end()) {
      label = iter->second;
    }
  }

  std::stringstream ss;
  ss << "track " << track.id << " (" << label << ")";
  return ss.str();
}

}  // namespace khronos
