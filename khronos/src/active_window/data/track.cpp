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

#include "khronos/active_window/data/track.h"

#include <fstream>

#include <nlohmann/json.hpp>
#include <spark_dsg/serialization/json_conversions.h>  // Eigen adl_serializer (Vector3f, VectorXf, ...)

namespace khronos {
namespace {

using json = nlohmann::json;

json toJson(const GlobalIndexSet& voxels) {
  json arr = json::array();
  for (const GlobalIndex& voxel : voxels) {
    arr.push_back({voxel.x(), voxel.y(), voxel.z()});
  }
  return arr;
}

GlobalIndexSet globalIndexSetFromJson(const json& j) {
  GlobalIndexSet voxels;
  for (const auto& entry : j) {
    voxels.insert(GlobalIndex(entry.at(0).get<int>(), entry.at(1).get<int>(), entry.at(2).get<int>()));
  }
  return voxels;
}

}  // namespace

void to_json(json& j, const Observation& obs) {
  j = json{{"stamp", obs.stamp},
          {"semantic_cluster_id", obs.semantic_cluster_id},
          {"dynamic_cluster_id", obs.dynamic_cluster_id},
          {"sensor", obs.sensor}};
}

void from_json(const json& j, Observation& obs) {
  obs.stamp = j.at("stamp").get<TimeStamp>();
  obs.semantic_cluster_id = j.at("semantic_cluster_id").get<int>();
  obs.dynamic_cluster_id = j.at("dynamic_cluster_id").get<int>();
  // Tolerant read: older saved tracks predate the "sensor" field.
  obs.sensor = j.value("sensor", std::string());
}

void to_json(json& j, const Track& track) {
  j = json{{"id", track.id},
          {"last_seen", track.last_seen},
          {"first_seen", track.first_seen},
          {"observations", track.observations},
          {"last_bounding_box", boundingBoxToJson(track.last_bounding_box)},
          {"last_voxels", toJson(track.last_voxels)},
          {"last_points", track.last_points},
          {"last_voxel_size", track.last_voxel_size},
          {"last_centroid", track.last_centroid},
          {"num_features", track.num_features},
          {"is_dynamic", track.is_dynamic},
          {"confidence", track.confidence}};

  j["semantics"] = track.semantics ? track.semantics->toJson() : json(nullptr);
}

void from_json(const json& j, Track& track) {
  track.id = j.at("id").get<int>();
  track.last_seen = j.at("last_seen").get<TimeStamp>();
  track.first_seen = j.at("first_seen").get<TimeStamp>();
  track.observations = j.at("observations").get<Observations>();
  track.last_bounding_box = boundingBoxFromJson(j.at("last_bounding_box"));
  track.last_voxels = globalIndexSetFromJson(j.at("last_voxels"));
  track.last_points = j.at("last_points").get<Points>();
  track.last_voxel_size = j.at("last_voxel_size").get<float>();
  track.last_centroid = j.at("last_centroid").get<Point>();
  track.num_features = j.at("num_features").get<size_t>();
  track.is_dynamic = j.at("is_dynamic").get<bool>();
  track.confidence = j.at("confidence").get<float>();

  if (!j.at("semantics").is_null()) {
    track.semantics = SemanticClusterInfo::fromJson(j.at("semantics"));
  }
}

void Track::save(const std::string& filepath) const {
  std::ofstream file(filepath);
  if (!file.is_open()) {
    LOG(ERROR) << "[Track] Failed to open '" << filepath << "' for writing.";
    return;
  }
  file << json(*this).dump(2);
}

Track Track::load(const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    LOG(ERROR) << "[Track] Failed to open '" << filepath << "' for reading.";
    return Track();
  }
  json j;
  file >> j;
  return j.get<Track>();
}

void Track::updateSemantics(const std::optional<SemanticClusterInfo>& other) {
  if (!other) {
    return;
  }

  if (!semantics) {
    semantics = other;
    return;
  }

  const bool other_has_feature = other->feature.size() > 0;
  const bool has_feature = semantics->feature.size() > 0;
  if (has_feature && !other_has_feature) {
    return;  // prefer to keep feature
  }

  if (other_has_feature && !has_feature) {
    semantics->feature = other->feature;
    ++num_features;
    return;
  }

  // incremental mean
  const auto total_features = static_cast<float>(num_features + 1);
  const auto prev_weight = static_cast<float>(num_features) / total_features;
  const auto new_weight = 1.0f / total_features;
  semantics->feature = prev_weight * semantics->feature + new_weight * other->feature;
  ++num_features;
}

}  // namespace khronos
