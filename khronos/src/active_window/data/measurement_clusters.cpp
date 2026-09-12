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

#include "khronos/active_window/data/measurement_clusters.h"

#include <nlohmann/json.hpp>
#include <spark_dsg/serialization/json_conversions.h>  // Eigen adl_serializer (Vector3f, VectorXf, ...)

namespace khronos {

nlohmann::json boundingBoxToJson(const BoundingBox& bbox) {
  return nlohmann::json{{"type", static_cast<int32_t>(bbox.type)},
                        {"dimensions", bbox.dimensions},
                        {"world_P_center", bbox.world_P_center},
                        {"world_R_center", Eigen::Quaternionf(bbox.world_R_center)}};
}

BoundingBox boundingBoxFromJson(const nlohmann::json& j) {
  BoundingBox bbox;
  bbox.type = static_cast<BoundingBox::Type>(j.at("type").get<int32_t>());
  bbox.dimensions = j.at("dimensions").get<Eigen::Vector3f>();
  bbox.world_P_center = j.at("world_P_center").get<Eigen::Vector3f>();
  bbox.world_R_center = j.at("world_R_center").get<Eigen::Quaternionf>().toRotationMatrix();
  return bbox;
}

nlohmann::json SemanticClusterInfo::toJson() const {
  nlohmann::json j{{"category_id", category_id}};
  if (feature.size() > 1) {
    j["feature"] = feature;
  }
  return j;
}

SemanticClusterInfo SemanticClusterInfo::fromJson(const nlohmann::json& j) {
  SemanticClusterInfo semantics(j.at("category_id").get<int>());
  if (j.contains("feature")) {
    semantics.feature = j.at("feature").get<FeatureVector>();
  }
  return semantics;
}

nlohmann::json MeasurementCluster::toJson() const {
  nlohmann::json j{{"id", id}, {"bounding_box", boundingBoxToJson(bounding_box)}};
  j["semantics"] = semantics ? semantics->toJson() : nlohmann::json(nullptr);
  return j;
}

MeasurementCluster MeasurementCluster::fromJson(const nlohmann::json& j) {
  MeasurementCluster cluster;
  cluster.id = j.at("id").get<int>();
  cluster.bounding_box = boundingBoxFromJson(j.at("bounding_box"));
  if (!j.at("semantics").is_null()) {
    cluster.semantics = SemanticClusterInfo::fromJson(j.at("semantics"));
  }
  return cluster;
}

void to_json(nlohmann::json& j, const SemanticClusterInfo& semantics) { j = semantics.toJson(); }

void from_json(const nlohmann::json& j, SemanticClusterInfo& semantics) {
  semantics = SemanticClusterInfo::fromJson(j);
}

void to_json(nlohmann::json& j, const MeasurementCluster& cluster) { j = cluster.toJson(); }

void from_json(const nlohmann::json& j, MeasurementCluster& cluster) {
  cluster = MeasurementCluster::fromJson(j);
}

}  // namespace khronos
