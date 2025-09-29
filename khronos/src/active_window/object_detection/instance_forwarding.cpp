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

#include "khronos/active_window/object_detection/instance_forwarding.h"

#include <string>
#include <vector>

#include "khronos/utils/geometry_utils.h"

namespace khronos {

void declare_config(InstanceForwarding::Config& config) {
  using namespace config;
  name("InstanceForwarding");
  field(config.verbosity, "verbosity");
  field(config.max_range, "max_range", "m");
  field(config.min_cluster_size, "min_cluster_size");
  field(config.max_cluster_size, "max_cluster_size");
  field(config.min_object_volume, "min_object_volume", "m");
  field(config.max_object_volume, "max_object_volume", "m");
  field(config.max_background_score, "max_background_score");
  config.background.setOptional();
  field(config.background, "background");
  config.metric.setOptional();
  field(config.metric, "metric");
}

InstanceForwarding::InstanceForwarding(const Config& config)
    : config(config::checkValid(config)),
      filter_by_volume_(config.min_object_volume > 0.0 || config.max_object_volume > 0.0),
      background_(config.background.create()),
      metric_(config.metric.create()) {
  if (background_) {
    CHECK(metric_) << "Specify the metric if background is set.";
  }
}

void InstanceForwarding::processInput(const VolumetricMap& /* map */, FrameData& data) {
  processing_stamp_ = data.input.timestamp_ns;
  Timer timer("object_detection/all", processing_stamp_);

  extractSemanticClusters(data);
}

void InstanceForwarding::extractSemanticClusters(FrameData& data) {
  // Forward the semantic image from the input.
  // NOTE(lschmid): This assumes both images have the same type.
  data.object_image = data.input.label_image;

  // Extract clusters.
  std::unordered_map<FrameData::ObjectImageType, Pixels> clusters;
  for (int u = 0; u < data.input.label_image.cols; u++) {
    for (int v = 0; v < data.input.label_image.rows; v++) {
      const auto& id = data.input.label_image.at<InputData::LabelType>(v, u);
      if (id == 0) {
        continue;
      }

      // Filter background based on given prompt
      if (background_) {
        const auto feature = data.input.label_features.find(id);
        if (feature == data.input.label_features.end()) {
          continue;
        }
        auto score = background_->getBestScore(*metric_, feature->second);
        if (score.score > config.max_background_score) {
          continue;
        }
      }

      if (config.max_range > 0.f) {
        const float range = data.input.range_image.at<InputData::RangeType>(v, u);
        if (range > config.max_range) {
          continue;
        }
      }
      data.object_image.at<FrameData::ObjectImageType>(v, u) = id;
      clusters[id].emplace_back(u, v);
    }
  }

  for (const auto& [id, pixels] : clusters) {
    const auto curr_num_pixels = static_cast<int>(pixels.size());
    if (curr_num_pixels < config.min_cluster_size ||
        (config.max_cluster_size > 0 && curr_num_pixels > config.max_cluster_size)) {
      continue;
    }

    MeasurementCluster cluster;
    cluster.pixels.insert(cluster.pixels.end(), pixels.begin(), pixels.end());
    cluster.id = id;

    if (filter_by_volume_) {
      const auto bbox = BoundingBox(utils::VertexMapAdaptor(cluster.pixels, data.input.vertex_map));
      const auto volume = bbox.volume();
      if (volume < config.min_object_volume ||
          (config.max_object_volume > 0.0 && volume > config.max_object_volume)) {
        continue;
      }
    }

    // Closed set version
    if (data.input.label_features.empty()) {
      cluster.semantics = SemanticClusterInfo(id);
    }
    // TODO(Yun) For now all semantic id is the same (so all label checks are invalid)
    const auto feature = data.input.label_features.find(id);
    if (feature != data.input.label_features.end()) {
      cluster.semantics = SemanticClusterInfo(id, feature->second);
    }

    data.semantic_clusters.emplace_back(std::move(cluster));
  }
}

}  // namespace khronos
