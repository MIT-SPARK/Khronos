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

#include <cmath>
#include <string>
#include <vector>

#include "khronos/utils/geometry_utils.h"

namespace khronos {
namespace {

static const auto registration =
    config::RegistrationWithConfig<ObjectDetector, InstanceForwarding, InstanceForwarding::Config>(
        "InstanceForwarding");

static const auto open_vocab_registration =
    config::RegistrationWithConfig<InstanceFilter,
                                   OpenVocabBackgroundFilter,
                                   OpenVocabBackgroundFilter::Config>("OpenVocabBackgroundFilter");

static const auto category_registration =
    config::RegistrationWithConfig<InstanceFilter, CategoryFilter, CategoryFilter::Config>(
        "CategoryFilter");

std::vector<int32_t> getDefaultInvalidLabels() {
  const auto& invalid = hydra::GlobalInfo::instance().labelspace().invalid_labels;
  return {invalid.begin(), invalid.end()};
}

std::optional<SemanticClusterInfo> extractSemantics(const FrameData& data,
                                                    InputData::InstanceType id,
                                                    const Pixels& pixels) {
  if (!data.input.label_features.empty()) {
    const auto feature = data.input.label_features.find(id);
    if (feature != data.input.label_features.end()) {
      return SemanticClusterInfo(feature->second);
    }

    return std::nullopt;
  }

  if (data.input.label_image.empty()) {
    return std::nullopt;
  }

  const auto [u, v] = pixels.front();
  const auto category_id = data.input.label_image.at<InputData::LabelType>(v, u);
  return SemanticClusterInfo(category_id);
}

}  // namespace

void declare_config(CategoryFilter::Config& config) {
  using namespace config;
  name("CategoryFilter::Config");
  field(config.invalid, "invalid");
}

CategoryFilter::Config::Config() : invalid(getDefaultInvalidLabels()) {}

CategoryFilter::CategoryFilter(const Config& config)
    : config(config::checkValid(config)), invalid_(config.invalid.begin(), config.invalid.end()) {}

bool CategoryFilter::valid(const FrameData& data, int32_t, const Pixels& pixels) const {
  if (pixels.empty() || data.input.label_image.empty()) {
    return false;
  }

  const auto [u, v] = pixels.front();
  const auto label = data.input.label_image.at<InputData::LabelType>(v, u);
  return !invalid_.count(label);
}

void declare_config(OpenVocabBackgroundFilter::Config& config) {
  using namespace config;
  name("OpenVocabBackgroundFilter::Config");
  field(config.max_background_score, "max_background_score");
  field(config.background, "background");
  field(config.metric, "metric");
}

OpenVocabBackgroundFilter::OpenVocabBackgroundFilter(const Config& config)
    : config(config::checkValid(config)),
      background_(config.background.create()),
      metric_(config.metric.create()) {}

bool OpenVocabBackgroundFilter::valid(const FrameData& data, int32_t id, const Pixels&) const {
  // Filter background based on given prompt
  const auto feature = data.input.label_features.find(id);
  if (feature == data.input.label_features.end()) {
    return false;
  }

  auto score = background_->getBestScore(*metric_, feature->second);
  if (score.score > config.max_background_score) {
    return false;
  }

  return true;
}

void declare_config(InstanceForwarding::Config& config) {
  using namespace config;
  name("InstanceForwarding");
  field(config.verbosity, "verbosity");
  field(config.max_range, "max_range", "m");
  field(config.min_range, "min_range", "m");
  field(config.zero_is_unlabeled, "zero_is_unlabeled");
  field(config.min_cluster_size, "min_cluster_size");
  field(config.max_cluster_size, "max_cluster_size");
  field(config.min_object_volume, "min_object_volume", "m");
  field(config.max_object_volume, "max_object_volume", "m");
  config.instance_filter.setOptional();
  field(config.instance_filter, "instance_filter");
  field(config.max_background_score, "max_background_score");
  field(config.instance_id, "instance_id");
  config.background.setOptional();
  field(config.background, "background");
  config.metric.setOptional();
  field(config.metric, "metric");

  {
    NameSpace ns("outlier_filter");
    field(config.outlier_filter_enabled, "enabled");
    field(config.outlier_filter_eps, "eps", "m");
    field(config.outlier_filter_min_points, "min_points");
  }
}

InstanceForwarding::InstanceForwarding(const Config& config)
    : config(config::checkValid(config)),
      filter_by_volume_(config.min_object_volume > 0.0 || config.max_object_volume > 0.0),
      instance_filter_(config.instance_filter.create()) {}

void InstanceForwarding::processInput(const VolumetricMap& /* map */, FrameData& data) {
  processing_stamp_ = data.input.timestamp_ns;
  Timer timer("object_detection/all", processing_stamp_);

  extractSemanticClusters(data);
}

void InstanceForwarding::extractSemanticClusters(FrameData& data) {
  // Extract clusters
  std::unordered_map<FrameData::ObjectImageType, Pixels> clusters;
  for (int u = 0; u < data.input.instance_image.cols; u++) {
    for (int v = 0; v < data.input.instance_image.rows; v++) {
      const auto id = data.input.instance_image.at<InputData::InstanceType>(v, u);
      if (config.zero_is_unlabeled && id == 0) {
        continue;
      }

      const auto range = data.input.range_image.at<InputData::RangeType>(v, u);
      if (range < config.min_range || (config.max_range > 0.f && range > config.max_range)) {
        continue;
      }

      const auto& vertex = data.input.vertex_map.at<InputData::VertexType>(v, u);
      if (!std::isfinite(vertex[0]) || !std::isfinite(vertex[1]) || !std::isfinite(vertex[2])) {
        continue;
      }

      if (config.max_range > 0.f || config.min_range > 0.f) {
        const float range = data.input.range_image.at<InputData::RangeType>(v, u);
        if (range < config.min_range || (config.max_range > 0.f && range > config.max_range)) {
          continue;
        }
      }

      data.object_image.at<FrameData::ObjectImageType>(v, u) = id;
      clusters[id].emplace_back(u, v);
    }
  }

  // Filter clusters and populate object image
  for (const auto& [id, pixels] : clusters) {
    const auto curr_num_pixels = static_cast<int>(pixels.size());
    if (curr_num_pixels < config.min_cluster_size ||
        (config.max_cluster_size > 0 && curr_num_pixels > config.max_cluster_size)) {
      continue;
    }

    MeasurementCluster cluster;
    cluster.pixels.insert(cluster.pixels.end(), pixels.begin(), pixels.end());
    cluster.id = id;

    if (config.outlier_filter_enabled) {
      Points points;
      points.reserve(cluster.pixels.size());
      for (const auto& pixel : cluster.pixels) {
        const auto& vertex = data.input.vertex_map.at<InputData::VertexType>(pixel.v, pixel.u);
        points.emplace_back(vertex[0], vertex[1], vertex[2]);
      }
      const auto inlier_indices = utils::largestDbscanCluster(
          points, config.outlier_filter_eps, config.outlier_filter_min_points);
      if (inlier_indices.empty()) {
        continue;
      }
      Pixels filtered_pixels;
      filtered_pixels.reserve(inlier_indices.size());
      for (const size_t idx : inlier_indices) {
        filtered_pixels.push_back(cluster.pixels[idx]);
      }
      cluster.pixels = std::move(filtered_pixels);
    }

    if (filter_by_volume_) {
      const auto bbox = BoundingBox(utils::VertexMapAdaptor(pixels, data.input.vertex_map));
      const auto volume = bbox.volume();
      if (volume < config.min_object_volume ||
          (config.max_object_volume > 0.0 && volume > config.max_object_volume)) {
        continue;
      }
    }

    if (instance_filter_ && !instance_filter_->valid(data, id, pixels)) {
      continue;
    }

    // Technically we could fill the object image during extraction and not worry about filtered
    // instances but this is probably better
    for (const auto& [u, v] : pixels) {
      data.object_image.at<FrameData::ObjectImageType>(v, u) = id;
    }

    MeasurementCluster cluster;
    cluster.id = id;
    cluster.pixels.insert(cluster.pixels.end(), pixels.begin(), pixels.end());
    cluster.semantics = extractSemantics(data, id, pixels);
    data.semantic_clusters.emplace_back(std::move(cluster));
  }
}

}  // namespace khronos
