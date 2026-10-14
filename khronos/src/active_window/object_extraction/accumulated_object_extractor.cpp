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

#include "khronos/active_window/object_extraction/accumulated_object_extractor.h"

#include <glog/logging.h>
#include <spatial_hash/grid.h>

namespace khronos {

void declare_config(AccumulatedObjectExtractor::Config& config) {
  using namespace config;
  name("AccumulatedObjectExtractor::Config");
  field(config.verbosity, "verbosity");
  field(config.min_object_allocation_confidence, "min_object_allocation_confidence");
  field(config.min_object_volume, "min_object_volume", "m^3");
  field(config.max_object_volume, "max_object_volume", "m^3");

  checkInRange(
      config.min_object_allocation_confidence, 0.0f, 1.0f, "min_object_allocation_confidence");
  check(config.min_object_volume, GE, 0.0f, "min_object_volume");
  check(config.max_object_volume, GE, config.min_object_volume, "max_object_volume");
}

AccumulatedObjectExtractor::AccumulatedObjectExtractor(const Config& config)
    : config(config::checkValid(config)) {}

KhronosObjectAttributes::Ptr AccumulatedObjectExtractor::extractObject(
    const Track& track,
    const FrameDataBuffer& /* frame_data */) {
  if (track.confidence <= config.min_object_allocation_confidence) {
    CLOG(5) << "[AccumulatedObjectExtractor] Dropping track " << track.id << ": low confidence ("
            << track.confidence << " <= " << config.min_object_allocation_confidence << ").";
    return nullptr;
  }

  auto object = track.is_dynamic ? extractDynamicObject(track) : extractStaticObject(track);
  if (!object) {
    return nullptr;
  }

  if (track.semantics) {
    object->semantic_label = track.semantics->category_id;
    object->semantic_feature = track.semantics->feature;
  }
  object->first_observed_ns = {track.first_seen};
  object->last_observed_ns = {track.last_seen};
  object->position = object->bounding_box.world_P_center.cast<double>();
  return object;
}

KhronosObjectAttributes::Ptr AccumulatedObjectExtractor::extractStaticObject(
    const Track& track) const {
  if (!track.accumulated_bounding_box.isValid()) {
    if (track.accumulated_voxels.empty()) {
      LOG_FIRST_N(WARNING, 1) << "[AccumulatedObjectExtractor] Track " << track.id
                              << " has no accumulated voxels. Run the tracker with "
                                 "accumulate_voxels: true.";
    }
    CLOG(5) << "[AccumulatedObjectExtractor] Dropping track " << track.id
            << ": no confirmed voxels.";
    return nullptr;
  }

  const BoundingBox& bbox = track.accumulated_bounding_box;
  const float volume = bbox.volume();
  if (volume < config.min_object_volume || volume > config.max_object_volume) {
    CLOG(5) << "[AccumulatedObjectExtractor] Dropping track " << track.id << ": volume " << volume
            << " outside [" << config.min_object_volume << ", " << config.max_object_volume
            << "].";
    return nullptr;
  }

  auto object = std::make_unique<KhronosObjectAttributes>();
  object->bounding_box = bbox;

  // Confirmed voxel centers as points, relative to the bounding box center.
  const auto voxels = track.confirmedVoxels();
  object->mesh.resizeVertices(voxels.size());
  size_t i = 0;
  for (const GlobalIndex& index : voxels) {
    const Point center = spatial_hash::centerPointFromIndex(index, track.accumulated_voxel_size);
    object->mesh.setPos(i++, center - bbox.world_P_center);
  }

  CLOG(5) << "[AccumulatedObjectExtractor] Extracted track " << track.id << " with volume "
          << volume << " and " << voxels.size() << " points.";
  return object;
}

KhronosObjectAttributes::Ptr AccumulatedObjectExtractor::extractDynamicObject(
    const Track& track) const {
  auto object = std::make_unique<KhronosObjectAttributes>();
  object->bounding_box = track.last_bounding_box;
  object->trajectory_positions = {track.last_centroid};
  object->trajectory_timestamps = {track.last_seen};
  CLOG(5) << "[AccumulatedObjectExtractor] Extracted dynamic track " << track.id << ".";
  return object;
}

}  // namespace khronos
