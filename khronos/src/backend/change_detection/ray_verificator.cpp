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

#include "khronos/backend/change_detection/ray_verificator.h"

#include <stdlib.h>

namespace khronos {

void declare_config(RayVerificator::Config& config) {
  using namespace config;
  name("RayVerificator");
  field(config.verbosity, "verbosity");
  field(config.block_size, "block_size", "m");
  field(config.radial_tolerance, "radial_tolerance", "m");
  field(config.depth_tolerance, "depth_tolerance", "m");
  enum_field(config.ray_policy,
             "ray_policy",
             {"First", "Last", "FirstAndLast", "Middle", "All", "Random", "Random3"});
  field(config.active_window_duration, "active_window_duration", "s");
  field(config.prefix, "prefix");

  check(config.block_size, GT, 0.f, "block_size");
  check(config.radial_tolerance, GT, 0.f, "radial_tolerance");
  check(config.depth_tolerance, GT, 0.f, "depth_tolerance");
}

RayVerificator::RayVerificator(const Config& config)
    : config(config::checkValid(config)), grid_(config.block_size), seed_(time(NULL)) {}

RayVerificator::CheckResult RayVerificator::check(const Point& point,
                                                  const uint64_t earliest,
                                                  const uint64_t latest,
                                                  CheckDetails* details) const {
  CheckResult result;

  if (rays_.empty()) {
    CLOG(6) << "Point unobserved: no measurements.";
    return result;
  }

  // Lookup all candidate views that could have observed the point.
  const auto it = block_seen_by_rays_.find(grid_.toIndex(point));
  if (it == block_seen_by_rays_.end()) {
    CLOG(6) << "Point unobserved: no measurements_.";
    return result;
  }

  // TODO(lschmid): This is a bit inefficient, we could cache the lookup.
  const RayLookup lookup(*dsg_, config);

  // Check all candidate views.
  for (size_t ray_index : it->second) {
    const Ray& ray = rays_.at(ray_index);
    if (ray.timestamp < earliest || ray.timestamp > latest) {
      // This measurement is out of the temporal range to check.
      continue;
    }

    const Point source = lookup.getSource(ray);
    const Point direction = (point - source).normalized();
    const float depth = (point - source).norm();

    // Check all vertices representd as rays.
    const Point vertex = lookup.getTarget(ray);
    const float radial_distance = (point - source).cross(source - vertex).norm() / depth;

    if (details) {
      details->start.emplace_back(source);
      details->end.emplace_back(vertex);
    }
    if (radial_distance > config.radial_tolerance) {
      // No overlap on the ray.
      if (details) {
        details->range.emplace_back(0.f);
        details->result.emplace_back(CheckDetails::Result::kNoOverlap);
      }
      continue;
    }

    const float depth_distance = (vertex - source).dot(direction);
    if (details) {
      details->range.emplace_back(depth_distance);
    }
    if (depth - depth_distance > config.depth_tolerance) {
      // This is an occlusion, the point has not been observed.
      if (details) {
        details->result.emplace_back(CheckDetails::Result::kOccludded);
      }
      continue;
    }

    if (depth_distance - depth > config.depth_tolerance) {
      // This is a ray through our point, so it's evidence of absence.
      result.absent.emplace_back(ray.timestamp);
      if (details) {
        details->result.emplace_back(CheckDetails::Result::kAbsent);
      }
      continue;
    }

    // This means the point is within the tolerance, it's a match.
    result.present.emplace_back(ray.timestamp);
    if (details) {
      details->result.emplace_back(CheckDetails::Result::kMatch);
    }
  }

  return result;
}

void RayVerificator::setDsg(std::shared_ptr<const DynamicSceneGraph> dsg) {
  // Clear the current state and add DSG data from the start.
  dsg_ = std::move(dsg);
  rays_.clear();
  timestamps_.clear();
  block_seen_by_rays_.clear();
  vertices_in_block_.clear();
  objects_in_block_.clear();
  previous_node_index_ = 0;
  previous_vertex_index_ = 0;
  previous_object_index_ = 0;
  addPoseNodes();
  addVertices();
}

void RayVerificator::updateDsg() {
  // Add only the new parts to the measurements and hash.
  addPoseNodes();
  BlockIndexSet observed_blocks = addVertices();
  addObjectsToHash();

  // Compute the newly re-observed vertices and objects.
  reobserved_vertices_.clear();
  reobserved_objects_.clear();
  for (const auto& index : observed_blocks) {
    const auto it = vertices_in_block_.find(index);
    if (it != vertices_in_block_.end()) {
      reobserved_vertices_.insert(it->second.begin(), it->second.end());
    }
    const auto it2 = objects_in_block_.find(index);
    if (it2 != objects_in_block_.end()) {
      reobserved_objects_.insert(it2->second.begin(), it2->second.end());
    }
  }
}

void RayVerificator::addPoseNodes() {
  if (!dsg_ || !dsg_->hasLayer(DsgLayers::AGENTS, config.prefix.key)) {
    return;
  }

  // Add all new sensor poses to the possible timestamps.
  const auto& nodes = dsg_->getLayer(DsgLayers::AGENTS, config.prefix.key).nodes();
  for (size_t i = previous_node_index_; i < nodes.size(); ++i) {
    const auto& node = *nodes[i];
    // NOTE(lschmid): These should be ordered already so timestamps should be sorted by
    // construction.
    if (node.timestamp) timestamps_.emplace_back(static_cast<uint64_t>(node.timestamp->count()));
  }
  previous_node_index_ = nodes.size();
}

BlockIndexSet RayVerificator::addVertices() {
  BlockIndexSet observed_blocks;

  // For all vertices, compute the sources they belong to and add them to the library of rays.
  const auto& vertices = dsg_->mesh()->points;
  const auto& first_seen = dsg_->mesh()->first_seen_stamps;
  auto last_seen = dsg_->mesh()->stamps;
  if (config.active_window_duration > 0) {
    const uint64_t offset_ns = config.active_window_duration * 1e9;
    for (auto& stamp : last_seen) {
      stamp -= offset_ns;
    }
  }

  for (size_t i = previous_vertex_index_; i < vertices.size(); ++i) {
    // Add the vertex to the hash.
    vertices_in_block_[grid_.toIndex(vertices.at(i))].insert(i);

    // Compute which measurements this vertex belongs to.
    const auto source_indices = computeVertexSources(first_seen.at(i), last_seen.at(i));
    if (source_indices.empty()) {
      continue;
    }

    // Create the rays and add them to the hash.
    for (const size_t source_index : source_indices) {
      rays_.emplace_back(timestamps_[source_index], source_index, i);
      observed_blocks.merge(addRayToHash(rays_.size() - 1));
    }
  }
  previous_vertex_index_ = vertices.size();

  return observed_blocks;
}

std::unordered_set<size_t> RayVerificator::computeVertexSources(const size_t first_seen,
                                                                const size_t last_seen) {
  std::unordered_set<size_t> result;

  // Compute which source points (indicated by timestamps) are relevant for this vertex.
  if (config.ray_policy == Config::RayPolicy::kFirst ||
      config.ray_policy == Config::RayPolicy::kFirstAndLast) {
    const auto it = std::upper_bound(timestamps_.begin(), timestamps_.end(), first_seen);
    if (it != timestamps_.end()) {
      result.insert(it - timestamps_.begin());
    }
  }
  if (config.ray_policy == Config::RayPolicy::kLast ||
      config.ray_policy == Config::RayPolicy::kFirstAndLast) {
    const auto it = std::lower_bound(timestamps_.begin(), timestamps_.end(), last_seen);
    if (it != timestamps_.end()) {
      result.insert(it - timestamps_.begin());
    }
  }
  if (config.ray_policy == Config::RayPolicy::kMiddle) {
    const size_t stamp = (last_seen + first_seen) / 2;
    const auto it = std::lower_bound(timestamps_.begin(), timestamps_.end(), stamp);
    if (it != timestamps_.end()) {
      result.insert(it - timestamps_.begin());
    }
  }
  if (config.ray_policy == Config::RayPolicy::kAll) {
    const auto it_lower = std::upper_bound(timestamps_.begin(), timestamps_.end(), first_seen);
    const auto it_upper = std::lower_bound(timestamps_.begin(), timestamps_.end(), last_seen);
    for (auto it = it_lower; it < it_upper; ++it) {
      result.insert(it - timestamps_.begin());
    }
  }
  if (config.ray_policy == Config::RayPolicy::kRandom ||
      config.ray_policy == Config::RayPolicy::kRandom3) {
    const auto it_lower = std::upper_bound(timestamps_.begin(), timestamps_.end(), first_seen);
    const auto it_upper = std::lower_bound(timestamps_.begin(), timestamps_.end(), last_seen);
    const size_t range = std::distance(it_lower, it_upper);
    const size_t start = std::distance(timestamps_.begin(), it_lower);
    if (range > 0) {
      for (size_t i = 0; i < (config.ray_policy == Config::RayPolicy::kRandom3 ? 3 : 1); ++i) {
        size_t index = start + rand_r(&seed_) % range;
        if (index < timestamps_.size()) {
          result.insert(index);
        }
      }
    }
  }
  return result;
}

void RayVerificator::recomputeHash() {
  // Populate the block hash for lookup of poses, vertices, and objects later.
  block_seen_by_rays_.clear();
  vertices_in_block_.clear();
  objects_in_block_.clear();
  for (size_t i = 0; i < rays_.size(); ++i) {
    addRayToHash(i);
  }
  addObjectsToHash();
}

BlockIndexSet RayVerificator::addRayToHash(const size_t ray_index) {
  BlockIndexSet observed_blocks;
  const Ray& ray = rays_.at(ray_index);
  // TODO(lschmid): This is a bit inefficient, we could cache the lookup.
  const RayLookup lookup(*dsg_, config);
  const Point source = lookup.getSource(ray);
  const Point target = lookup.getTarget(ray);
  const Point direction = (target - source).normalized();
  const float max_depth = (target - source).norm();
  const float ray_step = config.block_size / 4;
  float ray_distance = 0.f;

  // NOTE(lschmid): This ray marching may miss some corner case blocks but that's fine, this is a
  // preliminary implementation.
  while (ray_distance <= max_depth) {
    ray_distance += ray_step;
    const Point ray_point = source + ray_distance * direction;
    const BlockIndex index = grid_.toIndex(ray_point);
    block_seen_by_rays_[index].insert(ray_index);
    observed_blocks.insert(index);
  }
  return observed_blocks;
}

void RayVerificator::addObjectsToHash() {
  if (!dsg_->hasLayer(DsgLayers::OBJECTS)) {
    return;
  }

  for (const auto& [id, node] : dsg_->getLayer(DsgLayers::OBJECTS).nodes()) {
    // NOTE(lschmid): This makes the implicit assumption that objects are added in order.
    if (id <= previous_object_index_) {
      continue;
    }
    previous_object_index_ = id;

    // Iterate over all vertices and add them to the hash.
    const auto& attrs = node->attributes<KhronosObjectAttributes>();
    BlockIndexSet block_indices;
    for (const auto& vertex : attrs.mesh.points) {
      block_indices.insert(grid_.toIndex(vertex));
    }

    for (const auto& index : block_indices) {
      objects_in_block_[index].insert(id);
    }
  }
}

}  // namespace khronos
