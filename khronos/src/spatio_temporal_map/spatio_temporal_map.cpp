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

#include "khronos/spatio_temporal_map/spatio_temporal_map.h"

#include <fstream>
#include <numeric>

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <spark_dsg/serialization/binary_serialization.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>
#include <spark_dsg/serialization/versioning.h>

namespace khronos {

void declare_config(SpatioTemporalMap::Config& config) {
  using namespace config;
  name("SpatioTemporalMap");
  field(config.verbosity, "verbosity");
  field(config.finalize_incrementally, "finalize_incrementally");
}

SpatioTemporalMap::SpatioTemporalMap(const Config& config)
    : config(config::checkValid(config)), finalized_(config.finalize_incrementally) {}

void SpatioTemporalMap::copyMembers(const SpatioTemporalMap& other) {
  const_cast<Config&>(config) = other.config;
  stamps_ = other.stamps_;
  dsgs_ = other.dsgs_;
  earliest_ = other.earliest_;
  latest_ = other.latest_;
  current_time_ = other.current_time_;
  finalized_ = other.finalized_;
}

void SpatioTemporalMap::moveMembers(SpatioTemporalMap&& other) {
  const_cast<Config&>(config) = std::move(other.config);
  stamps_ = std::move(other.stamps_);
  dsgs_ = std::move(other.dsgs_);
  earliest_ = other.earliest_;
  latest_ = other.latest_;
  current_time_ = other.current_time_;
  finalized_ = other.finalized_;
}

SpatioTemporalMap::SpatioTemporalMap(const SpatioTemporalMap& other) { copyMembers(other); }

SpatioTemporalMap& SpatioTemporalMap::operator=(const SpatioTemporalMap& other) {
  copyMembers(other);
  return *this;
}

SpatioTemporalMap::SpatioTemporalMap(SpatioTemporalMap&& other) noexcept {
  moveMembers(std::move(other));
}

SpatioTemporalMap& SpatioTemporalMap::operator=(SpatioTemporalMap&& other) noexcept {
  moveMembers(std::move(other));
  return *this;
}

void SpatioTemporalMap::update(const DynamicSceneGraph::Ptr& dsg, TimeStamp stamp) {
  stamps_.push_back(stamp);
  // TODO(lschmid): Consider hard copies here.
  dsgs_.push_back(dsg);

  if (config.finalize_incrementally) {
    finalizeDsg(*dsg);

  } else {
    finalized_ = false;
  }
}

void SpatioTemporalMap::finalize() {
  if (finalized_) {
    return;
  }

  // Process all DSGs that have not yet been finalized.
  for (auto& dsg : dsgs_) {
    finalizeDsg(*dsg);
  }
  finalized_ = true;
}

void SpatioTemporalMap::finalizeDsg(DynamicSceneGraph& dsg) {
  finalizeMesh(*dsg.mesh());
  updateTimingInfo(dsg);
}

const DynamicSceneGraph& SpatioTemporalMap::getDsg(TimeStamp robot_time) {
  return *getDsgPtr(robot_time);
}

DynamicSceneGraph::Ptr SpatioTemporalMap::getDsgPtr(TimeStamp robot_time) {
  if (!finalized_) {
    finalize();
  }
  current_time_ = std::clamp(robot_time, earliest_, latest_);

  // If no change in time no change in DSG is needed.
  if (previous_time_ == current_time_) {
    return current_dsg_;
  }

  // Find the next closest DSG as data source.
  size_t new_dsg_idx =
      std::lower_bound(stamps_.begin(), stamps_.end(), current_time_) - stamps_.begin();
  new_dsg_idx = std::min(new_dsg_idx, stamps_.size() - 1);

  // If the source changes we need to reset the DSG.
  if (current_dsg_idx_ != new_dsg_idx) {
    current_dsg_idx_ = new_dsg_idx;
    current_dsg_ = dsgs_[current_dsg_idx_]->clone();
    previous_time_ = stamps_[current_dsg_idx_];
  }

  // Update the DSG based on robot time.
  if (current_time_ < previous_time_) {
    moveMeshBackward();
    moveAgentBackward();
    moveObjectsBackward();
    moveDynamicObjectAttributesBackward();
  } else if (current_time_ > previous_time_) {
    moveMeshForward();
    moveAgentForward();
    moveObjectsForward();
    moveDynamicObjectAttributesForward();
  }

  // Update time tracking.
  previous_time_ = current_time_;
  return current_dsg_;
}

void SpatioTemporalMap::moveMeshForward() {
  if (!current_dsg_->hasMesh()) {
    return;
  }

  auto& mesh = *current_dsg_->mesh();
  const auto& src_mesh = *dsgs_[current_dsg_idx_]->mesh();

  // Add vertices up to the robot time.
  const auto vertex_it = std::upper_bound(src_mesh.first_seen_stamps.begin(),
                                          src_mesh.first_seen_stamps.end(),
                                          current_time_,
                                          std::less<uint64_t>());

  const size_t num_new_vertices = vertex_it - src_mesh.first_seen_stamps.begin();
  const size_t num_old_vertices = mesh.numVertices();
  mesh.resizeVertices(num_new_vertices);
  for (size_t i = num_old_vertices; i < num_new_vertices; ++i) {
    mesh.setPos(i, src_mesh.pos(i));
    mesh.setColor(i, src_mesh.color(i));
    mesh.setFirstSeenTimestamp(i, src_mesh.firstSeenTimestamp(i));
    mesh.setTimestamp(i, src_mesh.timestamp(i));
  }
}

void SpatioTemporalMap::moveAgentForward() {
  if (!current_dsg_->hasLayer(DsgLayers::AGENTS, robot_prefix_.key)) {
    return;
  }

  // Add nodes starting from the current one.
  const auto& src_layer = dsgs_[current_dsg_idx_]->getLayer(DsgLayers::AGENTS, robot_prefix_.key);
  const size_t num_current_nodes =
      current_dsg_->getLayer(DsgLayers::AGENTS, robot_prefix_.key).numNodes();
  for (auto it = src_layer.nodes().begin() + num_current_nodes; it != src_layer.nodes().end();
       ++it) {
    const auto& node = **it;
    if (static_cast<size_t>(node.timestamp->count()) > current_time_) {
      return;
    }
    current_dsg_->emplacePrevDynamicNode(
        src_layer.id, node.id, node.timestamp.value(), node.attributes().clone());
  }
}

void SpatioTemporalMap::moveObjectsForward() {
  if (!current_dsg_->hasLayer(DsgLayers::OBJECTS)) {
    return;
  }

  const auto& src_layer = dsgs_[current_dsg_idx_]->getLayer(DsgLayers::OBJECTS);
  for (const auto& [id, node] : src_layer.nodes()) {
    if (current_dsg_->hasNode(id)) {
      continue;
    }
    auto& attrs = node->attributes<KhronosObjectAttributes>();
    if (attrs.details["AW_time"].front() > current_time_) {
      continue;
    }
    auto new_attrs = attrs.clone();
    // NOTE(lschmid): Clear dynamic attrs, these will be updated separately.
    auto& new_khronos_attrs = reinterpret_cast<KhronosObjectAttributes&>(*new_attrs);
    new_khronos_attrs.trajectory_timestamps.clear();
    new_khronos_attrs.trajectory_positions.clear();
    new_khronos_attrs.dynamic_object_points.clear();
    current_dsg_->emplaceNode(src_layer.id, id, std::move(new_attrs));
  }
}

void SpatioTemporalMap::moveDynamicObjectAttributesForward() {
  if (!current_dsg_->hasLayer(DsgLayers::OBJECTS)) {
    return;
  }

  for (const auto& [id, node] : current_dsg_->getLayer(DsgLayers::OBJECTS).nodes()) {
    auto& attrs = node->attributes<KhronosObjectAttributes>();
    const auto& src_attrs =
        dsgs_[current_dsg_idx_]->getNode(id).attributes<KhronosObjectAttributes>();
    if (src_attrs.trajectory_timestamps.size() == attrs.trajectory_timestamps.size()) {
      continue;
    }

    const auto it = std::upper_bound(src_attrs.trajectory_timestamps.begin(),
                                     src_attrs.trajectory_timestamps.end(),
                                     current_time_);
    const size_t num_old = attrs.trajectory_timestamps.size();
    const size_t num_new = it - src_attrs.trajectory_timestamps.begin();
    attrs.trajectory_timestamps.insert(attrs.trajectory_timestamps.end(),
                                       src_attrs.trajectory_timestamps.begin() + num_old,
                                       src_attrs.trajectory_timestamps.begin() + num_new);
    attrs.trajectory_positions.insert(attrs.trajectory_positions.end(),
                                      src_attrs.trajectory_positions.begin() + num_old,
                                      src_attrs.trajectory_positions.begin() + num_new);
    if (!src_attrs.dynamic_object_points.empty()) {
      attrs.dynamic_object_points.insert(attrs.dynamic_object_points.end(),
                                         src_attrs.dynamic_object_points.begin() + num_old,
                                         src_attrs.dynamic_object_points.begin() + num_new);
    }
  }
}

void SpatioTemporalMap::moveMeshBackward() {
  if (!current_dsg_->hasMesh()) {
    return;
  }

  // Prune all vertices that are newer than the robot time.
  // NOTE(lschmid): Vertices and faces are sorted by timestamp in pre-processing.
  auto& mesh = *current_dsg_->mesh();
  const auto vertex_it = std::lower_bound(mesh.first_seen_stamps.begin(),
                                          mesh.first_seen_stamps.end(),
                                          current_time_,
                                          std::less<uint64_t>());
  const size_t new_vertices = vertex_it - mesh.first_seen_stamps.begin();
  mesh.resizeVertices(new_vertices);
}

void SpatioTemporalMap::moveAgentBackward() {
  if (!current_dsg_->hasLayer(DsgLayers::AGENTS, robot_prefix_.key)) {
    return;
  }

  // Remove all nodes that are newer than the robot time.
  std::unordered_set<NodeId> nodes_to_remove;
  auto& agent_layer = current_dsg_->getLayer(DsgLayers::AGENTS, robot_prefix_.key);
  for (auto it = agent_layer.nodes().rbegin(); it != agent_layer.nodes().rend(); ++it) {
    const auto& node = *it;
    if (!node) {
      continue;
    }
    if (static_cast<size_t>(node->timestamp->count()) > current_time_) {
      nodes_to_remove.insert(node->id);
    } else {
      // NOTE(lschmid): The agent nodes are ordered by timestamp.
      break;
    }
  }
  for (const auto& node_id : nodes_to_remove) {
    current_dsg_->removeNode(node_id);
  }
}

void SpatioTemporalMap::moveObjectsBackward() {
  if (!current_dsg_->hasLayer(DsgLayers::OBJECTS)) {
    return;
  }

  // Remove all nodes that are newer than the robot time.
  std::unordered_set<NodeId> nodes_to_remove;
  for (auto& [id, node] : current_dsg_->getLayer(DsgLayers::OBJECTS).nodes()) {
    auto& attrs = node->attributes<KhronosObjectAttributes>();
    if (attrs.details["AW_time"].front() > current_time_) {
      nodes_to_remove.insert(id);
    }
  }
  for (const auto& node_id : nodes_to_remove) {
    current_dsg_->removeNode(node_id);
  }
}
void SpatioTemporalMap::moveDynamicObjectAttributesBackward() {
  if (!current_dsg_->hasLayer(DsgLayers::OBJECTS)) {
    return;
  }

  for (const auto& [id, node] : current_dsg_->getLayer(DsgLayers::OBJECTS).nodes()) {
    auto& attrs = node->attributes<KhronosObjectAttributes>();
    if (attrs.trajectory_timestamps.empty()) {
      continue;
    }

    if (attrs.trajectory_timestamps.back() < current_time_) {
      continue;
    }
    const auto it = std::upper_bound(
        attrs.trajectory_timestamps.begin(), attrs.trajectory_timestamps.end(), current_time_);
    const size_t num_new = it - attrs.trajectory_timestamps.begin();
    attrs.trajectory_timestamps.resize(num_new);
    attrs.trajectory_positions.resize(num_new);
    if (!attrs.dynamic_object_points.empty()) {
      attrs.dynamic_object_points.resize(num_new);
    }
  }
}

void SpatioTemporalMap::updateTimingInfo(const DynamicSceneGraph& dsg) {
  // Compute the prensence time based on the mesh.
  earliest_ = std::min(earliest_, dsg.mesh()->first_seen_stamps.front());
  latest_ = std::max(latest_, dsg.mesh()->stamps.back());
}

void SpatioTemporalMap::finalizeMesh(Mesh& mesh) {
  // Sort the mesh vertices and indices for easier addition and removal.
  const Mesh old_mesh = mesh;

  // Sort the vertices.
  const auto sorted_indices = sortIndices(old_mesh.first_seen_stamps);
  std::unordered_map<size_t, size_t> old_to_new;
  old_to_new.reserve(mesh.numVertices());
  for (size_t i = 0; i < mesh.numVertices(); ++i) {
    mesh.setPos(i, old_mesh.pos(sorted_indices[i]));
    mesh.setColor(i, old_mesh.color(sorted_indices[i]));
    mesh.setFirstSeenTimestamp(i, old_mesh.firstSeenTimestamp(sorted_indices[i]));
    mesh.setTimestamp(i, old_mesh.timestamp(sorted_indices[i]));
    old_to_new[sorted_indices[i]] = i;
  }

  // Update the face indices of the old mesh to the new indices.
  for (auto& face : mesh.faces) {
    for (auto& vertex : face) {
      vertex = old_to_new[vertex];
    }
  }
}

bool SpatioTemporalMap::save(std::string filepath) const {
  // Fix extension if needed.
  if (filepath.find('.') == std::string::npos) {
    filepath += kExtension;
  }

  // Setup file.
  std::ofstream out(filepath, std::ios::out | std::ios::binary);
  if (!out.is_open()) {
    LOG(ERROR) << "Could not open file " << filepath << " for writing.";
    return false;
  }

  // Save.
  std::vector<uint8_t> buffer;
  spark_dsg::serialization::BinarySerializer serializer(&buffer);

  // Write map meta data.
  serializer.write(kSerializationVersion);
  serializer.write(config.finalize_incrementally);
  serializer.write(stamps_.size());
  serializer.write(stamps_);
  serializer.write(earliest_);
  serializer.write(latest_);
  serializer.write(finalized_);
  serializer.write(spark_dsg::io::FileHeader::current().serializeToBinary());

  // Write DSGs.
  for (const auto& dsg : dsgs_) {
    std::vector<uint8_t> dsg_buffer;
    spark_dsg::io::binary::writeGraph(*dsg, dsg_buffer, true);
    serializer.write(dsg_buffer);
  }

  out.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
  return true;
}

std::unique_ptr<SpatioTemporalMap> SpatioTemporalMap::load(std::string filepath) {
  // Fix extension if needed.
  if (filepath.find('.') == std::string::npos) {
    filepath = filepath + kExtension;
  }

  // Setup file.
  std::ifstream in(filepath, std::ios::in | std::ios::binary);
  if (!in.is_open()) {
    LOG(ERROR) << "Could not open file " << filepath << " for reading.";
    return nullptr;
  }

  // Read file.
  std::vector<uint8_t> buffer(std::istreambuf_iterator<char>(in), {});
  spark_dsg::serialization::BinaryDeserializer deserializer(buffer);

  // Read map meta data.
  int version;
  deserializer.read(version);
  Config config;
  deserializer.read(config.finalize_incrementally);
  auto result = std::make_unique<SpatioTemporalMap>(config);
  size_t num_dsgs;
  deserializer.read(num_dsgs);
  deserializer.read(result->stamps_);
  deserializer.read(result->earliest_);
  deserializer.read(result->latest_);
  deserializer.read(result->finalized_);

  std::vector<uint8_t> header_buffer;
  deserializer.read(header_buffer);
  const auto header = spark_dsg::io::FileHeader::deserializeFromBinary(header_buffer);
  spark_dsg::io::GlobalInfo::ScopedInfo info(header.value());

  // Read DSGs.
  for (size_t i = 0; i < num_dsgs; ++i) {
    std::vector<uint8_t> dsg_buffer;
    deserializer.read(dsg_buffer);
    result->dsgs_.push_back(spark_dsg::io::binary::readGraph(dsg_buffer));
  }
  return result;
}

std::vector<size_t> sortIndices(const std::vector<uint64_t>& values) {
  std::vector<size_t> idx(values.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::stable_sort(
      idx.begin(), idx.end(), [&values](size_t i1, size_t i2) { return values[i1] < values[i2]; });
  return idx;
}

}  // namespace khronos
