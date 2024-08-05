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

#include "khronos_ros/visualization/dsg_visualizer_plugins/khronos_mesh_plugin.h"

#include <spark_dsg/colormaps.h>

#include "khronos_ros/utils/ros_conversions.h"
#include "khronos_ros/visualization/visualization_utils.h"

namespace khronos {

void declare_config(KhronosMeshPlugin::Config& config) {
  using namespace config;
  name("KhronosMeshPlugin");
  field(config.verbosity, "verbosity");
  field(config.changes_topic, "changes_topic");
  field(config.background_changes_file, "background_changes_file");
  field(config.object_changes_file, "object_changes_file");
  field(config.mesh_visualizer, "mesh_visualizer");
  checkCondition(!config.changes_topic.empty(), "'changes_topic' can not be empty");
}

KhronosMeshPlugin::KhronosMeshPlugin(const Config& config,
                                     const ros::NodeHandle& nh,
                                     const std::string& name)
    : VisualizerPlugin(nh, name),
      config(config::checkValid(config)),
      dynamic_config_(nh_, ""),
      mesh_visualizer_(config.mesh_visualizer, nh_) {
  dynamic_config_.setUpdateCallback([this](const auto&) { has_changes_ = true; });
  changes_sub_ = nh_.subscribe(config.changes_topic, 1, &KhronosMeshPlugin::changesCallback, this);

  // Load files if specified.
  if (!config.background_changes_file.empty()) {
    if (changes_.background_changes.load(config.background_changes_file)) {
      CLOG(1) << "[KhronosMeshPlugin] Loaded background changes from '"
              << config.background_changes_file << "'.";
    } else if (config.verbosity > 0) {
      LOG(WARNING) << "[KhronosMeshPlugin] Background changes '" << config.background_changes_file
                   << "' are set but could not be loaded.";
    }
  }
  if (!config.object_changes_file.empty()) {
    if (changes_.object_changes.load(config.object_changes_file)) {
      CLOG(1) << "[KhronosMeshPlugin] Loaded object changes from '" << config.object_changes_file
              << "'.";
    } else if (config.verbosity > 0) {
      LOG(WARNING) << "[KhronosMeshPlugin] Object changes '" << config.object_changes_file
                   << "' are set but could not be loaded.";
    }
  }
}

void KhronosMeshPlugin::draw(const std_msgs::Header& /* header */, const DynamicSceneGraph& graph) {
  // Compute time stamps for normalization.
  computeTimingData(graph);

  // Visualize the background.
  publishBackgroundMesh(graph);

  // Visualize the objects.
  publishObjectMesh(graph);
}

void KhronosMeshPlugin::reset(const std_msgs::Header& /*header*/) {
  mesh_visualizer_.resetBackground();
  mesh_visualizer_.resetObjects();
}

void KhronosMeshPlugin::publishBackgroundMesh(const DynamicSceneGraph& dsg) {
  // Nothing to do if there is no mesh.
  if (!dsg.hasMesh() || dsg.mesh()->empty()) {
    return;
  }

  // If the background is not being recolored, just publish the mesh.
  const auto& config = dynamic_config_.get();
  if (config.color_mode == 0 || !config.recolor_background) {
    mesh_visualizer_.drawBackground(dsg);
    return;
  }

  // Publish the colored mesh.
  mesh_visualizer_.drawBackground(dsg, getBackgroundColoring(dsg));
}

void KhronosMeshPlugin::publishObjectMesh(const DynamicSceneGraph& dsg) {
  // Nothing to do if there is no objects.
  if (!dsg.hasLayer(DsgLayers::OBJECTS)) {
    return;
  }

  // If the objects are not being recolored, just publish the mesh.
  const auto& config = dynamic_config_.get();
  if (!config.recolor_objects || config.color_mode == 0) {
    mesh_visualizer_.drawObjects(dsg);
    return;
  }

  // Setup the coloring function.
  std::function<Color(NodeId, const KhronosObjectAttributes&)> coloring_fn;
  if (config.color_mode == 1) {
    // First seen.
    coloring_fn = [this](NodeId, const KhronosObjectAttributes& attrs) {
      return hydra::colorFromTime(time_start_, time_end_, attrs.first_observed_ns.front());
    };
  } else if (config.color_mode == 2) {
    // Last seen.
    coloring_fn = [this](NodeId, const KhronosObjectAttributes& attrs) {
      return hydra::colorFromTime(time_start_, time_end_, attrs.last_observed_ns.front());
    };
  } else if (config.color_mode == 3) {
    // Duration.
    uint64_t max_duration = 0;
    const auto& layer = dsg.getLayer(DsgLayers::OBJECTS);
    for (const auto& [id, node] : layer.nodes()) {
      const auto& attrs = node->attributes<KhronosObjectAttributes>();
      const uint64_t duration = attrs.last_observed_ns.front() - attrs.first_observed_ns.front();
      max_duration = std::max(max_duration, duration);
    }
    coloring_fn = [max_duration](NodeId, const KhronosObjectAttributes& attrs) {
      const uint64_t duration = attrs.last_observed_ns.front() - attrs.first_observed_ns.front();
      return hydra::colorFromTime(0, max_duration, duration);
    };
  } else if (config.color_mode == 4) {
    // Changes (=presence).
    coloring_fn = [this](NodeId, const KhronosObjectAttributes& attrs) {
      if (attrs.first_observed_ns.front() > current_query_time_) {
        return spark_dsg::colormaps::gray(0.7);  // Not yet observed.
      } else if (attrs.last_observed_ns.front() < current_query_time_) {
        return Color::red();  // Absent.
      } else {
        return Color::blue();  // Present.
      }
    };
  } else {
    LOG(ERROR) << "Unknown color mode: " << config.color_mode;
    return;
  }

  // Translate the constant object colors into colorings. Note that the colors need to stay in
  // scope till after the message is passed.
  // TODO(lschmid): Also a candidate to clean up at some point.
  KhronosMeshVisualizer::ObjectColors colorings;
  const auto& layer = dsg.getLayer(DsgLayers::OBJECTS);
  for (const auto& [id, node] : layer.nodes()) {
    const auto& attrs = node->attributes<KhronosObjectAttributes>();
    colorings[id] = coloring_fn(id, attrs);
  }

  // Publish the mesh.
  mesh_visualizer_.drawObjects(dsg, colorings);
}

void KhronosMeshPlugin::computeTimingData(const DynamicSceneGraph& dsg) {
  const auto& config = dynamic_config_.get();

  // Compute the min and max time stamp in the Background, ignoring unitialized values.
  time_start_ = std::numeric_limits<uint64_t>::max();
  time_end_ = 0;
  if (config.recolor_background && dsg.hasMesh()) {
    time_start_ = std::numeric_limits<uint64_t>::max();
    for (const uint64_t first_seen_stamp : dsg.mesh()->first_seen_stamps) {
      if (first_seen_stamp != 0) {
        time_start_ = std::min(time_start_, first_seen_stamp);
      }
    }
    const auto last_seen_stamps = dsg.mesh()->stamps;
    if (!last_seen_stamps.empty()) {
      time_end_ = *std::max_element(last_seen_stamps.begin(), last_seen_stamps.end());
    }
  }

  // Compute the min and max time stamp in the objects, ignoring unitialized values.
  if (config.recolor_objects) {
    const auto& layer = dsg.getLayer(DsgLayers::OBJECTS);
    for (const auto& [id, node] : layer.nodes()) {
      const auto& attrs = node->attributes<KhronosObjectAttributes>();
      if (!attrs.first_observed_ns.empty()) {
        time_start_ = std::min(time_start_, attrs.first_observed_ns.front());
      }
      if (!attrs.last_observed_ns.empty()) {
        time_end_ = std::max(time_end_, attrs.last_observed_ns.front());
      }
    }
  }

  // Query time.
  current_query_time_ =
      time_start_ + (time_end_ - time_start_) * config.object_changes_at_time * 0.01;
}

hydra::MeshColoring::Ptr KhronosMeshPlugin::getBackgroundColoring(
    const DynamicSceneGraph& dsg) const {
  const auto& config = dynamic_config_.get();
  if (config.color_mode == 1) {
    auto coloring = std::make_shared<hydra::FirstSeenMeshColoring>();
    coloring->setBounds(time_start_, time_end_);
    return coloring;
  } else if (config.color_mode == 2) {
    auto coloring = std::make_shared<hydra::LastSeenMeshColoring>();
    coloring->setBounds(time_start_, time_end_);
    return coloring;
  } else if (config.color_mode == 3) {
    auto coloring = std::make_shared<hydra::SeenDurationMeshColoring>();
    auto mesh = dsg.mesh();
    if (mesh) {
      coloring->setMesh(*mesh);
    }
    return coloring;
  } else if (config.color_mode == 4) {
    return std::make_shared<ChangeMeshColoring>(changes_.background_changes);
  }
  if (config.color_mode != 0) {
    LOG(ERROR) << "Unknown color mode: " << dynamic_config_.get().color_mode
               << ", using 0 (color) instead.";
  }
  return nullptr;
}

void KhronosMeshPlugin::changesCallback(const khronos_msgs::Changes& msg) {
  changes_ = fromMsg(msg);
}

Color colorFromChangeState(ChangeState state) {
  switch (state) {
    case ChangeState::kAbsent:
      return Color::red();
    case ChangeState::kPersistent:
      return Color::blue();
    case ChangeState::kUnobserved:
      return spark_dsg::colormaps::gray(0.7);
  }
  return Color::black();
}

Color ChangeMeshColoring::getVertexColor(const Mesh&, size_t i) const {
  return colorFromChangeState(changes_[i]);
}

}  // namespace khronos
