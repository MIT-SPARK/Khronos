/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */

// Adapted from Khronos, original notice replicated below:
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

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/colormaps.h>
#include <spark_dsg/node_symbol.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_multi_system/crisp_mesh_plugin.h"
#include "hydra_visualizer/color/color_parsing.h"
#include "hydra_visualizer/drawing.h"

namespace hydra {
namespace {

static const auto registration = config::RegistrationWithConfig<VisualizerPlugin,
                                                                CrispMeshPlugin,
                                                                CrispMeshPlugin::Config,
                                                                ianvs::NodeHandle,
                                                                std::string>("CrispMeshPlugin");

inline std::string node_namespace(spark_dsg::NodeSymbol id) { return "crisp_mesh_" + id.str(true); }

}  // namespace

namespace colormaps = spark_dsg::colormaps;
using spark_dsg::Color;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::KhronosObjectAttributes;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using BaseInterface = rclcpp::node_interfaces::NodeBaseInterface;
using rclcpp::CallbackGroupType;

void declare_config(CrispMeshPlugin::Config& config) {
  using namespace config;
  name("CrispMeshPlugin");
  field(config.min_embedding_diff, "min_embedding_diff");
  field(config.queue_size, "queue_size");
  field(config.layer, "layer");
  field(config.color, "color");
  check(config.min_embedding_diff, GE, 0.0f, "min_embedding_diff");
  check(config.queue_size, GT, 0, "queue_size");
}

CrispMeshPlugin::CrispMeshPlugin(const Config& config,
                                 ianvs::NodeHandle nh,
                                 const std::string& name)
    : VisualizerPlugin(name),
      config(config::checkValid(config)),
      tf_broadcaster_(nh.node()),
      pub_(nh.create_publisher<kimera_pgmo_msgs::msg::Mesh>("crisp_meshes", config.queue_size)),
      color_adapter_(config.color.create()),
      group_(nh.as<BaseInterface>()->create_callback_group(CallbackGroupType::MutuallyExclusive)),
      client_(nh.create_client<ReconstructionSrv>("reconstruct_object",
                                                  rclcpp::ServicesQoS(),
                                                  group_)) {}

void CrispMeshPlugin::draw(const std_msgs::msg::Header& header, const DynamicSceneGraph& graph) {
  if (!graph.hasLayer(config.layer)) {
    return;
  }

  if (pub_->get_subscription_count() == 0) {
    return;
  }

  std::set<spark_dsg::NodeId> curr_nodes;
  const auto& layer = graph.getLayer(config.layer);
  if (color_adapter_) {
    color_adapter_->setGraph(graph, layer.id.layer);
  }

  for (const auto& [node_id, node] : layer.nodes()) {
    const auto attrs = node->tryAttributes<KhronosObjectAttributes>();
    if (!attrs) {
      continue;
    }

    curr_nodes.insert(node_id);
    const auto ns = node_namespace(node_id);
    geometry_msgs::msg::TransformStamped tf;
    tf.header = header;
    tf.child_frame_id = ns;
    tf2::toMsg(attrs->position, tf.transform.translation);
    tf2::convert(attrs->world_R_object, tf.transform.rotation);
    tf_broadcaster_.sendTransform(tf);

    bool need_update = true;
    auto iter = embedding_cache_.find(node_id);
    if (iter == embedding_cache_.end()) {
      iter = embedding_cache_.emplace(node_id, attrs->semantic_feature).first;
    } else {
      // const auto diff = (attrs->semantic_feature - iter->second).norm();
      // need_update = diff > config.min_embedding_diff;
      need_update = false;
    }

    if (!need_update) {
      continue;
    }

    auto req = std::make_shared<ReconstructionSrv::Request>();
    const auto meta = attrs->metadata.get();
    if (meta.contains("scale")) {
      req->scale = meta["scale"].get<float>();
    }

    req->shape_code.insert(req->shape_code.end(),
                           attrs->semantic_feature.reshaped().begin(),
                           attrs->semantic_feature.reshaped().end());

    const auto rep = ianvs::call_service(*client_, req);
    if (!rep) {
      LOG(ERROR) << "CRISP service call failed!";
      continue;
    }

    spark_dsg::Color color;
    if (color_adapter_) {
      color = color_adapter_->getColor(graph, *node);
    }

    VLOG(1) << "Got response of " << rep->vertices.size() << " vertices and "
            << rep->triangles.size() << " faces";

    const auto rgb = visualizer::makeColorMsg(color, 1.0);
    auto msg = std::make_unique<kimera_pgmo_msgs::msg::Mesh>();
    msg->header = header;
    msg->header.frame_id = ns;
    msg->ns = ns;
    msg->vertices.resize(rep->vertices.size() / 3);
    for (size_t i = 0; i + 2 < rep->vertices.size(); i += 3) {
      auto& p = msg->vertices[i / 3];
      p.x = rep->vertices[i];
      p.y = rep->vertices[i + 1];
      p.z = rep->vertices[i + 2];
    }

    msg->triangles.resize(rep->triangles.size() / 3);
    for (size_t i = 0; i + 2 < rep->triangles.size(); i += 3) {
      auto& face = msg->triangles[i / 3];
      face.vertex_indices[0] = rep->triangles[i];
      face.vertex_indices[1] = rep->triangles[i + 1];
      face.vertex_indices[2] = rep->triangles[i + 2];
    }

    msg->vertex_colors = std::vector<std_msgs::msg::ColorRGBA>(msg->vertices.size(), rgb);
    pub_->publish(std::move(msg));
  }

  // Delete objects no longer present.
  auto iter = embedding_cache_.begin();
  while (iter != embedding_cache_.end()) {
    if (curr_nodes.count(iter->first)) {
      ++iter;
    } else {
      kimera_pgmo_msgs::msg::Mesh msg;
      msg.header = header;
      msg.ns = node_namespace(iter->first);
      pub_->publish(msg);
      iter = embedding_cache_.erase(iter);
    }
  }
}

void CrispMeshPlugin::reset(const std_msgs::msg::Header& header) {
  // Reset static meshes.
  for (const auto& [id, vec] : embedding_cache_) {
    kimera_pgmo_msgs::msg::Mesh msg;
    msg.header = header;
    msg.ns = node_namespace(id);
    pub_->publish(msg);
  }

  embedding_cache_.clear();
}

}  // namespace hydra
