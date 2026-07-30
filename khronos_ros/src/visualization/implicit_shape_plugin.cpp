#include "khronos_ros/visualization/implicit_shape_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/colormaps.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/color_parsing.h"

namespace hydra {
namespace {

static const auto registration = config::RegistrationWithConfig<VisualizerPlugin,
                                                                ImplicitShapePlugin,
                                                                ImplicitShapePlugin::Config,
                                                                ianvs::NodeHandle,
                                                                std::string>("ImplicitShapePlugin");

inline std::string node_namespace(spark_dsg::NodeSymbol id) { return "crisp_mesh_" + id.str(true); }

void fillRequest(const spark_dsg::KhronosObjectAttributes& attrs,
                 khronos_msgs::srv::ReconstructImplicitObject::Request& req) {
  const auto meta = attrs.metadata.get();
  if (meta.contains("scale")) {
    req.scale = meta["scale"].get<float>();
  }

  req.shape_code.insert(req.shape_code.end(),
                        attrs.semantic_feature.reshaped().begin(),
                        attrs.semantic_feature.reshaped().end());
}

void fillMesh(const std_msgs::msg::Header& header,
              const khronos_msgs::srv::ReconstructImplicitObject::Response& response,
              const std::string& ns,
              const spark_dsg::Color& color,
              kimera_pgmo_msgs::msg::Mesh& msg) {
  const auto rgb = visualizer::makeColorMsg(color, 1.0);
  msg.header = header;
  msg.header.frame_id = ns;
  msg.ns = ns;
  msg.vertices.resize(response.vertices.size() / 3);
  for (size_t i = 0; i + 2 < response.vertices.size(); i += 3) {
    auto& v = msg.vertices[i / 3];
    v.pos.x = response.vertices[i];
    v.pos.y = response.vertices[i + 1];
    v.pos.z = response.vertices[i + 2];
    v.has_color = true;
    v.color = rgb;
  }

  msg.triangles.resize(response.triangles.size() / 3);
  for (size_t i = 0; i + 2 < response.triangles.size(); i += 3) {
    auto& face = msg.triangles[i / 3];
    face.vertex_indices[0] = response.triangles[i];
    face.vertex_indices[1] = response.triangles[i + 1];
    face.vertex_indices[2] = response.triangles[i + 2];
  }
}

}  // namespace

using spark_dsg::KhronosObjectAttributes;
using spark_dsg::SceneGraph;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using BaseInterface = rclcpp::node_interfaces::NodeBaseInterface;
using rclcpp::CallbackGroupType;

void declare_config(ImplicitShapePlugin::Config& config) {
  using namespace config;
  name("ImplicitShapePlugin");
  field(config.min_embedding_diff, "min_embedding_diff");
  field(config.queue_size, "queue_size");
  field(config.layer, "layer");
  field(config.color, "color");
  check(config.min_embedding_diff, GE, 0.0f, "min_embedding_diff");
  check(config.queue_size, GT, 0, "queue_size");
}

ImplicitShapePlugin::ImplicitShapePlugin(const Config& config,
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

void ImplicitShapePlugin::draw(const std_msgs::msg::Header& header, const SceneGraph& graph) {
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
    if (!attrs || !attrs->semantic_feature.size()) {
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
    fillRequest(*attrs, *req);
    const auto rep = ianvs::call_service(*client_, req);
    if (!rep) {
      LOG(ERROR) << "CRISP service call failed!";
      continue;
    }

    spark_dsg::Color color;
    if (color_adapter_) {
      color = color_adapter_->getColor(graph, *node);
    }

    const auto num_verts = rep->vertices.size();
    const auto num_faces = rep->triangles.size();
    VLOG(1) << "Got " << num_verts << " vertices and " << num_faces << " faces";

    auto msg = std::make_unique<kimera_pgmo_msgs::msg::Mesh>();
    fillMesh(header, *rep, ns, color, *msg);
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

void ImplicitShapePlugin::reset(const std_msgs::msg::Header& header) {
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
