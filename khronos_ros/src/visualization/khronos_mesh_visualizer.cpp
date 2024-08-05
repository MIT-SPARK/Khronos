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

#include "khronos_ros/visualization/khronos_mesh_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <hydra_visualizer/color/mesh_color_adaptor.h>
#include <kimera_pgmo_msgs/KimeraPgmoMesh.h>
#include <kimera_pgmo_ros/conversion/ros_conversion.h>

#include "khronos_ros/utils/ros_conversions.h"
#include "khronos_ros/visualization/visualization_utils.h"

namespace hydra {

Eigen::Vector3f pgmoGetVertex(const MeshColorAdaptor& mesh_adaptor,
                              size_t i,
                              kimera_pgmo::traits::VertexTraits* traits) {
  const auto c = mesh_adaptor.getVertexColor(i);
  traits->color = kimera_pgmo::traits::Color{{c.r, c.g, c.b, c.a}};
  return mesh_adaptor.mesh.pos(i);
}

size_t pgmoNumVertices(const MeshColorAdaptor& mesh_adaptor) {
  return pgmoNumVertices(mesh_adaptor.mesh);
}

size_t pgmoNumFaces(const MeshColorAdaptor& mesh_adaptor) {
  return pgmoNumFaces(mesh_adaptor.mesh);
}

kimera_pgmo::traits::Face pgmoGetFace(const MeshColorAdaptor& mesh_adaptor, size_t i) {
  return pgmoGetFace(mesh_adaptor.mesh, i);
}

}  // namespace hydra

namespace khronos {

void declare_config(KhronosMeshVisualizer::Config& config) {
  using namespace config;
  name("KhronosMeshVisualizer");
  field(config.verbosity, "verbosity");
  field(config.queue_size, "background_queue_size");
  field(config.global_frame_name, "global_frame_name");

  check(config.queue_size, GE, 0, "object_queue_size");
  checkCondition(!config.global_frame_name.empty(), "'global_frame_name' can not be empty.");
}

KhronosMeshVisualizer::KhronosMeshVisualizer(const Config& config, const ros::NodeHandle& nh)
    : config(config::checkValid(config)), nh_(nh) {
  // Advertise the publishers.
  pub_ = nh_.advertise<kimera_pgmo_msgs::KimeraPgmoMesh>("mesh", config.queue_size, true);
}

void KhronosMeshVisualizer::draw(const DynamicSceneGraph& dsg,
                                 const Coloring& background_coloring,
                                 const ObjectColors& object_colorings) {
  drawBackground(dsg, background_coloring);
  drawObjects(dsg, object_colorings);
}

void KhronosMeshVisualizer::drawBackground(const DynamicSceneGraph& dsg, const Coloring& coloring) {
  // Nothing to do if there is no mesh.
  if (!dsg.hasMesh() || dsg.mesh()->empty() || pub_.getNumSubscribers() == 0) {
    return;
  }
  std_msgs::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = ros::Time::now();
  publishMesh(header, kBackgroundNs, *dsg.mesh(), coloring);
}

void KhronosMeshVisualizer::drawObjects(const DynamicSceneGraph& dsg,
                                        const ObjectColors& colorings) {
  if (pub_.getNumSubscribers() == 0 || !dsg.hasLayer(DsgLayers::OBJECTS)) {
    return;
  }
  // Setup header.
  std_msgs::Header header;
  header.stamp = ros::Time::now();

  // Visualize all objects.
  std::unordered_set<uint64_t> present_objects;
  const auto& objects = dsg.getLayer(DsgLayers::OBJECTS);
  for (const auto& [node_id, node] : objects.nodes()) {
    const uint64_t id = spark_dsg::NodeSymbol(node_id).categoryId();
    const auto& attrs = node->attributes<KhronosObjectAttributes>();

    if (attrs.mesh.empty()) {
      previous_objects_.erase(id);
      continue;
    }

    // Always update the transform.
    header.frame_id = config.global_frame_name;
    publishTransform(header, attrs, id);
    present_objects.emplace(id);

    Coloring coloring = nullptr;
    const auto it2 = colorings.find(node_id);
    if (it2 != colorings.end()) {
      coloring = std::make_shared<hydra::UniformMeshColoring>(it2->second);
    }
    header.frame_id = getFrameName(id);
    publishMesh(header, getNamespace(id), attrs.mesh, coloring);
  }

  // Delete objects no longer present.
  for (auto it = previous_objects_.begin(); it != previous_objects_.end();) {
    if (!present_objects.count(*it)) {
      clearMesh(getNamespace(*it));
      it = previous_objects_.erase(it);
    } else {
      ++it;
    }
  }
  previous_objects_.insert(present_objects.begin(), present_objects.end());
}

void KhronosMeshVisualizer::reset() {
  resetBackground();
  resetObjects();
}

void KhronosMeshVisualizer::resetBackground() { clearMesh(kBackgroundNs); }

void KhronosMeshVisualizer::resetObjects() {
  for (const auto& id : previous_objects_) {
    clearMesh(getNamespace(id));
  }
  previous_objects_.clear();
}

std::string KhronosMeshVisualizer::getNamespace(const uint64_t id) {
  return "objects/" + std::to_string(NodeSymbol(id));
}

std::string KhronosMeshVisualizer::getFrameName(const uint64_t id) {
  return "khronos_object_" + std::to_string(id);
}

void KhronosMeshVisualizer::clearMesh(const std::string& ns) {
  // An empty message will clear the object in the rviz plugin.
  kimera_pgmo_msgs::KimeraPgmoMesh msg;
  msg.ns = ns;
  pub_.publish(msg);
}

void KhronosMeshVisualizer::publishMesh(const std_msgs::Header& header,
                                        const std::string& ns,
                                        const Mesh& mesh,
                                        const Coloring& coloring) {
  const hydra::MeshColorAdaptor adaptor(mesh, coloring);
  auto msg = kimera_pgmo::conversions::toMsg(adaptor);
  msg.header = header;
  msg.ns = ns;
  pub_.publish(msg);
}

void KhronosMeshVisualizer::publishTransform(const std_msgs::Header& header,
                                             const KhronosObjectAttributes& attrs,
                                             const uint64_t id) {
  geometry_msgs::TransformStamped msg;
  msg.header = header;
  msg.child_frame_id = getFrameName(id);
  // The khronos meshes are in bounding box coordinates.
  msg.transform.translation.x = attrs.bounding_box.world_P_center.x();
  msg.transform.translation.y = attrs.bounding_box.world_P_center.y();
  msg.transform.translation.z = attrs.bounding_box.world_P_center.z();
  const Eigen::Quaternionf quat(attrs.bounding_box.world_R_center);
  msg.transform.rotation.w = quat.w();
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  tf_broadcaster_.sendTransform(msg);
}

}  // namespace khronos
