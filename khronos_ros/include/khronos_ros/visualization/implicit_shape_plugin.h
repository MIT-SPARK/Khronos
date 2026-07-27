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

#pragma once

#include <map>
#include <string>

#include <Eigen/Dense>
#include <hydra_visualizer/adapters/node_color.h>
#include <hydra_visualizer/plugins/visualizer_plugin.h>
#include <ianvs/node_handle.h>
#include <khronos_msgs/srv/reconstruct_implicit_object.hpp>
#include <kimera_pgmo_msgs/msg/mesh.hpp>
#include <rclcpp/publisher.hpp>
#include <spark_dsg/scene_graph.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hydra {

//! @brief Plugin to render and visualize crisp object meshes based on shape codes
class ImplicitShapePlugin : public VisualizerPlugin {
 public:
  using ReconstructionSrv = khronos_msgs::srv::ReconstructImplicitObject;

  struct Config {
    //! Minimum difference in embedding before requesting new mesh
    float min_embedding_diff = 0.0f;
    //! Queue size for publishing.
    int queue_size = 100;
    //! Layer to draw objects for
    std::string layer = spark_dsg::DsgLayers::OBJECTS;
    //! Color per mesh
    config::VirtualConfig<NodeColorAdapter> color{LabelColorAdapter::Config()};
  } const config;

  ImplicitShapePlugin(const Config& config, ianvs::NodeHandle nh, const std::string& name);

  void draw(const std_msgs::msg::Header& header, const spark_dsg::SceneGraph& graph) override;

  void reset(const std_msgs::msg::Header& header) override;

 private:
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Publisher<kimera_pgmo_msgs::msg::Mesh>::SharedPtr pub_;
  std::map<spark_dsg::NodeId, Eigen::VectorXf> embedding_cache_;
  NodeColorAdapter::Ptr color_adapter_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Client<ReconstructionSrv>::SharedPtr client_;
};

void declare_config(ImplicitShapePlugin::Config& config);

}  // namespace hydra
