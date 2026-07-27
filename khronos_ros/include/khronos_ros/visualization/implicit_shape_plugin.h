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
