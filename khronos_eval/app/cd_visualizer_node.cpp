#include <config_utilities/parsing/ros.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <hydra_visualizer/dsg_visualizer.h>
#include <hydra_visualizer/plugins/mesh_plugin.h>
#include <ros/ros.h>

#include "khronos_eval/cd_visualizer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cd_visualizer_node");

  // Setup logging
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Setup DSG vis node.
  ros::NodeHandle nh("~");
  const auto dsg_config = config::fromRos<hydra::DsgVisualizer::Config>(nh);

  // Change detection visualizer.
  auto graph_wrapper = dsg_config.graph.create();
  auto stamped_graph = graph_wrapper->get();
  if (stamped_graph) {
    auto dsg = stamped_graph.graph;
    khronos::ChangeDetectionVisualizer cd_vis(nh, dsg);
  } else {
    LOG(ERROR) << "No graph available";
  }

  return 0;
}
