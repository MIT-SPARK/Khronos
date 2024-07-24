

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <hydra_ros/visualizer/hydra_visualizer.h>
#include <khronos_ros/visualization/dsg_visualizer_plugins/khronos_mesh_plugin.h>
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
  hydra::HydraVisualizer dsg_vis(nh);
  std::string visualizer_ns;
  nh.param<std::string>("visualizer_ns", visualizer_ns, "~");
  ros::NodeHandle visualizer_nh(visualizer_ns);

  // Add khronos mesh plugin.
  dsg_vis.clearPlugins();
  dsg_vis.addPlugin(std::make_unique<khronos::KhronosMeshPlugin>(visualizer_nh, "mesh"));
  dsg_vis.loadGraph();

  // Change detection visualizer.x
  khronos::ChangeDetectionVisualizer cd_vis(nh, dsg_vis.visualizer_->getGraph());

  // Run.
  dsg_vis.spin();

  return 0;
}
