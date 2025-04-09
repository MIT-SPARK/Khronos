

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "khronos_eval/eval_visualizer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dsg_visualizer_node");

  // Setup logging
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Setup node.
  ros::NodeHandle nh("~");
  khronos::EvalVisualizer node(nh);
  node.spin();

  return 0;
}
