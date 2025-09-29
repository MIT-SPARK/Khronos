#include <config_utilities/parsing/context.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "khronos_eval/eval_visualizer.h"

int main(int argc, char** argv) {
  config::initContext(argc, argv, true);
  rclcpp::init(argc, argv);

  // Setup logging
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Start Ros.
  auto node = std::make_shared<rclcpp::Node>("dsg_visualizer_node");
  ianvs::NodeHandle nh(*node);

  const auto config = config::fromContext<khronos::EvalVisualizer::Config>();
  khronos::EvalVisualizer viz(config, nh);
  viz.start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
