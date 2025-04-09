
#include <filesystem>
#include <string>

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>
#include <glog/logging.h>
#include <khronos/common/common_types.h>

#include "khronos_eval/ground_truth/tesse_dynamic_object_ground_truth_builder.h"

/**
 * @brief Create Ground truth DSG from tesse simulation data.
 */
int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Check arguments.
  if (argc != 2 || argv[1] == std::string("--help") || argv[1] == std::string("-h")) {
    std::cout << "Usage: " << argv[0] << " config_file" << std::endl;
    return 0;
  }

  // ROS setup
  ros::init(argc, argv, "create_dynamic_object_gt");
  ros::NodeHandle nh;

  // Run
  auto config =
      config::fromYamlFile<khronos::TesseDynamicObjectGroundTruthBuilder::Config>(argv[1]);
  khronos::TesseDynamicObjectGroundTruthBuilder builder(config);
  builder.run();

  return 0;
}
