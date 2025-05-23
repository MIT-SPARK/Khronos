
#include <filesystem>
#include <string>
#include <iostream>

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>
#include <glog/logging.h>
#include <khronos/common/common_types.h>

#include "khronos_eval/ground_truth/tesse_ground_truth_builder.h"

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

  // Run
  auto config = config::fromYamlFile<khronos::TesseGroundTruthBuilder::Config>(argv[1]);
  khronos::TesseGroundTruthBuilder builder(config);
  builder.run();

  return 0;
}
