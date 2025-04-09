#include <filesystem>
#include <string>
#include <iostream>

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>

#include <glog/logging.h>
#include <khronos/common/common_types.h>

#include "khronos_eval/ground_truth/gt_consolidator.h"

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
  auto config = config::fromYamlFile<khronos::GTConsolidator::Config>(argv[1]);
  khronos::GTConsolidator consolidator(config);
  consolidator.run();

  return 0;
}
