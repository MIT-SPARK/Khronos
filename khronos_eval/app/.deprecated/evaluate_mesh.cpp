
#include <filesystem>
#include <string>

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>
#include <glog/logging.h>
#include <khronos/common/common_types.h>

#include "khronos_eval/mesh_evaluator.h"

/**
 * @brief Evaluates a single mesh against the ground truth.
 */
int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Check arguments.
  if (argc < 5 || argc > 6 || argv[1] == std::string("--help") || argv[1] == std::string("-h")) {
    std::cout << "Usage: " << argv[0]
              << " config_file ground_truth_file dsg_file output_file [name]" << std::endl;
    return 0;
  }

  // Parse arguments.
  auto config = config::fromYamlFile<khronos::MeshEvaluator::Config>(argv[1]);
  config::checkValid(config);
  const std::string ground_truth_file = argv[2];
  const std::string dsg_file = argv[3];
  const std::string output_file = argv[4];
  const std::string name = argc == 6 ? argv[5] : "";

  LOG(INFO) << "Evaluating mesh:\n" << config;

  // Run evaluation step by step. These steps issue warnings on their own.
  khronos::MeshEvaluator evaluator(config);

  if (!evaluator.loadGroundTruthMesh(ground_truth_file)) {
    return -1;
  }

  if (!evaluator.loadDSG(dsg_file)) {
    return -1;
  }

  if (!evaluator.evaluate(name, output_file)) {
    return -1;
  }

  LOG(INFO) << "Evaluation complete.";
  return 0;
}
