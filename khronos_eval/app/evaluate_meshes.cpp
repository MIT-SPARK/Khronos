
#include <filesystem>
#include <string>

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>
#include <glog/logging.h>
#include <khronos/common/common_types.h>

#include "khronos_eval/mesh_evaluator.h"

/**
 * @brief Evaluates a number of meshes, writing results to a file in the same directory as the mesh.
 */
int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Check arguments.
  if (argc < 3 || argv[1] == std::string("--help") || argv[1] == std::string("-h")) {
    std::cout << "Usage: " << argv[0] << " config_file ground_truth_file dsg_file1 [dsg_file2, ...]"
              << std::endl;
    return 0;
  }

  // Parse arguments.
  auto config = config::fromYamlFile<khronos::MeshEvaluator::Config>(argv[1]);
  config::checkValid(config);
  const std::string ground_truth_file = argv[2];

  // Run evaluation step by step. These steps issue warnings on their own.
  khronos::MeshEvaluator evaluator(config);
  std::cout << "Evaluating meshes:\n" << config << std::endl;
  std::cout << "Loading ground truth '" << ground_truth_file << std::endl;
  if (!evaluator.loadGroundTruthMesh(ground_truth_file)) {
    return -1;
  }

  // Parse all meshes.
  for (int i = 3; i < argc; ++i) {
    const std::string dsg_file = argv[i];
    const std::string output_file =
        std::filesystem::path(dsg_file).parent_path().string() + "/mesh_metrics.csv";
    const std::string name = std::filesystem::path(dsg_file).filename().stem().string();

    std::cout << "Loading DSG '" << dsg_file << std::endl;
    if (!evaluator.loadDSG(dsg_file)) {
      std::cout << "Failed to load DSG '" << dsg_file << "', skipping." << std::endl;
      continue;
    }

    std::cout << "Evaluating '" << name << "' and writing to '" << output_file << std::endl;
    if (!evaluator.evaluate(name, output_file)) {
      std::cout << "Failed to evaluate '" << name << "', skipping." << std::endl;
      continue;
    }
  }

  LOG(INFO) << "Evaluated " << (argc - 3) << " meshes. Evaluation complete.";
  return 0;
}
