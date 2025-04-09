
#include <filesystem>
#include <string>

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>
#include <glog/logging.h>
#include <khronos/common/common_types.h>
#include <khronos_ros/experiments/experiment_directory.h>
#include <khronos_ros/experiments/experiment_logger.h>

#include "khronos_eval/mesh_evaluator.h"

/**
 * @brief Evaluates all meshes in an experiment directory.
 */
int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Check arguments.
  if (argc < 4 || argc > 5 || argv[1] == std::string("--help") || argv[1] == std::string("-h")) {
    std::cout << "Usage: " << argv[0]
              << " config_file experiment_directory ground_truth_file [force_recompute]"
              << std::endl;
    return 0;
  }

  // Parse arguments.
  auto config = config::fromYamlFile<khronos::MeshEvaluator::Config>(argv[1]);
  config::checkValid(config);
  const bool force_recompute = argc == 5 && argv[4] == std::string("true");
  const std::string experiment_directory = argv[2];
  const std::string ground_truth_file = argv[3];

  // Fixed names of experiment directory structure to parse.
  const std::string result_dir_name = "metrics";
  const std::string result_file_name = "mesh.csv";
  const std::string map_file_names = "backend/dsg_with_mesh.sparkdsg";

  // Check if the experiment directory exists and is valid.
  if (!khronos::isExperimentDirectory(experiment_directory)) {
    return -1;
  }

  // Setup experiment logging.
  khronos::ExperimentLogger logger(experiment_directory);
  if (logger.hasFlag("Mesh Evaluated") && !force_recompute) {
    LOG(INFO) << "Meshes for '" << experiment_directory << "' already evaluated.";
    return 0;
  }

  // Get and all map files.
  std::vector<std::string> map_dirs = khronos::getMapDirectories(experiment_directory);
  if (map_dirs.empty()) {
    LOG(INFO) << "No maps to evaluate in '" << experiment_directory << "'.";
    return 0;
  }

  // Load the ground truth mesh.
  khronos::MeshEvaluator evaluator(config);
  if (!evaluator.loadGroundTruthMesh(ground_truth_file)) {
    return 1;
  }
  logger.log("Evaluating meshes.");
  LOG(INFO) << "Evaluating meshes\n" << config;

  // Verify all map files exist.
  for (auto it = map_dirs.begin(); it != map_dirs.end();) {
    const std::string full_file_name = experiment_directory + "/" + *it + "/" + map_file_names;
    if (!std::filesystem::exists(full_file_name)) {
      const std::string msg =
          "Map file '" + *it + "/" + map_file_names + "' does not exist, skipping.";
      LOG(WARNING) << msg;
      logger.log(msg);
      it = map_dirs.erase(it);
    } else {
      ++it;
    }
  }
  if (map_dirs.empty()) {
    LOG(INFO) << "No maps to evaluate in '" << experiment_directory << "'.";
    logger.log("Evaluation finished: No maps to evaluate in '" + experiment_directory + "'.");
    return 1;
  }

  // Setup output directory.
  const std::string output_file =
      experiment_directory + "/" + result_dir_name + "/" + result_file_name;

  if (std::filesystem::exists(experiment_directory + "/" + result_dir_name)) {
    if (std::filesystem::exists(output_file)) {
      // File already exists so force_recompute is set.
      std::filesystem::remove(output_file);
      const std::string msg =
          "Force recompute: Removing existing '" + result_dir_name + "/" + result_file_name + "'.";
      LOG(INFO) << msg;
      logger.log(msg);
    }
  } else {
    std::filesystem::create_directory(experiment_directory + "/" + result_dir_name);
  }

  // Evaluate all maps.
  for (const auto& map_dir : map_dirs) {
    if (!evaluator.loadDSG(experiment_directory + "/" + map_dir + "/" + map_file_names)) {
      logger.log("Evaluation '" + map_dir + "' failed: Could not load DSG.");
      continue;
    }

    if (evaluator.evaluate(map_dir, output_file)) {
      logger.log("Evaluation '" + map_dir + "' completed successfully.");
    } else {
      logger.log("Evaluation '" + map_dir + "' failed: Evaluation failed.");
    }
  }

  // Verify all data was written to the results.
  if (!std::filesystem::exists(output_file)) {
    const std::string msg =
        "Mesh evaluation completed unsuccessfully: No output file was generated.";
    LOG(ERROR) << msg;
    logger.log(msg);
    return 1;
  }
  std::ifstream result_file(output_file);
  std::string line;
  int lines;
  for (lines = 0; std::getline(result_file, line); lines++) {
  }
  if (lines == 0) {
    const std::string msg =
        "Mesh evaluation completed unsuccessfully: No data was written to the output file.";
    LOG(ERROR) << msg;
    logger.log(msg);
    return 1;
  }

  // Success.
  const std::string msg = "Mesh evaluation completed successfully: Wrote " + std::to_string(lines) +
                          " lines to '" + result_dir_name + "/" + result_file_name + "'.";
  logger.log(msg);
  LOG(INFO) << msg;
  logger.setFlag("Mesh Evaluated");

  return 0;
}
