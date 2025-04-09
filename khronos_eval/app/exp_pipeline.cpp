#include <filesystem>
#include <iostream>
#include <string>

#include <config_utilities/parsing/yaml.h>

#include "khronos_eval/pipeline_evaluator.h"

namespace khronos {

PipelineEvaluator::EvaluationConfig parseConfig(int argc, char** argv) {
  // Load config.
  YAML::Node config_yaml = YAML::LoadFile(argv[1]);
  auto config = config::fromYaml<khronos::PipelineEvaluator::EvaluationConfig>(config_yaml);

  // Overwrite params if requested.
  if (argc >= 3) {
    config.experiment_directory = argv[2];
  }
  if (argc >= 4) {
    config.force_recompute = argv[3] == std::string("true");
  }
  if (argc >= 5) {
    config.run_evaluation = argv[4] == std::string("true");
  }
  if (argc >= 6) {
    config.only_evaluate_final_map = argv[5] == std::string("true");
  }

  config.output_dir = config.experiment_directory + "/pipeline";
  config.results_dir = config.experiment_directory + "/results";

  return config;
}

}  // namespace khronos

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Check arguments.
  if (argc < 2 || argc > 7 || argv[1] == std::string("--help") || argv[1] == std::string("-h")) {
    std::cout << "Usage: " << argv[0]
              << " <config_file> [experiment_directory] [force_recompute] [run_evaluation] "
                 "[only_evaluate_final_map]"
              << std::endl;
    return 0;
  }

  // Get and validate config.
  const khronos::PipelineEvaluator::EvaluationConfig config = khronos::parseConfig(argc, argv);
  khronos::PipelineEvaluator evaluator(config);
  evaluator.run();

  return 0;
}
