
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
    config.run_pipeline = argv[4] == std::string("true");
  }
  if (argc >= 6) {
    config.run_evaluation = argv[5] == std::string("true");
  }
  if (argc >= 7) {
    config.only_evaluate_final_map = argv[6] == std::string("true");
  }

  config.output_dir = config.experiment_directory + "/pipeline";
  config.results_dir = config.experiment_directory + "/results";

  // Parse config lists with common data.
  // Change detection.
  std::unique_ptr<PipelineEvaluator::ChangeDetectionConfig> common_change;
  for (const auto& run : config.change_detection) {
    if (run.name == "common") {
      common_change = std::make_unique<PipelineEvaluator::ChangeDetectionConfig>(run);
    }
  }
  if (common_change) {
    size_t index = 0;
    for (PipelineEvaluator::ChangeDetectionConfig& run : config.change_detection) {
      const std::string name = run.name;
      if (name == "common") {
        index++;
        continue;
      }
      // Set common values, then overwrite the specific ones per run.
      run = *common_change;
      config::internal::Visitor::setValues(run, config_yaml["change_detection"][index++]);
      run.name = name;
    }

    config.change_detection.erase(
        std::remove_if(config.change_detection.begin(),
                       config.change_detection.end(),
                       [](const PipelineEvaluator::ChangeDetectionConfig& run) {
                         return run.name == "common";
                       }),
        config.change_detection.end());
  }

  // Reconciliation.
  std::unique_ptr<PipelineEvaluator::ReconciliationConfig> common_reconcile;
  for (const auto& run : config.reconciliation) {
    if (run.name == "common") {
      common_reconcile = std::make_unique<PipelineEvaluator::ReconciliationConfig>(run);
    }
  }
  if (common_reconcile) {
    size_t index = 0;
    for (PipelineEvaluator::ReconciliationConfig& run : config.reconciliation) {
      const std::string name = run.name;
      if (name == "common") {
        index++;
        continue;
      }
      // Set common values, then overwrite the specific ones per run.
      run = *common_reconcile;
      config::internal::Visitor::setValues(run, config_yaml["reconciliation"][index++]);
      run.name = name;
    }
    config.reconciliation.erase(
        std::remove_if(config.reconciliation.begin(),
                       config.reconciliation.end(),
                       [](const PipelineEvaluator::ReconciliationConfig& run) {
                         return run.name == "common";
                       }),
        config.reconciliation.end());
  }
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
              << " <config_file> [experiment_directory] [force_recompute] [run_pipeline] "
                 "[run_evaluation] [only_evaluate_final_map]"
              << std::endl;
    return 0;
  }

  // Get and validate conig.
  const khronos::PipelineEvaluator::EvaluationConfig config = khronos::parseConfig(argc, argv);
  khronos::PipelineEvaluator evaluator(config);
  evaluator.run();

  return 0;
}
