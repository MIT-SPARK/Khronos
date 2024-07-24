
#include <filesystem>
#include <iostream>
#include <string>

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>
#include <khronos/backend/reconciliation/reconciler.h>
#include <khronos/common/common_types.h>
#include <khronos_ros/experiments/experiment_directory.h>
#include <khronos_ros/experiments/experiment_logger.h>

#include "khronos_eval/mesh_evaluator.h"

struct RunConfig : public khronos::Reconciler::Config {
  std::string name;
};

void declare_config(RunConfig& config) {
  using namespace config;
  name("RunConfig");
  field(config.name, "name");
  base<khronos::Reconciler::Config>(config);
}

struct ReconciliationConfig {
  // Directory to load the data from.
  std::string experiment_directory;

  // Output directory to save the data to. If not set defaults to
  // 'experiment_directory/reconciliation'.
  std::string output_dir;

  // Config to use for reconciliation.
  std::vector<RunConfig> runs;

  // If true overwrite existing reconciliation runs.
  bool force_recompute = false;

  // If set, also evaluate the meshes of the reconciled maps.
  std::string ground_truth_file;
};

void declare_config(ReconciliationConfig& config) {
  using namespace config;
  name("ReconciliationConfig");
  field(config.experiment_directory, "experiment_directory");
  field(config.output_dir, "output_dir");
  if (config.output_dir.empty()) {
    config.output_dir = config.experiment_directory + "/reconciliation";
  }
  field(config.force_recompute, "force_recompute");
  field(config.runs, "runs");
  field(config.ground_truth_file, "ground_truth_file");

  checkCondition(config.runs.size() > 0, "No runs specified.");
}

/**
 * @brief reconcile all maps of an experiment directory
 */
int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Check arguments.
  if (argc < 2 || argc > 5 || argv[1] == std::string("--help") || argv[1] == std::string("-h")) {
    std::cout << "Usage: " << argv[0]
              << " <config_file> [experiment_directory] [output_dir] [force_recompute]"
              << std::endl;
    return 0;
  }

  // Load config.
  auto config = config::fromYamlFile<ReconciliationConfig>(argv[1]);
  config::checkValid(config);

  // Overwrite params if requested.
  if (argc >= 3) {
    config.experiment_directory = argv[2];
  }
  if (argc >= 4) {
    config.output_dir = argv[3];
  }
  if (argc >= 5) {
    config.force_recompute = argv[4] == std::string("true");
  }

  // Check if the experiment directory exists and is valid.
  if (!khronos::isExperimentDirectory(config.experiment_directory)) {
    return -1;
  }

  // Setup output directory.
  if (!std::filesystem::exists(config.output_dir)) {
    std::filesystem::create_directories(config.output_dir);
  }
  khronos::ExperimentLogger logger(config.output_dir);
  logger.alsoLogToConsole();

  // Get and all map files.
  std::vector<std::string> map_names = khronos::getMapNames(config.experiment_directory);
  if (map_names.empty()) {
    logger.log("No maps to evaluate in '" + config.experiment_directory + "'.");
    return 0;
  }

  // Load the data.
  logger.log("Loading data.");
  std::map<std::string, std::pair<khronos::DynamicSceneGraph::Ptr, khronos::Changes>> data;
  for (auto it = map_names.begin(); it != map_names.end();) {
    const std::string dsg_file =
        config.experiment_directory + "/maps/" + *it + "/backend/dsg_with_mesh.sparkdsg";
    const std::string changes_file =
        config.experiment_directory + "/maps/" + *it + "/backend/object_changes.csv";
    if (!std::filesystem::exists(dsg_file)) {
      logger.log("DSG file '" + dsg_file + "' does not exist, skipping.");
      it = map_names.erase(it);
      continue;
    }
    if (!std::filesystem::exists(changes_file)) {
      logger.log("Changes file '" + changes_file + "' does not exist, skipping.");
      it = map_names.erase(it);
      continue;
    }
    auto dsg = khronos::DynamicSceneGraph::load(dsg_file);
    khronos::Changes changes;
    if (!changes.object_changes.load(changes_file)) {
      logger.log("Failed to load changes file '" + changes_file + "', skipping.");
      it = map_names.erase(it);
      continue;
    }
    data[*it] = {dsg, changes};
    ++it;
  }

  // Run all reconcilers.
  for (const RunConfig& run : config.runs) {
    // Verify output directory.
    const std::string run_dir = config.output_dir + "/" + run.name;
    if (std::filesystem::exists(run_dir)) {
      if (config.force_recompute) {
        logger.log("Force recompute: Clearing existing '" + run_dir + "'.");
        std::filesystem::remove_all(run_dir);
      } else {
        logger.log("Target directory '" + run.name + "' already exists. Skipping.");
        continue;
      }
    } else {
      std::filesystem::create_directories(run_dir);
    }

    // Setup the reconciler.
    khronos::Reconciler reconciler(run);

    // Save config.
    logger.log("Running reconciler '" + run.name + "'.");
    std::ofstream config_file(run_dir + "/config.txt");
    config_file << run;
    config_file.close();
    std::ofstream outfile(run_dir + "/data.csv");
    outfile << "Name,VerticesBefore,FacesBefore,VerticesAfter,FacesAfter" << std::endl;

    // Reconcile all maps.
    for (const std::string& map_name : map_names) {
      const auto dsg = data[map_name].first;
      const khronos::Changes& changes = data[map_name].second;

      // Reconcile.
      outfile << map_name << "," << dsg->mesh()->numVertices() << "," << dsg->mesh()->numFaces()
              << ",";
      auto map = reconciler.reconcile(*dsg, changes);

      // Save DSG, and timing.
      map.getDSG().save(run_dir + "/" + map_name + "_dsg_with_mesh.sparkdsg");
      hydra::timing::ElapsedTimeRecorder::instance().logStats(run_dir + "/" + map_name +
                                                              "_timing.csv");
      hydra::timing::ElapsedTimeRecorder::instance().reset();
      outfile << map.getDSG().mesh()->numVertices() << "," << map.getDSG().mesh()->numFaces()
              << std::endl;
      logger.log("Reconciled map '" + map_name + "'.");
    }
    outfile.close();
  }
  logger.log("Reconciliation completed successfully.");

  return 0;
}
