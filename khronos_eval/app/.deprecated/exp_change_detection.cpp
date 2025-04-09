
#include <filesystem>
#include <iostream>
#include <string>

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>
#include <khronos/backend/change_detection/sequential_change_detector.h>
#include <khronos/common/common_types.h>
#include <khronos_ros/experiments/experiment_directory.h>
#include <khronos_ros/experiments/experiment_logger.h>

struct RunConfig : public khronos::SequentialChangeDetector::Config {
  std::string name;
};

void declare_config(RunConfig& config) {
  using namespace config;
  name("RunConfig");
  field(config.name, "name");
  base<khronos::SequentialChangeDetector::Config>(config);
}

struct ChangeDetectionConfig {
  // Directory to load the data from.
  std::string experiment_directory;

  // Output directory to save the data to. If not set defaults to
  // 'experiment_directory/change_detection'.
  std::string output_dir;

  // Config to use for change_detection.
  std::vector<RunConfig> runs;

  // If true overwrite existing change_detection runs.
  bool force_recompute = false;

  // If true only evaluate the final map. If false evaluate all maps.
  bool only_evaluate_final_map = false;
};

void declare_config(ChangeDetectionConfig& config) {
  using namespace config;
  name("ChangeDetectionConfig");
  field(config.experiment_directory, "experiment_directory");
  field(config.output_dir, "output_dir");
  if (config.output_dir.empty()) {
    config.output_dir = config.experiment_directory + "/change_detection";
  }
  field(config.force_recompute, "force_recompute");
  field(config.runs, "runs");
  field(config.only_evaluate_final_map, "only_evaluate_final_map");

  checkCondition(config.runs.size() > 0, "No runs specified.");
}

/**
 * @brief Run change detection on an experiment directory.
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

  // Load config and parse common values to config.
  YAML::Node config_yaml = YAML::LoadFile(argv[1]);
  auto config = config::fromYaml<ChangeDetectionConfig>(config_yaml);
  const RunConfig common = config::fromYaml<RunConfig>(config_yaml, "common");
  size_t index = 0;
  for (RunConfig& run : config.runs) {
    const std::string name = run.name;
    // Set common values, then overwrite the specific ones per run.
    run = common;
    config::internal::Visitor::setValues(run, config_yaml["runs"][index++]);
    run.name = name;
  }

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

  // Validate conig.
  config::checkValid(config);
  LOG(INFO) << "\n" << config;

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
  if (config.only_evaluate_final_map) {
    map_names = {map_names.back()};
  }

  // Load the data.
  logger.log("Loading data.");
  std::map<std::string, std::pair<khronos::DynamicSceneGraph::Ptr, khronos::RPGOMerges>> data;
  for (auto it = map_names.begin(); it != map_names.end();) {
    const std::string dsg_file =
        config.experiment_directory + "/maps/" + *it + "/backend/dsg_with_mesh.sparkdsg";
    const std::string rpgo_file =
        config.experiment_directory + "/maps/" + *it + "/backend/proposed_merge.csv";
    if (!std::filesystem::exists(dsg_file)) {
      logger.log("DSG file '" + dsg_file + "' does not exist, skipping.");
      it = map_names.erase(it);
      continue;
    }
    if (!std::filesystem::exists(rpgo_file)) {
      logger.log("RPGO merge file '" + rpgo_file + "' does not exist, skipping.");
      it = map_names.erase(it);
      continue;
    }
    khronos::DynamicSceneGraph::Ptr dsg = khronos::DynamicSceneGraph::load(dsg_file);
    auto rpgo_merges = khronos::RPGOMerges::fromFile(rpgo_file);
    data[*it] = {dsg, std::move(rpgo_merges)};
    ++it;
  }

  // Run all change detectors.
  for (const RunConfig& run : config.runs) {
    // Verify output directory.
    const std::string run_dir = config.output_dir + "/" + run.name;
    if (std::filesystem::exists(run_dir)) {
      if (config.force_recompute) {
        logger.log("Force recompute: Clearing existing '" + run_dir + "'.");
        std::filesystem::remove_all(run_dir);
        std::filesystem::create_directories(run_dir);
      } else {
        logger.log("Target directory '" + run.name + "' already exists. Skipping.");
        continue;
      }
    } else {
      std::filesystem::create_directories(run_dir);
    }

    // Setup the change detector.
    khronos::SequentialChangeDetector change_detector(run);

    // Save config and stats.
    logger.log("Running change detection '" + run.name + "'.");
    std::ofstream config_file(run_dir + "/config.txt");
    config_file << run;
    config_file.close();
    std::ofstream outfile(run_dir + "/data.csv");
    outfile << "Name,BGNumAbsent,BGNumPresent,BGNumUnobserved" << std::endl;

    // Run on all maps.
    for (const std::string& map_name : map_names) {
      const std::string dsg_file =
          config.experiment_directory + "/maps/" + map_name + "/backend/dsg_with_mesh.sparkdsg";
      const khronos::DynamicSceneGraph::Ptr dsg = data[map_name].first;
      const khronos::RPGOMerges& rpgo_merges = data[map_name].second;
      change_detector.setDsg(dsg);
      khronos::Changes change_state = change_detector.detectChanges(rpgo_merges, true);

      // Write object change state to file.
      if (run.objects) {
        change_state.object_changes.save(run_dir + "/" + map_name + "_object_changes.csv");
      }

      // Recolor the DSG and count change_states.
      if (run.background) {
        auto& vertices = dsg->mesh()->colors;
        size_t num_absent = 0;
        size_t num_persistent = 0;
        size_t num_unobserved = 0;

        for (size_t i = 0; i < vertices.size(); ++i) {
          auto& vertex = vertices[i];
          switch (change_state.background_changes[i]) {
            case (khronos::ChangeState::kAbsent): {
              vertex.r = 255;
              vertex.g = 0;
              vertex.b = 0;
              ++num_absent;
              break;
            }
            case (khronos::ChangeState::kPersistent): {
              vertex.r = 0;
              vertex.g = 0;
              vertex.b = 255;
              ++num_persistent;
              break;
            }
            case (khronos::ChangeState::kUnobserved): {
              vertex.r = 128;
              vertex.g = 128;
              vertex.b = 128;
              ++num_unobserved;
              break;
            }
          }
        }

        outfile << map_name << "," << num_absent << "," << num_persistent << "," << num_unobserved
                << std::endl;

        // Save DSG, and timing.
        dsg->save(run_dir + "/" + map_name + "_dsg_recolored.sparkdsg");
      }
      hydra::timing::ElapsedTimeRecorder::instance().logStats(run_dir + "/" + map_name +
                                                              "_timing.csv");
      hydra::timing::ElapsedTimeRecorder::instance().reset();
      logger.log("Ran change detection on map '" + map_name + "'.");
    }
    outfile.close();
  }
  logger.log("Change detection completed successfully.");

  return 0;
}
