#include "khronos_eval/pipeline_evaluator.h"

#include <filesystem>
#include <iostream>

#include <khronos/utils/output_file_utils.h>
#include <pcl/io/ply_io.h>

namespace khronos {

void declare_config(PipelineEvaluator::ChangeDetectionConfig& config) {
  using namespace config;
  name("ChangeDetectionConfig");
  field(config.name, "name");
  field(config.run_change_detection_incrementally, "run_change_detection_incrementally");
  base<khronos::SequentialChangeDetector::Config>(config);
  checkCondition(!config.name.empty(), "'name' must be set");
}

void declare_config(PipelineEvaluator::ReconciliationConfig& config) {
  using namespace config;
  name("ReconciliationConfig");
  field(config.name, "name");
  base<khronos::Reconciler::Config>(config);
  checkCondition(!config.name.empty(), "'name' must be set");
}

void declare_config(PipelineEvaluator::EvaluationConfig& config) {
  using namespace config;
  name("EvaluationConfig");
  field(config.experiment_directory, "experiment_directory");
  field(config.ground_truth_dsg_file, "ground_truth_dsg_file");
  field(config.ground_truth_background_file, "ground_truth_background_file");
  field(config.ground_truth_dynamic_dsg_file, "ground_truth_dynamic_dsg_file");
  field(config.output_dir, "output_dir");
  if (config.output_dir.empty()) {
    config.output_dir = config.experiment_directory + "/pipeline";
  }
  field(config.results_dir, "results_dir");
  if (config.results_dir.empty()) {
    config.results_dir = config.experiment_directory + "/results";
  }
  field(config.force_recompute, "force_recompute");
  field(config.only_evaluate_final_map, "only_evaluate_final_map");
  field(config.save_reconciled_dsg, "save_reconciled_dsg");
  field(config.run_pipeline, "run_pipeline");
  field(config.run_evaluation, "run_evaluation");
  field(config.save_visualization_detals, "save_visualization_detals");
  field(config.change_detection, "change_detection");
  field(config.reconciliation, "reconciliation");

  NameSpace ns("evaluation");
  field(config.evaluate_mesh, "evaluate_mesh");
  field(config.mesh_evaluation, "mesh_evaluation");

  field(config.evaluate_objects, "evaluate_objects");
  field(config.object_evaluation, "object_evaluation");
  field(config.tmp_strandmon_disappeared_at, "tmp_strandmon_disappeared_at");

  field(config.evaluate_dynamic_objects, "evaluate_dynamic_objects");
  field(config.dynamic_object_evaluation, "dynamic_object_evaluation");

  check<Path::IsFile>(config.ground_truth_dsg_file, "ground_truth_dsg_file");
  check<Path::IsFile>(config.ground_truth_background_file, "ground_truth_background_file");
}

PipelineEvaluator::PipelineEvaluator(const EvaluationConfig& config)
    : config(config::checkValid(config)) {}

void PipelineEvaluator::run() {
  // Setup the intial part.
  if (!configureOutputDirectories()) {
    return;
  }

  hydra::PipelineConfig global_config;
  global_config.store_visualization_details = config.save_visualization_detals;
  hydra::GlobalInfo::reset();
  hydra::GlobalInfo::init(global_config);

  if (config.run_pipeline) {
    // Run all change detectors.
    for (const ChangeDetectionConfig& change_detection_config : config.change_detection) {
      const std::string run_dir = config.output_dir + "/" + change_detection_config.name;
      runChangeDetection(change_detection_config, run_dir);

      // Run all reconcilers.
      for (const ReconciliationConfig& reconciliation_config : config.reconciliation) {
        runReconciliation(reconciliation_config, run_dir, change_detection_config.name);
      }
    }
    log("All change detection and reconciliation runs completed.");
  }

  // Run evaluations.
  if (!config.run_evaluation) {
    return;
  }
  // Setup results directory.
  if (!ensureDirectoryExists(config.results_dir)) {
    LOG(ERROR) << "Failed to create results directory '" << config.results_dir << "'.";
    return;
  }

  setLogger(config.results_dir);
  log("Running evaluations.");
  for (const ChangeDetectionConfig& change_detection_config : config.change_detection) {
    for (const ReconciliationConfig& reconciliation_config : config.reconciliation) {
      runEvaluation(change_detection_config.name, reconciliation_config.name);
    }
  }
  log("All evaluation runs completed.");
}

void PipelineEvaluator::evaluateObjects(const std::string& change_detector_name,
                                        const std::string& reconciler_name,
                                        const std::string& output_dir,
                                        const std::string& vis_dir) {
  const std::string obj_file = output_dir + "/static_objects.csv";
  if (!setupOutputFile(obj_file)) {
    return;
  }

  // Setup data for evaluation. This will be lazily loaded.
  loadGTDsg();
  loadDsgStamps();
  loadReconciledDsgs(change_detector_name, reconciler_name);

  // Evaluate. Note that is vis_dir is set the object evaluator will store condensed visualization
  // output.
  ObjectEvaluator evaluator(config.object_evaluation);
  evaluator.setOutputFile(obj_file, vis_dir);
  evaluator.setGroundTruthDSG(gt_dsg_);
  for (size_t i = 0; i < map_names_.size(); ++i) {
    const std::string& map_name = map_names_[i];
    evaluator.setEvalDSG(reconciled_dsgs_[map_name]);
    for (size_t j = 0; j <= i; ++j) {
      const uint64_t query_time = map_timestamps_[map_names_[j]];
      evaluator.evaluate(map_name, query_time);
    }
    log("Evaluated objects for map '" + map_name + "'.");
  }
  log("Evaluated objects.");
}

void PipelineEvaluator::evaluateDynamicObjects(const std::string& output_dir) {
  const std::string obj_file = output_dir + "/dynamic_objects.csv";
  if (!setupOutputFile(obj_file)) {
    return;
  }

  // Setup data for evaluation. This will be lazily loaded.
  loadGTDsgDynamic();
  loadDsgStamps();
  loadMapData();

  // Evaluate.
  DynamicObjectEvaluator evaluator(config.dynamic_object_evaluation);
  evaluator.setOutputFile(obj_file);
  evaluator.setGroundTruthDSG(gt_dsg_dynamic_);
  for (size_t i = 0; i < map_names_.size(); ++i) {
    const std::string& map_name = map_names_[i];
    evaluator.setEvalDSG(map_data_[map_name].first);
    for (size_t j = 0; j <= i; ++j) {
      const uint64_t query_time = map_timestamps_[map_names_[j]];
      evaluator.evaluate(map_name, query_time);
    }
    log("Evaluated dynamic objects for map '" + map_name + "'.");
  }
  log("Evaluated dynamic objects.");
}

void PipelineEvaluator::evaluateBackground(const std::string& change_detector_name,
                                           const std::string& reconciler_name,
                                           const std::string& output_dir) {
  const std::string bg_file = output_dir + "/background_mesh.csv";
  if (!setupOutputFile(bg_file)) {
    return;
  }

  // Lazy load the ground truth background.
  loadGTBackground();
  loadReconciledDsgs(change_detector_name, reconciler_name);

  // Evaluate the background.
  MeshEvaluator mesh_evaluator(config.mesh_evaluation);
  mesh_evaluator.setGroundTruthCloud(gt_background_);
  for (const auto& [map_name, dsg] : reconciled_dsgs_) {
    mesh_evaluator.setDSG(dsg);
    mesh_evaluator.evaluate(map_name, bg_file);
    log("Evaluated background for map '" + map_name + "'.");
  }
}

void PipelineEvaluator::runEvaluation(const std::string& change_detector_name,
                                      const std::string& reconciler_name) {
  // Setup output directory.
  const std::string run_dir =
      config.results_dir + "/" + change_detector_name + "/" + reconciler_name;
  std::filesystem::create_directories(run_dir);
  const std::string vis_dir = config.experiment_directory + "/eval_visualization/" +
                              change_detector_name + "/" + reconciler_name;
  std::filesystem::create_directories(vis_dir);

  // Clear the DSGs, as these contain the data specific to this evaluation. All relevant data will
  // be loaded lazily.
  reconciled_dsgs_.clear();

  // Run the evaluation.
  // For all robot times and all earlier query times. If force_recompute the directory is already
  // cleared.
  log("Evaluating '" + change_detector_name + "' with '" + reconciler_name + "'.");

  // Evaluate background.
  if (config.evaluate_mesh) {
    evaluateBackground(change_detector_name, reconciler_name, run_dir);
  }

  // Objects.
  if (config.evaluate_objects) {
    evaluateObjects(change_detector_name, reconciler_name, run_dir, vis_dir);
  }
  if (config.evaluate_dynamic_objects) {
    std::string dynamic_run_dir = config.results_dir + "/no_cd/no_recon";
    evaluateDynamicObjects(dynamic_run_dir);
  }

  // Other stuff.
}

void PipelineEvaluator::runReconciliation(const ReconciliationConfig& config,
                                          const std::string& run_dir,
                                          const std::string& change_detector_name) {
  const std::string run_dir2 = run_dir + "/" + config.name;

  // Run all reconcilers.
  if (!setupOutputDirectory(run_dir2)) {
    return;
  }

  // Setup the reconciler.
  log("Running reconciler '" + config.name + "'.");
  loadMapData();
  khronos::Reconciler reconciler(config);

  // Save config.
  std::ofstream config_file(run_dir2 + "/reconciliation_config.txt");
  config_file << config;
  config_file.close();
  std::ofstream outfile(run_dir2 + "/reconciliation_data.csv");
  outfile << "Name,VerticesBefore,FacesBefore,VerticesAfter,FacesAfter" << std::endl;

  if (!changes_data_.count(change_detector_name)) {
    changes_data_[change_detector_name] = {};
  }
  auto& changes_data = changes_data_.at(change_detector_name);

  // Reconcile all maps.
  for (const std::string& map_name : map_names_) {
    // Verify data.
    Changes changes;
    auto it = changes_data.find(map_name);
    if (it != changes_data.end()) {
      changes = it->second;
    } else if (!changes.object_changes.load(run_dir + "/" + map_name + "_object_changes.csv")) {
      log("No object change data for map '" + map_name + "'. Skipping.");
      continue;
    } else if (!changes.background_changes.load(run_dir + "/" + map_name +
                                                "_background_changes.csv")) {
      log("No background change data for map '" + map_name + "'. Skipping.");
      continue;
    }

    const auto dsg = map_data_.at(map_name).first->clone();

    // Reconcile.
    outfile << map_name << "," << dsg->mesh()->numVertices() << "," << dsg->mesh()->numFaces()
            << ",";
    reconciler.reconcile(*dsg, changes, 0);

    // Save DSG, and timing.
    if (config.save_reconciled_dsg) {
      dsg->save(run_dir2 + "/" + map_name + "_reconciled_dsg.sparkdsg");
    }
    hydra::timing::ElapsedTimeRecorder::instance().logStats(run_dir2 + "/" + map_name +
                                                            "_reconciliation_timing.csv");
    hydra::timing::ElapsedTimeRecorder::instance().reset();
    outfile << dsg->mesh()->numVertices() << "," << dsg->mesh()->numVertices() << std::endl;

    // Store the data for later evaluation.
    reconciled_dsg_data_[change_detector_name][config.name][map_name] = dsg;
    log("Reconciled map '" + map_name + "'.");
  }
  outfile.close();
  log("Reconciliation '" + config.name + "' completed successfully.");
}

void PipelineEvaluator::runChangeDetection(const ChangeDetectionConfig& config,
                                           const std::string& run_dir) {
  if (!setupOutputDirectory(run_dir)) {
    return;
  }
  // Lazy loading of data as might not be required.
  loadMapData();

  // Save config and stats.
  log("Running change detection '" + config.name + "'.");
  std::ofstream config_file(run_dir + "/change_detection_config.txt");
  config_file << config;
  config_file.close();

  // Run on all maps.
  for (const std::string& map_name : map_names_) {
    const DynamicSceneGraph::Ptr dsg = map_data_.at(map_name).first;
    const RPGOMerges& rpgo_merges = map_data_.at(map_name).second;

    // Run change detection.
    SequentialChangeDetector change_detector(config);
    change_detector.setDsg(dsg);
    const bool reset_changes = !config.run_change_detection_incrementally;
    const Changes changes = change_detector.detectChanges(rpgo_merges, reset_changes);

    // Write change states to file.
    if (config.objects) {
      changes.object_changes.save(run_dir + "/" + map_name + "_object_changes.csv");
    }
    if (config.background) {
      changes.background_changes.save(run_dir + "/" + map_name + "_background_changes.csv");
    }

    // Save timing.
    hydra::timing::ElapsedTimeRecorder::instance().logStats(run_dir + "/" + map_name +
                                                            "_change_detection_timing.csv");
    hydra::timing::ElapsedTimeRecorder::instance().reset();

    // Keep the changes in memory for later use.
    changes_data_[config.name][map_name] = std::move(changes);
    log("Ran change detection on map '" + map_name + "'.");
  }
  log("Change detection '" + config.name + "' completed successfully.");
}

void PipelineEvaluator::loadDsgStamps() {
  if (!map_timestamps_.empty()) {
    return;
  }
  for (const std::string& map_name : map_names_) {
    const std::string stamp_file =
        config.experiment_directory + "/maps/" + map_name + "/timestamp.txt";
    if (!std::filesystem::exists(stamp_file)) {
      log("Timestamp file '" + stamp_file + "' does not exist, skipping.");
      continue;
    }
    uint64_t value;
    std::ifstream stamp_stream(stamp_file);
    stamp_stream >> value;
    map_timestamps_[map_name] = value;
  }
  log("Loaded " + std::to_string(map_timestamps_.size()) + " map timestamps.");
}

void PipelineEvaluator::loadMapData() {
  // Lazy loading of data as might not be required.
  if (!map_data_.empty()) {
    return;
  }

  log("Loading khronos data.");
  for (auto it = map_names_.begin(); it != map_names_.end();) {
    // Verify all files exist.
    const std::string dsg_file =
        config.experiment_directory + "/maps/" + *it + "/backend/dsg_with_mesh.sparkdsg";

    if (!std::filesystem::exists(dsg_file)) {
      log("DSG file '" + dsg_file + "' does not exist, skipping.");
      it = map_names_.erase(it);
      continue;
    }

    const std::string rpgo_file =
        config.experiment_directory + "/maps/" + *it + "/backend/proposed_merge.csv";
    if (!std::filesystem::exists(rpgo_file)) {
      log("RPGO merge file '" + rpgo_file + "' does not exist, skipping.");
      it = map_names_.erase(it);
      continue;
    }

    const std::string stamp_file = config.experiment_directory + "/maps/" + *it + "/timestamp.txt";
    if (!std::filesystem::exists(stamp_file)) {
      log("Timestamp file '" + stamp_file + "' does not exist, skipping.");
      it = map_names_.erase(it);
      continue;
    }

    // Also load stamp if we're already parsing.
    uint64_t value;
    std::ifstream stamp_stream(stamp_file);
    stamp_stream >> value;
    map_timestamps_[*it] = value;

    // Load files.
    const DynamicSceneGraph::Ptr dsg = DynamicSceneGraph::load(dsg_file);
    const RPGOMerges rpgo_merges = RPGOMerges::fromFile(rpgo_file);
    map_data_[*it] = {dsg, std::move(rpgo_merges)};
    log("Loaded khronos data for map '" + *it + "'.");
    ++it;
  }
  log("Loaded " + std::to_string(map_data_.size()) + " khronos maps.");
}

void PipelineEvaluator::loadGTBackground() {
  if (!gt_background_.empty()) {
    return;
  }
  log("Loading ground truth mesh from '" + config.ground_truth_background_file + "'.");
  if (!std::filesystem::exists(config.ground_truth_background_file)) {
    log("Ground truth file '" + config.ground_truth_background_file + "' does not exist.");
    return;
  }

  // Load the ground truth as arbitrary polygon mesh and extract the vertices or points if it is a
  // cloud afterwards.
  pcl::PLYReader ply_reader;
  pcl::PolygonMesh gt_mesh;
  if (ply_reader.read(config.ground_truth_background_file, gt_mesh) != 0) {
    log("Failed to load ground truth mesh '" + config.ground_truth_background_file + "'.");
    return;
  }

  // Setup the points and neighbor search.
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(gt_mesh.cloud, vertices);
  if (vertices.empty()) {
    log("Error: Ground truth mesh '" + config.ground_truth_background_file + "' has no vertices.");
    return;
  }
  gt_background_.resize(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    gt_background_[i] = vertices[i].getVector3fMap();
  }
}

void PipelineEvaluator::loadGTDsg() {
  if (gt_dsg_) {
    return;
  }

  log("Loading ground truth DSG from '" + config.ground_truth_dsg_file + "'.");
  gt_dsg_ = DynamicSceneGraph::load(config.ground_truth_dsg_file);
  if (!gt_dsg_) {
    log("Failed to load ground truth DSG from '" + config.ground_truth_dsg_file + "'.");
  }
}

void PipelineEvaluator::loadGTDsgDynamic() {
  if (gt_dsg_dynamic_) {
    LOG(ERROR) << "gt_dsg_dynamic_ is already set!";
    return;
  }

  log("Loading ground truth dynamic object DSG from '" + config.ground_truth_dynamic_dsg_file +
      "'.");
  gt_dsg_dynamic_ = DynamicSceneGraph::load(config.ground_truth_dynamic_dsg_file);
  if (!gt_dsg_dynamic_) {
    log("Failed to load ground truth dynamic object DSG from '" +
        config.ground_truth_dynamic_dsg_file + "'.");
  }
}

void PipelineEvaluator::loadReconciledDsgs(const std::string& change_detector_name,
                                           const std::string& reconciler_name) {
  for (const std::string& map_name : map_names_) {
    // Load reconciled DSGs from the already cached data if possible.
    DynamicSceneGraph::Ptr dsg =
        reconciled_dsg_data_[change_detector_name][reconciler_name][map_name];
    if (dsg) {
      reconciled_dsgs_[map_name] = dsg;
      continue;
    }

    // Try to load it.
    const std::string dsg_file = config.output_dir + "/" + change_detector_name + "/" +
                                 reconciler_name + "/" + map_name + "_reconciled_dsg.sparkdsg";
    if (!std::filesystem::exists(dsg_file)) {
      log("Reconciled DSG file '" + dsg_file + "' does not exist, skipping.");
      continue;
    }
    log("Loading reconciled DSG from '" + dsg_file + "'.");
    dsg = DynamicSceneGraph::load(dsg_file);
    if (!dsg) {
      log("Failed to load reconciled DSG file '" + dsg_file + "', skipping.");
      continue;
    }

    // Add it to the result and cache it for later.
    reconciled_dsgs_[map_name] = dsg;
    reconciled_dsg_data_[change_detector_name][reconciler_name][map_name] = dsg;
  }
}

bool PipelineEvaluator::configureOutputDirectories() {
  LOG(INFO) << "\n" << config;

  // Check if the experiment directory exists and is valid.
  if (!khronos::isExperimentDirectory(config.experiment_directory)) {
    return false;
  }

  // Setup output directory.
  if (!ensureDirectoryExists(config.output_dir)) {
    return false;
  }
  setLogger(config.output_dir);

  // Save config.
  std::ofstream config_file(config.output_dir + "/evaluation_config.txt");
  config_file << config;
  config_file.close();

  // Get all map files.
  map_names_ = khronos::getMapNames(config.experiment_directory);
  if (map_names_.empty()) {
    log("No maps to evaluate in '" + config.experiment_directory + "'.");
    return false;
  }
  if (config.only_evaluate_final_map) {
    map_names_ = {map_names_.back()};
  }
  return true;
}

bool PipelineEvaluator::setupOutputDirectory(const std::string& dir_name) const {
  if (std::filesystem::exists(dir_name) && !std::filesystem::is_empty(dir_name)) {
    if (config.force_recompute) {
      log("Force recompute: Clearing existing '" + dir_name + "'.");
      std::filesystem::remove_all(dir_name);
      std::filesystem::create_directories(dir_name);
    } else {
      log("Target directory '" + dir_name + "' already exists. Skipping.");
      return false;
    }
  } else {
    std::filesystem::create_directories(dir_name);
  }
  return true;
}

bool PipelineEvaluator::setupOutputFile(const std::string& file_name) const {
  if (!std::filesystem::exists(file_name)) {
    return true;
  }
  if (config.force_recompute) {
    log("Force recompute: Clearing existing '" + file_name + "'.");
    std::filesystem::remove(file_name);
    return true;
  }
  log("Target file '" + file_name + "' already exists. Skipping.");
  return false;
}

void PipelineEvaluator::setLogger(const std::string& dir_name) {
  logger_ = std::make_shared<khronos::ExperimentLogger>(dir_name);
  logger_->alsoLogToConsole();
}

void PipelineEvaluator::log(const std::string& msg) const {
  if (logger_) {
    logger_->log(msg);
  }
}

}  // namespace khronos
