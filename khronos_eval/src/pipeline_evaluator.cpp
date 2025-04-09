#include "khronos_eval/pipeline_evaluator.h"

#include <filesystem>
#include <iostream>

#include <khronos/spatio_temporal_map/spatio_temporal_map.h>
#include <khronos/utils/output_file_utils.h>
#include <pcl/io/ply_io.h>

namespace khronos {

void declare_config(PipelineEvaluator::EvaluationConfig& config) {
  using namespace config;
  name("EvaluationConfig");
  field(config.experiment_directory, "experiment_directory");
  field(config.ground_truth_dsg_file, "ground_truth_dsg_file");
  field(config.ground_truth_background_file, "ground_truth_background_file");
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
  field(config.run_evaluation, "run_evaluation");
  field(config.store_visualization_details, "store_visualization_details");

  NameSpace ns("evaluation");
  field(config.evaluate_mesh, "evaluate_mesh");
  field(config.mesh_evaluation, "mesh_evaluation");

  field(config.evaluate_objects, "evaluate_objects");
  field(config.object_evaluation, "object_evaluation");

  field(config.evaluate_dynamic_objects, "evaluate_dynamic_objects");
  field(config.dynamic_object_evaluation, "dynamic_object_evaluation");

  check<Path::IsFile>(config.ground_truth_dsg_file, "ground_truth_dsg_file");
  check<Path::IsFile>(config.ground_truth_background_file, "ground_truth_background_file");
}

PipelineEvaluator::PipelineEvaluator(const EvaluationConfig& config)
    : config(config::checkValid(config)) {}

void PipelineEvaluator::run() {
  // Setup the initial part.
  if (!configureOutputDirectories()) {
    return;
  }

  hydra::PipelineConfig global_config;
  global_config.store_visualization_details = config.store_visualization_details;
  hydra::GlobalInfo::reset();
  hydra::GlobalInfo::init(global_config);

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
  runEvaluation();
  log("All evaluation runs completed.");
}

void PipelineEvaluator::evaluateObjects(const std::string& output_dir, const std::string& vis_dir) {
  const std::string obj_file = output_dir + "/static_objects.csv";
  if (!setupOutputFile(obj_file)) {
    return;
  }

  // Setup data for evaluation. This will be lazily loaded.
  loadGTDsg();
  loadSpatioTemporalMap();

  // Evaluate. Note that if vis_dir is set, the object evaluator will store condensed visualization
  // output.
  ObjectEvaluator evaluator(config.object_evaluation);
  evaluator.setOutputFile(obj_file, vis_dir);
  evaluator.setGroundTruthDSG(gt_dsg_);
  for (size_t i = 0; i < reconciled_dsgs_.size(); ++i) {
    evaluator.setEvalDSG(reconciled_dsgs_[i]);
    for (size_t j = 0; j <= i; ++j) {
      const uint64_t query_time = map_timestamps_[i];
      evaluator.evaluate(std::to_string(i), query_time);
    }
    log("Evaluated objects for map '" + std::to_string(i) + "'.");
  }
  log("Evaluated objects.");
}

void PipelineEvaluator::evaluateDynamicObjects(const std::string& output_dir) {
  const std::string obj_file = output_dir + "/dynamic_objects.csv";
  if (!setupOutputFile(obj_file)) {
    return;
  }

  // Setup data for evaluation. This will be lazily loaded.
  loadSpatioTemporalMap();

  // Evaluate.
  DynamicObjectEvaluator evaluator(config.dynamic_object_evaluation);
  evaluator.setOutputFile(obj_file);
  evaluator.setGroundTruthDSG(gt_dsg_);
  for (size_t i = 0; i < reconciled_dsgs_.size(); ++i) {
    evaluator.setEvalDSG(reconciled_dsgs_[i]);
    for (size_t j = 0; j <= i; ++j) {
      const uint64_t query_time = map_timestamps_[i];
      evaluator.evaluate(std::to_string(i), query_time);
    }
    log("Evaluated dynamic objects for map '" + std::to_string(i) + "'.");
  }
  log("Evaluated dynamic objects.");
}

void PipelineEvaluator::evaluateBackground(const std::string& output_dir) {
  const std::string bg_file = output_dir + "/background_mesh.csv";
  if (!setupOutputFile(bg_file)) {
    return;
  }

  // Lazy load the ground truth background.
  loadGTBackground();
  loadSpatioTemporalMap();

  // Evaluate the background.
  MeshEvaluator mesh_evaluator(config.mesh_evaluation);
  mesh_evaluator.setGroundTruthCloud(gt_background_);
  for (size_t i = 0; i < reconciled_dsgs_.size(); ++i) {
    mesh_evaluator.setDSG(reconciled_dsgs_[i]);
    mesh_evaluator.evaluate(std::to_string(i), bg_file);
    log("Evaluated background for map '" + std::to_string(i) + "'.");
  }
}

void PipelineEvaluator::runEvaluation() {
  // Setup output directory.
  const std::string run_dir = config.results_dir;
  std::filesystem::create_directories(run_dir);
  const std::string vis_dir = config.experiment_directory + "/eval_visualization";
  std::filesystem::create_directories(vis_dir);

  // Clear the DSGs, as these contain the data specific to this evaluation. All relevant data will
  // be loaded lazily.
  reconciled_dsgs_.clear();

  // Run the evaluation.
  log("Evaluating.");

  // Evaluate background.
  if (config.evaluate_mesh) {
    evaluateBackground(run_dir);
  }

  // Objects.
  if (config.evaluate_objects) {
    evaluateObjects(run_dir, vis_dir);
  }
  if (config.evaluate_dynamic_objects) {
    std::string dynamic_run_dir = config.results_dir;
    evaluateDynamicObjects(dynamic_run_dir);
  }

  // Save the map timestamps.
  std::ofstream map_timestamps_file(run_dir + "/map_timestamps.txt");
  for (const auto& timestamp : map_timestamps_) {
    map_timestamps_file << timestamp << std::endl;
  }
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

void PipelineEvaluator::loadSpatioTemporalMap() {
  reconciled_dsgs_.clear();
  map_timestamps_.clear();

  const std::string map_file = config.experiment_directory + "/final.4dmap";
  if (!std::filesystem::exists(map_file)) {
    log("4dmap file '" + map_file + "' does not exist.");
    return;
  }

  log("Loading SpatioTemporalMap from '" + map_file + "'.");
  auto spatio_temporal_map = khronos::SpatioTemporalMap::load(map_file);

  // Extract DSGs from the SpatioTemporalMap
  for (const auto& timestamp : spatio_temporal_map->stamps()) {
    const DynamicSceneGraph::Ptr dsg = spatio_temporal_map->getDsgPtr(timestamp);
    reconciled_dsgs_.emplace_back(dsg);
    map_timestamps_.emplace_back(timestamp);
  }
  log("Loaded " + std::to_string(reconciled_dsgs_.size()) + " DSGs from 4dmap.");
}

}  // namespace khronos
