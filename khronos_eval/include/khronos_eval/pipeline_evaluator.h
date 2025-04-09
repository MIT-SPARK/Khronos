#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <config_utilities/types/path.h>
#include <khronos/backend/reconciliation/reconciler.h>
#include <khronos/common/common_types.h>
#include <khronos/utils/output_file_utils.h>
#include <khronos_ros/experiments/experiment_directory.h>
#include <khronos_ros/experiments/experiment_logger.h>

#include "khronos_eval/dynamic_object_evaluator.h"
#include "khronos_eval/mesh_evaluator.h"
#include "khronos_eval/object_evaluator.h"

namespace khronos {

/**
 * @brief Class that handles complete processing of a khronos output directory. Run the full khronos
 * pipeline (change detection + reconciliation + evaluation) on an experiment directory. Allows
 * specifying "grid search" over change detectors and reconcilers. Re-uses data from memory where
 * possible to avoid long serialization times, but also supports loading from disk for already
 * processed data.
 * @note This script will eventually replace all current exp scripts that focus on single aspects.
 */

class PipelineEvaluator {
 public:
  struct EvaluationConfig {
    // ========== Which data to parse and store ==========
    // Directory to load the data from.
    std::string experiment_directory;

    // Ground truth dsg without mesh to perform evaluation against.
    std::string ground_truth_dsg_file;

    // Ground truth background pointcloud to perform evaluation against.
    std::string ground_truth_background_file;

    // Output directory to save the change and reconciliation data to. If not set defaults to
    // 'experiment_directory/pipeline'.
    std::string output_dir;

    // Output directory to save the results to. If not set defaults to 'output_dir/results'.
    std::string results_dir;

    // ========== Evaluation settings ==========
    // If true run the pipeline and then specified evaluations. If false only run evaluations.
    bool run_pipeline = true;

    // If true run the evaluations. If false only run the pipeline.
    bool run_evaluation = true;

    // If true overwrite existing change_detection runs.
    bool force_recompute = false;

    // If true only evaluate the final map. If false evaluate all maps.
    bool only_evaluate_final_map = false;

    // Whether to save the reconciled DSGs (takes up some disk space).
    bool save_reconciled_dsg = true;

    // Whether to save additional details for visualization.
    bool store_visualization_details = false;

    // ========== Which evaluations to perform. ==========
    bool evaluate_mesh = true;
    MeshEvaluator::Config mesh_evaluation;

    bool evaluate_objects = true;
    ObjectEvaluator::Config object_evaluation;

    bool evaluate_dynamic_objects = true;
    DynamicObjectEvaluator::Config dynamic_object_evaluation;
  };

  // Construction.
  explicit PipelineEvaluator(const EvaluationConfig& config);
  virtual ~PipelineEvaluator() = default;

  // Run the evaluation.
  void run();

 protected:
  // Processing.
  bool configureOutputDirectories();
  void loadGTBackground();
  void loadGTDsg();
  void loadDsgStamps();
  void loadSpatioTemporalMap();
  void runEvaluation();
  void evaluateBackground(const std::string& output_dir);
  void evaluateObjects(const std::string& output_dir, const std::string& vis_dir);
  void evaluateDynamicObjects(const std::string& output_dir);

  // Utility.
  bool setupOutputDirectory(const std::string& dir_name) const;
  bool setupOutputFile(const std::string& file_name) const;
  void setLogger(const std::string& dir_name);
  void log(const std::string& msg) const;

 private:
  bool saveObjectPresenceTimesToFile(
      const DynamicSceneGraph::Ptr& dsg,
      const std::map<NodeId, Reconciler::ObjectReconciliationDetail>& object_details,
      const std::string& filename) const;

  const EvaluationConfig config;
  ExperimentLogger::Ptr logger_;
  std::vector<uint64_t> map_timestamps_;
  std::vector<DynamicSceneGraph::Ptr> reconciled_dsgs_;
  DynamicSceneGraph::Ptr gt_dsg_;
  Points gt_background_;
};

void declare_config(PipelineEvaluator::EvaluationConfig& config);

}  // namespace khronos
