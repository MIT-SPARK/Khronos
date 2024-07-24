#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <config_utilities/types/path.h>
#include <khronos/backend/change_detection/sequential_change_detector.h>
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
  // Configs.
  struct ChangeDetectionConfig : public khronos::SequentialChangeDetector::Config {
    std::string name;

    // If true, run change detection incrementally always based on the previous map. Note that this
    // ignores any loopclosures of the online system. If false, run change detection from scratch on
    // every map.
    bool run_change_detection_incrementally = false;
  };

  struct ReconciliationConfig : public khronos::Reconciler::Config {
    std::string name;
  };

  struct EvaluationConfig {
    // ========== Which data to parse and store ==========
    // Directory to load the data from.
    std::string experiment_directory;

    // Ground truth dsg without mesh to perform evaluation against.
    std::string ground_truth_dsg_file;

    // Ground truth background pointcloud to perform evaluation against.
    std::string ground_truth_background_file;

    // Ground truth dynamic object dsg file.
    std::string ground_truth_dynamic_dsg_file;

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
    bool save_visualization_detals = false;

    // ========== Which combinations of modules to run ==========
    // Which change detectors to run. Leave empty to skip change detection. Use a config named
    // 'common' to set values for all detectors overriden by the values specifically set.
    std::vector<ChangeDetectionConfig> change_detection;

    // Which reconcilers to run. Leave empty to skip reconciliation. Use a config named 'common' to
    // set values for all reconcilers overriden by the values specifically set.
    std::vector<ReconciliationConfig> reconciliation;

    // ========== Which evaluations to perform. ==========
    bool evaluate_mesh = true;
    MeshEvaluator::Config mesh_evaluation;

    bool evaluate_objects = true;
    ObjectEvaluator::Config object_evaluation;

    bool evaluate_dynamic_objects = true;
    DynamicObjectEvaluator::Config dynamic_object_evaluation;

    int tmp_strandmon_disappeared_at = 8;  // WHen the chair disappeared. [5-11]
  };

  // Construction.
  explicit PipelineEvaluator(const EvaluationConfig& config);
  virtual ~PipelineEvaluator() = default;

  // Run the evaluation.
  void run();

 protected:
  // Processing.
  bool configureOutputDirectories();
  void loadMapData();
  void loadGTBackground();
  void loadGTDsg();
  void loadGTDsgDynamic();
  void loadDsgStamps();
  void loadReconciledDsgs(const std::string& change_detector_name,
                          const std::string& reconciler_name);
  void runChangeDetection(const ChangeDetectionConfig& config, const std::string& run_dir);
  void runReconciliation(const ReconciliationConfig& config,
                         const std::string& run_dir,
                         const std::string& change_detector_name);
  void runEvaluation(const std::string& change_detector_name, const std::string& reconciler_name);
  void evaluateBackground(const std::string& change_detector_name,
                          const std::string& reconciler_name,
                          const std::string& output_dir);
  void evaluateObjects(const std::string& change_detector_name,
                       const std::string& reconciler_name,
                       const std::string& output_dir,
                       const std::string& vis_dir);
  void evaluateDynamicObjects(const std::string& output_dir);

  // Utility.
  bool setupOutputDirectory(const std::string& dir_name) const;
  // Check if file exists and clear if should be overwritten.
  bool setupOutputFile(const std::string& file_name) const;

  void setLogger(const std::string& dir_name);
  void log(const std::string& msg) const;

 private:
  const EvaluationConfig config;

  // Members.
  ExperimentLogger::Ptr logger_;

  // Which map names to process. This data is always set.
  std::vector<std::string> map_names_;
  std::map<std::string, uint64_t> map_timestamps_;

  // Lazy loaded data.
  // Khronos backend outputs to be processed by the pipeline. map_name -> (dsg, rpgo_merges)
  std::map<std::string, std::pair<DynamicSceneGraph::Ptr, RPGOMerges>> map_data_;
  // Changes detected by the pipeline. change_detector -> map_name -> changes
  std::map<std::string, std::map<std::string, khronos::Changes>> changes_data_;
  // Output of the pipeline. change_detector -> reconciler -> map_name -> reconciled_dsg.
  std::map<std::string, std::map<std::string, std::map<std::string, DynamicSceneGraph::Ptr>>>
      reconciled_dsg_data_;
  // The selected DSGs for the current evaluation. map_name -> reconciled_dsg.
  std::map<std::string, DynamicSceneGraph::Ptr> reconciled_dsgs_;
  // Ground truth DSG.
  DynamicSceneGraph::Ptr gt_dsg_;
  DynamicSceneGraph::Ptr gt_dsg_dynamic_;
  Points gt_background_;
};

void declare_config(PipelineEvaluator::ChangeDetectionConfig& config);
void declare_config(PipelineEvaluator::ReconciliationConfig& config);
void declare_config(PipelineEvaluator::EvaluationConfig& config);

}  // namespace khronos
