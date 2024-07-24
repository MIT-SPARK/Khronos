#pragma once

#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/utils/nearest_neighbor_utilities.h>
namespace khronos {

/**
 * @brief Class to evaluate a mesh against a ground truth mesh.
 */
class ObjectEvaluator {
 public:
  // Config.
  struct Config {
    int verbosity = 0;
    // If true only objects of same semantic class can be matched.
    bool match_require_semantics = true;

    // Distance [m] surfaces can be apart to be considered a match.
    float max_matching_distance = 0.5f;

    // If true fuse over and undersegmentation before computing change detection metrics.
    bool compensate_missegmentation = true;

    // Only evaluate every n-th surface point to speed up computation.
    int surface_subsampling_rate = 1;

    // CSV file listing all GT changes in the scene.
    std::string changes_file;

    // Which method to use for associating objects.
    enum class AssociationMethod {
      SURFACE,
      CENTROID,
      BOUNDINGBOX
    } association_method = AssociationMethod::CENTROID;
  } const config;

  // Construction.
  explicit ObjectEvaluator(const Config& config);
  virtual ~ObjectEvaluator() = default;

  // Evaluation interface.
  /**
   * @brief Set the output file to write the results to. Optionally set a directory to save
   * condensed visualization information.
   * @param file_name The file to write the results to.
   * @param eval_vis_dir Optional: directory to save condensed visualization information to.
   */
  void setOutputFile(const std::string& file_name, const std::string& eval_vis_dir = "");
  void setGroundTruthDSG(DynamicSceneGraph::Ptr dsg);
  void setEvalDSG(DynamicSceneGraph::Ptr dsg);
  bool evaluate(const std::string& name, const uint64_t query_time);

 protected:
  friend class EvalVisualizer;

  // struct that contains relevant data for easy evaluation.
  struct DsgState {
    // Quick access to all valid objects.
    std::unordered_map<NodeId, KhronosObjectAttributes> objects;

    // Number of objects that could not be processed.
    size_t num_objects_invalid = 0;

    // The complete DSG as underlying data.
    DynamicSceneGraph::Ptr dsg;
  };

  struct RawMetrics {
    // associations <from, to>
    std::unordered_map<NodeId, NodeId> associations;
    // Set of all <from> that were not associated.
    std::set<NodeId> failed;
  };

  struct SegmentationMetrics {
    // Out of the objects that are present and matched, how many are over/under segmented.
    int num_oversegmented = 0;
    int num_undersegmented = 0;
    int num_correct = 0;
    float mean_oversegmentation_degree = 0.f;
    float mean_undersegmentation_degree = 0.f;
    int max_oversegmentation_degree = 0;
    int max_undersegmentation_degree = 0;
  };

  struct ChangeMetrics {
    // Out of the detected objects, what is the accuracy of the change detection.
    int appeared_tp = 0;
    int disappeared_tp = 0;
    int appeared_fp = 0;
    int disappeared_fp = 0;
    int appeared_hallucinated_p = 0;  // Halucinated positives = fp without a match.
    int disappeared_hallucinated_p = 0;
    int appeared_fn = 0;
    int disappeared_fn = 0;
    int appeared_tn = 0;
    int disappeared_tn = 0;
    int appeared_missed_p = 0;  // Mised positives = fn without a match.
    int disappeared_missed_p = 0;
  };

  struct DetectionMetrics {
    // Out of the objects present in the believe and the ground truth, how many are detected.
    // Excludes over segmentation.
    int num_detected = 0;      // True positives.
    int num_missed = 0;        // False negatives.
    int num_hallucinated = 0;  // False positives.
    // There's not really a true negative for object detection.
  };

  // Struct to apply timings to the evaluation.
  struct GtChange {
    NodeId id;
    // Timestamps the object changed. 0 will be mapped to +/- inf and ingnored.
    uint64_t appeared;
    uint64_t disappeared;
  };

  // Evaluation functions.
  RawMetrics computeAssociationsSurface(const DsgState& from,
                                        const DsgState& to) const;  // Very accurate.
  RawMetrics computeAssociationsCentroid(const DsgState& from, const DsgState& to) const;  // Fast.
  RawMetrics computeAssociationsBoundingBox(const DsgState& from, const DsgState& to) const;
  SegmentationMetrics computeSegmentationMetrics(const uint64_t query_time) const;
  ChangeMetrics computeChangeMetrics(const uint64_t query_time) const;
  DetectionMetrics computeDetectionMetrics(const uint64_t query_time) const;

  // Helper Functions.
  // NOTE: If query time is !=0 this filters by whether the objects are present at time query_time.
  //  Returns <to_node_id, num_associations>.
  std::unordered_map<NodeId, int> computeAssociationCardinality(
      bool from_gt,
      const uint64_t query_time = 0) const;

  // Writing to csv file.
  std::string writeHeader() const;
  std::string writeMetrics(const std::string& name,
                           const uint64_t query_time,
                           const SegmentationMetrics& segmentation,
                           const ChangeMetrics& changes,
                           const DetectionMetrics& detections) const;

  // Tools to save visualization information.
  bool setupVisFiles(const std::string& vis_dir);
  void saveVisData(const std::string& name, const uint64_t query_time);
  void saveVisObject(const std::string& name,
                     const uint64_t query_time,
                     const NodeId id,
                     const KhronosObjectAttributes& attrs,
                     bool is_gt);

 private:
  // Ground truth data.
  DsgState gt_dsg_;
  DsgState eval_dsg_;
  std::map<NodeId, std::vector<GtChange>> gt_changes_;

  // Cache associations data.
  bool associations_updated_ = false;
  RawMetrics dsg_to_gt_;
  RawMetrics gt_to_dsg_;

  // Variables.
  bool is_setup_ = false;
  std::fstream output_file_;
  std::ofstream vis_obj_file_;
  std::ofstream vis_assoc_file_;
  bool eval_vis_setup_ = false;
  std::function<RawMetrics(const DsgState& from, const DsgState& to)> computeAssociations;
};

void declare_config(ObjectEvaluator::Config& config);

}  // namespace khronos
