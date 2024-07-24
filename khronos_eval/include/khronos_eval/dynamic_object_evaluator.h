#pragma once

#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/utils/nearest_neighbor_utilities.h>

#include "khronos/common/common_types.h"
// TODO(marcus): this should all be included in the ObjectEvaluator
namespace khronos {

class DynamicObjectEvaluator {
 public:
  struct Config {
    int verbosity = 0;
    bool compensate_missegmentation = true;
    float max_matching_distance = 0.5f;
  } const config;

  explicit DynamicObjectEvaluator(const Config& config);
  virtual ~DynamicObjectEvaluator() = default;

  void setOutputFile(const std::string& file_name);
  void setGroundTruthDSG(DynamicSceneGraph::Ptr dsg);
  void setEvalDSG(DynamicSceneGraph::Ptr dsg);
  bool evaluate(const std::string& name, const uint64_t query_time);

 protected:
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
    std::map<uint64_t, std::unordered_map<NodeId, NodeId>> associations;
    // Set of all <from> that were not associated.
    std::map<uint64_t, std::set<NodeId>> failed;
  };

  struct DetectionMetrics {
    // Out of the objects present in the believe and the ground truth, how many are detected.
    // Excludes over segmentation.
    int num_detected = 0;      // True positives.
    int num_missed = 0;        // False negatives.
    int num_hallucinated = 0;  // False positives.
    // There's not really a true negative for object detection.
  };

  RawMetrics computeAssociationsCentroid(const DsgState& from,
                                         const DsgState& to,
                                         const std::set<uint64_t>& timestamps) const;
  DetectionMetrics computeDetectionMetrics(const uint64_t query_time) const;

  // Writing to csv file.
  std::string writeHeader() const;
  std::string writeMetrics(const std::string& name,
                           const uint64_t query_time,
                           const DetectionMetrics& detections) const;

 private:
  DsgState gt_dsg_;
  DsgState eval_dsg_;

  RawMetrics dsg_to_gt_;
  RawMetrics gt_to_dsg_;

  bool is_setup_ = false;
  std::fstream output_file_;

  std::function<RawMetrics(const DsgState& from, const DsgState& to)> computeAssociations;
};

void declare_config(DynamicObjectEvaluator::Config& config);

}  // namespace khronos
