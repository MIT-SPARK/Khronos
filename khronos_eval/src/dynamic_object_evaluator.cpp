#include "khronos_eval/dynamic_object_evaluator.h"

#include <algorithm>
#include <filesystem>

#include <khronos/utils/khronos_attribute_utils.h>
#include <khronos/utils/output_file_utils.h>

namespace khronos {

void declare_config(DynamicObjectEvaluator::Config& config) {
  using namespace config;
  name("DynamicObjectEvaluator");
  field(config.verbosity, "verbosity");
  field(config.compensate_missegmentation, "compensate_missegmentation");
  field(config.max_matching_distance, "max_matching_distance", "m");
}

DynamicObjectEvaluator::DynamicObjectEvaluator(const Config& config)
    : config(config::checkValid(config)) {}

void DynamicObjectEvaluator::setOutputFile(const std::string& file_name) {
  const std::string output_dir = std::filesystem::path(file_name).parent_path();
  if (ensureDirectoryExists(output_dir)) {
    openOrAppendToCsv(file_name, output_file_, writeHeader());
  }

  is_setup_ = true;
}

bool DynamicObjectEvaluator::evaluate(const std::string& name, const uint64_t query_time) {
  if (!is_setup_ || !output_file_) {
    return false;
  }
  if (!gt_dsg_.dsg) {
    LOG(ERROR) << "Ground truth DSG not set.";
    return false;
  }
  if (!eval_dsg_.dsg) {
    LOG(ERROR) << "Eval DSG not set.";
    return false;
  }

  // Get all timestamps with dynamic objects up to the query_time.
  std::set<uint64_t> timestamps;
  for (const auto& [id, attrs] : eval_dsg_.objects) {
    for (const auto& ts : attrs.trajectory_timestamps) {
      if (ts <= query_time) {  // should always be true for eval_dsg
        timestamps.insert(ts);
      }
    }
  }
  for (const auto& [id, attrs] : gt_dsg_.objects) {
    for (const auto& ts : attrs.trajectory_timestamps) {
      if (ts <= query_time) {
        timestamps.insert(ts);
      }
    }
  }

  // Compute associations
  dsg_to_gt_ = computeAssociationsCentroid(eval_dsg_, gt_dsg_, timestamps);
  gt_to_dsg_ = computeAssociationsCentroid(gt_dsg_, eval_dsg_, timestamps);

  // Compute the results.
  const DetectionMetrics detections = computeDetectionMetrics(query_time);

  // Write the output to file.
  output_file_ << writeMetrics(name, query_time, detections);

  return true;
}

void DynamicObjectEvaluator::setGroundTruthDSG(DynamicSceneGraph::Ptr dsg) {
  if (!dsg) {
    LOG(ERROR) << "Ground truth DSG pointer is not valid.";
    return;
  }
  gt_dsg_.dsg = std::move(dsg);
  gt_dsg_.objects.clear();
  gt_dsg_.num_objects_invalid = 0;

  const auto& nodes = gt_dsg_.dsg->getLayer(DsgLayers::OBJECTS).nodes();
  for (const auto& [id, node] : nodes) {
    auto attrs = dynamic_cast<KhronosObjectAttributes*>(node->getAttributesPtr());
    // Ignore static objects
    if (attrs->trajectory_timestamps.size() == 0) {
      continue;
    }
    // Check if object was marked abset for some reason
    if (!attrs) {
      gt_dsg_.num_objects_invalid++;
      continue;
    }
    // Sort fields and add object
    std::sort(attrs->first_observed_ns.begin(), attrs->first_observed_ns.end());
    std::sort(attrs->last_observed_ns.begin(), attrs->last_observed_ns.end());
    attrs->first_observed_ns.erase(
        std::unique(attrs->first_observed_ns.begin(), attrs->first_observed_ns.end()),
        attrs->first_observed_ns.end());
    attrs->last_observed_ns.erase(
        std::unique(attrs->last_observed_ns.begin(), attrs->last_observed_ns.end()),
        attrs->last_observed_ns.end());
    gt_dsg_.objects.emplace(id, *attrs);
  }
}

void DynamicObjectEvaluator::setEvalDSG(DynamicSceneGraph::Ptr dsg) {
  if (!dsg) {
    LOG(ERROR) << "Eval DSG pointer is not valid.";
    return;
  }
  eval_dsg_.dsg = std::move(dsg);
  eval_dsg_.objects.clear();
  eval_dsg_.num_objects_invalid = 0;

  const auto& nodes = eval_dsg_.dsg->getLayer(DsgLayers::OBJECTS).nodes();
  for (const auto& [id, node] : nodes) {
    auto attrs = dynamic_cast<KhronosObjectAttributes*>(node->getAttributesPtr());
    // Ignore static objects
    if (attrs->trajectory_timestamps.size() == 0) {
      continue;
    }
    if (!attrs) {
      eval_dsg_.num_objects_invalid++;
      continue;
    }

    KhronosObjectAttributes new_attrs = *attrs;

    // For Dynablox we don't have first/last observed, so fill it in manually
    // from trajectory timestamps.
    if (new_attrs.first_observed_ns.empty()) {
      new_attrs.first_observed_ns.push_back(new_attrs.trajectory_timestamps.front());
    }
    if (new_attrs.last_observed_ns.empty()) {
      new_attrs.last_observed_ns.push_back(new_attrs.trajectory_timestamps.back());
    }

    eval_dsg_.objects.emplace(id, std::move(new_attrs));
  }
}

DynamicObjectEvaluator::RawMetrics DynamicObjectEvaluator::computeAssociationsCentroid(
    const DsgState& from,
    const DsgState& to,
    const std::set<uint64_t>& timestamps) const {
  // Find all associations of from to to.
  RawMetrics result;
  const float max_distance_sq = config.max_matching_distance * config.max_matching_distance;

  // Iterate over every frame
  for (const auto& timestamp : timestamps) {
    std::pair<Points, std::vector<NodeId>> to_points;
    // Build a pointcloud of only the available objects centroids.
    for (const auto& [id, attrs] : to.objects) {
      auto it = std::find(
          attrs.trajectory_timestamps.begin(), attrs.trajectory_timestamps.end(), timestamp);
      if (it != attrs.trajectory_timestamps.end()) {
        Point to_centroid =
            attrs.trajectory_positions[std::distance(attrs.trajectory_timestamps.begin(), it)];
        to_points.first.emplace_back(to_centroid);
        to_points.second.emplace_back(id);
      }
    }

    // Find associations via nearest neighbors.
    result.associations[timestamp] = std::unordered_map<NodeId, NodeId>();
    for (const auto& [id, attrs] : from.objects) {
      auto it = std::find(
          attrs.trajectory_timestamps.begin(), attrs.trajectory_timestamps.end(), timestamp);
      if (it == attrs.trajectory_timestamps.end()) {
        continue;
      }
      Point from_centroid =
          attrs.trajectory_positions[std::distance(attrs.trajectory_timestamps.begin(), it)];
      const auto& ids = to_points.second;
      hydra::PointNeighborSearch search(to_points.first);
      float distance_squared;
      size_t index;
      if (!search.search(from_centroid, distance_squared, index) ||
          distance_squared > max_distance_sq) {
        CLOG(4) << NodeSymbol(id) << ": no candidates found. dist squared: " << distance_squared;
        result.failed[timestamp].emplace(id);
        continue;
      }

      result.associations[timestamp][id] = ids[index];
      CLOG(4) << NodeSymbol(id) << " -> " << NodeSymbol(ids[index]) << " ("
              << std::sqrt(distance_squared) << "m).";
    }
  }

  return result;
}

DynamicObjectEvaluator::DetectionMetrics DynamicObjectEvaluator::computeDetectionMetrics(
    const uint64_t query_time) const {
  DetectionMetrics result;
  int already_visited = 0;

  // Ignore oversegmentation by filtering multiple visits to the same object.
  for (const auto& [timestamp, associations] : dsg_to_gt_.associations) {
    std::unordered_set<NodeId> visited;
    for (const auto& [from, to] : associations) {
      // Exclude double lookups if configured.
      if (config.compensate_missegmentation) {
        if (visited.count(to)) {
          already_visited++;
          continue;
        }
        visited.emplace(to);
      }
      // Found a matching GT object for this DSG object; true positive.
      // NOTE: possible oversegmentation without visited list.
      result.num_detected++;
    }
  }

  // Add all objects that are not associated.
  for (const auto& [timestamp, ids] : dsg_to_gt_.failed) {
    for (size_t i = 0; i < ids.size(); i++) {
      // Failed to find a matching GT object for this DSG object; false positive.
      result.num_hallucinated++;
    }
  }

  // Add all objects that are not associated.
  for (const auto& [timestamp, ids] : gt_to_dsg_.failed) {
    for (size_t i = 0; i < ids.size(); i++) {
      // Failed to find a matching DSG object for this GT object; false negative.
      result.num_missed++;
    }
  }

  LOG_IF(ERROR, already_visited > 0) << "already_visited: " << already_visited;
  return result;
}

std::string DynamicObjectEvaluator::writeHeader() const {
  std::stringstream header;
  header << "Name,Query,NumGtLoaded,NumDsgLoaded,NumGtNotLoaded,NumDsgNotLoaded,NumObjDetected,"
            "NumObjMissed,NumObjHallucinated";
  return header.str();
}

std::string DynamicObjectEvaluator::writeMetrics(const std::string& name,
                                                 const uint64_t query_time,
                                                 const DetectionMetrics& detections) const {
  std::stringstream result;
  result << "\n"
         << name << "," << query_time << "," << gt_dsg_.objects.size() << ","
         << eval_dsg_.objects.size() << "," << gt_dsg_.num_objects_invalid << ","
         << eval_dsg_.num_objects_invalid << "," << detections.num_detected << ","
         << detections.num_missed << "," << detections.num_hallucinated;
  return result.str();
}

}  // namespace khronos
