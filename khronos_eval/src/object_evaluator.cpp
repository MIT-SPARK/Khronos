#include "khronos_eval/object_evaluator.h"

#include <filesystem>

#include <hydra/utils/csv_reader.h>
#include <khronos/backend/change_state.h>
#include <khronos/common/utils/index_getter.h>
#include <khronos/utils/khronos_attribute_utils.h>
#include <khronos/utils/output_file_utils.h>
#include <pcl/io/ply_io.h>

namespace khronos {

void declare_config(ObjectEvaluator::Config& config) {
  using namespace config;
  name("ObjectEvaluator");
  field(config.verbosity, "verbosity");
  field(config.match_require_semantics, "match_require_semantics");
  field(config.max_matching_distance, "max_matching_distance", "m");
  field(config.changes_file, "changes_file");
  field(config.surface_subsampling_rate, "surface_subsampling_rate");
  enum_field(config.association_method,
             "association_method",
             {{ObjectEvaluator::Config::AssociationMethod::SURFACE, "Surface"},
              {ObjectEvaluator::Config::AssociationMethod::CENTROID, "Centroid"},
              {ObjectEvaluator::Config::AssociationMethod::BOUNDINGBOX, "BoundingBox"}});

  check(config.surface_subsampling_rate, GT, 0, "surface_subsampling_rate");
}

ObjectEvaluator::ObjectEvaluator(const Config& config) : config(config::checkValid(config)) {
  // Setup the association methods.
  switch (config.association_method) {
    case Config::AssociationMethod::SURFACE:
      computeAssociations = std::bind(&ObjectEvaluator::computeAssociationsSurface,
                                      this,
                                      std::placeholders::_1,
                                      std::placeholders::_2);
      break;
    case Config::AssociationMethod::CENTROID:
      computeAssociations = std::bind(&ObjectEvaluator::computeAssociationsCentroid,
                                      this,
                                      std::placeholders::_1,
                                      std::placeholders::_2);
      break;
    case Config::AssociationMethod::BOUNDINGBOX:
      computeAssociations = std::bind(&ObjectEvaluator::computeAssociationsBoundingBox,
                                      this,
                                      std::placeholders::_1,
                                      std::placeholders::_2);
      break;
    default:
      LOG(ERROR) << "Unknown association method.";
      return;
  }

  // Read the changes file.
  if (config.changes_file.empty()) {
    return;
  }
  hydra::CsvReader reader(config.changes_file);
  if (!reader.isSetup()) {
    LOG(WARNING) << "Ground truth changes file is set but could not be read!";
    return;
  }

  // Check the data is correct and read it.
  for (const auto& header : {"ObjectSymbol", "AppearedAt", "DisappearedAt"}) {
    if (!reader.hasHeader(header)) {
      LOG(WARNING) << "Ground truth changes file is missing header '" << header << "'.";
      return;
    }
  }
  size_t num_changes_loaded = 0;
  for (size_t i = 0; i < reader.numRows(); ++i) {
    const std::string symbol_string = reader.getEntry("ObjectSymbol", i);
    // Assumes objects are given as "O(#)".
    NodeSymbol symbol(symbol_string.at(0),
                      std::stoi(symbol_string.substr(2, symbol_string.size() - 1)));
    GtChange gt_change;
    gt_change.id = symbol;
    gt_change.appeared = std::stoull(reader.getEntry("AppearedAt", i));
    gt_change.disappeared = std::stoull(reader.getEntry("DisappearedAt", i));
    gt_changes_[symbol].emplace_back(std::move(gt_change));
    num_changes_loaded++;
  }
  LOG(INFO) << "Loaded " << num_changes_loaded << " ground truth changes.";
  is_setup_ = true;
}

void ObjectEvaluator::setOutputFile(const std::string& file_name, const std::string& eval_vis_dir) {
  // Verify that the output can be setup correctly. This appends to the output file if it already
  // exists and has identical data.
  const std::string output_dir = std::filesystem::path(file_name).parent_path();
  if (ensureDirectoryExists(output_dir)) {
    openOrAppendToCsv(file_name, output_file_, writeHeader());
  }

  if (ensureDirectoryExists(eval_vis_dir)) {
    eval_vis_setup_ = setupVisFiles(eval_vis_dir);
  }
}

bool ObjectEvaluator::evaluate(const std::string& name, const uint64_t query_time) {
  // Verify data is set.
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

  // Evalaute the dsg vs ground truth and vice versa.
  if (!associations_updated_) {
    dsg_to_gt_ = computeAssociations(eval_dsg_, gt_dsg_);
    gt_to_dsg_ = computeAssociations(gt_dsg_, eval_dsg_);
    associations_updated_ = true;
  }

  // Compute the results.
  const SegmentationMetrics segmentation = computeSegmentationMetrics(query_time);
  const ChangeMetrics changes = computeChangeMetrics(query_time);
  const DetectionMetrics detections = computeDetectionMetrics(query_time);

  // Write the output to file.
  output_file_ << writeMetrics(name, query_time, segmentation, changes, detections);

  // If visualizaton data is to be stored, store it.
  saveVisData(name, query_time);

  return true;
}

void ObjectEvaluator::setGroundTruthDSG(DynamicSceneGraph::Ptr dsg) {
  // Ensure the DSG is valid.
  if (!dsg) {
    LOG(ERROR) << "Ground truth DSG pointer is not valid.";
    return;
  }
  gt_dsg_.dsg = std::move(dsg);
  gt_dsg_.objects.clear();
  gt_dsg_.num_objects_invalid = 0;

  // Add all valid objects to the state and set their first and last observed time.
  const auto& nodes = gt_dsg_.dsg->getLayer(DsgLayers::OBJECTS).nodes();
  for (const auto& [id, node] : nodes) {
    // Check if the object was marked absent for some reason.
    auto attrs = dynamic_cast<KhronosObjectAttributes*>(node->getAttributesPtr());
    if (!attrs) {
      gt_dsg_.num_objects_invalid++;
      continue;
    }

    // Check the changes times.
    const auto it = gt_changes_.find(id);
    if (it == gt_changes_.end()) {
      attrs->first_observed_ns = {0};
      attrs->last_observed_ns = {std::numeric_limits<uint64_t>::max()};
      gt_dsg_.objects.emplace(id, *attrs);
      // No change times.
      continue;
    }

    // Add all changes, remove duplciates, and sort.
    for (const GtChange& change : it->second) {
      attrs->first_observed_ns.emplace_back(change.appeared);
      attrs->last_observed_ns.emplace_back(
          change.disappeared == 0 ? std::numeric_limits<uint64_t>::max() : change.disappeared);
    }
    std::sort(attrs->first_observed_ns.begin(), attrs->first_observed_ns.end());
    std::sort(attrs->last_observed_ns.begin(), attrs->last_observed_ns.end());
    attrs->first_observed_ns.erase(
        std::unique(attrs->first_observed_ns.begin(), attrs->first_observed_ns.end()),
        attrs->first_observed_ns.end());
    attrs->last_observed_ns.erase(
        std::unique(attrs->last_observed_ns.begin(), attrs->last_observed_ns.end()),
        attrs->last_observed_ns.end());

    // Translate points into world frame.
    for (auto& vertex : attrs->mesh.points) {
      vertex = attrs->bounding_box.pointToWorldFrame(vertex);
    }

    gt_dsg_.objects.emplace(id, *attrs);
  }
  associations_updated_ = false;
}

void ObjectEvaluator::setEvalDSG(DynamicSceneGraph::Ptr dsg) {
  // Ensure the DSG is valid.
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
    if (!attrs) {
      eval_dsg_.num_objects_invalid++;
      continue;
    }
    if (attrs->first_observed_ns.empty() || attrs->last_observed_ns.empty()) {
      eval_dsg_.num_objects_invalid++;
      continue;
    }

    // Translate points into world frame.
    KhronosObjectAttributes new_attrs = *attrs;
    for (auto& vertex : new_attrs.mesh.points) {
      vertex = new_attrs.bounding_box.pointToWorldFrame(vertex);
    }
    eval_dsg_.objects.emplace(id, std::move(new_attrs));
  }
  associations_updated_ = false;
}

ObjectEvaluator::RawMetrics ObjectEvaluator::computeAssociationsSurface(const DsgState& from,
                                                                        const DsgState& to) const {
  // Find all associations of from to to.
  RawMetrics result;
  const float max_distance_sq = config.max_matching_distance * config.max_matching_distance;

  // Build a pointcloud of only the available objects.
  std::unordered_map<int, std::pair<Points, std::vector<NodeId>>> to_points_by_class;
  for (const auto& [id, attrs] : to.objects) {
    const int label = config.match_require_semantics ? attrs.semantic_label : 0;
    auto& points = to_points_by_class[label].first;
    auto& ids = to_points_by_class[label].second;
    points.insert(points.end(), attrs.mesh.points.begin(), attrs.mesh.points.end());
    ids.insert(ids.end(), attrs.mesh.points.size(), id);
  }

  // Find accosiations via maximum voting of all surface points.
  for (const auto& [id, attrs] : from.objects) {
    const int label = config.match_require_semantics ? attrs.semantic_label : 0;
    const auto& ids = to_points_by_class[label].second;
    hydra::PointNeighborSearch search(to_points_by_class[label].first);
    std::unordered_map<NodeId, int> votes;
    for (size_t i = 0; i < attrs.mesh.numVertices(); i += config.surface_subsampling_rate) {
      float distance_squared;
      size_t index;
      if (!search.search(attrs.mesh.pos(i), distance_squared, index)) {
        continue;
      }
      if (distance_squared > max_distance_sq) {
        continue;
      }
      votes[ids[index]]++;
    }

    // Extract the highest vote.
    if (votes.empty()) {
      CLOG(4) << NodeSymbol(id) << ": no candidates found.";
      result.failed.emplace(id);
      continue;
    }
    const auto it = std::max_element(votes.begin(), votes.end(), [](const auto& a, const auto& b) {
      return a.second < b.second;
    });
    result.associations[id] = it->first;
    CLOG(4) << NodeSymbol(id) << " -> " << NodeSymbol(it->first) << " ("
            << it->second * 100 / attrs.mesh.numVertices() << "%).";
  }

  return result;
}

ObjectEvaluator::RawMetrics ObjectEvaluator::computeAssociationsCentroid(const DsgState& from,
                                                                         const DsgState& to) const {
  // Find all associations of from to to.
  RawMetrics result;
  const float max_distance_sq = config.max_matching_distance * config.max_matching_distance;

  // Build a pointcloud of only the available objects centroids.
  std::unordered_map<int, std::pair<Points, std::vector<NodeId>>> to_points_by_class;
  for (const auto& [id, attrs] : to.objects) {
    const int label = config.match_require_semantics ? attrs.semantic_label : 0;
    to_points_by_class[label].first.emplace_back(computeSurfaceCentroid(attrs));
    to_points_by_class[label].second.emplace_back(id);
  }

  // Find accosiations via nearest neighbors.
  for (const auto& [id, attrs] : from.objects) {
    const int label = config.match_require_semantics ? attrs.semantic_label : 0;
    const auto& ids = to_points_by_class[label].second;
    hydra::PointNeighborSearch search(to_points_by_class[label].first);
    float distance_squared;
    size_t index;
    if (!search.search(computeSurfaceCentroid(attrs), distance_squared, index) ||
        distance_squared > max_distance_sq) {
      CLOG(4) << NodeSymbol(id) << ": no candidates found.";
      continue;
    }

    result.associations[id] = ids[index];
    CLOG(4) << NodeSymbol(id) << " -> " << NodeSymbol(ids[index]) << " ("
            << std::sqrt(distance_squared) << "m).";
  }

  return result;
}

ObjectEvaluator::RawMetrics ObjectEvaluator::computeAssociationsBoundingBox(
    const DsgState& from,
    const DsgState& to) const {
  // Find all associations of from to to.
  RawMetrics result;

  // Exhaustively search all bounding boxes for maximum overlap.
  for (const auto& [from_id, from_attrs] : from.objects) {
    float max_iou = config.max_matching_distance;
    NodeId max_id = 0;
    for (const auto& [to_id, to_attrs] : to.objects) {
      if (config.match_require_semantics && from_attrs.semantic_label != to_attrs.semantic_label) {
        continue;
      }

      if (!from_attrs.bounding_box.intersects(to_attrs.bounding_box)) {
        continue;
      }
      const float iou = from_attrs.bounding_box.computeIoU(to_attrs.bounding_box);

      if (iou > max_iou) {
        max_iou = iou;
        max_id = to_id;
      }
    }

    if (max_iou == 0.f) {
      result.failed.emplace(from_id);
      CLOG(4) << NodeSymbol(from_id) << ": no candidates found.";
    } else {
      result.associations[from_id] = max_id;
      CLOG(4) << NodeSymbol(from_id) << " -> " << NodeSymbol(max_id) << " (" << max_iou << ").";
    }
  }

  return result;
}

ObjectEvaluator::SegmentationMetrics ObjectEvaluator::computeSegmentationMetrics(
    const uint64_t query_time) const {
  SegmentationMetrics result;

  // Compute oversegmentation (dsg_to_gt).
  std::unordered_map<NodeId, int> num_associations =
      computeAssociationCardinality(false, query_time);  // dsg_to_gt
  for (const auto& [_, cardinality] : num_associations) {
    if (cardinality == 1) {
      result.num_correct++;
      continue;
    }
    result.num_oversegmented++;
    result.mean_oversegmentation_degree += cardinality;
    result.max_oversegmentation_degree = std::max(result.max_oversegmentation_degree, cardinality);
  }
  result.mean_oversegmentation_degree /= result.num_oversegmented;

  // Compute undersegmentation (gt_to_dsg).
  num_associations = computeAssociationCardinality(true, query_time);  // gt_to_dsg
  for (const auto& [_, cardinality] : num_associations) {
    if (cardinality <= 1) {
      continue;
    }
    result.num_undersegmented++;
    result.mean_undersegmentation_degree += cardinality;
    result.max_undersegmentation_degree =
        std::max(result.max_undersegmentation_degree, cardinality);
  }
  result.mean_undersegmentation_degree /= result.num_undersegmented;

  return result;
}

ObjectEvaluator::ChangeMetrics ObjectEvaluator::computeChangeMetrics(
    const uint64_t query_time) const {
  ChangeMetrics result;

  // NOTE(lschmid): This evaluation assumes that the Optimistic reconciler is used, where
  // first seen > 0 indicates it has appeared and last seen < max indicates it has disappeared.

  // Appeared. p=present, a=have appeared/disappeared
  std::stringstream info;
  info << "\n------ DSG to GT ------";
  std::unordered_set<NodeId> visited;
  for (const auto& [from, to] : dsg_to_gt_.associations) {
    // Exclude double lookups if configured.
    if (config.compensate_missegmentation) {
      if (visited.count(to)) {
        continue;
      }
      visited.emplace(to);
    }

    const auto& from_attrs = eval_dsg_.objects.at(from);
    const auto& to_attrs = gt_dsg_.objects.at(to);
    const bool to_appeared = hasAppeared(to_attrs, query_time);
    const bool to_disappeared = hasDisappeared(to_attrs, query_time);
    const bool from_appeared = hasAppeared(from_attrs, query_time);
    const bool from_disappeared = hasDisappeared(from_attrs, query_time);

    info << "\n"
         << NodeSymbol(from) << " [" << (from_appeared ? "A" : "") << (from_disappeared ? "D" : "")
         << "] -> " << NodeSymbol(to) << " [" << (to_appeared ? "A" : "")
         << (to_disappeared ? "D" : "") << "]: ";

    // TP: DSG p+a and GT p+a.
    if (from_appeared && to_appeared) {
      result.appeared_tp++;
      info << "TPA ";
    }
    if (from_disappeared && to_disappeared) {
      result.disappeared_tp++;
      info << "TPD ";
    }

    // FP: DSG p+a but not GT p or not GT a.
    if (from_appeared && !to_appeared) {
      result.appeared_fp++;
      info << "FPA ";
    }
    if (from_disappeared && !to_disappeared) {
      result.disappeared_fp++;
      info << "FPD ";
    }

    // TN: DSG p and !a and GT p and !a.
    if (!from_appeared && !to_appeared) {
      result.appeared_tn++;
      info << "TNA ";
    }
    if (!from_disappeared && !from_appeared) {
      result.disappeared_tn++;
      info << "TND ";
    }
  }

  // Add all objects that are not associated.
  for (const auto& id : dsg_to_gt_.failed) {
    const auto& attrs = eval_dsg_.objects.at(id);
    const bool appeared = hasAppeared(attrs, query_time);
    const bool disappeared = hasDisappeared(attrs, query_time);
    if (appeared) {
      result.appeared_hallucinated_p++;
    }
    if (disappeared) {
      result.disappeared_hallucinated_p++;
    }
  }

  info << "\n------ GT to DSG ------";
  visited.clear();
  for (const auto& [from, to] : gt_to_dsg_.associations) {
    // Exclude double lookups if configured.
    if (config.compensate_missegmentation) {
      if (visited.count(to)) {
        continue;
      }
      visited.emplace(to);
    }

    const auto& from_attrs = gt_dsg_.objects.at(from);
    const auto& to_attrs = eval_dsg_.objects.at(to);
    const bool to_appeared = hasAppeared(to_attrs, query_time);
    const bool to_disappeared = hasDisappeared(to_attrs, query_time);
    const bool from_appeared = hasAppeared(from_attrs, query_time);
    const bool from_disappeared = hasDisappeared(from_attrs, query_time);

    info << "\n"
         << NodeSymbol(from) << " [" << (from_appeared ? "A" : "") << (from_disappeared ? "D" : "")
         << "] -> " << NodeSymbol(to) << " [" << (to_appeared ? "A" : "")
         << (to_disappeared ? "D" : "") << "]: ";

    // FN: GT p+a but not DSG p or not DSG a.
    if (from_appeared && !to_appeared) {
      result.appeared_fn++;
      info << "FNA ";
    }
    if (from_disappeared && !to_disappeared) {
      result.disappeared_fn++;
      info << "FND ";
    }
  }

  // Add all objects that are not associated.
  for (const auto& id : gt_to_dsg_.failed) {
    const auto& attrs = gt_dsg_.objects.at(id);
    const bool appeared = hasAppeared(attrs, query_time);
    const bool disappeared = hasDisappeared(attrs, query_time);
    if (appeared) {
      result.appeared_missed_p++;
    }
    if (disappeared) {
      result.disappeared_missed_p++;
    }
  }
  CLOG(4) << info.str();
  return result;
}

ObjectEvaluator::DetectionMetrics ObjectEvaluator::computeDetectionMetrics(
    const uint64_t query_time) const {
  DetectionMetrics result;

  // Ignore oversegmentation by filtering multiple visits to the same object.
  std::unordered_set<NodeId> visited;
  for (const auto& [from, to] : dsg_to_gt_.associations) {
    // Exclude double lookups if configured.
    if (config.compensate_missegmentation) {
      if (visited.count(to)) {
        continue;
      }
      visited.emplace(to);
    }

    const auto& from_attrs = eval_dsg_.objects.at(from);
    const auto& to_attrs = gt_dsg_.objects.at(to);
    const bool to_present = isPresent(to_attrs, query_time);
    const bool from_present = isPresent(from_attrs, query_time);

    if (from_present && to_present) {
      result.num_detected++;
    } else if (from_present && !to_present) {
      result.num_hallucinated++;
    }
  }

  // Add all objects that are not associated.
  for (const auto id : dsg_to_gt_.failed) {
    const auto& attrs = eval_dsg_.objects.at(id);
    if (isPresent(attrs, query_time)) {
      result.num_hallucinated++;
    }
  }

  visited.clear();
  for (const auto& [from, to] : gt_to_dsg_.associations) {
    // Exclude double lookups if configured.
    if (config.compensate_missegmentation) {
      if (visited.count(to)) {
        continue;
      }
      visited.emplace(to);
    }

    const auto& from_attrs = gt_dsg_.objects.at(from);
    const auto& to_attrs = eval_dsg_.objects.at(to);
    const bool to_present = isPresent(to_attrs, query_time);
    const bool from_present = isPresent(from_attrs, query_time);

    if (from_present && !to_present) {
      result.num_missed++;
    }
  }
  // Add all objects that are not associated.
  for (const auto id : gt_to_dsg_.failed) {
    const auto& attrs = gt_dsg_.objects.at(id);
    if (isPresent(attrs, query_time)) {
      result.num_missed++;
    }
  }

  return result;
}

std::string ObjectEvaluator::writeHeader() const {
  std::stringstream header;
  header
      << "Name,Query,NumGtLoaded,NumDsgLoaded,NumGtNotLoaded,NumDsgNotLoaded,NumOversegmented,"
         "NumUndersegmented,NumCorrect,MeanOversegmentationDegree,MeanUndersegmentationDegree,"
         "MaxOversegmentationDegree,MaxUndersegmentationDegree,AppearedTP,DisappearedTP,"
         "AppearedFP,DisappearedFP,AppearedHallucinatedP,DisappearedHallucinatedP,AppearedFN,"
         "DisappearedFN,AppearedTN,DisappearedTN,AppearedMissedP,DisappearedMissedP,NumObjDetected,"
         "NumObjMissed,NumObjHallucinated";
  return header.str();
}

std::string ObjectEvaluator::writeMetrics(const std::string& name,
                                          const uint64_t query_time,
                                          const SegmentationMetrics& segmentation,
                                          const ChangeMetrics& changes,
                                          const DetectionMetrics& detections) const {
  std::stringstream result;
  result << "\n"
         << name << "," << query_time << "," << gt_dsg_.objects.size() << ","
         << eval_dsg_.objects.size() << "," << gt_dsg_.num_objects_invalid << ","
         << eval_dsg_.num_objects_invalid << "," << segmentation.num_oversegmented << ","
         << segmentation.num_undersegmented << "," << segmentation.num_correct << ","
         << segmentation.mean_oversegmentation_degree << ","
         << segmentation.mean_undersegmentation_degree << ","
         << segmentation.max_oversegmentation_degree << ","
         << segmentation.max_undersegmentation_degree << "," << changes.appeared_tp << ","
         << changes.disappeared_tp << "," << changes.appeared_fp << "," << changes.disappeared_fp
         << "," << changes.appeared_hallucinated_p << "," << changes.disappeared_hallucinated_p
         << "," << changes.appeared_fn << "," << changes.disappeared_fn << ","
         << changes.appeared_tn << "," << changes.disappeared_tn << "," << changes.appeared_missed_p
         << "," << changes.disappeared_missed_p << "," << detections.num_detected << ","
         << detections.num_missed << "," << detections.num_hallucinated;
  return result.str();
}

std::unordered_map<NodeId, int> ObjectEvaluator::computeAssociationCardinality(
    bool from_gt,
    const uint64_t query_time) const {
  std::unordered_map<NodeId, int> num_associations;
  for (const auto& [from, to] : (from_gt ? gt_to_dsg_ : dsg_to_gt_).associations) {
    // Check for presence of either object.
    if (query_time != 0) {
      const auto& from_attrs = (from_gt ? gt_dsg_ : eval_dsg_).objects.at(from);
      if (from_attrs.last_observed_ns.front() < query_time ||
          from_attrs.first_observed_ns.front() > query_time) {
        continue;
      }

      const auto& to_attrs = (from_gt ? eval_dsg_ : gt_dsg_).objects.at(to);
      if (to_attrs.last_observed_ns.front() < query_time ||
          to_attrs.first_observed_ns.front() > query_time) {
        continue;
      }
    }
    num_associations[to] += 1;
  }

  return num_associations;
}

bool ObjectEvaluator::setupVisFiles(const std::string& vis_dir) {
  vis_obj_file_.open(vis_dir + "/objects.csv");
  if (!vis_obj_file_.is_open()) {
    LOG(ERROR) << "Could not open '" << vis_dir << "/objects.csv' for writing.";
    return false;
  }
  vis_assoc_file_.open(vis_dir + "/associations.csv");
  if (!vis_assoc_file_.is_open()) {
    LOG(ERROR) << "Could not open '" << vis_dir << "/associations.csv' for writing.";
    vis_obj_file_.close();
    return false;
  }

  // Write headers.
  vis_obj_file_
      << "MapName,QueryTime,IsGT,ID,Label,CentroidX,CentroidY,CentroidZ,BBPosX,BBPosY,BBPosZ,"
         "BBDimX,BBDimY,BBDimZ,Present,HasAppeared,HasDisappeared"
      << std::endl;
  vis_assoc_file_ << "MapName,QueryTime,FromGT,FromID,ToID" << std::endl;

  return true;
}

void ObjectEvaluator::saveVisData(const std::string& name, const uint64_t query_time) {
  if (!eval_vis_setup_) {
    return;
  }

  // Save the objects.
  for (const auto& [id, attrs] : gt_dsg_.objects) {
    saveVisObject(name, query_time, id, attrs, true);
  }
  for (const auto& [id, attrs] : eval_dsg_.objects) {
    saveVisObject(name, query_time, id, attrs, false);
  }

  // Save associations.
  for (const auto& [from, to] : dsg_to_gt_.associations) {
    vis_assoc_file_ << name << "," << query_time << ",0," << NodeSymbol(from) << ","
                    << NodeSymbol(to) << std::endl;
  }
  for (const auto& from : dsg_to_gt_.failed) {
    vis_assoc_file_ << name << "," << query_time << ",0," << NodeSymbol(from) << ",Failed"
                    << std::endl;
  }
  for (const auto& [from, to] : gt_to_dsg_.associations) {
    vis_assoc_file_ << name << "," << query_time << ",1," << NodeSymbol(from) << ","
                    << NodeSymbol(to) << std::endl;
  }
  for (const auto& from : gt_to_dsg_.failed) {
    vis_assoc_file_ << name << "," << query_time << ",1," << NodeSymbol(from) << ",Failed"
                    << std::endl;
  }
}

void ObjectEvaluator::saveVisObject(const std::string& name,
                                    const uint64_t query_time,
                                    const NodeId id,
                                    const KhronosObjectAttributes& attrs,
                                    bool is_gt) {
  const bool present = isPresent(attrs, query_time);
  const bool appeared = hasAppeared(attrs, query_time);
  const bool disappeared = hasDisappeared(attrs, query_time);
  const Point centroid = computeSurfaceCentroid(attrs);
  vis_obj_file_ << name << "," << query_time << "," << is_gt << "," << NodeSymbol(id) << ","
                << attrs.semantic_label << "," << centroid.x() << "," << centroid.y() << ","
                << centroid.z() << "," << attrs.bounding_box.world_P_center.x() << ","
                << attrs.bounding_box.world_P_center.y() << ","
                << attrs.bounding_box.world_P_center.z() << "," << attrs.bounding_box.dimensions.x()
                << "," << attrs.bounding_box.dimensions.y() << ","
                << attrs.bounding_box.dimensions.z() << "," << present << "," << appeared << ","
                << disappeared << std::endl;
}
}  // namespace khronos
