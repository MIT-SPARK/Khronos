#include "khronos_eval/ground_truth/gt_consolidator.h"

#include <filesystem>

#include <config_utilities/parsing/yaml.h>
#include <config_utilities/types/path.h>
#include <hydra/utils/csv_reader.h>

#include "khronos_eval/utils/io_utils.h"

namespace khronos {

void declare_config(GTConsolidator::Config& config) {
  using namespace config;
  name("GTConsolidator");
  field<Path>(config.gt_dsg_with_mesh_file, "gt_dsg_with_mesh_file");
  field<Path>(config.gt_dynamic_dsg_file, "gt_dynamic_dsg_file");
  field<Path>(config.gt_changes_file, "gt_changes_file");
  field<Path>(config.output_dir, "output_dir");

  field(config.save_dsg, "save_dsg");
  field(config.save_dsg_with_mesh, "save_dsg_with_mesh");

  check<Path::IsFile>(config.gt_dsg_with_mesh_file, "gt_dsg_with_mesh_file");
  check<Path::HasExtension>(config.gt_dsg_with_mesh_file, ".json", "gt_dsg_with_mesh_file");
  check<Path::IsFile>(config.gt_dynamic_dsg_file, "gt_dynamic_dsg_file");
  check<Path::HasExtension>(config.gt_dynamic_dsg_file, ".json", "gt_dynamic_dsg_file");
  check<Path::IsFile>(config.gt_changes_file, "gt_changes_file");
  check<Path::HasExtension>(config.gt_changes_file, ".csv", "gt_changes_file");
  check<Path::IsSet>(config.output_dir, "output_dir");
  checkCondition(config.save_dsg || config.save_dsg_with_mesh,
                 "No output will be generated. Set 'save_dsg' and/or 'save_dsg_with_mesh' to true.");
}

GTConsolidator::GTConsolidator(const Config& config) : config(config::checkValid(config)) {}

void GTConsolidator::run() {
  std::filesystem::path output_dir(config.output_dir);
  loadData();
  consolidateDSG();
  saveDSG();
}

void GTConsolidator::loadData() {
  gt_dsg_with_mesh_ = DynamicSceneGraph::load(config.gt_dsg_with_mesh_file);
  gt_dynamic_dsg_ = DynamicSceneGraph::load(config.gt_dynamic_dsg_file);
  gt_changes_ = readChanges(config.gt_changes_file);

  if (gt_dsg_with_mesh_ && gt_dynamic_dsg_ && !gt_changes_.empty()) {
    is_setup_ = true;
  }

  LOG(INFO) << "Loaded " << gt_dsg_with_mesh_->getLayer(DsgLayers::OBJECTS).nodes().size()
            << " static objects.";
  LOG(INFO) << "Loaded " << gt_dynamic_dsg_->getLayer(DsgLayers::OBJECTS).nodes().size()
            << " dynamic objects.";
  LOG(INFO) << "Loaded " << gt_changes_.size() << " ground truth changes.";
}

std::map<NodeSymbol, std::vector<GTConsolidator::GtChange>> GTConsolidator::readChanges(
    const std::string& changes_file) {
  // Read the changes file.
  if (changes_file.empty()) {
    return {};
  }

  std::map<NodeSymbol, std::vector<GTConsolidator::GtChange>> gt_changes;
  hydra::CsvReader reader(changes_file);
  if (!reader.isSetup()) {
    LOG(WARNING) << "Ground truth changes file is set but could not be read!";
    return {};
  }

  // Check the data is correct and read it.
  for (const auto& header : {"ObjectSymbol", "AppearedAt", "DisappearedAt"}) {
    if (!reader.hasHeader(header)) {
      LOG(WARNING) << "Ground truth changes file is missing header '" << header << "'.";
      return {};
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
    gt_changes[symbol].emplace_back(std::move(gt_change));
    num_changes_loaded++;
  }
  return gt_changes;
}

void GTConsolidator::consolidateDSG() {
  if (!is_setup_) {
    LOG(ERROR) << "GTConsolidator is not setup!";
    return;
  }

  LOG(INFO) << "Consolidating DSG...";
  size_t num_static_objects_invalid = 0;
  size_t num_static_objects_valid = 0;
  size_t num_dynamic_objects_invalid = 0;
  size_t num_dynamic_objects_valid = 0;

  // Add changes into the static DSG.
  auto& static_nodes = gt_dsg_with_mesh_->getLayer(DsgLayers::OBJECTS).nodes();
  for (auto& [id, node] : static_nodes) {
    // Check if the object was marked absent for some reason.
    auto attrs = node->tryAttributes<KhronosObjectAttributes>();
    if (!attrs) {
      num_static_objects_invalid++;
      continue;
    }
    num_static_objects_valid++;

    // Check the changes times.
    const auto it = gt_changes_.find(id);
    if (it == gt_changes_.end()) {
      attrs->first_observed_ns = {0};
      attrs->last_observed_ns = {std::numeric_limits<uint64_t>::max()};
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
  }

  // Add dynamic objects to the DSG.
  auto& dynamic_nodes = gt_dynamic_dsg_->getLayer(DsgLayers::OBJECTS).nodes();
  for (auto& [id, node] : dynamic_nodes) {
    auto attrs = node->tryAttributes<KhronosObjectAttributes>();
    if (!attrs || attrs->trajectory_positions.empty()) {
      num_dynamic_objects_invalid++;
      continue;
    }
    num_dynamic_objects_valid++;

    NodeId new_node_id = gt_dsg_with_mesh_->getLayer(DsgLayers::OBJECTS).nodes().size();
    if (gt_dsg_with_mesh_->hasNode(new_node_id)) {
      LOG(FATAL) << "Node " << new_node_id
                 << " already exists in static DSG! Figure out a way to deal with this.";
    }
    gt_dsg_with_mesh_->emplaceNode(DsgLayers::OBJECTS, new_node_id, std::make_unique<KhronosObjectAttributes>(*attrs));
  }

  LOG(INFO) << "Consolidated DSG.";
  LOG(INFO) << "Found " << num_static_objects_invalid << " invalid static objects.";
  LOG(INFO) << "Found " << num_static_objects_valid << " valid static objects.";
  LOG(INFO) << "Found " << num_dynamic_objects_invalid << " invalid dynamic objects.";
  LOG(INFO) << "Found " << num_dynamic_objects_valid << " valid dynamic objects.";
}

void GTConsolidator::saveDSG() {
  if (config.save_dsg) {
    gt_dsg_with_mesh_->save(config.output_dir + "/gt_dsg_consolidated.json", false);
    LOG(INFO) << "Saved DSG.";
  }
  if (config.save_dsg_with_mesh) {
    gt_dsg_with_mesh_->save(config.output_dir + "/gt_dsg_with_mesh_consolidated.json", true);
    LOG(INFO) << "Saved DSG with mesh.";
  }
}

}  // namespace khronos
