#pragma once

#include <config_utilities/config_utilities.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/scene_graph_types.h>
#include <khronos/common/common_types.h>

namespace khronos {

class GTConsolidator {
 public:
  // Config.
  struct Config {
    // Input files.
    std::string gt_dsg_with_mesh_file;
    std::string gt_dynamic_dsg_file;
    std::string gt_changes_file;

    // Output files.
    std::string output_dir;
    bool save_dsg = true;
    bool save_dsg_with_mesh = true;
  } const config;

  explicit GTConsolidator(const Config& config);
  virtual ~GTConsolidator() = default;

  // Interface.
  void run();

 protected:
  // Struct to store the changes.
  struct GtChange {
    NodeId id;
    // Timestamps the object changed. 0 will be mapped to +/- inf and ingnored.
    uint64_t appeared;
    uint64_t disappeared;
  };

  // Data.
  DynamicSceneGraph::Ptr gt_dsg_with_mesh_;
  DynamicSceneGraph::Ptr gt_dynamic_dsg_;
  std::map<NodeSymbol, std::vector<GtChange>> gt_changes_;

 private:
  // Helper functions.
  void loadData();
  void consolidateDSG();
  void saveDSG();
  std::map<NodeSymbol, std::vector<GtChange>> readChanges(
      const std::string& changes_file);

  bool is_setup_ = false;
};

void declare_config(GTConsolidator::Config& config);

}  // namespace khronos
