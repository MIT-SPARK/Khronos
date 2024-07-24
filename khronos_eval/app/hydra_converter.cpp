
#include <filesystem>
#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <khronos/common/common_types.h>
#include <khronos_ros/experiments/experiment_directory.h>

using namespace khronos;

std::vector<std::string> getMapNames2(const std::string& directory) {
  std::vector<std::string> map_names;
  const std::string map_dir = directory + "/maps";
  if (!std::filesystem::is_directory(map_dir)) {
    return map_names;
  }
  for (const auto& entry : std::filesystem::directory_iterator(map_dir)) {
    if (entry.is_directory()) {
      map_names.emplace_back(entry.path().filename().string());
    }
  }
  std::sort(map_names.begin(), map_names.end());
  return map_names;
}

/**
 * @brief reconcile all maps of an experiment directory
 */
int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Check arguments.
  if (argc != 2 || argv[1] == std::string("--help") || argv[1] == std::string("-h")) {
    std::cout << "Usage: " << argv[0] << " <target_directory>" << std::endl;
    return 0;
  }

  // Read all input map names.
  const std::vector<std::string> map_names = getMapNames2(argv[1]);
  std::cout << "Found " << map_names.size() << " maps." << std::endl;
  if (map_names.empty()) {
    std::cout << "No maps found in '" << argv[1] << "'." << std::endl;
    return 0;
  }

  // Process all maps.
  for (const auto& map_name : map_names) {
    // Load panoptic map.
    std::cout << "Loading map " << map_name << " ..." << std::endl;
    auto hydra_dsg =
        DynamicSceneGraph::load(std::string(argv[1]) + "/maps/" + map_name + "/dsg_with_mesh");
    const auto& hydra_vertices = hydra_dsg->mesh()->points;

    // Setup DSG.
    DynamicSceneGraph::LayerIds layer_ids;
    for (const auto& layer :
         {DsgLayers::OBJECTS, DsgLayers::PLACES, DsgLayers::ROOMS, DsgLayers::BUILDINGS}) {
      layer_ids.push_back(layer);
    }
    khronos::DynamicSceneGraph output_dsg(layer_ids);
    output_dsg.setMesh(std::make_unique<spark_dsg::Mesh>());
    auto& bg_mesh = *output_dsg.mesh();

    // Parse all objects in hydra dsg.
    size_t num_objects = 0;
    std::unordered_set<size_t> object_vertices;
    const auto& layer = hydra_dsg->getLayer(DsgLayers::OBJECTS);
    for (const auto& [node_id, node] : layer.nodes()) {
      const auto hydra_attrs = node->attributes<spark_dsg::ObjectNodeAttributes>();

      // Create the new parsed object.
      auto attrs = std::make_unique<KhronosObjectAttributes>();
      attrs->semantic_label = hydra_attrs.semantic_label;
      attrs->color = hydra_attrs.color;
      NodeSymbol object_symbol('O', num_objects++);
      attrs->name = hydra_attrs.name;
      attrs->is_active = false;
      attrs->first_observed_ns.emplace_back(0);
      attrs->last_observed_ns.emplace_back(std::numeric_limits<uint64_t>::max());

      Point min = Point::Constant(std::numeric_limits<float>::max());
      Point max = Point::Constant(std::numeric_limits<float>::lowest());
      for (const auto& idx : hydra_attrs.mesh_connections) {
        object_vertices.insert(idx);
        const auto& point = hydra_vertices[idx];
        attrs->mesh.points.emplace_back(point);
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
      }
      for (auto& point : attrs->mesh.points) {
        point -= min;
      }

      attrs->bounding_box = spark_dsg::BoundingBox(min, max);
      attrs->position = ((min + max) / 2).cast<double>();
      output_dsg.emplaceNode(DsgLayers::OBJECTS, object_symbol, std::move(attrs));
    }

    // Parse the background.
    bg_mesh.points.reserve(hydra_vertices.size() - object_vertices.size());
    for (size_t i = 0; i < hydra_vertices.size(); ++i) {
      if (!object_vertices.count(i)) {
        bg_mesh.points.emplace_back(hydra_vertices[i]);
      }
    }
    std::cout << "Parsed " << num_objects << " objects and " << bg_mesh.points.size()
              << " background vertices." << std::endl;

    // Emulate the pipeline evaluation setup.
    const std::string output_dir = std::string(argv[1]) + "/pipeline/Hydra/Hydra";
    const std::string output_file_name = output_dir + "/" + map_name + "_reconciled_dsg";
    std::filesystem::create_directories(output_dir);
    output_dsg.save(output_file_name, true);
    std::cout << "Saved '" << output_file_name << "'." << std::endl;
  }

  return 0;
}
