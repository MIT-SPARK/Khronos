#include "khronos_eval/ground_truth/tesse_ground_truth_builder.h"

#include <filesystem>

#include <config_utilities/parsing/yaml.h>
#include <config_utilities/types/path.h>
#include <hydra/common/robot_prefix_config.h>
#include <khronos/utils/geometry_utils.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <spark_dsg/colormaps.h>

#include "khronos_eval/utils/io_utils.h"

namespace khronos {

void declare_config(TesseGroundTruthBuilder::Config& config) {
  using namespace config;
  name("TesseGroundTruthBuilder");
  field<Path>(config.ground_truth_cloud_file, "ground_truth_cloud_file");
  field<Path>(config.semantic_colors_file, "semantic_colors_file");
  field<Path>(config.label_space_file, "label_space_file");
  field<Path>(config.label_remap_file, "label_remap_file");
  field<Path>(config.khronos_observed_dsg_file, "khronos_observed_dsg_file");

  field(config.min_object_separation, "min_object_separation", "m");
  field(config.unknown_label_is_background, "unknown_label_is_background");
  field(config.max_observation_distance, "max_observation_distance", "m");

  field<Path>(config.output_directory, "output_directory");
  field(config.save_dsgs, "save_dsgs");
  field(config.save_ply_clouds, "save_ply_clouds");

  check<Path::IsFile>(config.ground_truth_cloud_file, "ground_truth_cloud_file");
  check<Path::HasExtension>(config.ground_truth_cloud_file, ".ply", "ground_truth_cloud_file");
  check<Path::IsFile>(config.semantic_colors_file, "semantic_colors_file");
  check<Path::HasExtension>(config.semantic_colors_file, ".csv", "semantic_colors_file");
  check<Path::IsFile>(config.label_space_file, "label_space_file");
  check<Path::HasExtension>(config.label_space_file, ".yaml", "label_space_file");
  check<Path::IsFile>(config.label_remap_file, "label_remap_file");
  check<Path::HasExtension>(config.label_remap_file, ".yaml", "label_remap_file");
  check<Path::IsFile>(config.khronos_observed_dsg_file, "khronos_observed_dsg_file");
  check<Path::IsSet>(config.output_directory, "output_directory");
  check(config.min_object_separation, GT, 0.f, "min_object_separation");
  check(config.max_observation_distance, GT, 0.f, "max_observation_distance");
  checkCondition(config.save_dsgs || config.save_ply_clouds,
                 "No output will be generated. Set 'save_dsgs' and/or 'save_ply_clouds' to true.");
}

TesseGroundTruthBuilder::TesseGroundTruthBuilder(const Config& config)
    : config(config::checkValid(config)),
      color_map_(*hydra::SemanticColorMap::fromCsv(config.semantic_colors_file)),
      label_space_(config::fromYamlFile<hydra::LabelSpaceConfig>(config.label_space_file)),
      label_remapper_(config.label_remap_file) {}

void TesseGroundTruthBuilder::run() {
  // General.
  std::cout << "Running TESSE/Khronos ground truth extraction." << std::endl;
  setupDsg();

  // Load the objects cloud.
  Points vertices;
  Labels labels;
  Labels is_object;
  if (!loadPoints(vertices, labels, is_object)) {
    return;
  }

  // Separate cloud into by semantics.
  SemanticPointGroups point_groups = splitLabels(vertices, labels, is_object);

  // Add background to DSG.
  setDsgBackground(point_groups.front().first);
  point_groups.erase(point_groups.begin());

  // Add objects to DSG.
  extractObjects(point_groups);

  // TODO(lschmid): All the other things for a specific run.
  // Condition on GT observations.

  // Remove objects (and BG) that are never observed.
  pruneUnobservedAreas();

  // Add seen timestamps?

  // Somehow add the true presence timestamps.

  // Add the trajectory.
  extractTrajectory();

  // Store the final DSG.
  saveOutput();
  std::cout << "Ground truth extraction completed." << std::endl;
}

void TesseGroundTruthBuilder::pruneUnobservedAreas() {
  std::cout << "Pruning unobserved areas..." << std::endl;

  // Setup Neighbor search.
  Points observed_points;
  const auto& observed_vertices = observed_dsg_->mesh()->points;
  observed_points.insert(observed_points.end(), observed_vertices.begin(), observed_vertices.end());
  const hydra::PointNeighborSearch search(observed_points);
  const float max_distance_sq = config.max_observation_distance * config.max_observation_distance;

  // Prune background.
  auto& background = dsg_->mesh()->points;
  const size_t num_previous_points = background.size();
  background.erase(std::remove_if(background.begin(),
                                  background.end(),
                                  [&search, max_distance_sq](const spark_dsg::Mesh::Pos& point) {
                                    float distance_sq;
                                    size_t index;
                                    search.search(point, distance_sq, index);
                                    return distance_sq > max_distance_sq;
                                  }),
                   background.end());
  std::cout << " - Pruned " << num_previous_points - background.size() << " ("
            << static_cast<double>(num_previous_points - background.size()) / num_previous_points *
                   100
            << "%) background points." << std::endl;

  // Prune objects.
  const auto& layer = dsg_->getLayer(DsgLayers::OBJECTS);
  std::vector<NodeId> to_remove;
  const size_t num_previous_objects = layer.nodes().size();
  for (const auto& [id, node] : layer.nodes()) {
    const auto& attrs = node->attributes<KhronosObjectAttributes>();
    bool found = false;
    // Currently require only a single observation.
    for (const auto& vertex : attrs.mesh.points) {
      float distance_sq;
      size_t index;
      const auto vertex_world = attrs.bounding_box.pointToWorldFrame(vertex);
      search.search(vertex_world, distance_sq, index);
      if (distance_sq <= max_distance_sq) {
        found = true;
        break;
      }
    }
    if (!found) {
      to_remove.push_back(id);
    }
  }

  for (const auto id : to_remove) {
    dsg_->removeNode(id);
  }
  std::cout << " - Pruned " << to_remove.size() << " ("
            << static_cast<double>(to_remove.size()) / num_previous_objects * 100 << "%) objects."
            << std::endl;
}

void TesseGroundTruthBuilder::extractTrajectory() {
  return;
  const hydra::RobotPrefixConfig config;
  const auto& layer = observed_dsg_->getLayer(
      observed_dsg_->getLayerKey(DsgLayers::AGENTS).value().layer, config.key);
  size_t num_poses = 0;
  for (const auto& [node_id, node] : layer.nodes()) {
    // TODO(lschmid): This does not quite end up in the right place of the DSG. Skip for now.
    dsg_->emplaceNode(layer.id, node->id, node->attributes().clone());
    num_poses++;
  }
  std::cout << "Wrote " << num_poses << " agent poses to DSG." << std::endl;
}

void TesseGroundTruthBuilder::saveOutput() const {
  // Make sure output directory exists.
  std::cout << "Saving output data to '" << config.output_directory << "'..." << std::endl;
  if (config.save_dsgs) {
    std::filesystem::create_directories(config.output_directory);
    // Save the final DSG.
    const std::string dsg_with_mesh = config.output_directory + "/gt_dsg_with_mesh.json";
    dsg_->save(dsg_with_mesh, true);
    std::cout << " - Saved " << dsg_with_mesh << "'." << std::endl;

    const std::string dsg_without_mesh = config.output_directory + "/gt_dsg.json";
    dsg_->save(dsg_without_mesh, false);
    std::cout << " - Saved " << dsg_without_mesh << "'." << std::endl;
  }

  if (config.save_ply_clouds) {
    // Write the pointclouds as PLY files.
    pcl::PLYWriter writer;

    // Write BG.
    pcl::PointCloud<pcl::PointXYZRGBA> points;
    points.resize(dsg_->mesh()->numVertices());
    for (size_t i = 0; i < dsg_->mesh()->numVertices(); ++i) {
      const auto& vertex = dsg_->mesh()->pos(i);
      const auto& color = dsg_->mesh()->color(i);
      auto& new_vertex = points[i];
      new_vertex.x = vertex.x();
      new_vertex.y = vertex.y();
      new_vertex.z = vertex.z();
      new_vertex.r = color.r;
      new_vertex.g = color.g;
      new_vertex.b = color.b;
      new_vertex.a = color.a;
    }
    pcl::PCLPointCloud2 pcl_cloud;
    pcl::toPCLPointCloud2(points, pcl_cloud);
    const std::string background = config.output_directory + "/gt_background.ply";
    writer.writeBinary(background, pcl_cloud);
    std::cout << " - Saved '" << background << "'." << std::endl;

    // Write objects, colored by instance.
    points.clear();
    size_t ids_per_revolution = 15;
    const auto& layer = dsg_->getLayer(DsgLayers::OBJECTS);
    for (const auto& [id, node] : layer.nodes()) {
      const auto& attrs = node->attributes<KhronosObjectAttributes>();
      points.reserve(points.size() + attrs.mesh.numVertices());
      const auto color =
          spark_dsg::colormaps::rainbowId(NodeSymbol(id).categoryId(), ids_per_revolution);
      for (size_t i = 0; i < attrs.mesh.numVertices(); ++i) {
        auto& new_vertex = points.emplace_back();
        const auto& vertex = attrs.mesh.pos(i);
        const auto vertex_world = attrs.bounding_box.pointToWorldFrame(vertex);
        new_vertex.x = vertex_world.x();
        new_vertex.y = vertex_world.y();
        new_vertex.z = vertex_world.z();
        new_vertex.r = color.r;
        new_vertex.g = color.g;
        new_vertex.b = color.b;
        new_vertex.a = color.a;
      }
    }
    pcl::toPCLPointCloud2(points, pcl_cloud);
    const std::string objects = config.output_directory + "/gt_objects.ply";
    writer.writeBinary(objects, pcl_cloud);
    std::cout << " - Saved '" << objects << "'." << std::endl;
  }
}

void TesseGroundTruthBuilder::extractObjects(const SemanticPointGroups& point_groups) {
  num_objects_ = 0;
  std::cout << "Extracting objects..." << std::endl;
  for (const auto& [points, label] : point_groups) {
    // Check if it is a valid object.
    if (label == kInvalidLabel) {
      std::cout << " - Found " << points.size()
                << " points with unknown semantic (color) label. These will be ignored."
                << std::endl;
      continue;
    }
    // Split the cluster into points.
    std::vector<Points> clusters = euclideanClustering(points);
    std::cout << " - Label " << label << ": split " << points.size() << " points into "
              << clusters.size() << " clusters." << std::endl;

    for (const Points& cluster : clusters) {
      // Add the object to the DSG.
      addDsgObject(cluster, label);
    }
  }
  std::cout << "Extracted " << num_objects_ << " objects." << std::endl;
}

void TesseGroundTruthBuilder::addDsgObject(const Points& vertices, const Label label) {
  NodeSymbol symbol('O', num_objects_);
  num_objects_++;
  KhronosObjectAttributes attrs;

  attrs.name = symbol.str();
  attrs.semantic_label = label;

  attrs.color = color_map_.getColorFromLabel(label);

  // TODO(marcus): check if you need this
  khronos::Point bbox_world_P_center = Point(0, 0, 0);
  for (const auto& vertex : vertices) {
    bbox_world_P_center += vertex;
  }
  bbox_world_P_center /= vertices.size();
  attrs.bounding_box = BoundingBox(vertices);
  attrs.bounding_box.world_P_center = bbox_world_P_center;
  attrs.position = bbox_world_P_center.cast<double>();

  attrs.mesh.resizeVertices(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    attrs.mesh.setPos(i, vertices[i] - bbox_world_P_center);
  }

  dsg_->addOrUpdateNode(DsgLayers::OBJECTS, symbol, attrs.clone());
}

std::vector<Points> TesseGroundTruthBuilder::euclideanClustering(const Points& points) const {
  // Use PCL for eucledean clustering within each semantic class.
  // Setup KDTree.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(points.size());
  for (const Point& point : points) {
    cloud->emplace_back(point.x(), point.y(), point.z());
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;

  // Clustering.
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(config.min_object_separation);
  ec.setMinClusterSize(0);
  ec.setMaxClusterSize(points.size());
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  // Extract the result.
  std::vector<Points> result;
  for (const auto& cluster : cluster_indices) {
    Points& new_points = result.emplace_back();
    new_points.reserve(cluster.indices.size());
    for (const auto& idx : cluster.indices) {
      const pcl::PointXYZ& point = (*cloud)[idx];
      new_points.emplace_back(point.x, point.y, point.z);
    }
  }
  return result;
}

void TesseGroundTruthBuilder::setDsgBackground(const Points& vertices) {
  auto mesh = std::make_shared<spark_dsg::Mesh>();
  mesh->resizeVertices(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    mesh->setPos(i, vertices[i]);
  }
  dsg_->setMesh(mesh);
  std::cout << "Wrote " << mesh->numVertices() << " vertices to DSG background." << std::endl;
}

TesseGroundTruthBuilder::SemanticPointGroups TesseGroundTruthBuilder::splitLabels(
    const Points& vertices,
    const Labels& labels,
    const Labels& is_object) const {
  SemanticPointGroups result;
  // First entry is the background.
  result.emplace_back();

  std::unordered_map<uint32_t, Points> clusters;
  for (size_t i = 0; i < vertices.size(); ++i) {
    if (is_object[i]) {
      clusters[labels[i]].push_back(vertices[i]);
    } else {
      result[0].first.push_back(vertices[i]);
    }
  }

  for (const auto& cluster : clusters) {
    result.emplace_back(cluster.second, cluster.first);
  }
  std::cout << "Split " << vertices.size() << " points into " << result.size()
            << " semantic groups." << std::endl;
  return result;
}

bool TesseGroundTruthBuilder::loadPoints(Points& vertices, Labels& labels, Labels& is_object) {
  // Load the data.
  std::cout << "Loading data..." << std::endl;
  std::vector<Color> colors;
  if (!loadPlyCloud(config.ground_truth_cloud_file, vertices, colors)) {
    return false;
  }
  std::cout << " - Loaded " << vertices.size() << " ground truth points." << std::endl;

  // Load the observed DSG.
  observed_dsg_ = DynamicSceneGraph::load(config.khronos_observed_dsg_file);
  if (!observed_dsg_) {
    LOG(ERROR) << "Failed to load observed DSG '" << config.khronos_observed_dsg_file << "'.";
    return false;
  }
  std::cout << " - Loaded observed DSG with " << observed_dsg_->mesh()->numVertices() << " points."
            << std::endl;

  // Convert the colors to labels.
  labels = colorsToLabels(colors);
  std::cout << "Converted colors to labels." << std::endl;

  // Check for object and BG labels.
  is_object.resize(labels.size());
  if (config.unknown_label_is_background) {
    for (size_t i = 0; i < labels.size(); ++i) {
      is_object[i] = label_space_.isObject(labels[i]);
    }
  } else {
    // Will be filtered by object extractor.
    for (size_t i = 0; i < labels.size(); ++i) {
      is_object[i] = label_space_.isObject(labels[i]);
    }
  }
  std::cout << "Computed object and background labels." << std::endl;

  return true;
}

void TesseGroundTruthBuilder::setupDsg() {
  const std::map<std::string, spark_dsg::LayerKey> layers = {{DsgLayers::OBJECTS, 2},
                                                             {DsgLayers::PLACES, 3},
                                                             {DsgLayers::ROOMS, 4},
                                                             {DsgLayers::BUILDINGS, 5}};
  dsg_ = DynamicSceneGraph::fromNames(layers);
}

TesseGroundTruthBuilder::Labels TesseGroundTruthBuilder::colorsToLabels(
    const Colors& colors) const {
  Labels labels;
  labels.reserve(colors.size());
  for (const Color& color : colors) {
    const auto label = color_map_.getLabelFromColor(color);
    labels.push_back(label ? *label : kInvalidLabel);
  }
  return labels;
}

}  // namespace khronos
