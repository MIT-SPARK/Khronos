#include "khronos_eval/mesh_evaluator.h"

#include <filesystem>

#include <pcl/io/ply_io.h>

namespace khronos {

void declare_config(MeshEvaluator::Config& config) {
  using namespace config;
  name("MeshEvaluator");
  field<ThreadNumConversion>(config.num_threads, "num_threads");
  field(config.vertex_batch_size, "vertex_batch_size");
  field(config.thresholds, "thresholds", "m");
  check(config.num_threads, GT, 0, "num_threads");
  check(config.vertex_batch_size, GT, 0, "vertex_batch_size");
  checkCondition(!config.thresholds.empty(), "thresholds must not be empty.");
}

MeshEvaluator::MeshEvaluator(const Config& config) : config(config::checkValid(config)) {
  thresholds_squared_ = config.thresholds;
  for (auto& threshold : thresholds_squared_) {
    threshold *= threshold;
  }
}

bool MeshEvaluator::loadGroundTruthMesh(const std::string& file_path) {
  // Load the ground truth mesh.
  log("Loading ground truth mesh from '" + file_path + "'...");
  if (!std::filesystem::exists(file_path)) {
    logError("Ground truth file '" + file_path + "' does not exist.");
    return false;
  }

  // Load the ground truth as arbitrary polygon mesh and extract the vertices or points if it is a
  // cloud afterwards.
  pcl::PLYReader ply_reader;
  pcl::PolygonMesh gt_mesh;
  if (ply_reader.read(file_path, gt_mesh) != 0) {
    logError("Failed to load ground truth mesh '" + file_path + "'.");
    return false;
  }

  // Setup the points and neighbor search.
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(gt_mesh.cloud, vertices);
  if (vertices.empty()) {
    LOG(ERROR) << "Ground truth mesh '" << file_path << "' has no vertices.";
    return false;
  }
  gt_mesh_vertices_.resize(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    gt_mesh_vertices_[i] = vertices[i].getVector3fMap();
  }
  gt_mesh_search_ = std::make_unique<hydra::PointNeighborSearch>(gt_mesh_vertices_);
  log("Loaded ground truth mesh with " + std::to_string(gt_mesh_vertices_.size()) + " vertices.");
  return true;
}

bool MeshEvaluator::setGroundTruthCloud(const Points& points) {
  gt_mesh_vertices_ = points;
  gt_mesh_search_ = std::make_unique<hydra::PointNeighborSearch>(gt_mesh_vertices_);
  return true;
}

bool MeshEvaluator::setDSG(DynamicSceneGraph::Ptr dsg) {
  const auto& vertices = dsg->mesh()->points;
  dsg_mesh_vertices_.resize(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    dsg_mesh_vertices_[i] = vertices[i];
  }
  dsg_mesh_search_ = std::make_unique<hydra::PointNeighborSearch>(dsg_mesh_vertices_);
  return true;
}

bool MeshEvaluator::loadDSG(const std::string& file_path) {
  // Load the DSG mesh.
  log("Loading DSG from '" + file_path + "'...");
  if (!std::filesystem::exists(file_path)) {
    LOG(ERROR) << "DSG file '" << file_path << "' does not exist.";
    return false;
  }
  DynamicSceneGraph::Ptr dsg = DynamicSceneGraph::load(file_path);
  if (!dsg) {
    LOG(ERROR) << "Failed to load DSG '" << file_path << "'.";
    return false;
  }

  // Setup the points and neighbor search.
  if (!setDSG(dsg)) {
    return false;
  }
  if (dsg_mesh_vertices_.empty()) {
    LOG(ERROR) << "DSG '" << file_path << "' has no vertices.";

    return false;
  }

  std::cout << "[Mesh Evaluator] Loaded DSG mesh with " << dsg_mesh_vertices_.size() << " vertices."
            << std::endl;
  return true;
}

bool MeshEvaluator::evaluate(const std::string& name, const std::string& output_file) {
  // Verify gt and scene graph are loaded.
  if (!gt_mesh_search_) {
    logError("No ground truth mesh is loaded.");
    return false;
  }
  if (!dsg_mesh_search_) {
    logError("No dsg mesh is loaded.");
    return false;
  }

  // Verify that the output can be setup correctly. This appends to the output file if it already
  // exists and has identical data.
  const std::string output_dir = std::filesystem::path(output_file).parent_path();
  if (!std::filesystem::exists(output_dir)) {
    if (!std::filesystem::create_directories(output_dir)) {
      logError("Failed to create output directory '" + output_dir + "'.");
      return false;
    }
  }

  const bool outfile_existed = std::filesystem::exists(output_file);
  std::fstream out_file(output_file, std::ios::in | std::ios::app);
  if (!out_file) {
    logError("Failed to open output file '" + output_file + "'.");
    return false;
  }
  const std::string header = writeHeader();
  if (outfile_existed) {
    std::string previous_header;
    out_file.seekg(std::ios_base::beg);
    getline(out_file, previous_header);
    out_file.seekp(std::ios_base::end);
    out_file.clear();
    if (previous_header.find(header) != 0) {
      logError("Output file '" + output_file +
               "' already exists but has different header, skipping.");
      return false;
    }
    log("Appending to existing output file '" + output_file + "'.");
  } else {
    out_file << header;
  }

  // Evalaute the dsg vs ground truth and vice versa in parallel.
  const RawMetrics dsg_to_gt = evaluate(dsg_mesh_vertices_, *gt_mesh_search_);
  const RawMetrics gt_to_dsg = evaluate(gt_mesh_vertices_, *dsg_mesh_search_);

  // Compute the results.
  const Metrics result = computeResults(dsg_to_gt, gt_to_dsg);

  // Write the output to file.
  out_file << writeMetrics(name, result);
  out_file.close();
  log("Finished evaluating '" + name + "'.");
  return true;
}

MeshEvaluator::RawMetrics MeshEvaluator::evaluate(const Points& vertices,
                                                  const hydra::PointNeighborSearch& search) {
  current_vertex_index_ = 0;
  maximum_vertex_index_ = vertices.size();
  std::vector<std::future<RawMetrics>> threads;
  for (int i = 0; i < config.num_threads; ++i) {
    threads.emplace_back(
        std::async(std::launch::async, [&]() { return evaluatePart(vertices, search); }));
  }
  RawMetrics result(config.thresholds.size());
  for (auto& thread : threads) {
    result.merge(thread.get());
  }
  return result;
}

MeshEvaluator::RawMetrics MeshEvaluator::evaluatePart(const Points& vertices,
                                                      const hydra::PointNeighborSearch& search) {
  RawMetrics data(config.thresholds.size());
  size_t start, stop;
  while (getBatch(start, stop)) {
    for (size_t i = start; i < stop; ++i) {
      // Find the nearest neghbor of the vertex.
      float distance_squared;
      size_t neighbor_index;
      if (!search.search(vertices[i], distance_squared, neighbor_index)) {
        ++data.num_failed_lookups;
        continue;
      }

      // Add the distance to the data.
      for (size_t j = 0; j < thresholds_squared_.size(); ++j) {
        if (distance_squared <= thresholds_squared_[j]) {
          data.inliers[j] += 1;
          data.acummuluated_distances[j] += std::sqrt(distance_squared);
          data.acummuluated_distances_squared[j] += distance_squared;
        } else {
          data.outliers[j] += 1;
        }
      }
    }
  }
  return data;
}

MeshEvaluator::Metrics MeshEvaluator::computeResults(const RawMetrics& dsg_to_gt,
                                                     const RawMetrics& gt_to_dsg) const {
  Metrics result;

  // Compute number failed points.
  result.gt_failed_lookups = gt_to_dsg.num_failed_lookups;
  result.dsg_failed_lookups = dsg_to_gt.num_failed_lookups;

  for (size_t i = 0; i < dsg_to_gt.inliers.size(); ++i) {
    // Compute number of valid points.
    result.gt_inliers.emplace_back(gt_to_dsg.inliers[i]);
    result.dsg_inliers.emplace_back(dsg_to_gt.inliers[i]);
    result.gt_outliers.emplace_back(gt_to_dsg.outliers[i]);
    result.dsg_outliers.emplace_back(dsg_to_gt.outliers[i]);

    // Compute accuracy and completeness.
    const double acc = static_cast<double>(dsg_to_gt.inliers[i]) /
                       (result.dsg_inliers[i] + result.dsg_outliers[i]);
    const double comp =
        static_cast<double>(gt_to_dsg.inliers[i]) / (result.gt_inliers[i] + result.gt_outliers[i]);
    result.accuracy.emplace_back(acc);
    result.completeness.emplace_back(comp);

    // Compute RMSE, MAD, and Chamfer distance.
    const double rmse =
        std::sqrt(dsg_to_gt.acummuluated_distances_squared[i] / result.dsg_inliers[i]);
    const double mad = dsg_to_gt.acummuluated_distances[i] / result.dsg_inliers[i];
    const double chamfer = dsg_to_gt.acummuluated_distances[i] / result.dsg_inliers[i] +
                           gt_to_dsg.acummuluated_distances[i] / result.gt_inliers[i];
    result.rmse.emplace_back(rmse);
    result.mad.emplace_back(mad);
    result.chamfer.emplace_back(chamfer);
  }

  return result;
}

std::string MeshEvaluator::writeHeader() const {
  std::stringstream header;
  header << "Name";
  for (size_t i = 0; i < config.thresholds.size(); ++i) {
    header << ",Accuracy@" << config.thresholds[i];
  }
  for (size_t i = 0; i < config.thresholds.size(); ++i) {
    header << ",Completeness@" << config.thresholds[i];
  }
  for (size_t i = 0; i < config.thresholds.size(); ++i) {
    header << ",RMSE@" << config.thresholds[i];
  }
  for (size_t i = 0; i < config.thresholds.size(); ++i) {
    header << ",MAD@" << config.thresholds[i];
  }
  for (size_t i = 0; i < config.thresholds.size(); ++i) {
    header << ",Chamfer@" << config.thresholds[i];
  }
  for (size_t i = 0; i < config.thresholds.size(); ++i) {
    header << ",GTInliers@" << config.thresholds[i];
  }
  for (size_t i = 0; i < config.thresholds.size(); ++i) {
    header << ",GTOutliers@" << config.thresholds[i];
  }
  for (size_t i = 0; i < config.thresholds.size(); ++i) {
    header << ",DSGInliers@" << config.thresholds[i];
  }
  for (size_t i = 0; i < config.thresholds.size(); ++i) {
    header << ",DSGOutliers@" << config.thresholds[i];
  }
  header << ",NumGTFailed,NumDSGFailed";
  return header.str();
}

std::string MeshEvaluator::writeMetrics(const std::string& name, const Metrics& metrics) const {
  std::stringstream result;
  result << "\n" << name;
  for (const float acc : metrics.accuracy) {
    result << "," << acc;
  }
  for (const float comp : metrics.completeness) {
    result << "," << comp;
  }
  for (const float rmse : metrics.rmse) {
    result << "," << rmse;
  }
  for (const float mad : metrics.mad) {
    result << "," << mad;
  }
  for (const float chamfer : metrics.chamfer) {
    result << "," << chamfer;
  }
  for (const size_t inliers : metrics.gt_inliers) {
    result << "," << inliers;
  }
  for (const size_t outliers : metrics.gt_outliers) {
    result << "," << outliers;
  }
  for (const size_t inliers : metrics.dsg_inliers) {
    result << "," << inliers;
  }
  for (const size_t outliers : metrics.dsg_outliers) {
    result << "," << outliers;
  }
  result << "," << metrics.gt_failed_lookups << "," << metrics.dsg_failed_lookups;
  return result.str();
}

bool MeshEvaluator::getBatch(size_t& start, size_t& stop) {
  std::lock_guard<std::mutex> lock(index_mutex_);
  if (current_vertex_index_ >= maximum_vertex_index_) {
    return false;
  }
  start = current_vertex_index_;
  stop = std::min(current_vertex_index_ + config.vertex_batch_size, maximum_vertex_index_);
  current_vertex_index_ = stop;
  return true;
}

void MeshEvaluator::RawMetrics::merge(const RawMetrics& other) {
  for (size_t i = 0; i < inliers.size(); ++i) {
    inliers[i] += other.inliers[i];
    outliers[i] += other.outliers[i];
    acummuluated_distances[i] += other.acummuluated_distances[i];
    acummuluated_distances_squared[i] += other.acummuluated_distances_squared[i];
  }
  num_failed_lookups += other.num_failed_lookups;
}

}  // namespace khronos
