#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/utils/nearest_neighbor_utilities.h>
namespace khronos {
/**
 * @brief Class to evaluate a mesh against a ground truth mesh.
 */
class MeshEvaluator {
 public:
  // Config.
  struct Config {
    // Number of threads to use for evaluation.
    int num_threads = std::thread::hardware_concurrency();

    // Number of vertices to process in parallel in each batch.
    size_t vertex_batch_size = 1000;

    // Thresholds to evaluate the accuracy and copleteness of the mesh at in meters. This is also
    // the truncation distances to evaluate the RMSE, MAD, and Chamfer distance at in meters.
    std::vector<float> thresholds = {0.1};

    // TODO(lschmid): add option to color the mesh by the error.
  } const config;

  // Construction.
  explicit MeshEvaluator(const Config& config);
  virtual ~MeshEvaluator() = default;

  // Evaluation interface.
  /**
   * @brief Load the ground truth mesh from file.
   * @param file_path Full path to the .ply file.
   * @returns True if the file was loaded successfully. False otherwise.
   */
  bool loadGroundTruthMesh(const std::string& file_path);

  /**
   * @brief Set the ground truth cloud to evaluate against.
   * @param points The ground truth background cloud.
   */
  bool setGroundTruthCloud(const Points& points);

  /**
   * @brief Load the DSG mesh from file.
   * @param file_path Full path to the dsg with mesh file.
   * @returns True if the file was loaded successfully. False otherwise.
   */
  bool loadDSG(const std::string& file_path);

  /**
   * @brief Set the DSG to evaluate.
   * @param dsg The DSG to evaluate.
   */
  bool setDSG(DynamicSceneGraph::Ptr dsg);

  /**
   * @brief Evaluate the loaded dsg using the loaded groundtruth as specified in the request.
   * @param name Name for the evaluation entry.
   * @param output_file Full path to the output file to write the results to. If the file exists and
   * is compatible with the current request, the results will be appended.
   * @returns True if the evaluation was successful. False otherwise.
   */
  bool evaluate(const std::string& name, const std::string& output_file);

 protected:
  // Evaluation utilities.
  // Intermediate metrics to accumulate results.
  struct RawMetrics {
    explicit RawMetrics(size_t num_thresholds)
        : inliers(num_thresholds, 0),
          outliers(num_thresholds, 0),
          acummuluated_distances(num_thresholds, 0.),
          acummuluated_distances_squared(num_thresholds, 0.) {}

    // Number of points within the truncation threshold.
    std::vector<uint64_t> inliers;

    // Number of points outside the truncation threshold.
    std::vector<uint64_t> outliers;

    // Accumulated absolute distances for each truncation threshold.
    std::vector<double> acummuluated_distances;

    // Accumulated  squared distances for each truncation threshold.
    std::vector<double> acummuluated_distances_squared;

    // Number of points for which no nearest neighbor was found.
    uint64_t num_failed_lookups = 0;

    void merge(const RawMetrics& other);
  };

  // Final metrics.
  struct Metrics {
    // Accuracy and completeness for each accuracy threshold.
    std::vector<float> accuracy;
    std::vector<float> completeness;

    // RMSE, MAD, and Chamfer distance for each truncation threshold.
    std::vector<float> rmse;
    std::vector<float> mad;
    std::vector<float> chamfer;

    // Number of points considered for evaluation in the ground truth and the DSG.
    std::vector<uint64_t> gt_inliers;
    std::vector<uint64_t> dsg_inliers;
    std::vector<uint64_t> gt_outliers;
    std::vector<uint64_t> dsg_outliers;
    uint64_t gt_failed_lookups;
    uint64_t dsg_failed_lookups;
  };

  // Get the next batch of vertices to process for each thread.
  bool getBatch(size_t& start, size_t& stop);

  // Evaluate the the vertices vs the the data in the search in parallel.
  RawMetrics evaluate(const Points& vertices, const hydra::PointNeighborSearch& search);

  // Evaluation function of each thread.
  RawMetrics evaluatePart(const Points& vertices, const hydra::PointNeighborSearch& search);

  // Compute the final metrics from the intermediate metrics.
  Metrics computeResults(const RawMetrics& dsg_to_gt, const RawMetrics& gt_to_dsg) const;

  // Writing to csv file.
  std::string writeHeader() const;
  std::string writeMetrics(const std::string& name, const Metrics& metrics) const;

  void log(const std::string& message) const {
    std::cout << "[Mesh Evaluator] " << message << std::endl;
  }
  void logError(const std::string& message) const {
    std::cout << "\033[31mERROR: [Mesh Evaluator] " << message << "\033[0m" << std::endl;
  }

 private:
  // Ground truth data.
  Points gt_mesh_vertices_;
  std::unique_ptr<hydra::PointNeighborSearch> gt_mesh_search_;

  // Current DSG to be evaluated.
  Points dsg_mesh_vertices_;
  std::unique_ptr<hydra::PointNeighborSearch> dsg_mesh_search_;

  // Information for batching.
  size_t current_vertex_index_;
  size_t maximum_vertex_index_;
  std::mutex index_mutex_;

  // Cached data.
  std::vector<float> thresholds_squared_;
};

void declare_config(MeshEvaluator::Config& config);

}  // namespace khronos
