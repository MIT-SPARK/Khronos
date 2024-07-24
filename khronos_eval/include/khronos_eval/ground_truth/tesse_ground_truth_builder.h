#pragma once

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/common/semantic_color_map.h>
#include <hydra/utils/nearest_neighbor_utilities.h>
namespace khronos {
/**
 * @brief That loads the outputs of the tesse simulator and puts them into a ground truth DSG.
 */
class TesseGroundTruthBuilder {
 public:
  // Config.
  struct Config {
    // PLY file containing a point cloud of only the complete ground truth mesh colored by
    // semantics.
    std::string ground_truth_cloud_file;

    // CSV file to parse the semantic colors to semantic IDs.
    std::string semantic_colors_file;

    // CSV file specifying which lables belong to objects or background in khronos.
    std::string khronos_panoptic_file;

    // JSON file containing a high-resolution mesh of the entire observed scene with timestamps and
    // GT poses to adust the ground truth to the experiment trajectory.
    std::string khronos_observed_dsg_file;

    // Where to save the final DSG and other results.
    std::string output_directory;

    // Minimum distance between objects in the input cloud in meters. This distance should be larger
    // than the distance between surface points of the same object.
    float min_object_separation = 0.1f;

    // If true add points with unknown colors to the background. If false exclude them from the DSG.
    bool unknown_label_is_background = true;

    // If true save the resulting DSG with and without mesh.
    bool save_dsgs = true;

    // If true additionally store the objects and background as PLY files, coloredby instances.
    bool save_ply_clouds = true;

    // Maximum distance between observed and GT point to count as observed in meters.
    float max_observation_distance = 0.1;
  } const config;

  // Types.
  using Colors = std::vector<Color>;
  using Label = uint32_t;
  using Labels = std::vector<Label>;
  using SemanticPointGroup = std::pair<Points, Label>;  // vertices, label
  using SemanticPointGroups = std::vector<SemanticPointGroup>;

  // Construction.
  explicit TesseGroundTruthBuilder(const Config& config);
  virtual ~TesseGroundTruthBuilder() = default;

  // Interface interface.
  void run();

 protected:
  // Helper functions.
  // Setup the DSG that will contain the groundtruth data.
  void setupDsg();
  // Load all objects from the objects only cloud.
  bool loadPoints(Points& vertices, Labels& labels, Labels& is_object);
  SemanticPointGroups splitLabels(const Points& vertices,
                                  const Labels& labels,
                                  const Labels& is_object) const;
  Labels colorsToLabels(const Colors& colors) const;
  void setDsgBackground(const Points& vertices);
  void extractObjects(const SemanticPointGroups& point_groups);
  std::vector<Points> euclideanClustering(const Points& points) const;
  void addDsgObject(const Points& vertices, const Label label);
  void saveOutput() const;
  void extractTrajectory();
  void pruneUnobservedAreas();

 private:
  // The ground truth DSG being built.
  DynamicSceneGraph::Ptr dsg_;
  DynamicSceneGraph::Ptr observed_dsg_;

  // The semantic colors to semantic labels mapping.
  hydra::SemanticColorMap label_map_;

  // Variables.
  size_t num_objects_ = 0;

  // Constants.
  static constexpr Label kInvalidLabel = std::numeric_limits<uint32_t>::max();
};

void declare_config(TesseGroundTruthBuilder::Config& config);

}  // namespace khronos
