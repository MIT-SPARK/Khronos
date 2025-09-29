#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <ianvs/node_handle.h>
#include <config_utilities/dynamic_config.h>
#include <khronos/common/common_types.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace khronos {

// Data structure to keep all visualization data for a single object.
struct VisObject {
  Point centroid;
  BoundingBox bbox;
  std::string id;
  int label;
  bool is_present;
  bool has_appeared;
  bool has_disappeared;
};

// The complete state at a specific time combination.
struct VisData {
  std::unordered_map<std::string, VisObject> gt_objects;
  std::unordered_map<std::string, VisObject> dsg_objects;
  std::unordered_map<std::string, std::string> gt_to_dsg;
  std::unordered_map<std::string, std::string> dsg_to_gt;
};

class EvalVisualizer {
 public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  struct Config {
    enum class ColorMode : int {
      Label = 0,
      Presence = 1,
      ChangeState = 2,
    } object_color = ColorMode::Label;
    size_t robot_time = 0;
    size_t query_time = 0;
    double z_offset = 5.0;
    double bbox_scale = 0.03;
    double centroid_scale = 0.3;
    double id_scale = 0.3;
    double association_scale = 0.05;
    std::filesystem::path visualization_directory;
  };

  EvalVisualizer(const Config& config, ianvs::NodeHandle nh);
  virtual ~EvalVisualizer() = default;

  void start();

 private:
  config::DynamicConfig<Config> config;

  // ROS.
  ianvs::NodeHandle nh_;
  ianvs::NodeHandle::Timer timer_;
  rclcpp::Publisher<Marker>::SharedPtr centroid_gt_pub_;
  rclcpp::Publisher<Marker>::SharedPtr centroid_dsg_pub_;
  rclcpp::Publisher<Marker>::SharedPtr bbox_gt_pub_;
  rclcpp::Publisher<Marker>::SharedPtr bbox_dsg_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr association_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr object_id_gt_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr object_id_dsg_pub_;

  // Members.
  std::mutex mutex_;

  // Data to visualize. data_[robot_time_idx][query_time_idx] = data.
  std::vector<std::vector<VisData>> data_;
  // The time stamps of the data. times_[idx] = timestamp.
  std::vector<uint64_t> times_;
  std::vector<float> times_seconds_;

  // tracking data.
  size_t previous_robot_time_idx_ = 0;
  size_t previous_query_time_idx_ = 0;
  size_t num_previous_id_gt_markers_ = 0;
  size_t num_previous_id_dsg_markers_ = 0;

  // Helper functions.
  void loadData();
  void draw();
  void reset();

  void drawCentroids(const VisData& data);
  void drawBboxes(const VisData& data);
  void drawAssociations(const VisData& data);
  void drawObjectIds(const VisData& data);

  std::function<Color(const VisObject&)> getObjectColoringFunction() const;

  // The cached messages to visualize.
  std_msgs::msg::Header header_;
  std::unique_ptr<Marker> centroid_gt_marker_;
  std::unique_ptr<Marker> centroid_dsg_marker_;
  std::unique_ptr<Marker> bbox_gt_marker_;
  std::unique_ptr<Marker> bbox_dsg_marker_;
  std::unique_ptr<MarkerArray> association_marker_;
  std::unique_ptr<MarkerArray> object_id_gt_marker_;
  std::unique_ptr<MarkerArray> object_id_dsg_marker_;
};

void declare_config(EvalVisualizer::Config& config);

}  // namespace khronos
