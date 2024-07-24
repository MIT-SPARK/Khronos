#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <hydra_ros/visualizer/config_manager.h>
#include <khronos/common/common_types.h>
#include <khronos_msgs/KhronosEvalVisConfig.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
  // Dynamic config.
  using Config = hydra::ConfigWrapper<khronos_msgs::KhronosEvalVisConfig>;

  // Construction.
  explicit EvalVisualizer(const ros::NodeHandle& nh);
  virtual ~EvalVisualizer() = default;

  // Run.
  void spin();

 private:
  Config config;

  // ROS.
  ros::NodeHandle nh_;
  ros::Publisher centroid_gt_pub_;
  ros::Publisher centroid_dsg_pub_;
  ros::Publisher bbox_gt_pub_;
  ros::Publisher bbox_dsg_pub_;
  ros::Publisher association_pub_;
  ros::Publisher object_id_gt_pub_;
  ros::Publisher object_id_dsg_pub_;

  // Members.
  std::mutex mutex_;

  // Data to visualize. data_[robot_time_idx][query_time_idx] = data.
  std::vector<std::vector<VisData>> data_;
  // The time stamps of the data. times_[idx] = timestamp.
  std::vector<uint64_t> times_;
  std::vector<float> times_seconds_;

  // tracking data.
  int previous_robot_time_idx_ = -1;
  int previous_query_time_idx_ = -1;
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
  std_msgs::Header header_;
  std::unique_ptr<visualization_msgs::Marker> centroid_gt_marker_;
  std::unique_ptr<visualization_msgs::Marker> centroid_dsg_marker_;
  std::unique_ptr<visualization_msgs::Marker> bbox_gt_marker_;
  std::unique_ptr<visualization_msgs::Marker> bbox_dsg_marker_;
  std::unique_ptr<visualization_msgs::MarkerArray> association_marker_;
  std::unique_ptr<visualization_msgs::MarkerArray> object_id_gt_marker_;
  std::unique_ptr<visualization_msgs::MarkerArray> object_id_dsg_marker_;
};

void declare_config(EvalVisualizer::Config& config);

}  // namespace khronos
