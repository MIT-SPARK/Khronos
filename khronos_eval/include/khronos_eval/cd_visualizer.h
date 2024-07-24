#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <hydra_ros/visualizer/config_manager.h>
#include <khronos/backend/change_detection/ray_verificator.h>
#include <khronos/common/common_types.h>
#include <khronos_msgs/KhronosChangeDetectionVisConfig.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace khronos {

class ChangeDetectionVisualizer {
 public:
  // configs.
  using DynamicConfig = khronos_msgs::KhronosChangeDetectionVisConfig;
  struct StaticConfig {
    int verbosity = 4;

    // Config of the ray verificator to use.
    RayVerificator::Config ray_verificator;
  };

  // Construction.
  ChangeDetectionVisualizer(const ros::NodeHandle& nh,
                            std::shared_ptr<const DynamicSceneGraph> graph);
  virtual ~ChangeDetectionVisualizer() = default;

  // Run.
  void spin();

 private:
  // Configs.
  StaticConfig config;
  hydra::ConfigWrapper<DynamicConfig> dynamic_config_manager_;
  const DynamicConfig& dynamic_config_;

  // ROS.
  ros::NodeHandle nh_;
  ros::Publisher points_pub_;
  ros::Publisher lines_pub_;
  ros::Publisher target_pub_;
  ros::Publisher hash_pub_;

  // data.
  std::shared_ptr<const DynamicSceneGraph> graph_;
  std::unique_ptr<std::thread> spin_thread_;

  // Members.
  std::mutex mutex_;
  std::unique_ptr<RayVerificator> ray_verificator_;
  std::unique_ptr<KhronosObjectAttributes> object_;

  // tracking data.
  int previous_ray_policy_ = -1;
  int previous_object_id_ = -1;

  // Helper functions.
  void draw();
  void reset();

  void setupRayVerificator();
  void setupObject();
  void drawRays();
  void drawHashBlock();

  // The cached messages to visualize.
  std_msgs::Header header_;
  std::unique_ptr<visualization_msgs::Marker> target_marker_;
  std::unique_ptr<visualization_msgs::Marker> point_marker_;
  std::unique_ptr<visualization_msgs::Marker> line_marker_;
  std::unique_ptr<visualization_msgs::Marker> hash_marker_;
};

void declare_config(ChangeDetectionVisualizer::StaticConfig& config);

}  // namespace khronos
