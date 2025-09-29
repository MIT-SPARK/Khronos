#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <config_utilities/dynamic_config.h>
#include <ianvs/node_handle.h>
#include <khronos/backend/change_detection/ray_verificator.h>
#include <khronos/common/common_types.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace khronos {

class ChangeDetectionVisualizer {
 public:
  using Marker = visualization_msgs::msg::Marker;

  // configs.
  struct DynamicConfig {
    RayVerificator::Config::RayPolicy ray_policy = RayVerificator::Config::RayPolicy::kMiddle;
    int object_id = 0;
    size_t point_id = 0;
    double points_scale = 0.1;
    double line_scale = 0.02;
    double target_scale = 0.3;
    bool show_missed_rays = false;
  };

  struct StaticConfig {
    int verbosity = 4;

    // Config of the ray verificator to use.
    RayVerificator::Config ray_verificator;
  };

  // Construction.
  ChangeDetectionVisualizer(const StaticConfig& config,
                            ianvs::NodeHandle nh,
                            std::shared_ptr<const DynamicSceneGraph> graph);
  ~ChangeDetectionVisualizer() = default;

  // Run.
  void start();

 private:
  // Configs.
  StaticConfig config;
  config::DynamicConfig<DynamicConfig> dynamic_config_;

  // ROS.
  ianvs::NodeHandle nh_;
  ianvs::NodeHandle::Timer timer_;
  rclcpp::Publisher<Marker>::SharedPtr points_pub_;
  rclcpp::Publisher<Marker>::SharedPtr lines_pub_;
  rclcpp::Publisher<Marker>::SharedPtr target_pub_;
  rclcpp::Publisher<Marker>::SharedPtr hash_pub_;

  // data.
  std::shared_ptr<const DynamicSceneGraph> graph_;
  std::unique_ptr<std::thread> spin_thread_;

  // Members.
  std::mutex mutex_;
  std::unique_ptr<RayVerificator> ray_verificator_;
  std::unique_ptr<KhronosObjectAttributes> object_;

  // tracking data.
  RayVerificator::Config::RayPolicy previous_ray_policy_ =
      RayVerificator::Config::RayPolicy::kMiddle;
  int previous_object_id_ = -1;

  // Helper functions.
  void draw();
  void reset();

  void setupRayVerificator();
  void setupObject();
  void drawRays();
  void drawHashBlock();

  // The cached messages to visualize.
  std_msgs::msg::Header header_;
  std::unique_ptr<Marker> target_marker_;
  std::unique_ptr<Marker> point_marker_;
  std::unique_ptr<Marker> line_marker_;
  std::unique_ptr<Marker> hash_marker_;
};

void declare_config(ChangeDetectionVisualizer::StaticConfig& config);

void declare_config(ChangeDetectionVisualizer::DynamicConfig& config);

}  // namespace khronos
