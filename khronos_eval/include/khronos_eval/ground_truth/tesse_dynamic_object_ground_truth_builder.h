#pragma once

#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/common/semantic_color_map.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/tfMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "khronos/common/common_types.h"

namespace khronos {
class TesseDynamicObjectGroundTruthBuilder {
 public:
  using Colors = std::vector<Color>;
  using ColorKey = std::tuple<int, int, int>;  // for std::map
  using ManyPointsStamped = std::vector<std::pair<TimeStamp, Points>>;

  struct Config {
    std::string rosbag_file;
    std::string semantic_colors_file;
    std::string output_directory;

    std::vector<std::vector<int>> dynamic_object_colors;
    std::vector<int> remove_object_ids;

    float fx, fy, cx, cy = 0.0;

    std::string rgb_topic = "/tesse/seg_cam/rgb/image_raw";
    std::string depth_topic = "/tesse/depth_cam/mono/image_raw";

    std::string camera_frame_id = "left_cam";
    std::string body_frame_id = "base_link_gt";
    std::string world_frame_id = "world";

    float max_observation_distance = 5.0;  // meters
  } const config;

  // Construction
  explicit TesseDynamicObjectGroundTruthBuilder(const Config& config);
  virtual ~TesseDynamicObjectGroundTruthBuilder() = default;

  // Interface
  void run();

 protected:
  void setupDsg();
  void processRosbag();
  Point get3DPointFromDepth(const ros::Time& timestamp,
                            const int x,
                            const int y,
                            const float depth);
  float getDistance(const ros::Time& timestamp, const Point& pixel_point);
  KhronosObjectAttributes::Ptr getKhronosAttributesFromObjectMap(
      const std::vector<std::pair<TimeStamp, Points>>& object_map);
  Point getCentroidFromPoints(const Points& points);
  void extractImagesFromBag(const std::string& bag_file,
                            const std::string& topic,
                            std::vector<sensor_msgs::ImageConstPtr>* images);
  void extractTfsFromBag(const std::string& bag_file);
  ros::Time findClosestTimestamp(const ros::Time& query_time);
  void saveOutput();

 private:
  std::vector<Color> colors_dynamic_;
  DynamicSceneGraph::Ptr dsg_;
  hydra::SemanticColorMap label_map_;
  size_t num_objects_ = 0;
  tf2_ros::Buffer tfBuffer_;
  std::set<ros::Time> transform_timestamps_;
};

void declare_config(TesseDynamicObjectGroundTruthBuilder::Config& config);

}  // namespace khronos
