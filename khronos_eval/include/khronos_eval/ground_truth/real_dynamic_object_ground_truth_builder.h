#pragma once

#include <set>
#include <string>
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
class RealDynamicObjectGroundTruthBuilder {
 public:
  struct Config {
    std::string rosbag_file;
    std::string dsg_file;
    std::string output_directory;

    float fx, fy, cx, cy = 0.0;

    std::string rgb_topic = "/rgb_camera/image_raw";
    std::string depth_topic = "/depth_camera/image_raw";

    float max_observation_distance = 5.0;  // meters
  } const config;

  // Construction
  explicit RealDynamicObjectGroundTruthBuilder(const Config& config);
  virtual ~RealDynamicObjectGroundTruthBuilder() = default;

  // Interface
  void run();

 protected:
  void setupDsg();
  void processRosbag();
  void mouseClickCallback(int event, int x, int y, int flags);
  static void staticMouseClickCallback(int event, int x, int y, int flags, void* userdata);
  Point get3DPointFromDepth(const ros::Time& timestamp,
                            const int x,
                            const int y,
                            const float depth);
  float getDistance(const ros::Time& timestamp, const Point& pixel_point);
  KhronosObjectAttributes::Ptr getKhronosAttributesFromPoints();
  void extractImagesFromBag(const std::string& bag_file,
                            const std::string& topic,
                            std::vector<sensor_msgs::ImageConstPtr>* images);
  void extractPosesFromDsg(const std::string& dsg_file);
  ros::Time findClosestTimestamp(const ros::Time& query_time);
  void saveOutput();

 private:
  std::vector<sensor_msgs::ImageConstPtr> rgb_images_, depth_images_;
  std::vector<TimeStamp> timestamps_;
  std::vector<std::pair<TimeStamp, Point>> current_points_;
  DynamicSceneGraph::Ptr dsg_;
  size_t i_img_ = 0;
  size_t num_objects_ = 0;
  tf2_ros::Buffer tfBuffer_;
  std::set<ros::Time> transform_timestamps_;
};

void declare_config(RealDynamicObjectGroundTruthBuilder::Config& config);

}  // namespace khronos
