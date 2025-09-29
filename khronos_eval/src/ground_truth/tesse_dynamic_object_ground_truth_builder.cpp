#include "khronos_eval/ground_truth/tesse_dynamic_object_ground_truth_builder.h"

#include <algorithm>
#include <filesystem>

#include <config_utilities/types/path.h>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "khronos/utils/geometry_utils.h"
#include "khronos/utils/khronos_attribute_utils.h"
#include "khronos_eval/utils/io_utils.h"

namespace khronos {

void declare_config(TesseDynamicObjectGroundTruthBuilder::Config& config) {
  using namespace config;
  name("TesseDynamicObjectGroundTruthBuilder");
  field<Path>(config.rosbag_file, "rosbag_file");
  field<Path>(config.semantic_colors_file, "semantic_colors_file");
  field<Path>(config.output_directory, "output_directory");
  field(config.dynamic_object_colors, "dynamic_object_colors");
  field(config.remove_object_ids, "remove_object_ids");
  field(config.fx, "fx");
  field(config.fy, "fy");
  field(config.cx, "cx");
  field(config.cy, "cy");
  field(config.rgb_topic, "rgb_topic");
  field(config.depth_topic, "depth_topic");
  field(config.camera_frame_id, "camera_frame_id");
  field(config.body_frame_id, "body_frame_id");
  field(config.world_frame_id, "world_frame_id");
  field(config.max_observation_distance, "max_observation_distance", "m");

  check<Path::IsFile>(config.rosbag_file, "rosbag_file");
  // ROS2 bags are directories, not .bag files
  check<Path::IsDirectory>(config.rosbag_file, "rosbag_file");
  check<Path::IsFile>(config.semantic_colors_file, "semantic_colors_file");
  check<Path::HasExtension>(config.semantic_colors_file, ".csv", "semantic_colors_file");
  check<Path::IsSet>(config.output_directory, "output_directory");
  check(config.fx, GT, 0.f, "fx");
  check(config.fy, GT, 0.f, "fy");
  check(config.cx, GT, 0.f, "cx");
  check(config.cy, GT, 0.f, "cy");
  check(config.max_observation_distance, GT, 0.f, "max_observation_distance");
}

TesseDynamicObjectGroundTruthBuilder::TesseDynamicObjectGroundTruthBuilder(const Config& config)
    : config(config::checkValid(config)),
      label_map_(*hydra::SemanticColorMap::fromCsv(config.semantic_colors_file)),
      tfBuffer_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)) {}

void TesseDynamicObjectGroundTruthBuilder::run() {
  std::cout << "Running TESSE/Khronos ground truth dynamic objects extraction." << std::endl;
  setupDsg();

  // Process the rosbag for output objects
  processRosbag();

  // Remove any objects as requested
  for (const auto& id : config.remove_object_ids) {
    NodeSymbol symbol('O', id);
    dsg_->removeNode(NodeId(symbol));
    std::cout << "Removed object: " << symbol.str() << " from dsg." << std::endl;
  }

  // Save output
  saveOutput();
  std::cout << "Ground truth dynamic objects extraction complete." << std::endl;
}

void TesseDynamicObjectGroundTruthBuilder::processRosbag() {
  // Get all relevant images from bag
  std::vector<std::shared_ptr<sensor_msgs::msg::Image>> rgb_images, depth_images;
  extractImagesFromBag(config.rosbag_file, config.rgb_topic, &rgb_images);
  extractImagesFromBag(config.rosbag_file, config.depth_topic, &depth_images);
  extractTfsFromBag(config.rosbag_file);

  // Keep track of total timestamps and timestamps with dynamic objects
  std::vector<TimeStamp> timestamps;

  // Sanity check: matching pairs of images for each timestamp
  CHECK_EQ(rgb_images.size(), depth_images.size())
      << "RGB and Depth have different number of images";
  for (size_t i = 0; i < rgb_images.size(); i++) {
    CHECK_EQ(rclcpp::Time(rgb_images[i]->header.stamp).nanoseconds(), 
           rclcpp::Time(depth_images[i]->header.stamp).nanoseconds())
        << "RBG Image " << i << " has header " << rclcpp::Time(rgb_images[i]->header.stamp).nanoseconds()
        << " while Depth Image " << i << " has header  " << rclcpp::Time(depth_images[i]->header.stamp).nanoseconds();
    timestamps.push_back(TimeStamp(rclcpp::Time(rgb_images[i]->header.stamp).nanoseconds()));
  }
  CHECK_EQ(timestamps.size(), rgb_images.size());

  // Track temporally-contiguous dynamic objects
  std::map<ColorKey, ManyPointsStamped> new_object_map;
  for (const auto& color : colors_dynamic_) {
    new_object_map[ColorKey(color.r, color.g, color.b)] = ManyPointsStamped();
  }

  for (size_t i = 0; i < rgb_images.size(); i++) {
    cv_bridge::CvImagePtr cv_ptr_rgb;
    cv_bridge::CvImagePtr cv_ptr_depth;
    const TimeStamp timestamp = timestamps[i];

    // Get RGB image to read directly
    try {
      cv_ptr_rgb = cv_bridge::toCvCopy(rgb_images[i], "rgb8");
    } catch (cv_bridge::Exception& e) {
      LOG(FATAL) << "cv_bridge exception: " << e.what();
    }
    const cv::Mat& rgb_image = cv_ptr_rgb->image;

    // Get depth image to read directly
    try {
      cv_ptr_depth = cv_bridge::toCvCopy(depth_images[i], "32FC1");
    } catch (cv_bridge::Exception& e) {
      LOG(FATAL) << "cv_bridge exception: " << e.what();
    }
    const cv::Mat& depth_image = cv_ptr_depth->image;

    // Iterate over all pixels
    for (const auto& color : colors_dynamic_) {
      Points points;
      for (int y = 0; y < rgb_image.rows; ++y) {
        for (int x = 0; x < rgb_image.cols; ++x) {
          cv::Vec3b pixel = rgb_image.at<cv::Vec3b>(y, x);

          // Check each color we want to filter over
          if (Color(pixel[0], pixel[1], pixel[2]) == color) {
            // Check for the active window's activation distance
            float depth = depth_image.at<float>(y, x);
            if (depth > 0.1) {  // min depth for simulator to be accurate
              Point pixel_point = get3DPointFromDepth(rclcpp::Time(rgb_images[i]->header.stamp), x, y, depth);
              float distance = getDistance(rclcpp::Time(rgb_images[i]->header.stamp), pixel_point);

              // Save the point as a valid dynamic object component
              if (distance > 0 && distance <= config.max_observation_distance) {
                // LOG(INFO) << "Pushing back a point! " << distance << " " << depth;
                points.push_back(pixel_point);
              }
            }
          }
        }
      }

      // Check if we have a temporal break for this color
      if (points.size() > 0) {
        auto& new_object_map_color = new_object_map.at(ColorKey(color.r, color.g, color.b));
        if (new_object_map_color.size() > 0 &&
            new_object_map_color.back().first != timestamps[i - 1]) {
          // Current timestamp is more than one frame ahead of last timestamp,
          // so we save the old object
          // ASSUMPTION: Only one dynamic object of a particular class will be visible at a time
          // NOTE: to relax this, you need to detect connected components without oversegmenting
          // (e.g., when occlusions split an object).
          KhronosObjectAttributes::Ptr object =
              getKhronosAttributesFromObjectMap(new_object_map_color);

          // Add all object to the output DSG
          NodeSymbol symbol('O', num_objects_);
          object->name = symbol.str();
          dsg_->addOrUpdateNode(DsgLayers::OBJECTS, symbol, std::move(object));
          num_objects_++;
          std::cout << "Added object: " << symbol.str() << " to dsg." << std::endl;

          // Clear out current color object tracker to begin tracking a new object
          new_object_map_color.clear();
        }

        // Add the new points to the color tracker
        new_object_map_color.push_back(std::make_pair(timestamp, points));
      }
    }
  }

  // Add the last tracked objects to the output
  // TODO(marcus): copy-pasted from above here, simplify
  for (const auto& color : colors_dynamic_) {
    auto& new_object_map_color = new_object_map.at(ColorKey(color.r, color.g, color.b));
    if (new_object_map_color.size() > 0) {
      KhronosObjectAttributes::Ptr object = getKhronosAttributesFromObjectMap(new_object_map_color);
      // Add all object to the output DSG
      NodeSymbol symbol('O', num_objects_);
      object->name = symbol.str();
      dsg_->addOrUpdateNode(DsgLayers::OBJECTS, symbol, std::move(object));
      num_objects_++;
      std::cout << "Added object: " << symbol.str() << " to dsg." << std::endl;

      new_object_map_color.clear();
    }
  }

  std::cout << "Got " << num_objects_ << " objects." << std::endl;
}

Point TesseDynamicObjectGroundTruthBuilder::get3DPointFromDepth(const rclcpp::Time& timestamp,
                                                                const int x,
                                                                const int y,
                                                                const float depth) {
  // Convert pixel to 3D point in camera coordinates
  float camera_x = (x - config.cx) * depth / config.fx;
  float camera_y = (y - config.cy) * depth / config.fy;
  float camera_z = depth;
  Point pointInCameraFrame(camera_x, camera_y, camera_z);

  const auto timestamp_closest = findClosestTimestamp(timestamp);

  // Convert point type
  geometry_msgs::msg::PointStamped pointInCameraFrameMsg;
  pointInCameraFrameMsg.point.x = pointInCameraFrame.x();
  pointInCameraFrameMsg.point.y = pointInCameraFrame.y();
  pointInCameraFrameMsg.point.z = pointInCameraFrame.z();
  pointInCameraFrameMsg.header.frame_id = config.camera_frame_id;
  pointInCameraFrameMsg.header.stamp = timestamp_closest;

  // Transform the point to the world frame
  geometry_msgs::msg::PointStamped pointInWorldFrameMsg;
  try {
    tf2::doTransform(pointInCameraFrameMsg, pointInWorldFrameMsg,
                     tfBuffer_.lookupTransform(config.world_frame_id, config.camera_frame_id,
                                               timestamp_closest));
  } catch (tf2::TransformException& ex) {
    LOG(INFO) << "closest timestamp to " << timestamp.nanoseconds() << " is " 
              << timestamp_closest.nanoseconds();
    LOG(WARNING) << "Couldn't get transform at " << timestamp_closest.nanoseconds();
    LOG(FATAL) << ex.what();
  }

  return Point(
      pointInWorldFrameMsg.point.x, pointInWorldFrameMsg.point.y, pointInWorldFrameMsg.point.z);
}

float TesseDynamicObjectGroundTruthBuilder::getDistance(const rclcpp::Time& timestamp,
                                                        const Point& pixel_point) {
  // Get the camera's pose at the given timestamp
  geometry_msgs::msg::TransformStamped cameraToWorldTransform;
  const auto timestamp_closest = findClosestTimestamp(timestamp);
  try {
    cameraToWorldTransform =
        tfBuffer_.lookupTransform(config.world_frame_id, config.camera_frame_id, timestamp_closest);
  } catch (tf2::TransformException& ex) {
    LOG(FATAL) << ex.what();
  }

  // Extract the camera position from the transform
  Point cam_point(cameraToWorldTransform.transform.translation.x,
                  cameraToWorldTransform.transform.translation.y,
                  cameraToWorldTransform.transform.translation.z);

  // Compute the Euclidean distance
  return (cam_point - pixel_point).norm();
}

KhronosObjectAttributes::Ptr
TesseDynamicObjectGroundTruthBuilder::getKhronosAttributesFromObjectMap(
    const ManyPointsStamped& object_map) {
  auto object = std::make_unique<KhronosObjectAttributes>();
  object->first_observed_ns.emplace_back(object_map.front().first);
  object->last_observed_ns.emplace_back(object_map.back().first);
  // object->semantic_id = static_cast<int>(label_map_.getLabelFromColor(toVoxblox(color)));
  object->bounding_box = BoundingBox(object_map.back().second);
  // Do them all separately to sidestep Vector3f -> Vector3d.
  object->position = object->bounding_box.world_P_center.cast<double>();
  object->trajectory_positions.reserve(object_map.size());
  object->trajectory_timestamps.reserve(object_map.size());
  object->dynamic_object_points.reserve(object_map.size());
  for (const auto& pair : object_map) {
    object->trajectory_positions.push_back(getCentroidFromPoints(pair.second));
    object->trajectory_timestamps.push_back(pair.first);
    object->dynamic_object_points.push_back(pair.second);
  }
  return object;
}

Point TesseDynamicObjectGroundTruthBuilder::getCentroidFromPoints(const Points& points) {
  // Average all points
  Point centroid = Point(0, 0, 0);
  for (const auto& point : points) {
    centroid.x() += point.x();
    centroid.y() += point.y();
    centroid.z() += point.z();
  }
  return centroid / points.size();
}

void TesseDynamicObjectGroundTruthBuilder::setupDsg() {
  const std::map<std::string, spark_dsg::LayerKey> layers = {{DsgLayers::OBJECTS, 2}};
  dsg_ = DynamicSceneGraph::fromNames(layers);

  // Get the colors of dynamic objects
  for (const auto& color : config.dynamic_object_colors) {
    CHECK_EQ(color.size(), 4);
    colors_dynamic_.push_back(Color(color[0], color[1], color[2], color[3]));
  }
}

void TesseDynamicObjectGroundTruthBuilder::extractImagesFromBag(
    const std::string& bag_file,
    const std::string& topic,
    std::vector<std::shared_ptr<sensor_msgs::msg::Image>>* images) {
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_file;
  storage_options.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  
  reader.open(storage_options, converter_options);
  
  // Save all matching image messages
  rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
  while (reader.has_next()) {
    auto msg = reader.read_next();
    if (msg->topic_name == topic) {
      auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      serializer.deserialize_message(&serialized_msg, img_msg.get());
      images->push_back(img_msg);
    }
  }
  
  // Sort images by timestamp (should be given for free but not guaranteed)
  std::sort(images->begin(),
            images->end(),
            [](const std::shared_ptr<sensor_msgs::msg::Image>& a, 
               const std::shared_ptr<sensor_msgs::msg::Image>& b) {
              return rclcpp::Time(a->header.stamp) < rclcpp::Time(b->header.stamp);
            });
}

void TesseDynamicObjectGroundTruthBuilder::extractTfsFromBag(const std::string& bag_file) {
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_file;
  storage_options.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  
  reader.open(storage_options, converter_options);
  
  rclcpp::Time min_time = rclcpp::Time::max();
  rclcpp::Time max_time = rclcpp::Time(0);
  
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
  while (reader.has_next()) {
    auto msg = reader.read_next();
    if (msg->topic_name == "/tf" || msg->topic_name == "/tf_static") {
      auto tf_message = std::make_shared<tf2_msgs::msg::TFMessage>();
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      serializer.deserialize_message(&serialized_msg, tf_message.get());
      
      // Iterate over all transforms in the message and add them to the buffer
      for (const auto& transform : tf_message->transforms) {
        tfBuffer_.setTransform(transform, "default_authority", msg->topic_name == "/tf_static");
        rclcpp::Time stamp(transform.header.stamp);
        if (stamp < min_time) {
          min_time = stamp;
        }
        if (stamp > max_time) {
          max_time = stamp;
        }
        if (transform.child_frame_id == config.body_frame_id ||
            transform.child_frame_id == config.camera_frame_id) {
          transform_timestamps_.insert(stamp);
        }
      }
    }
  }
}

rclcpp::Time TesseDynamicObjectGroundTruthBuilder::findClosestTimestamp(const rclcpp::Time& query_time) {
  auto lower = transform_timestamps_.lower_bound(query_time);
  auto upper = lower;

  if (lower != transform_timestamps_.begin()) {
    lower = std::prev(lower);
  }

  if (upper == transform_timestamps_.end() || query_time - *lower <= *upper - query_time) {
    return *lower;
  }

  return *upper;
}

void TesseDynamicObjectGroundTruthBuilder::saveOutput() {
  std::cout << "Saving output data to '" << config.output_directory << "'..." << std::endl;
  std::filesystem::create_directories(config.output_directory);
  const std::string dsg_name = config.output_directory + "/gt_dynamic_objects_dsg.json";
  dsg_->save(dsg_name, false);
  std::cout << " - Saved " << dsg_name << "'." << std::endl;
}

}  // namespace khronos
