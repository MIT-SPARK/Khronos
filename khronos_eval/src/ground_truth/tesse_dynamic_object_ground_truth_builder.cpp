#include "khronos_eval/ground_truth/tesse_dynamic_object_ground_truth_builder.h"

#include <algorithm>
#include <filesystem>

#include <config_utilities/types/path.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  check<Path::HasExtension>(config.rosbag_file, ".bag", "rosbag_file");
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
      tfBuffer_(ros::Duration(3600)) {}

void TesseDynamicObjectGroundTruthBuilder::run() {
  std::cout << "Running TESSE/Khronos ground truth dynamic objects extraction." << std::endl;
  setupDsg();

  // Process the rosbag for output objects
  processRosbag();

  // Remove any objects as requested
  for (const auto& id : config.remove_object_ids) {
    NodeSymbol symbol('O', id);
    dsg_->removeNode(NodeId(symbol));
    std::cout << "Removed object: " << symbol.getLabel() << " from dsg." << std::endl;
  }

  // Save output
  saveOutput();
  std::cout << "Ground truth dynamic objects extraction complete." << std::endl;
}

void TesseDynamicObjectGroundTruthBuilder::processRosbag() {
  // Get all relevant images from bag
  std::vector<sensor_msgs::ImageConstPtr> rgb_images, depth_images;
  extractImagesFromBag(config.rosbag_file, config.rgb_topic, &rgb_images);
  extractImagesFromBag(config.rosbag_file, config.depth_topic, &depth_images);
  extractTfsFromBag(config.rosbag_file);

  // Keep track of total timestamps and timestamps with dynamic objects
  std::vector<TimeStamp> timestamps;

  // Sanity check: matching pairs of images for each timestamp
  CHECK_EQ(rgb_images.size(), depth_images.size())
      << "RGB and Depth have different number of images";
  for (size_t i = 0; i < rgb_images.size(); i++) {
    CHECK_EQ(rgb_images[i]->header.stamp, depth_images[i]->header.stamp)
        << "RBG Image " << i << " has header " << rgb_images[i]->header.stamp
        << " while Depth Image " << i << " has header  " << depth_images[i]->header.stamp;
    timestamps.push_back(TimeStamp(rgb_images[i]->header.stamp.toNSec()));
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
      cv_ptr_rgb = cv_bridge::toCvCopy(rgb_images[i], sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
      LOG(FATAL) << "cv_bridge exception: " << e.what();
    }
    const cv::Mat& rgb_image = cv_ptr_rgb->image;

    // Get depth image to read directly
    try {
      cv_ptr_depth = cv_bridge::toCvCopy(depth_images[i], sensor_msgs::image_encodings::TYPE_32FC1);
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
              Point pixel_point = get3DPointFromDepth(rgb_images[i]->header.stamp, x, y, depth);
              float distance = getDistance(rgb_images[i]->header.stamp, pixel_point);

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
          object->name = symbol.getLabel();
          dsg_->addOrUpdateNode(DsgLayers::OBJECTS, symbol, std::move(object));
          num_objects_++;
          std::cout << "Added object: " << symbol.getLabel() << " to dsg." << std::endl;

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
      object->name = symbol.getLabel();
      dsg_->addOrUpdateNode(DsgLayers::OBJECTS, symbol, std::move(object));
      num_objects_++;
      std::cout << "Added object: " << symbol.getLabel() << " to dsg." << std::endl;

      new_object_map_color.clear();
    }
  }

  std::cout << "Got " << num_objects_ << " objects." << std::endl;
}

Point TesseDynamicObjectGroundTruthBuilder::get3DPointFromDepth(const ros::Time& timestamp,
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
  geometry_msgs::PointStamped pointInCameraFrameMsg;
  pointInCameraFrameMsg.point.x = pointInCameraFrame.x();
  pointInCameraFrameMsg.point.y = pointInCameraFrame.y();
  pointInCameraFrameMsg.point.z = pointInCameraFrame.z();
  pointInCameraFrameMsg.header.frame_id = config.camera_frame_id;
  pointInCameraFrameMsg.header.stamp = timestamp_closest;

  // Transform the point to the world frame
  geometry_msgs::PointStamped pointInWorldFrameMsg;
  try {
    pointInWorldFrameMsg = tfBuffer_.transform(
        pointInCameraFrameMsg, config.world_frame_id, timestamp_closest, config.camera_frame_id);
  } catch (tf2::TransformException& ex) {
    LOG(INFO) << "closest timestamp to " << timestamp << " is " << timestamp_closest;
    LOG(WARNING) << "Couldn't get transform at " << timestamp_closest.toNSec();
    LOG(FATAL) << ex.what();
  }

  return Point(
      pointInWorldFrameMsg.point.x, pointInWorldFrameMsg.point.y, pointInWorldFrameMsg.point.z);
}

float TesseDynamicObjectGroundTruthBuilder::getDistance(const ros::Time& timestamp,
                                                        const Point& pixel_point) {
  // Get the camera's pose at the given timestamp
  geometry_msgs::TransformStamped cameraToWorldTransform;
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
  // TODO(marcus): may need something like mesh_layer_id here
  const DynamicSceneGraph::LayerIds layer_ids = {
      DsgLayers::OBJECTS,
  };
  dsg_ = std::make_shared<DynamicSceneGraph>(layer_ids);

  // Get the colors of dynamic objects
  for (const auto& color : config.dynamic_object_colors) {
    CHECK_EQ(color.size(), 4);
    colors_dynamic_.push_back(Color(color[0], color[1], color[2], color[3]));
  }
}

void TesseDynamicObjectGroundTruthBuilder::extractImagesFromBag(
    const std::string& bag_file,
    const std::string& topic,
    std::vector<sensor_msgs::ImageConstPtr>* images) {
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);

  std::vector<std::string> topics = {topic};
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // Save all matching image messages
  // TODO(marcus): Can we reserve the space for images before doing this?
  for (const rosbag::MessageInstance& m : view) {
    sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
    if (img_msg != nullptr) {
      images->push_back(img_msg);
    }
  }

  bag.close();

  // Sort images by timestamp (should be given for free but not guaranteed)
  std::sort(images->begin(),
            images->end(),
            [](const sensor_msgs::ImageConstPtr& a, const sensor_msgs::ImageConstPtr& b) {
              return a->header.stamp < b->header.stamp;
            });
}

void TesseDynamicObjectGroundTruthBuilder::extractTfsFromBag(const std::string& bag_file) {
  tf2_ros::TransformListener tfListener(tfBuffer_);

  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);

  std::vector<std::string> topics = {"/tf", "/tf_static"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Time min_time = ros::TIME_MAX;
  ros::Time max_time = ros::TIME_MIN;

  for (const rosbag::MessageInstance& m : view) {
    tf2_msgs::TFMessage::ConstPtr tf_message = m.instantiate<tf2_msgs::TFMessage>();
    if (tf_message != nullptr) {
      // Iterate over all transforms in the message and add them to the buffer
      for (const auto& transform : tf_message->transforms) {
        tfBuffer_.setTransform(transform, "default_authority", m.getTopic() == "/tf_static");
        if (transform.header.stamp < min_time) {
          min_time = transform.header.stamp;
        }
        if (transform.header.stamp > max_time) {
          max_time = transform.header.stamp;
        }
        if (transform.child_frame_id == config.body_frame_id ||
            transform.child_frame_id == config.camera_frame_id) {
          transform_timestamps_.insert(transform.header.stamp);
        }
      }
    }
  }
  bag.close();
}

ros::Time TesseDynamicObjectGroundTruthBuilder::findClosestTimestamp(const ros::Time& query_time) {
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
  const std::string dsg_name = config.output_directory + "/gt_dynamic_objects_dsg.sparkdsg";
  dsg_->save(dsg_name, false);
  std::cout << " - Saved " << dsg_name << "'." << std::endl;
}

}  // namespace khronos
