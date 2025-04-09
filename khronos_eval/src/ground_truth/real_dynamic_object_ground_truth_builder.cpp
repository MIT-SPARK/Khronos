#include "khronos_eval/ground_truth/real_dynamic_object_ground_truth_builder.h"

#include <algorithm>
#include <filesystem>

#include <config_utilities/types/path.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <hydra/common/robot_prefix_config.h>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "khronos/utils/khronos_attribute_utils.h"
#include "khronos_eval/utils/io_utils.h"

namespace khronos {

void declare_config(RealDynamicObjectGroundTruthBuilder::Config& config) {
  using namespace config;
  name("RealDynamicObjectGroundTruthBuilder");
  field<Path>(config.rosbag_file, "rosbag_file");
  field<Path>(config.dsg_file, "dsg_file");
  field<Path>(config.output_directory, "output_directory");
  field(config.fx, "fx");
  field(config.fy, "fy");
  field(config.cx, "cx");
  field(config.cy, "cy");
  field(config.rgb_topic, "rgb_topic");
  field(config.depth_topic, "depth_topic");
  field(config.max_observation_distance, "max_observation_distance", "m");

  check<Path::IsFile>(config.rosbag_file, "rosbag_file");
  check<Path::HasExtension>(config.rosbag_file, ".bag", "rosbag_file");
  check<Path::IsFile>(config.dsg_file, "dsg_file");
  check<Path::HasExtension>(config.dsg_file, ".sparkdsg", "dsg_file");
  check<Path::IsSet>(config.output_directory, "output_directory");
  check(config.fx, GT, 0.f, "fx");
  check(config.fy, GT, 0.f, "fy");
  check(config.cx, GT, 0.f, "cx");
  check(config.cy, GT, 0.f, "cy");
  check(config.max_observation_distance, GT, 0.f, "max_observation_distance");
}

RealDynamicObjectGroundTruthBuilder::RealDynamicObjectGroundTruthBuilder(const Config& config)
    : config(config::checkValid(config)), current_points_(), tfBuffer_(ros::Duration(3600)) {}

void RealDynamicObjectGroundTruthBuilder::run() {
  std::cout << "Running Real-Data ground truth dynamic objects extraction." << std::endl;
  setupDsg();

  // Process the rosbag for output objects
  processRosbag();

  // Save output
  saveOutput();
  std::cout << "Ground truth dynamic objects extraction complete." << std::endl;
}

void RealDynamicObjectGroundTruthBuilder::mouseClickCallback(int event, int x, int y, int flags) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    // Get depth image to read directly
    cv_bridge::CvImagePtr cv_ptr_depth;
    try {
      cv_ptr_depth =
          cv_bridge::toCvCopy(depth_images_.at(i_img_), sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
      LOG(FATAL) << "cv_bridge exception: " << e.what();
    }
    const cv::Mat& depth_image = cv_ptr_depth->image;

    float depth = depth_image.at<float>(y, x);

    if (depth > 0.01) {  // min depth for accuracy
      Point pixel_point = get3DPointFromDepth(rgb_images_.at(i_img_)->header.stamp, x, y, depth);
      float distance = getDistance(rgb_images_.at(i_img_)->header.stamp, pixel_point);

      // Save the point as a valid dynamic object component
      if (distance > 0 && distance <= config.max_observation_distance) {
        // LOG(INFO) << "Pushing back a point! " << distance << " " << depth;
        current_points_.push_back(std::make_pair(timestamps_.at(i_img_), pixel_point));
      }
    }
  }
}

void RealDynamicObjectGroundTruthBuilder::staticMouseClickCallback(int event,
                                                                   int x,
                                                                   int y,
                                                                   int flags,
                                                                   void* userdata) {
  RealDynamicObjectGroundTruthBuilder* self =
      static_cast<RealDynamicObjectGroundTruthBuilder*>(userdata);
  self->mouseClickCallback(event, x, y, flags);
}

void RealDynamicObjectGroundTruthBuilder::processRosbag() {
  // Get all relevant images from bag
  extractImagesFromBag(config.rosbag_file, config.rgb_topic, &rgb_images_);
  extractImagesFromBag(config.rosbag_file, config.depth_topic, &depth_images_);
  extractPosesFromDsg(config.dsg_file);

  // Sanity check: matching pairs of images for each timestamp
  CHECK_EQ(rgb_images_.size(), depth_images_.size())
      << "RGB and Depth have different number of images";
  for (size_t i = 0; i < rgb_images_.size(); i++) {
    CHECK_EQ(rgb_images_[i]->header.stamp, depth_images_[i]->header.stamp)
        << "RBG Image " << i << " has header " << rgb_images_[i]->header.stamp
        << " while Depth Image " << i << " has header  " << depth_images_[i]->header.stamp;
    timestamps_.push_back(TimeStamp(rgb_images_[i]->header.stamp.toNSec()));
  }
  CHECK_EQ(timestamps_.size(), rgb_images_.size());

  // Set up OpenCV window and mouse callback
  cv::namedWindow("Image Window");
  cv::setMouseCallback(
      "Image Window", RealDynamicObjectGroundTruthBuilder::staticMouseClickCallback, this);

  // Manually annotate ground truth dynamic objects
  for (i_img_ = 0; i_img_ < rgb_images_.size(); i_img_++) {
    cv_bridge::CvImagePtr cv_ptr_rgb;

    // Get RGB image to read directly
    try {
      cv_ptr_rgb = cv_bridge::toCvCopy(rgb_images_[i_img_], sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
      LOG(FATAL) << "cv_bridge exception: " << e.what();
    }
    const cv::Mat& rgb_image = cv_ptr_rgb->image;

    // Show RGB images and wait for clicks
    cv::imshow("RGB Image", rgb_image);
    cv::waitKey(1);

    // Check if we have a temporal break
    if (current_points_.size() > 0) {
      if (current_points_.back().first != timestamps_.at(i_img_ - 1)) {
        KhronosObjectAttributes::Ptr object = getKhronosAttributesFromPoints();

        // Add object to output DSG
        NodeSymbol symbol('O', num_objects_);
        object->name = symbol.getLabel();
        dsg_->addOrUpdateNode(DsgLayers::OBJECTS, symbol, std::move(object));
        num_objects_++;
        std::cout << "Added object: " << symbol.getLabel() << " to dsg." << std::endl;

        current_points_.clear();
      }
    }
  }

  // Add the last tracked objects to the output
  // TODO(marcus): copy-pasted from above here, simplify
  if (current_points_.size() > 0) {
    KhronosObjectAttributes::Ptr object = getKhronosAttributesFromPoints();
    // Add all object to the output DSG
    NodeSymbol symbol('O', num_objects_);
    object->name = symbol.getLabel();
    dsg_->addOrUpdateNode(DsgLayers::OBJECTS, symbol, std::move(object));
    num_objects_++;
    std::cout << "Added object: " << symbol.getLabel() << " to dsg." << std::endl;

    current_points_.clear();
  }

  std::cout << "Got " << num_objects_ << " objects." << std::endl;
}

Point RealDynamicObjectGroundTruthBuilder::get3DPointFromDepth(const ros::Time& timestamp,
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
  pointInCameraFrameMsg.header.frame_id = "left_cam";
  pointInCameraFrameMsg.header.stamp = timestamp_closest;

  // Transform the point to the world frame
  geometry_msgs::PointStamped pointInWorldFrameMsg;
  try {
    pointInWorldFrameMsg =
        tfBuffer_.transform(pointInCameraFrameMsg, "world", timestamp_closest, "left_cam");
  } catch (tf2::TransformException& ex) {
    LOG(INFO) << "closest timestamp to " << timestamp << " is " << timestamp_closest;
    LOG(WARNING) << "Couldn't get transform at " << timestamp_closest.toNSec();
    LOG(FATAL) << ex.what();
  }

  return Point(
      pointInWorldFrameMsg.point.x, pointInWorldFrameMsg.point.y, pointInWorldFrameMsg.point.z);
}

float RealDynamicObjectGroundTruthBuilder::getDistance(const ros::Time& timestamp,
                                                       const Point& pixel_point) {
  // Get the camera's pose at the given timestamp
  geometry_msgs::TransformStamped cameraToWorldTransform;
  const auto timestamp_closest = findClosestTimestamp(timestamp);
  try {
    cameraToWorldTransform = tfBuffer_.lookupTransform("world", "left_cam", timestamp_closest);
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

KhronosObjectAttributes::Ptr RealDynamicObjectGroundTruthBuilder::getKhronosAttributesFromPoints() {
  auto object = std::make_unique<KhronosObjectAttributes>();
  object->first_observed_ns.emplace_back(current_points_.front().first);
  object->last_observed_ns.emplace_back(current_points_.back().first);
  // object->semantic_id = static_cast<int>(label_map_.getLabelFromColor(toVoxblox(color)));
  // BoundingBox bbox;
  // bbox.setup(object_map.back().second);
  // object->bounding_box = bbox.toSpark();
  // Do them all separately to sidestep Vector3f -> Vector3d.
  // object->position.x() = bbox.world_P_center.x();
  // object->position.y() = bbox.world_P_center.y();
  // object->position.z() = bbox.world_P_center.z();
  object->trajectory_positions.reserve(current_points_.size());
  object->trajectory_timestamps.reserve(current_points_.size());
  object->dynamic_object_points.reserve(current_points_.size());
  for (const auto& pair : current_points_) {
    object->trajectory_positions.push_back(pair.second);
    object->trajectory_timestamps.push_back(pair.first);
    // object->dynamic_object_points.push_back(pair.second);
  }
  return object;
}

void RealDynamicObjectGroundTruthBuilder::setupDsg() {
  // TODO(marcus): may need something like mesh_layer_id here
  const DynamicSceneGraph::LayerIds layer_ids = {
      DsgLayers::OBJECTS,
  };
  dsg_ = std::make_shared<DynamicSceneGraph>(layer_ids);
}

void RealDynamicObjectGroundTruthBuilder::extractImagesFromBag(
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

void RealDynamicObjectGroundTruthBuilder::extractPosesFromDsg(const std::string& dsg_file) {
  tf2_ros::TransformListener tfListener(tfBuffer_);

  // Get each pose from the agent layer
  DynamicSceneGraph::Ptr eval_dsg = DynamicSceneGraph::load(dsg_file);
  const hydra::RobotPrefixConfig config;  // needed for proper indexing
  const auto& layer = eval_dsg->getLayer(DsgLayers::AGENTS, config.key);
  for (const auto& node : layer.nodes()) {
    // for (size_t i = 0; i < nodes.size(); i++) {
    // const auto& node = *nodes[i];
    // auto dynamic_node = dynamic_cast<const spark_dsg::DynamicSceneGraphNode*>(node.get());
    TimeStamp timestamp(static_cast<uint64_t>(node->timestamp->count()));
    const auto& position = node->attributes<spark_dsg::AgentNodeAttributes>().position;
    const auto& world_R_body = node->attributes<spark_dsg::AgentNodeAttributes>().world_R_body;

    // Update a transform and fake it into the buffer for easy use later
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time(toSeconds(timestamp));
    // TODO(marcus): put the frames back into the config! No hardcoding.
    transform.header.frame_id = "world";
    transform.child_frame_id = "left_cam";
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation.x = world_R_body.x();
    transform.transform.rotation.y = world_R_body.y();
    transform.transform.rotation.z = world_R_body.z();
    transform.transform.rotation.w = world_R_body.w();
    tfBuffer_.setTransform(transform, "default_authority", false);
  }
}

ros::Time RealDynamicObjectGroundTruthBuilder::findClosestTimestamp(const ros::Time& query_time) {
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

void RealDynamicObjectGroundTruthBuilder::saveOutput() {
  std::cout << "Saving output data to '" << config.output_directory << "'..." << std::endl;
  std::filesystem::create_directories(config.output_directory);
  const std::string dsg_name = config.output_directory + "/gt_dynamic_objects_dsg.sparkdsg";
  dsg_->save(dsg_name, false);
  std::cout << " - Saved " << dsg_name << "'." << std::endl;
}

}  // namespace khronos
