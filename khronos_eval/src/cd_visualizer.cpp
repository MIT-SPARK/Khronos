#include "khronos_eval/cd_visualizer.h"

#include <filesystem>

#include <config_utilities/parsing/ros.h>
#include <khronos_ros/visualization/visualization_utils.h>
#include <ros/ros.h>

namespace khronos {

void declare_config(ChangeDetectionVisualizer::StaticConfig& config) {
  using namespace config;
  name("RayVerificator");
  field(config.verbosity, "verbosity");
  field(config.ray_verificator, "ray_verificator");
}

ChangeDetectionVisualizer::ChangeDetectionVisualizer(const ros::NodeHandle& nh,
                                                     std::shared_ptr<const DynamicSceneGraph> graph)
    : config(config::checkValid(config::fromRos<StaticConfig>(nh))),
      dynamic_config_manager_(nh, "khronos/eval_vis"),
      dynamic_config_(dynamic_config_manager_.get()),
      nh_(nh),
      graph_(std::move(graph)) {
  // ROS.
  points_pub_ = nh_.advertise<visualization_msgs::Marker>("cd_points", 1);
  lines_pub_ = nh_.advertise<visualization_msgs::Marker>("cd_lines", 1);
  target_pub_ = nh_.advertise<visualization_msgs::Marker>("cd_target", 1);
  hash_pub_ = nh_.advertise<visualization_msgs::Marker>("cd_hash_block", 1);

  // Use the same cached header throughout.
  header_.frame_id = "world";

  // Register dynamic reconfigure.
  dynamic_config_manager_.setUpdateCallback([this]() { reset(); });

  // Spin.
  spin_thread_.reset(new std::thread(&ChangeDetectionVisualizer::spin, this));
}

void ChangeDetectionVisualizer::spin() {
  // Continuously publish in case things change.
  while (ros::ok()) {
    draw();
    ros::spinOnce();
  }
}

void ChangeDetectionVisualizer::reset() {
  // Clear all markers to force re-creation of markers.
  std::lock_guard<std::mutex> lock(mutex_);
  point_marker_.reset();
  line_marker_.reset();
  target_marker_.reset();
  hash_marker_.reset();

  // Reset the ray verificator if the policy changed.
  if (dynamic_config_.ray_policy != previous_ray_policy_) {
    previous_ray_policy_ = dynamic_config_.ray_policy;
    ray_verificator_.reset();
  }

  // Reset the query.
  if (dynamic_config_.object_id != previous_object_id_) {
    previous_object_id_ = dynamic_config_.object_id;
    object_.reset();
  }
}

void ChangeDetectionVisualizer::draw() {
  std::lock_guard<std::mutex> lock(mutex_);
  header_.stamp = ros::Time::now();

  setupRayVerificator();
  setupObject();
  drawRays();
  drawHashBlock();
}

void ChangeDetectionVisualizer::drawRays() {
  if (points_pub_.getNumSubscribers() == 0 && lines_pub_.getNumSubscribers() == 0 &&
      target_pub_.getNumSubscribers() == 0) {
    return;
  }

  if (!point_marker_) {
    // Select the Points.
    size_t idx = dynamic_config_.point_id;
    if (idx >= object_->mesh.numVertices()) {
      LOG(WARNING) << "Point index " << idx
                   << " is out of range (max: " << object_->mesh.numVertices() - 1 << ")";
      idx = object_->mesh.numVertices() - 1;
    }
    const Point query_point = object_->bounding_box.pointToWorldFrame(object_->mesh.pos(idx));
    RayVerificator::CheckDetails data;
    ray_verificator_->check(query_point,
                            object_->last_observed_ns.front(),
                            std::numeric_limits<uint64_t>::max(),
                            &data);
    size_t num_no_overlap = 0;

    // Target marker setup.
    target_marker_ = std::make_unique<visualization_msgs::Marker>();
    target_marker_->type = visualization_msgs::Marker::SPHERE;
    target_marker_->scale = setScale(dynamic_config_.target_scale);
    target_marker_->pose.position = setPoint(query_point);
    target_marker_->pose.orientation.w = 1.0;
    target_marker_->header = header_;
    target_marker_->action = visualization_msgs::Marker::ADD;
    target_marker_->color = setColor(Color(0, 255, 0));

    // Point marker setup
    visualization_msgs::Marker points_marker;
    points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    points_marker.scale = setScale(dynamic_config_.points_scale);
    points_marker.pose.orientation.w = 1.0;
    points_marker.header = header_;
    points_marker.action = visualization_msgs::Marker::ADD;

    // Line marker setup
    visualization_msgs::Marker lines_marker;
    lines_marker.type = visualization_msgs::Marker::LINE_LIST;
    lines_marker.scale.x = dynamic_config_.line_scale;
    lines_marker.pose.orientation.w = 1.f;
    lines_marker.header = header_;

    // All source target, and interception points.
    for (size_t i = 0; i < data.start.size(); ++i) {
      const auto start = setPoint(data.start[i]);
      const auto end = setPoint(data.end[i]);
      const auto gray = setColor(Color(150, 150, 150));
      const auto intersection =
          setPoint(data.start[i] + data.range[i] * (data.end[i] - data.start[i]) /
                                       (data.end[i] - data.start[i]).norm());

      switch (data.result[i]) {
        case RayVerificator::CheckDetails::Result::kNoOverlap: {
          if (dynamic_config_.show_missed_rays) {
            lines_marker.points.push_back(start);
            lines_marker.points.push_back(end);
            lines_marker.colors.push_back(gray);
            lines_marker.colors.push_back(gray);
            points_marker.points.push_back(start);
            points_marker.colors.push_back(gray);
            points_marker.points.push_back(end);
            points_marker.colors.push_back(gray);
          }
          num_no_overlap++;
          break;
        }
        case RayVerificator::CheckDetails::Result::kOccludded: {
          lines_marker.points.push_back(start);
          lines_marker.points.push_back(end);
          lines_marker.colors.push_back(gray);
          lines_marker.colors.push_back(gray);
          points_marker.points.push_back(start);
          points_marker.colors.push_back(gray);
          points_marker.points.push_back(end);
          points_marker.colors.push_back(gray);
          break;
        }
        case RayVerificator::CheckDetails::Result::kAbsent: {
          const auto red = setColor(Color(255, 0, 0));
          lines_marker.points.push_back(start);
          lines_marker.points.push_back(intersection);
          lines_marker.colors.push_back(red);
          lines_marker.colors.push_back(red);
          points_marker.points.push_back(start);
          points_marker.colors.push_back(red);
          points_marker.points.push_back(intersection);
          points_marker.colors.push_back(red);
          break;
        }
        case RayVerificator::CheckDetails::Result::kMatch: {
          const auto blue = setColor(Color(0, 0, 255));
          lines_marker.points.push_back(start);
          lines_marker.points.push_back(intersection);
          lines_marker.colors.push_back(blue);
          lines_marker.colors.push_back(blue);
          points_marker.points.push_back(start);
          points_marker.colors.push_back(blue);
          points_marker.points.push_back(intersection);
          points_marker.colors.push_back(blue);
          break;
        }
        default: {
          num_no_overlap++;
          break;
        }
      }
    }
    point_marker_ = std::make_unique<visualization_msgs::Marker>(points_marker);
    line_marker_ = std::make_unique<visualization_msgs::Marker>(lines_marker);
    CLOG(1) << "Showing O(" << dynamic_config_.object_id << ") point " << idx << "/"
            << object_->mesh.numVertices() << " (" << data.start.size() - num_no_overlap << "/"
            << data.start.size() << " rays active)." << std::endl;
  }

  // Publish the markers.
  if (points_pub_.getNumSubscribers() > 0) {
    points_pub_.publish(*point_marker_);
  }
  if (lines_pub_.getNumSubscribers() > 0) {
    lines_pub_.publish(*line_marker_);
  }
  if (target_pub_.getNumSubscribers() > 0) {
    target_pub_.publish(*target_marker_);
  }
}

void ChangeDetectionVisualizer::drawHashBlock() {
  if (hash_pub_.getNumSubscribers() == 0) {
    return;
  }

  if (!hash_marker_) {
    // Select the Points.
    size_t idx = dynamic_config_.point_id;
    if (idx >= object_->mesh.numVertices()) {
      // No need to warn here, this is already done in drawRays.
      idx = object_->mesh.numVertices() - 1;
    }
    const Point query_point = object_->bounding_box.pointToWorldFrame(object_->mesh.pos(idx));

    // Target marker setup.
    hash_marker_ = std::make_unique<visualization_msgs::Marker>();
    hash_marker_->type = visualization_msgs::Marker::CUBE;
    const float block_size = ray_verificator_->getConfig().block_size;
    hash_marker_->scale = setScale(block_size);
    const BlockIndex block_index =
        voxblox::getGridIndexFromPoint<BlockIndex>(query_point, 1.f / block_size);
    const Point center = voxblox::getCenterPointFromGridIndex(block_index, block_size);
    hash_marker_->pose.position = setPoint(center);
    hash_marker_->pose.orientation.w = 1.0;
    hash_marker_->header = header_;
    hash_marker_->action = visualization_msgs::Marker::ADD;
    hash_marker_->color.g = 1.f;
    hash_marker_->color.a = 0.5f;
  }

  // Publish the markers.
  if (hash_pub_.getNumSubscribers() > 0) {
    hash_pub_.publish(*hash_marker_);
  }
}

void ChangeDetectionVisualizer::setupRayVerificator() {
  if (ray_verificator_) {
    return;
  }

  // Create the ray verificator.
  RayVerificator::Config config = config.ray_verificator;
  config.ray_policy = static_cast<RayVerificator::Config::RayPolicy>(dynamic_config_.ray_policy);
  ray_verificator_ = std::make_unique<RayVerificator>(config);
  ray_verificator_->setDsg(graph_);
  CLOG(1) << "Setup Ray verificator:\n" << config << std::endl;
  previous_ray_policy_ = dynamic_config_.ray_policy;
}

void ChangeDetectionVisualizer::setupObject() {
  if (object_) {
    return;
  }

  // Select the object.
  const auto& nodes = graph_->getLayer(DsgLayers::OBJECTS).nodes();
  NodeSymbol symbol('O', dynamic_config_.object_id);
  if (!nodes.count(symbol)) {
    LOG(WARNING) << "Object " << symbol << " does not exist.";
    const auto& node = nodes.begin()->second.get();
    previous_object_id_ = NodeSymbol(node->id).categoryId();
    object_ =
        std::make_unique<KhronosObjectAttributes>(node->attributes<KhronosObjectAttributes>());
    return;
  }
  object_ = std::make_unique<KhronosObjectAttributes>(
      graph_->getNode(symbol).attributes<KhronosObjectAttributes>());
  previous_object_id_ = dynamic_config_.object_id;
}

}  // namespace khronos
