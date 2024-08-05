/** -----------------------------------------------------------------------------
 * Copyright (c) 2024 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:      Lukas Schmid <lschmid@mit.edu>, Marcus Abate <mabate@mit.edu>,
 *               Yun Chang <yunchang@mit.edu>, Luca Carlone <lcarlone@mit.edu>
 * AFFILIATION:  MIT SPARK Lab, Massachusetts Institute of Technology
 * YEAR:         2024
 * SOURCE:       https://github.com/MIT-SPARK/Khronos
 * LICENSE:      BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#include "khronos_ros/visualization/spatio_temporal_visualizer.h"

#include <algorithm>
#include <numeric>
#include <thread>

#include <config_utilities/parsing/ros.h>
#include <config_utilities/types/path.h>
#include <khronos_msgs/SpatioTemporalVisualizerState.h>
#include <spark_dsg/colormaps.h>
#include <visualization_msgs/MarkerArray.h>

#include "khronos/utils/geometry_utils.h"
#include "khronos/utils/khronos_attribute_utils.h"
#include "khronos_ros/experiments/experiment_directory.h"
#include "khronos_ros/visualization/visualization_utils.h"

namespace khronos_msgs {

void declare_config(KhronosSpatioTemporalVisConfig& config) {
  using namespace config;
  name("DynamicConfig");
  field(config.background_color, "background_color");
  field(config.object_bbox_color, "object_bbox_color");
  field(config.dynamic_object_color, "dynamic_object_color");
  field(config.play_rate, "play_rate");
}

}  // namespace khronos_msgs

namespace khronos {

namespace colormaps = spark_dsg::colormaps;

void declare_config(SpatioTemporalVisualizer::Config& config) {
  using namespace config;
  name("SpatioTemporalVisualizer");
  field(config.verbosity, "verbosity");
  field(config.map_file, "map_file");
  field(config.max_frame_rate, "max_frame_rate");
  field(config.global_frame_name, "global_frame_name");
  field(config.robot_prefix, "robot_prefix");
  field(config.dynamic_config, "dynamic_config");
  field(config.mesh_visualizer, "mesh_visualizer");
  field(config.initial_robot_time, "initial_robot_time");
  field(config.initial_query_time, "initial_query_time");
  enum_field(config.initial_time_mode, "initial_time_mode", {"Robot", "Query", "Online"});

  checkCondition(!config.global_frame_name.empty(), "'global_frame_name' can not be empty.");
  check<Path::Exists>(config.map_file, "map_file");
  check<Path::HasExtension>(config.map_file, ".4dmap", "map_file");
  checkInRange(config.initial_query_time, 0.f, 1.f, "initial_query_time");
  checkInRange(config.initial_robot_time, 0.f, 1.f, "initial_robot_time");
}

SpatioTemporalVisualizer::SpatioTemporalVisualizer(const ros::NodeHandle& nh)
    : config(config::checkValid(config::fromRos<Config>(nh))),
      dynamic_config_(nh, ""),
      nh_(nh),
      layer_ids_({
          DsgLayers::OBJECTS,
          DsgLayers::PLACES,
          DsgLayers::ROOMS,
          DsgLayers::BUILDINGS,
      }),
      min_time_delta_(config.max_frame_rate > 0 ? 1.0 / config.max_frame_rate : 0),
      mesh_visualizer_(config.mesh_visualizer, nh),
      dsg_renderer_(nh),
      map_(SpatioTemporalMap::Config()) {
  // Load all data and setup.
  LOG(INFO) << "Config:\n" << config;
  loadMap();
  initializeDynamicConfig();
  reset();
  setupRos();

  // Advertize that the visualizer is ready.
  is_setup_srv_ = nh_.advertiseService("is_setup", &SpatioTemporalVisualizer::isSetupCb, this);
  LOG(INFO) << "SpatioTemporalVisualizer initialized.";
}

void SpatioTemporalVisualizer::spin() {
  while (!should_shutdown_ && ros::ok()) {
    ros::spinOnce();
    std::lock_guard<std::mutex> lock(mutex_);
    updatePlayTime();
    updateDsgTime();
    draw();
  }
}

void SpatioTemporalVisualizer::draw() {
  if (!needs_redraw_) {
    return;
  }
  // Update colorings as needed.
  // NOTE(lschmid): These could also be implemented incrementally but not worth the tracking I
  // think, we're already getting 1000's of FPS.
  recolorObjectDsgBoundingBoxes();
  drawDynamicObjects();
  drawAgent();

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = config.global_frame_name;
  dsg_renderer_.draw(header, *current_dsg_);

  const auto mesh = current_dsg_->mesh();
  auto mesh_coloring = getBackgroundMeshColoring();
  if (mesh && mesh_coloring) {
    mesh_coloring->setMesh(*mesh);
  }
  mesh_visualizer_.drawBackground(*current_dsg_, mesh_coloring);
  mesh_visualizer_.drawObjects(*current_dsg_, getObjectMeshColors());
  needs_redraw_ = false;
}

void SpatioTemporalVisualizer::updatePlayTime() {
  if (!playing_) {
    return;
  }

  // Update the time based on the play rate.
  const double current_wall_time = ros::WallTime::now().toSec();
  const double wall_delta = current_wall_time - previous_wall_time_;
  if (wall_delta < min_time_delta_) {
    return;
  }
  const size_t time_delta = (wall_delta * dynamic_config_.get().play_rate * 1e9);
  previous_wall_time_ = current_wall_time;

  // Update the time based on the play direction.
  if (play_forward_) {
    if (time_mode_ != Config::TimeMode::QUERY) {
      robot_time_ = std::min(robot_time_ + time_delta, map_.latest());
    }
    if (time_mode_ != Config::TimeMode::ROBOT) {
      query_time_ = std::min(query_time_ + time_delta, robot_time_);
    }
  } else {
    if (time_mode_ != Config::TimeMode::ROBOT) {
      query_time_ = std::max(query_time_ - time_delta, map_.earliest());
    }
    if (time_mode_ != Config::TimeMode::QUERY) {
      robot_time_ = std::max(robot_time_ - time_delta, map_.earliest());
    }
  }

  // Check whether the limits have been reached.
  if (play_forward_) {
    if ((time_mode_ == Config::TimeMode::ROBOT && robot_time_ == map_.latest()) ||
        (time_mode_ == Config::TimeMode::QUERY && query_time_ == robot_time_) ||
        (time_mode_ == Config::TimeMode::ONLINE && robot_time_ == map_.latest())) {
      playing_ = false;
    }
  } else {
    if ((time_mode_ == Config::TimeMode::ROBOT && robot_time_ == map_.earliest()) ||
        (time_mode_ == Config::TimeMode::QUERY && query_time_ == map_.earliest()) ||
        (time_mode_ == Config::TimeMode::ONLINE && robot_time_ == map_.earliest())) {
      playing_ = false;
    }
  }
}

void SpatioTemporalVisualizer::updateDsgTime() {
  // If no change in time no change in DSG is needed.
  if (robot_time_ == prev_robot_time_ && query_time_ == prev_query_time_) {
    return;
  }
  current_dsg_ = map_.getDsgPtr(robot_time_);

  // Update time tracking.
  prev_query_time_ = query_time_;
  prev_robot_time_ = robot_time_;
  needs_redraw_ = true;
  khronos_msgs::SpatioTemporalVisualizerState msg;
  msg.robot_time = robot_time_;
  msg.query_time = query_time_;
  state_pub_.publish(msg);
}

void SpatioTemporalVisualizer::reset() {
  // Create a new empty scene graph.
  current_dsg_.reset(new DynamicSceneGraph(layer_ids_));
  current_dsg_->createDynamicLayer(DsgLayers::AGENTS, config.robot_prefix.key);

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = config.global_frame_name;
  dsg_renderer_.reset(header);

  mesh_visualizer_.resetBackground();
  mesh_visualizer_.resetObjects();
  resetDynamicObjects();
  resetStaticObjects();
  resetAgent();
}

void SpatioTemporalVisualizer::drawAgent() {
  // Draw all trajectory nodes from the query to the robot time.
  if (!current_dsg_->hasLayer(DsgLayers::AGENTS, config.robot_prefix.key)) {
    return;
  }

  const auto& layer = current_dsg_->getLayer(DsgLayers::AGENTS, config.robot_prefix.key);

  // TODO(lschmid): make this params.
  const float future_alpha = 0.4;
  const float future_scale = 0.06;
  const float past_scale = 0.12;
  const float past_alpha = 1;
  const Color color = Color(200, 255, 150);

  visualization_msgs::Marker msg;
  msg.header.frame_id = config.global_frame_name;
  msg.header.stamp = ros::Time::now();
  msg.ns = "future_agent";
  msg.id = 0;
  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::SPHERE_LIST;
  msg.color.a = future_alpha;
  msg.color.r = color.r / 255.f;
  msg.color.g = color.g / 255.f;
  msg.color.b = color.b / 255.f;
  msg.scale.x = future_scale;
  msg.scale.y = future_scale;
  msg.scale.z = future_scale;
  msg.pose.orientation.w = 1.f;

  auto past_msg = msg;
  past_msg.ns = "past_agent";
  past_msg.color.a = past_alpha;
  past_msg.scale.x = past_scale;
  past_msg.scale.y = past_scale;
  past_msg.scale.z = past_scale;

  for (const auto& node : layer.nodes()) {
    if (!node) {
      continue;
    }
    const auto& attrs = node->attributes<spark_dsg::AgentNodeAttributes>();
    if (static_cast<size_t>(node->timestamp->count()) > query_time_) {
      msg.points.emplace_back(setPoint(attrs.position.cast<float>()));
    } else {
      past_msg.points.emplace_back(setPoint(attrs.position.cast<float>()));
    }
  }

  if (!msg.points.empty()) {
    agent_pub_.publish(msg);
  }
  if (!past_msg.points.empty()) {
    agent_pub_.publish(past_msg);
  }
}

void SpatioTemporalVisualizer::resetAgent() {
  visualization_msgs::Marker msg;
  msg.header.frame_id = config.global_frame_name;
  msg.header.stamp = ros::Time::now();
  msg.ns = "future_agent";
  msg.id = 0;
  msg.action = visualization_msgs::Marker::DELETEALL;
  agent_pub_.publish(msg);
}

void SpatioTemporalVisualizer::drawDynamicObjects() {
  // Visualize all current markers.
  visualization_msgs::MarkerArray msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = config.global_frame_name;
  size_t num_objects = 0;
  if (current_dsg_->hasLayer(DsgLayers::OBJECTS)) {
    for (const auto& [id, node] : current_dsg_->getLayer(DsgLayers::OBJECTS).nodes()) {
      const auto& attrs = node->attributes<KhronosObjectAttributes>();
      if (attrs.trajectory_timestamps.empty()) {
        continue;
      }
      visualizeDynamicObject(header, attrs, num_objects++, msg);
    }
  }

  // Erase previous markers.
  for (size_t id = num_objects; id < num_prev_dynamic_objects_; ++id) {
    for (const auto& ns : {"start_bbox",
                           "current_bbox",
                           "end_bbox"
                           "future_start_box"
                           "future_end_bbox",
                           "trajectory",
                           "future_trajectory",
                           "points"}) {
      auto& marker2 = msg.markers.emplace_back();
      marker2.id = id;
      marker2.ns = ns;
      marker2.action = visualization_msgs::Marker::DELETE;
    }
  }
  num_prev_dynamic_objects_ = num_objects;
  dynamic_obj_pub_.publish(msg);
}

void SpatioTemporalVisualizer::visualizeDynamicObject(const std_msgs::Header& header,
                                                      const KhronosObjectAttributes& attrs,
                                                      const size_t id,
                                                      visualization_msgs::MarkerArray& msg) const {
  // TODO(lschmid): future potential params.
  const float line_scale = 0.05;
  const float point_scale = 0.03;
  const float future_alpha = 0.3;

  // Setup shared preliminaries.
  const Color color =
      dynamic_config_.get().dynamic_object_color == 1 ? Color::pink() : colormaps::rainbowId(id);
  const Color point_color =
      dynamic_config_.get().dynamic_object_color != 0 ? Color::pink() : colormaps::rainbowId(id);
  size_t qt_index = std::lower_bound(attrs.trajectory_timestamps.begin(),
                                     attrs.trajectory_timestamps.end(),
                                     query_time_,
                                     std::less<uint64_t>()) -
                    attrs.trajectory_timestamps.begin();
  const bool motion_started = attrs.trajectory_timestamps.front() <= query_time_;
  const bool motion_finished = attrs.trajectory_timestamps.back() + 3e8 < query_time_;
  qt_index = std::min(qt_index, attrs.trajectory_timestamps.size() - 1);
  const bool has_future = qt_index < attrs.trajectory_timestamps.size() - 1;

  std::vector<std::string> ns_not_present;
  BoundingBox bbox = attrs.bounding_box;
  if (motion_started) {
    // There is a start box and some trajectory.
    // Start bbox
    bbox.world_P_center = attrs.trajectory_positions.front();
    auto& marker = msg.markers.emplace_back(setBoundingBox(bbox, color, header));
    marker.id = id;
    marker.ns = "start_bbox";

    // Past trajectory.
    visualization_msgs::Marker& line = msg.markers.emplace_back();
    line.action = visualization_msgs::Marker::ADD;
    line.color = marker.color;
    line.header = header;
    line.id = id;
    line.ns = "trajectory";
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.points.reserve(qt_index + 1);
    line.scale.x = line_scale;
    line.pose.orientation.w = 1.f;
    for (size_t i = 0; i < qt_index; ++i) {
      line.points.emplace_back(setPoint(attrs.trajectory_positions[i]));
      line.points.emplace_back(setPoint(attrs.trajectory_positions[i + 1]));
    }

    ns_not_present.push_back("future_start_bbox");
  } else {
    // Everything in the future.
    // future start bbox.
    bbox.world_P_center = attrs.trajectory_positions.front();
    auto& marker = msg.markers.emplace_back(setBoundingBox(bbox, color, header));
    marker.id = id;
    marker.ns = "future_start_bbox";
    marker.color.a = future_alpha;

    ns_not_present.push_back("trajectory");
    ns_not_present.push_back("start_bbox");
  }

  if (motion_finished) {
    // Only the end box to add.
    // end bbox.
    bbox.world_P_center = attrs.trajectory_positions.back();
    auto& marker = msg.markers.emplace_back(setBoundingBox(bbox, color, header));
    marker.id = id;
    marker.ns = "end_bbox";

    ns_not_present.push_back("current_bbox");
    ns_not_present.push_back("points");
  } else if (motion_started) {
    // There is motion currently happening.
    // current bbox.
    if (attrs.dynamic_object_points.empty()) {
      bbox.world_P_center = attrs.trajectory_positions[qt_index];
      msg.markers.emplace_back(setBoundingBox(bbox, color, header));
    } else {
      BoundingBox current_bb(attrs.dynamic_object_points[qt_index]);
      msg.markers.emplace_back(setBoundingBox(current_bb, color, header));
    }
    auto& marker = msg.markers.back();
    marker.id = id;
    marker.ns = "current_bbox";

    // points.
    if (attrs.dynamic_object_points.empty()) {
      ns_not_present.push_back("points");
    } else {
      visualization_msgs::Marker& points = msg.markers.emplace_back();
      points.action = visualization_msgs::Marker::ADD;
      points.color = setColor(point_color);
      points.header = header;
      points.id = id;
      points.ns = "points";
      points.type = visualization_msgs::Marker::POINTS;
      points.points.reserve(attrs.dynamic_object_points[qt_index].size());
      points.scale = setScale(point_scale);
      points.pose.orientation.w = 1.f;
      for (const auto& point : attrs.dynamic_object_points[qt_index]) {
        points.points.emplace_back(setPoint(point));
      }
    }

    ns_not_present.push_back("end_bbox");
  } else {
    ns_not_present.push_back("current_bbox");
    ns_not_present.push_back("points");
    ns_not_present.push_back("end_bbox");
  }

  // If we have future knowledge of the motion.
  if (has_future) {
    // future end bbox.
    bbox.world_P_center = attrs.trajectory_positions.back();
    auto& marker = msg.markers.emplace_back(setBoundingBox(bbox, color, header));
    marker.id = id;
    marker.ns = "future_end_bbox";
    marker.color.a = future_alpha;

    // future trajectory.
    visualization_msgs::Marker& line = msg.markers.emplace_back();
    line.action = visualization_msgs::Marker::ADD;
    line.color = marker.color;
    line.header = header;
    line.id = id;
    line.ns = "future_trajectory";
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.points.reserve((attrs.trajectory_positions.size() - qt_index) * 2);
    line.scale.x = line_scale;
    line.pose.orientation.w = 1.f;
    for (size_t i = qt_index; i < attrs.trajectory_positions.size() - 1; ++i) {
      line.points.emplace_back(setPoint(attrs.trajectory_positions[i]));
      line.points.emplace_back(setPoint(attrs.trajectory_positions[i + 1]));
    }
  } else {
    ns_not_present.push_back("future_end_bbox");
    ns_not_present.push_back("future_trajectory");
  }

  for (const auto& ns : ns_not_present) {
    auto& marker2 = msg.markers.emplace_back();
    marker2.id = id;
    marker2.ns = ns;
    marker2.action = visualization_msgs::Marker::DELETE;
  }
}

void SpatioTemporalVisualizer::resetDynamicObjects() {
  visualization_msgs::MarkerArray msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = config.global_frame_name;
  // Erase previous markers.
  for (size_t id = 0; id < num_prev_dynamic_objects_; ++id) {
    for (const auto& ns : {"start_bbox",
                           "current_bbox",
                           "end_bbox"
                           "future_start_box"
                           "future_end_bbox",
                           "trajectory",
                           "future_trajectory",
                           "points"}) {
      auto& marker2 = msg.markers.emplace_back();
      marker2.id = id;
      marker2.ns = ns;
      marker2.action = visualization_msgs::Marker::DELETE;
    }
  }
  num_prev_dynamic_objects_ = 0;
  dynamic_obj_pub_.publish(msg);
}

void SpatioTemporalVisualizer::resetStaticObjects() {
  visualization_msgs::MarkerArray msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = config.global_frame_name;
  // Erase previous markers.
  for (size_t id = 0; id < num_prev_static_objects_; ++id) {
    auto& marker = msg.markers.emplace_back();
    marker.id = id;
    marker.ns = "static_bbox";
    marker.action = visualization_msgs::Marker::DELETE;
  }
  num_prev_static_objects_ = 0;
  static_obj_pub_.publish(msg);
}

void SpatioTemporalVisualizer::onConfigUpdate() { needs_redraw_ = true; }

KhronosMeshVisualizer::ObjectColors SpatioTemporalVisualizer::getObjectMeshColors() const {
  return {};
}

hydra::MeshColoring::Ptr SpatioTemporalVisualizer::getBackgroundMeshColoring() const {
  const auto& config = dynamic_config_.get();
  if (config.background_color == 1) {
    return std::make_shared<hydra::FirstSeenMeshColoring>();
  }
  if (config.background_color == 2) {
    return std::make_shared<hydra::LastSeenMeshColoring>();
  }
  return nullptr;
}

void SpatioTemporalVisualizer::recolorObjectDsgBoundingBoxes() {
  if (!current_dsg_->hasLayer(DsgLayers::OBJECTS)) {
    return;
  }

  // Setup a coloring function.
  const auto& dyn_config = dynamic_config_.get();
  std::function<Color(const KhronosObjectAttributes&, const NodeId)> coloring_fn;
  if (dyn_config.object_bbox_color == 0) {
    // Semantics.
    coloring_fn = [](const KhronosObjectAttributes& attrs, const NodeId) {
      return colormaps::rainbowId(attrs.semantic_label);
    };
  } else if (dyn_config.object_bbox_color == 1) {
    // Instance.
    coloring_fn = [](const KhronosObjectAttributes&, const NodeId id) {
      return colormaps::rainbowId(NodeSymbol(id).categoryId());
    };
  } else {
    // Presence.
    coloring_fn = [this](const KhronosObjectAttributes& attrs, const NodeId) {
      if (isPresent(attrs, query_time_)) {
        if (hasAppeared(attrs, query_time_)) {
          return Color::green();
        } else {
          return Color::blue();
        }
      } else {
        if (hasDisappeared(attrs, query_time_)) {
          return Color::red();
        }
        return colormaps::gray(0.7);
      }
    };
  }

  visualization_msgs::MarkerArray static_objs_msg;
  // Set the node colors.
  for (const auto& [id, node] : current_dsg_->getLayer(DsgLayers::OBJECTS).nodes()) {
    auto& attrs = node->attributes<KhronosObjectAttributes>();
    if (!attrs.trajectory_timestamps.empty()) {
      continue;
    }
    Color color = coloring_fn(attrs, id);
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = config.global_frame_name;

    visualizeStaticObject(header, attrs, id, color, static_objs_msg);
    num_prev_static_objects_++;
    attrs.color = color;
  }
  static_obj_pub_.publish(static_objs_msg);
}

void SpatioTemporalVisualizer::visualizeStaticObject(const std_msgs::Header& header,
                                                     const KhronosObjectAttributes& attrs,
                                                     const NodeId id,
                                                     const Color& color,
                                                     visualization_msgs::MarkerArray& msg) const {
  // TODO(lschmid): future potential params.
  const float line_scale = 0.03;
  // const float alpha = 0.3;

  auto& marker =
      msg.markers.emplace_back(setBoundingBox(attrs.bounding_box, color, header, line_scale));
  marker.id = id;
  marker.ns = "static_bbox";
}

bool SpatioTemporalVisualizer::playCb(std_srvs::SetBool::Request& req,
                                      std_srvs::SetBool::Response& /* res */) {
  playing_ = req.data;
  previous_wall_time_ = ros::WallTime::now().toSec();
  return true;
}

bool SpatioTemporalVisualizer::setPlayForwardCb(std_srvs::SetBool::Request& req,
                                                std_srvs::SetBool::Response& /* res */) {
  play_forward_ = req.data;
  return true;
}

bool SpatioTemporalVisualizer::isSetupCb(
    khronos_msgs::SpatioTemporalVisualizerSetup::Request& /* req */,
    khronos_msgs::SpatioTemporalVisualizerSetup::Response& res) {
  res.map_stamps = {map_.earliest()};
  res.map_stamps.insert(res.map_stamps.end(), map_.stamps().begin(), map_.stamps().end());
  res.initial_robot_time = robot_time_;
  res.initial_query_time = query_time_;
  res.initial_time_mode = static_cast<int>(time_mode_);
  return true;
}

bool SpatioTemporalVisualizer::setTimeModeCb(
    khronos_msgs::SpatioTemporalVisualizerSetTimeMode::Request& req,
    khronos_msgs::SpatioTemporalVisualizerSetTimeMode::Response& res) {
  // Set the time mode.
  if (req.time_mode > 2) {
    LOG(ERROR) << "Invalid time mode '" << req.time_mode << "'.";
    res.resulting_time_mode = static_cast<int>(time_mode_);
    return false;
  }
  if (playing_) {
    LOG(WARNING) << "Can not set time mode while playing.";
    res.resulting_time_mode = static_cast<int>(time_mode_);
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  time_mode_ = static_cast<Config::TimeMode>(req.time_mode);
  if (time_mode_ == Config::TimeMode::ONLINE) {
    query_time_ = robot_time_;
  }
  res.resulting_time_mode = static_cast<int>(time_mode_);
  return true;
}

bool SpatioTemporalVisualizer::setStateCb(
    khronos_msgs::SpatioTemporalVisualizerSetState::Request& req,
    khronos_msgs::SpatioTemporalVisualizerSetState::Response& /* res*/) {
  if (playing_) {
    // NOTE(lschmid): Could change this in the future.
    LOG(WARNING) << "Can not set time while playing.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (req.state.robot_time > 0) {
    robot_time_ = req.state.robot_time;
    robot_time_ = std::clamp(robot_time_, map_.earliest(), map_.latest());
  }
  if (req.state.query_time > 0) {
    query_time_ = req.state.query_time;
    query_time_ = std::clamp(query_time_, map_.earliest(), robot_time_);
  }

  return true;
}

void SpatioTemporalVisualizer::setupRos() {
  // Advertise services.
  play_srv_ = nh_.advertiseService("play", &SpatioTemporalVisualizer::playCb, this);
  set_play_forward_srv_ =
      nh_.advertiseService("set_play_forward", &SpatioTemporalVisualizer::setPlayForwardCb, this);
  set_time_mode_srv_ =
      nh_.advertiseService("set_time_mode", &SpatioTemporalVisualizer::setTimeModeCb, this);
  set_state_srv_ = nh_.advertiseService("set_state", &SpatioTemporalVisualizer::setStateCb, this);
  state_pub_ = nh_.advertise<khronos_msgs::SpatioTemporalVisualizerState>("state", 1, true);
  dynamic_obj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("dynamic_objects", 100, true);
  static_obj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("static_objects", 100, true);
  agent_pub_ = nh_.advertise<visualization_msgs::Marker>("agent", 100, true);
}

void SpatioTemporalVisualizer::loadMap() {
  LOG(INFO) << "Loading 4D map from '" << config.map_file << "' ...";
  auto map = SpatioTemporalMap::load(config.map_file);
  if (!map) {
    LOG(ERROR) << "Failed to load map from '" << config.map_file << "'.";
    return;
  }
  map_ = std::move(*map);

  // Initialize times and time mode.
  robot_time_ = config.initial_robot_time * (map_.latest() - map_.earliest()) + map_.earliest();
  query_time_ = config.initial_query_time * (map_.latest() - map_.earliest()) + map_.earliest();
  robot_time_ = std::clamp(robot_time_, map_.earliest(), map_.latest());
  query_time_ = std::clamp(query_time_, map_.earliest(), robot_time_);

  CLOG(1) << "Loaded map with " << map_.numTimeSteps() << " DSGs.\n"
          << "Earliest: " << map_.earliest() << " (" << stampToSec(map_.earliest())
          << "s)\nLatest: " << map_.latest() << " (" << stampToSec(map_.latest())
          << "s).\nRobot time: " << robot_time_ << " (" << stampToSec(robot_time_)
          << "s)\nQuery time: " << query_time_ << " (" << stampToSec(query_time_) << "s).";

  time_mode_ = config.initial_time_mode;
}

void SpatioTemporalVisualizer::initializeDynamicConfig() {
  // Initialize dynamic config in separate thread so this is not blocking the spin.
  std::thread([this]() {
    dynamic_reconfigure::Reconfigure dyn_config_init;
    auto& c = dyn_config_init.request.config;
    c.ints.emplace_back().name = "background_color";
    c.ints.back().value = config.dynamic_config.background_color;
    c.ints.emplace_back().name = "object_bbox_color";
    c.ints.back().value = config.dynamic_config.object_bbox_color;
    c.doubles.emplace_back().name = "play_rate";
    c.doubles.back().value = config.dynamic_config.play_rate;
    c.doubles.emplace_back().name = "fade_duration";
    c.doubles.back().value = config.dynamic_config.fade_duration;
    const std::string dyn_srv_name = nh_.resolveName("spatio_temporal_visualizer/set_parameters");
    ros::service::waitForService(dyn_srv_name);
    ros::service::call(dyn_srv_name, dyn_config_init);
  }).detach();
  dynamic_config_.setUpdateCallback(std::bind(&SpatioTemporalVisualizer::onConfigUpdate, this));
}

float SpatioTemporalVisualizer::stampToSec(const uint64_t stamp) const {
  return (stamp - map_.earliest()) / 1e9;
}

}  // namespace khronos
