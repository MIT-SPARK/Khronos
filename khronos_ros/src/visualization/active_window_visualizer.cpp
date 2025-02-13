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

#include "khronos_ros/visualization/active_window_visualizer.h"

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <khronos/utils/geometry_utils.h>
#include <spark_dsg/colormaps.h>

#include "khronos_ros/utils/ros_conversions.h"
#include "khronos_ros/visualization/visualization_utils.h"

namespace khronos {

namespace colormaps = spark_dsg::colormaps;

void declare_config(ActiveWindowVisualizer::Config& config) {
  using namespace config;
  name("ActiveWindowVisualizer");
  field(config.verbosity, "verbosity");
  field(config.global_frame_name, "global_frame_name");
  field(config.queue_size, "queue_size");
  field(config.slice_height, "slice_height", "m");
  field(config.slice_height_is_relative, "slice_height_is_relative");
  field(config.show_unknown_voxels, "show_unknown_voxels");
  field(config.dynamic_point_scale, "dynamic_point_scale", "m");
  field(config.detection_visualization_alpha, "detection_visualization_alpha");
  field(config.id_color_revolutions, "id_color_revolutions");
  field(config.bounding_box_line_width, "bounding_box_line_width");

  check(config.queue_size, GT, 0, "queue_size");
  check(config.dynamic_point_scale, GT, 0, "dynamic_point_scale");
  checkCondition(!config.global_frame_name.empty(), "param 'global_frame_name' must not be empty");
  checkInRange(
      config.detection_visualization_alpha, 0.f, 1.f, "detection_visualization_alpha", false, true);
  check(config.id_color_revolutions, GT, 0, "id_color_revolutions");
  check(config.bounding_box_line_width, GT, 0, "bounding_box_line_width");
}

ActiveWindowVisualizer::ActiveWindowVisualizer(const ros::NodeHandle& nh)
    : ActiveWindowVisualizer(config::fromRos<ActiveWindowVisualizer::Config>(nh), nh) {}

ActiveWindowVisualizer::ActiveWindowVisualizer(const Config& config, const ros::NodeHandle& nh)
    : config(config::checkValid(config)), nh_(nh) {
  // Advertise all visualization topics.
  ever_free_slice_pub_ =
      nh_.advertise<visualization_msgs::Marker>("ever_free_slice", config.queue_size);
  tsdf_slice_pub_ = nh_.advertise<visualization_msgs::Marker>("tsdf_slice", config.queue_size);
  tracking_slice_pub_ =
      nh_.advertise<visualization_msgs::Marker>("tracking_slice", config.queue_size);
  dynamic_points_pub_ =
      nh_.advertise<visualization_msgs::Marker>("dynamic_points", config.queue_size);
  dynamic_image_pub_ = nh_.advertise<sensor_msgs::Image>("dynamic_image", config.queue_size);
  object_image_pub_ = nh_.advertise<sensor_msgs::Image>("object_image", config.queue_size);
  semantic_image_pub_ = nh_.advertise<sensor_msgs::Image>("semantic_image", config.queue_size);
  object_bb_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("object_bounding_boxes", config.queue_size);
  track_bbox_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("tracking/bounding_box", config.queue_size);
  track_voxels_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("tracking/voxels", config.queue_size);
  track_pixels_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("tracking/pixels", config.queue_size);
  track_image_pub_ = nh_.advertise<sensor_msgs::Image>("tracking/image", config.queue_size);
}

void ActiveWindowVisualizer::visualizeAll(const VolumetricMap& map,
                                          const FrameData& data,
                                          const Tracks& tracks) {
  stamp_.fromNSec(data.input.timestamp_ns);
  Timer timer("visualize/active_window/all", stamp_.toNSec());
  stamp_is_set_ = true;
  robot_pose_ = data.input.world_T_body;

  // Visualization.
  visualizeAllMaps(map);
  visualizeAllFrameData(data);
  visualizeAllTracks(tracks, data);
  stamp_is_set_ = false;
}

void ActiveWindowVisualizer::visualizeAllMaps(const VolumetricMap& map) {
  const bool use_stamp = !stamp_is_set_;
  if (use_stamp) {
    stamp_ = ros::Time::now();
    stamp_is_set_ = true;
  }
  Timer timer("visualize/active_window/map", stamp_.toNSec());
  visualizeEverFreeSlice(map);
  visualizeTsdfSlice(map);
  visualizeTrackingSlice(map);
  if (use_stamp) {
    stamp_is_set_ = false;
  }
}
void ActiveWindowVisualizer::visualizeAllFrameData(const FrameData& data) {
  const bool use_stamp = !stamp_is_set_;
  if (use_stamp) {
    stamp_.fromNSec(data.input.timestamp_ns);
    stamp_is_set_ = true;
  }
  Timer timer("visualize/active_window/frame_data", stamp_.toNSec());
  visualizeDynamicPoints(data);
  visualizeDynamicImage(data);
  visualizeObjectImage(data);
  visualizeSemanticImage(data);
  visualizeObjectBoundingBoxes(data);
  if (use_stamp) {
    stamp_is_set_ = false;
  }
}

void ActiveWindowVisualizer::visualizeAllTracks(const Tracks& tracks, const FrameData& data) {
  const bool use_stamp = !stamp_is_set_;
  if (use_stamp) {
    stamp_ = ros::Time::now();
    stamp_is_set_ = true;
  }
  Timer timer("visualize/active_window/tracking", stamp_.toNSec());
  visualizeTrackBoundingBoxes(tracks);
  visualizeTrackVoxels(tracks);
  visualizeTrackPixels(tracks);
  visualizeTrackingImage(tracks, data);
  if (use_stamp) {
    stamp_is_set_ = false;
  }
}

void ActiveWindowVisualizer::visualizeTrackBoundingBoxes(const Tracks& tracks) {
  if (track_bbox_pub_.getNumSubscribers() == 0u || tracks.empty()) {
    return;
  }

  visualization_msgs::MarkerArray msg;
  size_t id = 0u;
  std_msgs::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = getStamp();
  for (const Track& track : tracks) {
    BoundingBox bbox = track.last_bounding_box;
    if (bbox.type == BoundingBox::Type::INVALID) {
      // Compute the Bounding box if visualization is turned on and no Bbox exists already.
      if (!track.last_points.empty()) {
        bbox = BoundingBox(track.last_points);
      } else if (!track.last_voxels.empty()) {
        Points points;
        points.reserve(track.last_voxels.size());
        for (const auto& voxel : track.last_voxels) {
          points.emplace_back(spatial_hash::centerPointFromIndex(voxel, track.last_voxel_size));
        }
        bbox = BoundingBox(points);
      } else {
        continue;
      }
    }

    auto& marker = msg.markers.emplace_back(setBoundingBox(
        bbox, colormaps::quality(track.confidence), header, config.bounding_box_line_width));
    marker.id = id++;
  }
  deletePreviousMarkers(msg, num_previous_bbox_tracks_);
  num_previous_bbox_tracks_ = id;
  if (msg.markers.empty()) {
    return;
  }
  track_bbox_pub_.publish(msg);
}

void ActiveWindowVisualizer::visualizeTrackVoxels(const Tracks& tracks) {
  if (track_voxels_pub_.getNumSubscribers() == 0u || tracks.empty()) {
    return;
  }

  visualization_msgs::MarkerArray msg;
  size_t id = 0u;
  std_msgs::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = getStamp();
  for (const Track& track : tracks) {
    if (track.last_voxels.empty()) {
      continue;
    }
    auto& marker = msg.markers.emplace_back();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id++;
    marker.header = header;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale = setScale(track.last_voxel_size);
    marker.color = setColor(colormaps::quality(track.confidence));
    marker.points.reserve(track.last_voxels.size());
    marker.pose.orientation.w = 1.f;
    for (const auto& voxel : track.last_voxels) {
      marker.points.emplace_back(
          setPoint(spatial_hash::centerPointFromIndex(voxel, track.last_voxel_size)));
    }
  }
  deletePreviousMarkers(msg, num_previous_voxel_tracks_);
  num_previous_voxel_tracks_ = id;
  if (msg.markers.empty()) {
    return;
  }
  track_voxels_pub_.publish(msg);
}

void ActiveWindowVisualizer::visualizeTrackPixels(const Tracks& tracks) {
  if (track_pixels_pub_.getNumSubscribers() == 0u || tracks.empty()) {
    return;
  }

  visualization_msgs::MarkerArray msg;
  size_t id = 0u;
  std_msgs::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = getStamp();
  for (const Track& track : tracks) {
    if (track.last_points.empty()) {
      continue;
    }
    auto& marker = msg.markers.emplace_back();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id++;
    marker.header = header;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale = setScale(config.dynamic_point_scale);
    marker.color = setColor(colormaps::quality(track.confidence));
    marker.points.reserve(track.last_points.size());
    for (const auto& point : track.last_points) {
      marker.points.emplace_back(setPoint(point));
    }
  }
  deletePreviousMarkers(msg, num_previous_pixel_tracks_);
  num_previous_pixel_tracks_ = id;
  if (msg.markers.empty()) {
    return;
  }
  track_pixels_pub_.publish(msg);
}

void ActiveWindowVisualizer::visualizeTrackingImage(const Tracks& tracks,
                                                    const FrameData& data) const {
  if (track_image_pub_.getNumSubscribers() == 0) {
    return;
  }
  // Overlay tracking confidence pixels on the color image.
  cv_bridge::CvImage msg(std_msgs::Header(), "rgb8", data.input.color_image.clone());
  const Transform sensor_T_world = data.input.getSensorPose();
  const Sensor& sensor = data.input.getSensor();
  for (const auto& track : tracks) {
    std::set<Pixel> reprojected_pixels;

    for (const Point& point : track.last_points) {
      int u, v;
      const auto p_sensor = sensor_T_world * Eigen::Vector3d(point[0], point[1], point[2]);
      if (sensor.projectPointToImagePlane(p_sensor.cast<float>(), u, v)) {
        reprojected_pixels.emplace(u, v);
      }
    }
    const Color color = colormaps::quality(track.confidence);
    for (const auto& pixel : reprojected_pixels) {
      applyColor(
          color, msg.image.at<cv::Vec3b>(pixel.v, pixel.u), config.detection_visualization_alpha);
    }
  }

  msg.header.stamp.fromNSec(data.input.timestamp_ns);
  track_image_pub_.publish(msg.toImageMsg());
}

void ActiveWindowVisualizer::visualizeObjectBoundingBoxes(const FrameData& data) {
  if (object_bb_pub_.getNumSubscribers() == 0u) {
    return;
  }
  Timer timer("visualize/object_bb", stamp_.toNSec());

  visualization_msgs::MarkerArray msg;
  msg.markers.reserve(data.semantic_clusters.size());
  size_t id = 0u;
  std_msgs::Header header;
  header.frame_id = config.global_frame_name;
  header.stamp = getStamp();
  for (const auto& cluster : data.semantic_clusters) {
    if (cluster.bounding_box.isValid()) {
      auto& marker = msg.markers.emplace_back(
          setBoundingBox(cluster.bounding_box,
                         colormaps::rainbowId(cluster.id, config.id_color_revolutions),
                         header,
                         config.bounding_box_line_width));
      marker.id = id++;
    }
  }
  deletePreviousMarkers(msg, num_previous_object_bbs_);
  if (msg.markers.size() > 0) {
    num_previous_object_bbs_ = id;
    object_bb_pub_.publish(msg);
  }
}

void ActiveWindowVisualizer::visualizeEverFreeSlice(const VolumetricMap& map) const {
  if (ever_free_slice_pub_.getNumSubscribers() == 0) {
    return;
  }
  Timer timer("visualize_ever_free_slice", stamp_.toNSec());
  const TrackingLayer& layer = *map.getTrackingLayer();
  visualization_msgs::Marker msg;

  // Common properties.
  msg.action = visualization_msgs::Marker::ADD;
  msg.id = 0;
  msg.header.stamp = getStamp();
  msg.header.frame_id = config.global_frame_name;
  msg.type = visualization_msgs::Marker::CUBE_LIST;
  msg.scale = setScale(map.config.voxel_size);
  msg.pose.orientation.w = 1.f;

  // Setup the slice.
  float slice_height = config.slice_height;
  if (config.slice_height_is_relative) {
    slice_height += robot_pose_.translation().z();
  }
  const Point slice_coords(0, 0, slice_height);
  const VoxelKey slice_key = map.getTrackingLayer()->getVoxelKey(slice_coords);

  // Visualize.
  for (const TrackingBlock& block : layer) {
    if (block.index.z() != slice_key.first.z()) {
      continue;
    }

    for (size_t x = 0; x < block.voxels_per_side; ++x) {
      for (size_t y = 0; y < block.voxels_per_side; ++y) {
        const VoxelIndex voxel_index(x, y, slice_key.second.z());
        const TrackingVoxel& voxel = block.getVoxel(voxel_index);

        const bool is_unknown = voxel.last_observed == 0u;
        if (is_unknown && !config.show_unknown_voxels) {
          continue;
        }

        Point coords = block.getVoxelPosition(voxel_index);
        msg.points.emplace_back(setPoint(coords));
        if (is_unknown) {
          msg.colors.emplace_back(setColor(Color::gray()));
        } else if (voxel.ever_free) {
          // Free voxel.
          msg.colors.emplace_back(setColor(Color::green()));
        } else {
          // Occupied voxel.
          msg.colors.emplace_back(setColor(Color::red()));
        }
      }
    }
  }
  if (!msg.points.empty()) {
    ever_free_slice_pub_.publish(msg);
  }
}

void ActiveWindowVisualizer::visualizeTrackingSlice(const VolumetricMap& map) const {
  if (tracking_slice_pub_.getNumSubscribers() == 0 || !map.getTrackingLayer()) {
    return;
  }
  Timer timer("visualize_tracking_slice", stamp_.toNSec());
  const TrackingLayer& layer = *map.getTrackingLayer();
  visualization_msgs::Marker msg;

  // Common properties.
  msg.action = visualization_msgs::Marker::ADD;
  msg.id = 0;
  msg.header.stamp = getStamp();
  msg.header.frame_id = config.global_frame_name;
  msg.type = visualization_msgs::Marker::CUBE_LIST;
  msg.scale = setScale(map.config.voxel_size);
  msg.pose.orientation.w = 1.f;

  // Setup the slice.
  float slice_height = config.slice_height;
  if (config.slice_height_is_relative) {
    slice_height += robot_pose_.translation().z();
  }
  const Point slice_coords(0, 0, slice_height);
  const VoxelKey slice_key = layer.getVoxelKey(slice_coords);

  // Visualize.
  for (const TrackingBlock& block : layer) {
    if (block.index.z() != slice_key.first.z()) {
      continue;
    }

    for (size_t x = 0; x < block.voxels_per_side; ++x) {
      for (size_t y = 0; y < block.voxels_per_side; ++y) {
        const VoxelIndex voxel_index(x, y, slice_key.second.z());
        const TrackingVoxel& voxel = block.getVoxel(voxel_index);

        const bool is_unknown = voxel.last_observed == 0u;
        if (is_unknown && !config.show_unknown_voxels) {
          continue;
        }

        Point coords = block.getVoxelPosition(voxel_index);
        msg.points.emplace_back(setPoint(coords));
        if (is_unknown) {
          msg.colors.emplace_back(setColor(Color::gray()));
        } else {
          const float age = stamp_.toSec() - toSeconds(voxel.last_observed);
          constexpr float max_age = 3;  // TMP: should be the param.
          if (age > max_age) {
            msg.colors.emplace_back(setColor(Color::black()));
          } else {
            msg.colors.emplace_back(setColor(colormaps::ironbow(1.0f - age / max_age)));
          }
        }
      }
    }
  }
  if (!msg.points.empty()) {
    tracking_slice_pub_.publish(msg);
  }
}

void ActiveWindowVisualizer::visualizeTsdfSlice(const VolumetricMap& map) const {
  if (tsdf_slice_pub_.getNumSubscribers() == 0) {
    return;
  }
  Timer timer("visualize_tsdf_slice", stamp_.toNSec());
  const TsdfLayer& layer = map.getTsdfLayer();
  visualization_msgs::Marker msg;

  // Common properties.
  msg.action = visualization_msgs::Marker::ADD;
  msg.id = 0;
  msg.header.stamp = getStamp();
  msg.header.frame_id = config.global_frame_name;
  msg.type = visualization_msgs::Marker::CUBE_LIST;
  msg.scale = setScale(map.config.voxel_size);
  msg.pose.orientation.w = 1.f;

  // Setup the slice.
  float slice_height = config.slice_height;
  if (config.slice_height_is_relative) {
    slice_height += robot_pose_.translation().z();
  }
  const Point slice_coords(0, 0, slice_height);
  const VoxelKey slice_key = layer.getVoxelKey(slice_coords);

  // Visualize.

  for (const TsdfBlock& block : layer) {
    if (block.index.z() != slice_key.first.z()) {
      continue;
    }

    for (size_t x = 0; x < block.voxels_per_side; ++x) {
      for (size_t y = 0; y < block.voxels_per_side; ++y) {
        const VoxelIndex voxel_index(x, y, slice_key.second.z());
        const TsdfVoxel& voxel = block.getVoxel(voxel_index);
        Point coords = block.getVoxelPosition(voxel_index);
        msg.points.emplace_back(setPoint(coords));
        if (voxel.weight < 1e-6) {
          msg.colors.emplace_back(setColor(Color::gray()));
        } else {
          const float value = 0.5 + 0.5 * voxel.distance / map.config.truncation_distance;
          msg.colors.emplace_back(setColor(colormaps::quality(value)));
        }
      }
    }
  }
  if (!msg.points.empty()) {
    tsdf_slice_pub_.publish(msg);
  }
}

void ActiveWindowVisualizer::visualizeDynamicPoints(const FrameData& data) const {
  if (dynamic_points_pub_.getNumSubscribers() == 0) {
    return;
  }
  Timer timer("visualize/dynamic_points", stamp_.toNSec());

  visualization_msgs::Marker msg;
  msg.action = visualization_msgs::Marker::ADD;
  msg.id = 0;
  msg.header.stamp.fromNSec(data.input.timestamp_ns);
  msg.header.frame_id = config.global_frame_name;
  msg.type = visualization_msgs::Marker::POINTS;

  // Get all cluster points.
  for (const auto& cluster : data.dynamic_clusters) {
    for (const Pixel& p : cluster.pixels) {
      const auto& point = data.input.vertex_map.at<InputData::VertexType>(p.v, p.u);
      const Point p_W(point[0], point[1], point[2]);
      msg.points.push_back(setPoint(p_W));
    }
  }

  if (msg.points.empty()) {
    msg.action = visualization_msgs::Marker::DELETE;
  } else {
    msg.scale = setScale(config.dynamic_point_scale);
    msg.color = setColor(Color::red());
  }
  dynamic_points_pub_.publish(msg);
}

void ActiveWindowVisualizer::visualizeDynamicImage(const FrameData& data) const {
  if (dynamic_image_pub_.getNumSubscribers() == 0) {
    return;
  }
  Timer timer("visualize/dynamioc_image", stamp_.toNSec());
  // Overlay dynamic pixels as red on the color image.
  cv_bridge::CvImage msg(std_msgs::Header(), "rgb8", data.input.color_image.clone());
  for (int u = 0; u < msg.image.cols; ++u) {
    for (int v = 0; v < msg.image.rows; ++v) {
      const int id = data.dynamic_image.at<FrameData::DynamicImageType>(v, u);
      if (id != 0) {
        applyColor(colormaps::rainbowId(id, config.id_color_revolutions),
                   msg.image.at<cv::Vec3b>(v, u),
                   config.detection_visualization_alpha);
      }
    }
  }
  msg.header.stamp.fromNSec(data.input.timestamp_ns);
  dynamic_image_pub_.publish(msg.toImageMsg());
}

void ActiveWindowVisualizer::visualizeObjectImage(const FrameData& data) const {
  if (object_image_pub_.getNumSubscribers() == 0) {
    return;
  }
  Timer timer("visualize/object_iamge", stamp_.toNSec());

  // Overlay dynamic pixels colored by id on the color image.
  cv_bridge::CvImage msg(std_msgs::Header(), "rgb8", data.input.color_image.clone());
  std::set<int> ids;
  for (int u = 0; u < msg.image.cols; ++u) {
    for (int v = 0; v < msg.image.rows; ++v) {
      const int id = data.object_image.at<FrameData::ObjectImageType>(v, u);
      ids.emplace(id);
      if (id != 0) {
        applyColor(colormaps::rainbowId(id, config.id_color_revolutions),
                   msg.image.at<cv::Vec3b>(v, u),
                   config.detection_visualization_alpha);
      }
    }
  }
  msg.header.stamp.fromNSec(data.input.timestamp_ns);
  object_image_pub_.publish(msg.toImageMsg());
}

void ActiveWindowVisualizer::visualizeSemanticImage(const FrameData& data) const {
  if (semantic_image_pub_.getNumSubscribers() == 0) {
    return;
  }
  Timer timer("visualize_semantic_image", stamp_.toNSec());

  // Overlay semantic pixels colored by id on the color image.
  cv_bridge::CvImage msg(std_msgs::Header(), "rgb8", data.input.color_image.clone());
  for (int u = 0; u < msg.image.cols; ++u) {
    for (int v = 0; v < msg.image.rows; ++v) {
      const int id = data.input.label_image.at<InputData::LabelType>(v, u);
      if (id != 0) {
        applyColor(colormaps::rainbowId(id, config.id_color_revolutions),
                   msg.image.at<cv::Vec3b>(v, u),
                   config.detection_visualization_alpha);
      }
    }
  }
  msg.header.stamp.fromNSec(data.input.timestamp_ns);
  semantic_image_pub_.publish(msg.toImageMsg());
}

void ActiveWindowVisualizer::deletePreviousMarkers(visualization_msgs::MarkerArray& msg,
                                                   size_t num_previous_markers) {
  // Unfortunately this is needed by some rviz versions as the DELETE_ALL action
  // does not work.
  if (msg.markers.size() >= num_previous_markers) {
    return;
  }
  msg.markers.reserve(num_previous_markers);
  for (size_t i = msg.markers.size(); i < num_previous_markers; ++i) {
    auto& marker = msg.markers.emplace_back();
    marker.action = visualization_msgs::Marker::DELETE;
    marker.id = i;
  }
}

}  // namespace khronos
