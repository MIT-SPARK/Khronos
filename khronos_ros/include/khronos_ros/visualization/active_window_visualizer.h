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

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <hydra/common/global_info.h>
#include <hydra_visualizer/utils/marker_tracker.h>
#include <ianvs/node_handle.h>
#include <khronos/active_window/active_window.h>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace khronos {

class ActiveWindowVisualizer : public ActiveWindow::KhronosSink {
 public:
  // Config.
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // Global frame name in which the map is visualized.
    std::string global_frame_name = hydra::GlobalInfo::instance().getFrames().map;

    // Publisher queue sizes.
    int queue_size = 10;

    // Height (z-coord) of all slices to be visualized in world frame in meters.
    float slice_height = 0.f;

    // If true, calls via visualize all will use this height relative to the
    // sensor pose.
    bool slice_height_is_relative = false;

    // Size of visualized points in meters
    float dynamic_point_scale = 0.03f;

    // If true visualize unknown voxels as gray in the slice. If false show only
    // observed voxels.
    bool show_unknown_voxels = true;

    // Number of color revolutions used to recolor integer IDs.
    int id_color_revolutions = 12;

    // Overlay of detections on the color image for visualization in [0, 1].
    float detection_visualization_alpha = 0.7f;

    // Width in meters of lines indicating bounding boxes.
    float bounding_box_line_width = 0.03f;
  } const config;

  // Construction.
  explicit ActiveWindowVisualizer(const Config& config, const ianvs::NodeHandle* nh = nullptr);
  virtual ~ActiveWindowVisualizer() = default;

  // Visualization.
  /**
   * @brief Visualize everything. Since visualization only happens if someone
   * subscribeds to it it's fine to call this every time.
   * @param map The current map to visualize.
   * @param data The current data after processing to visualize.
   * @param tracks The current tracks in the active window to visualize. If a bounding box for a
   * track is newly computed it will be stored in the track.
   */
  void call(const FrameData& data, const VolumetricMap& map, const Tracks& tracks) const override;

  // Aggregated visualizations.
  void visualizeAllMaps(const VolumetricMap& map, float robot_height = 0.0f) const;
  void visualizeAllFrameData(const FrameData& data) const;
  void visualizeAllTracks(const Tracks& tracks, const FrameData& data) const;

  // Individual visualizations.
  void visualizeEverFreeSlice(const VolumetricMap& map, float robot_height = 0.0f) const;
  void visualizeTsdfSlice(const VolumetricMap& map, float robot_height = 0.0f) const;
  void visualizeTrackingSlice(const VolumetricMap& map, float robot_height = 0.0f) const;
  void visualizeDynamicPoints(const FrameData& data) const;
  void visualizeDynamicImage(const FrameData& data) const;
  void visualizeObjectImage(const FrameData& data) const;
  void visualizeSemanticImage(const FrameData& data) const;
  void visualizeObjectBoundingBoxes(const FrameData& data) const;
  void visualizeTrackBoundingBoxes(const Tracks& tracks) const;
  void visualizeTrackVoxels(const Tracks& tracks) const;
  void visualizeTrackPixels(const Tracks& tracks) const;
  void visualizeTrackingImage(const Tracks& tracks, const FrameData& data) const;

 private:
  // ROS.
  ianvs::NodeHandle nh_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ever_free_slice_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tsdf_slice_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tracking_slice_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dynamic_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dynamic_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr object_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr semantic_image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_bb_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_bbox_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_voxels_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_pixels_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr track_image_pub_;

  // Variables.
  mutable rclcpp::Time stamp_;
  mutable bool stamp_is_set_ = false;
  mutable hydra::MarkerTracker object_bbs_tracker_;
  mutable hydra::MarkerTracker bbox_tracks_tracker_;
  mutable hydra::MarkerTracker voxel_tracks_tracker_;
  mutable hydra::MarkerTracker pixel_tracks_tracker_;

  // Time stamp caching for synchronization of multiple visualizations.
  rclcpp::Time getStamp() const { return stamp_is_set_ ? stamp_ : nh_.now(); }
};

void declare_config(ActiveWindowVisualizer::Config& config);

}  // namespace khronos
