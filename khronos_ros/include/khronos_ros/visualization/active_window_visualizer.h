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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <hydra/common/global_info.h>
#include <khronos/active_window/data/frame_data.h>
#include <khronos/active_window/data/reconstruction_types.h>
#include <khronos/active_window/data/track.h>
#include <ros/node_handle.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace khronos {

class ActiveWindowVisualizer {
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
  explicit ActiveWindowVisualizer(const ros::NodeHandle& nh);
  ActiveWindowVisualizer(const Config& config, const ros::NodeHandle& nh);
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
  void visualizeAll(const VolumetricMap& map, const FrameData& data, const Tracks& tracks);

  // Aggregated visualizations.
  void visualizeAllMaps(const VolumetricMap& map);
  void visualizeAllFrameData(const FrameData& data);
  void visualizeAllTracks(const Tracks& tracks, const FrameData& data);

  // Individual visualizations.
  void visualizeEverFreeSlice(const VolumetricMap& map) const;
  void visualizeTsdfSlice(const VolumetricMap& map) const;
  void visualizeTrackingSlice(const VolumetricMap& map) const;
  void visualizeDynamicPoints(const FrameData& data) const;
  void visualizeDynamicImage(const FrameData& data) const;
  void visualizeObjectImage(const FrameData& data) const;
  void visualizeSemanticImage(const FrameData& data) const;
  void visualizeObjectBoundingBoxes(const FrameData& data);
  void visualizeTrackBoundingBoxes(const Tracks& tracks);
  void visualizeTrackVoxels(const Tracks& tracks);
  void visualizeTrackPixels(const Tracks& tracks);
  void visualizeTrackingImage(const Tracks& tracks, const FrameData& data) const;

 private:
  // ROS.
  ros::NodeHandle nh_;
  ros::Publisher ever_free_slice_pub_;
  ros::Publisher tsdf_slice_pub_;
  ros::Publisher tracking_slice_pub_;
  ros::Publisher dynamic_points_pub_;
  ros::Publisher dynamic_image_pub_;
  ros::Publisher object_image_pub_;
  ros::Publisher semantic_image_pub_;
  ros::Publisher object_bb_pub_;
  ros::Publisher track_bbox_pub_;
  ros::Publisher track_voxels_pub_;
  ros::Publisher track_pixels_pub_;
  ros::Publisher track_image_pub_;

  // Variables.
  ros::Time stamp_;
  bool stamp_is_set_ = false;
  Transform robot_pose_;
  size_t num_previous_object_bbs_ = 0u;
  size_t num_previous_bbox_tracks_ = 0u;
  size_t num_previous_voxel_tracks_ = 0u;
  size_t num_previous_pixel_tracks_ = 0u;

  // Time stamp caching for synchronization of multiple visualizations.
  ros::Time getStamp() const { return stamp_is_set_ ? stamp_ : ros::Time::now(); }

  // Visualization Utility.
  static void deletePreviousMarkers(visualization_msgs::MarkerArray& msg,
                                    size_t num_previous_markers);
};

void declare_config(ActiveWindowVisualizer::Config& config);

}  // namespace khronos
