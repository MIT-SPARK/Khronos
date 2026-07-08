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

#include <set>
#include <string>

#include <hydra/common/global_info.h>
#include <hydra/utils/logging.h>

#include "khronos/active_window/active_window.h"

namespace khronos {

/**
 * @brief Active-window KhronosSink that dumps raw per-observation data for every track to disk,
 * one folder per track (`track_<id>/`), for offline debugging of tracker fragmentation. Runs
 * incrementally: each frame, any track observed at the current stamp gets its latest observation
 * appended to its folder. Each data product (masks, color/depth images, poses, camera intrinsics,
 * colored point clouds, track metadata) can be toggled independently in the config.
 */
class ActiveWindowTrackSaver : public ActiveWindow::KhronosSink {
 public:
  // Config.
  struct Config : hydra::VerbosityConfig {
    Config()
        : hydra::VerbosityConfig{hydra::GlobalInfo::instance().getConfig().default_verbosity} {}

    //! Base output directory. A timestamped subdirectory (khronos_tracks_<YYYYMMDD_HHMMSS>) is
    //! created underneath it on first use to avoid collisions between runs.
    std::string output_directory;

    //! Save the binary instance mask for the track's observation this frame.
    bool save_masks = true;

    //! Save the color image and a red-mask overlay visualization for this frame.
    bool save_color = true;

    //! Save the depth image (16-bit PNG in mm + raw float32 binary) for this frame.
    bool save_depth = true;

    //! Save the sensor pose in world frame (translation, rotation, 4x4 transform) for this frame.
    bool save_pose = true;

    //! Save the camera intrinsics once per track (only for pinhole hydra::Camera sensors).
    bool save_camera_intrinsics = true;

    //! Save a masked, colored 3D point cloud (.ply) reconstructed from the vertex map this frame.
    bool save_pointcloud = false;

    //! Reference frame the saved point cloud is expressed in. One of "world" (absolute map frame,
    //! matches the raw vertex_map), "robot"/"body" (relative to the current world_T_body pose, so
    //! coincident tracks show up near the origin), or "sensor" (relative to the sensor optical pose).
    std::string pointcloud_frame = "world";

    //! Save/overwrite a track_meta.json summary (semantics, confidence, bbox, observation count).
    bool save_metadata = true;
  } const config;

  // Construction.
  explicit ActiveWindowTrackSaver(const Config& config);
  virtual ~ActiveWindowTrackSaver() = default;

  /**
   * @brief For every track observed in this frame, append its latest observation's data to its
   * track_<id>/ folder under the run's timestamped output directory.
   * @param data The current frame data (only the current frame is available; no history buffer).
   * @param map The current volumetric map (unused; present for KhronosSink interface).
   * @param tracks The current tracks in the active window.
   */
  void call(const FrameData& data, const VolumetricMap& map, const Tracks& tracks) const override;

 private:
  //! Lazily create (once) and return the timestamped base output directory for this run.
  std::string getBaseDir() const;

  //! Directory for a specific track, creating it if needed.
  std::string getTrackDir(int track_id) const;

  void saveMask(const std::string& track_dir,
                const std::string& ts_str,
                const FrameData& data,
                int semantic_cluster_id,
                cv::Mat* binary_mask_out) const;
  void saveColor(const std::string& track_dir,
                 const std::string& ts_str,
                 const FrameData& data,
                 const cv::Mat* binary_mask) const;
  void saveDepth(const std::string& track_dir, const std::string& ts_str, const FrameData& data) const;
  void savePose(const std::string& track_dir, const std::string& ts_str, const FrameData& data) const;
  void saveCameraIntrinsics(const std::string& track_dir, const FrameData& data, int track_id) const;
  void savePointcloud(const std::string& track_dir,
                      const std::string& ts_str,
                      const FrameData& data,
                      const cv::Mat* binary_mask) const;
  void saveMetadata(const std::string& track_dir, const Track& track) const;

  //! Cached base output directory for this run, computed lazily on first call().
  mutable std::string base_dir_;

  //! Track IDs whose camera_intrinsics.json has already been written.
  mutable std::set<int> intrinsics_saved_;
};

void declare_config(ActiveWindowTrackSaver::Config& config);

}  // namespace khronos
