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

#include <string>

#include <hydra/common/global_info.h>
#include <hydra/utils/logging.h>

#include "khronos/active_window/active_window.h"

namespace khronos {

/**
 * @brief Active-window KhronosSink that dumps raw per-frame observation data (color/depth/pose +
 * full per-frame segmentation, via FrameData::save) once per real frame into a shared
 * `observations/<stamp>/` folder, and per-track state (track.json, mask/overlay crops, point
 * clouds) into `tracks/track_<id>/` folders that symlink back to the shared observation data
 * instead of duplicating it. Used for offline debugging of tracker fragmentation (see
 * tracker_debug.md): the observations/ folder is a complete, algorithm-agnostic record of every
 * frame (saved unconditionally, regardless of whether any track claims it), so alternate
 * detector/tracker configs can be replayed against it later.
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

    //! Save observations/<stamp>/ (color/depth/pose + full per-frame segmentation) for every
    //! frame, via FrameData::save. Unconditional per frame (not gated on tracker output) so the
    //! saved data can be replayed against a different detector/tracker config later.
    bool save_observations = true;

    //! Save the binary instance mask crop for the track's observation this frame.
    bool save_masks = true;

    //! Save the camera intrinsics once per run (run_dir/camera_intrinsics.json).
    bool save_camera_intrinsics = true;

    //! Save a masked, colored 3D point cloud (.ply) reconstructed from the vertex map this frame.
    bool save_pointcloud = false;

    //! Reference frame the saved point cloud is expressed in. One of "world" (absolute map frame,
    //! matches the raw vertex_map), "robot"/"body" (relative to the current world_T_body pose, so
    //! coincident tracks show up near the origin), or "sensor" (relative to the sensor optical pose).
    std::string pointcloud_frame = "world";

    //! Save/overwrite a track.json with the full serialized Track (see Track::save), enough to
    //! reconstruct the Track object for offline tracker replay.
    bool save_track_json = true;

    //! Create symlinks in each track_<id>/ folder pointing back to the shared observations/<ts>/
    //! color/depth/pose files, for convenient single-folder browsing without duplicating the data.
    bool save_track_symlinks = true;
  } const config;

  // Construction.
  explicit ActiveWindowTrackSaver(const Config& config);
  virtual ~ActiveWindowTrackSaver() = default;

  /**
   * @brief Saves this frame's observation data (once, unconditionally) and, for every track
   * observed this frame, its track-specific state.
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
  void saveOverlay(const std::string& track_dir,
                   const std::string& ts_str,
                   const FrameData& data,
                   const cv::Mat* binary_mask) const;
  void savePointcloud(const std::string& track_dir,
                      const std::string& ts_str,
                      const FrameData& data,
                      const cv::Mat* binary_mask) const;
  void saveTrackJson(const std::string& track_dir, const Track& track) const;
  void createTrackSymlinks(const std::string& track_dir, const std::string& ts_str) const;

  //! Cached base output directory for this run, computed lazily on first call().
  mutable std::string base_dir_;

  //! Whether camera_intrinsics.json has already been written for this run.
  mutable bool intrinsics_saved_ = false;
};

void declare_config(ActiveWindowTrackSaver::Config& config);

}  // namespace khronos
