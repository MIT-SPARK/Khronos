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

#include "khronos/active_window/track_saver_sink.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <hydra/input/camera.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "khronos/active_window/data/frame_data.h"
#include "khronos/utils/output_file_utils.h"

namespace khronos {
namespace {

static const auto registration = config::RegistrationWithConfig<ActiveWindow::KhronosSink,
                                                                 ActiveWindowTrackSaver,
                                                                 ActiveWindowTrackSaver::Config>(
    "ActiveWindowTrackSaver");

}  // namespace

void declare_config(ActiveWindowTrackSaver::Config& config) {
  using namespace config;
  name("ActiveWindowTrackSaver");
  field(config.verbosity, "verbosity");
  field(config.output_directory, "output_directory");
  field(config.save_observations, "save_observations");
  field(config.save_masks, "save_masks");
  field(config.save_camera_intrinsics, "save_camera_intrinsics");
  field(config.save_pointcloud, "save_pointcloud");
  field(config.pointcloud_frame, "pointcloud_frame");
  field(config.save_track_json, "save_track_json");
  field(config.save_track_symlinks, "save_track_symlinks");
  checkCondition(!config.output_directory.empty(), "output_directory must not be empty");
  checkCondition(config.pointcloud_frame == "world" || config.pointcloud_frame == "robot" ||
                     config.pointcloud_frame == "body" || config.pointcloud_frame == "sensor",
                 "pointcloud_frame must be one of 'world', 'robot'/'body', or 'sensor'");
}

ActiveWindowTrackSaver::ActiveWindowTrackSaver(const Config& config)
    : config(config::checkValid(config)) {
  MLOG(1) << "[ActiveWindowTrackSaver] Initialized, output_directory: " << config.output_directory;
}

std::string ActiveWindowTrackSaver::getBaseDir() const {
  if (base_dir_.empty()) {
    const auto now = std::chrono::system_clock::now();
    const auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
    base_dir_ = config.output_directory + "/khronos_tracks_" + ss.str();
    ensureDirectoryExists(base_dir_);
    MLOG(1) << "[ActiveWindowTrackSaver] Saving track data to: " << base_dir_;
  }
  return base_dir_;
}

std::string ActiveWindowTrackSaver::getTrackDir(int track_id) const {
  const std::string track_dir = getBaseDir() + "/tracks/track_" + std::to_string(track_id);
  ensureDirectoryExists(track_dir);
  return track_dir;
}

void ActiveWindowTrackSaver::saveMask(const std::string& track_dir,
                                      const std::string& ts_str,
                                      const FrameData& data,
                                      int semantic_cluster_id,
                                      cv::Mat* binary_mask_out) const {
  if (semantic_cluster_id < 0 || data.object_image.empty()) {
    return;
  }
  const cv::Mat binary_mask = (data.object_image == semantic_cluster_id);
  cv::imwrite(track_dir + "/mask_" + ts_str + ".png", binary_mask);
  if (binary_mask_out) {
    *binary_mask_out = binary_mask;
  }
}

void ActiveWindowTrackSaver::saveOverlay(const std::string& track_dir,
                                         const std::string& ts_str,
                                         const FrameData& data,
                                         const cv::Mat* binary_mask) const {
  if (data.input.color_image.empty() || !binary_mask || binary_mask->empty()) {
    return;
  }
  cv::Mat bgr_image;
  cv::cvtColor(data.input.color_image, bgr_image, cv::COLOR_RGB2BGR);

  cv::Mat overlay = bgr_image.clone();
  overlay.setTo(cv::Scalar(0, 0, 255), *binary_mask);  // Red in BGR.
  cv::Mat blended;
  cv::addWeighted(bgr_image, 0.6, overlay, 0.4, 0, blended);
  cv::imwrite(track_dir + "/overlay_" + ts_str + ".png", blended);
}

void ActiveWindowTrackSaver::savePointcloud(const std::string& track_dir,
                                            const std::string& ts_str,
                                            const FrameData& data,
                                            const cv::Mat* binary_mask) const {
  const cv::Mat& vertex_map = data.input.vertex_map;
  if (vertex_map.empty()) {
    return;
  }
  const bool has_color = !data.input.color_image.empty() &&
                         data.input.color_image.size() == vertex_map.size();
  const bool has_mask = binary_mask && !binary_mask->empty() &&
                        binary_mask->size() == vertex_map.size();

  // vertex_map is always in world/map frame (ActiveWindow::createData requests
  // vertices_in_world_frame=true). Reference-frame transform to express saved points relative to
  // the requested frame instead, so e.g. "robot" makes spatially-coincident tracks show up near
  // the origin regardless of where the robot was in the world.
  Eigen::Isometry3f frame_T_world = Eigen::Isometry3f::Identity();
  if (config.pointcloud_frame == "robot" || config.pointcloud_frame == "body") {
    frame_T_world = data.input.world_T_body.inverse().cast<float>();
  } else if (config.pointcloud_frame == "sensor") {
    frame_T_world = data.input.getSensorPose().inverse().cast<float>();
  }

  std::vector<cv::Vec3f> points;
  std::vector<cv::Vec3b> colors;
  points.reserve(vertex_map.total());
  for (int r = 0; r < vertex_map.rows; ++r) {
    for (int c = 0; c < vertex_map.cols; ++c) {
      if (has_mask && (*binary_mask).at<uint8_t>(r, c) == 0) {
        continue;
      }
      const cv::Vec3f point = vertex_map.at<cv::Vec3f>(r, c);
      if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
        continue;
      }
      const Eigen::Vector3f point_in_frame =
          frame_T_world * Eigen::Vector3f(point[0], point[1], point[2]);
      points.push_back(cv::Vec3f(point_in_frame.x(), point_in_frame.y(), point_in_frame.z()));
      colors.push_back(has_color ? data.input.color_image.at<cv::Vec3b>(r, c)
                                 : cv::Vec3b(255, 255, 255));
    }
  }
  if (points.empty()) {
    return;
  }

  std::ofstream ply_stream(track_dir + "/cloud_" + ts_str + ".ply");
  if (!ply_stream.is_open()) {
    return;
  }
  ply_stream << "ply\nformat ascii 1.0\n"
            << "element vertex " << points.size() << "\n"
            << "property float x\nproperty float y\nproperty float z\n"
            << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            << "end_header\n";
  ply_stream << std::fixed << std::setprecision(6);
  for (size_t i = 0; i < points.size(); ++i) {
    // color_image is RGB order; write as-is (r, g, b).
    ply_stream << points[i][0] << " " << points[i][1] << " " << points[i][2] << " "
              << static_cast<int>(colors[i][0]) << " " << static_cast<int>(colors[i][1]) << " "
              << static_cast<int>(colors[i][2]) << "\n";
  }
}

void ActiveWindowTrackSaver::saveTrackJson(const std::string& track_dir, const Track& track) const {
  // Delegates to Track::save so track.json is a full, reconstructable serialization (see
  // khronos/active_window/data/track.h) rather than a lossy summary -- this is the exact input
  // the offline tracker-replay harness needs.
  track.save(track_dir + "/track.json");
}

void ActiveWindowTrackSaver::createTrackSymlinks(const std::string& track_dir,
                                                 const std::string& ts_str) const {
  namespace fs = std::filesystem;
  const std::string observation_rel = "../../observations/" + ts_str;
  const std::pair<std::string, std::string> links[] = {
      {"color_" + ts_str + ".png", observation_rel + "/color.png"},
      {"depth_" + ts_str + ".png", observation_rel + "/depth.png"},
      {"depth_" + ts_str + ".bin", observation_rel + "/depth.bin"},
      {"pose_" + ts_str + ".txt", observation_rel + "/pose.txt"},
  };
  for (const auto& [link_name, target] : links) {
    const fs::path link_path = fs::path(track_dir) / link_name;
    std::error_code ec;
    if (fs::exists(fs::symlink_status(link_path))) {
      continue;  // Already created for this observation (e.g. sink restarted mid-run).
    }
    fs::create_symlink(target, link_path, ec);
    if (ec) {
      LOG(WARNING) << "[ActiveWindowTrackSaver] Failed to symlink " << link_path << " -> " << target
                  << ": " << ec.message();
    }
  }
}

void ActiveWindowTrackSaver::call(const FrameData& data,
                                  const VolumetricMap& /*map*/,
                                  const Tracks& tracks) const {
  const std::string ts_str = std::to_string(data.input.timestamp_ns);

  if (config.save_observations) {
    data.save(getBaseDir() + "/observations/" + ts_str);
  }
  if (config.save_camera_intrinsics && !intrinsics_saved_) {
    const auto* camera = dynamic_cast<const hydra::Camera*>(&data.input.getSensor());
    if (camera) {
      saveCameraIntrinsics(*camera, getBaseDir());
      intrinsics_saved_ = true;
    }
  }

  for (const auto& track : tracks) {
    // Only save data for tracks observed in this exact frame; the sink only has access to the
    // current frame, so past observations cannot be backfilled here.
    if (track.observations.empty() || track.last_seen != data.input.timestamp_ns) {
      continue;
    }
    const Observation& obs = track.observations.back();
    const std::string track_dir = getTrackDir(track.id);

    cv::Mat binary_mask;
    if (config.save_masks) {
      saveMask(track_dir, ts_str, data, obs.semantic_cluster_id, &binary_mask);
      saveOverlay(track_dir, ts_str, data, binary_mask.empty() ? nullptr : &binary_mask);
    }
    if (config.save_pointcloud) {
      savePointcloud(track_dir, ts_str, data, binary_mask.empty() ? nullptr : &binary_mask);
    }
    if (config.save_track_json) {
      saveTrackJson(track_dir, track);
    }
    if (config.save_track_symlinks) {
      createTrackSymlinks(track_dir, ts_str);
    }
  }
}

}  // namespace khronos
