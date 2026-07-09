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

#include "khronos/active_window/data/frame_data.h"

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <unordered_map>

#include <hydra/input/camera.h>
#include <hydra/input/sensor_extrinsics.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "khronos/utils/json_utils.h"
#include "khronos/utils/output_file_utils.h"

namespace khronos {
namespace {

using json = nlohmann::json;

void saveColorDepthPose(const FrameData& data, const std::string& dir) {
  if (!data.input.color_image.empty()) {
    cv::Mat bgr_image;
    cv::cvtColor(data.input.color_image, bgr_image, cv::COLOR_RGB2BGR);
    cv::imwrite(dir + "/color.png", bgr_image);
  }

  if (!data.input.depth_image.empty()) {
    // 16-bit PNG scaled to mm for easy viewing.
    cv::Mat depth_mm;
    data.input.depth_image.convertTo(depth_mm, CV_16UC1, 1000.0);
    cv::imwrite(dir + "/depth.png", depth_mm);

    // Raw float32 binary for exact values.
    std::ofstream depth_stream(dir + "/depth.bin", std::ios::binary);
    if (depth_stream.is_open()) {
      depth_stream.write(reinterpret_cast<const char*>(data.input.depth_image.data),
                         data.input.depth_image.total() * data.input.depth_image.elemSize());
    }
  }

  std::ofstream pose_stream(dir + "/pose.txt");
  if (pose_stream.is_open()) {
    const Eigen::Isometry3d world_T_sensor = data.input.getSensorPose();
    pose_stream << std::fixed << std::setprecision(9);

    pose_stream << "# Sensor pose in world frame\n";
    pose_stream << "# Translation (x, y, z):\n";
    pose_stream << world_T_sensor.translation().x() << " " << world_T_sensor.translation().y() << " "
               << world_T_sensor.translation().z() << "\n";

    pose_stream << "# Rotation matrix (3x3):\n";
    const Eigen::Matrix3d rotation = world_T_sensor.rotation();
    for (int i = 0; i < 3; ++i) {
      pose_stream << rotation(i, 0) << " " << rotation(i, 1) << " " << rotation(i, 2) << "\n";
    }

    pose_stream << "# Full transformation matrix (4x4):\n";
    const Eigen::Matrix4d transform = world_T_sensor.matrix();
    for (int i = 0; i < 4; ++i) {
      pose_stream << transform(i, 0) << " " << transform(i, 1) << " " << transform(i, 2) << " "
                  << transform(i, 3) << "\n";
    }
  }
}

void saveRawInt32(const cv::Mat& image, const std::string& path) {
  if (image.empty()) {
    return;
  }
  std::ofstream stream(path, std::ios::binary);
  if (stream.is_open()) {
    stream.write(reinterpret_cast<const char*>(image.data), image.total() * image.elemSize());
  }
}

json clusterToJson(const MeasurementCluster& cluster, bool is_dynamic) {
  json j{{"id", cluster.id}, {"is_dynamic", is_dynamic}, {"bounding_box", toJson(cluster.bounding_box)}};
  j["semantics"] = cluster.semantics ? toJson(*cluster.semantics) : json(nullptr);
  return j;
}

void saveClustersJson(const FrameData& data, const std::string& dir) {
  saveRawInt32(data.object_image, dir + "/object_image.bin");
  saveRawInt32(data.dynamic_image, dir + "/dynamic_image.bin");

  json clusters = json::array();
  for (const auto& cluster : data.semantic_clusters) {
    clusters.push_back(clusterToJson(cluster, /*is_dynamic=*/false));
  }
  for (const auto& cluster : data.dynamic_clusters) {
    clusters.push_back(clusterToJson(cluster, /*is_dynamic=*/true));
  }

  std::ofstream stream(dir + "/clusters.json");
  if (stream.is_open()) {
    stream << clusters.dump(2);
  }
}

// Parses the pose.txt format written by saveColorDepthPose: reads the trailing "Full
// transformation matrix (4x4)" block (the last 4 non-comment, non-empty lines).
std::optional<Eigen::Isometry3d> loadPose(const std::string& dir) {
  std::ifstream file(dir + "/pose.txt");
  if (!file.is_open()) {
    LOG(ERROR) << "[FrameData] Missing pose.txt in " << dir;
    return std::nullopt;
  }

  std::vector<std::array<double, 4>> rows;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    std::vector<double> values((std::istream_iterator<double>(iss)), std::istream_iterator<double>());
    if (values.size() == 4) {
      rows.push_back({values[0], values[1], values[2], values[3]});
    }
  }

  if (rows.size() != 4) {
    LOG(ERROR) << "[FrameData] Malformed pose.txt in " << dir
              << " (expected a trailing 4x4 matrix block)";
    return std::nullopt;
  }

  Eigen::Matrix4d matrix;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      matrix(r, c) = rows[r][c];
    }
  }
  Eigen::Isometry3d pose;
  pose.matrix() = matrix;
  return pose;
}

bool loadDepth(const std::string& dir, int width, int height, cv::Mat* depth) {
  std::ifstream file(dir + "/depth.bin", std::ios::binary);
  if (!file.is_open()) {
    LOG(ERROR) << "[FrameData] Missing depth.bin in " << dir;
    return false;
  }
  depth->create(height, width, CV_32FC1);
  file.read(reinterpret_cast<char*>(depth->data), depth->total() * depth->elemSize());
  return static_cast<bool>(file) || file.eof();
}

// Not fatal if missing/unreadable: color is not needed for tracking itself (only depth/pose/
// clusters are), only for optional visualization (e.g. overlay images); callers should treat an
// empty color_image as "not available" rather than failing the whole load.
cv::Mat loadColor(const std::string& dir) {
  cv::Mat bgr_image = cv::imread(dir + "/color.png", cv::IMREAD_COLOR);
  if (bgr_image.empty()) {
    return {};
  }
  cv::Mat rgb_image;
  cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);
  return rgb_image;
}

bool loadRawInt32(const std::string& path, int width, int height, cv::Mat* out) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    return false;
  }
  out->create(height, width, CV_32SC1);
  file.read(reinterpret_cast<char*>(out->data), out->total() * out->elemSize());
  return static_cast<bool>(file) || file.eof();
}

std::unordered_map<int, Pixels> gatherPixelsById(const cv::Mat& image,
                                                 const cv::Mat& depth_image,
                                                 float min_range,
                                                 float max_range) {
  std::unordered_map<int, Pixels> pixels_by_id;
  if (image.empty()) {
    return pixels_by_id;
  }
  for (int v = 0; v < image.rows; ++v) {
    for (int u = 0; u < image.cols; ++u) {
      const int32_t id = image.at<int32_t>(v, u);
      if (id == 0) {
        continue;
      }
      // Mirrors instance_forwarding.cpp's own pixel-validity gate (checks the raw depth/range
      // value directly, not downstream-computed geometry): skip non-finite depth (no such guard
      // exists anywhere in the production pipeline that builds cluster.pixels -- see
      // tracker_debug.md bugfix writeup -- so this reconstruction filters explicitly to keep
      // bbox/IoU computations well-defined regardless of pixel iteration order) and skip pixels
      // outside [min_range, max_range], exactly matching instance_forwarding.cpp's own check.
      const float depth = depth_image.at<float>(v, u);
      if (!std::isfinite(depth)) {
        continue;
      }
      if (depth < min_range || (max_range > 0.f && depth > max_range)) {
        continue;
      }
      pixels_by_id[id].emplace_back(u, v);
    }
  }
  return pixels_by_id;
}

void loadClustersJson(const std::string& dir,
                      int width,
                      int height,
                      float min_range,
                      float max_range,
                      FrameData* frame_data) {
  std::ifstream clusters_file(dir + "/clusters.json");
  if (!clusters_file.is_open()) {
    LOG(WARNING) << "[FrameData] Missing clusters.json in " << dir << "; treating as no detections.";
    return;
  }
  json clusters_json;
  clusters_file >> clusters_json;

  cv::Mat object_image;
  cv::Mat dynamic_image;
  if (loadRawInt32(dir + "/object_image.bin", width, height, &object_image)) {
    frame_data->object_image = object_image;
  }
  if (loadRawInt32(dir + "/dynamic_image.bin", width, height, &dynamic_image)) {
    frame_data->dynamic_image = dynamic_image;
  }

  const auto& depth_image = frame_data->input.depth_image;
  const auto object_pixels = gatherPixelsById(object_image, depth_image, min_range, max_range);
  const auto dynamic_pixels = gatherPixelsById(dynamic_image, depth_image, min_range, max_range);

  for (const auto& entry : clusters_json) {
    const bool is_dynamic = entry.at("is_dynamic").get<bool>();

    MeasurementCluster cluster;
    cluster.id = entry.at("id").get<int>();
    cluster.bounding_box = boundingBoxFromJson(entry.at("bounding_box"));
    if (!entry.at("semantics").is_null()) {
      cluster.semantics = semanticsFromJson(entry.at("semantics"));
    }

    const auto& pixels_by_id = is_dynamic ? dynamic_pixels : object_pixels;
    const auto it = pixels_by_id.find(cluster.id);
    if (it != pixels_by_id.end()) {
      cluster.pixels = it->second;
    }

    if (is_dynamic) {
      frame_data->dynamic_clusters.push_back(cluster);
    } else {
      frame_data->semantic_clusters.push_back(cluster);
    }
  }
}

}  // namespace

void FrameData::save(const std::string& observation_dir) const {
  ensureDirectoryExists(observation_dir);
  saveColorDepthPose(*this, observation_dir);
  saveClustersJson(*this, observation_dir);
}

FrameData::Ptr FrameData::load(const std::string& observation_dir,
                               const std::shared_ptr<hydra::Camera>& camera,
                               TimeStamp stamp,
                               float min_range,
                               float max_range) {
  if (!camera) {
    LOG(ERROR) << "[FrameData] load() requires a valid camera.";
    return nullptr;
  }

  const auto pose = loadPose(observation_dir);
  if (!pose) {
    return nullptr;
  }

  hydra::InputData input(camera);
  input.timestamp_ns = stamp;
  input.world_T_body = *pose;  // valid since camera extrinsics are identity (loadCameraIntrinsics).

  input.color_image = loadColor(observation_dir);

  const auto& cam_config = camera->getConfig();
  if (!loadDepth(observation_dir, cam_config.width, cam_config.height, &input.depth_image)) {
    return nullptr;
  }

  if (!camera->finalizeRepresentations(input, /*force_world_frame=*/true)) {
    LOG(ERROR) << "[FrameData] Camera::finalizeRepresentations failed for " << observation_dir;
    return nullptr;
  }

  auto frame_data = std::make_shared<FrameData>(input);
  loadClustersJson(
      observation_dir, cam_config.width, cam_config.height, min_range, max_range, frame_data.get());
  return frame_data;
}

void saveCameraIntrinsics(const hydra::Camera& camera, const std::string& run_dir) {
  const auto& cfg = camera.getConfig();
  std::ofstream stream(run_dir + "/camera_intrinsics.json");
  if (!stream.is_open()) {
    LOG(ERROR) << "[FrameData] Failed to open camera_intrinsics.json for writing in " << run_dir;
    return;
  }
  const json j{{"fx", cfg.fx},
              {"fy", cfg.fy},
              {"cx", cfg.cx},
              {"cy", cfg.cy},
              {"width", cfg.width},
              {"height", cfg.height}};
  stream << j.dump(2);
}

std::shared_ptr<hydra::Camera> loadCameraIntrinsics(const std::string& run_dir) {
  std::ifstream file(run_dir + "/camera_intrinsics.json");
  if (!file.is_open()) {
    LOG(ERROR) << "[FrameData] Missing camera_intrinsics.json in " << run_dir;
    return nullptr;
  }
  json j;
  file >> j;

  hydra::Camera::Config config;
  // Sensor::Config requires min_range > 0 and max_range > min_range; the sink does not save the
  // original sensor's range limits, so use permissive defaults wide enough for indoor/outdoor use.
  config.min_range = 0.01;
  config.max_range = 100.0;
  config.fx = j.at("fx").get<float>();
  config.fy = j.at("fy").get<float>();
  config.cx = j.at("cx").get<float>();
  config.cy = j.at("cy").get<float>();
  config.width = j.at("width").get<int>();
  config.height = j.at("height").get<int>();
  // Identity extrinsics: the saved pose.txt already IS the sensor pose in world frame, so using
  // it directly as world_T_body with identity body_T_sensor reproduces getSensorPose() exactly.
  config.extrinsics = hydra::IdentitySensorExtrinsics::Config{};

  return std::make_shared<hydra::Camera>(config, "reconstructed_camera");
}

}  // namespace khronos
