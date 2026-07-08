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

// Standalone round-trip check for FrameData::save/FrameData::load and saveCameraIntrinsics/
// loadCameraIntrinsics (khronos/active_window/data/frame_data.h). Builds a synthetic FrameData
// with one semantic and one dynamic cluster (distinct pixel regions, category_id, and feature
// vectors), saves it plus the camera intrinsics, reloads both, and compares field-by-field.
// Exits non-zero and prints a diagnostic on the first mismatch. This validates Step 3 of the
// tracker-debugging pipeline (see tracker_debug.md) -- the full per-frame segmentation format
// (object_image/dynamic_image + clusters.json) that resolves Step 2's "no per-frame semantics"
// caveat.

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <set>

#include <hydra/input/camera.h>
#include <hydra/input/sensor_extrinsics.h>

#include "khronos/active_window/data/frame_data.h"

namespace {

using khronos::FeatureVector;
using khronos::FrameData;
using khronos::MeasurementCluster;
using khronos::Pixel;
using khronos::SemanticClusterInfo;
using khronos::TimeStamp;

constexpr int kWidth = 8;
constexpr int kHeight = 6;
constexpr int kSemanticId = 5;
constexpr int kDynamicId = 9;
constexpr TimeStamp kStamp = 123456789;

std::shared_ptr<hydra::Camera> makeCamera() {
  hydra::Camera::Config config;
  config.min_range = 0.01;
  config.max_range = 100.0;
  config.fx = 50.5f;
  config.fy = 51.5f;
  config.cx = 4.0f;
  config.cy = 3.0f;
  config.width = kWidth;
  config.height = kHeight;
  config.extrinsics = hydra::IdentitySensorExtrinsics::Config{};
  return std::make_shared<hydra::Camera>(config, "test_camera");
}

FrameData::Ptr makeSampleFrameData(const std::shared_ptr<hydra::Camera>& camera) {
  hydra::InputData input(camera);
  input.timestamp_ns = kStamp;
  input.world_T_body = Eigen::Translation3d(1.0, 2.0, 0.5) * Eigen::Quaterniond::Identity();

  input.color_image = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(10, 20, 30));
  input.depth_image = cv::Mat(kHeight, kWidth, CV_32FC1, cv::Scalar(2.0f));

  auto frame_data = std::make_shared<FrameData>(input);

  frame_data->object_image = cv::Mat::zeros(kHeight, kWidth, CV_32SC1);
  frame_data->object_image(cv::Rect(0, 0, 2, 2)).setTo(kSemanticId);

  frame_data->dynamic_image = cv::Mat::zeros(kHeight, kWidth, CV_32SC1);
  frame_data->dynamic_image(cv::Rect(4, 3, 2, 2)).setTo(kDynamicId);

  MeasurementCluster semantic_cluster;
  semantic_cluster.id = kSemanticId;
  // Must match the object_image region set above (Rect(0, 0, 2, 2)) so the round-trip comparison
  // (original pixels vs. pixels reconstructed by scanning object_image) is apples-to-apples.
  semantic_cluster.pixels = {Pixel(0, 0), Pixel(1, 0), Pixel(0, 1), Pixel(1, 1)};
  semantic_cluster.bounding_box =
      spark_dsg::BoundingBox(Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
  FeatureVector feature(3);
  feature << 0.5f, 0.25f, 0.125f;
  semantic_cluster.semantics = SemanticClusterInfo(3, feature);
  frame_data->semantic_clusters = {semantic_cluster};

  MeasurementCluster dynamic_cluster;
  dynamic_cluster.id = kDynamicId;
  // Must match the dynamic_image region set above (Rect(4, 3, 2, 2)).
  dynamic_cluster.pixels = {Pixel(4, 3), Pixel(5, 3), Pixel(4, 4), Pixel(5, 4)};
  dynamic_cluster.bounding_box =
      spark_dsg::BoundingBox(Eigen::Vector3f(0.5f, 0.5f, 0.5f), Eigen::Vector3f(1.0f, 1.0f, 0.0f));
  dynamic_cluster.semantics = SemanticClusterInfo(-1);  // no openset feature.
  frame_data->dynamic_clusters = {dynamic_cluster};

  return frame_data;
}

bool checkEqual(bool condition, const std::string& what) {
  if (!condition) {
    std::cerr << "MISMATCH: " << what << std::endl;
  }
  return condition;
}

bool pixelsMatch(const khronos::Pixels& lhs, const khronos::Pixels& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  std::set<std::pair<int, int>> lhs_set;
  std::set<std::pair<int, int>> rhs_set;
  for (const Pixel& p : lhs) {
    lhs_set.emplace(p.u, p.v);
  }
  for (const Pixel& p : rhs) {
    rhs_set.emplace(p.u, p.v);
  }
  return lhs_set == rhs_set;
}

bool clusterMatch(const MeasurementCluster& original, const MeasurementCluster& reloaded,
                  const std::string& label) {
  bool ok = true;
  ok &= checkEqual(original.id == reloaded.id, label + ".id");
  ok &= checkEqual(pixelsMatch(original.pixels, reloaded.pixels), label + ".pixels");
  ok &= checkEqual(original.bounding_box == reloaded.bounding_box, label + ".bounding_box");
  ok &= checkEqual(original.semantics.has_value() == reloaded.semantics.has_value(),
                   label + ".semantics.has_value");
  if (ok && original.semantics) {
    ok &= checkEqual(original.semantics->category_id == reloaded.semantics->category_id,
                     label + ".semantics.category_id");
    ok &= checkEqual((original.semantics->feature - reloaded.semantics->feature).norm() < 1e-5f,
                     label + ".semantics.feature");
  }
  return ok;
}

}  // namespace

int main() {
  const auto tmp_dir = std::filesystem::temp_directory_path() / "khronos_frame_data_roundtrip_test";
  std::filesystem::remove_all(tmp_dir);
  std::filesystem::create_directories(tmp_dir);
  const auto observation_dir = tmp_dir / "observations" / std::to_string(kStamp);

  const auto camera = makeCamera();
  const auto original = makeSampleFrameData(camera);

  original->save(observation_dir.string());
  khronos::saveCameraIntrinsics(*camera, tmp_dir.string());

  const auto reloaded_camera = khronos::loadCameraIntrinsics(tmp_dir.string());
  const auto reloaded = FrameData::load(observation_dir.string(), reloaded_camera, kStamp);

  std::filesystem::remove_all(tmp_dir);

  if (!reloaded_camera) {
    std::cerr << "FrameData round-trip FAILED: loadCameraIntrinsics returned nullptr." << std::endl;
    return EXIT_FAILURE;
  }
  if (!reloaded) {
    std::cerr << "FrameData round-trip FAILED: FrameData::load returned nullptr." << std::endl;
    return EXIT_FAILURE;
  }

  bool ok = true;
  const auto& original_cfg = camera->getConfig();
  const auto& reloaded_cfg = reloaded_camera->getConfig();
  ok &= checkEqual(std::abs(original_cfg.fx - reloaded_cfg.fx) < 1e-3f, "camera.fx");
  ok &= checkEqual(std::abs(original_cfg.fy - reloaded_cfg.fy) < 1e-3f, "camera.fy");
  ok &= checkEqual(std::abs(original_cfg.cx - reloaded_cfg.cx) < 1e-3f, "camera.cx");
  ok &= checkEqual(std::abs(original_cfg.cy - reloaded_cfg.cy) < 1e-3f, "camera.cy");
  ok &= checkEqual(original_cfg.width == reloaded_cfg.width, "camera.width");
  ok &= checkEqual(original_cfg.height == reloaded_cfg.height, "camera.height");

  ok &= checkEqual(reloaded->input.timestamp_ns == kStamp, "input.timestamp_ns");
  ok &= checkEqual(reloaded->semantic_clusters.size() == 1, "semantic_clusters.size");
  ok &= checkEqual(reloaded->dynamic_clusters.size() == 1, "dynamic_clusters.size");

  if (ok && reloaded->semantic_clusters.size() == 1) {
    ok &= clusterMatch(original->semantic_clusters[0], reloaded->semantic_clusters[0], "semantic");
  }
  if (ok && reloaded->dynamic_clusters.size() == 1) {
    ok &= clusterMatch(original->dynamic_clusters[0], reloaded->dynamic_clusters[0], "dynamic");
  }

  if (!ok) {
    std::cerr << "FrameData round-trip FAILED." << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "FrameData round-trip OK: all fields match." << std::endl;
  return EXIT_SUCCESS;
}
