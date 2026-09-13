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

#include <memory>
#include <string>
#include <vector>

#include <hydra/common/global_info.h>
#include <hydra/input/input_data.h>
#include <opencv2/core.hpp>

#include "khronos/active_window/data/measurement_clusters.h"
#include "khronos/common/common_types.h"

namespace hydra {
class Camera;
}  // namespace hydra

namespace khronos {

using hydra::InputData;

/**
 * @brief Data structure that collects also other data to be passed around with
 * each input.
 */
struct FrameData {
  using Ptr = std::shared_ptr<FrameData>;
  using ConstPtr = std::shared_ptr<const FrameData>;

  // Associated raw input data.
  const InputData input;

  // Dynamic clusters in this frame.
  std::vector<MeasurementCluster> dynamic_clusters;

  // ID of detected dynamic clusters in this frame as 8UC1. 0 indicates static.
  Image dynamic_image;
  using DynamicImageType = int;

  // Semantic clusters in this frame.
  std::vector<MeasurementCluster> semantic_clusters;

  // ID of detected semantic clusters in this frame as 8UC1. 0 indicates background.
  // NOTE(lschmid): Per frame we store a maximum of 255 objects. If a larger number is
  // needed, the allocation and projective integration (interpolators) need to be updated.
  Image object_image;
  using ObjectImageType = int;

  explicit FrameData(const hydra::InputData& input) : input(input) {}

  /**
   * @brief Serialize this frame's essential-for-tracking fields to `observation_dir` (created if
   * needed): color/depth images, pose, and the full per-frame segmentation (object_image/
   * dynamic_image + clusters.json with each cluster's real id/semantics/bounding_box). Does not
   * save camera intrinsics -- see saveCameraIntrinsics/loadCameraIntrinsics below; the sensor is a
   * run-level constant, not part of any single frame.
   * @param observation_dir Destination directory, e.g. ".../observations/<stamp>".
   */
  void save(const std::string& observation_dir) const;

  /**
   * @brief Reconstruct a FrameData from a directory written by save(), given the sensor to
   * associate it with (see loadCameraIntrinsics) and its timestamp.
   * @param observation_dir Directory written by save(), e.g. ".../observations/<stamp>".
   * @param camera Sensor to associate with the reconstructed input (intrinsics + identity
   * extrinsics; see loadCameraIntrinsics).
   * @param stamp Timestamp (ns) to assign to the reconstructed frame.
   * @param min_range Minimum depth [m] for a pixel to be included in a cluster's pixels; mirrors
   * instance_forwarding.cpp's own pixel-validity gate (`min_range: 0.05` in the deployed config).
   * 0 (default) disables this check. Non-finite depth is always excluded regardless of this value
   * (no such guard exists in the production pipeline that builds cluster.pixels, so this
   * reconstruction filters explicitly to keep bbox/IoU computations well-defined).
   * @param max_range Maximum depth [m] for a pixel to be included; mirrors instance_forwarding.cpp
   * (`max_range: 10.0` in the deployed config). 0 (default) disables this check.
   * @return The reconstructed FrameData, or nullptr if any required file is missing/malformed.
   */
  static FrameData::Ptr load(const std::string& observation_dir,
                             const std::shared_ptr<hydra::Camera>& camera,
                             TimeStamp stamp,
                             float min_range = 0.f,
                             float max_range = 0.f);
};

/**
 * @brief Save the run-level camera intrinsics once (not part of any single FrameData::save()
 * call -- the sensor is constant across all frames from the same run). Free functions (not
 * members of FrameData or Camera) since hydra::Camera is not a khronos-owned type.
 * @param camera Sensor to persist.
 * @param run_dir Run root directory, e.g. ".../khronos_tracks_<run_ts>".
 */
void saveCameraIntrinsics(const hydra::Camera& camera, const std::string& run_dir);

/**
 * @brief Load the run-level camera intrinsics saved by saveCameraIntrinsics(), constructing a
 * Camera with identity extrinsics (the saved pose_<ts>.txt already IS the sensor pose in world
 * frame, so using it directly as world_T_body with identity body_T_sensor reproduces
 * getSensorPose() exactly).
 * @param run_dir Run root directory, e.g. ".../khronos_tracks_<run_ts>".
 * @return The reconstructed Camera, or nullptr if camera_intrinsics.json is missing/malformed.
 */
std::shared_ptr<hydra::Camera> loadCameraIntrinsics(const std::string& run_dir);

}  // namespace khronos
