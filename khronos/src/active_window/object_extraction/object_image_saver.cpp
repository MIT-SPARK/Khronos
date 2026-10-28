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

#include "khronos/active_window/object_extraction/object_image_saver.h"

#include <algorithm>
#include <numeric>

#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <hydra/input/camera.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace khronos {

ObjectImageSaver::ObjectImageSaver(Selection selection, const std::string& owner)
    : selection_(selection),
      owner_(owner),
      image_store_(hydra::GlobalInfo::instance().imageStore()) {}

void ObjectImageSaver::save(const Track& track, const FrameDataBuffer& frame_data) const {
  if (!image_store_) {
    return;
  }

  save(track, collectSemanticFrames(track, frame_data));
}

void ObjectImageSaver::save(const Track& track,
                            const std::vector<std::pair<FrameData::Ptr, int>>& frames) const {
  if (!image_store_) {
    return;
  }

  // Extraction runs on a worker thread, so failures must not escape.
  try {
    saveImpl(track, frames);
  } catch (const std::exception& e) {
    LOG(WARNING) << "[" << owner_ << "] Failed to save images of track " << track.id << ": "
                 << e.what();
  }
}

void ObjectImageSaver::saveImpl(const Track& track,
                                const std::vector<std::pair<FrameData::Ptr, int>>& frames) const {
  // Observations of this track and their scores.
  struct Candidate {
    const FrameData* frame;
    const MeasurementCluster* cluster;
    double score;
  };

  std::vector<Candidate> candidates;
  for (const auto& [frame, id] : frames) {
    const auto it = std::find_if(frame->semantic_clusters.begin(),
                                 frame->semantic_clusters.end(),
                                 [id = id](const auto& cluster) { return cluster.id == id; });
    if (it == frame->semantic_clusters.end()) {
      continue;
    }

    const double score = selection_ == Selection::kLargestBBox
                             ? static_cast<double>(it->bounding_box.volume())
                             : static_cast<double>(it->pixels.size());
    candidates.push_back({frame.get(), &*it, score});
  }

  if (candidates.empty()) {
    return;
  }

  // Only the best observations keep their images.
  std::vector<bool> kept(candidates.size(), true);
  if (selection_ != Selection::kAll) {
    std::vector<size_t> order(candidates.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&candidates](size_t lhs, size_t rhs) {
      return candidates[lhs].score > candidates[rhs].score;
    });
    const auto num_kept = image_store_->config.max_images_per_object;
    for (size_t i = num_kept; i < order.size(); ++i) {
      kept[order[i]] = false;
    }
  }

  auto& store = *image_store_;
  const int robot_id = hydra::GlobalInfo::instance().getRobotPrefix().id;
  std::vector<hydra::ImageStore::Observation> observations;
  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto& [frame, cluster, score] = candidates[i];
    const auto& input = frame->input;
    const auto& sensor = input.getSensor();
    const auto camera = dynamic_cast<const hydra::Camera*>(&sensor);
    hydra::ImageStore::Frame record;
    record.robot_id = robot_id;
    record.sensor_id =
        store.addSensor(robot_id,
                        sensor.name,
                        camera ? hydra::CameraCalib::fromCamera(*camera) : hydra::CameraCalib());
    record.timestamp_ns = input.timestamp_ns;
    record.kind = hydra::ImageStore::FrameKind::Observation;
    record.world_T_body = input.world_T_body;

    hydra::ImageStore::Observation obs;
    obs.timestamp_ns = input.timestamp_ns;
    obs.score = score;
    obs.kept = kept[i];

    // Axis-aligned extent of the 3D bounding box (works for AABB and RAABB).
    const auto corners = cluster->bounding_box.corners();
    Eigen::Vector3f min_c = corners[0];
    Eigen::Vector3f max_c = corners[0];
    for (const auto& c : corners) {
      min_c = min_c.cwiseMin(c);
      max_c = max_c.cwiseMax(c);
    }

    obs.bbox3d = std::make_pair(min_c.cast<double>(), max_c.cast<double>());

    // Segmentation mask of the object and its 2D bounding box (inclusive pixel bounds).
    const auto& reference = input.color_image.empty() ? input.depth_image : input.color_image;
    if (!reference.empty()) {
      cv::Mat mask = cv::Mat::zeros(reference.size(), CV_8UC1);
      for (const auto& pixel : cluster->pixels) {
        if (pixel.u >= 0 && pixel.u < mask.cols && pixel.v >= 0 && pixel.v < mask.rows) {
          mask.at<uint8_t>(pixel.v, pixel.u) = 255;
        }
      }

      const cv::Rect bbox_2d = cv::boundingRect(mask);
      if (bbox_2d.area() > 0) {
        obs.bbox2d = std::array<int, 4>{
            bbox_2d.x, bbox_2d.y, bbox_2d.x + bbox_2d.width - 1, bbox_2d.y + bbox_2d.height - 1};
      }

      obs.mask_rle = hydra::ImageStore::encodeMaskRle(mask);
      if (obs.kept) {
        obs.mask = mask;
      }
    }

    if (obs.kept) {
      const auto color =
          input.color_image.empty() ? cv::Mat() : hydra::colorToKeyframe(input.color_image);
      obs.frame_id = store.addFrame(record, color, input.depth_image);
    } else {
      obs.frame_id = store.addFrame(record);
    }

    observations.push_back(std::move(obs));
  }

  store.addObservations(robot_id, track.id, observations);
}

std::vector<std::pair<FrameData::Ptr, int>> ObjectImageSaver::collectSemanticFrames(
    const Track& track,
    const FrameDataBuffer& frame_data) {
  std::vector<std::pair<FrameData::Ptr, int>> result;
  for (const Observation& observation : track.observations) {
    if (observation.semantic_cluster_id == -1) {
      // TODO(nathan) this might need to be relaxed
      continue;
    }
    const auto frame = frame_data.getData(observation.stamp, observation.sensor);
    if (!frame) {
      continue;
    }
    result.emplace_back(frame, observation.semantic_cluster_id);
  }
  return result;
}

}  // namespace khronos
