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

#include "khronos/active_window/object_extraction/track_logger.h"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <hydra/input/camera.h>
#include <hydra/utils/data_directory.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace khronos {
namespace {

using hydra::jsonNumber;
using hydra::jsonString;

// non-finite entries are written as null (JSON has no nan/inf)
std::string poseToJson(const Eigen::Isometry3d& pose) {
  std::stringstream ss;
  ss << "[";
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      ss << (r || c ? ", " : "") << jsonNumber(pose.matrix()(r, c));
    }
  }
  ss << "]";
  return ss.str();
}

std::string sensorToJson(const hydra::InputData& input, const cv::Mat& reference) {
  const auto& sensor = input.getSensor();
  const auto camera = dynamic_cast<const hydra::Camera*>(&sensor);
  std::stringstream ss;
  ss << "{\"name\": " << jsonString(sensor.name) << ", ";
  if (camera) {
    const auto& cam = camera->getConfig();
    ss << "\"width\": " << cam.width << ", \"height\": " << cam.height
       << ", \"fx\": " << jsonNumber(cam.fx) << ", \"fy\": " << jsonNumber(cam.fy)
       << ", \"cx\": " << jsonNumber(cam.cx) << ", \"cy\": " << jsonNumber(cam.cy);
  } else {
    ss << "\"width\": " << reference.cols << ", \"height\": " << reference.rows
       << ", \"fx\": null, \"fy\": null, \"cx\": null, \"cy\": null";
  }
  ss << ", \"body_T_sensor\": " << poseToJson(sensor.body_T_sensor()) << "}";
  return ss.str();
}

void writeImage(const std::filesystem::path& path,
                const cv::Mat& image,
                const std::vector<int>& params = {}) {
  std::vector<uchar> bytes;
  if (!cv::imencode(path.extension().string(), image, bytes, params)) {
    throw std::runtime_error("failed to encode '" + path.string() + "'");
  }
  hydra::writeFileAtomic(path, std::string(bytes.begin(), bytes.end()));
}

}  // namespace

void declare_config(TrackLogger::Config& config) {
  using namespace config;
  name("TrackLogger::Config");
  field<Path>(config.log_path, "log_path");
  field(config.max_images_per_object, "max_images_per_object");
  enum_field(config.score,
             "score",
             {{TrackLogger::Config::Score::kSegmentSize, "segment_size"},
              {TrackLogger::Config::Score::kBBoxVolume, "bbox_volume"}});
  field(config.save_depth, "save_depth");
  field(config.save_mask_rle, "save_mask_rle");
  field(config.keep_images_of_dropped_tracks, "keep_images_of_dropped_tracks");
  field(config.jpeg_quality, "jpeg_quality");
  field(config.max_queue_size, "max_queue_size");
  checkInRange(config.jpeg_quality, 0, 100, "jpeg_quality");
}

TrackLogger::TrackLogger(const Config& config) : config(config::checkValid(config)) {
  if (config.log_path.empty()) {
    return;
  }

  std::filesystem::create_directories(config.log_path);
  thread_ = std::make_unique<std::thread>(&TrackLogger::spin, this);
}

TrackLogger::~TrackLogger() {
  if (!thread_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    should_shutdown_ = true;
  }

  cv_.notify_all();
  thread_->join();
}

void TrackLogger::log(const Track& track, const FrameDataBuffer& frame_data, bool extracted) {
  if (!thread_) {
    return;
  }

  try {
    auto request = std::make_shared<Request>(Request{track, {}, extracted});
    for (const auto& observation : track.observations) {
      if (observation.semantic_cluster_id == -1) {
        continue;
      }

      const auto frame = frame_data.getData(observation.stamp, observation.sensor);
      if (frame) {
        request->frames.emplace_back(frame, observation.semantic_cluster_id);
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (config.max_queue_size && queue_.size() >= config.max_queue_size) {
      LOG(WARNING) << "[TrackLogger] Queue full (" << queue_.size() << "), dropping track "
                   << track.id;
      ++num_dropped_;
      return;
    }

    queue_.push_back(request);
  } catch (const std::exception& e) {
    LOG(WARNING) << "[TrackLogger] Failed to queue track " << track.id << ": " << e.what();
    return;
  }

  cv_.notify_all();
}

void TrackLogger::flush() {
  if (!thread_) {
    return;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  paused_ = false;
  cv_.notify_all();
  cv_.wait(lock, [this] { return queue_.empty() && !writing_; });
}

void TrackLogger::pause(bool paused) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    paused_ = paused;
  }

  cv_.notify_all();
}

size_t TrackLogger::numDropped() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return num_dropped_;
}

void TrackLogger::spin() {
  while (true) {
    std::shared_ptr<Request> request;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      // shutdown overrides a pause: every queued track is still written
      cv_.wait(lock, [this] { return should_shutdown_ || (!paused_ && !queue_.empty()); });
      if (queue_.empty()) {
        return;  // only exit once every queued track is written
      }

      request = queue_.front();
      queue_.pop_front();
      writing_ = true;
    }

    try {
      write(*request);
    } catch (const std::exception& e) {
      LOG(WARNING) << "[TrackLogger] Failed to write track " << request->track.id << ": "
                   << e.what();
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      writing_ = false;
    }

    cv_.notify_all();
  }
}

std::string TrackLogger::encodeMaskRle(const cv::Mat& mask) {
  std::stringstream ss;
  ss << "[";
  if (mask.empty()) {
    ss << "]";
    return ss.str();
  }

  CV_Assert(mask.type() == CV_8UC1);
  bool foreground = false;
  size_t run = 0;
  bool first = true;
  for (int r = 0; r < mask.rows; ++r) {
    const auto row = mask.ptr<uint8_t>(r);
    for (int c = 0; c < mask.cols; ++c) {
      const bool value = row[c] != 0;
      if (value == foreground) {
        ++run;
        continue;
      }

      ss << (first ? "" : ",") << run;
      first = false;
      foreground = value;
      run = 1;
    }
  }

  ss << (first ? "" : ",") << run << "]";
  return ss.str();
}

void TrackLogger::write(const Request& request) const {
  const auto& track = request.track;
  struct Candidate {
    const FrameData* frame;
    const MeasurementCluster* cluster;
    double score;
  };

  std::vector<Candidate> candidates;
  for (const auto& [frame, id] : request.frames) {
    const auto it = std::find_if(frame->semantic_clusters.begin(),
                                 frame->semantic_clusters.end(),
                                 [id = id](const auto& cluster) { return cluster.id == id; });
    if (it == frame->semantic_clusters.end()) {
      continue;
    }

    const double score = config.score == Config::Score::kBBoxVolume
                             ? static_cast<double>(it->bounding_box.volume())
                             : static_cast<double>(it->pixels.size());
    candidates.push_back({frame.get(), &*it, score});
  }

  // only the best observations of extracted tracks keep their images; tracks that extraction
  // dropped (mostly short-lived, low-confidence detections) are metadata only by default
  const size_t max_kept =
      request.extracted || config.keep_images_of_dropped_tracks ? config.max_images_per_object : 0;
  std::vector<bool> kept(candidates.size(), true);
  std::vector<size_t> order(candidates.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&candidates](size_t lhs, size_t rhs) {
    return candidates[lhs].score > candidates[rhs].score;
  });
  for (size_t i = max_kept; i < order.size(); ++i) {
    kept[order[i]] = false;
  }

  const auto track_path = config.log_path / std::to_string(track.id);
  std::filesystem::create_directories(track_path);

  std::stringstream ss;
  ss << "{\"version\": 1, \"robot_id\": " << hydra::GlobalInfo::instance().getRobotPrefix().id
     << ", \"track_id\": " << track.id << ", \"absorbed_track_ids\": [";
  bool first = true;
  for (const auto id : track.absorbed_track_ids) {
    ss << (first ? "" : ", ") << id;
    first = false;
  }

  ss << "], \"extracted\": " << (request.extracted ? "true" : "false") << ", \"semantic_label\": ";
  if (track.semantics) {
    ss << track.semantics->category_id;
  } else {
    ss << "null";
  }

  ss << ", \"observations\": [";
  for (size_t k = 0; k < candidates.size(); ++k) {
    const auto& [frame, cluster, score] = candidates[k];
    const auto& input = frame->input;
    const auto& reference = input.color_image.empty() ? input.depth_image : input.color_image;

    // full-size segmentation mask and its inclusive 2D bounding box
    cv::Mat mask;
    std::optional<cv::Rect> bbox;
    if (!reference.empty()) {
      mask = cv::Mat::zeros(reference.size(), CV_8UC1);
      for (const auto& pixel : cluster->pixels) {
        if (pixel.u >= 0 && pixel.u < mask.cols && pixel.v >= 0 && pixel.v < mask.rows) {
          mask.at<uint8_t>(pixel.v, pixel.u) = 255;
        }
      }

      const auto rect = cv::boundingRect(mask);
      if (rect.area() > 0) {
        bbox = rect;
      }
    }

    const std::string prefix = "obs_" + std::to_string(k) + "_";
    std::optional<std::string> color_file, depth_file, mask_file;
    if (kept[k]) {
      if (input.color_image.channels() == 3) {
        cv::Mat bgr;
        cv::cvtColor(input.color_image, bgr, cv::COLOR_RGB2BGR);
        color_file = prefix + "color.jpg";
        writeImage(track_path / *color_file, bgr, {cv::IMWRITE_JPEG_QUALITY, config.jpeg_quality});
      } else if (!input.color_image.empty()) {
        color_file = prefix + "color.jpg";
        writeImage(track_path / *color_file,
                   input.color_image,
                   {cv::IMWRITE_JPEG_QUALITY, config.jpeg_quality});
      }

      if (config.save_depth && input.depth_image.type() == CV_32FC1) {
        cv::Mat depth_m = input.depth_image.clone();
        cv::patchNaNs(depth_m, 0.0);
        depth_m.setTo(0.0f, depth_m == std::numeric_limits<float>::infinity());
        depth_m.setTo(0.0f, depth_m < 0.0f);
        cv::Mat depth_mm;
        depth_m.convertTo(depth_mm, CV_16UC1, 1000.0);  // saturates at 65535
        depth_file = prefix + "depth.png";
        writeImage(track_path / *depth_file, depth_mm);
      }

      if (!mask.empty()) {
        mask_file = prefix + "mask.png";
        writeImage(track_path / *mask_file, mask);
      }
    }

    const auto file_or_null = [](const std::optional<std::string>& file) {
      return file ? jsonString(*file) : std::string("null");
    };

    ss << (k ? "," : "") << "\n  {\"k\": " << k << ", \"timestamp_ns\": " << input.timestamp_ns
       << ", \"world_T_body\": " << poseToJson(input.world_T_body)
       << ", \"sensor\": " << sensorToJson(input, reference) << ", \"bbox_2d\": ";
    if (bbox) {
      ss << "[" << bbox->x << ", " << bbox->y << ", " << bbox->x + bbox->width - 1 << ", "
         << bbox->y + bbox->height - 1 << "]";
    } else {
      ss << "null";
    }

    ss << ", \"mask_rle\": ";
    if (!kept[k] && config.save_mask_rle && bbox) {
      ss << "\"" << encodeMaskRle(mask) << "\"";
    } else {
      ss << "null";
    }

    ss << ", \"score\": " << jsonNumber(score) << ", \"kept\": " << (kept[k] ? "true" : "false")
       << ", \"color\": " << file_or_null(color_file) << ", \"depth\": " << file_or_null(depth_file)
       << ", \"mask\": " << file_or_null(mask_file) << "}";
  }

  ss << (candidates.empty() ? "]}\n" : "\n]}\n");
  hydra::writeFileAtomic(track_path / "track.json", ss.str());
}

}  // namespace khronos
