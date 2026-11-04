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
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <unordered_set>

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

std::string encodeImage(const std::string& extension,
                        const cv::Mat& image,
                        const std::vector<int>& params = {}) {
  std::vector<uchar> bytes;
  if (!cv::imencode(extension, image, bytes, params)) {
    throw std::runtime_error("failed to encode a '" + extension + "' image");
  }

  return std::string(bytes.begin(), bytes.end());
}

// Ranking of observations for keeping images: higher score first, then older, then by sensor (a
// total order, so the kept set does not depend on which frames were still buffered)
bool ranksBefore(double lhs_score,
                 TimeStamp lhs_stamp,
                 const std::string& lhs_sensor,
                 double rhs_score,
                 TimeStamp rhs_stamp,
                 const std::string& rhs_sensor) {
  if (lhs_score != rhs_score) {
    return lhs_score > rhs_score;
  }

  return lhs_stamp != rhs_stamp ? lhs_stamp < rhs_stamp : lhs_sensor < rhs_sensor;
}

bool ranksBefore(const TrackLogger::EvictedObservation& lhs,
                 const TrackLogger::EvictedObservation& rhs) {
  return ranksBefore(lhs.score, lhs.stamp, lhs.sensor, rhs.score, rhs.stamp, rhs.sensor);
}

const MeasurementCluster* findCluster(const FrameData& frame, int id) {
  const auto it =
      std::find_if(frame.semantic_clusters.begin(),
                   frame.semantic_clusters.end(),
                   [id](const MeasurementCluster& cluster) { return cluster.id == id; });
  return it == frame.semantic_clusters.end() ? nullptr : &*it;
}

// An observation's full-size segmentation mask, its inclusive 2D bounding box and its
// track.json metadata members
struct RenderedObservation {
  cv::Mat mask;
  std::optional<cv::Rect> bbox;
  std::string metadata;
};

RenderedObservation render(const FrameData& frame, const MeasurementCluster& cluster) {
  RenderedObservation result;
  const auto& input = frame.input;
  const auto& reference = input.color_image.empty() ? input.depth_image : input.color_image;
  if (!reference.empty()) {
    result.mask = cv::Mat::zeros(reference.size(), CV_8UC1);
    for (const auto& pixel : cluster.pixels) {
      if (pixel.u >= 0 && pixel.u < result.mask.cols && pixel.v >= 0 &&
          pixel.v < result.mask.rows) {
        result.mask.at<uint8_t>(pixel.v, pixel.u) = 255;
      }
    }

    const auto rect = cv::boundingRect(result.mask);
    if (rect.area() > 0) {
      result.bbox = rect;
    }
  }

  std::stringstream ss;
  ss << "\"timestamp_ns\": " << input.timestamp_ns
     << ", \"world_T_body\": " << poseToJson(input.world_T_body)
     << ", \"sensor\": " << sensorToJson(input, reference) << ", \"bbox_2d\": ";
  if (result.bbox) {
    const auto& bbox = *result.bbox;
    ss << "[" << bbox.x << ", " << bbox.y << ", " << bbox.x + bbox.width - 1 << ", "
       << bbox.y + bbox.height - 1 << "]";
  } else {
    ss << "null";
  }

  result.metadata = ss.str();
  return result;
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

  LOG_IF(INFO, peak_evicted_bytes_ > 0)
      << "[TrackLogger] Snapshots of frames dropped from the frame buffer peaked at "
      << peak_evicted_bytes_ / 1.0e6 << " MB";

  {
    std::lock_guard<std::mutex> lock(mutex_);
    should_shutdown_ = true;
  }

  cv_.notify_all();
  thread_->join();
}

void TrackLogger::snapshot(const FrameData& frame, const Tracks& tracks) {
  if (!thread_) {
    return;
  }

  try {
    const auto stamp = frame.input.timestamp_ns;
    const auto& sensor = frame.input.getSensor().name;
    std::lock_guard<std::mutex> lock(evicted_mutex_);

    // Move the snapshots of absorbed tracks to their absorber and drop those of tracks that are
    // gone (e.g. deleted, or absorbed by a track that is gone)
    std::unordered_set<int> live;
    for (const auto& track : tracks) {
      live.insert(track.id);
      for (const auto id : track.absorbed_track_ids) {
        const auto it = evicted_.find(id);
        if (it == evicted_.end()) {
          continue;
        }

        auto absorbed = std::move(it->second);
        evicted_.erase(it);
        auto& target = evicted_[track.id];
        std::move(absorbed.begin(), absorbed.end(), std::back_inserter(target));
        limitImages(target);
      }
    }

    for (auto it = evicted_.begin(); it != evicted_.end();) {
      it = live.count(it->first) ? std::next(it) : evicted_.erase(it);
    }

    std::optional<Images> frame_images;  // color and depth are encoded once per frame
    for (const auto& track : tracks) {
      for (const auto& observation : track.observations) {
        if (observation.stamp != stamp || observation.sensor != sensor ||
            observation.semantic_cluster_id == -1) {
          continue;
        }

        const auto cluster = findCluster(frame, observation.semantic_cluster_id);
        if (!cluster) {
          continue;
        }

        auto rendered = render(frame, *cluster);
        EvictedObservation evicted;
        evicted.stamp = stamp;
        evicted.sensor = sensor;
        evicted.score = score(*cluster);
        evicted.metadata = std::move(rendered.metadata);
        if (config.save_mask_rle && rendered.bbox) {
          evicted.mask_rle = encodeMaskRle(rendered.mask);
        }

        auto& track_evicted = evicted_[track.id];
        // only encode images if the observation is among the best of the track so far
        size_t num_better = 0;
        for (const auto& other : track_evicted) {
          num_better += other.images && ranksBefore(other, evicted);
        }

        if (num_better < config.max_images_per_object) {
          auto images =
              encodeImages(frame.input, rendered.mask, frame_images ? &*frame_images : nullptr);
          if (!frame_images) {
            frame_images = images;
          }

          evicted.images = std::make_shared<Images>(std::move(images));
        }

        track_evicted.push_back(std::move(evicted));
        limitImages(track_evicted);
      }
    }

    updateEvictedBytes();
  } catch (const std::exception& e) {
    LOG(WARNING) << "[TrackLogger] Failed to snapshot frame " << frame.input.timestamp_ns << ": "
                 << e.what();
  }
}

void TrackLogger::limitImages(EvictedObservations& observations) const {
  std::vector<EvictedObservation*> with_images;
  for (auto& observation : observations) {
    if (observation.images) {
      with_images.push_back(&observation);
    }
  }

  if (with_images.size() <= config.max_images_per_object) {
    return;
  }

  std::sort(with_images.begin(), with_images.end(), [](const auto* lhs, const auto* rhs) {
    return ranksBefore(*lhs, *rhs);
  });
  for (size_t i = config.max_images_per_object; i < with_images.size(); ++i) {
    with_images[i]->images.reset();
  }
}

void TrackLogger::updateEvictedBytes() {
  // shared color/depth files are counted once
  std::unordered_set<const std::string*> files;
  size_t bytes = 0;
  for (const auto& [id, observations] : evicted_) {
    bytes += observations.capacity() * sizeof(EvictedObservation);
    for (const auto& observation : observations) {
      bytes += observation.sensor.capacity() + observation.metadata.capacity() +
               observation.mask_rle.capacity();
      if (!observation.images) {
        continue;
      }

      const auto& images = *observation.images;
      bytes += sizeof(Images) + images.mask.capacity();
      for (const auto& file : {images.color, images.depth}) {
        if (file && files.insert(file.get()).second) {
          bytes += file->capacity();
        }
      }
    }
  }

  evicted_bytes_ = bytes;
  peak_evicted_bytes_ = std::max(peak_evicted_bytes_, bytes);
}

TrackLogger::EvictedObservations TrackLogger::takeEvicted(const Track& track) {
  EvictedObservations result;
  if (!thread_) {
    return result;
  }

  std::lock_guard<std::mutex> lock(evicted_mutex_);
  std::vector<int> ids(track.absorbed_track_ids.begin(), track.absorbed_track_ids.end());
  ids.push_back(track.id);
  for (const auto id : ids) {
    const auto it = evicted_.find(id);
    if (it == evicted_.end()) {
      continue;
    }

    std::move(it->second.begin(), it->second.end(), std::back_inserter(result));
    evicted_.erase(it);
  }

  // absorbed tracks may each hold their best images; only the track's best are needed
  limitImages(result);
  updateEvictedBytes();
  return result;
}

size_t TrackLogger::numEvicted(bool with_images) const {
  std::lock_guard<std::mutex> lock(evicted_mutex_);
  size_t result = 0;
  for (const auto& [id, observations] : evicted_) {
    for (const auto& observation : observations) {
      result += !with_images || observation.images;
    }
  }

  return result;
}

size_t TrackLogger::evictedBytes() const {
  std::lock_guard<std::mutex> lock(evicted_mutex_);
  return evicted_bytes_;
}

size_t TrackLogger::peakEvictedBytes() const {
  std::lock_guard<std::mutex> lock(evicted_mutex_);
  return peak_evicted_bytes_;
}

void TrackLogger::log(const Track& track,
                      const FrameDataBuffer& frame_data,
                      bool extracted,
                      EvictedObservations evicted) {
  if (!thread_) {
    return;
  }

  try {
    std::map<std::pair<TimeStamp, std::string>, size_t> evicted_index;
    for (size_t i = 0; i < evicted.size(); ++i) {
      evicted_index.emplace(std::make_pair(evicted[i].stamp, evicted[i].sensor), i);
    }

    auto request = std::make_shared<Request>(Request{track, {}, extracted});
    for (const auto& observation : track.observations) {
      if (observation.semantic_cluster_id == -1) {
        continue;
      }

      const auto frame = frame_data.getData(observation.stamp, observation.sensor);
      if (frame) {
        request->sources.push_back({frame, observation.semantic_cluster_id, std::nullopt});
        continue;
      }

      const auto it = evicted_index.find({observation.stamp, observation.sensor});
      if (it != evicted_index.end()) {
        request->sources.push_back(
            {nullptr, observation.semantic_cluster_id, std::move(evicted[it->second])});
        evicted_index.erase(it);
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

double TrackLogger::score(const MeasurementCluster& cluster) const {
  return config.score == Config::Score::kBBoxVolume
             ? static_cast<double>(cluster.bounding_box.volume())
             : static_cast<double>(cluster.pixels.size());
}

TrackLogger::Images TrackLogger::encodeImages(const hydra::InputData& input,
                                              const cv::Mat& mask,
                                              const Images* frame) const {
  Images images;
  if (frame) {
    images.color = frame->color;
    images.depth = frame->depth;
  } else {
    const std::vector<int> jpeg_params{cv::IMWRITE_JPEG_QUALITY, config.jpeg_quality};
    if (input.color_image.channels() == 3) {
      cv::Mat bgr;
      cv::cvtColor(input.color_image, bgr, cv::COLOR_RGB2BGR);
      images.color = std::make_shared<std::string>(encodeImage(".jpg", bgr, jpeg_params));
    } else if (!input.color_image.empty()) {
      images.color =
          std::make_shared<std::string>(encodeImage(".jpg", input.color_image, jpeg_params));
    }

    if (config.save_depth && input.depth_image.type() == CV_32FC1) {
      cv::Mat depth_m = input.depth_image.clone();
      cv::patchNaNs(depth_m, 0.0);
      depth_m.setTo(0.0f, depth_m == std::numeric_limits<float>::infinity());
      depth_m.setTo(0.0f, depth_m < 0.0f);
      cv::Mat depth_mm;
      depth_m.convertTo(depth_mm, CV_16UC1, 1000.0);  // saturates at 65535
      images.depth = std::make_shared<std::string>(encodeImage(".png", depth_mm));
    }
  }

  if (!mask.empty()) {
    images.mask = encodeImage(".png", mask);
  }

  return images;
}

void TrackLogger::write(const Request& request) const {
  const auto& track = request.track;
  struct Candidate {
    const FrameData* frame;  // buffered frame, or
    const MeasurementCluster* cluster;
    const EvictedObservation* evicted;  // snapshot of a dropped frame
    TimeStamp stamp;
    const std::string* sensor;
    double score;
  };

  std::vector<Candidate> candidates;
  for (const auto& source : request.sources) {
    if (source.evicted) {
      const auto& evicted = *source.evicted;
      candidates.push_back(
          {nullptr, nullptr, &evicted, evicted.stamp, &evicted.sensor, evicted.score});
      continue;
    }

    const auto cluster = findCluster(*source.frame, source.cluster_id);
    if (!cluster) {
      continue;
    }

    const auto& input = source.frame->input;
    candidates.push_back({source.frame.get(),
                          cluster,
                          nullptr,
                          input.timestamp_ns,
                          &input.getSensor().name,
                          score(*cluster)});
  }

  // only the best observations of extracted tracks keep their images; tracks that extraction
  // dropped (mostly short-lived, low-confidence detections) are metadata only by default
  const size_t max_kept =
      request.extracted || config.keep_images_of_dropped_tracks ? config.max_images_per_object : 0;
  std::vector<bool> kept(candidates.size(), true);
  std::vector<size_t> order(candidates.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&candidates](size_t lhs, size_t rhs) {
    const auto& a = candidates[lhs];
    const auto& b = candidates[rhs];
    return ranksBefore(a.score, a.stamp, *a.sensor, b.score, b.stamp, *b.sensor);
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
    const auto& candidate = candidates[k];
    std::string metadata;
    std::string mask_rle;
    std::shared_ptr<const Images> images;
    if (candidate.frame) {
      const auto rendered = render(*candidate.frame, *candidate.cluster);
      metadata = rendered.metadata;
      if (kept[k]) {
        images = std::make_shared<Images>(encodeImages(candidate.frame->input, rendered.mask));
      } else if (config.save_mask_rle && rendered.bbox) {
        mask_rle = encodeMaskRle(rendered.mask);
      }
    } else {
      metadata = candidate.evicted->metadata;
      mask_rle = candidate.evicted->mask_rle;
      images = candidate.evicted->images;
      if (kept[k] && !images) {
        // not expected: snapshots keep the images of the track's best observations
        LOG(WARNING) << "[TrackLogger] Track " << track.id << ": no images for observation "
                     << candidate.stamp;
        kept[k] = false;
      }
    }

    const std::string prefix = "obs_" + std::to_string(k) + "_";
    std::optional<std::string> color_file, depth_file, mask_file;
    if (kept[k]) {
      const auto write_file = [&](const std::string* bytes, const std::string& suffix) {
        if (!bytes || bytes->empty()) {
          return std::optional<std::string>();
        }

        const auto file = prefix + suffix;
        hydra::writeFileAtomic(track_path / file, *bytes);
        return std::optional<std::string>(file);
      };

      color_file = write_file(images->color.get(), "color.jpg");
      depth_file = write_file(images->depth.get(), "depth.png");
      mask_file = write_file(&images->mask, "mask.png");
    }

    const auto file_or_null = [](const std::optional<std::string>& file) {
      return file ? jsonString(*file) : std::string("null");
    };

    ss << (k ? "," : "") << "\n  {\"k\": " << k << ", " << metadata << ", \"mask_rle\": ";
    if (!kept[k] && !mask_rle.empty()) {
      ss << "\"" << mask_rle << "\"";
    } else {
      ss << "null";
    }

    ss << ", \"score\": " << jsonNumber(candidate.score)
       << ", \"kept\": " << (kept[k] ? "true" : "false")
       << ", \"color\": " << file_or_null(color_file) << ", \"depth\": " << file_or_null(depth_file)
       << ", \"mask\": " << file_or_null(mask_file) << "}";
  }

  ss << (candidates.empty() ? "]}\n" : "\n]}\n");
  hydra::writeFileAtomic(track_path / "track.json", ss.str());
}

}  // namespace khronos
