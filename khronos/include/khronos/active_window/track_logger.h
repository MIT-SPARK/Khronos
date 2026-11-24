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
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <hydra/common/robot_config.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "khronos/active_window/data/frame_data_buffer.h"
#include "khronos/active_window/data/track.h"

namespace khronos {

/**
 * @brief Logs the observations of tracks handed to object extraction to disk (format: Hydra's
 * hydra_python/image_store/FORMAT.md, section 2): <log_path>/<track_id>/track.json plus
 * color/depth/mask images for the best max_images_per_object observations, and
 * <log_path>/sensors.yaml with the record (hydra::input::writeSensor) of every sensor the tracks
 * reference. Files are written by a background thread.
 *
 * Frames can leave the frame buffer while a track still references them (max_buffer_size). The
 * owner calls snapshot() with each frame the buffer drops; the logger then keeps what it needs of
 * the live tracks' observations in that frame (metadata, plus the encoded images while the
 * observation ranks within its track's best max_images_per_object), so the logged track is the
 * same as with an unbounded buffer.
 */
class TrackLogger {
 public:
  struct Config {
    //! Directory to write tracks to (empty = off)
    std::filesystem::path log_path;
    //! Number of observations per track that keep their images (ranked by score)
    size_t max_images_per_object = 1;
    //! Score observations by segmented pixel count or 3D bounding box volume
    enum class Score { kSegmentSize, kBBoxVolume } score = Score::kSegmentSize;
    //! Also write depth images (uint16 millimeters) for kept observations
    bool save_depth = false;
    //! Record a run-length encoded mask for observations that do not keep their images
    bool save_mask_rle = true;
    //! Keep images of dropped tracks (otherwise save only metadata)
    bool keep_images_of_dropped_tracks = false;
    //! JPEG quality of the color images
    int jpeg_quality = 95;
    //! Maximum number of tracks waiting to be written (0 = unbounded)
    size_t max_queue_size = 100;
  } const config;

  explicit TrackLogger(const Config& config, const hydra::RobotConfig& robot = {});
  ~TrackLogger();

  //! Encoded images; observations in one frame share color and depth files.
  struct Images {
    std::shared_ptr<const std::string> color;  //!< JPEG
    std::shared_ptr<const std::string> depth;  //!< 16-bit PNG, millimeters
    std::string mask;                          //!< 8-bit PNG
  };

  //! Observation retained after its frame leaves the buffer
  struct EvictedObservation {
    TimeStamp stamp = 0;
    std::string sensor;
    double score = 0.0;
    //! Timestamp, pose, sensor, image size and 2D bounding box for track.json
    nlohmann::ordered_json metadata;
    //! Run-length encoded mask (empty if none or not configured)
    std::string mask_rle;
    //! Images of observations ranked within max_images_per_object
    std::shared_ptr<const Images> images;
  };

  using EvictedObservations = std::vector<EvictedObservation>;

  bool enabled() const { return static_cast<bool>(thread_); }

  /**
   * @brief Record the observations the given tracks have in a frame that is dropped from the
   * frame buffer. Snapshots of tracks absorbed by one of the tracks move to it; snapshots of
   * tracks that are no longer in `tracks` (and were not taken) are dropped. No-op if disabled.
   */
  void snapshot(const FrameData& frame, const Tracks& tracks);

  /**
   * @brief Take the snapshots of a track and of the tracks it absorbed (call when the track leaves
   * the tracker, then pass them to log()).
   */
  EvictedObservations takeEvicted(const Track& track);

  /**
   * @brief Queue the track and its frames (all observations with a semantic cluster whose frame is
   * in the buffer or in `evicted`) for writing. Never throws.
   * @param extracted Whether object extraction produced a node for the track
   * @param evicted Snapshots of the track's observations whose frames left the buffer
   */
  void log(const Track& track,
           const FrameDataBuffer& frame_data,
           bool extracted,
           EvictedObservations evicted = {});

  //! Block until all queued tracks are written
  void flush();

  //! Row-major run lengths of a binary mask, starting with background: "[r0,r1,...]"
  static std::string encodeMaskRle(const cv::Mat& mask);

 private:
  const size_t robot_id_;
  //! An observation to write: its buffered frame and cluster id, or its snapshot
  struct Source {
    FrameData::Ptr frame;
    int cluster_id = -1;
    std::optional<EvictedObservation> evicted;
  };

  struct Request {
    Track track;
    std::vector<Source> sources;
    bool extracted;
  };

  void spin();
  void write(const Request& request) const;
  //! Remember the record of a sensor for sensors.yaml (first one per name)
  void noteSensor(const hydra::Sensor& sensor) const;
  //! Rewrite sensors.yaml if a sensor was added since it was last written
  void writeSensors() const;
  double score(const MeasurementCluster& cluster) const;
  //! Encode images, reusing color and depth from frame when provided
  Images encodeImages(const hydra::InputData& input,
                      const cv::Mat& mask,
                      const Images* frame = nullptr) const;
  //! Drop the images of all but the best max_images_per_object observations
  void limitImages(EvictedObservations& observations) const;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Request> queue_;
  bool writing_ = false;
  bool should_shutdown_ = false;
  std::unique_ptr<std::thread> thread_;

  //! Sensor records (YAML text) by name, written to sensors.yaml
  mutable std::mutex sensors_mutex_;
  mutable std::map<std::string, std::string> sensors_;
  mutable bool sensors_written_ = true;

  //! Dropped-frame observations by track id; snapshot/take own access
  std::mutex evicted_mutex_;
  std::unordered_map<int, EvictedObservations> evicted_;
};

void declare_config(TrackLogger::Config& config);

}  // namespace khronos
