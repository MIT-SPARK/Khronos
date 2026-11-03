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
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "khronos/active_window/data/frame_data_buffer.h"
#include "khronos/active_window/data/track.h"

namespace khronos {

/**
 * @brief Logs the observations of tracks handed to object extraction to disk (format: Hydra's
 * hydra_image_store/FORMAT.md, section 2): <log_path>/<track_id>/track.json plus color/depth/mask
 * images for the best max_images_per_object observations. Files are written by a background
 * thread.
 */
class TrackLogger {
 public:
  struct Config {
    //! Directory to write tracks to (empty = off)
    std::filesystem::path log_path;
    //! Number of observations per track that keep their images (ranked by score)
    size_t max_images_per_object = 1;
    //! How observations are scored: number of segmented pixels or 3D bounding box volume
    enum class Score { kSegmentSize, kBBoxVolume } score = Score::kSegmentSize;
    //! Also write depth images (uint16 millimeters) for kept observations
    bool save_depth = false;
    //! Record a run-length encoded mask for observations that do not keep their images
    bool save_mask_rle = true;
    //! Also keep images for tracks that extraction dropped (otherwise they are metadata only)
    bool keep_images_of_dropped_tracks = false;
    //! JPEG quality of the color images
    int jpeg_quality = 95;
    //! Maximum number of tracks waiting to be written (0 = unbounded)
    size_t max_queue_size = 100;
  } const config;

  explicit TrackLogger(const Config& config);
  ~TrackLogger();

  bool enabled() const { return static_cast<bool>(thread_); }

  /**
   * @brief Queue the track and its frames (all observations with a semantic cluster in the
   * buffer) for writing. Never throws.
   * @param extracted Whether object extraction produced a node for the track
   */
  void log(const Track& track, const FrameDataBuffer& frame_data, bool extracted);

  //! Block until all queued tracks are written (resumes a paused logger)
  void flush();

  //! Stop (or resume) writing queued tracks; queued tracks are kept, new ones are queued
  //! up to max_queue_size (mainly for tests)
  void pause(bool paused);

  //! Number of tracks dropped because the queue was full
  size_t numDropped() const;

  //! Row-major run lengths of a binary mask, starting with background: "[r0,r1,...]"
  static std::string encodeMaskRle(const cv::Mat& mask);

 private:
  struct Request {
    Track track;
    std::vector<std::pair<FrameData::Ptr, int>> frames;
    bool extracted;
  };

  void spin();
  void write(const Request& request) const;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::shared_ptr<Request>> queue_;
  bool writing_ = false;
  bool paused_ = false;
  size_t num_dropped_ = 0;
  bool should_shutdown_ = false;
  std::unique_ptr<std::thread> thread_;
};

void declare_config(TrackLogger::Config& config);

}  // namespace khronos
