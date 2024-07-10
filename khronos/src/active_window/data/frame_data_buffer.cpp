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

#include "khronos/active_window/data/frame_data_buffer.h"

#include <map>
#include <vector>

#include <config_utilities/config_utilities.h>

namespace khronos {

void declare_config(FrameDataBuffer::Config& config) {
  using namespace config;
  name("FrameDataBuffer");
  field(config.max_buffer_size, "max_buffer_size");
  field(config.store_every_n_frames, "store_every_n_frames");
  check(config.max_buffer_size, GT, 0, "max_buffer_size");
}

FrameDataBuffer::FrameDataBuffer(const Config& config) : config(config::checkValid(config)) {}

void FrameDataBuffer::trimBuffer(const Tracks& tracks) {
  // Erase all data that no track depends on anymore.
  auto it = buffer_.begin();
  while (it != buffer_.end()) {
    bool has_track = false;
    const FrameData& frame_data = **it;
    for (const Track& track : tracks) {
      // Check the frame time stamps.
      const auto it2 = std::find_if(track.observations.begin(),
                                    track.observations.end(),
                                    [&frame_data](const Observation& observation) {
                                      return observation.stamp == frame_data.input.timestamp_ns;
                                    });
      if (it2 != track.observations.end()) {
        has_track = true;
        break;
      }
    }

    if (!has_track) {
      it = buffer_.erase(it);
    } else {
      ++it;
    }
  }
  oldest_time_stamp_ = buffer_.front()->input.timestamp_ns;
}

void FrameDataBuffer::storeData(const FrameData::Ptr& data) {
  // Track how often to store a frame.
  const bool store_data = input_counter_ == 0;
  input_counter_ = (input_counter_ + 1) % config.store_every_n_frames;

  if (!store_data) {
    // Overwrtie latest frames if not adding to the buffer.
    if (!buffer_.empty()) {
      buffer_.pop_back();
    }
    buffer_.emplace_back(data);
    return;
  }

  buffer_.emplace_back(data);

  // Remove the oldest frames if the buffer size is exceeded.
  if (buffer_.size() > config.max_buffer_size) {
    buffer_.pop_front();
    oldest_time_stamp_ = buffer_.front()->input.timestamp_ns;
  }
}

FrameData::Ptr FrameDataBuffer::getData(const TimeStamp time_stamp) const {
  if (time_stamp < oldest_time_stamp_) {
    return nullptr;
  }
  const auto it =
      std::find_if(buffer_.begin(), buffer_.end(), [time_stamp](const FrameData::Ptr& data) {
        return data->input.timestamp_ns == time_stamp;
      });
  if (it == buffer_.end()) {
    return nullptr;
  }
  return *it;
}

}  // namespace khronos
