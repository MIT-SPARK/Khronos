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

#include "khronos/active_window/object_extraction/object_worker_pool.h"

#include <config_utilities/config.h>

namespace khronos {

using namespace std::chrono_literals;
using hydra::timing::ElapsedTimeRecorder;
using spark_dsg::NodeAttributes;

void declare_config(ObjectWorkerPool::Config& config) {
  using namespace config;
  name("ObjectWorkerPool::Config");
  field(config.num_workers, "num_workers");
  field(config.poll_time_us, "poll_time_us");
  field(config.verbosity, "verbosity");
}

ObjectWorkerPool::Request::Request(TimeStamp stamp,
                                   const Track& track,
                                   const FrameDataBuffer& frame_data)
    : stamp(stamp), track(track), frame_data(frame_data) {}

ObjectWorkerPool::ObjectWorkerPool(const Config& config,
                                   std::unique_ptr<ObjectExtractor>&& extractor)
    : config(config), should_shutdown_(false), curr_workers_(0), extractor_(std::move(extractor)) {
  if (extractor_) {
    spin_thread_ = std::make_unique<std::thread>(&ObjectWorkerPool::spin, this);
  }
}

ObjectWorkerPool::~ObjectWorkerPool() { stop(); }

void ObjectWorkerPool::stop() {
  should_shutdown_ = true;
  if (spin_thread_) {
    spin_thread_->join();
  }

  spin_thread_.reset();
}

void ObjectWorkerPool::join() {
  CLOG(5) << "Joining " << curr_workers_ << " extraction threads!";
  while (curr_workers_ > 0) {
    CLOG(10) << "Waiting on " << curr_workers_ << " extraction threads to finish!";
    std::this_thread::sleep_for(10ms);
  }
}

size_t ObjectWorkerPool::numRunning() const { return curr_workers_; }

void ObjectWorkerPool::submit(TimeStamp stamp,
                              const Track& track,
                              const FrameDataBuffer& frame_data) {
  if (!extractor_) {
    return;
  }

  work_queue_.push(std::make_shared<Request>(stamp, track, frame_data));
}

KhronosObjectAttributes::Ptr ObjectWorkerPool::runBlocking(const Track& track,
                                                           const FrameDataBuffer& data) const {
  if (!extractor_) {
    return nullptr;
  }

  return extractor_->extractObject(track, data);
}

void ObjectWorkerPool::fill(hydra::LayerUpdate& update) {
  std::lock_guard<std::mutex> lock(output_mutex_);
  std::move(output_.begin(), output_.end(), std::back_inserter(update.attributes));
  output_.clear();
}

void ObjectWorkerPool::spin() {
  if (!extractor_) {
    return;
  }

  while (!should_shutdown_) {
    if (!work_queue_.poll(config.poll_time_us)) {
      continue;
    }

    if (config.num_workers && curr_workers_ >= config.num_workers) {
      continue;
    }

    curr_workers_++;
    std::thread(&ObjectWorkerPool::runOnce, this, work_queue_.pop()).detach();
  }
}

void ObjectWorkerPool::runOnce(Request::Ptr req) const {
  const auto start = std::chrono::high_resolution_clock::now();
  auto attrs = extractor_->extractObject(req->track, req->frame_data);
  const auto stop = std::chrono::high_resolution_clock::now();

  ElapsedTimeRecorder::instance().record("active_window/extract_object", req->stamp, start - stop);

  curr_workers_--;
  if (attrs) {
    std::lock_guard<std::mutex> lock(output_mutex_);
    output_.emplace_back(std::move(attrs));
  }
}

}  // namespace khronos
