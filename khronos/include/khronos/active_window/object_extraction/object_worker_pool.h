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

#include <list>
#include <memory>
#include <thread>

#include <hydra/common/global_info.h>
#include <hydra/common/graph_update.h>
#include <hydra/common/message_queue.h>

#include "khronos/active_window/object_extraction/object_extractor.h"

namespace khronos {

/**
 * @brief Thread pool for object extraction
 */
class ObjectWorkerPool {
 public:
  struct Config {
    //! Total number of workers. If 0, use unlimited workers (not recommended).
    size_t num_workers = 2;
    //! Poll period
    size_t poll_time_us = 1000;
    //! Verbosity for worker pool
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;
  } const config;

  struct Request {
    using Ptr = std::shared_ptr<Request>;
    Request(TimeStamp stamp, const Track& track, const FrameDataBuffer& frame_data);

    TimeStamp stamp;
    Track track;
    FrameDataBuffer frame_data;
  };

  ObjectWorkerPool(const Config& config, std::unique_ptr<ObjectExtractor>&& extractor);
  ~ObjectWorkerPool();

  void stop();
  void join();
  size_t numRunning() const;

  void submit(TimeStamp stamp, const Track& track, const FrameDataBuffer& frame_data);
  KhronosObjectAttributes::Ptr runBlocking(const Track& track,
                                           const FrameDataBuffer& frame_data) const;

  void fill(hydra::LayerUpdate& update);

 private:
  void spin();
  void runOnce(Request::Ptr request) const;

  std::atomic<bool> should_shutdown_;
  std::unique_ptr<std::thread> spin_thread_;
  mutable std::atomic<size_t> curr_workers_;

  std::unique_ptr<ObjectExtractor> extractor_;
  hydra::MessageQueue<Request::Ptr> work_queue_;

  mutable std::mutex output_mutex_;
  std::unique_ptr<std::thread> thread_;
  mutable std::list<spark_dsg::NodeAttributes::Ptr> output_;
};

void declare_config(ObjectWorkerPool::Config& config);

}  // namespace khronos
