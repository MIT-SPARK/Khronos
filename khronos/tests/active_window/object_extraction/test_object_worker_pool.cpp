/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <khronos/active_window/object_extraction/object_worker_pool.h>

namespace khronos {
namespace {

struct SlowExtractor : ObjectExtractor {
  KhronosObjectAttributes::Ptr extractObject(const Track& track, const FrameDataBuffer&) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (track.id % 2) {
      return nullptr;  // failed extraction -> delete update
    }
    return std::make_unique<KhronosObjectAttributes>();
  }
};

}  // namespace

// join() must wait for requests still in the queue as well as running workers, so a
// blocking extraction (e.g. ActiveWindow::finishMapping) sees every submitted track.
TEST(ObjectWorkerPool, JoinWaitsForQueuedRequests) {
  ObjectWorkerPool::Config config;
  config.num_workers = 2;
  ObjectWorkerPool pool(config, std::make_unique<SlowExtractor>());
  const FrameDataBuffer frames(FrameDataBuffer::Config{});

  constexpr int num_tracks = 7;
  for (int i = 0; i < num_tracks; ++i) {
    Track track;
    track.id = i;
    pool.submit(0, track, frames);
  }

  pool.join();
  EXPECT_EQ(pool.numRunning(), 0u);

  hydra::LayerUpdate update(2);
  pool.fill(update);
  ASSERT_EQ(update.updates.size(), static_cast<size_t>(num_tracks));
  size_t num_deleted = 0;
  for (const auto& entry : update.updates) {
    ASSERT_TRUE(entry.track_id);
    const bool deleted = entry.update_type == hydra::NodeUpdate::UpdateType::Delete;
    EXPECT_EQ(deleted, *entry.track_id % 2 == 1);
    num_deleted += deleted ? 1 : 0;
  }
  EXPECT_EQ(num_deleted, 3u);
}

}  // namespace khronos
