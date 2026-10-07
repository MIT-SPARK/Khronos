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

#include <gtest/gtest.h>
#include <hydra/input/camera.h>
#include <hydra/input/sensor_extrinsics.h>

#include "khronos/active_window/data/frame_data_buffer.h"

namespace khronos {
namespace {

std::shared_ptr<hydra::Camera> makeCamera() {
  hydra::Camera::Config config;
  config.min_range = 0.01;
  config.max_range = 100.0;
  config.fx = 10.0f;
  config.fy = 10.0f;
  config.cx = 1.0f;
  config.cy = 1.0f;
  config.width = 2;
  config.height = 2;
  config.extrinsics = hydra::IdentitySensorExtrinsics::Config{};
  return std::make_shared<hydra::Camera>(config, "test_camera");
}

class FrameDataBufferTest : public ::testing::Test {
 protected:
  void SetUp() override { camera = makeCamera(); }

  FrameData::Ptr makeFrame(TimeStamp stamp) const {
    hydra::InputData input(camera);
    input.timestamp_ns = stamp;
    return std::make_shared<FrameData>(input);
  }

  bool has(const FrameDataBuffer& buffer, TimeStamp stamp) const {
    return buffer.getData(stamp, "test_camera") != nullptr;
  }

  std::shared_ptr<hydra::Camera> camera;
};

TEST_F(FrameDataBufferTest, StoresEveryFrameByDefault) {
  FrameDataBuffer buffer(FrameDataBuffer::Config{});
  for (TimeStamp stamp = 1; stamp <= 5; ++stamp) {
    buffer.storeData(makeFrame(stamp));
  }
  EXPECT_EQ(buffer.size(), 5u);
  for (TimeStamp stamp = 1; stamp <= 5; ++stamp) {
    EXPECT_TRUE(has(buffer, stamp)) << stamp;
  }
}

TEST_F(FrameDataBufferTest, MaxBufferSizeEvictsOldest) {
  FrameDataBuffer::Config config;
  config.max_buffer_size = 3;
  FrameDataBuffer buffer(config);
  for (TimeStamp stamp = 1; stamp <= 5; ++stamp) {
    buffer.storeData(makeFrame(stamp));
  }
  EXPECT_EQ(buffer.size(), 3u);
  EXPECT_FALSE(has(buffer, 2));
  EXPECT_TRUE(has(buffer, 3));
  EXPECT_TRUE(has(buffer, 5));
}

TEST_F(FrameDataBufferTest, StoresEveryNthFrame) {
  FrameDataBuffer::Config config;
  config.store_every_n_frames = 3;
  FrameDataBuffer buffer(config);
  for (TimeStamp stamp = 1; stamp <= 7; ++stamp) {
    buffer.storeData(makeFrame(stamp));
    // The latest frame is always available, even if it is not kept.
    EXPECT_EQ(buffer.getLatestData().input.timestamp_ns, stamp);
  }
  // Every 3rd frame (1, 4, 7) is kept; skipped frames are only held until the next one arrives.
  for (TimeStamp stamp = 1; stamp <= 7; ++stamp) {
    EXPECT_EQ(has(buffer, stamp), stamp == 1 || stamp == 4 || stamp == 7) << stamp;
  }
}

// Regression: ActiveWindow trims unreferenced frames before every storeData(). A skipped
// (not kept) frame must never evict a frame that a track still references.
TEST_F(FrameDataBufferTest, SkippedFrameDoesNotEvictReferencedFrame) {
  FrameDataBuffer::Config config;
  config.store_every_n_frames = 2;
  FrameDataBuffer buffer(config);

  Track track;
  track.observations.emplace_back(1, 0, -1, "test_camera");  // only frame 1 is referenced

  for (TimeStamp stamp = 1; stamp <= 6; ++stamp) {
    buffer.trimBuffer({track});
    buffer.storeData(makeFrame(stamp));
    EXPECT_TRUE(has(buffer, 1)) << "referenced frame evicted when storing frame " << stamp;
  }
}

}  // namespace
}  // namespace khronos
