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

#include "khronos/active_window/tracking/observation_vote_filter.h"

namespace khronos {
namespace {

constexpr int kClusterId = 1;
constexpr int kSide = 3;  // cluster is a kSide x kSide patch of pixels

std::shared_ptr<hydra::Camera> makeCamera() {
  hydra::Camera::Config config;
  config.min_range = 0.01;
  config.max_range = 100.0;
  config.fx = 10.0f;
  config.fy = 10.0f;
  config.cx = 1.0f;
  config.cy = 1.0f;
  config.width = kSide;
  config.height = kSide;
  config.extrinsics = hydra::IdentitySensorExtrinsics::Config{};
  return std::make_shared<hydra::Camera>(config, "test_camera");
}

// A frame whose single cluster covers a 0.3 m x 0.3 m patch (9 voxels at 0.1 m) at `offset`.
FrameData::Ptr makeFrame(const std::shared_ptr<hydra::Camera>& camera,
                         TimeStamp stamp,
                         const Point& offset) {
  hydra::InputData input(camera);
  input.timestamp_ns = stamp;
  input.vertex_map = cv::Mat(kSide, kSide, CV_32FC3);
  MeasurementCluster cluster;
  cluster.id = kClusterId;
  for (int v = 0; v < kSide; ++v) {
    for (int u = 0; u < kSide; ++u) {
      const Point p = offset + Point(0.1f * u + 0.05f, 0.1f * v + 0.05f, 0.05f);
      input.vertex_map.at<InputData::VertexType>(v, u) = InputData::VertexType(p.x(), p.y(), p.z());
      cluster.pixels.emplace_back(u, v);
    }
  }
  auto frame = std::make_shared<FrameData>(input);
  frame->semantic_clusters.push_back(cluster);
  return frame;
}

ObservationVoteFilter::Config enabledConfig() {
  ObservationVoteFilter::Config config;
  config.enabled = true;
  return config;
}

class ObservationVoteFilterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    camera = makeCamera();
    FrameDataBuffer::Config buffer_config;
    buffer_config.max_buffer_size = 100;
    buffer = std::make_unique<FrameDataBuffer>(buffer_config);

    // 12 consistent frames at the origin, then 3 "smeared" frames each at a different far offset.
    TimeStamp stamp = 1;
    for (int i = 0; i < 12; ++i, ++stamp) {
      addObservation(stamp, Point::Zero());
    }
    for (int i = 0; i < 3; ++i, ++stamp) {
      addObservation(stamp, Point(5.0f + i, 0.0f, 0.0f));
    }
  }

  void addObservation(TimeStamp stamp, const Point& offset) {
    buffer->storeData(makeFrame(camera, stamp, offset));
    track.observations.emplace_back(stamp, kClusterId, -1, "test_camera");
  }

  std::shared_ptr<hydra::Camera> camera;
  std::unique_ptr<FrameDataBuffer> buffer;
  Track track;
};

TEST_F(ObservationVoteFilterTest, DisabledIsNoop) {
  ObservationVoteFilter filter(ObservationVoteFilter::Config{});
  EXPECT_EQ(filter.filter(*buffer, track), 0u);
  EXPECT_EQ(track.observations.size(), 15u);
  EXPECT_TRUE(track.filtered_observations.empty());
}

TEST_F(ObservationVoteFilterTest, PrunesInconsistentObservations) {
  ObservationVoteFilter filter(enabledConfig());
  EXPECT_EQ(filter.filter(*buffer, track), 3u);
  ASSERT_EQ(track.observations.size(), 12u);
  ASSERT_EQ(track.filtered_observations.size(), 3u);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(track.filtered_observations[i].stamp, 13u + i);
  }
}

TEST_F(ObservationVoteFilterTest, RepeatedFilteringIsStable) {
  ObservationVoteFilter filter(enabledConfig());
  filter.filter(*buffer, track);
  EXPECT_EQ(filter.filter(*buffer, track), 3u);
  EXPECT_EQ(track.observations.size(), 12u);
  EXPECT_EQ(track.filtered_observations.size(), 3u);
  EXPECT_TRUE(std::is_sorted(
      track.observations.begin(), track.observations.end(), [](const auto& a, const auto& b) {
        return a.stamp < b.stamp;
      }));
}

TEST_F(ObservationVoteFilterTest, KeepsObservationsWithoutBufferedFrames) {
  track.observations.emplace_back(1000, kClusterId, -1, "test_camera");  // never buffered
  ObservationVoteFilter filter(enabledConfig());
  EXPECT_EQ(filter.filter(*buffer, track), 3u);
  EXPECT_EQ(track.observations.size(), 13u);
  EXPECT_EQ(track.observations.back().stamp, 1000u);
}

}  // namespace
}  // namespace khronos
