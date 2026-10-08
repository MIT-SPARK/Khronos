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

#include "khronos/active_window/tracking/observation_vote_filter.h"

#include <algorithm>
#include <cmath>
#include <optional>

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <spatial_hash/grid.h>

#include "khronos/utils/geometry_utils.h"

namespace khronos {
namespace {

// Voxel index of every finite point of the observation's cluster, or nullopt if the observation
// cannot be evaluated (frame evicted from the buffer, or cluster not found).
std::optional<std::vector<GlobalIndex>> observationVoxels(const FrameDataBuffer& frame_data,
                                                          const Observation& obs,
                                                          const spatial_hash::Grid<GlobalIndex>& grid) {
  if (obs.semantic_cluster_id < 0) {
    return std::nullopt;
  }
  const auto frame = frame_data.getData(obs.stamp, obs.sensor);
  if (!frame) {
    return std::nullopt;
  }
  const auto cluster = std::find_if(frame->semantic_clusters.begin(),
                                    frame->semantic_clusters.end(),
                                    [&obs](const auto& c) { return c.id == obs.semantic_cluster_id; });
  if (cluster == frame->semantic_clusters.end()) {
    return std::nullopt;
  }

  std::vector<GlobalIndex> voxels;
  voxels.reserve(cluster->pixels.size());
  for (const auto& pixel : cluster->pixels) {
    const auto& p = frame->input.vertex_map.at<InputData::VertexType>(pixel.v, pixel.u);
    if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
      voxels.push_back(grid.toIndex(Point(p[0], p[1], p[2])));
    }
  }
  if (voxels.empty()) {
    return std::nullopt;
  }
  return voxels;
}

}  // namespace

void declare_config(ObservationVoteFilter::Config& config) {
  using namespace config;
  name("ObservationVoteFilter");
  field(config.enabled, "enabled");
  field(config.voxel_size, "voxel_size", "m");
  field(config.vote_fraction, "vote_fraction");
  field(config.min_votes, "min_votes");
  field(config.dbscan_eps, "dbscan_eps", "m");
  field(config.dbscan_min_points, "dbscan_min_points");
  field(config.min_inlier_ratio, "min_inlier_ratio");
  field(config.min_frames, "min_frames");
  check(config.voxel_size, GT, 0.f, "voxel_size");
  checkInRange(config.vote_fraction, 0.f, 1.f, "vote_fraction");
  checkInRange(config.min_inlier_ratio, 0.f, 1.f, "min_inlier_ratio");
}

ObservationVoteFilter::ObservationVoteFilter(const Config& config)
    : config(config::checkValid(config)) {}

size_t ObservationVoteFilter::filter(const FrameDataBuffer& frame_data, Track& track) const {
  if (!config.enabled) {
    return 0;
  }

  // Re-evaluate everything if this track was filtered before.
  if (!track.filtered_observations.empty()) {
    track.observations.insert(track.observations.end(),
                              track.filtered_observations.begin(),
                              track.filtered_observations.end());
    track.filtered_observations.clear();
    std::sort(track.observations.begin(),
              track.observations.end(),
              [](const Observation& a, const Observation& b) { return a.stamp < b.stamp; });
  }

  // Vote: every evaluable frame votes once for each voxel its cluster touches.
  const spatial_hash::Grid<GlobalIndex> grid(config.voxel_size);
  std::vector<std::optional<std::vector<GlobalIndex>>> obs_voxels;
  obs_voxels.reserve(track.observations.size());
  GlobalIndexMap<int> votes;
  int num_frames = 0;
  for (const auto& obs : track.observations) {
    obs_voxels.push_back(observationVoxels(frame_data, obs, grid));
    if (!obs_voxels.back()) {
      continue;
    }
    ++num_frames;
    const GlobalIndexSet unique(obs_voxels.back()->begin(), obs_voxels.back()->end());
    for (const auto& voxel : unique) {
      ++votes[voxel];
    }
  }
  if (num_frames < config.min_frames) {
    return 0;
  }

  // Keep consistently observed voxels and take the largest spatial cluster as the object.
  const int min_votes = std::max(
      config.min_votes, static_cast<int>(std::ceil(config.vote_fraction * num_frames)));
  std::vector<GlobalIndex> candidates;
  Points centers;
  for (const auto& [voxel, count] : votes) {
    if (count >= min_votes) {
      candidates.push_back(voxel);
      centers.push_back(grid.toPoint(voxel));
    }
  }
  const auto inliers =
      utils::largestDbscanCluster(centers, config.dbscan_eps, config.dbscan_min_points);
  if (inliers.empty()) {
    return 0;
  }
  GlobalIndexSet object;
  for (const auto idx : inliers) {
    object.insert(candidates[idx]);
  }

  // Split observations by how much of each frame's cluster lies on the object.
  Observations kept, filtered;
  size_t kept_evaluated = 0;
  for (size_t i = 0; i < track.observations.size(); ++i) {
    const auto& voxels = obs_voxels[i];
    if (!voxels) {
      kept.push_back(track.observations[i]);
      continue;
    }
    const auto num_inside = std::count_if(
        voxels->begin(), voxels->end(), [&object](const auto& v) { return object.count(v); });
    if (static_cast<float>(num_inside) / voxels->size() >= config.min_inlier_ratio) {
      kept.push_back(track.observations[i]);
      ++kept_evaluated;
    } else {
      filtered.push_back(track.observations[i]);
    }
  }
  if (kept_evaluated == 0) {
    return 0;
  }

  track.observations = std::move(kept);
  track.filtered_observations = std::move(filtered);
  return track.filtered_observations.size();
}

}  // namespace khronos
