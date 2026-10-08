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

#include "khronos/active_window/data/frame_data_buffer.h"
#include "khronos/active_window/data/track.h"

namespace khronos {

/**
 * @brief Prunes a track's observations whose cluster points disagree with the object's
 * consistently observed location, before the track is handed to object extraction.
 *
 * Runs once, when a track turns inactive. All buffered frames of the track vote on
 * `voxel_size` voxels (one vote per frame per voxel); voxels hit by at least
 * max(min_votes, vote_fraction * #frames) frames are kept, and the largest DBSCAN cluster of
 * those is taken as the object. Observations whose points fall inside that cluster less than
 * `min_inlier_ratio` of the time (e.g. frames with biased far-range depth or leaking masks) are
 * moved to Track::filtered_observations, so extraction only uses the consistent frames.
 */
struct ObservationVoteFilter {
  struct Config {
    //! Whether to filter observations at all.
    bool enabled = false;
    //! Voxel size [m] used for voting.
    float voxel_size = 0.1f;
    //! Fraction of the track's evaluated frames that must hit a voxel to keep it.
    float vote_fraction = 0.15f;
    //! Minimum absolute number of frames that must hit a voxel to keep it.
    int min_votes = 3;
    //! DBSCAN neighborhood radius [m] over the kept voxel centers.
    float dbscan_eps = 0.15f;
    //! DBSCAN minimum number of neighbors over the kept voxel centers.
    int dbscan_min_points = 3;
    //! Minimum fraction of an observation's points inside the object for it to be kept.
    float min_inlier_ratio = 0.5f;
    //! Minimum number of evaluable frames before filtering is attempted.
    int min_frames = 5;
  } const config;

  explicit ObservationVoteFilter(const Config& config);

  /**
   * @brief Filter the track's observations in place. Previously filtered observations are
   * merged back first so repeated calls (e.g. a track re-activating) re-evaluate all frames.
   * Observations that cannot be evaluated (no buffered frame or cluster) are always kept. If the
   * vote yields no object or would reject every evaluated frame, the track is left unchanged.
   * @returns Number of observations moved to filtered_observations.
   */
  size_t filter(const FrameDataBuffer& frame_data, Track& track) const;
};

void declare_config(ObservationVoteFilter::Config& config);

}  // namespace khronos
