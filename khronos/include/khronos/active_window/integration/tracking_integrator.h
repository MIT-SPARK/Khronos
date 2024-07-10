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

#include <memory>
#include <thread>

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>
#include <hydra/reconstruction/index_getter.h>
#include <spatial_hash/neighbor_utils.h>

#include "khronos/active_window/data/frame_data.h"
#include "khronos/active_window/data/reconstruction_types.h"
#include "khronos/common/common_types.h"

namespace khronos {

class TrackingIntegrator {
 public:
  using BlockIndexGetter = hydra::IndexGetter<BlockIndex>;

  // Config.
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // Duration in seconds a voxel can be not observed to still count as occupied for
    // motion detection to compensate sensor sparsity or noise. Note that this value
    // must be larger than the frame rate to allow continuous tracking.
    float temporal_buffer = 1.f;

    // Consecutive duration in seconds a voxel must be free to become ever-free.
    float burn_in_period = 1.f;

    // SDF distance below which a voxel is considered occupied [m]. Negative values
    // indicate multiples of the voxel size.
    float tsdf_occupancy_threshold = -1.5;

    // Number of neighbors to consider for the spatial robustness check {6, 18, 26}.
    int neighbor_connectivity = 18;

    // Duration in seconds a voxel can be out of view before loosing the 'active' status
    // and exiting the active window.
    float temporal_window = 3.f;

    // Number of threads to use.
    int num_threads = hydra::GlobalInfo::instance().getConfig().default_num_threads;
  } const config;

  // Construction.
  explicit TrackingIntegrator(const Config& config);
  virtual ~TrackingIntegrator() = default;

  /**
   * @brief Update the tracking information and ever-free state of all blocks whose TSDF
   * was updated in the last frame in parallel.
   * @param data Input data of the frame.
   * @param map Map to search for blocks and update them.
   */
  void updateBlocks(const FrameData& data, VolumetricMap& map) const;

  /**
   * @brief Based on if tracking is active or in-active, reset TSDF and tracking voxel
   * @param map Map to search for blocks and update them.
   */
  void resetInactive(VolumetricMap& map) const;

  /**
   * @brief Update the tracking information of the specified block single-threaded.
   * @param data Input data of the frame.
   * @param index_getter Index getter for blocks to update.
   * @param map Map to update the bloxck in.
   * @return True if the block still contains active voxels. False otherwise.
   */
  void updateBlockTracking(const FrameData* data,
                           BlockIndexGetter* index_getter,
                           VolumetricMap* map) const;

  /**
   * @brief Update the ever-free state of the specified block single-threaded.
   * @param data Input data of the frame.
   * @param index_getter Index getter for blocks to update.
   * @param map Map to update the bloxck in.
   */
  void updateBlockEverFree(const FrameData* data,
                           BlockIndexGetter* index_getter,
                           VolumetricMap* map) const;

  /**
   * @brief Track whether the TSDF voxel was observed and is considered occupied.
   * @param tsdf_voxel The corresponding TSDF voxel to perform occupancy and observation
   * checks.
   * @param tracking_voxel The tracking voxel to update.
   * @param time_stamp Time stamp of the current measurement.
   * @param tsdf_threshold Threshold to consider a TSDF voxel occupied in meters.
   * @return True if the voxel has changed, false otherwise.
   */
  bool updateTrackingDuration(TsdfVoxel& tsdf_voxel,
                              TrackingVoxel& tracking_voxel,
                              const TimeStamp& time_stamp,
                              float tsdf_threshold) const;

  /**
   * @brief Check whether an individual tracking voxel meets the requirements to be
   * considered observed and free.
   * @param voxel The voxel to check.
   * @return True if the voxel is free, false otherwise.
   */
  bool voxelIsFree(const TrackingVoxel& voxel, const TimeStamp& stamp) const;
};

void declare_config(TrackingIntegrator::Config& config);

}  // namespace khronos
