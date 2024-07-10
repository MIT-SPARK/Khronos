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

#include "khronos/active_window/integration/tracking_integrator.h"

#include <future>
#include <memory>
#include <thread>
#include <vector>

#include <hydra/reconstruction/index_getter.h>
#include <spatial_hash/neighbor_utils.h>

namespace khronos {

void declare_config(TrackingIntegrator::Config& config) {
  using namespace config;
  name("TrackingIntegrator");
  field(config.verbosity, "verbosity");
  field(config.temporal_buffer, "temporal_buffer", "s");
  field(config.burn_in_period, "burn_in_period", "s");
  field(config.tsdf_occupancy_threshold, "tsdf_occupancy_threshold", "m");
  field(config.neighbor_connectivity, "neighbor_connectivity");
  field(config.temporal_window, "temporal_window", "s");
  field<ThreadNumConversion>(config.num_threads, "num_threads");

  checkIsOneOf(config.neighbor_connectivity, {6, 18, 26}, "neighbor_connectivity");
  check(config.num_threads, GE, 1, "num_threads");
  check(config.temporal_buffer, GT, 0, "temporal_buffer");
  check(config.tsdf_occupancy_threshold, NE, 0, "tsdf_occupancy_threshold");
  check(config.temporal_window, GT, 0, "temporal_window");
}

TrackingIntegrator::TrackingIntegrator(const TrackingIntegrator::Config& config)
    : config(config::checkValid(config)) {}

void TrackingIntegrator::updateBlocks(const FrameData& data, VolumetricMap& map) const {
  Timer timer("integration/tracking", data.input.timestamp_ns);

  // Get all and the updated blocks.
  const BlockIndices all_blocks = map.getTsdfLayer().allocatedBlockIndices();
  const BlockIndices updated_blocks =
      map.getTsdfLayer().blockIndicesWithCondition(TsdfBlock::trackingUpdated);
  CLOG(4) << "[Tracking Integrator] Updating " << all_blocks.size() << " total blocks and "
          << updated_blocks.size() << " ever-free blocks.";

  // Update tracking information and occupancy counter in parallel by block.
  BlockIndexGetter all_indices(all_blocks);
  std::vector<std::thread> threads;
  for (int i = 0; i < config.num_threads; ++i) {
    threads.emplace_back(&TrackingIntegrator::updateBlockTracking, this, &data, &all_indices, &map);
  }
  for (auto& thread : threads) {
    thread.join();
  }

  // Labels tsdf-updated voxels as ever-free if they satisfy the criteria. Since
  // neighbors are looked up in parallel this needs to happen after the tracking update
  // is completed.
  BlockIndexGetter updated_indices(updated_blocks);
  threads.clear();
  for (int i = 0; i < config.num_threads; ++i) {
    threads.emplace_back(
        &TrackingIntegrator::updateBlockEverFree, this, &data, &updated_indices, &map);
  }
  for (auto& thread : threads) {
    thread.join();
  }
}

void TrackingIntegrator::resetInactive(VolumetricMap& map) const {
  for (const auto& index : map.getTsdfLayer().allocatedBlockIndices()) {
    TrackingBlock::Ptr tracking_block = map.getTrackingLayer()->getBlockPtr(index);
    if (!tracking_block) {
      continue;
    }

    bool remove_block = true;
    for (size_t linear_index = 0; linear_index < tracking_block->numVoxels(); ++linear_index) {
      TrackingVoxel& tracking_voxel = tracking_block->getVoxel(linear_index);
      if (!tracking_voxel.to_remove) {
        remove_block = false;
        break;
      }
    }

    if (remove_block) {
      map.removeBlock(index);
    }
  }
}

void TrackingIntegrator::updateBlockTracking(const FrameData* data,
                                             BlockIndexGetter* index_getter,
                                             VolumetricMap* map) const {
  const float tsdf_threshold = config.tsdf_occupancy_threshold < 0
                                   ? config.tsdf_occupancy_threshold * -map->config.voxel_size
                                   : config.tsdf_occupancy_threshold;
  BlockIndex block_index;
  while (index_getter->getNextIndex(block_index)) {
    // Unset the update flag.
    TsdfBlock::Ptr tsdf_block = map->getTsdfLayer().getBlockPtr(block_index);
    if (!tsdf_block) {
      continue;
    }
    tsdf_block->tracking_updated = false;
    TrackingBlock::Ptr tracking_block = map->getTrackingLayer()->getBlockPtr(block_index);
    if (!tracking_block) {
      continue;
    }

    // Update all voxels.
    bool contains_active_data = false;
    for (size_t linear_index = 0; linear_index < tracking_block->numVoxels(); ++linear_index) {
      TsdfVoxel& tsdf_voxel = tsdf_block->getVoxel(linear_index);
      TrackingVoxel& tracking_voxel = tracking_block->getVoxel(linear_index);
      updateTrackingDuration(tsdf_voxel, tracking_voxel, data->input.timestamp_ns, tsdf_threshold);
      if (tracking_voxel.active) {
        contains_active_data = true;
      }
    }

    // Update the block state.
    tracking_block->has_active_data = contains_active_data;
  }
}

void TrackingIntegrator::updateBlockEverFree(const FrameData* data,
                                             BlockIndexGetter* index_getter,
                                             VolumetricMap* map) const {
  auto& tracking_layer = *map->getTrackingLayer();
  const spatial_hash::VoxelNeighborSearch search(tracking_layer, config.neighbor_connectivity);
  BlockIndex block_index;
  while (index_getter->getNextIndex(block_index)) {
    TrackingBlock::Ptr block = tracking_layer.getBlockPtr(block_index);
    if (!block) {
      continue;
    }

    // Check all voxels.
    for (size_t linear_index = 0; linear_index < block->numVoxels(); ++linear_index) {
      TrackingVoxel& voxel = block->getVoxel(linear_index);

      // If already ever-free we can save the cost of checking the neighbourhood.
      if (voxel.ever_free || !voxelIsFree(voxel, data->input.timestamp_ns)) {
        continue;
      }

      // Check the neighbourhood for unobserved or occupied voxels.
      bool neighbor_occupied_or_unobserved = false;
      for (const auto& neighbor_key : search.neighborKeys(block->getVoxelKey(linear_index))) {
        const TrackingBlock* neighbor_block;
        if (neighbor_key.first == block_index) {
          // Often will be the same block.
          neighbor_block = block.get();
        } else {
          neighbor_block = tracking_layer.getBlockPtr(neighbor_key.first).get();
          if (neighbor_block == nullptr) {
            // Block does not exist.
            neighbor_occupied_or_unobserved = true;
            break;
          }
        }

        // Check the voxel if it is unobserved or static.
        const TrackingVoxel& neighbor_voxel = neighbor_block->getVoxel(neighbor_key.second);
        if (neighbor_voxel.ever_free) {
          continue;
        }
        if (!voxelIsFree(neighbor_voxel, data->input.timestamp_ns)) {
          neighbor_occupied_or_unobserved = true;
          break;
        }
      }

      // Only observed free space, can be labeled as ever-free.
      if (!neighbor_occupied_or_unobserved) {
        voxel.ever_free = true;
      }
    }
  }
}

bool TrackingIntegrator::updateTrackingDuration(TsdfVoxel& tsdf_voxel,
                                                TrackingVoxel& tracking_voxel,
                                                const TimeStamp& time_stamp,
                                                float tsdf_threshold) const {
  // TODO(lschmid): This integrator can probably be simplified.
  // Allow for breaks of temporal_buffer between occupied observations to
  // compensate for lidar sparsity.
  if (tsdf_voxel.distance < tsdf_threshold) {
    // The voxel is considered occupied
    tracking_voxel.last_occupied = time_stamp;
  }

  const bool was_active = tracking_voxel.active;
  tracking_voxel.active =
      toSeconds(tracking_voxel.last_observed) >= toSeconds(time_stamp) - config.temporal_window;

  // Deactiavted voxels are reset. But this messes with the meshing...
  if (was_active && !tracking_voxel.active) {
    tracking_voxel.to_remove = true;
    return true;
  }
  return false;
}

bool TrackingIntegrator::voxelIsFree(const TrackingVoxel& voxel, const TimeStamp& stamp) const {
  // Only observed voxels that were unoccupied for the burn_in_period are free.
  return toSeconds(voxel.last_occupied) < toSeconds(stamp) - config.temporal_buffer &&
         voxel.last_observed != 0u;
}

}  // namespace khronos
