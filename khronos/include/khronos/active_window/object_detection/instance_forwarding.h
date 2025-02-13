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

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <config_utilities/virtual_config.h>
#include <hydra/common/global_info.h>
#include <hydra/openset/embedding_distances.h>
#include <hydra/openset/embedding_group.h>
#include <hydra/reconstruction/volumetric_map.h>
#include <spatial_hash/neighbor_utils.h>

#include "khronos/active_window/data/frame_data.h"
#include "khronos/active_window/object_detection/object_detector.h"
#include "khronos/common/common_types.h"

namespace khronos {

/**
 * @brief Proxy object detector that forwards already detected object instances.
 */
class InstanceForwarding : public ObjectDetector {
 public:
  // Config.
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;
    // Maximum depth values to consider for object extraction in meters. Use 0 for infinity.
    float max_range = 0.f;

    // Discard clusters with fewer pixels than this.
    int min_cluster_size = 0;

    // Discard clusters with more pixels than this (<= 0 disables).
    int max_cluster_size = -1;

    // Discard clusters with less volume than this
    double min_object_volume = 0.0;

    // Discard clusters with more volume than this (if enabled)
    double max_object_volume = -1.0;

    // Discard clusters that is overly similary to background
    double max_background_score = 0.2;

    // Background is specified by the following embedding group (prompts)
    config::VirtualConfig<hydra::EmbeddingGroup> background;
    config::VirtualConfig<hydra::EmbeddingDistance> metric{hydra::CosineDistance::Config()};
  } const config;

  // Construction.
  explicit InstanceForwarding(const Config& config);
  virtual ~InstanceForwarding() = default;

  // Inputs.

  /**
   * @brief Performs semantic object extraction by forwarding instances in the object image input
   * @param map Volumetric map used for voxel-based clustering.
   * @param data Frame data to read input images and write results to.
   */
  void processInput(const VolumetricMap& /* map */, FrameData& data) override;

  // Processing.

  /**
   * @brief Performs semantic object extraction by forwarding instances in the object image input
   * @param data Frame data to read input images and write results to.
   */
  void extractSemanticClusters(FrameData& data);

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<ObjectDetector,
                                     InstanceForwarding,
                                     InstanceForwarding::Config>("InstanceForwarding");

  const bool filter_by_volume_;

  // Filter out background.
  hydra::EmbeddingGroup::Ptr background_;
  std::unique_ptr<hydra::EmbeddingDistance> metric_;

  TimeStamp processing_stamp_;
};

void declare_config(InstanceForwarding::Config& config);

}  // namespace khronos
