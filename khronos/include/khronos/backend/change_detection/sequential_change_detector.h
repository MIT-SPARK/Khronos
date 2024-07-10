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

#include <config_utilities/config_utilities.h>

#include "khronos/backend/change_detection/background/background_change_detector.h"
#include "khronos/backend/change_detection/objects/object_change_detector.h"
#include "khronos/backend/change_detection/ray_change_detector.h"
#include "khronos/backend/change_detection/ray_verificator.h"
#include "khronos/backend/change_state.h"
#include "khronos/common/common_types.h"

namespace khronos {

/**
 * @brief The sequential change detector takes a spatially optimized DSG as input, and then performs
 * change detection on it (thus sequential).
 */
class SequentialChangeDetector {
 public:
  // Config.
  struct Config {
    // Config for the object change detector to use. Leave empty to disable object change detection.
    config::VirtualConfig<ObjectChangeDetector> objects;

    // Config for the background change detector to use. Leave empty to disable background change
    // detection.
    config::VirtualConfig<BackgroundChangeDetector> background;

    // TODO(lschmid): The ray verificator might not always be needed but add here since it's shared.
    RayVerificator::Config ray_verificator;
    RayChangeDetector::Config ray_change_detector;
  } const config;

  // Construction.
  explicit SequentialChangeDetector(const Config& config);
  virtual ~SequentialChangeDetector() = default;

  // Perform change detection.
  /**
   * @brief Detects all changes and associations in the currently set DSG for it's current spatial
   * configuration. Will update the change detection state incrementally as needed.
   * @param rpgo_merges All proposed merges of objects from rpgo.
   * @param stamp The current time stamp.
   * @param had_loopclosure Whether the DSG had a loop closure since the last change detection. This
   * will trigger re-indexing of the spatial hash for ray verification and re-computation of all
   * incremental changes.
   * @return The current best estimated configuration of the DSG after change detection.
   */
  virtual const Changes& detectChanges(const RPGOMerges& rpgo_merges,
                                       TimeStamp stamp,
                                       bool had_loopclosure = false);

  /**
   * @brief Sets the DSG to use for change detection. The DSG can incrementally grow and deform.
   * @param dsg The DSG to use.
   */
  virtual void setDsg(std::shared_ptr<const DynamicSceneGraph> dsg);

  // Accessors.
  const Changes& getChanges() const { return changes_; }

 private:
  // Change detectors.
  RayVerificator::Ptr ray_verificator_;
  RayChangeDetector::Ptr ray_change_detector_;
  std::unique_ptr<ObjectChangeDetector> object_change_detector_;
  std::unique_ptr<BackgroundChangeDetector> background_change_detector_;

  // Variables.
  // Current DSG.
  std::shared_ptr<const DynamicSceneGraph> dsg_;

  // Current change detection state for icnremental updates.
  Changes changes_;
};

void declare_config(SequentialChangeDetector::Config& config);

}  // namespace khronos
