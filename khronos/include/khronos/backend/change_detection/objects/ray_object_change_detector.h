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

#include <vector>

#include <config_utilities/config_utilities.h>

#include "khronos/backend/change_detection/objects/object_change_detector.h"
#include "khronos/backend/change_detection/ray_change_detector.h"
#include "khronos/backend/change_detection/ray_verificator.h"

namespace khronos {

/**
 * @brief Interface class for object-level change detection. The sequential change detector takes a
 * spatially optimized DSG as input, and then performs change detection on it (thus sequential).
 */
class RayObjectChangeDetector : public ObjectChangeDetector {
 public:
  // Config.
  struct Config {
    // Only check for changes in meshes that have been observed at least this far apart [s].
    float time_filtering_threshold = 5.f;

    // Subsample the vertices of objects during change detection. This saves computation for large
    // or high resolution meshes.
    int query_subsampling = 100;
  } const config;

  // Construction.
  RayObjectChangeDetector(const Config& confg,
                          RayVerificator::ConstPtr ray_verificator,
                          RayChangeDetector::ConstPtr ray_change_detector);

  // Implement interfaces.
  void detectChanges(const DynamicSceneGraph& dsg,
                     const RPGOMerges& rpgo_merges,
                     ObjectChanges& changes) override;

 protected:
  // Helper functions.
  void checkObjectMerge(const RPGOMerges& rpgo_merges, ObjectChange& change) const;
  void checkObjectObservation(KhronosObjectAttributes& attrs, ObjectChange& change) const;

 private:
  // Members.
  const RayVerificator::ConstPtr ray_verificator_;
  const RayChangeDetector::ConstPtr ray_change_detector_;

  // cached values.
  const uint64_t time_filtering_threshold_ns_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<ObjectChangeDetector,
                                     RayObjectChangeDetector,
                                     RayObjectChangeDetector::Config,
                                     RayVerificator::Ptr,
                                     RayChangeDetector::Ptr>("RayObjectChangeDetector");
};

void declare_config(RayObjectChangeDetector::Config& config);

}  // namespace khronos
