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

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>
#include <hydra/utils/logging.h>

#include "khronos/active_window/object_extraction/object_extractor.h"
#include "khronos/active_window/object_extraction/object_image_saver.h"

namespace khronos {

/**
 * @brief Extracts objects directly from the geometry the tracker accumulated (requires the tracker
 * to run with accumulate_voxels). No reconstruction: the confirmed voxel centers become the
 * object's mesh vertices (points, no faces) and the accumulated bounding box its bounding box.
 */
class AccumulatedObjectExtractor : public ObjectExtractor {
 public:
  struct Config : hydra::VerbosityConfig {
    Config()
        : hydra::VerbosityConfig{hydra::GlobalInfo::instance().getConfig().default_verbosity} {}

    // Extract only objects with an existence confidence larger than this value [0, 1].
    float min_object_allocation_confidence = 0.5f;

    // Extract only static objects with a bounding box volume in this range [m3].
    float min_object_volume = 0.1f;
    float max_object_volume = 4.0f;

    // Score that ranks the observations of an object (see ObjectImageSaver). The
    // observations of extracted static objects are recorded in the image store of the pipeline if
    // one is configured (see hydra::GlobalInfo::imageStore). Every observation is recorded;
    // the images of the best max_images_per_object observations of the store are kept.
    // Hydra's graph updater records which node each track becomes.
    using ImageSelection = ObjectImageSaver::Selection;
    ImageSelection image_selection_criteria = ImageSelection::kLargestBBox;
  } const config;

  // Construction.
  explicit AccumulatedObjectExtractor(const Config& config);
  virtual ~AccumulatedObjectExtractor() = default;

  KhronosObjectAttributes::Ptr extractObject(const Track& track,
                                             const FrameDataBuffer& frame_data) override;

 private:
  KhronosObjectAttributes::Ptr extractStaticObject(const Track& track) const;
  KhronosObjectAttributes::Ptr extractDynamicObject(const Track& track) const;

  ObjectImageSaver image_saver_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<ObjectExtractor,
                                     AccumulatedObjectExtractor,
                                     AccumulatedObjectExtractor::Config>(
          "AccumulatedObjectExtractor");
};

void declare_config(AccumulatedObjectExtractor::Config& config);

}  // namespace khronos
