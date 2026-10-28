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

#include <string>
#include <utility>
#include <vector>

#include <hydra/utils/image_store.h>

#include "khronos/active_window/data/frame_data_buffer.h"
#include "khronos/active_window/data/track.h"

namespace khronos {

/**
 * @brief Records the observations of extracted objects in the image store of the pipeline
 * (see hydra::GlobalInfo::imageStore; nothing is recorded if no store is configured). Shared
 * by the object extractors.
 */
class ObjectImageSaver {
 public:
  // Score that ranks the observations of an object: the volume of its 3D bounding box or
  // its number of segmented pixels. `all` scores by pixels and keeps every image.
  enum class Selection { kLargestBBox, kSegmentSize, kAll };

  /**
   * @param selection How to rank the observations; the best max_images_per_object of the store
   * keep their images.
   * @param owner Name used in log messages.
   */
  ObjectImageSaver(Selection selection, const std::string& owner);

  bool enabled() const { return image_store_ != nullptr; }

  /**
   * @brief Record every observation of the track that has a semantic measurement in the frame
   * data. Never throws (extraction runs on worker threads).
   */
  void save(const Track& track, const FrameDataBuffer& frame_data) const;

  /**
   * @brief Record the given observations (frame and semantic cluster id) of the track.
   */
  void save(const Track& track, const std::vector<std::pair<FrameData::Ptr, int>>& frames) const;

  /**
   * @brief Collect all frames that have a semantic measurement for the given track.
   * @returns A list of frames and the corresponding semantic cluster ID.
   */
  static std::vector<std::pair<FrameData::Ptr, int>> collectSemanticFrames(
      const Track& track,
      const FrameDataBuffer& frame_data);

 private:
  void saveImpl(const Track& track,
                const std::vector<std::pair<FrameData::Ptr, int>>& frames) const;

  const Selection selection_;
  const std::string owner_;
  hydra::ImageStore::Ptr image_store_;
};

}  // namespace khronos
