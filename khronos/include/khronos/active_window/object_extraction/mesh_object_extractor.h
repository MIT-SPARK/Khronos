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

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/common/global_info.h>
#include <hydra/reconstruction/mesh_integrator.h>

#include "khronos/active_window/integration/projective_integrator.h"
#include "khronos/active_window/object_extraction/object_extractor.h"

namespace khronos {

/**
 * @brief An object extractor that performs mesh-based 3D reconstruction of objects to extract.
 */
class MeshObjectExtractor : public ObjectExtractor {
 public:
  struct Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // Extract only objects with an existence confidence larger than this value [0, 1].
    float min_object_allocation_confidence = 0.5f;

    // Extract only objects with a volume larger than this value in m3.
    float min_object_volume = 0.1f;

    // Extract only objects with a volume smaller than this value in m3.
    float max_object_volume = 4.0f;

    // Only extract objects if the reconstruction is not empty.
    bool only_extract_reconstructed_objects = false;

    // Dynamic objects are only valid if trajectory is longer than specified length.
    float min_dynamic_displacement = 0.2f;

    // Only add vertices with a confidence larger than this to the object
    // reconstruction.
    float min_object_reconstruction_confidence = 0.5f;

    // Only add vertices with more observations than this to the object reconstruction.
    int min_object_reconstruction_observations = 10;

    // Resolution at which extracted objects are reconstructed. Positive values
    // indicate the voxel size in meters. Negative values indicate a fraction of
    // the extent. A value of 0 skips reconstruction.
    float object_reconstruction_resolution = -0.02;

    // If true, don't prune the mesh to only the relevant parts but isntead color it by confidence
    // for debugging.
    bool visualize_classification = false;

    hydra::ProjectiveIntegratorConfig projective_integrator;
    hydra::MeshIntegratorConfig mesh_integrator;
  } const config;

  // Construction.
  explicit MeshObjectExtractor(const Config& config);
  virtual ~MeshObjectExtractor() = default;

  KhronosObjectAttributes::Ptr extractObject(const Track& track,
                                             const FrameDataBuffer& frame_data) override;

  /**
   * @brief Extract a dynamic object from the given track.
   * @param track The track representing the object.
   * @param frame_data The frame data buffer to extract the object from.
   * @returns The extracted object attributes if valid, nullptr otherwise.
   */
  KhronosObjectAttributes::Ptr extractDynamicObject(const Track& track,
                                                    const FrameDataBuffer& frame_data) const;

  /**
   * @brief Extract a static object from the given track.
   * @param track The track representing the object.
   * @param frame_data The frame data buffer to extract the object from.
   * @returns The extracted object attributes if valid, nullptr otherwise.
   */
  KhronosObjectAttributes::Ptr extractStaticObject(const Track& track,
                                                   const FrameDataBuffer& frame_datam) const;
  /**
   * @brief Check if a track meets the minimum criteria for object extraction.
   */
  bool trackIsValid(const Track& track) const;

  /**
   * @brief Get a human readable name for the given track for printing.
   */
  static std::string getTrackName(const Track& track);

  /**
   * @brief Collect all frames that have a semantic measurement for the given track.
   * @param track The track to collect frames for.
   * @param frame_data The frame data buffer to collect frames from.
   * @returns A list of frames and the corresponding semantic cluster ID.
   */
  static std::vector<std::pair<FrameData::Ptr, int>> collectSemanticFrames(
      const Track& track,
      const FrameDataBuffer& frame_data);

  /**
   * @brief Compute tje maximal spatial extent covered by all frames.
   * @param frames  A list of frames and the corresponding semantic cluster ID to get the extent
   * for.
   */
  BoundingBox computeExtent(const std::vector<std::pair<FrameData::Ptr, int>>& frames) const;

  /**
   * @brief Compute the confidence of a voxel belonging to the object of interest.
   */
  virtual float computeConfidence(const TsdfVoxel& tsdf_voxel,
                                  const hydra::SemanticVoxel& confidence_voxel) const;

 private:
  ProjectiveIntegrator integrator_;
  hydra::MeshIntegrator mesh_integrator_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<ObjectExtractor,
                                     MeshObjectExtractor,
                                     MeshObjectExtractor::Config>("MeshObjectExtractor");
};

void declare_config(MeshObjectExtractor::Config& config);

}  // namespace khronos
