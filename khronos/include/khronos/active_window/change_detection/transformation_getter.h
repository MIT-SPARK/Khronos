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
#include <optional>

#include <Eigen/Geometry>
#include <config_utilities/config_utilities.h>

namespace khronos {

/**
 * @brief Abstract interface for obtaining the current_T_prior transform.
 *
 * Implementations may query TF, return identity, or use any other source.
 * Returns std::nullopt when no valid transform is available (e.g. TF lookup
 * failure, or change below threshold); the caller skips the update in that case.
 */
class TransformationGetter {
 public:
  using Ptr = std::unique_ptr<TransformationGetter>;
  virtual ~TransformationGetter() = default;

  /**
   * @brief Get the latest odom_T_prior (current_T_prior) transform.
   * @return The transform, or nullopt if unavailable / no significant change.
   */
  virtual std::optional<Eigen::Isometry3d> getTransformation() const = 0;
};

/**
 * @brief Always returns Eigen::Isometry3d::Identity() — useful when no prior
 * map relocalization is needed (prior and current frames are the same).
 */
class IdentityTransformationGetter : public TransformationGetter {
 public:
  struct Config {} const config;

  explicit IdentityTransformationGetter(const Config& config);

  /**
   * @brief Returns nullopt — the transform remains at its default (Identity).
   * Use this getter when no relocalization is needed (prior and current frames
   * are the same).
   */
  std::optional<Eigen::Isometry3d> getTransformation() const override;
};

void declare_config(IdentityTransformationGetter::Config& config);

}  // namespace khronos
