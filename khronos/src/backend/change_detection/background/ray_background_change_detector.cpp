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

#include "khronos/backend/change_detection/background/ray_background_change_detector.h"

namespace khronos {

void declare_config(RayBackgroundChangeDetector::Config& config) {
  using namespace config;
  name("RayBackgroundChangeDetector");
  field(config.time_filtering_threshold, "time_filtering_threshold", "s");

  check(config.time_filtering_threshold, GT, 0.f, "time_filtering_threshold");
}

RayBackgroundChangeDetector::RayBackgroundChangeDetector(
    const Config& config,
    RayVerificator::ConstPtr ray_verificator,
    RayChangeDetector::ConstPtr ray_change_detector)
    : config(config::checkValid(config)),
      ray_verificator_(std::move(ray_verificator)),
      ray_change_detector_(std::move(ray_change_detector)),
      time_filtering_threshold_ns_(config.time_filtering_threshold * 1e9) {}

void RayBackgroundChangeDetector::detectChanges(const DynamicSceneGraph& dsg,
                                                BackgroundChanges& changes) {
  // Simply check all vertices using the ray verificator.
  const auto& mesh = *dsg.mesh();
  changes.reserve(mesh.numVertices());

  // Compute changes for new vertices.
  for (size_t i = changes.size(); i < mesh.numVertices(); ++i) {
    changes.emplace_back(checkVertex(mesh, i));
  }

  // Re-compute changes for re-observed vertices.
  size_t num_updates = 0;
  for (const size_t i : ray_verificator_->getReobservedVertices()) {
    if (i >= mesh.numVertices()) {
      continue;
    }
    const ChangeState change = checkVertex(mesh, i);
    if (change != changes.at(i)) {
      ++num_updates;
    }
    changes.at(i) = change;
  }

  if (!ray_verificator_->getReobservedVertices().empty()) {
    CLOG(3) << "[RayBackgroundChangeDetector] Recomputed "
            << ray_verificator_->getReobservedVertices().size() << " vertices, finding "
            << num_updates << " changed entries.";
  }
}

ChangeState RayBackgroundChangeDetector::checkVertex(const spark_dsg::Mesh& mesh,
                                                     size_t index) const {
  const RayVerificator::CheckResult check_result = ray_verificator_->check(
      mesh.pos(index), mesh.timestamp(index) + time_filtering_threshold_ns_);
  const RayChangeDetector::ChangeResult change_result =
      ray_change_detector_->detectChanges(check_result, true);
  if (change_result.closest_absent) {
    return ChangeState::kAbsent;
  } else if (change_result.furthest_persistent) {
    return ChangeState::kPersistent;
  } else {
    return ChangeState::kUnobserved;
  }
}

}  // namespace khronos
