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

#include "khronos/active_window/tracking/semantic_matching.h"

#include <limits>
#include <sstream>

#include <hydra/openset/embedding_distances.h>

namespace khronos {
namespace {

float computeCosineSim(const FeatureVector& lhs, const FeatureVector& rhs) {
  static const auto metric = hydra::CosineDistance();
  return metric.score(lhs, rhs);
}

}  // namespace

std::string toString(const SemanticMatchResult& result) {
  std::ostringstream out;
  switch (result.status) {
    case SemanticMatchResult::Status::kNoSemantics:
      out << "no match (invalid semantics)";
      break;
    case SemanticMatchResult::Status::kMismatchedCategories:
      out << "no match (categories are different: " << result.lhs_category.value_or(-1) << " vs "
          << result.rhs_category.value_or(-1) << ")";
      break;
    case SemanticMatchResult::Status::kMismatchedFeatures:
      out << "no match (feature dimensions disagree)";
      break;
    case SemanticMatchResult::Status::kLowSimiliarity:
      out << "no match (low similiarity: "
          << result.similiarity.value_or(std::numeric_limits<float>::quiet_NaN()) << ")";
      break;
    case SemanticMatchResult::Status::kMatch:
      out << "match!";
      break;
  }

  return out.str();
}

SemanticMatchResult semanticsMatch(const std::optional<SemanticClusterInfo>& lhs,
                                   const std::optional<SemanticClusterInfo>& rhs,
                                   float min_cosine_sim) {
  if (lhs.has_value() != rhs.has_value()) {
    return {SemanticMatchResult::Status::kNoSemantics};
  }

  if (!lhs && !rhs) {
    return {SemanticMatchResult::Status::kMatch};
  }

  // For openset cases, all objects have the same (unknown) semantic ID.
  if (lhs->category_id != rhs->category_id) {
    return {SemanticMatchResult::Status::kMismatchedCategories,
           std::nullopt,
           lhs->category_id,
           rhs->category_id};
  }

  if (lhs->feature.size() != rhs->feature.size()) {
    return {SemanticMatchResult::Status::kMismatchedFeatures};
  }

  if (lhs->feature.size() <= 0) {
    return {SemanticMatchResult::Status::kMatch};  // no openset features
  }

  const auto cosine_sim = computeCosineSim(lhs->feature, rhs->feature);
  if (cosine_sim < min_cosine_sim) {
    return {SemanticMatchResult::Status::kLowSimiliarity, cosine_sim};
  }

  return {SemanticMatchResult::Status::kMatch, cosine_sim};
}

}  // namespace khronos
