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

#include "khronos/active_window/data/measurement_clusters.h"

namespace khronos {

/**
 * @brief Result of comparing two SemanticClusterInfo for a match. Explains why a comparison
 * failed (or succeeded) for logging/debugging.
 */
struct SemanticMatchResult {
  inline operator bool() const { return status == Status::kMatch; }

  enum class Status {
    kNoSemantics,
    kMismatchedCategories,
    kMismatchedFeatures,
    kLowSimiliarity,
    kMatch,
  } const status;

  const std::optional<float> similiarity = std::nullopt;
  const std::optional<int> lhs_category = std::nullopt;
  const std::optional<int> rhs_category = std::nullopt;
};

// NOTE: intentionally a named function (not `operator<<`). A free `operator<<` declared in
// namespace khronos would hide (via ordinary unqualified lookup, which stops at the first
// enclosing namespace containing any `operator<<` declaration) the global config-printing
// template `operator<<(ostream&, const ConfigT&)` for any unqualified `os << some_config` call
// elsewhere in namespace khronos in a TU that includes this header — even though the two are
// unrelated types. `toString` sidesteps that lookup pitfall entirely.
std::string toString(const SemanticMatchResult& result);

/**
 * @brief Compare two optional SemanticClusterInfo for a semantic match: both present, same
 * category_id, and (if openset features are populated) cosine similarity >= min_cosine_sim.
 * @param lhs First semantic info to compare (may be std::nullopt).
 * @param rhs Second semantic info to compare (may be std::nullopt).
 * @param min_cosine_sim Minimum feature cosine similarity to accept a match when both sides
 * carry openset features.
 */
SemanticMatchResult semanticsMatch(const std::optional<SemanticClusterInfo>& lhs,
                                   const std::optional<SemanticClusterInfo>& rhs,
                                   float min_cosine_sim);

}  // namespace khronos
