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

#include <config_utilities/types/path.h>
#include "khronos/active_window/change_detection/active_window_change_detector.h"

namespace khronos {
namespace {

static const auto registration =
    config::RegistrationWithConfig<ActiveWindow::KhronosSink,
                                   ActiveWindowChangeDetector,
                                   ActiveWindowChangeDetector::Config>("ActiveWindowChangeDetector");
}

void declare_config(ActiveWindowChangeDetector::Config& config) {
  using namespace config;
  name("ActiveWindowChangeDetector");
  field(config.verbosity, "verbosity");
  field<Path::Absolute>(config.prior_map_path, "prior_map_path");
  check<Path::Exists>(config.prior_map_path, "prior_map_path");
//   check<Path::Extension>(config.prior_map_path, "prior_map_path", ".spark_dsg"); // Add the check back sometimes. 
}

ActiveWindowChangeDetector::ActiveWindowChangeDetector(const Config& config)
    : config(config::checkValid(config)){

}

void ActiveWindowChangeDetector::call(const FrameData& data,
                                      const VolumetricMap& map,
                                      const Tracks& tracks) const {
}

void ActiveWindowChangeDetector::loadPriorMap(/* params */) { 
    
}

}  // namespace khronos
