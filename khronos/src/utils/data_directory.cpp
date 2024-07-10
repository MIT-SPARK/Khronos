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

#include "khronos/utils/data_directory.h"

#include <filesystem>

#include <glog/logging.h>

namespace khronos {

DataDirectory::DataDirectory(const std::string& data_path, bool allocate, bool overwrite) {
  if (data_path.empty()) {
    return;
  }

  auto path = std::filesystem::path(data_path);

  // Setup the directory.
  if (std::filesystem::exists(path)) {
    if (overwrite) {
      // Clear the existing directory to write new output.
      LOG(WARNING) << "[DataDirectory] Overwriting existing output directory: '" << path.string()
                   << "'.";
      std::filesystem::remove_all(path);
      path_ = path;
      std::filesystem::create_directories(path);
    } else if (allocate) {
      // Create a time stamped sub-directory.
      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      std::stringstream timestamp;
      timestamp << std::put_time(&tm, "%Y_%m_%d-%H_%M_%S");
      path_ = path / timestamp.str();
      std::filesystem::create_directories(path_);
    }
  } else if (allocate) {
    // Directory does not exist and should be allocated.
    path_ = path;
    std::filesystem::create_directories(path_);
  }
}

}  // namespace khronos
