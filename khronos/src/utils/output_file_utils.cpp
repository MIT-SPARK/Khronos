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

#include "khronos/utils/output_file_utils.h"

#include <filesystem>

#include <glog/logging.h>

namespace khronos {

bool ensureDirectoryExists(const std::string& path) {
  if (!std::filesystem::exists(path)) {
    if (!std::filesystem::create_directories(path)) {
      LOG(ERROR) << "Failed to create directory '" << path << "'.";
      return false;
    }
  }
  return true;
}

bool openOrAppendToCsv(const std::string& file_path,
                       std::fstream& file_stream,
                       const std::string& header,
                       bool overwrite) {
  const bool file_existed = std::filesystem::exists(file_path);
  file_stream = std::fstream(file_path, std::ios::in | std::ios::app);
  if (!file_stream) {
    LOG(ERROR) << "Failed to open file '" << file_path << "'.";
    return false;
  }
  if (file_existed) {
    std::string previous_header;
    file_stream.seekg(std::ios_base::beg);
    getline(file_stream, previous_header);
    file_stream.seekp(std::ios_base::end);
    file_stream.clear();
    if (previous_header.find(header) != 0) {
      if (overwrite) {
        file_stream.close();
        file_stream.open(file_path, std::ios::out | std::ios::trunc);
        if (!file_stream) {
          LOG(ERROR) << "Failed to open file '" << file_path << "'.";
          return false;
        }
      } else {
        LOG(ERROR) << "Output file '" << file_path
                   << "' already exists but has different header, skipping.";
        return false;
      }
    }
    LOG(INFO) << "Appending to existing output file '" << file_path << "'.";
  } else {
    file_stream << header;
  }
  return true;
}

}  // namespace khronos
