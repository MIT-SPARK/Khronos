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

#include "khronos_ros/experiments/experiment_directory.h"

#include <algorithm>
#include <filesystem>

#include <glog/logging.h>

#include "khronos_ros/experiments/experiment_logger.h"

namespace khronos {

bool isExperimentDirectory(const std::string& directory) {
  if (!std::filesystem::is_directory(directory)) {
    LOG(ERROR) << "Experiment directory '" << directory << "' does not exist.";
    return false;
  }
  if (!std::filesystem::exists(directory + "/experiment_log.txt")) {
    LOG(ERROR) << "Directory '" << directory
               << "' is not a khronos experiment (has no 'experiment_log.txt').";
    return false;
  }
  ExperimentLogger logger(directory);
  if (!logger.hasFlag("Experiment Finished Cleanly")) {
    LOG(WARNING) << "Experiment '" << directory << "' did not finish cleanly.";
  }
  return true;
}

std::vector<std::string> getMapDirectories(const std::string& directory) {
  std::vector<std::string> map_dirs;
  if (std::filesystem::is_directory(directory + "/map")) {
    map_dirs.emplace_back("map");
  }
  const std::string map_dir = directory + "/maps";
  if (std::filesystem::is_directory(map_dir)) {
    for (const auto& entry : std::filesystem::directory_iterator(map_dir)) {
      if (entry.is_directory()) {
        map_dirs.emplace_back("maps/" + entry.path().filename().string());
      }
    }
  }
  return map_dirs;
}

std::vector<std::string> getMapNames(const std::string& directory) {
  std::vector<std::string> map_names;
  const std::string map_dir = directory + "/maps";
  if (std::filesystem::is_directory(map_dir)) {
    for (const auto& entry : std::filesystem::directory_iterator(map_dir)) {
      if (entry.is_directory()) {
        map_names.emplace_back(entry.path().filename().string());
      }
    }
  }
  std::sort(map_names.begin(), map_names.end());
  return map_names;
}

std::vector<uint64_t> getMapTimes(const std::string& directory,
                                  const std::vector<std::string>& map_names) {
  std::vector<uint64_t> result;
  result.reserve(map_names.size());
  for (const std::string& map_name : map_names) {
    const std::string stamp_file = directory + "/maps/" + map_name + "/timestamp.txt";
    if (!std::filesystem::exists(stamp_file)) {
      LOG(WARNING) << "Timestamp file '" << stamp_file << "' does not exist, skipping.";
      continue;
    }
    uint64_t value;
    std::ifstream stamp_stream(stamp_file);
    stamp_stream >> value;
    result.emplace_back(value);
  }
  return result;
}

}  // namespace khronos
