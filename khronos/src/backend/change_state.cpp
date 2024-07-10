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

#include "khronos/backend/change_state.h"

#include <fstream>
#include <sstream>
#include <string>

#include <glog/logging.h>
#include <hydra/utils/csv_reader.h>
namespace khronos {

bool RPGOMerges::save(const std::string& filename) const {
  std::ofstream save_file(filename);
  if (!save_file.is_open()) {
    LOG(ERROR) << "Could not open file '" << filename << "' for writing.";
    return false;
  }
  std::string header;
  for (const std::string& h : kHeaderNames) {
    header += h + ",";
  }
  header.pop_back();
  save_file << header << std::endl;

  for (const auto& merge : *this) {
    save_file << merge.from_node << "," << merge.to_node << "," << merge.is_valid << std::endl;
  }
  save_file.close();
  return true;
}

bool RPGOMerges::load(const std::string& filename) {
  // Read data.
  hydra::CsvReader reader(filename);
  if (!reader.isSetup()) {
    return false;
  }

  // Check header format.
  for (const std::string& header : kHeaderNames) {
    if (!reader.hasHeader(header)) {
      LOG(WARNING) << "Could not find header '" << header << "' in file '" << filename << "'.";
      return false;
    }
  }

  // Set the data from the reader.
  resize(reader.numRows());
  for (size_t i = 0; i < reader.numRows(); ++i) {
    RPGOMerge& merge = at(i);
    merge.from_node = std::stoull(reader.getEntry(kHeaderNames[0], i));
    merge.to_node = std::stoull(reader.getEntry(kHeaderNames[1], i));
    merge.is_valid = std::stoi(reader.getEntry(kHeaderNames[2], i));
  }
  return true;
}

RPGOMerges RPGOMerges::fromFile(const std::string& filename) {
  RPGOMerges result;
  result.load(filename);
  return result;
}

bool ObjectChanges::save(const std::string& filename) const {
  std::ofstream save_file(filename);
  if (!save_file.is_open()) {
    LOG(ERROR) << "Could not open file '" << filename << "' for writing.";
    return false;
  }
  std::string header;
  for (const std::string& h : kHeaderNames) {
    header += h + ",";
  }
  header.pop_back();
  save_file << header << std::endl;

  for (const auto& object_change : *this) {
    save_file << object_change.node_id << "," << object_change.merged_id << ","
              << object_change.first_absent << "," << object_change.last_absent << ","
              << object_change.first_persistent << "," << object_change.last_persistent
              << std::endl;
  }
  save_file.close();
  return true;
}

// TODO(lschmid): Probably can factor out a lot of the common parts into some io utils.
bool ObjectChanges::load(const std::string& filename) {
  // Read data.
  hydra::CsvReader reader(filename);
  if (!reader.isSetup()) {
    return false;
  }

  // Check header format.
  if (reader.checkRequiredHeaders(kHeaderNames)) {
    return false;
  }

  // Set the data from the reader.
  resize(reader.numRows());
  for (size_t i = 0; i < reader.numRows(); ++i) {
    ObjectChange& change = at(i);
    change.node_id = std::stoull(reader.getEntry(kHeaderNames[0], i));
    change.merged_id = std::stoull(reader.getEntry(kHeaderNames[1], i));
    change.first_absent = std::stoull(reader.getEntry(kHeaderNames[2], i));
    change.last_absent = std::stoull(reader.getEntry(kHeaderNames[3], i));
    change.first_persistent = std::stoull(reader.getEntry(kHeaderNames[4], i));
    change.last_persistent = std::stoull(reader.getEntry(kHeaderNames[5], i));
  }
  return true;
}

ObjectChanges ObjectChanges::fromFile(const std::string& filename) {
  ObjectChanges result;
  result.load(filename);
  return result;
}

std::vector<ObjectChange>::iterator ObjectChanges::find(NodeId node_id) {
  return std::find_if(
      begin(), end(), [node_id](const ObjectChange& change) { return change.node_id == node_id; });
}

std::vector<ObjectChange>::const_iterator ObjectChanges::find(NodeId node_id) const {
  return std::find_if(
      begin(), end(), [node_id](const ObjectChange& change) { return change.node_id == node_id; });
}

bool BackgroundChanges::save(const std::string& filename) const {
  std::ofstream save_file(filename);
  if (!save_file.is_open()) {
    LOG(ERROR) << "Could not open file '" << filename << "' for writing.";
    return false;
  }
  // Write as a single line, but as csv so other tools can also read it.
  bool first = true;
  for (const auto& change : *this) {
    if (first) {
      save_file << static_cast<int>(change);
      first = false;
    } else {
      save_file << "," << static_cast<int>(change);
    }
  }
  save_file.close();
  return true;
}

bool BackgroundChanges::load(const std::string& filename) {
  // Read data.
  std::ifstream read_file(filename);
  if (!read_file.is_open()) {
    LOG(ERROR) << "Could not open file '" << filename << "' for reading.";
    return false;
  }

  // Convert to string.
  std::stringstream buffer;
  buffer << read_file.rdbuf();
  std::string string_values = buffer.str();

  // Read values.
  clear();
  size_t pos = 0;
  while ((pos = string_values.find(',')) != std::string::npos) {
    emplace_back(static_cast<ChangeState>(std::stoi(string_values.substr(0, pos))));
    string_values.erase(0, pos + 1);
  }
  emplace_back(static_cast<ChangeState>(std::stoi(string_values)));
  return true;
}

BackgroundChanges BackgroundChanges::fromFile(const std::string& filename) {
  BackgroundChanges result;
  result.load(filename);
  return result;
}

}  // namespace khronos
