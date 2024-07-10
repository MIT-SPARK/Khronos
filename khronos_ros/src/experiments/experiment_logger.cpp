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

#include "khronos_ros/experiments/experiment_logger.h"

#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <glog/logging.h>

namespace khronos {

ExperimentLogger::ExperimentLogger(const std::string& data_dir, const std::string& log_file_name) {
  log_file_.open(data_dir + "/" + log_file_name, std::ios::in | std::ios::app);
  if (!log_file_.is_open()) {
    LOG(ERROR) << "[ExperimentLogger] Could not open log file '" << data_dir + "/" + log_file_name
               << "'. No logging will be performed.";
  }
}
ExperimentLogger::~ExperimentLogger() {
  if (log_file_.is_open()) {
    log_file_.close();
  }
}

void ExperimentLogger::log(const std::string& msg) {
  if (log_file_.is_open()) {
    log_file_ << getTimeStamp() << msg << std::endl;
  }
  if (also_log_to_console_) {
    std::cout << msg << std::endl;
  }
}

void ExperimentLogger::setFlag(const std::string& flag, const std::string& value) {
  if (log_file_.is_open()) {
    log_file_ << "[FLAG] [" << flag << "] " << value << std::endl;
  }
}

bool ExperimentLogger::hasFlag(const std::string& flag) {
  bool found = false;
  if (log_file_.is_open()) {
    log_file_.seekg(std::ios_base::beg);
    std::string line;
    const std::string flag_str = "[FLAG] [" + flag + "] ";
    while (std::getline(log_file_, line)) {
      if (line.find(flag_str) == 0) {
        found = true;
        break;
      }
    }
  }
  log_file_.seekp(std::ios_base::end);
  log_file_.clear();
  return found;
}

std::string ExperimentLogger::getFlag(const std::string& flag) {
  std::string result;
  if (log_file_.is_open()) {
    log_file_.seekg(std::ios_base::beg);
    std::string line;
    const std::string flag_str = "[FLAG] [" + flag + "] ";
    while (std::getline(log_file_, line)) {
      if (line.find(flag_str) == 0) {
        result = line.substr(flag_str.size());
        break;
      }
    }
  }
  log_file_.seekp(std::ios_base::end);
  log_file_.clear();
  return result;
}

std::string ExperimentLogger::getTimeStamp() const {
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  std::stringstream ss;
  ss << std::put_time(&tm, "[%d.%m.%Y %H:%M:%S] ");
  return ss.str();
}

}  // namespace khronos
