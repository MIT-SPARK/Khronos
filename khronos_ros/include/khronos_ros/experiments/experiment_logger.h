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

#include <fstream>
#include <memory>
#include <string>

namespace khronos {

/**
 * @brief Utility class that logs experiment and evaluation meta information to a log file. This
 * assumes that always only one logger in one thread is manipulating the file.
 */
class ExperimentLogger {
 public:
  // Types.
  using Ptr = std::shared_ptr<ExperimentLogger>;
  using ConstPtr = std::shared_ptr<const ExperimentLogger>;

  // Construction.
  explicit ExperimentLogger(const std::string& data_dir,
                            const std::string& log_file_name = LOG_FILE_NAME);
  ~ExperimentLogger();

  /**
   * @brief Check if logging is successfully setup.
   */
  bool isLogging() const { return log_file_.is_open(); }

  /**
   * @brief Set whether to also log to console.
   */
  void alsoLogToConsole(bool also_log_to_console = true) {
    also_log_to_console_ = also_log_to_console;
  }

  /**
   * @brief Add a message about the experiment and evaluation to the experiment log.
   * @param msg The message to add as single line string.
   */
  void log(const std::string& msg);

  /**
   * @brief Add a flag to the experiment log.
   * @param flag The name of the flag.
   * @param value Optional value for the flag as single line string.
   */
  void setFlag(const std::string& flag, const std::string& value = "");

  /**
   * @brief Check if a flag is set in the experiment log.
   * @param flag The name of the flag.
   * @return True if the flag is set, false otherwise.
   */
  bool hasFlag(const std::string& flag);

  /**
   * @brief Get the value of a flag from the experiment log if it is set. Returns the first flag
   * value if multiple.
   * @param flag The name of the flag.
   * @return The value of the flag as single line string.
   */
  std::string getFlag(const std::string& flag);

 protected:
  // Utility functions.
  std::string getTimeStamp() const;

 private:
  bool also_log_to_console_ = false;
  std::fstream log_file_;
  inline static const std::string LOG_FILE_NAME = "experiment_log.txt";
};

}  // namespace khronos
