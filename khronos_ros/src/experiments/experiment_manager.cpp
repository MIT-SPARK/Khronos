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

#include "khronos_ros/experiments/experiment_manager.h"

#include <array>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <glog/logging.h>
#include <ianvs/spin_functions.h>
#include <khronos/common/common_types.h>

namespace khronos {
namespace {

struct Proc {
  Proc(const std::string& cmd) : pipe_(popen(cmd.c_str(), "r")) {}

  ~Proc() {
    if (pipe_) {
      pclose(pipe_);
    }
  }

  operator bool() const { return pipe_ != nullptr; }

  std::string exec() {
    std::array<char, 128> buffer;
    std::string output;
    while (fgets(buffer.data(), buffer.size(), pipe_) != nullptr) {
      output += buffer.data();
    }

    return output;
  }

  FILE* pipe_;
};

}  // namespace

void declare_config(ExperimentManager::Config& config) {
  using namespace config;
  name("ExperimentManager");
  field(config.verbosity, "verbosity");
  field(config.output_dir, "output_dir");
  field(config.overwrite, "overwrite");
  field(config.log_config, "log_config");
  field(config.log_timing, "log_timing");
  field(config.log_timing_details, "log_timing_details");
  field(config.save_every_n_frames, "save_every_n_frames");
  field(config.save_full_state, "save_full_state");
  field(config.exit_after_clock, "exit_after_clock");
}

ExperimentManager::ExperimentManager(const Config& config,
                                     ianvs::NodeHandle nh,
                                     std::shared_ptr<KhronosPipeline> khronos)
    : config(config::checkValid(config)),
      data_dir_(config.output_dir, hydra::DataDirectory::Config{true, config.overwrite}),
      khronos_(std::move(khronos)),
      nh_(nh) {
  // Verify output directory.
  if (!data_dir_) {
    CLOG(1) << "[ExperimentManager] No output directory specified. Not saving experiment data.";
    return;
  }
  logger_ = std::make_shared<ExperimentLogger>(data_dir_.path());
  logger_->alsoLogToConsole(config.verbosity > 0);
  logger_->log("Setup output directory '" + data_dir_.path().string() + "'.");

  // Setup timers.
  hydra::timing::ElapsedTimeRecorder::instance().timing_disabled = !config.log_timing;
  hydra::timing::ElapsedTimeRecorder::instance().disable_output = false;

  // Setup saving the map.
  if (config.save_every_n_frames > 0) {
    addBackendCallback(
        config.save_every_n_frames,
        [this](TimeStamp timestamp, const auto&, const auto&) { saveMap(timestamp); });
  }

  // Setup the khronos callback for potential frame-based evaluation.
  if (!frontend_callbacks_.empty()) {
    khronos_->setActiveWindowEvaluationCallback(
        std::bind(&khronos::ExperimentManager::evaluateFrontend,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3));
  }
  if (!backend_callbacks_.empty()) {
    khronos_->setBackendEvaluationCallback(std::bind(&khronos::ExperimentManager::evaluateBackend,
                                                     this,
                                                     std::placeholders::_1,
                                                     std::placeholders::_2,
                                                     std::placeholders::_3));
  }

  CLOG(1) << "[ExperimentManager] Setup output directory: '" << data_dir_.path() << "'.";

  // Setup service for termination.
  finish_mapping_and_save_srv_ = nh_.create_service<std_srvs::srv::Empty>(
      "finish_mapping_and_save", &ExperimentManager::finishMappingAndSaveCallback, this);
}

ExperimentManager::~ExperimentManager() { CLOG(1) << "[ExperimentManager] Finished."; }

void ExperimentManager::run() {
  // Run
  logConfigs();
  if (logger_) {
    logger_->log("Starting experiment.");
  }
  LOG(INFO) << "[ExperimentManager] Initializing pipeline...";
  khronos_->init();
  LOG(INFO) << "[ExperimentManager] Starting pipeline...";
  khronos_->start();
  ianvs::spinAndWait(nh_, config.exit_after_clock);
  if (logger_) {
    logger_->log("Stopping experiment.");
  }
  khronos_->stop();

  // Log and save data if requested.
  if (!final_map_saved_) {
    logData();
  }
}

void ExperimentManager::addFrontendCallback(size_t every_n_frames,
                                            ActiveWindowEvaluationCallback callback) {
  frontend_callbacks_.emplace_back(every_n_frames, callback);
  frontend_callback_counters_.emplace_back(0);
}

void ExperimentManager::addBackendCallback(size_t every_n_frames,
                                           BackendEvaluationCallback callback) {
  backend_callbacks_.emplace_back(every_n_frames, callback);
  backend_callback_counters_.emplace_back(0);
}

void ExperimentManager::finishMappingAndSaveCallback(
    const std_srvs::srv::Empty::Request::SharedPtr&,
    std_srvs::srv::Empty::Response::SharedPtr) {
  khronos_->stop();
  logData();
  final_map_saved_ = true;
}

void ExperimentManager::evaluateFrontend(const VolumetricMap& map,
                                         const FrameData& data,
                                         const Tracks& tracks) {
  Timer timer("evaluate/frontend/all", data.input.timestamp_ns);
  if (!data_dir_) {
    return;
  }
  // Perform all requested evaluations if their callback frequency is met.
  last_frontend_time_stamp_ = data.input.timestamp_ns;
  for (size_t i = 0; i < frontend_callbacks_.size(); ++i) {
    frontend_callback_counters_[i]++;
    if (frontend_callback_counters_[i] >= frontend_callbacks_[i].first) {
      frontend_callbacks_[i].second(map, data, tracks);
      frontend_callback_counters_[i] = 0;
    }
  }
}

void ExperimentManager::evaluateBackend(TimeStamp timestamp_ns,
                                        const DynamicSceneGraph& dsg,
                                        const kimera_pgmo::DeformationGraph& deformation_graph) {
  if (!data_dir_) {
    return;
  }

  // Perform all requested evaluations if their callback frequency is met.
  Timer timer("evaluate/backend/all", timestamp_ns);
  last_backend_time_stamp_ = timestamp_ns;
  for (size_t i = 0; i < backend_callbacks_.size(); ++i) {
    backend_callback_counters_[i]++;
    if (backend_callback_counters_[i] >= backend_callbacks_[i].first) {
      backend_callbacks_[i].second(timestamp_ns, dsg, deformation_graph);
      backend_callback_counters_[i] = 0;
    }
  }
}

void ExperimentManager::saveMap(size_t timestamp) {
  std::stringstream map_dir;
  map_dir << "snapshots/" << std::setw(5) << std::setfill('0') << num_saved_maps_++;

  // Ensure the snapshots directory exists
  const auto snapshots_path = data_dir_.path() / "snapshots";
  if (!std::filesystem::exists(snapshots_path)) {
    std::filesystem::create_directories(snapshots_path);
  }

  khronos_->save(data_dir_.child(map_dir.str()), config.save_full_state);
  std::ofstream out_file(data_dir_.path() / map_dir.str() / "timestamp.txt");
  out_file << timestamp;
  out_file.close();
}

void ExperimentManager::logData() {
  if (!data_dir_) {
    return;
  }
  CLOG(1) << "[ExperimentManager] Writing data to output directory: '" << data_dir_.path()
          << "' ...";

  // Log timing.
  if (config.log_timing || config.log_timing_details) {
    const auto timing_path = data_dir_.path("timing");
    hydra::timing::ElapsedTimeRecorder::instance().logStats(timing_path / "stats.csv");
    if (config.log_timing_details) {
      hydra::timing::ElapsedTimeRecorder::instance().logTimers(timing_path);
    }
  }

  if (evaluationIsOn()) {
    // TODO(Yun): someone figure out how they want these logs organized
    if (config.save_every_n_frames > 0) {
      // Don't save another snapshot here - the periodic saves are sufficient
      // Just log how many snapshots were saved
      const std::string snapshots_path = data_dir_.path() / "snapshots";
      if (std::filesystem::exists(snapshots_path)) {
        const std::size_t number_snapshots = std::distance(
            std::filesystem::directory_iterator(snapshots_path),
            std::filesystem::directory_iterator{});
        logger_->log("Saved '" + std::to_string(number_snapshots) + "' snapshots during operation.");
      } else if (config.save_every_n_frames > 0) {
        logger_->log("Snapshots directory not found, periodic saves may have failed.");
      }
    }

    // Save the resulting spatio-temporal map (final state only).
    khronos_->save(data_dir_);
    logger_->log("Saved final spatio-temporal map to '" + data_dir_.path().string() + "'.");
    logger_->setFlag("Experiment Finished Cleanly");
  }
  CLOG(1) << "[ExperimentManager] Finished writing data to output directory: '" << data_dir_.path()
          << "'.";
}

void ExperimentManager::logConfigs() {
  if (!config.log_config) {
    return;
  }
  const std::string params = config::toString(khronos_->config);
  LOG(INFO) << "[ExperimentManager] Config: \n" << params;

  if (!data_dir_) {
    return;
  }

  // Store git commit hash for reproducibility.
  // Assume we're running from ros2_ws directory and check ./src/khronos
  std::filesystem::path khronos_path = std::filesystem::current_path() / "src" / "khronos";

  if (std::filesystem::exists(khronos_path / ".git")) {
    std::string git_hash;
    std::string git_hash_cmd = "cd " + khronos_path.string() + " && git rev-parse HEAD 2>/dev/null";

    if (exec(git_hash_cmd, git_hash) && !git_hash.empty()) {
      // Remove trailing newline
      size_t last_char = git_hash.find_last_not_of("\n\r");
      if (last_char != std::string::npos) {
        git_hash.erase(last_char + 1);
      }

      logger_->log("Git hash: " + git_hash);

      // Store git hash in a separate file
      const auto git_hash_path = data_dir_.path() / "git_hash.txt";
      std::ofstream hash_file(git_hash_path);
      if (hash_file.is_open()) {
        hash_file << git_hash << std::endl;
        hash_file.close();
        logger_->log("Wrote git hash to '" + git_hash_path.string() + "'.");
      }

      // Check if there are uncommitted changes
      std::string git_status;
      std::string git_status_cmd = "cd " + khronos_path.string() + " && git status --porcelain 2>/dev/null";
      if (exec(git_status_cmd, git_status) && !git_status.empty()) {
        logger_->log("WARNING: Uncommitted changes detected in repository!");

        // Store the git status for reference
        const auto git_status_path = data_dir_.path() / "git_status.txt";
        std::ofstream status_file(git_status_path);
        if (status_file.is_open()) {
          status_file << git_status;
          status_file.close();
          logger_->log("Wrote git status to '" + git_status_path.string() + "'.");
        }
      }
    } else {
      logger_->log("Failed to retrieve git hash.");
    }
  } else {
    logger_->log("Git repository not found at: " + khronos_path.string());
  }

  // Print and store the realized config.
  const auto config_path = data_dir_.path() / "config.txt";
  std::ofstream out_file(config_path);
  if (out_file.is_open()) {
    out_file << params;
    out_file.close();
    logger_->log("Wrote config to '" + config_path.string() + "'.");
  } else {
    logger_->log("Failed to write config to '" + config_path.string() + "'.");
  }
}

bool ExperimentManager::exec(const std::string& cmd, std::string& output) {
  Proc proc(cmd);
  if (!proc) {
    return false;
  }

  output = proc.exec();
  return true;
}

}  // namespace khronos
