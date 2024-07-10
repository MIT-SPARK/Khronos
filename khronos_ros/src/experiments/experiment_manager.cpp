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

#include <config_utilities/parsing/ros.h>
#include <glog/logging.h>
#include <hydra_ros/utils/node_utilities.h>
#include <khronos/common/common_types.h>

namespace khronos {

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
}

ExperimentManager::ExperimentManager(const ros::NodeHandle& nh,
                                     std::shared_ptr<KhronosPipeline> khronos)
    : config(config::checkValid(config::fromRos<Config>(nh))),
      data_dir_(config.output_dir, true, config.overwrite),
      khronos_(std::move(khronos)),
      nh_(nh) {
  // Verify output directory.
  if (!data_dir_) {
    CLOG(1) << "[ExperimentManager] No output directory specified. Not saving experiment data.";
    return;
  }
  logger_ = std::make_shared<ExperimentLogger>(data_dir_);
  logger_->alsoLogToConsole(config.verbosity > 0);
  logger_->log("Setup output directory '" + data_dir_.getPath() + "'.");

  // Setup timers.
  if (config.log_timing || config.log_timing_details) {
    std::filesystem::create_directories(data_dir_.getPath() + "/timing");
  }
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

  CLOG(1) << "[ExperimentManager] Setup output directory: '" << data_dir_.getPath() << "'.";

  // Setup service for termination.
  finish_mapping_and_save_srv_ = nh_.advertiseService(
      "finish_mapping_and_save", &ExperimentManager::finishMappingAndSaveCallback, this);
}

ExperimentManager::~ExperimentManager() { CLOG(1) << "[ExperimentManager] Finished."; }

void ExperimentManager::run() {
  // Run
  logConfigs();
  if (logger_) {
    logger_->log("Starting experiment.");
  }
  khronos_->start();
  hydra::spinAndWait(nh_);
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

bool ExperimentManager::finishMappingAndSaveCallback(std_srvs::Empty::Request& /* req */,
                                                     std_srvs::Empty::Response& /* res */) {
  khronos_->finishMapping();
  logData();
  final_map_saved_ = true;
  return true;
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
  map_dir << data_dir_.getPath() << "/maps/" << std::setw(5) << std::setfill('0')
          << num_saved_maps_++;

  hydra::LogConfig log_config;
  log_config.log_dir = map_dir.str();
  hydra::LogSetup log_setup(log_config);

  khronos_->save(hydra::LogSetup(log_config), config.save_full_state);
  std::ofstream out_file(map_dir.str() + "/timestamp.txt");
  out_file << timestamp;
  out_file.close();
}

void ExperimentManager::logData() {
  if (!data_dir_) {
    return;
  }
  CLOG(1) << "[ExperimentManager] Writing data to output directory: '" << data_dir_.getPath()
          << "' ...";

  // Log timing.
  if (config.log_timing || config.log_timing_details) {
    hydra::LogConfig log_config;
    log_config.log_dir = data_dir_.getPath() + "/timing";
    log_config.log_raw_timers_to_single_dir = true;
    hydra::timing::ElapsedTimeRecorder::instance().logStats(log_config.log_dir + "/stats.csv");
    if (config.log_timing_details) {
      hydra::timing::ElapsedTimeRecorder::instance().logAllElapsed(hydra::LogSetup(log_config));
    }
  }
  if (evaluationIsOn()) {
    if (config.save_every_n_frames > 0) {
      // Save the final map if periodically saving.
      saveMap(last_backend_time_stamp_);
      const std::string map_path = data_dir_.getPath() + "/maps";
      const std::size_t number_maps = std::distance(std::filesystem::directory_iterator(map_path),
                                                    std::filesystem::directory_iterator{});
      logger_->log("Saved '" + std::to_string(number_maps) + "' maps.");
    }

    // Save the resulting spatio-temporal map.
    khronos_->save4DMap(data_dir_.getPath() + "/final.4dmap");
    logger_->log("Saved final spatio-temporal map to '" + data_dir_.getPath() + "'.");
    logger_->setFlag("Experiment Finished Cleanly");
  }
  CLOG(1) << "[ExperimentManager] Finished writing data to output directory: '"
          << data_dir_.getPath() << "'.";
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

  // Store ros params.
  const std::string params_file = data_dir_.getPath() + "/rosparams.yaml";
  std::string command = "rosparam dump " + params_file;
  if (system(command.c_str()) == 0) {
    logger_->log("Wrote ROS params to '" + params_file + "'.");
  } else {
    logger_->log("Failed to write ROS params to '" + params_file + "'.");
  }

  // Store git commit hash for reproducibility.
  std::string git_hash;
  command = "eval 'roscd khronos && git rev-parse HEAD'";
  if (exec(command.c_str(), git_hash)) {
    logger_->log("Running Khronos on git commit: " + git_hash);
  } else {
    logger_->log("Failed to get git hash.");
  }

  // Print and store the realized config.
  std::ofstream out_file(data_dir_.getPath() + "/config.txt");
  if (out_file.is_open()) {
    out_file << params;
    out_file.close();
    logger_->log("Wrote config to '" + data_dir_.getPath() + "/config.txt'.");
  } else {
    logger_->log("Failed to write config to '" + data_dir_.getPath() + "/config.txt'.");
  }
}

bool ExperimentManager::exec(const char* cmd, std::string& output) {
  std::array<char, 128> buffer;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    return false;
  }
  output.clear();
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    output += buffer.data();
  }
  return true;
}

}  // namespace khronos
