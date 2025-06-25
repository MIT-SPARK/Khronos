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

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <config_utilities/config_utilities.h>
#include <hydra/active_window/active_window_module.h>
#include <hydra/common/global_info.h>
#include <hydra/common/output_sink.h>
#include <hydra/reconstruction/mesh_integrator.h>
#include <hydra/reconstruction/projective_integrator.h>

#include "khronos/active_window/data/frame_data.h"
#include "khronos/active_window/data/frame_data_buffer.h"
#include "khronos/active_window/data/reconstruction_types.h"
#include "khronos/active_window/integration/tracking_integrator.h"
#include "khronos/active_window/motion_detection/motion_detector.h"
#include "khronos/active_window/object_detection/object_detector.h"
#include "khronos/active_window/object_extraction/object_worker_pool.h"
#include "khronos/active_window/tracking/tracker.h"
#include "khronos/common/common_types.h"

namespace khronos {

class ActiveWindow : public hydra::ActiveWindowModule {
 public:
  using KhronosSink = hydra::OutputSink<const FrameData&, const VolumetricMap&, const Tracks&>;

  // Config.
  struct Config : hydra::ActiveWindowModule::Config {
    int verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;

    // Minimum duration for full updates and sending an output message [s]. A value of 0 will
    // perform full updates on every frame.
    float min_output_separation = 0.0f;

    // If true, run object extraction in detached threads to avoid blocking the active window.
    bool detach_object_extraction = true;

    // Configs of the sub-modules to create.
    hydra::ProjectiveIntegrator::Config projective_integrator;
    TrackingIntegrator::Config tracking_integrator;
    config::VirtualConfig<MotionDetector> motion_detector;
    config::VirtualConfig<ObjectDetector> object_detector;
    config::VirtualConfig<Tracker> tracker;
    config::VirtualConfig<ObjectExtractor> object_extractor;
    ObjectWorkerPool::Config extraction_worker;
    hydra::MeshIntegratorConfig mesh_integrator;
    FrameDataBuffer::Config frame_data_buffer;
    std::vector<KhronosSink::Factory> khronos_sinks;

    // override layer defaults of Hydra
    Config() : hydra::ActiveWindowModule::Config(false, true) {}
  } const config;

  // Construction.
  ActiveWindow(const Config& config, const OutputQueue::Ptr& output_queue);
  virtual ~ActiveWindow() = default;

  std::string printInfo() const override;

  // Access. These are not thread-safe!
  VolumetricMap& getMap() { return map_; }
  const VolumetricMap& getMap() const { return map_; }
  const FrameData& getLatestFrameData() const { return frame_data_buffer_.getLatestData(); }
  const Tracks& getTracks() const { return tracker_->getTracks(); }

  // Module setup.
  /**
   * @brief Add a sink to the active window. The sink will be called whenever the active window
   * finishes processing a frame.
   * @param sink The sink to add.
   */
  void addKhronosSink(const KhronosSink::Ptr& sink);

  // Interaction.
  /**
   * @brief Finishes the mapping process of the active window. This will act as
   * if infinite time without further observations have passed and will reset
   * the currently trackd data inside the active window. Pushes all data currently in the active
   * window to the output. NOTE(lschmid): This currently duplicates the last timestamp received.
   * Maybe better to take a separate stamp as input? Also, the final output data will not contain a
   * transform.
   */
  void finishMapping();

  // TMP: Extract objects for evaluation.
  std::vector<std::shared_ptr<KhronosObjectAttributes>> extractObjects();

 protected:
  // Spin the active window in a separate thread.
  hydra::ActiveWindowOutput::Ptr spinOnce(const hydra::InputPacket& input) override;

  // Processing.
  /**
   * @brief Create a data package from the given input, initializing all
   * subsequently used fields and computing the range and vertex images.
   * @param input The input data packet.
   * @returns The Khronos data with normalized input data and allocated internal types.
   */
  std::unique_ptr<FrameData> createData(const hydra::InputPacket& input) const;

  /**
   * @brief Update the volumetric map with the given data.
   * @param data The data to use for updating the map.
   */
  void updateMap(const FrameData& data);

  /**
   * @brief Extract all objects and background meshes that have turned inactive,
   * i.e. are exiting the active window. Archive and remove inactive blocks from the map.
   * @param data The frame data corresponding to this output.
   * @param threaded If true, extract objects in detached threads. Otherwise wait for object
   * extraction to finish.
   */
  hydra::ActiveWindowOutput::Ptr extractOutputData(const FrameData& data, bool threaded);

  /**
   * @brief Extract all objects that have turned inactive, i.e. are exiting the
   * active and meet the minimum confidence requirement.
   * @param output The output data to store the extracted objects in.
   */
  void extractInactiveObjects();

 protected:
  // Members.
  hydra::ProjectiveIntegrator integrator_;
  TrackingIntegrator tracking_integrator_;
  hydra::MeshIntegrator mesh_integrator_;
  std::unique_ptr<MotionDetector> motion_detector_;
  std::unique_ptr<ObjectDetector> object_detector_;
  std::unique_ptr<Tracker> tracker_;
  ObjectWorkerPool extraction_worker_;

  std::mutex mutex_;
  KhronosSink::List sinks_;

  // Internal processing.
  // Keep frames in buffer for later extraction of objects.
  FrameDataBuffer frame_data_buffer_;

  // Variables.
  TimeStamp latest_stamp_;
  TimeStamp last_full_upated_ = 0;
  size_t num_frames_processed_ = 0;  // For info only.

 private:
  inline static const auto registration_ = config::
      RegistrationWithConfig<hydra::ActiveWindowModule, ActiveWindow, Config, OutputQueue::Ptr>(
          "ActiveWindow");
};

void declare_config(ActiveWindow::Config& config);

}  // namespace khronos
