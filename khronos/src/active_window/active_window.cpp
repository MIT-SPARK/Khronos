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

#include "khronos/active_window/active_window.h"

#include <hydra/common/global_info.h>
#include <hydra/input/input_conversion.h>
#include <hydra/input/sensor_utilities.h>
#include <hydra/reconstruction/integration_masking.h>

#include "khronos/utils/geometry_utils.h"
#include "khronos/utils/khronos_attribute_utils.h"

namespace khronos {

void declare_config(ActiveWindow::Config& config) {
  using namespace config;
  name("ActiveWindow::Config");
  base<hydra::ActiveWindowModule::Config>(config);
  field(config.verbosity, "verbosity");
  field(config.detach_object_extraction, "detach_object_extraction");
  field(config.min_output_separation, "min_output_separation", "s");
  field(config.projective_integrator, "projective_integrator");
  field(config.tracking_integrator, "tracking_integrator");
  config.motion_detector.setOptional();
  field(config.motion_detector, "motion_detector");
  config.object_detector.setOptional();
  field(config.object_detector, "object_detector");
  config.tracker.setOptional();
  field(config.tracker, "tracker");
  config.object_extractor.setOptional();
  field(config.object_extractor, "object_extractor");
  field(config.extraction_worker, "extraction_worker");
  field(config.mesh_integrator, "mesh_integrator");
  field(config.frame_data_buffer, "frame_data_buffer");
  field(config.khronos_sinks, "khronos_sinks");
}

ActiveWindow::ActiveWindow(const Config& config, const OutputQueue::Ptr& output_queue)
    : hydra::ActiveWindowModule(config, output_queue),
      config(config::checkValid(config)),
      integrator_(config.projective_integrator),
      tracking_integrator_(config.tracking_integrator),
      mesh_integrator_(config.mesh_integrator),
      extraction_worker_(config.extraction_worker, config.object_extractor.create()),
      sinks_(KhronosSink::instantiate(config.khronos_sinks)),
      frame_data_buffer_(config.frame_data_buffer) {
  // Create member processors as specified in the configs.
  motion_detector_ = config.motion_detector.create();
  if (!motion_detector_) {
    motion_detector_ = std::make_unique<MotionDetector>();
  }
  object_detector_ = config.object_detector.create();
  if (!object_detector_) {
    object_detector_ = std::make_unique<ObjectDetector>();
  }
  tracker_ = config.tracker.create();
  if (!tracker_) {
    tracker_ = std::make_unique<Tracker>();
    if (config.motion_detector || config.object_detector) {
      LOG(WARNING)
          << "[Khronos Active Window] Tracker was not specified but motion and/or object detector "
             "is present. These detections will not be tracked or extracted.";
    }
  }

  if (!map_.config.with_tracking) {
    LOG(WARNING) << "[Khronos Active Window] Tracking layer disabled for volumetric map! Tracking "
                    "layer is strongly recommended as block archival and motion detection may not "
                    "work as intended!";
  }
}

std::string ActiveWindow::printInfo() const {
  return config::toString(config) + "\n" + KhronosSink::printSinks(sinks_);
}

void ActiveWindow::addKhronosSink(const KhronosSink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

hydra::ActiveWindowOutput::Ptr ActiveWindow::spinOnce(const hydra::InputPacket& input) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_stamp_ = input.timestamp_ns;
  Timer timer("active_window/all", latest_stamp_);

  // Create a data package for the given input.
  std::shared_ptr<FrameData> data = createData(input);

  // Detect dynamic points.
  motion_detector_->processInput(map_, *data);

  // Extract semantic objects.
  object_detector_->processInput(map_, *data);

  // Initialize tracking for new detections and track and associate objects
  // throughout frames.
  tracker_->processInput(*data);

  // Volumetric reconstruction in active window map.
  updateMap(*data);

  // Save the frame for later use and free-up memory of frames no longer used.
  frame_data_buffer_.trimBuffer(tracker_->getTracks());
  frame_data_buffer_.storeData(data);

  CLOG(4) << "[Khronos Active Window] Frame data buffer size: " << frame_data_buffer_.size()
          << ", object extraction threads running: " << extraction_worker_.numRunning() << ".";
  if (num_frames_processed_ % 10 == 0) {
    CLOG(3) << "[Khronos Active Window] Processed input frame " << num_frames_processed_ << " ("
            << input.timestamp_ns << "). Queues: " << input_queue_->size() << " input,  "
            << output_queue_->size() << " frontend.";
  }
  ++num_frames_processed_;

  Timer sink_timer("active_window/sinks", latest_stamp_);
  KhronosSink::callAll(sinks_, *data, map_, tracker_->getTracks());
  sink_timer.stop();

  // TODO(lschmid): Double check this does what's intended. Check whether we can move mesh
  // extraction here and whether it makes sense to move launching object threads before this.
  if (last_full_upated_ + fromSeconds(config.min_output_separation) > latest_stamp_) {
    return nullptr;
  }

  // Extract the resulting output and push to frontend queue.
  CLOG(5) << "[Khronos Active Window] Extracting output data.";
  auto output = extractOutputData(*data, config.detach_object_extraction);
  output->sensor_data = std::make_shared<hydra::InputData>(data->input);
  last_full_upated_ = latest_stamp_;

  // unset update flags
  for (const auto& block : map_.getTsdfLayer()) {
    block.clearUpdated();
  }

  return output;
}

void ActiveWindow::finishMapping() {
  std::lock_guard<std::mutex> lock(mutex_);

  // Mark all voxels and tracks as inactive, then extract the resulting output. This will duplicate
  // / append to the last received frame pose and stamp.
  for (auto& block : *map_.getTrackingLayer()) {
    block.has_active_data = false;
  }
  for (Track& track : tracker_->getTracks()) {
    track.is_active = false;
  }
  // Extract objects and wait for them to finish extraction.
  extractOutputData(frame_data_buffer_.getLatestData(), false);
}

std::vector<std::shared_ptr<KhronosObjectAttributes>> ActiveWindow::extractObjects() {
  std::vector<std::shared_ptr<KhronosObjectAttributes>> result;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& track : tracker_->getTracks()) {
    auto output_object = extraction_worker_.runBlocking(track, frame_data_buffer_);
    if (output_object) {
      result.emplace_back(std::move(output_object));
    }
  }
  return result;
}

void ActiveWindow::updateMap(const FrameData& data) {
  Timer timer("active_window/update_map", latest_stamp_);

  // Perform projective TSDF integration for all potentially visible blocks.
  // TODO(nathan) also add semantic masking
  cv::Mat integration_mask;
  hydra::maskNonZero(data.dynamic_image, integration_mask);
  integrator_.updateMap(data.input, map_, true, integration_mask);

  // Update the tracking information for all touched blocks. This resets
  // deactivated voxels so needs to come after meshing.
  tracking_integrator_.updateBlocks(data, map_);
}

hydra::ActiveWindowOutput::Ptr ActiveWindow::extractOutputData(const FrameData& data,
                                                               bool threaded) {
  // Extract background mesh and objects that leaves the active window.
  Timer timer("active_window/extract_output", latest_stamp_);

  // Reconstruct the mesh from TSDF.
  mesh_integrator_.generateMesh(map_, true, true);

  auto output = std::make_shared<hydra::ActiveWindowOutput>();
  output->timestamp_ns = data.input.timestamp_ns;
  output->world_t_body = data.input.world_T_body.translation();
  output->world_R_body = data.input.world_T_body.rotation();
  output->setMap(map_.cloneUpdated());

  // NOTE(nathan) comes after cloning the map and generating the mesh to preserve updated blocks
  // that have left the temporal window. This can only happen if the active window has a temporal
  // window smaller than min_input_separation_s (or in other rarer situations where the data period
  // is larger than the temporal window)
  tracking_integrator_.resetInactive(map_, &output->archived_mesh_indices);
  CLOG(4) << "[Khronos Active Window] Archiving " << output->archived_mesh_indices.size()
          << " blocks.";

  extractInactiveObjects();
  if (!threaded) {
    extraction_worker_.join();
  }

  // TODO(nathan) fix the layer update to not use LayerId
  auto update = std::make_shared<hydra::LayerUpdate>(2);
  output->graph_update[update->layer] = update;
  extraction_worker_.fill(*update);
  return output;
}

void ActiveWindow::extractInactiveObjects() {
  // Extract all inactive objects to the output and remove the track from the
  // tracker.
  auto it = tracker_->getTracks().begin();
  while (it != tracker_->getTracks().end()) {
    if (it->is_active) {
      it++;
      continue;
    }

    // NOTE(lschmid) Move the track and copy the frame data buffer to the thread. The buffer will
    // keep relevant frames alive while the AW updates.
    extraction_worker_.submit(latest_stamp_, std::move(*it), frame_data_buffer_);
    it = tracker_->getTracks().erase(it);
  }
}

std::unique_ptr<FrameData> ActiveWindow::createData(const hydra::InputPacket& input) const {
  Timer timer("active_window/create_data", latest_stamp_);
  // Compute all required other data from the inputs. Right now also allocates
  // all other data (such as the dynamic and object image), so we don't need to
  // check for this if these modules are disabled.

  // Normalize raw input packet into the standard format.
  auto input_data = hydra::conversions::parseInputPacket(input, true, map_.hasSemantics());
  if (!input_data) {
    LOG(ERROR) << "[Khronos Active Window] Input packet preprocessing failed. Skipping frame.";
    return nullptr;
  }

  // Allocate other data internally carried by Khronos.
  std::unique_ptr<FrameData> result = std::make_unique<FrameData>(std::move(*input_data));
  result->dynamic_image = cv::Mat::zeros(result->input.depth_image.size(), CV_32SC1);
  result->object_image = cv::Mat::zeros(result->input.depth_image.size(), CV_32SC1);
  return result;
}

}  // namespace khronos
