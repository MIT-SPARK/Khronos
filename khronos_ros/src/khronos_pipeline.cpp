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

#include "khronos_ros/khronos_pipeline.h"

#include <filesystem>
#include <memory>

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/ros.h>
#include <hydra/common/global_info.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra_ros/backend/ros_backend_publisher.h>
#include <hydra_ros/frontend/ros_frontend_publisher.h>
#include <hydra_ros/loop_closure/ros_lcd_registration.h>
#include <khronos/utils/khronos_attribute_utils.h>
#include <khronos_msgs/Changes.h>

#include "khronos_ros/utils/ros_conversions.h"
#include "khronos_ros/utils/ros_namespaces.h"

namespace khronos {

void declare_config(KhronosPipeline::Config& config) {
  using namespace config;
  name("KhronosPipeline");
  field(config.verbosity, "verbosity");
  field(config.finish_processing_on_shutdown, "finish_processing_on_shutdown");
  field(config.save_active_window_objects, "save_active_window_objects");
  base<hydra::PipelineConfig>(config);

  // Module configs.
  field(config.input, "input");
  field(config.active_window, "active_window");
  field(config.frontend, "frontend");
  field(config.backend, "backend");
}

KhronosPipeline::Config initializeConfig(const ros::NodeHandle& nh) {
  auto config = config::fromRos<KhronosPipeline::Config>(nh);
  config.enable_places = false;
  return config;
}

KhronosPipeline::KhronosPipeline(const ros::NodeHandle& nh)
    : config(config::checkValid(initializeConfig(nh))),
      nh_(nh),
      map_visualizer_(ros::NodeHandle(nh, RosNs::VISUALIZATION)) {
  hydra::GlobalInfo::init(config, 0, true);
  setupDsgs();
  setupMembers();
  setupRos();
}

void KhronosPipeline::start() {
  input_module_->start();
  active_window_->start();
  frontend_->start();
  backend_->start();
  if (lcd_) {
    lcd_->start();
  }
  CLOG(1) << "[Khronos Pipeline] Started.";
}

void KhronosPipeline::stop() {
  CLOG(1) << "[Khronos Pipeline] Stopping...";
  hydra::GlobalInfo::instance().setForceShutdown(!config.finish_processing_on_shutdown);

  input_module_->stop();
  if (config.finish_processing_on_shutdown) {
    CLOG(1) << "[Khronos Pipeline] Finishing processing " << active_window_->queue()->size()
            << " input frames...";
  }

  active_window_->stop();
  if (config.finish_processing_on_shutdown) {
    active_window_->finishMapping();
    CLOG(1) << "[Khronos Pipeline] Finishing processing " << frontend_->queue()->size()
            << " frontend packets ...";
  }
  frontend_->stop();

  if (config.finish_processing_on_shutdown) {
    CLOG(1) << "[Khronos Pipeline] Finishing processing "
            << hydra::PipelineQueues::instance().backend_queue.size() << " backend packets ...";
  }
  backend_->stop();
  if (config.finish_processing_on_shutdown) {
    CLOG(1) << "[Khronos Pipeline] Running final optimization.";
    backend_->finishProcessing();
  }

  if (lcd_) {
    lcd_->stop();
  }
  CLOG(1) << "[Khronos Pipeline] Stopped.";
}

bool KhronosPipeline::save(const hydra::LogSetup& log_setup, bool save_full_state) {
  if (!log_setup.valid()) {
    LOG(WARNING) << "[Khronos Pipeline] Could not save: Invalid log setup.";
    return false;
  }

  // Save state of all modules.
  if (save_full_state) {
    active_window_->save(log_setup);
    frontend_->save(log_setup);
    if (lcd_) {
      lcd_->save(log_setup);
    }
    backend_->save(log_setup);
  }

  // Always save evaluation relevant data.
  backend_->saveProposedMerges(log_setup);
  DynamicSceneGraph::Ptr dsg = backend_->getDsg().clone();
  const auto backend_path = log_setup.getLogDir("backend");

  // Add all objects that are currently in the active window.
  auto aw_objects = active_window_->extractObjects();
  size_t current_object_id = 0;
  for (const auto& [id, attrs] : dsg->getLayer(DsgLayers::OBJECTS).nodes()) {
    current_object_id = std::max(current_object_id, NodeSymbol(id).categoryId());
  }

  for (auto& object : aw_objects) {
    NodeSymbol object_symbol('O', ++current_object_id);
    dsg->emplaceNode(DsgLayers::OBJECTS,
                     object_symbol,
                     std::make_unique<KhronosObjectAttributes>(std::move(*object)));
  }
  dsg->save(backend_path + "/dsg_with_mesh", true);

  CLOG(2) << "[Khronos Pipeline] Saved " << (save_full_state ? "full state" : "evaluation DSG")
          << " to '" << log_setup.getLogDir() << "'.";
  return true;
}

void KhronosPipeline::finishMapping() {
  // Call to stop() to prevent new frames from being added.
  stop();
  active_window_->finishMapping();
  map_visualizer_.visualizeAllMaps(active_window_->getMap());
}

void KhronosPipeline::setupDsgs() {
  // Populate the scene graphs and shared state.
  SharedDsgInfo::Config layer_config{{{DsgLayers::OBJECTS, 2},
                                      {DsgLayers::PLACES, 3},
                                      {DsgLayers::ROOMS, 4},
                                      {DsgLayers::BUILDINGS, 5}}};
  // Frontend scene graph.
  frontend_dsg_ = std::make_shared<SharedDsgInfo>(layer_config);
  frontend_dsg_->graph->setMesh(std::make_shared<spark_dsg::Mesh>(true, true, false, true));

  // Bacend scene graph.
  backend_dsg_ = std::make_shared<SharedDsgInfo>(layer_config);
  backend_dsg_->graph->setMesh(std::make_shared<spark_dsg::Mesh>(true, true, false, true));

  // Shared state.
  shared_state_.reset(new SharedModuleState());
  shared_state_->lcd_graph = std::make_shared<SharedDsgInfo>(layer_config);
  shared_state_->backend_graph = std::make_shared<SharedDsgInfo>(layer_config);
}

void KhronosPipeline::setupMembers() {
  // Setup the frontend.
  frontend_ = std::make_shared<GraphBuilder>(config.frontend, frontend_dsg_, shared_state_);

  // Setup the active window.
  active_window_ = std::make_shared<ActiveWindow>(config.active_window, frontend_->queue());

  // Setup the input module.
  input_module_ = std::make_shared<hydra::RosInputModule>(config.input, active_window_->queue());

  // Setup the backend.
  backend_ = std::make_shared<Backend>(config.backend, backend_dsg_, shared_state_);

  // Setup lood closure detection.
  // TODO(lschmid): Streamline all of this with optional configs.
  if (config.enable_lcd) {
    auto lcd_config = config::fromRos<hydra::LoopClosureConfig>(nh_, RosNs::LOOPCLOSURE);
    lcd_config.detector.num_semantic_classes = hydra::GlobalInfo::instance().getTotalLabels();
    CLOG(3) << "Number of classes for LCD: " << lcd_config.detector.num_semantic_classes;
    lcd_.reset(new LoopClosureModule(lcd_config, shared_state_));

    // noop if bow vectors are not enabled
    bow_subscriber_ = std::make_unique<BowSubscriber>(nh_);
    if (lcd_config.detector.enable_agent_registration) {
      lcd_->getDetector().setRegistrationSolver(0, std::make_unique<hydra::lcd::DsgAgentSolver>());
    }
  }
}

void KhronosPipeline::setupRos() {
  // Add callbacks to all modules for visualization and evaluation.
  active_window_->addSink(ActiveWindow::Sink::fromCallback(
      [this](const auto& frame_data, const auto& map, const auto& tracks) {
        map_visualizer_.visualizeAll(map, frame_data, tracks);
        if (evaluate_aw_) {
          aw_evaluation_callback_(map, frame_data, tracks);
        }
      }));
  frontend_->addSink(
      std::make_shared<hydra::RosFrontendPublisher>(ros::NodeHandle(nh_, RosNs::FRONTEND)));
  backend_->addSink(
      std::make_shared<hydra::RosBackendPublisher>(ros::NodeHandle(nh_, RosNs::BACKEND)));
  backend_->addSink(Backend::Sink::fromMethod(&KhronosPipeline::sendChanges, this));
  backend_->addSink(
      Backend::Sink::fromCallback([this](uint64_t timestamp_ns, const auto& dsg, const auto& dfg) {
        if (evaluate_backend_) {
          backend_evaluation_callback_(timestamp_ns, dsg, dfg);
        }
      }));

  // Ros publishers.
  changes_pub_ = nh_.advertise<khronos_msgs::Changes>("changes", 10, true);
  finish_mapping_srv_ =
      nh_.advertiseService("finish_mapping", &KhronosPipeline::finishMappingCallback, this);
}

std::string KhronosPipeline::getConfigInfo() const {
  // TODO(lschmid): Find a more general way to do this when integrating with hydra.
  std::stringstream ss;
  ss << config::toString(hydra::GlobalInfo::instance().getConfig()) << "\n";
  ss << config::toString(config) << "\n";
  ss << config::toString(active_window_->config) << "\n";
  ss << config::toString(frontend_->config) << "\n";
  ss << config::toString(backend_->config);
  return ss.str();
}

void KhronosPipeline::sendChanges(uint64_t timestamp_ns,
                                  const DynamicSceneGraph&,
                                  const kimera_pgmo::DeformationGraph&) const {
  khronos_msgs::Changes msg = toMsg(backend_->getChanges());
  msg.header.stamp.fromNSec(timestamp_ns);
  changes_pub_.publish(msg);
}

bool KhronosPipeline::finishMappingCallback(std_srvs::Empty::Request& /* req */,
                                            std_srvs::Empty::Response& /* res */) {
  finishMapping();
  return true;
}

void KhronosPipeline::setActiveWindowEvaluationCallback(
    const ActiveWindowEvaluationCallback& callback_function) {
  aw_evaluation_callback_ = callback_function;
  evaluate_aw_ = true;
}

void KhronosPipeline::setBackendEvaluationCallback(
    const BackendEvaluationCallback& callback_function) {
  backend_evaluation_callback_ = callback_function;
  evaluate_backend_ = true;
}

}  // namespace khronos
