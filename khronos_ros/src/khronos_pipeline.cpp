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
#include <config_utilities/parsing/context.h>
#include <hydra/common/global_info.h>
#include <hydra/common/pipeline_queues.h>
#include <hydra_ros/backend/ros_backend_publisher.h>
#include <hydra_ros/frontend/ros_frontend_publisher.h>
#include <khronos/active_window/active_window.h>
#include <khronos/backend/backend.h>
#include <khronos/utils/khronos_attribute_utils.h>
#include <khronos_msgs/msg/changes.hpp>

#include "khronos_ros/utils/ros_conversions.h"
#include "khronos_ros/utils/ros_namespaces.h"
#include "khronos_ros/visualization/active_window_visualizer.h"

namespace khronos {

using ChangeMsg = khronos_msgs::msg::Changes;

KhronosPipeline::Config::Config() : hydra::HydraRosPipeline::Config() {
  active_window = ActiveWindow::Config{};
  backend = Backend::Config{};
  verbosity = hydra::GlobalInfo::instance().getConfig().default_verbosity;
}

void declare_config(KhronosPipeline::Config& config) {
  using namespace config;
  name("KhronosPipeline::Config");
  base<hydra::HydraRosPipeline::Config>(config);
}

KhronosPipeline::KhronosPipeline(ianvs::NodeHandle nh)
    : hydra::HydraRosPipeline(0, 5),
      config(config::checkValid(config::fromContext<Config>())),
      nh_(nh),
      changes_pub_(nh_.create_publisher<ChangeMsg>("changes", rclcpp::QoS(10).transient_local())),
      khronos_backend_(nullptr),
      khronos_active_window_(nullptr) {}

void KhronosPipeline::init() {
  HydraRosPipeline::init();
  khronos_active_window_ = dynamic_cast<ActiveWindow*>(active_window_.get());
  khronos_backend_ = dynamic_cast<Backend*>(backend_.get());

  backend_->addSink(
      Backend::Sink::fromCallback([this](uint64_t timestamp_ns, const auto& dsg, const auto& dfg) {
        if (backend_evaluation_callback_) {
        backend_evaluation_callback_(timestamp_ns, dsg, dfg);
        }
      }));

  if (khronos_active_window_) {
    // Need to make the nodehandle private to get private topics in the aw_visualizer.
    const auto aw_nh = nh_ / "~" / RosNs::VISUALIZATION;
    khronos_active_window_->addKhronosSink(std::make_shared<ActiveWindowVisualizer>(
        config::fromContext<ActiveWindowVisualizer::Config>(RosNs::VISUALIZATION), &aw_nh));
    khronos_active_window_->addKhronosSink(ActiveWindow::KhronosSink::fromCallback(
        [this](const auto& frame_data, const auto& map, const auto& tracks) {
          if (aw_evaluation_callback_) {
          aw_evaluation_callback_(map, frame_data, tracks);
          }
        }));
  } else {
    LOG(ERROR) << "Active window is not khronos::ActiveWindow!";
  }

  if (khronos_backend_) {
    khronos_backend_->addChangeSink(
        Backend::ChangeSink::fromCallback([this](TimeStamp stamp, const Changes& changes) {
          auto msg = toMsg(changes);
          msg.header.stamp = rclcpp::Time(stamp);
          changes_pub_->publish(msg);
        }));
  } else {
    LOG(ERROR) << "Backend is not khronos::Backend!";
  }
}

bool KhronosPipeline::save(const hydra::DataDirectory& log_setup, bool save_full_state) {
  if (!log_setup) {
    LOG(WARNING) << "[Khronos Pipeline] Could not save: Invalid log setup.";
    return false;
  }

  // Save state of all modules.
  if (save_full_state) {
    // HydraRosPipeline::save calls save on all modules including backend
    HydraRosPipeline::save(log_setup);
  } else {
    // If not saving full state, we still need to save the backend for evaluation
    if (khronos_backend_) {
      khronos_backend_->saveProposedMerges(log_setup);
      // Save the backend data including the 4D map
      khronos_backend_->save(log_setup);
    }
  }

  // TODO(nathan) not threadsafe!
  auto dsg = backend_dsg_->graph->clone();
  const auto backend_path = log_setup.path("backend");

  if (khronos_active_window_) {
    // Add all objects that are currently in the active window.
    auto aw_objects = khronos_active_window_->extractObjects();
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
  }

  dsg->save(backend_path / "dsg_with_mesh", true);
  CLOG(2) << "[Khronos Pipeline] Saved " << (save_full_state ? "full state" : "evaluation DSG")
          << " to '" << log_setup.path() << "'.";
  return true;
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

void KhronosPipeline::setActiveWindowEvaluationCallback(const ActiveWindowEvaluationCallback& cb) {
  aw_evaluation_callback_ = cb;
}

void KhronosPipeline::setBackendEvaluationCallback(const BackendEvaluationCallback& callback) {
  backend_evaluation_callback_ = callback;
}

}  // namespace khronos
