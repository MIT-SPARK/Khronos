#include <hydra/utils/logging.h>
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

#include <config_utilities/parsing/context.h>
#include <ianvs/node_init.h>
#include <ianvs/ros_log_sink.h>

#include "khronos_ros/experiments/experiment_manager.h"
#include "khronos_ros/khronos_pipeline.h"
#include "khronos_ros/utils/ros_namespaces.h"

namespace khronos {

struct NodeSettings {
  std::string log_level = "info";
  bool forward_logs_to_ros = false;
};

void declare_config(NodeSettings& config) {
  config::field(config.log_level, "log_level");
  config::field(config.forward_logs_to_ros, "forward_logs_to_ros");
}

}  // namespace khronos

int main(int argc, char** argv) {
  config::initContext(argc, argv, true);
  config::setConfigSettingsFromContext();
  [[maybe_unused]] const auto node = ianvs::init_node(argc, argv, "khronos_node");
  auto nh = ianvs::NodeHandle::this_node();

  const auto logging = config::fromContext<khronos::NodeSettings>();
  hydra::logging::setLogLevel(logging.log_level);
  if (logging.forward_logs_to_ros) {
    hydra::logging::setSinks({ianvs::makeRosSink(nh.logger())});
  }

  // Allocate the run directory before modules start logging.
  const auto experiment_config =
      config::fromContext<khronos::ExperimentManager::Config>(khronos::RosNs::EXPERIMENT);
  auto run_config = config::fromContext<hydra::RunContext::Config>();
  if (experiment_config.output_dir.empty()) {
    run_config.output.overwrite = experiment_config.overwrite;
    run_config.output.use_timestamp = !experiment_config.overwrite;
  }

  const auto context = std::make_shared<hydra::RunContext>(run_config);
  const auto khronos_config = config::fromContext<khronos::KhronosPipeline::Config>();
  auto khronos = std::make_shared<khronos::KhronosPipeline>(khronos_config, context, nh);
  khronos::ExperimentManager manager(experiment_config, nh, khronos, context->logs);

  // Run the experiment.
  manager.run();
  hydra::logging::logger()->flush();
  if (logging.forward_logs_to_ros) {
    hydra::logging::setLogToStderr();
  }

  return 0;
}
