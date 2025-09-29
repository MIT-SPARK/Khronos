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

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/context.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "khronos_ros/visualization/spatio_temporal_visualizer.h"

struct NodeSettings {
  khronos::SpatioTemporalVisualizer::Config config;
  int glog_verbosity = 1;
  int glog_level = 0;
};

void declare_config(NodeSettings& config) {
  using namespace config;
  name("NodeSettings");
  field(config.config, "config", false);
  field(config.glog_verbosity, "glog_verbosity");
  field(config.glog_level, "glog_level");
}

int main(int argc, char** argv) {
  config::initContext(argc, argv, true);
  rclcpp::init(argc, argv);

  const auto node_settings = config::fromContext<NodeSettings>();

  // Setup logging
  FLAGS_minloglevel = node_settings.glog_level;
  FLAGS_v = node_settings.glog_verbosity;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  auto node = std::make_shared<rclcpp::Node>("spatio_temporal_visualizer");
  ianvs::NodeHandle nh(*node, "spatio_temporal_visualizer");

  // Spin.
  rclcpp::executors::MultiThreadedExecutor executor;
  {  // start visualizer scope
    khronos::SpatioTemporalVisualizer viz(node_settings.config, nh);
    executor.add_node(node);
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}
