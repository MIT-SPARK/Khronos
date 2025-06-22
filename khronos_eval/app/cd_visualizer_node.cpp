#include <config_utilities/external_registry.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/settings.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <hydra_visualizer/visualizer_node.h>
#include <rclcpp/rclcpp.hpp>

#include "khronos_eval/cd_visualizer.h"

struct ExternalPluginConfig {
  bool allow_plugins = true;
  bool verbose_plugins = false;
  bool trace_plugin_allocations = false;
  std::vector<std::string> paths;
};

struct NodeSettings {
  ExternalPluginConfig external_plugins;
  int glog_verbosity = 1;
  int glog_level = 0;
};

void declare_config(ExternalPluginConfig& config) {
  using namespace config;
  name("ExternalPluginConfig");
  field(config.allow_plugins, "allow_plugins");
  field(config.verbose_plugins, "verbose_plugins");
  field(config.trace_plugin_allocations, "trace_plugin_allocations");
  field(config.paths, "paths");
}

void declare_config(NodeSettings& config) {
  using namespace config;
  name("NodeSettings");
  field(config.external_plugins, "external_plugins");
  field(config.glog_verbosity, "glog_verbosity");
  field(config.glog_level, "glog_level");
}

int main(int argc, char** argv) {
  config::initContext(argc, argv, true);
  rclcpp::init(argc, argv);

  const auto node_settings = config::fromContext<NodeSettings>();

  FLAGS_minloglevel = node_settings.glog_level;
  FLAGS_v = node_settings.glog_verbosity;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  auto& settings = config::Settings();
  settings.allow_external_libraries = node_settings.external_plugins.allow_plugins;
  settings.verbose_external_load = node_settings.external_plugins.verbose_plugins;
  settings.print_external_allocations = node_settings.external_plugins.trace_plugin_allocations;
  [[maybe_unused]] const auto plugins =
      config::loadExternalFactories(node_settings.external_plugins.paths);

  auto node = std::make_shared<rclcpp::Node>("cd_visualizer_node");
  ianvs::NodeHandle nh(*node);

  const auto dsg_config = config::fromContext<hydra::DsgVisualizer::Config>();
  auto graph_wrapper = dsg_config.graph.create();
  auto stamped_graph = graph_wrapper->get();
  if (!stamped_graph) {
    LOG(ERROR) << "No graph available";
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  {  // start visualizer scope
    auto dsg = stamped_graph.graph;
    khronos::ChangeDetectionVisualizer cd_vis(
        config::fromContext<khronos::ChangeDetectionVisualizer::StaticConfig>(), nh, dsg);
    cd_vis.start();
    executor.add_node(node);
    executor.spin();
  }  // end visualizer scope

  rclcpp::shutdown();
  return 0;
}
