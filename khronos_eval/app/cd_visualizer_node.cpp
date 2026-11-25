#include <config_utilities/external_registry.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/settings.h>
#include <hydra/common/run_settings.h>
#include <hydra/utils/logging.h>
#include <hydra_visualizer/visualizer_node.h>
#include <rclcpp/rclcpp.hpp>

#include "khronos_eval/cd_visualizer.h"

struct NodeSettings {
  std::vector<std::string> external_library_paths;

  std::string log_level = "info";
};

void declare_config(NodeSettings& config) {
  using namespace config;
  name("NodeSettings");
  field(config.external_library_paths, "external_library_paths");

  field(config.log_level, "log_level");
}

int main(int argc, char** argv) {
  config::initContext(argc, argv, true);
  config::setConfigSettingsFromContext();
  rclcpp::init(argc, argv);

  const auto node_settings = config::fromContext<NodeSettings>();

  hydra::logging::setLogLevel(node_settings.log_level);
  hydra::RunSettings::init(config::fromContext<hydra::RunSettings::Config>());

  [[maybe_unused]] const auto plugins =
      config::loadExternalFactories(node_settings.external_library_paths);

  auto node = std::make_shared<rclcpp::Node>("cd_visualizer_node");
  ianvs::NodeHandle nh(*node);

  const auto dsg_config = config::fromContext<hydra::DsgVisualizer::Config>();
  auto graph_wrapper = dsg_config.graph.create();
  auto stamped_graph = graph_wrapper->get();
  if (!stamped_graph) {
    HYDRA_LOG(ERROR, "No graph available");
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
