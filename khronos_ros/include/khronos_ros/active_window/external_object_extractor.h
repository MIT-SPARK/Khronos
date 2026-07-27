#pragma once
#include <set>

#include <ianvs/node_handle.h>
#include <khronos/active_window/object_extraction/object_extractor.h>
#include <khronos_msgs/srv/extract_implicit_object.hpp>
#include <spark_dsg/node_attributes.h>

namespace hydra_multi_system {

using spark_dsg::KhronosObjectAttributes;

class CrispObjectExtractor : public khronos::ObjectExtractor {
 public:
  using InferenceSrv = khronos_msgs::srv::ExtractImplicitObject;

  struct Config {
    float min_object_allocation_confidence = 0.5f;
    std::set<int32_t> excluded_labels;
  } const config;

  explicit CrispObjectExtractor(Config& config);
  virtual ~CrispObjectExtractor() = default;

  virtual KhronosObjectAttributes::Ptr extractObject(const khronos::Track& track,
                                                     const khronos::FrameDataBuffer& buffer);

 private:
  ianvs::NodeHandle nh_;
  rclcpp::Client<InferenceSrv>::SharedPtr client_;
};

}  // namespace hydra_multi_system
