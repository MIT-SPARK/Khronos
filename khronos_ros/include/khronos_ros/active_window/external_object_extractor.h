#pragma once
#include <set>

#include <hydra_multi_system_msgs/srv/crisp_object_inference.hpp>
#include <ianvs/node_handle.h>
#include <khronos/active_window/object_extraction/object_extractor.h>
#include <spark_dsg/node_attributes.h>

namespace hydra_multi_system {

using spark_dsg::KhronosObjectAttributes;

class CrispObjectExtractor : public khronos::ObjectExtractor {
 public:
  using InferenceSrv = hydra_multi_system_msgs::srv::CrispObjectInference;

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
