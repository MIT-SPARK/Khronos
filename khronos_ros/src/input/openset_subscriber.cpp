#include "khronos_ros/input/openset_subscriber.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>

namespace khronos {

struct OpenSetSubscriber::Impl {
  Impl(const ros::NodeHandle& _nh, const std::string& topic, uint32_t queue_size)
      : nh(_nh), sub(nh, topic, queue_size) {}

  ros::NodeHandle nh;
  message_filters::Subscriber<MsgType> sub;
};

OpenSetSubscriber::OpenSetSubscriber() = default;

OpenSetSubscriber::OpenSetSubscriber(const ros::NodeHandle& nh, uint32_t queue_size)
    : impl_(std::make_shared<Impl>(nh, "semantic/image_raw", queue_size)) {}

OpenSetSubscriber::~OpenSetSubscriber() = default;

OpenSetSubscriber::Filter& OpenSetSubscriber::getFilter() const {
  return CHECK_NOTNULL(impl_)->sub;
}

void OpenSetSubscriber::fillInput(const MsgType& msg, hydra::ImageInputPacket& packet) {
  try {
    packet.labels = cv_bridge::toCvCopy(msg.image)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert depth image: " << e.what();
  }

  CHECK_EQ(msg.mask_ids.size(), msg.features.size());
  for (size_t i = 0; i < msg.mask_ids.size(); ++i) {
    const auto& vec = msg.features[i].data;
    packet.label_features.emplace(msg.mask_ids[i],
                                  Eigen::Map<const hydra::FeatureVector>(vec.data(), vec.size()));
  }
}

void declare_config(OpenSetImageReceiver::Config& config) {
  using namespace config;
  name("OpenSetImageReceiver::Config");
  base<hydra::RosDataReceiver::Config>(config);
}

OpenSetImageReceiver::OpenSetImageReceiver(const Config& config, const std::string& sensor_name)
    : ImageReceiverImpl<OpenSetSubscriber>(config, sensor_name) {}

namespace {
static const auto registration =
    config::RegistrationWithConfig<hydra::DataReceiver,
                                   OpenSetImageReceiver,
                                   OpenSetImageReceiver::Config,
                                   std::string>("OpenSetImageReceiver");
}

}  // namespace khronos
