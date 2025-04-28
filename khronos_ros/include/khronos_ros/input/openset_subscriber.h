#pragma once
#include <hydra_ros/input/image_receiver.h>
#include <message_filters/subscriber.h>
#include <semantic_inference_msgs/msg/feature_image.hpp>

namespace khronos {

class OpenSetSubscriber {
 public:
  using MsgType = semantic_inference_msgs::msg::FeatureImage;
  using Filter = message_filters::SimpleFilter<MsgType>;

  OpenSetSubscriber();
  explicit OpenSetSubscriber(const ros::NodeHandle& nh, uint32_t queue_size = 1);
  ~OpenSetSubscriber();

  Filter& getFilter() const;
  void fillInput(const MsgType& msg, hydra::ImageInputPacket& packet);

 protected:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

class OpenSetImageReceiver : public hydra::ImageReceiverImpl<OpenSetSubscriber> {
 public:
  struct Config : RosDataReceiver::Config {};
  OpenSetImageReceiver(const Config& config, const std::string& sensor_name);
  virtual ~OpenSetImageReceiver() = default;
};

void declare_config(OpenSetImageReceiver::Config& config);

}  // namespace khronos
