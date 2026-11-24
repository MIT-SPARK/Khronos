#include <chrono>
#include <memory>
#include <optional>
#include <thread>

#include <gtest/gtest.h>
#include <ianvs/node_init.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "khronos_ros/active_window/tf_transformation_getter.h"

namespace khronos {
namespace {

using namespace std::chrono_literals;

class TFTransformationGetterTest : public ::testing::Test {
 protected:
  TFTransformationGetter::Config makeConfig(const std::string& suffix) const {
    TFTransformationGetter::Config config;
    config.prior_frame_id = "map_" + suffix;
    config.robot_frame_id = "odom_" + suffix;
    config.tf_lookup.wait_duration_s = 0.01;
    return config;
  }

  void publish(tf2_ros::StaticTransformBroadcaster& broadcaster,
               const TFTransformationGetter::Config& config,
               double x,
               double yaw = 0.0) const {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = ianvs::CurrentNode::get()->now();
    transform.header.frame_id = config.prior_frame_id;
    transform.child_frame_id = config.robot_frame_id;
    transform.transform.translation.x = x;
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    transform.transform.rotation.w = rotation.w();
    transform.transform.rotation.z = rotation.z();
    broadcaster.sendTransform(transform);
  }

  std::optional<Eigen::Isometry3d> waitForTransform(TFTransformationGetter& getter) const {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto transform = getter.getTransformation();
      if (transform) {
        return transform;
      }

      std::this_thread::sleep_for(5ms);
    }

    return std::nullopt;
  }
};

TEST_F(TFTransformationGetterTest, ReportsInverseTransformAfterTranslationOrRotationChanges) {
  const auto config = makeConfig("thresholds");
  TFTransformationGetter getter(config, hydra::RobotConfig{});
  EXPECT_FALSE(getter.getTransformation());
  tf2_ros::StaticTransformBroadcaster broadcaster(ianvs::CurrentNode::get()->shared_from_this());

  publish(broadcaster, config, 1.0);
  auto transform = waitForTransform(getter);
  ASSERT_TRUE(transform);
  EXPECT_TRUE(transform->translation().isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_TRUE(transform->rotation().isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_FALSE(getter.getTransformation());

  publish(broadcaster, config, 2.0);
  transform = waitForTransform(getter);
  ASSERT_TRUE(transform);
  EXPECT_TRUE(transform->translation().isApprox(Eigen::Vector3d(-2.0, 0.0, 0.0)));

  // Establish zero translation so the inverse cannot move when only yaw changes.
  publish(broadcaster, config, 0.0);
  transform = waitForTransform(getter);
  ASSERT_TRUE(transform);
  EXPECT_TRUE(transform->isApprox(Eigen::Isometry3d::Identity()));

  publish(broadcaster, config, 0.0, 0.2);
  transform = waitForTransform(getter);
  ASSERT_TRUE(transform);
  const Eigen::Isometry3d expected(Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(transform->isApprox(expected));
}

TEST_F(TFTransformationGetterTest, UsesTheRunOdomFrameWhenNoOverrideIsSet) {
  const auto frames = makeConfig("run_frames");
  auto config = frames;
  config.robot_frame_id.clear();
  hydra::RobotConfig robot;
  robot.frames.odom = frames.robot_frame_id;
  TFTransformationGetter getter(config, robot);
  tf2_ros::StaticTransformBroadcaster broadcaster(ianvs::CurrentNode::get());
  publish(broadcaster, frames, 3.0);

  const auto transform = waitForTransform(getter);
  ASSERT_TRUE(transform);
  EXPECT_TRUE(transform->translation().isApprox(Eigen::Vector3d(-3.0, 0.0, 0.0)));
}

TEST_F(TFTransformationGetterTest, ShutdownInterruptsPollingWithoutTransform) {
  auto config = makeConfig("shutdown");
  config.tf_lookup.max_tries = 0;
  config.tf_lookup.wait_duration_s = 5.0;
  auto getter = std::make_unique<TFTransformationGetter>(config, hydra::RobotConfig{});
  // Let the worker enter its polling wait; no TF will ever arrive for these frames.
  std::this_thread::sleep_for(50ms);
  const auto start = std::chrono::steady_clock::now();
  getter.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
}

}  // namespace
}  // namespace khronos
