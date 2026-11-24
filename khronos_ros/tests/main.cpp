#include <gtest/gtest.h>
#include <ianvs/node_init.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  const auto node = ianvs::init_node(argc, argv, "khronos_ros_tests");
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
