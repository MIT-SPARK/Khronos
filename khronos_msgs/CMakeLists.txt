cmake_minimum_required(VERSION 3.16)
project(khronos_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/Changes.msg
  msg/ObjectChange.msg
  msg/SpatioTemporalVisualizerState.msg
  srv/SpatioTemporalVisualizerSetState.srv
  srv/SpatioTemporalVisualizerSetTimeMode.srv
  srv/SpatioTemporalVisualizerSetup.srv
  DEPENDENCIES
  builtin_interfaces
  std_msgs
)

#generate_dynamic_reconfigure_options(
#cfg/KhronosChangeDetectionVis.cfg
#cfg/KhronosEvalVis.cfg
#cfg/KhronosMeshPlugin.cfg
#cfg/KhronosSpatioTemporalVis.cfg
#)

ament_package()
