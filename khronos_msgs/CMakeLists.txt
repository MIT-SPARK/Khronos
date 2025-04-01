cmake_minimum_required(VERSION 3.16)
project(khronos_msgs)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation
                                        dynamic_reconfigure)

add_message_files(
    FILES Changes.msg
          ObjectChange.msg
          SpatioTemporalVisualizerState.msg
)

add_service_files(
  FILES SpatioTemporalVisualizerSetState.srv
        SpatioTemporalVisualizerSetTimeMode.srv
        SpatioTemporalVisualizerSetup.srv
)

generate_dynamic_reconfigure_options(
  cfg/KhronosChangeDetectionVis.cfg
  cfg/KhronosEvalVis.cfg
  cfg/KhronosMeshPlugin.cfg
  cfg/KhronosSpatioTemporalVis.cfg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  message_runtime
  dynamic_reconfigure
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)
