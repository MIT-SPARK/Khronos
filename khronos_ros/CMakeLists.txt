cmake_minimum_required(VERSION 3.22.1)
project(khronos_ros VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

option(BUILD_SHARED_LIBS "Build shared libs" ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(gflags REQUIRED)
find_package(hydra_ros REQUIRED)
find_package(khronos REQUIRED)
find_package(khronos_msgs REQUIRED)

# ##############################################################################
# Libraries #
# ##############################################################################

add_library(
  ${PROJECT_NAME}
  src/utils/ros_conversions.cpp
  src/experiments/experiment_logger.cpp
  src/experiments/experiment_manager.cpp
  src/experiments/experiment_directory.cpp
  src/khronos_pipeline.cpp
  src/visualization/active_window_visualizer.cpp
  src/visualization/visualization_utils.cpp
  src/visualization/spatio_temporal_visualizer.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC khronos::khronos hydra_ros::hydra_ros PRIVATE cv_bridge::cv_bridge)
ament_target_dependencies(${PROJECT_NAME} PUBLIC khronos_msgs)

# ##############################################################################
# Executables #
# ##############################################################################

add_executable(khronos_node app/khronos_node.cpp)
target_link_libraries(khronos_node ${PROJECT_NAME} ${gflags_LIBRARIES})

add_executable(spatio_temporal_visualizer_node app/spatio_temporal_visualizer_node.cpp)
target_link_libraries(spatio_temporal_visualizer_node ${PROJECT_NAME} ${gflags_LIBRARIES})

# ##############################################################################
# Export #
# ##############################################################################

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS khronos_node spatio_temporal_visualizer_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  src/visualization/spatio_temporal_visualizer_gui.py
  scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(gflags hydra_ros khronos khronos_msgs)
ament_package()
