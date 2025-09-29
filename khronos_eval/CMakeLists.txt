cmake_minimum_required(VERSION 3.22.1)
project(khronos_eval)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

option(BUILD_SHARED_LIBS "Build shared libs" ON)

find_package(ament_cmake REQUIRED)
find_package(khronos_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(PCL_FIND_QUIETLY TRUE)
find_package(PCL REQUIRED COMPONENTS io kdtree common search)
find_package(gflags REQUIRED)
find_package(OpenCV REQUIRED)

# ##############################################################################
# Libraries #
# ##############################################################################

add_library(
  ${PROJECT_NAME}
  src/mesh_evaluator.cpp
  src/eval_visualizer.cpp
  src/cd_visualizer.cpp
  src/object_evaluator.cpp
  src/dynamic_object_evaluator.cpp
  src/pipeline_evaluator.cpp
  src/utils/io_utils.cpp
  src/ground_truth/gt_consolidator.cpp
  src/ground_truth/tesse_ground_truth_builder.cpp
  src/ground_truth/tesse_dynamic_object_ground_truth_builder.cpp
  src/ground_truth/real_dynamic_object_ground_truth_builder.cpp)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME} PUBLIC 
  khronos_ros 
  rosbag2_cpp 
  rosbag2_storage
  tf2_geometry_msgs
  cv_bridge)

# ##############################################################################
# Executables #
# ##############################################################################

add_executable(exp_pipeline app/exp_pipeline.cpp)
target_link_libraries(exp_pipeline ${PROJECT_NAME} ${gflags_LIBRARIES})

add_executable(eval_visualizer_node app/eval_visualizer_node.cpp)
target_link_libraries(eval_visualizer_node ${PROJECT_NAME} ${gflags_LIBRARIES})

add_executable(cd_visualizer_node app/cd_visualizer_node.cpp)
target_link_libraries(cd_visualizer_node ${PROJECT_NAME} ${gflags_LIBRARIES})

# ##############################################################################
# Export #
# ##############################################################################

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS exp_pipeline eval_visualizer_node cd_visualizer_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/evaluate_pipeline.sh DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  khronos_ros
  rosbag2_cpp
  rosbag2_storage
  tf2_geometry_msgs
  cv_bridge)
ament_package()
