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

set(PCL_FIND_QUIETLY TRUE)
find_package(PCL REQUIRED COMPONENTS io kdtree common search)
find_package(gflags REQUIRED)

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
  src/utils/io_utils.cpp)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} PUBLIC khronos_ros)

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

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(khronos_ros)
ament_package()
