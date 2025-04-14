cmake_minimum_required(VERSION 3.0.2)
project(khronos_eval)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-Wall)

find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
set(PCL_FIND_QUIETLY TRUE)
find_package(PCL REQUIRED COMPONENTS io kdtree common search)
find_package(catkin REQUIRED COMPONENTS khronos_ros roscpp)
find_package(gflags REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  khronos_ros
  roscpp
  DEPENDS
  PCL
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME})

# ##############################################################################
# Libraries #
# ##############################################################################

add_library(${PROJECT_NAME}
     src/mesh_evaluator.cpp
     src/eval_visualizer.cpp
     src/cd_visualizer.cpp
     src/object_evaluator.cpp
     src/dynamic_object_evaluator.cpp
     src/pipeline_evaluator.cpp
     src/utils/io_utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} gtsam ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

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
          exp_pipeline
          eval_visualizer_node
          cd_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
