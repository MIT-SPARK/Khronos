cmake_minimum_required(VERSION 3.0.2)
project(khronos_eval)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall)
find_package(catkin_simple REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
set(PCL_FIND_QUIETLY TRUE)
find_package(PCL REQUIRED COMPONENTS io kdtree common search)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
     src/mesh_evaluator.cpp
     src/eval_visualizer.cpp
     src/cd_visualizer.cpp
     src/object_evaluator.cpp
     src/dynamic_object_evaluator.cpp
     src/pipeline_evaluator.cpp
     src/utils/io_utils.cpp
)

target_link_libraries(${PROJECT_NAME} gtsam ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

###############
# Executables #
###############

cs_add_executable(exp_pipeline app/exp_pipeline.cpp)
target_link_libraries(exp_pipeline ${PROJECT_NAME})

cs_add_executable(eval_visualizer_node app/eval_visualizer_node.cpp)
target_link_libraries(eval_visualizer_node ${PROJECT_NAME})

cs_add_executable(cd_visualizer_node app/cd_visualizer_node.cpp)
target_link_libraries(cd_visualizer_node ${PROJECT_NAME})

##########
# Export #
##########

cs_install()
cs_export()
