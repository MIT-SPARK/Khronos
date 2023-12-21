cmake_minimum_required(VERSION 3.0.2)
project(khronos_ros VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra)

option(BUILD_SHARED_LIBS "Build shared libs" ON)

find_package(catkin_simple REQUIRED)
find_package(GTSAM REQUIRED)
find_package(hydra REQUIRED)
find_package(spark_dsg REQUIRED)
find_package(khronos REQUIRED)
find_package(gflags REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/utils/ros_conversions.cpp
        src/experiments/experiment_logger.cpp
        src/experiments/experiment_manager.cpp
        src/experiments/experiment_directory.cpp
        src/input/openset_subscriber.cpp
        src/khronos_pipeline.cpp
        src/visualization/active_window_visualizer.cpp
        src/visualization/khronos_mesh_visualizer.cpp
        src/visualization/visualization_utils.cpp
        src/visualization/spatio_temporal_visualizer.cpp
        )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} gtsam hydra::hydra spark_dsg::spark_dsg khronos::khronos)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


###############
# Executables #
###############

add_executable(khronos_node app/khronos_node.cpp)
target_link_libraries(khronos_node ${PROJECT_NAME} ${gflags_LIBRARIES})

add_executable(spatio_temporal_visualizer_node app/spatio_temporal_visualizer_node.cpp)
target_link_libraries(spatio_temporal_visualizer_node ${PROJECT_NAME} ${gflags_LIBRARIES})

##########
# Export #
##########

cs_install()
cs_export()
