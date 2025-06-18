cmake_minimum_required(VERSION 3.22.1)
project(khronos VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_compile_options(-Wall -Wextra)

option(BUILD_SHARED_LIBS "Build shared libs" ON)

find_package(config_utilities REQUIRED)
find_package(spatial_hash REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pose_graph_tools REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(spark_dsg REQUIRED)
find_package(kimera_pgmo REQUIRED)
find_package(hydra REQUIRED)

include(GNUInstallDirs)

#############
# Libraries #
#############

add_library(${PROJECT_NAME}
        src/active_window/active_window.cpp
        src/active_window/data/frame_data_buffer.cpp
        src/active_window/data/track.cpp
        src/active_window/integration/object_integrator.cpp
        src/active_window/integration/tracking_integrator.cpp
        src/active_window/motion_detection/free_space_motion_detector.cpp
        src/active_window/object_detection/connected_semantics.cpp
        src/active_window/object_detection/instance_forwarding.cpp
        src/active_window/object_extraction/mesh_object_extractor.cpp
        src/active_window/object_extraction/object_worker_pool.cpp
        src/active_window/tracking/external_tracker.cpp
        src/active_window/tracking/max_iou_tracker.cpp
        src/backend/backend.cpp
        src/backend/change_detection/background/ray_background_change_detector.cpp
        src/backend/change_detection/objects/ray_object_change_detector.cpp
        src/backend/change_detection/ray_change_detector.cpp
        src/backend/change_detection/ray_verificator.cpp
        src/backend/change_detection/sequential_change_detector.cpp
        src/backend/change_state.cpp
        src/backend/reconciliation/mesh/change_merger.cpp
        src/backend/reconciliation/mesh/mesh_merger.cpp
        src/backend/reconciliation/mesh/overwrite_mesh.cpp
        src/backend/reconciliation/reconciler.cpp
        src/backend/update_khronos_objects_functor.cpp
        src/spatio_temporal_map/spatio_temporal_map.cpp
        src/spatio_temporal_map/incremental_mesh.cpp
        src/utils/geometry_utils.cpp
        src/utils/khronos_attribute_utils.cpp
        src/utils/output_file_utils.cpp
        )
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${OpenCV_INCLUDE_DIRS}
        ${kimera_pgmo_INCLUDE_DIRS}
        ${hydra_INCLUDE_DIRS}
        PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${tf2_eigen_INCLUE_DIRS}
)
target_link_libraries(
${PROJECT_NAME}
        PUBLIC ${OpenCV_LIBRARIES}
        config_utilities::config_utilities
        spark_dsg::spark_dsg
        gtsam
        spatial_hash::spatial_hash
        ${kimera_pgmo_LIBRARIES}
        ${hydra_LIBRARIES}
        PRIVATE ${tf2_eigen_LIBRARIES}
)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
add_library(khronos::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

##########
# Export #
##########

include(KhronosInstall)
