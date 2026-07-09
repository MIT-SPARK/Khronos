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
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(spark_dsg REQUIRED)
find_package(kimera_pgmo REQUIRED)
find_package(hydra REQUIRED)
find_package(small_gicp REQUIRED)
find_package(CLI11 REQUIRED)

include(GNUInstallDirs)

#############
# Libraries #
#############

add_library(${PROJECT_NAME}_registration
    src/utils/icp_registration_utils.cpp
)
target_include_directories(${PROJECT_NAME}_registration
    PUBLIC $<INSTALL_INTERFACE:include>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${PROJECT_NAME}_registration
    PRIVATE small_gicp::small_gicp
)
set_property(TARGET ${PROJECT_NAME}_registration PROPERTY POSITION_INDEPENDENT_CODE ON)
add_library(khronos::${PROJECT_NAME}_registration ALIAS ${PROJECT_NAME}_registration)

add_library(${PROJECT_NAME}
        src/active_window/active_window.cpp
        src/active_window/change_detection/active_window_change_detector.cpp
        src/active_window/change_detection/transformation_getter.cpp
        src/active_window/data/frame_data.cpp
        src/active_window/data/frame_data_buffer.cpp
        src/active_window/data/track.cpp
        src/active_window/integration/object_integrator.cpp
        src/active_window/integration/tracking_integrator.cpp
        src/active_window/motion_detection/free_space_motion_detector.cpp
        src/active_window/object_detection/connected_semantics.cpp
        src/active_window/object_detection/instance_forwarding.cpp
        src/active_window/object_extraction/mesh_object_extractor.cpp
        src/active_window/object_extraction/object_worker_pool.cpp
        src/active_window/track_saver_sink.cpp
        src/active_window/tracking/external_tracker.cpp
        src/active_window/tracking/max_iou_tracker.cpp
        src/active_window/tracking/semantic_matching.cpp
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
        src/utils/json_utils.cpp
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
        ${PROJECT_NAME}_registration
        PRIVATE ${tf2_eigen_LIBRARIES}
)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
add_library(khronos::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()
###############
# Executables #
###############

# Standalone round-trip check for Track::save/Track::load (see tracker_debug.md Step 1).
add_executable(test_track_serialization app/test_track_serialization.cpp)
target_link_libraries(test_track_serialization PRIVATE ${PROJECT_NAME})
install(TARGETS test_track_serialization RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Replays a saved track's observations through MaxIoUTracker against another loaded track (see
# tracker_debug.md Step 2).
add_executable(test_track_association app/test_track_association.cpp)
target_link_libraries(test_track_association PRIVATE ${PROJECT_NAME} CLI11::CLI11)
install(TARGETS test_track_association RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Standalone round-trip check for FrameData::save/FrameData::load and
# saveCameraIntrinsics/loadCameraIntrinsics (see tracker_debug.md Step 3).
add_executable(test_frame_data_serialization app/test_frame_data_serialization.cpp)
target_link_libraries(test_frame_data_serialization PRIVATE ${PROJECT_NAME})
install(TARGETS test_frame_data_serialization RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Replays an entire saved run through a fresh MaxIoUTracker, with active-window eviction
# simulation, for parameter tuning (see tracker_debug.md Step 4).
add_executable(full_run_tracker_replay app/full_run_tracker_replay.cpp)
target_link_libraries(full_run_tracker_replay PRIVATE ${PROJECT_NAME} CLI11::CLI11)
install(TARGETS full_run_tracker_replay RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

##########
# Export #
##########

include(KhronosInstall)
