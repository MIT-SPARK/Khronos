cmake_minimum_required(VERSION 3.0.2)
project(khronos_eval)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall)
find_package(catkin_simple REQUIRED)
find_package(GTSAM REQUIRED)
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
     src/ground_truth/tesse_ground_truth_builder.cpp
     src/ground_truth/tesse_dynamic_object_ground_truth_builder.cpp
     src/ground_truth/real_dynamic_object_ground_truth_builder.cpp
     src/utils/io_utils.cpp
)

target_link_libraries(${PROJECT_NAME} gtsam ${PCL_LIBRARIES})

###############
# Executables #
###############

cs_add_executable(evaluate_mesh app/evaluate_mesh.cpp)
target_link_libraries(evaluate_mesh ${PROJECT_NAME})

cs_add_executable(evaluate_meshes app/evaluate_meshes.cpp)
target_link_libraries(evaluate_meshes ${PROJECT_NAME})

cs_add_executable(exp_evaluate_mesh app/exp_evaluate_mesh.cpp)
target_link_libraries(exp_evaluate_mesh ${PROJECT_NAME})

cs_add_executable(exp_reconcile app/exp_reconcile.cpp)
target_link_libraries(exp_reconcile ${PROJECT_NAME})

cs_add_executable(exp_change_detection app/exp_change_detection.cpp)
target_link_libraries(exp_change_detection ${PROJECT_NAME})

cs_add_executable(exp_pipeline app/exp_pipeline.cpp)
target_link_libraries(exp_pipeline ${PROJECT_NAME})

cs_add_executable(create_gt app/create_gt.cpp)
target_link_libraries(create_gt ${PROJECT_NAME})

cs_add_executable(create_dynamic_object_gt app/create_dynamic_object_gt.cpp)
target_link_libraries(create_dynamic_object_gt ${PROJECT_NAME})

cs_add_executable(create_dynamic_object_gt_real_data app/create_dynamic_object_gt_real_data.cpp)
target_link_libraries(create_dynamic_object_gt_real_data ${PROJECT_NAME})

cs_add_executable(eval_visualizer_node app/eval_visualizer_node.cpp)
target_link_libraries(eval_visualizer_node ${PROJECT_NAME})

cs_add_executable(cd_visualizer_node app/cd_visualizer_node.cpp)
target_link_libraries(cd_visualizer_node ${PROJECT_NAME})

cs_add_executable(hydra_converter app/hydra_converter.cpp)
target_link_libraries(hydra_converter ${PROJECT_NAME})

##########
# Export #
##########

cs_install()
cs_export()
