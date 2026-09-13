find_package(GTest REQUIRED)

include(GoogleTest)
enable_testing()

add_executable(
  test_${PROJECT_NAME}
  main.cpp
  active_window/data/test_frame_data_serialization.cpp
  active_window/data/test_track_serialization.cpp
  active_window/object_detection/test_instance_forwarding.cpp)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} GTest::gtest_main)
gtest_add_tests(TARGET test_${PROJECT_NAME})
