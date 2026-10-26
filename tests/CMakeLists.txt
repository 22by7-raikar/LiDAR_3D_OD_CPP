find_package(GTest REQUIRED)

add_executable(lidar_od_tests
  test_kdtree.cpp
  test_ground.cpp
  test_cluster.cpp
  test_box.cpp
  test_tracking.cpp
  test_config.cpp
)
target_link_libraries(lidar_od_tests PRIVATE lidar_od GTest::gtest GTest::gtest_main)
target_compile_definitions(lidar_od_tests PRIVATE
  LIDAR_OD_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(GoogleTest)
gtest_discover_tests(lidar_od_tests)
