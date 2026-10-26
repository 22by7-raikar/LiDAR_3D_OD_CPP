cmake_minimum_required(VERSION 3.16)
project(lidar_od_ros LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Build the core library from the repository root (two levels up).
set(LIDAR_OD_BUILD_APPS OFF CACHE BOOL "" FORCE)
set(LIDAR_OD_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../.. ${CMAKE_CURRENT_BINARY_DIR}/lidar_od_core)

add_executable(lidar_od_node src/lidar_od_node.cpp)
target_link_libraries(lidar_od_node lidar_od)
target_compile_features(lidar_od_node PRIVATE cxx_std_17)
ament_target_dependencies(lidar_od_node rclcpp sensor_msgs visualization_msgs geometry_msgs pcl_conversions)

install(TARGETS lidar_od_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../config/default.yaml DESTINATION share/${PROJECT_NAME}/config)

ament_package()
