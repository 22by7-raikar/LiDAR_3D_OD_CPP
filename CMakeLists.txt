cmake_minimum_required(VERSION 3.16)
project(lidar_od VERSION 2.0 LANGUAGES C CXX)

option(LIDAR_OD_BUILD_APPS "Build the command-line and viewer applications" ON)
option(LIDAR_OD_BUILD_VIEWER "Build the interactive PCL viewer (needs VTK)" ON)
option(LIDAR_OD_BUILD_TESTS "Build unit tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Ubuntu's VTK package (pulled in by PCL's CMake config) references MPI::MPI_C
# without finding MPI itself; finding it first avoids a configure error
# (this is also why the project enables C).
find_package(MPI QUIET COMPONENTS C)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP)

add_library(lidar_od
  src/types.cpp
  src/config.cpp
  src/io.cpp
  src/preprocess.cpp
  src/scan_layout.cpp
  src/ground/ground_segmenter.cpp
  src/ground/ransac_ground.cpp
  src/ground/region_ground.cpp
  src/ground/ray_ground.cpp
  src/ground/grid_ground.cpp
  src/cluster/clusterer.cpp
  src/cluster/euclidean_clusterer.cpp
  src/cluster/range_image_clusterer.cpp
  src/box/box_fitter.cpp
  src/tracking/kalman_filter.cpp
  src/tracking/hungarian.cpp
  src/tracking/tracker.cpp
  src/tracking/ego_motion.cpp
  src/pipeline.cpp
)
target_include_directories(lidar_od PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS})
target_compile_definitions(lidar_od PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(lidar_od PUBLIC ${PCL_LIBRARIES} PRIVATE yaml-cpp)
if(OpenMP_CXX_FOUND)
  target_link_libraries(lidar_od PRIVATE OpenMP::OpenMP_CXX)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(lidar_od PRIVATE -Wall -Wextra -Wpedantic)
endif()
set_target_properties(lidar_od PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(LIDAR_OD_BUILD_APPS)
  add_executable(lidar_od_run apps/run.cpp)
  target_link_libraries(lidar_od_run PRIVATE lidar_od)

  add_executable(lidar_od_stages apps/stages.cpp)
  target_link_libraries(lidar_od_stages PRIVATE lidar_od)

  if(LIDAR_OD_BUILD_VIEWER)
    find_package(PCL 1.10 QUIET COMPONENTS visualization)
    if(PCL_VISUALIZATION_FOUND)
      add_executable(lidar_od_viewer apps/viewer.cpp)
      target_link_libraries(lidar_od_viewer PRIVATE lidar_od ${PCL_VISUALIZATION_LIBRARIES})
    else()
      message(STATUS "PCL visualization not found: skipping lidar_od_viewer")
    endif()
  endif()
endif()

if(LIDAR_OD_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
