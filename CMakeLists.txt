cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(LiDAR_3D_Detection VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

# Find required packages
find_package(PCL 1.11 REQUIRED)

# Remove problematic library
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# Create executable
add_executable(environment 
    src/environment.cpp 
    src/render/render.cpp 
    src/processPointClouds.cpp
)

# Create tracking executable for bicyclist detection challenge
add_executable(environment_tracking
    src/environment_tracking.cpp
    src/render/render.cpp
    src/processPointClouds.cpp
    src/TrackerBase.cpp
    src/SimpleTracker.cpp
    src/KalmanTracker.cpp
)

# Target properties for main executable
target_include_directories(environment PRIVATE 
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_directories(environment PRIVATE 
    ${PCL_LIBRARY_DIRS}
)

target_compile_definitions(environment PRIVATE 
    ${PCL_DEFINITIONS}
)

target_link_libraries(environment PRIVATE 
    ${PCL_LIBRARIES}
)

# Target properties for tracking executable
target_include_directories(environment_tracking PRIVATE 
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_directories(environment_tracking PRIVATE 
    ${PCL_LIBRARY_DIRS}
)

target_compile_definitions(environment_tracking PRIVATE 
    ${PCL_DEFINITIONS}
)

target_link_libraries(environment_tracking PRIVATE 
    ${PCL_LIBRARIES}
)

# Enable more warnings for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(environment PRIVATE 
        -Wall -Wextra -Wpedantic
    )
    target_compile_options(environment_tracking PRIVATE 
        -Wall -Wextra -Wpedantic
    )
endif()