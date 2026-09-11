cmake_minimum_required(VERSION 3.22)
project(ranger_lio VERSION 0.1.0 LANGUAGES CXX
        DESCRIPTION "LiDAR-inertial odometry in C++17 with a GTSAM pose-graph backend")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(RANGER_BUILD_TESTS "Build unit tests (GTest)" ON)
option(RANGER_BUILD_APPS  "Build offline apps (may link rosbag2)" ON)

find_package(Eigen3 3.3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(small_gicp REQUIRED)
find_package(OpenMP)

# ---- core library: never links ROS -----------------------------------------
add_library(ranger_lio STATIC
  src/so3.cpp
  src/se3.cpp
)
add_library(ranger_lio::ranger_lio ALIAS ranger_lio)
target_include_directories(ranger_lio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ranger_lio PUBLIC Eigen3::Eigen gtsam small_gicp::small_gicp)
if(OpenMP_CXX_FOUND)
  target_link_libraries(ranger_lio PUBLIC OpenMP::OpenMP_CXX)
endif()
target_compile_options(ranger_lio PRIVATE -Wall -Wextra -Wpedantic)

# ---- apps -------------------------------------------------------------------
if(RANGER_BUILD_APPS)
  add_subdirectory(apps)
endif()

# ---- tests ------------------------------------------------------------------
if(RANGER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
