cmake_minimum_required(VERSION 3.5)
project(Bonxai)


if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

#---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

#---- Use C++17 ----
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#---- Use Address sanitixer if compiled in Debug mode ----
set (CMAKE_CXX_DEBUG_FLAGS "${CMAKE_CXX_DEBUG_FLAGS}  -fsanitize=address")
set (CMAKE_LINKER_DEBUG_FLAGS "${CMAKE_LINKER_DEBUG_FLAGS}  -fsanitize=address")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()

find_package(PCL REQUIRED COMPONENTS common io)

set(bonxai_map_SRC
    include/bonxai_map/pointcloud.hpp
    src/pointcloud.cpp
    include/bonxai_map/pointcloud.hpp
    src/pcl_utils.cpp
    )

add_library(bonxai_map ${bonxai_map_SRC})
target_link_libraries(bonxai_map PRIVATE ${PCL_LIBRARIES})

target_compile_features(bonxai_map PRIVATE cxx_std_17)
target_include_directories(bonxai_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

####################################################
find_package(benchmark QUIET)

if(NOT benchmark_FOUND)
    message("Google Benchmark not installed. Skipping benchmarks")
endif()

if(benchmark_FOUND AND PCL_FOUND)
    message(STATUS Building the benchmarks)
    add_subdirectory(benchmark)
endif()

####################################################
add_executable(map_playground test/bonxai_map_playground.cpp)
target_link_libraries(map_playground bonxai_map)
target_include_directories(map_playground PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

