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

find_package(benchmark QUIET)
find_package(PCL QUIET COMPONENTS common io)

if(NOT benchmark_FOUND)
    message("Google Benchmark not installed. Skipping benchmarks")
endif()

if(NOT PCL_FOUND)
    message("PCL not installed. Skipping benchmarks")
endif()

if(benchmark_FOUND AND PCL_FOUND)
    message(STATUS Building the benchmarks)
    add_subdirectory(benchmark)
endif()
