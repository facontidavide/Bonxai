cmake_minimum_required(VERSION 3.10)
project(Bonxai)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")

  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)

####### Add subdirectory cmake and dependencies ######
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)

if(CMAKE_COMPILER_IS_GNUCXX AND CODE_COVERAGE)
  message(STATUS "Configuring for code coverage build...")
  if (NOT CMAKE_BUILD_TYPE MATCHES Debug)
    message(WARNING "Non debug build may produce odd results for code coverage")
  endif()

  include(CodeCoverage)
  append_coverage_compiler_flags()
endif()

include(FetchContent)
include(cmake/CPM.cmake)

find_package(benchmark QUIET)
if(NOT benchmark_FOUND)
  message(STATUS "Benchmark not found. Fetching it from GitHub")
  CPMAddPackage(
        NAME benchmark
        GITHUB_REPOSITORY google/benchmark
        VERSION 1.9.1
        OPTIONS  "BENCHMARK_ENABLE_TESTING OFF" "CMAKE_BUILD_TYPE Release"
  )
endif()

find_package(LZ4 QUIET)
if(NOT LZ4_FOUND)
  message(STATUS "Benchmark not found. Fetching it from GitHub")
  CPMAddPackage(
    NAME lz4
    GITHUB_REPOSITORY lz4/lz4
    GIT_TAG v1.10.0
    OPTIONS "CMAKE_BUILD_TYPE Release"
  )
endif()

CPMAddPackage(
    NAME cxxopts
    GITHUB_REPOSITORY jarro2783/cxxopts
    GIT_TAG v3.2.0
)


# the following dependencies are larget and we expect them to be installed in the system
find_package(octomap QUIET)
find_package(Eigen3 QUIET)
find_package(PCL QUIET COMPONENTS common io filters)

####### Use Address sanitizer if compiled in Debug mode #######

option(BONXAI_SANITIZE "Add address sanitizer when compiling in Debug mode" OFF)

if(BONXAI_SANITIZE)
    set (CMAKE_CXX_DEBUG_FLAGS "${CMAKE_CXX_DEBUG_FLAGS}  -fsanitize=address")
    set (CMAKE_LINKER_DEBUG_FLAGS "${CMAKE_LINKER_DEBUG_FLAGS}  -fsanitize=address")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()

option(BONXAI_BUILD_TESTS "Build tests" ON)

if(BONXAI_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

####################################################
add_subdirectory(bonxai_core)
add_subdirectory(examples)

if(Eigen_FOUND AND PCL_FOUND )
  add_subdirectory(bonxai_map)
endif()

# Default behavior
find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    project(bonxai_ros)

    # Add colcon-specific configuration here
    message(STATUS "Building with colcon")

    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    ament_auto_add_library(bonxai_server SHARED
      bonxai_ros/src/bonxai_server.cpp
    )

    target_link_libraries(bonxai_server
      bonxai_map
      ${PCL_LIBRARIES}
    )

    rclcpp_components_register_node(bonxai_server
      PLUGIN "bonxai_server::BonxaiServer"
      EXECUTABLE bonxai_server_node
    )

    ament_auto_package(
      INSTALL_TO_SHARE
      bonxai_ros/launch
      bonxai_ros/rviz
      bonxai_ros/params
    )

else()

  message(STATUS "Building with cmake")

endif()
