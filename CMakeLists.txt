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
set(CMAKE_CXX_FLAGS "-fno-omit-frame-pointer -g")

#---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(PCL REQUIRED COMPONENTS common io)

find_package(benchmark QUIET)
find_package(octomap QUIET)
find_package(LZ4 QUIET)
find_package(Eigen3 QUIET)

#---- Use Address sanitizer if compiled in Debug mode ----
option(BONXAI_SANITIZE "Add address sanitizer when compiling in Debug mode" ON)

if(BONXAI_SANITIZE)
    set (CMAKE_CXX_DEBUG_FLAGS "${CMAKE_CXX_DEBUG_FLAGS}  -fsanitize=address")
    set (CMAKE_LINKER_DEBUG_FLAGS "${CMAKE_LINKER_DEBUG_FLAGS}  -fsanitize=address")
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()


####################################################
add_subdirectory(bonxai_core)
add_subdirectory(bonxai_map)


# Default behavior
find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    project(bonxai_ros)

    # Add colcon-specific configuration here
    message(STATUS "Building with colcon")

    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    include_directories(bonxai_ros/include 3rdparty)

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
