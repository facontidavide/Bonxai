cmake_minimum_required(VERSION 3.10)
project(bonxai_server)

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

#---- Use Address sanitizer if compiled in Debug mode ----
option(BONXAI_SANITIZE "Add address sanitizer when compiling in Debug mode" ON)

if(BONXAI_SANITIZE)
    set (CMAKE_CXX_DEBUG_FLAGS "${CMAKE_CXX_DEBUG_FLAGS}  -fsanitize=address")
    set (CMAKE_LINKER_DEBUG_FLAGS "${CMAKE_LINKER_DEBUG_FLAGS}  -fsanitize=address")
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(bonxai_server SHARED
  src/bonxai_server.cpp
)

rclcpp_components_register_node(bonxai_server
  PLUGIN "bonxai_server::BonxaiServer"
  EXECUTABLE bonxai_server_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch 
  rviz 
  params 
)
