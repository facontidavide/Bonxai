cmake_minimum_required(VERSION 3.10)
project(Bonxai VERSION 0.0.1 LANGUAGES CXX C)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")

  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)

include(GNUInstallDirs)

#---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(MPI QUIET COMPONENTS C)
find_package(PCL QUIET COMPONENTS common filters io)

find_package(benchmark QUIET)
find_package(octomap QUIET)
find_package(LZ4 QUIET)
find_package(Eigen3 QUIET)

#---- Address and undefined behavior sanitizers ----
# CMAKE_CXX_DEBUG_FLAGS / CMAKE_LINKER_DEBUG_FLAGS are not CMake variables, so
# this option used to do nothing at all.
option(BONXAI_SANITIZE "Build with the address and undefined behavior sanitizers" OFF)

if(BONXAI_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all)
    add_link_options(-fsanitize=address,undefined)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()


####################################################
enable_testing()

add_subdirectory(bonxai_core)
add_subdirectory(examples)

if(Eigen3_FOUND AND PCL_FOUND )
    add_subdirectory(bonxai_map)
endif()

include(CMakePackageConfigHelpers)

set(BONXAI_WITH_MAP OFF)
if(TARGET bonxai_map)
    set(BONXAI_WITH_MAP ON)
endif()

set(BONXAI_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/Bonxai")

install(EXPORT BonxaiTargets
    FILE BonxaiTargets.cmake
    NAMESPACE Bonxai::
    DESTINATION ${BONXAI_INSTALL_CMAKEDIR})

configure_package_config_file(
    cmake/BonxaiConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/BonxaiConfig.cmake"
    INSTALL_DESTINATION ${BONXAI_INSTALL_CMAKEDIR}
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/BonxaiConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/BonxaiConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/BonxaiConfigVersion.cmake"
    DESTINATION ${BONXAI_INSTALL_CMAKEDIR}
)

# Default behavior
find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    project(bonxai_ros)

    # Add colcon-specific configuration here
    message(STATUS "Building with colcon")

    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    find_package(std_srvs REQUIRED)

    include_directories(bonxai_ros/include 3rdparty)

    ament_auto_add_library(bonxai_server SHARED
      bonxai_ros/src/bonxai_server.cpp
    )

    target_link_libraries(bonxai_server
      bonxai_map
      ${PCL_LIBRARIES}
      ${std_srvs_TARGETS}
    )

    rclcpp_components_register_node(bonxai_server
      PLUGIN "bonxai_server::BonxaiServer"
      EXECUTABLE bonxai_server_node
    )

    # bonxai_core/bonxai_map are plain CMake targets; export them so other ament
    # packages in the workspace can link the probabilistic map
    install(
      TARGETS bonxai_core bonxai_map
      EXPORT bonxai_ros_targets
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include
    )
    install(
      DIRECTORY bonxai_core/include/ bonxai_map/include/
      DESTINATION include
    )
    ament_export_targets(bonxai_ros_targets HAS_LIBRARY_TARGET)
    ament_export_dependencies(Eigen3 PCL)

    ament_auto_package(
      INSTALL_TO_SHARE
      bonxai_ros/launch
      bonxai_ros/rviz
      bonxai_ros/params
    )

else()

  message(STATUS "Building with cmake")

endif()
