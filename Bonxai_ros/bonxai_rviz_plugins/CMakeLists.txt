cmake_minimum_required(VERSION 3.10)
project(bonxai_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

set(bonxai_rviz_plugins_headers_to_moc
  include/bonxai_rviz_plugins/bonxai_grid_display.hpp
)

foreach(header "${bonxai_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(bonxai_rviz_plugins_moc_files "${header}")
endforeach()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${bonxai_rviz_plugins_moc_files}
  src/bonxai_grid_display.cpp
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "bonxai_rviz_plugins_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    icons
)

