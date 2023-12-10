cmake_minimum_required(VERSION 3.8)
project(bonxai_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(bonxai_core REQUIRED)
find_package(bonxai_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker bonxai_core bonxai_msgs rclcpp std_msgs)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener bonxai_core bonxai_msgs rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
