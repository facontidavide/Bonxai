cmake_minimum_required(VERSION 3.5)
project(bonxai_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Disable Wredundant-decls warnings since rosidl generates redundant function declarations 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-redundant-decls")
endif()

find_package(ament_cmake REQUIRED)
find_package(bonxai_core REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
	"msg/Bonxai.msg"
	"msg/BonxaiWithPose.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
	DEPENDENCIES
    bonxai_core
    builtin_interfaces
		geometry_msgs
		std_msgs
)


install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)

ament_export_dependencies(rosidl_default_runtime)
ament_package()