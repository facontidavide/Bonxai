cmake_minimum_required(VERSION 3.10)
project(Boncai LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")

  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

#---- Use Address sanitizer if compiled in Debug mode ----
option(BONXAI_SANITIZE "Add address sanitizer when compiling in Debug mode" OFF)

if(BONXAI_SANITIZE)
    set (CMAKE_CXX_DEBUG_FLAGS "${CMAKE_CXX_DEBUG_FLAGS}  -fsanitize=address")
    set (CMAKE_LINKER_DEBUG_FLAGS "${CMAKE_LINKER_DEBUG_FLAGS}  -fsanitize=address")
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()

file(GLOB_RECURSE BONCAI_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE BONCAI_PRIVATE_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp")
file(GLOB_RECURSE BONCAI_PUBLIC_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")

add_library(boncai SHARED ${BONCAI_SOURCES} ${BONCAI_PRIVATE_HEADERS} ${BONCAI_PUBLIC_HEADERS})
add_dependencies(boncai bonxai_core)
target_link_libraries(boncai PRIVATE bonxai_core)
target_include_directories(boncai PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories(boncai PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(boncai_static STATIC ${BONCAI_SOURCES} ${BONCAI_PRIVATE_HEADERS} ${BONCAI_PUBLIC_HEADERS})
add_dependencies(boncai_static bonxai_core)
target_link_libraries(boncai_static PRIVATE bonxai_core)
target_include_directories(boncai_static PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories(boncai_static PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")