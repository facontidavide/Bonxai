cmake_minimum_required(VERSION 3.5)
project(Bonxai)

#---- Compile i nRelease by default ----
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

#---- Options ----
option(BENCHMARK_OCTOMAP "Benchmark against Octomap" ON)
option(BENCHMARK_OPENVDB "Benchmark against OpenVDB" ON)
option(BENCHMARK_LAMA "Benchmark against LaMa"   OFF)

find_package(benchmark REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

find_package(LZ4 QUIET)

set(BENCHMARK_SRC_FILES
    benchmark/benchmark_bonxai.cpp)

set(BENCHMARK_DEPENDENCIES
    benchmark::benchmark
    ${PCL_LIBRARIES} )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if( LZ4_FOUND )
    list(APPEND BENCHMARK_SRC_FILES benchmark/benchmark_compression.cpp)
    include_directories(${LZ4_INCLUDE_DIRS})
    list(APPEND BENCHMARK_DEPENDENCIES ${LZ4_LIBRARIES})
endif()

if( BENCHMARK_OCTOMAP )
    find_package(octomap REQUIRED)
    include_directories(${OCTOMAP_INCLUDE_DIRS})

    list(APPEND BENCHMARK_SRC_FILES benchmark/benchmark_octomap.cpp)
    list(APPEND BENCHMARK_DEPENDENCIES ${OCTOMAP_LIBRARIES})
endif()

if( BENCHMARK_OPENVDB )
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake/OpenVDB/")
    find_package(OpenVDB REQUIRED)
    find_package(TBB REQUIRED)
    add_definitions(-DBENCHMARK_OPENVDB)

    list(APPEND BENCHMARK_SRC_FILES benchmark/benchmark_openvdb.cpp)
    list(APPEND BENCHMARK_DEPENDENCIES ${OpenVDB_LIBRARIES} TBB::tbb)
endif()

if( BENCHMARK_LAMA )
    find_package(iris_lama REQUIRED)
    include_directories(${iris_lama_INCLUDE_DIRS})

    list(APPEND BENCHMARK_SRC_FILES benchmark/benchmark_lama.cpp)
    list(APPEND BENCHMARK_DEPENDENCIES ${iris_lama_LIBRARIES})
endif()

#if( BENCHMARK_OPEN3D )
#    find_package(Open3D REQUIRED)
#    set(GLIBCXX_USE_CXX11_ABI ON)
#    list(APPEND BENCHMARK_SRC_FILES benchmark/benchmark_open3d.cpp)
#    list(APPEND BENCHMARK_DEPENDENCIES Open3D::Open3D)
#endif()


include_directories( include )

message(${BENCHMARK_DEPENDENCIES})

add_executable(benchmark_bonxai
    ${BENCHMARK_SRC_FILES} )
target_link_libraries(benchmark_bonxai
    ${BENCHMARK_DEPENDENCIES})

if( BENCHMARK_OCTOMAP AND BENCHMARK_OPENVDB)
    add_executable(get_memory benchmark/get_memory.cpp )
    target_link_libraries(get_memory ${BENCHMARK_DEPENDENCIES})
endif()

add_executable(test_serialization test/test_serialization.cpp )



