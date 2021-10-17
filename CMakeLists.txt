cmake_minimum_required(VERSION 3.5)
project(Treexy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set (CMAKE_CXX_DEBUG_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
set (CMAKE_LINKER_DEBUG_FLAGS "${CMAKE_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BENCHMARK_OCTOMAP "Benchmark against Octomap" ON)
option(BENCHMARK_OPENVDB "Benchmark against OpenVDB" ON)
#option(BENCHMARK_OPEN3D "Benchmark against Open3D" OFF)

find_package(benchmark REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

set(BENCHMARK_SRC_FILES
    benchmark/benchmark_treexy.cpp)

set(BENCHMARK_DEPENDENCIES
    benchmark::benchmark
    ${PCL_LIBRARIES} )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

#if( BENCHMARK_OPEN3D )
#    find_package(Open3D REQUIRED)
#    set(GLIBCXX_USE_CXX11_ABI ON)
#    list(APPEND BENCHMARK_SRC_FILES benchmark/benchmark_open3d.cpp)
#    list(APPEND BENCHMARK_DEPENDENCIES Open3D::Open3D)
#endif()


include_directories( include )

message(${BENCHMARK_DEPENDENCIES})

add_executable(benchmark_treexy ${BENCHMARK_SRC_FILES} )
target_link_libraries(benchmark_treexy ${BENCHMARK_DEPENDENCIES} )

if( BENCHMARK_OCTOMAP AND BENCHMARK_OPENVDB)
    add_executable(get_memory benchmark/get_memory.cpp )
    target_link_libraries(get_memory ${BENCHMARK_DEPENDENCIES} )
endif()

