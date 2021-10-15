cmake_minimum_required(VERSION 3.5)
project(Treexy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -g -O1 -fsanitize=address")
#set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

option(BENCHMARK_OCTOMAP "Benchmark against Octomap" ON)
option(BENCHMARK_OPENVDB "Benchmark against OpenVDB" OFF)

find_package(benchmark REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

set(BENCHMARK_SRC_FILES
    benchmark/benchmark_treexy.cpp)

set(BENCHMARK_DEPENDENCIES
    benchmark::benchmark
    ${PCL_LIBRARIES} )

if( BENCHMARK_OCTOMAP )
    find_package(octomap REQUIRED)
    add_definitions(-DOCTOMAP_NODEBUGOUT -DBENCHMARK_OCTOMAP)
    list(APPEND BENCHMARK_SRC_FILES benchmark/benchmark_octomap.cpp)
    list(APPEND BENCHMARK_DEPENDENCIES ${OCTOMAP_LIBRARIES})
endif()

if( BENCHMARK_OPENVDB )
    find_package(TBB REQUIRED)
    add_definitions(-DBENCHMARK_OPENVDB)

    list(APPEND BENCHMARK_SRC_FILES benchmark/benchmark_openvdb.cpp)
    list(APPEND BENCHMARK_DEPENDENCIES
        openvdb
        TBB::tbb)
endif()

include_directories(
    include
    ${OCTOMAP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${TBB_INCLUDE_DIRS}
)

add_executable(benchmark_treexy ${BENCHMARK_SRC_FILES})
target_link_libraries(benchmark_treexy
    ${BENCHMARK_DEPENDENCIES}  )



