cmake_minimum_required(VERSION 3.5)
project(treexy)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -g -O1 -fsanitize=address")
#set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")


option(BENCHMARK_OCTOMAP "Benchmark against OctoMap" ON)
option(BENCHMARK_OPENVDB "Benchmark against OpenVDB" ON)

if( BENCHMARK_OCTOMAP OR BENCHMARK_OPENVDB )
    find_package(benchmark REQUIRED)
    find_package(PCL REQUIRED COMPONENTS io)
endif()

if( BENCHMARK_OCTOMAP )
    find_package(octomap REQUIRED)
    add_definitions(-DOCTOMAP_NODEBUGOUT -DBENCHMARK_OCTOMAP)
endif()

if( BENCHMARK_OPENVDB )
    find_package(OpenVDB REQUIRED)
    find_package(TBB REQUIRED)
    add_definitions(-DBENCHMARK_OPENVDB)
endif()


# find dependencies
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})


include_directories(
    include
    3rdparty
)

# ----------------------------------
#add_executable(benchmark_grid3d benchmarks/benchmark_grid3d.cpp)
#target_link_libraries(benchmark_grid3d
#    benchmark
#    ${DEPENDENCIES}  )

#add_executable(benchmark_voxel benchmarks/benchmark_voxel_filter.cpp)
#target_link_libraries(benchmark_voxel
#    benchmark
#    ${DEPENDENCIES}  )
# ----------------------------------

#add_executable(main_memory src/main_voxelmap.cpp)
#target_link_libraries(main_memory ${DEPENDENCIES}  )

#add_executable(main_voxel src/main_voxel_filter.cpp)
#target_link_libraries(main_voxel ${DEPENDENCIES}  )

#add_executable(main_octomap
#    src/Octomap.cpp
#    src/main_octomap.cpp)
#target_link_libraries(main_octomap ${DEPENDENCIES}  )




