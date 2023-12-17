
add_executable(benchmark_pointcloud benchmark_pointcloud.cpp)

target_compile_definitions(benchmark_pointcloud PUBLIC
    DATA_PATH="${PROJECT_SOURCE_DIR}/data/")

target_include_directories(benchmark_pointcloud PUBLIC ${octomap_INCLUDE_DIRS})
target_link_libraries(benchmark_pointcloud PUBLIC
    benchmark::benchmark
    bonxai_map
    ${OCTOMAP_LIBRARIES})

###############################################
add_executable(benchmark_kitti benchmark_kitti.cpp)

target_include_directories(benchmark_kitti PUBLIC
    $<BUILD_INTERFACE:${octomap_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rdparty> )

target_link_libraries(benchmark_kitti PUBLIC
    bonxai_map
    ${OCTOMAP_LIBRARIES})
