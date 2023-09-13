

add_executable(benchmark_pointcloud benchmark_pointcloud.cpp)

target_compile_definitions(benchmark_pointcloud PUBLIC
    DATA_PATH="${PROJECT_SOURCE_DIR}/data/")

target_include_directories(benchmark_pointcloud PUBLIC
    ${octomap_INCLUDE_DIRS})

target_link_libraries(benchmark_pointcloud PUBLIC
    benchmark::benchmark
    bonxai_map
    ${OCTOMAP_LIBRARIES})
