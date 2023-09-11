
find_package(octomap QUIET)

if(NOT octomap_FOUND)
    message("Missing octomap. Skipping related benchmark")
endif()

set(BENCHMARK_SRC_FILES benchmark_bonxai.cpp)
set(BENCHMARK_DEPENDENCIES benchmark::benchmark ${PCL_LIBRARIES})

if( octomap_FOUND )
    list(APPEND BENCHMARK_SRC_FILES benchmark_octomap.cpp)
    list(APPEND BENCHMARK_DEPENDENCIES ${OCTOMAP_LIBRARIES})
endif()

add_executable(benchmark_bonxai ${BENCHMARK_SRC_FILES} )

target_compile_definitions(benchmark_bonxai PRIVATE ${PCL_DEFINITIONS})
target_compile_definitions(benchmark_bonxai PUBLIC DATA_PATH="${PROJECT_SOURCE_DIR}/data/")
target_include_directories(benchmark_bonxai PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_directories(benchmark_bonxai PRIVATE ${PCL_LIBRARY_DIRS})
target_link_libraries(benchmark_bonxai ${BENCHMARK_DEPENDENCIES} )

if( octomap_FOUND )

    add_executable(bonxai_memory get_memory.cpp )

    target_compile_definitions(bonxai_memory PRIVATE ${PCL_DEFINITIONS})
    target_compile_definitions(bonxai_memory PUBLIC DATA_PATH="${PROJECT_SOURCE_DIR}/data/")
    target_include_directories(bonxai_memory PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_directories(bonxai_memory PRIVATE ${PCL_LIBRARY_DIRS})
    target_link_libraries(bonxai_memory ${BENCHMARK_DEPENDENCIES} )
endif()



