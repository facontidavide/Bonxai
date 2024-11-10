if(NOT octomap_FOUND)
    message("Missing octomap. Skipping related benchmarks")
endif()

############################################
function(compile_benchmark TARGET)
    target_compile_definitions(${TARGET} PUBLIC
        DATA_PATH="${PROJECT_SOURCE_DIR}/data/")
    target_include_directories(${TARGET} PUBLIC
        ${PCL_INCLUDE_DIRS}
        ${LZ4_INCLUDE_DIRS}
        ${octomap_INCLUDE_DIRS})
    target_link_libraries(${TARGET} PUBLIC
        benchmark::benchmark
        bonxai_core
        ${PCL_LIBRARIES}
        ${LZ4_LIBRARIES}
        ${OCTOMAP_LIBRARIES})
endfunction()

############################################

set(BENCHMARK_SRC benchmark_bonxai.cpp)
if(octomap_FOUND)
    list(APPEND BENCHMARK_SRC benchmark_octomap.cpp)
endif()

add_executable(benchmark_bonxai ${BENCHMARK_SRC})
compile_benchmark(benchmark_bonxai)

#############################################

if( octomap_FOUND )
    add_executable(bonxai_memory get_memory.cpp )
    compile_benchmark(bonxai_memory)

    if( LZ4_FOUND )
        add_executable(benchmark_compression benchmark_compression.cpp )
        compile_benchmark(benchmark_compression)
    endif()
endif()
