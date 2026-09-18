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

# these read the .pcd files in data/ through benchmark_utils.hpp, so they need PCL
if(PCL_FOUND)
    set(BENCHMARK_SRC benchmark_bonxai.cpp)
    if(octomap_FOUND)
        list(APPEND BENCHMARK_SRC benchmark_octomap.cpp)
    endif()

    add_executable(benchmark_bonxai ${BENCHMARK_SRC})
    compile_benchmark(benchmark_bonxai)

    if( octomap_FOUND )
        add_executable(bonxai_memory get_memory.cpp )
        compile_benchmark(bonxai_memory)

        if( LZ4_FOUND )
            add_executable(benchmark_compression benchmark_compression.cpp )
            compile_benchmark(benchmark_compression)
        endif()
    endif()
else()
    message("Missing PCL. Skipping the benchmarks that read the clouds in data/")
endif()

#############################################
# Bonxai vs NanoVDB. Off by default because it downloads the OpenVDB release
# tarball; enable with -DBONXAI_BENCHMARK_NANOVDB=ON, or point NANOVDB_ROOT at
# headers that are already on disk.

option(BONXAI_BENCHMARK_NANOVDB "Build the NanoVDB comparison benchmark" OFF)

if(BONXAI_BENCHMARK_NANOVDB)
    include(FetchNanoVDB)
    if(NanoVDB_FOUND)
        add_executable(benchmark_nanovdb benchmark_nanovdb.cpp)
        target_link_libraries(benchmark_nanovdb PRIVATE
            benchmark::benchmark
            bonxai_core
            nanovdb)
        # this benchmark uses the synthetic scan generator, so it needs neither
        # PCL nor the git-lfs .pcd files in data/
    endif()
endif()
