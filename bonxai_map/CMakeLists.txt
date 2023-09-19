
add_library(bonxai_map
    include/bonxai_map/pcl_utils.hpp
    include/bonxai_map/probabilistic_map.hpp
    src/pcl_utils.cpp
    src/probabilistic_map.cpp)

target_compile_definitions(bonxai_map PRIVATE
    ${PCL_DEFINITIONS})

target_include_directories(bonxai_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(bonxai_map PUBLIC
    bonxai_core
    ${PCL_LIBRARIES})

####################################################

if(benchmark_FOUND AND octomap_FOUND)
    add_subdirectory(benchmark)
endif()
