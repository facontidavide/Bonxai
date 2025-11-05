
add_library(bonxai_map
    include/bonxai_map/pcl_utils.hpp
    include/bonxai_map/probabilistic_map.hpp
    src/pcl_utils.cpp
    src/probabilistic_map.cpp)

add_library(Bonxai::bonxai_map ALIAS bonxai_map)

target_compile_definitions(bonxai_map PRIVATE
    ${PCL_DEFINITIONS})

target_include_directories(bonxai_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(bonxai_map PUBLIC
    bonxai_core
    ${PCL_LIBRARIES})

install(TARGETS bonxai_map
    EXPORT BonxaiTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

####################################################

if(benchmark_FOUND AND octomap_FOUND)
    add_subdirectory(benchmark)
endif()
