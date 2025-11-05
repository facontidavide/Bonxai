
add_library(bonxai_core INTERFACE
    include/bonxai/bonxai.hpp
    include/bonxai/serialization.hpp
    include/bonxai/grid_coord.hpp
)
set_target_properties(bonxai_core PROPERTIES LINKER_LANGUAGE CXX)

add_library(Bonxai::bonxai_core ALIAS bonxai_core)

target_include_directories(bonxai_core PUBLIC INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS bonxai_core
    EXPORT BonxaiTargets)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(benchmark_FOUND)
    add_subdirectory(benchmark)
endif()
