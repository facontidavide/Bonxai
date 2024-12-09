
add_library(bonxai_core INTERFACE
    include/bonxai/bonxai.hpp
    include/bonxai/serialization.hpp
    include/bonxai/grid_coord.hpp
)
set_target_properties(bonxai_core PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(bonxai_core PUBLIC INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

if(benchmark_FOUND)
    add_subdirectory(benchmark)
endif()

option(BONXAI_C_LIBRARY "Whether to build the Bonxai C library or not" OFF)

if(BONXAI_C_LIBRARY)
    option(BUILD_SHARED_LIBS "Build a shared library from the C API sources" ON)
    option(BUILD_STATIC_LIBS "Build a static library from the C API sources" ON)

    if(BUILD_SHARED_LIBS)
        add_library(bonxai SHARED
            include/bonxai/bonxai.h
            src/bonxai.cpp
            src/pointer_input_stream.hpp
            src/vector_output_stream.hpp)
        add_dependencies(bonxai bonxai_core)
        target_link_libraries(bonxai PRIVATE bonxai_core)
        target_include_directories(bonxai PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/bonxai_core/include")
    endif()

    if(BUILD_STATIC_LIBS)
        add_library(bonxai_static STATIC
            include/bonxai/bonxai.h
            src/bonxai.cpp
            src/pointer_input_stream.hpp
            src/vector_output_stream.hpp)
        add_dependencies(bonxai_static bonxai_core)
        target_link_libraries(bonxai_static PRIVATE bonxai_core)
        target_include_directories(bonxai_static PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/bonxai_core/include")
    endif()
endif()