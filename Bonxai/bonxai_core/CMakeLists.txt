
add_library(bonxai_core
    include/bonxai/bonxai.hpp
    include/bonxai/serialization.hpp
)
set_target_properties(bonxai_core PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(bonxai_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(
    DIRECTORY include/
    DESTINATION include/
)

install(
    TARGETS bonxai_core
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)


if(benchmark_FOUND)
    add_subdirectory(benchmark)
endif()

