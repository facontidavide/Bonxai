enable_testing()

add_executable(voxel_grid_test voxel_grid_test.cpp)
target_link_libraries(voxel_grid_test PRIVATE bonxai_core)

target_link_libraries(voxel_grid_test PRIVATE GTest::gtest GTest::gtest_main)

add_test(NAME voxel_grid_test COMMAND voxel_grid_test)
