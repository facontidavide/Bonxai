enable_testing()

foreach(TEST_NAME voxel_grid_test mask_test serialization_test)
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME} PRIVATE
        bonxai_core
        GTest::gtest
        GTest::gtest_main)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()
