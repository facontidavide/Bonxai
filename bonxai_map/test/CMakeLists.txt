enable_testing()

add_executable(probabilistic_map_test probabilistic_map_test.cpp)
target_link_libraries(probabilistic_map_test PRIVATE bonxai_map)

target_link_libraries(probabilistic_map_test PRIVATE GTest::gtest GTest::gtest_main)

add_test(NAME probabilistic_map_test COMMAND probabilistic_map_test)
