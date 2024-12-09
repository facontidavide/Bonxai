
CPMAddPackage(
  NAME googletest
  GITHUB_REPOSITORY google/googletest
  GIT_TAG v1.15.2
  OPTIONS "INSTALL_GTEST OFF" "gtest_force_shared_crt"
)

add_executable(test_grid test_grid.cpp)
target_link_libraries(test_grid PRIVATE bonxai_core gtest_main)
