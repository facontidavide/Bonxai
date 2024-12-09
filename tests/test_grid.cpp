#include <gtest/gtest.h>

#include "bonxai/bonxai.hpp"

TEST(Grid, CheckRange) {
  double resolution = 0.01;
  Bonxai::VoxelGrid<int> grid(resolution, 2, 3);

  auto accessor = grid.createAccessor();

  accessor.setValue({0, 0, 0}, 1);
  EXPECT_EQ(grid.activeCellsCount(), 1);
  EXPECT_EQ(grid.rootMap().size(), 1);

  // considering 5 bits for the ineer grid, this should fit into it
  const int MAX_INNER = 1 << 5;

  accessor.setValue({MAX_INNER - 1, 0, 0}, 1);
  EXPECT_EQ(grid.activeCellsCount(), 2);
  EXPECT_EQ(grid.rootMap().size(), 1);

  // this triggers the creation of a second inner grid
  accessor.setValue({-1, 0, 0}, 1);
  EXPECT_EQ(grid.activeCellsCount(), 3);
  EXPECT_EQ(grid.rootMap().size(), 2);

  accessor.setValue({-MAX_INNER, 0, 0}, 1);
  EXPECT_EQ(grid.activeCellsCount(), 4);
  EXPECT_EQ(grid.rootMap().size(), 2);

  accessor.setValue({0, -1, 0}, 1);
  accessor.setValue({0, 1, 0}, 1);
  accessor.setValue({0, 0, -1}, 1);
  accessor.setValue({0, 0, -1}, 1);
  EXPECT_EQ(grid.rootMap().size(), 4);

  for (const auto& [coord, inner] : grid.rootMap()) {
    std::cout << "Inner grid at: " << coord.x << ", " << coord.y << ", " << coord.z << std::endl;
  }
}
