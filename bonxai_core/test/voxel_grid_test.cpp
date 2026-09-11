#include <gtest/gtest.h>

#include "bonxai/bonxai.hpp"

TEST(VoxelGridValueSegfaultTest, AccessWithNullPtrInitially) {
  std::unique_ptr<Bonxai::VoxelGrid<int>> grid;
  grid = std::make_unique<Bonxai::VoxelGrid<int>>(1.0);

  auto accessor = grid->createAccessor();

  Bonxai::CoordT coord{10, 10, 10};

  // Cell doesn't exist, so we expect a nullptr.
  int* ptr1 = accessor.value(coord, false);
  EXPECT_EQ(ptr1, nullptr);

  // prev_leaf_ptr_ is nullptr now.

  Bonxai::CoordT coord2{11, 11, 11};
  // Create if is missing should create the cell and return a valid pointer even if prev_leaf_ptr is
  // currently null.
  int* ptr2 = accessor.value(coord2, true);
  ASSERT_NE(ptr2, nullptr);
  *ptr2 = 555;
}

TEST(VoxelGridSetCellOnTest, SetCellOnSegfaultWithNullPtrInitially) {
  std::unique_ptr<Bonxai::VoxelGrid<int>> grid;
  grid = std::make_unique<Bonxai::VoxelGrid<int>>(1.0);
  auto accessor = grid->createAccessor();

  // Define two coordinates that share the same inner_key.
  Bonxai::CoordT coord1{10, 10, 10};
  Bonxai::CoordT coord2{11, 11, 11};

  // Trigger a cache miss.
  int* ptr1 = accessor.value(coord1, false);
  EXPECT_EQ(ptr1, nullptr);

  // Attempt to set a cell ON in the same inner grid.
  int default_value = 777;
  bool was_on = accessor.setCellOn(coord2, default_value);

  // 5. Verify the state is now correct
  EXPECT_FALSE(was_on);

  int* ptr2 = accessor.value(coord2, false);
  ASSERT_NE(ptr2, nullptr);
  EXPECT_EQ(*ptr2, default_value);
}

TEST(VoxelGridStaleCacheTest, AccessorSurvivesClearMemory) {
  Bonxai::VoxelGrid<int> grid(1.0);
  auto accessor = grid.createAccessor();

  const Bonxai::CoordT coord{7, 2, 3};
  accessor.setValue(coord, 1);
  ASSERT_EQ(grid.activeCellsCount(), 1u);

  // Frees every inner and leaf node while the accessor still caches this coordinate.
  grid.clear(Bonxai::CLEAR_MEMORY);
  EXPECT_EQ(grid.activeCellsCount(), 0u);

  accessor.setValue(coord, 42);
  int* value = accessor.value(coord, false);
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 42);
  EXPECT_EQ(grid.activeCellsCount(), 1u);
}

TEST(VoxelGridStaleCacheTest, AccessorSurvivesReleaseUnusedMemory) {
  Bonxai::VoxelGrid<int> grid(1.0);
  auto accessor = grid.createAccessor();

  const Bonxai::CoordT coord{7, 2, 3};
  accessor.setValue(coord, 1);
  accessor.setCellOff(coord);
  grid.releaseUnusedMemory();

  accessor.setValue(coord, 42);
  int* value = accessor.value(coord, false);
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 42);
}

TEST(VoxelGridStaleCacheTest, ConstAccessorSurvivesClearMemory) {
  Bonxai::VoxelGrid<int> grid(1.0);
  auto writer = grid.createAccessor();
  const Bonxai::CoordT coord{7, 2, 3};
  writer.setValue(coord, 1);

  auto reader = grid.createConstAccessor();
  ASSERT_NE(reader.value(coord), nullptr);

  grid.clear(Bonxai::CLEAR_MEMORY);

  EXPECT_EQ(reader.value(coord), nullptr);
  EXPECT_FALSE(reader.isCellOn(coord));
}
