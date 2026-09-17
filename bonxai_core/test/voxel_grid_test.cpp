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

// leaf_bits >= 4 puts Mask::words_ on the heap, where destroying the leaf twice
// was a double free. Meaningful under -fsanitize=address.
TEST(VoxelGridAllocatorTest, LargeLeafIsDestroyedExactlyOnce) {
  for (uint8_t leaf_bits : {3, 4, 5}) {
    Bonxai::VoxelGrid<int, Bonxai::DynamicShape> grid(1.0, 2, leaf_bits);
    auto accessor = grid.createAccessor();
    for (int i = 0; i < 128; ++i) {
      accessor.setValue({i, i * 2, i * 3}, i);
    }
    EXPECT_EQ(grid.activeCellsCount(), 128u);
    grid.clear(Bonxai::CLEAR_MEMORY);
    EXPECT_EQ(grid.activeCellsCount(), 0u);
  }
}

TEST(VoxelGridShapeTest, StaticAndDynamicShapesAgree) {
  Bonxai::VoxelGrid<int> with_static(1.0);
  Bonxai::VoxelGrid<int, Bonxai::DynamicShape> with_dynamic(1.0);

  auto static_accessor = with_static.createAccessor();
  auto dynamic_accessor = with_dynamic.createAccessor();
  for (int i = -20; i < 20; ++i) {
    for (int j = -20; j < 20; ++j) {
      static_accessor.setValue({i, j, i + j}, i * 31 + j);
      dynamic_accessor.setValue({i, j, i + j}, i * 31 + j);
    }
  }
  EXPECT_EQ(with_static.activeCellsCount(), with_dynamic.activeCellsCount());

  auto static_reader = with_static.createConstAccessor();
  auto dynamic_reader = with_dynamic.createConstAccessor();
  for (int i = -20; i < 20; ++i) {
    for (int j = -20; j < 20; ++j) {
      const int* a = static_reader.value({i, j, i + j});
      const int* b = dynamic_reader.value({i, j, i + j});
      ASSERT_NE(a, nullptr);
      ASSERT_NE(b, nullptr);
      EXPECT_EQ(*a, *b);
    }
  }

  EXPECT_THROW((Bonxai::VoxelGrid<int>(1.0, 3, 4)), std::runtime_error);
  EXPECT_NO_THROW((Bonxai::VoxelGrid<int, Bonxai::DynamicShape>(1.0, 3, 4)));
}
