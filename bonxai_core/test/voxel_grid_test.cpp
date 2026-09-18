#include <gtest/gtest.h>

#include <map>
#include <tuple>
#include <vector>

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

// Negative coordinates exercise the sign bits of the key masking, which is easy
// to get wrong and which nothing else here covers.
TEST(VoxelGrid, NegativeAndFarCoordinates) {
  Bonxai::VoxelGrid<int> grid(0.1);
  auto accessor = grid.createAccessor();

  std::vector<Bonxai::CoordT> coords;
  for (int x : {-1000, -33, -32, -31, -1, 0, 1, 31, 32, 33, 1000}) {
    for (int y : {-33, -1, 0, 1, 33}) {
      coords.push_back({x, y, -x});
    }
  }
  coords.push_back({-2000000, 2000000, -2000000});

  int value = 0;
  for (const auto& coord : coords) {
    accessor.setValue(coord, value++);
  }
  EXPECT_EQ(grid.activeCellsCount(), coords.size());

  auto reader = grid.createConstAccessor();
  value = 0;
  for (const auto& coord : coords) {
    const int* found = reader.value(coord);
    ASSERT_NE(found, nullptr) << coord.x << " " << coord.y << " " << coord.z;
    EXPECT_EQ(*found, value++);
  }
}

// forEachCell rebuilds each coordinate from the root key and the two indices.
TEST(VoxelGrid, ForEachCellReportsTheOriginalCoordinates) {
  Bonxai::VoxelGrid<int> grid(0.1);
  std::map<std::tuple<int, int, int>, int> written;
  {
    auto accessor = grid.createAccessor();
    int value = 0;
    for (int x = -20; x <= 20; x += 3) {
      for (int y = -20; y <= 20; y += 5) {
        for (int z = -20; z <= 20; z += 7) {
          accessor.setValue({x, y, z}, value);
          written[{x, y, z}] = value++;
        }
      }
    }
  }

  std::map<std::tuple<int, int, int>, int> visited;
  grid.forEachCell([&](int& value, const Bonxai::CoordT& coord) {
    visited[{coord.x, coord.y, coord.z}] = value;
  });
  EXPECT_EQ(visited, written);
}

TEST(VoxelGrid, SetCellOffKeepsTheValueButHidesTheCell) {
  Bonxai::VoxelGrid<int> grid(1.0);
  auto accessor = grid.createAccessor();
  const Bonxai::CoordT coord{4, 5, 6};

  accessor.setValue(coord, 42);
  EXPECT_TRUE(accessor.setCellOff(coord));
  EXPECT_FALSE(accessor.setCellOff(coord));
  EXPECT_EQ(accessor.value(coord), nullptr);
  EXPECT_EQ(grid.activeCellsCount(), 0u);

  // turning it back on counts as creating it, so the default is written
  EXPECT_FALSE(accessor.setCellOn(coord, 7));
  const int* value = accessor.value(coord);
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 7);

  // whereas a cell that is already on keeps its value
  EXPECT_TRUE(accessor.setCellOn(coord, 11));
  EXPECT_EQ(*accessor.value(coord), 7);
}

TEST(VoxelGrid, ClearingKeepsTheGridUsable) {
  Bonxai::VoxelGrid<int> grid(1.0);
  auto fill = [&] {
    auto accessor = grid.createAccessor();
    for (int i = 0; i < 200; ++i) {
      accessor.setValue({i, -i, i * 2}, i);
    }
  };

  fill();
  EXPECT_EQ(grid.activeCellsCount(), 200u);

  grid.clear(Bonxai::SET_ALL_CELLS_OFF);
  EXPECT_EQ(grid.activeCellsCount(), 0u);
  fill();
  EXPECT_EQ(grid.activeCellsCount(), 200u);

  grid.clear(Bonxai::CLEAR_MEMORY);
  EXPECT_EQ(grid.activeCellsCount(), 0u);
  fill();
  EXPECT_EQ(grid.activeCellsCount(), 200u);

  auto reader = grid.createConstAccessor();
  for (int i = 0; i < 200; ++i) {
    const int* value = reader.value({i, -i, i * 2});
    ASSERT_NE(value, nullptr);
    EXPECT_EQ(*value, i);
  }
}

// releaseUnusedMemory drops leaves whose cells are all off, and hands their
// blocks back to the pool for the next grid to reuse.
TEST(VoxelGrid, ReleaseUnusedMemoryReclaimsEmptyLeaves) {
  Bonxai::VoxelGrid<int> grid(1.0);
  {
    auto accessor = grid.createAccessor();
    for (int i = 0; i < 500; ++i) {
      accessor.setValue({i * 8, 0, 0}, i);
    }
  }
  const size_t full = grid.memUsage();

  {
    auto accessor = grid.createAccessor();
    for (int i = 0; i < 500; ++i) {
      accessor.setCellOff({i * 8, 0, 0});
    }
  }
  grid.releaseUnusedMemory();

  EXPECT_EQ(grid.activeCellsCount(), 0u);
  EXPECT_LT(grid.memUsage(), full);
  EXPECT_TRUE(grid.rootMap().empty());

  // still usable through a fresh accessor
  auto accessor = grid.createAccessor();
  accessor.setValue({1, 2, 3}, 99);
  const int* value = accessor.value({1, 2, 3});
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 99);
}

TEST(BinaryVoxelGrid, TracksOccupancyWithoutValues) {
  Bonxai::BinaryVoxelGrid grid(0.5);
  auto accessor = grid.createAccessor();

  EXPECT_FALSE(accessor.setCellOn({1, 2, 3}));
  EXPECT_TRUE(accessor.setCellOn({1, 2, 3}));
  EXPECT_EQ(grid.activeCellsCount(), 1u);

  auto reader = grid.createConstAccessor();
  EXPECT_TRUE(reader.isCellOn({1, 2, 3}));
  EXPECT_FALSE(reader.isCellOn({1, 2, 4}));

  EXPECT_TRUE(accessor.setCellOff({1, 2, 3}));
  EXPECT_EQ(grid.activeCellsCount(), 0u);
}

TEST(VoxelGrid, PosToCoordAndBack) {
  const double resolution = 0.05;
  Bonxai::VoxelGrid<int> grid(resolution);

  // a point inside a voxel maps to that voxel, and the voxel maps back to its
  // lower corner
  EXPECT_EQ(grid.posToCoord(0.0, 0.0, 0.0), (Bonxai::CoordT{0, 0, 0}));
  EXPECT_EQ(grid.posToCoord(0.049, 0.0, 0.0), (Bonxai::CoordT{0, 0, 0}));
  EXPECT_EQ(grid.posToCoord(0.051, 0.0, 0.0), (Bonxai::CoordT{1, 0, 0}));
  // floor, not truncation: negatives round away from zero
  EXPECT_EQ(grid.posToCoord(-0.001, -0.049, -0.051), (Bonxai::CoordT{-1, -1, -2}));

  const auto pos = grid.coordToPos({3, -4, 5});
  EXPECT_DOUBLE_EQ(pos.x, 3 * resolution);
  EXPECT_DOUBLE_EQ(pos.y, -4 * resolution);
  EXPECT_DOUBLE_EQ(pos.z, 5 * resolution);
}

//----------------------------------------------------------------
// Accessors cache the inner/leaf nodes they visited last. clear(CLEAR_MEMORY)
// and releaseUnusedMemory() free those nodes, so the cache must be dropped:
// otherwise the accessor dereferences freed memory. See issue #52.
//----------------------------------------------------------------

TEST(VoxelGridStaleCache, AccessorSurvivesClearMemory) {
  Bonxai::VoxelGrid<int> grid(1.0);
  auto accessor = grid.createAccessor();

  const Bonxai::CoordT coord{7, 2, 3};
  accessor.setValue(coord, 1);
  ASSERT_EQ(grid.activeCellsCount(), 1u);

  // frees every inner and leaf node, while the accessor still caches this coordinate
  grid.clear(Bonxai::CLEAR_MEMORY);
  ASSERT_EQ(grid.activeCellsCount(), 0u);

  accessor.setValue(coord, 42);
  int* value = accessor.value(coord, false);
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 42);
  EXPECT_EQ(grid.activeCellsCount(), 1u);
}

TEST(VoxelGridStaleCache, AccessorSurvivesReleaseUnusedMemory) {
  Bonxai::VoxelGrid<int> grid(1.0);
  auto accessor = grid.createAccessor();

  const Bonxai::CoordT coord{7, 2, 3};
  accessor.setValue(coord, 1);
  accessor.setCellOff(coord);
  // the leaf is entirely OFF now, so it is released
  grid.releaseUnusedMemory();

  accessor.setValue(coord, 42);
  int* value = accessor.value(coord, false);
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 42);
  EXPECT_EQ(grid.activeCellsCount(), 1u);
}

TEST(VoxelGridStaleCache, AccessorSetCellOnSurvivesClearMemory) {
  Bonxai::BinaryVoxelGrid grid(1.0);
  auto accessor = grid.createAccessor();

  const Bonxai::CoordT coord{7, 2, 3};
  accessor.setCellOn(coord);
  grid.clear(Bonxai::CLEAR_MEMORY);

  EXPECT_FALSE(accessor.isCellOn(coord));
  accessor.setCellOn(coord);
  EXPECT_TRUE(accessor.isCellOn(coord));
  EXPECT_EQ(grid.activeCellsCount(), 1u);
}

TEST(VoxelGridStaleCache, ConstAccessorSurvivesClearMemory) {
  Bonxai::VoxelGrid<int> grid(1.0);
  auto writer = grid.createAccessor();
  const Bonxai::CoordT coord{7, 2, 3};
  writer.setValue(coord, 1);

  auto reader = grid.createConstAccessor();
  ASSERT_NE(reader.value(coord), nullptr);

  grid.clear(Bonxai::CLEAR_MEMORY);

  EXPECT_EQ(reader.value(coord), nullptr);
  EXPECT_FALSE(reader.isCellOn(coord));
  EXPECT_EQ(reader.getLeafGrid(coord), nullptr);
  EXPECT_EQ(reader.lastInnerGrid(), nullptr);
  EXPECT_EQ(reader.lastLeafGrid(), nullptr);
}

TEST(VoxelGridStaleCache, ConstAccessorDoesNotCacheAMiss) {
  Bonxai::VoxelGrid<int> grid(1.0);
  const Bonxai::CoordT coord{7, 2, 3};

  auto reader = grid.createConstAccessor();
  // the cell does not exist yet: the miss must not be cached
  EXPECT_EQ(reader.value(coord), nullptr);
  EXPECT_FALSE(reader.isCellOn(coord));

  auto writer = grid.createAccessor();
  writer.setValue(coord, 42);

  const int* value = reader.value(coord);
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 42);
  EXPECT_TRUE(reader.isCellOn(coord));
}

TEST(VoxelGridStaleCache, AccessorReadAfterWriteIsConsistent) {
  Bonxai::VoxelGrid<int> grid(1.0);
  auto accessor = grid.createAccessor();
  const Bonxai::CoordT coord{7, 2, 3};

  // the read-only methods are inherited from ConstAccessor, which keeps its own cache
  const Bonxai::VoxelGrid<int>::ConstAccessor& reader = accessor;

  // read of a missing cell, through the ConstAccessor part of the cache
  EXPECT_FALSE(accessor.isCellOn(coord));
  EXPECT_EQ(reader.value(coord), nullptr);

  // ... then a write through the Accessor part of the cache
  accessor.setValue(coord, 42);

  EXPECT_TRUE(accessor.isCellOn(coord));
  const int* value = reader.value(coord);
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 42);
}
