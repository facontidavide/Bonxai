#include <gtest/gtest.h>

#include <map>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "bonxai/bonxai.hpp"

namespace {

using Bonxai::CoordT;

using CompactGrid = Bonxai::VoxelGrid<int, Bonxai::StaticShape<>, Bonxai::CompactStorage>;
using DenseGrid = Bonxai::VoxelGrid<int>;

using Cells = std::map<std::tuple<int32_t, int32_t, int32_t>, int>;

template <typename Grid>
Cells cellsOf(const Grid& grid) {
  Cells cells;
  grid.forEachCell([&](const int& value, const CoordT& coord) {
    cells[{coord.x, coord.y, coord.z}] = value;
  });
  return cells;
}

/// Counts its live instances, to check that the grid constructs and destroys each value once.
struct Counted {
  static inline int alive = 0;
  std::string payload;

  explicit Counted(std::string text = {})
      : payload(std::move(text)) {
    ++alive;
  }
  Counted(const Counted& other)
      : payload(other.payload) {
    ++alive;
  }
  Counted(Counted&& other) noexcept
      : payload(std::move(other.payload)) {
    ++alive;
  }
  Counted& operator=(const Counted&) = default;
  Counted& operator=(Counted&&) noexcept = default;
  ~Counted() {
    --alive;
  }
};

}  // namespace

template <typename Shape>
class CompactStorageAgreement : public ::testing::Test {};

using Shapes = ::testing::Types<
    Bonxai::StaticShape<2, 3>, Bonxai::StaticShape<3, 2>, Bonxai::StaticShape<1, 1>>;
TYPED_TEST_SUITE(CompactStorageAgreement, Shapes);

// Random edits, through accessors that move each other's cached leaves, match a dense grid.
TYPED_TEST(CompactStorageAgreement, RandomEditsGiveTheSameCellsAsDenseStorage) {
  const uint8_t inner_bits = TypeParam::INNER_BITS;
  const uint8_t leaf_bits = TypeParam::LEAF_BITS;
  Bonxai::VoxelGrid<int, TypeParam> dense(1.0, inner_bits, leaf_bits);
  Bonxai::VoxelGrid<int, TypeParam, Bonxai::CompactStorage> compact(1.0, inner_bits, leaf_bits);
  auto dense_accessor = dense.createAccessor();
  auto compact_a = compact.createAccessor();
  auto compact_b = compact.createAccessor();
  auto compact_reader = compact.createConstAccessor();

  std::mt19937 rng(42);
  std::uniform_int_distribution<int32_t> axis(-20, 20);
  std::uniform_int_distribution<int> operation(0, 4);
  std::bernoulli_distribution use_a(0.5);

  for (int step = 0; step < 200000; ++step) {
    const CoordT coord{axis(rng), axis(rng), axis(rng)};
    auto& compact_accessor = use_a(rng) ? compact_a : compact_b;
    switch (operation(rng)) {
      case 0:
        ASSERT_EQ(dense_accessor.setValue(coord, step), compact_accessor.setValue(coord, step));
        break;
      case 1:
        ASSERT_EQ(dense_accessor.setCellOn(coord, step), compact_accessor.setCellOn(coord, step));
        break;
      case 2:
        ASSERT_EQ(dense_accessor.setCellOff(coord), compact_accessor.setCellOff(coord));
        break;
      case 3: {
        int* dense_value = dense_accessor.value(coord, true);
        int* compact_value = compact_accessor.value(coord, true);
        ASSERT_EQ(*dense_value, *compact_value);
        *dense_value = -step;
        *compact_value = -step;
        break;
      }
      default: {
        const int* dense_value = dense_accessor.value(coord);
        const int* compact_value = compact_reader.value(coord);
        ASSERT_EQ(dense_value == nullptr, compact_value == nullptr);
        if (dense_value != nullptr) {
          ASSERT_EQ(*dense_value, *compact_value);
        }
      }
    }
    if (step % 20000 == 19999) {
      dense.releaseUnusedMemory();
      compact.releaseUnusedMemory();
    }
    if (step == 100000) {
      dense.clear(Bonxai::SET_ALL_CELLS_OFF);
      compact.clear(Bonxai::SET_ALL_CELLS_OFF);
    }
  }

  EXPECT_EQ(cellsOf(compact), cellsOf(dense));
  EXPECT_EQ(compact.activeCellsCount(), dense.activeCellsCount());
}

TEST(CompactStorage, ConstAccessorFindsALeafThatAnotherAccessorMoved) {
  CompactGrid grid(1.0);
  auto writer = grid.createAccessor();
  auto reader = grid.createConstAccessor();

  writer.setValue({8, 0, 0}, 1);
  ASSERT_NE(reader.value({8, 0, 0}), nullptr);

  // a leaf before it in the same inner node: the leaves of that node are reallocated
  writer.setValue({0, 0, 0}, 2);
  writer.setValue({9, 0, 0}, 3);

  const int* value = reader.value({9, 0, 0});
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 3);
  EXPECT_EQ(*reader.value({8, 0, 0}), 1);
}

TEST(CompactStorage, AccessorFindsItsLeafAfterCreatingAnotherThroughGetLeafGrid) {
  CompactGrid grid(1.0);
  auto accessor = grid.createAccessor();

  accessor.setValue({8, 0, 0}, 1);
  ASSERT_NE(accessor.getLeafGrid({0, 0, 0}, true), nullptr);

  const int* value = accessor.value({8, 0, 0});
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 1);
}

TEST(CompactStorage, ConstAccessorFindsItsLeafAfterLookingUpAnotherRoot) {
  CompactGrid grid(1.0);
  auto writer = grid.createAccessor();
  auto reader = grid.createConstAccessor();

  writer.setValue({8, 0, 0}, 1);
  writer.setValue({100, 0, 0}, 2);
  ASSERT_NE(reader.value({8, 0, 0}), nullptr);
  ASSERT_NE(reader.getLeafGrid({100, 0, 0}), nullptr);
  writer.setValue({0, 0, 0}, 3);

  const int* value = reader.value({8, 0, 0});
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 1);
}

TEST(CompactStorage, ReleaseUnusedMemoryLeavesTheMemoryOfAnEmptyGrid) {
  CompactGrid grid(1.0);
  const size_t empty = grid.memUsage();
  auto accessor = grid.createAccessor();
  for (int i = 0; i < 500; ++i) {
    accessor.setValue({i * 8, 0, 0}, i);
  }
  for (int i = 0; i < 500; ++i) {
    accessor.setCellOff({i * 8, 0, 0});
  }
  grid.releaseUnusedMemory();

  EXPECT_TRUE(grid.rootMap().empty());
  EXPECT_LE(grid.memUsage() - grid.rootMap().memUsage(), empty);
}

// A surface: one plane of cells, an eighth of each leaf of 8x8x8 cells.
TEST(CompactStorage, ASurfaceTakesAFractionOfTheMemoryOfDenseStorage) {
  DenseGrid dense(1.0);
  CompactGrid compact(1.0);
  auto dense_accessor = dense.createAccessor();
  auto compact_accessor = compact.createAccessor();
  for (int x = 0; x < 256; ++x) {
    for (int y = 0; y < 256; ++y) {
      dense_accessor.setValue({x, y, 0}, x + y);
      compact_accessor.setValue({x, y, 0}, x + y);
    }
  }

  EXPECT_LT(compact.memUsage() * 4, dense.memUsage());
}

TEST(CompactStorage, BinaryGridTracksOccupancyWithoutValues) {
  Bonxai::VoxelGrid<Bonxai::EmptyVoxel, Bonxai::StaticShape<>, Bonxai::CompactStorage> grid(0.5);
  auto accessor = grid.createAccessor();

  EXPECT_FALSE(accessor.setCellOn({1, 2, 3}));
  EXPECT_TRUE(accessor.setCellOn({1, 2, 3}));
  EXPECT_FALSE(accessor.setCellOn({-1, 2, 3}));
  EXPECT_EQ(grid.activeCellsCount(), 2u);

  auto reader = grid.createConstAccessor();
  EXPECT_TRUE(reader.isCellOn({1, 2, 3}));
  EXPECT_FALSE(reader.isCellOn({1, 2, 4}));

  int visited = 0;
  grid.forEachCell([&](Bonxai::EmptyVoxel&, const CoordT&) { ++visited; });
  EXPECT_EQ(visited, 2);

  EXPECT_TRUE(accessor.setCellOff({1, 2, 3}));
  EXPECT_EQ(grid.activeCellsCount(), 1u);
}

TEST(CompactStorage, EveryValueIsDestroyedOnce) {
  using Grid = Bonxai::VoxelGrid<Counted, Bonxai::StaticShape<>, Bonxai::CompactStorage>;
  const std::string long_text(64, 'x');
  {
    Grid grid(1.0);
    auto accessor = grid.createAccessor();
    std::mt19937 rng(7);
    std::uniform_int_distribution<int32_t> axis(-12, 12);
    std::bernoulli_distribution turn_on(0.6);
    for (int step = 0; step < 20000; ++step) {
      const CoordT coord{axis(rng), axis(rng), axis(rng)};
      if (turn_on(rng)) {
        accessor.setValue(coord, Counted(long_text + std::to_string(step)));
      } else {
        accessor.setCellOff(coord);
      }
    }
    EXPECT_EQ(size_t(Counted::alive), grid.activeCellsCount());

    grid.forEachCell([&](Counted& value, const CoordT&) {
      EXPECT_EQ(value.payload.compare(0, long_text.size(), long_text), 0);
    });

    grid.releaseUnusedMemory();
    EXPECT_EQ(size_t(Counted::alive), grid.activeCellsCount());

    grid.clear(Bonxai::SET_ALL_CELLS_OFF);
    EXPECT_EQ(Counted::alive, 0);

    accessor.setValue({1, 1, 1}, Counted(long_text));
    Grid moved(std::move(grid));
    EXPECT_EQ(Counted::alive, 1);
    EXPECT_EQ(moved.activeCellsCount(), 1u);
  }
  EXPECT_EQ(Counted::alive, 0);
}

// Both ways of counting, whatever the build; full words are where the byte sums could overflow.
TEST(StaticMask, CountOnPrefixAgreesWithPopcount) {
  using Bonxai::details::CountOnPrefix;
  std::mt19937_64 rng(3);
  std::vector<uint64_t> words(64);
  for (int trial = 0; trial < 100; ++trial) {
    const bool full = trial % 4 == 0;
    for (auto& word : words) {
      word = full ? ~uint64_t(0) : rng();
    }
    const uint64_t last = full ? ~uint64_t(0) : rng();
    uint32_t expected = Bonxai::Mask::CountOn(last);
    for (uint32_t count = 0; count <= words.size(); ++count) {
      ASSERT_EQ((CountOnPrefix<64, false>(words.data(), count, last)), expected);
      ASSERT_EQ((CountOnPrefix<64, true>(words.data(), count, last)), expected);
      if (count <= 8) {
        ASSERT_EQ((CountOnPrefix<8, false>(words.data(), count, last)), expected);
        ASSERT_EQ((CountOnPrefix<8, true>(words.data(), count, last)), expected);
      }
      if (count < words.size()) {
        expected += Bonxai::Mask::CountOn(words[count]);
      }
    }
  }
}
