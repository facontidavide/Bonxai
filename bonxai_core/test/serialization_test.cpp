#include "bonxai/serialization.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <vector>

#include "bonxai/bonxai.hpp"

namespace {

struct Sample {
  Bonxai::CoordT coord;
  int value;
};

// Deliberately integer coordinates, including negatives and coordinates far
// from the origin, so that no floating point rounding enters the comparison.
std::vector<Sample> MakeSamples() {
  std::vector<Sample> samples;
  int value = 0;
  for (int x = -40; x <= 40; x += 3) {
    for (int y = -40; y <= 40; y += 7) {
      for (int z = -40; z <= 40; z += 11) {
        samples.push_back({{x, y, z}, value++});
      }
    }
  }
  samples.push_back({{100000, -100000, 50000}, value++});
  return samples;
}

template <typename GridT>
void Fill(GridT& grid, const std::vector<Sample>& samples) {
  auto accessor = grid.createAccessor();
  for (const auto& sample : samples) {
    accessor.setValue(sample.coord, sample.value);
  }
}

template <typename GridT>
void ExpectHolds(GridT& grid, const std::vector<Sample>& samples) {
  auto accessor = grid.createConstAccessor();
  for (const auto& sample : samples) {
    const int* value = accessor.value(sample.coord);
    ASSERT_NE(value, nullptr) << sample.coord.x << " " << sample.coord.y << " " << sample.coord.z;
    EXPECT_EQ(*value, sample.value);
  }
  EXPECT_EQ(grid.activeCellsCount(), samples.size());
}

template <typename GridT>
std::string ToString(const GridT& grid) {
  std::ostringstream out(std::ios::binary);
  Bonxai::Serialize(out, grid);
  return out.str();
}

}  // namespace

TEST(Serialization, RoundTripWithTheDefaultShape) {
  const auto samples = MakeSamples();
  Bonxai::VoxelGrid<int> grid(0.1);
  Fill(grid, samples);

  std::istringstream input(ToString(grid), std::ios::binary);
  char header[256];
  input.getline(header, 256);
  const auto info = Bonxai::GetHeaderInfo(header);
  EXPECT_EQ(info.inner_bits, grid.innetBits());
  EXPECT_EQ(info.leaf_bits, grid.leafBits());
  EXPECT_DOUBLE_EQ(info.resolution, grid.voxelSize());

  auto restored = Bonxai::Deserialize<int>(input, info);
  ExpectHolds(restored, samples);
}

// The shape of a grid read back from a file comes from its header, so it is not
// known when the code is compiled. This is what DynamicShape exists for, and
// why Deserialize defaults to it.
TEST(Serialization, RoundTripWithANonDefaultShape) {
  const auto samples = MakeSamples();
  for (const auto [inner_bits, leaf_bits] :
       {std::pair{1, 1}, std::pair{2, 3}, std::pair{3, 4}, std::pair{4, 2}, std::pair{2, 5}}) {
    Bonxai::VoxelGrid<int, Bonxai::DynamicShape> grid(0.1, inner_bits, leaf_bits);
    Fill(grid, samples);

    std::istringstream input(ToString(grid), std::ios::binary);
    char header[256];
    input.getline(header, 256);
    const auto info = Bonxai::GetHeaderInfo(header);
    EXPECT_EQ(int(info.inner_bits), inner_bits);
    EXPECT_EQ(int(info.leaf_bits), leaf_bits);

    auto restored = Bonxai::Deserialize<int>(input, info);
    EXPECT_EQ(restored.innetBits(), uint32_t(inner_bits));
    EXPECT_EQ(restored.leafBits(), uint32_t(leaf_bits));
    ExpectHolds(restored, samples);
  }
}

TEST(Serialization, ReadingIntoAMatchingStaticShapeWorks) {
  const auto samples = MakeSamples();
  Bonxai::VoxelGrid<int, Bonxai::StaticShape<3, 4>> grid(0.1, 3, 4);
  Fill(grid, samples);

  std::istringstream input(ToString(grid), std::ios::binary);
  char header[256];
  input.getline(header, 256);
  auto restored =
      Bonxai::Deserialize<int, Bonxai::StaticShape<3, 4>>(input, Bonxai::GetHeaderInfo(header));
  ExpectHolds(restored, samples);
}

TEST(Serialization, ReadingIntoAContradictoryStaticShapeThrows) {
  Bonxai::VoxelGrid<int, Bonxai::DynamicShape> grid(0.1, 3, 4);
  Fill(grid, MakeSamples());
  const std::string blob = ToString(grid);

  std::istringstream input(blob, std::ios::binary);
  char header[256];
  input.getline(header, 256);
  EXPECT_THROW(
      (Bonxai::Deserialize<int, Bonxai::StaticShape<2, 3>>(input, Bonxai::GetHeaderInfo(header))),
      std::runtime_error);
}

TEST(Serialization, TheDataTypeIsChecked) {
  Bonxai::VoxelGrid<int> grid(0.1);
  Fill(grid, MakeSamples());

  std::istringstream input(ToString(grid), std::ios::binary);
  char header[256];
  input.getline(header, 256);
  EXPECT_THROW(
      Bonxai::Deserialize<float>(input, Bonxai::GetHeaderInfo(header)), std::runtime_error);
}

TEST(Serialization, AnEmptyGridRoundTrips) {
  Bonxai::VoxelGrid<int> grid(0.25);
  std::istringstream input(ToString(grid), std::ios::binary);
  char header[256];
  input.getline(header, 256);
  auto restored = Bonxai::Deserialize<int>(input, Bonxai::GetHeaderInfo(header));
  EXPECT_EQ(restored.activeCellsCount(), 0u);
}
