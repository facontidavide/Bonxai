#include <benchmark/benchmark.h>
#include <openvdb/openvdb.h>
#include "benchmark_utils.hpp"

// To make a fair comparison, use Log2DIM values similar to Bonxai
using TreeType = openvdb::tree::Tree4<int32_t, 2, 2, 3>::Type;
using GridType = openvdb::Grid<TreeType>;

inline openvdb::Coord GetCoord(float x, float y, float z)
{
  static float INV_RES = 1.0 / VOXEL_RESOLUTION;

  return openvdb::Coord(static_cast<int32_t>(x * INV_RES) - std::signbit(x),
                        static_cast<int32_t>(y * INV_RES) - std::signbit(y),
                        static_cast<int32_t>(z * INV_RES) - std::signbit(z));
}

static void OpenVDB_Create(benchmark::State& state)
{
  openvdb::initialize();

  auto cloud = ReadCloud();

  for (auto _ : state)
  {
    GridType::Ptr grid = GridType::create();
    GridType::Accessor accessor = grid->getAccessor();
    grid->setGridClass(openvdb::GRID_LEVEL_SET);

    for (const auto& point : *cloud)
    {
      accessor.setValue(GetCoord(point.x, point.y, point.z), 42);
    }
  }
}

static void OpenVDB_Update(benchmark::State& state)
{
  auto cloud = ReadCloud();

  GridType::Ptr grid = GridType::create();
  grid->setGridClass(openvdb::GRID_LEVEL_SET);

  {
    GridType::Accessor accessor = grid->getAccessor();
    for (const auto& point : *cloud)
    {
      accessor.setValue(GetCoord(point.x, point.y, point.z), 42);
    }
  }

  for (auto _ : state)
  {
    GridType::Accessor accessor = grid->getAccessor();
    for (const auto& point : *cloud)
    {
      accessor.setValue(GetCoord(point.x, point.y, point.z), 42);
    }
  }
}

static void OpenVDB_IterateAllCells(benchmark::State& state)
{
  auto cloud = ReadCloud();

  GridType::Ptr grid = GridType::create();
  grid->setGridClass(openvdb::GRID_LEVEL_SET);
  GridType::Accessor accessor = grid->getAccessor();

  for (const auto& point : *cloud)
  {
    accessor.setValue(GetCoord(point.x, point.y, point.z), 42);
  }

  openvdb::tree::LeafManager<TreeType> leafManager(grid->tree());

  auto visitor = [&](const TreeType::LeafNodeType& leaf, size_t /*idx*/)
  {
    for (auto iter = leaf.beginValueOn(); iter; ++iter)
    {
      benchmark::DoNotOptimize(iter.getCoord());
      benchmark::DoNotOptimize(iter.getValue());
    }
  };

  for (auto _ : state)
  {
    leafManager.foreach(visitor, false);
  }
}

// Register the function as a benchmark
BENCHMARK(OpenVDB_Create);
BENCHMARK(OpenVDB_Update);
BENCHMARK(OpenVDB_IterateAllCells);
