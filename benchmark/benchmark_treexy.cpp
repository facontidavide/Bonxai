#include <benchmark/benchmark.h>
#include "benchmark_utils.hpp"
#include "treexy/treexy.hpp"
#include "treexy/serialization.hpp"

using namespace Treexy;

static void Treexy_Create(benchmark::State& state)
{
  auto cloud = ReadCloud();

  for (auto _ : state)
  {
    VoxelGrid<uint32_t> grid(VOXEL_RESOLUTION);
    auto accessor = grid.createAccessor();

    for (const auto& point : *cloud)
    {
      auto coord = grid.posToCoord(point.x, point.y, point.z);
      accessor.setValue(coord, 42);
    }
  }
}

static void Treexy_Update(benchmark::State& state)
{
  auto cloud = ReadCloud();
  VoxelGrid<uint32_t> grid(VOXEL_RESOLUTION);

  {
    auto accessor = grid.createAccessor();
    for (const auto& point : *cloud)
    {
      auto coord = grid.posToCoord(point.x, point.y, point.z);
      accessor.setValue(coord, 42);
    }
  }

  for (auto _ : state)
  {
    auto accessor = grid.createAccessor();

    for (const auto& point : *cloud)
    {
      auto coord = grid.posToCoord(point.x, point.y, point.z);
      accessor.setValue(coord, 42);
    }
    // std::cout <<"Update misses: " << accessor.cache_misses << std::endl;
  }
}

static void Treexy_IterateAllCells(benchmark::State& state)
{
  auto cloud = ReadCloud();

  VoxelGrid<uint32_t> grid(VOXEL_RESOLUTION);

  {
    auto accessor = grid.createAccessor();
    for (const auto& point : *cloud)
    {
      auto coord = grid.posToCoord(point.x, point.y, point.z);
      accessor.setValue(coord, 42);
    }
  }

  long count = 0;
  for (auto _ : state)
  {
    auto visitor = [&](uint32_t&, const CoordT&) { count++; };
    grid.forEachCell(visitor);
  }
}

// Register the function as a benchmark
BENCHMARK(Treexy_Create);
BENCHMARK(Treexy_Update);
BENCHMARK(Treexy_IterateAllCells);


// Run the benchmark
BENCHMARK_MAIN();
