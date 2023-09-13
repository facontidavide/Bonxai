#include <benchmark/benchmark.h>
#include "benchmark_utils.hpp"
#include "bonxai/bonxai.hpp"

using namespace Bonxai;

static void Bonxai_Create(benchmark::State& state)
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

static void Bonxai_Update(benchmark::State& state)
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

static void Bonxai_IterateAllCells(benchmark::State& state)
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
    auto visitor = [&](uint32_t&, const CoordT&) {
      benchmark::DoNotOptimize(count++);
    };
    grid.forEachCell(visitor);
  }
}

// Register the function as a benchmark
BENCHMARK(Bonxai_Create);
BENCHMARK(Bonxai_Update);
BENCHMARK(Bonxai_IterateAllCells);


// Run the benchmark
BENCHMARK_MAIN();
