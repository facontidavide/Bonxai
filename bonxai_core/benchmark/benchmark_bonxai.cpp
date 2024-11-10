#include <benchmark/benchmark.h>

#include "benchmark_utils.hpp"
#include "bonxai/bonxai.hpp"

using namespace Bonxai;

static void Bonxai_Create(benchmark::State& state) {
  const auto& params = TestParameters[state.range(0)];
  auto cloud = ReadCloud(params.filename);

  for (auto _ : state) {
    VoxelGrid<uint32_t> grid(params.voxel_size);
    auto accessor = grid.createAccessor();

    for (const auto& point : *cloud) {
      auto coord = grid.posToCoord(point.x, point.y, point.z);
      accessor.setValue(coord, 42);
    }
  }
}

static void Bonxai_Update(benchmark::State& state) {
  const auto& params = TestParameters[state.range(0)];
  auto cloud = ReadCloud(params.filename);
  VoxelGrid<uint32_t> grid(params.voxel_size);

  {
    auto accessor = grid.createAccessor();
    for (const auto& point : *cloud) {
      auto coord = grid.posToCoord(point.x, point.y, point.z);
      accessor.setValue(coord, 42);
    }
  }

  for (auto _ : state) {
    auto accessor = grid.createAccessor();

    for (const auto& point : *cloud) {
      auto coord = grid.posToCoord(point.x, point.y, point.z);
      accessor.setValue(coord, 42);
    }
  }
}

static void Bonxai_IterateAllCells(benchmark::State& state) {
  const auto& params = TestParameters[state.range(0)];
  auto cloud = ReadCloud(params.filename);

  VoxelGrid<uint32_t> grid(params.voxel_size);

  {
    auto accessor = grid.createAccessor();
    for (const auto& point : *cloud) {
      auto coord = grid.posToCoord(point.x, point.y, point.z);
      accessor.setValue(coord, 42);
    }
  }

  long count = 0;
  for (auto _ : state) {
    auto visitor = [&](uint32_t&, const CoordT&) { benchmark::DoNotOptimize(count++); };
    grid.forEachCell(visitor);
  }
}

// Register the function as a benchmark
BENCHMARK(Bonxai_Create)->Arg(0)->Arg(1)->MinTime(1);
BENCHMARK(Bonxai_Update)->Arg(0)->Arg(1)->MinTime(1);
BENCHMARK(Bonxai_IterateAllCells)->Arg(0)->Arg(1)->MinTime(1);

// Run the benchmark
BENCHMARK_MAIN();
