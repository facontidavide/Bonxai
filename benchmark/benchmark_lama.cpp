#include <benchmark/benchmark.h>
#include "benchmark_utils.hpp"

#include <lama/sdm/simple_occupancy_map.h>

static void Lama_Create(benchmark::State& state)
{
  auto cloud = ReadCloud();

  for (auto _ : state)
  {
    lama::SimpleOccupancyMap grid(VOXEL_RESOLUTION);

    for (const auto& point : *cloud)
    {
      Eigen::Vector3d coord(point.x, point.y, point.z);
      grid.setOccupied(coord);
    }
  }
}

static void Lama_Update(benchmark::State& state)
{
  auto cloud = ReadCloud();

  lama::SimpleOccupancyMap grid(VOXEL_RESOLUTION);

  for (const auto& point : *cloud)
  {
    Eigen::Vector3d coord(point.x, point.y, point.z);
    grid.setOccupied(coord);
  }

  for (auto _ : state)
  {
    for (const auto& point : *cloud)
    {
      Eigen::Vector3d coord(point.x, point.y, point.z);
      grid.setOccupied(coord);
    }
  }
}

static void Lama_IterateAllCells(benchmark::State& state)
{
  auto cloud = ReadCloud();

  lama::SimpleOccupancyMap grid(VOXEL_RESOLUTION);

  {
    for (const auto& point : *cloud)
    {
      Eigen::Vector3d coord(point.x, point.y, point.z);
      grid.setOccupied(coord);
    }
  }

  long count = 0;
  for (auto _ : state)
  {
    auto visitor = [&](const Eigen::Vector3ui&) { count++; };
    grid.visit_all_cells(visitor);
  }
}

// Register the function as a benchmark
BENCHMARK(Lama_Create);
BENCHMARK(Lama_Update);
BENCHMARK(Lama_IterateAllCells);

