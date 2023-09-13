#include <benchmark/benchmark.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include "benchmark_utils.hpp"

static void Octomap_Create(benchmark::State& state)
{
  auto cloud = ReadCloud();

  for (auto _ : state)
  {
    octomap::OcTree tree(VOXEL_RESOLUTION);
    for (const auto& point : *cloud)
    {
      octomap::point3d endpoint(point.x, point.y, point.z);
      tree.updateNode(endpoint, true);  // integrate 'occupied' measurement
    }
  }
}

static void Octomap_Update(benchmark::State& state)
{
  auto cloud = ReadCloud();

  octomap::OcTree tree(VOXEL_RESOLUTION);
  for (const auto& point : *cloud)
  {
    octomap::point3d endpoint(point.x, point.y, point.z);
    tree.updateNode(endpoint, true);  // integrate 'occupied' measurement
  }
  for (auto _ : state)
  {
    for (const auto& point : *cloud)
    {
      octomap::point3d endpoint(point.x, point.y, point.z);
      tree.updateNode(endpoint, true);  // integrate 'occupied' measurement
    }
  }
}

static void Octomap_IterateAllCells(benchmark::State& state)
{
  auto cloud = ReadCloud();

  octomap::OcTree tree(VOXEL_RESOLUTION);
  for (const auto& point : *cloud)
  {
    octomap::point3d endpoint(point.x, point.y, point.z);
    tree.updateNode(endpoint, true);  // integrate 'occupied' measurement
  }

  int count = 0;
  for (auto _ : state)
  {
    count = 0;
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
    {
      benchmark::DoNotOptimize(it.getCoordinate());
      benchmark::DoNotOptimize(it->getValue());
      count++;
    }
  }
}

// Register the function as a benchmark
BENCHMARK(Octomap_Create)->MinTime(2);
BENCHMARK(Octomap_Update)->MinTime(2);
BENCHMARK(Octomap_IterateAllCells)->MinTime(2);

