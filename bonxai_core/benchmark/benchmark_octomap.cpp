#include <benchmark/benchmark.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include "benchmark_utils.hpp"

static void Octomap_Create(benchmark::State& state) {
  const auto& params = TestParameters[state.range(0)];
  auto cloud = ReadCloud(params.filename);

  for (auto _ : state) {
    octomap::OcTree tree(params.voxel_size);
    for (const auto& point : *cloud) {
      octomap::point3d endpoint(point.x, point.y, point.z);
      tree.updateNode(endpoint, true);  // integrate 'occupied' measurement
    }
  }
}

static void Octomap_Update(benchmark::State& state) {
  const auto& params = TestParameters[state.range(0)];
  auto cloud = ReadCloud(params.filename);

  octomap::OcTree tree(params.voxel_size);
  for (const auto& point : *cloud) {
    octomap::point3d endpoint(point.x, point.y, point.z);
    tree.updateNode(endpoint, true);  // integrate 'occupied' measurement
  }
  for (auto _ : state) {
    for (const auto& point : *cloud) {
      octomap::point3d endpoint(point.x, point.y, point.z);
      tree.updateNode(endpoint, true);  // integrate 'occupied' measurement
    }
  }
}

static void Octomap_IterateAllCells(benchmark::State& state) {
  const auto& params = TestParameters[state.range(0)];
  auto cloud = ReadCloud(params.filename);

  octomap::OcTree tree(params.voxel_size);
  for (const auto& point : *cloud) {
    octomap::point3d endpoint(point.x, point.y, point.z);
    tree.updateNode(endpoint, true);  // integrate 'occupied' measurement
  }

  int count = 0;
  for (auto _ : state) {
    count = 0;
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
      benchmark::DoNotOptimize(it.getCoordinate());
      benchmark::DoNotOptimize(it->getValue());
      count++;
    }
  }
}

// Register the function as a benchmark
BENCHMARK(Octomap_Create)->Arg(0)->Arg(1)->MinTime(1);
BENCHMARK(Octomap_Update)->Arg(0)->Arg(1)->MinTime(1);
BENCHMARK(Octomap_IterateAllCells)->Arg(0)->Arg(1)->MinTime(1);
