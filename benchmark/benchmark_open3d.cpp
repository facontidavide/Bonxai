#include <benchmark/benchmark.h>
#include <open3d/geometry/Octree.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/VoxelGrid.h>
#include "benchmark_utils.hpp"

static void Open3D_Octree_Create(benchmark::State& state)
{
  auto cloud = ReadCloud();

  std::vector<Eigen::Vector3d> points_vector;
  for (const auto& point : *cloud)
  {
    points_vector.push_back({ point.x, point.y, point.z });
  }
  open3d::geometry::PointCloud pcl(points_vector);

  for (auto _ : state)
  {
    open3d::geometry::Octree octree;
    octree.ConvertFromPointCloud(pcl);
  }
}

static void Open3D_Octree_Update(benchmark::State&)
{
}

static void Open3D_Octree_IterateAllCells(benchmark::State&)
{
}

// Register the function as a benchmark
BENCHMARK(Open3D_Octree_Create);
BENCHMARK(Open3D_Octree_Update);
BENCHMARK(Open3D_Octree_IterateAllCells);
