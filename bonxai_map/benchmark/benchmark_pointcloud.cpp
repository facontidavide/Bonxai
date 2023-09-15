#include <benchmark/benchmark.h>
#include <filesystem>

#include "bonxai_map/pointcloud.hpp"
#include "bonxai_map/pcl_utils.hpp"
#include "octomap/OcTree.h"

using namespace Bonxai;
/*
static void Bonxai_ComputeRay(benchmark::State& state)
{
  auto filepath = std::filesystem::path(DATA_PATH) / "room_scan.pcd";
  std::vector<Eigen::Vector3d> points;
  ReadPointsFromPCD(filepath.generic_string(), points);

  std::vector<Bonxai::CoordT> ray;

  for (auto _ : state)
  {
    for(const auto& p: points)
    {
      ray.clear();
      Bonxai::ComputeRay(Eigen::Vector3d::Zero(), p, 0.02, ray);
    }
  }
}
*/
static void OctoMap_ComputeRay(benchmark::State& state)
{
  auto filepath = std::filesystem::path(DATA_PATH) / "room_scan.pcd";
  std::vector<Eigen::Vector3d> points;
  ReadPointsFromPCD(filepath.generic_string(), points);

  octomap::OcTree octree(0.02);
  octomap::KeyRay ray;


  for (auto _ : state)
  {
    for(const auto& p: points)
    {
      ray.reset();
      octree.computeRayKeys({0,0,0}, { float(p.x()), float(p.y()), float(p.z())}, ray);
    }
  }
}

// Register the function as a benchmark
//BENCHMARK(Bonxai_ComputeRay);
BENCHMARK(OctoMap_ComputeRay);

// Run the benchmark
BENCHMARK_MAIN();
