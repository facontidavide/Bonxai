#include <benchmark/benchmark.h>
#include <filesystem>

#include "bonxai_map/probabilistic_map.hpp"
#include "bonxai_map/pcl_utils.hpp"
#include "octomap/OcTree.h"

using namespace Bonxai;

static const double voxel_res = 0.02;
static auto filepath = std::filesystem::path(DATA_PATH) / "room_scan.pcd";

static void Bonxai_ComputeRay(benchmark::State& state)
{
  std::vector<Eigen::Vector3d> points;
  ReadPointsFromPCD(filepath.generic_string(), points);

  std::vector<Bonxai::CoordT> ray;

  double inv_resolution = 1.0 / voxel_res;

  for (auto _ : state)
  {
    for(const auto& p: points)
    {
      const auto coord = Bonxai::PosToCoord(p, inv_resolution);
      Bonxai::ComputeRay({0,0,0}, coord, ray);
    }
  }
}

static void OctoMap_ComputeRay(benchmark::State& state)
{
  std::vector<Eigen::Vector3d> points;
  ReadPointsFromPCD(filepath.generic_string(), points);

  octomap::OcTree octree(voxel_res);
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
BENCHMARK(Bonxai_ComputeRay);
BENCHMARK(OctoMap_ComputeRay);

// Run the benchmark
BENCHMARK_MAIN();
