#include <benchmark/benchmark.h>
#include <pcl/filters/voxel_grid.h>

#include "benchmark_utils.hpp"
#include "bonxai/bonxai.hpp"

using namespace Bonxai;

static const double COMPRESSION_RES = 0.02;

static void Bonxai_Downsample(benchmark::State& state) {
  auto cloud = ReadCloud<pcl::PointXYZ>("block_stack.pcd");
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  BinaryVoxelGrid grid(COMPRESSION_RES);

  for (auto _ : state) {
    grid.clear(Bonxai::SET_ALL_CELLS_OFF);
    auto accessor = grid.createAccessor();
    for (const auto& point : *cloud) {
      accessor.setCellOn(grid.posToCoord({point.x, point.y, point.z}));
    }
    cloud_filtered.clear();
    grid.forEachCell([&](const auto&, const Bonxai::CoordT& coord) {
      auto pos = grid.coordToPos(coord);
      cloud_filtered.push_back(pcl::PointXYZ(pos.x, pos.y, pos.z));
    });
  }
}

static void Bonxai_Downsample_FreshGrid(benchmark::State& state) {
  auto cloud = ReadCloud<pcl::PointXYZ>("block_stack.pcd");
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

  for (auto _ : state) {
    BinaryVoxelGrid grid(COMPRESSION_RES);

    auto accessor = grid.createAccessor();
    for (const auto& point : *cloud) {
      accessor.setCellOn(grid.posToCoord({point.x, point.y, point.z}));
    }
    cloud_filtered.clear();
    cloud_filtered.reserve(grid.activeCellsCount());
    grid.forEachCell([&](const auto&, const Bonxai::CoordT& coord) {
      auto pos = grid.coordToPos(coord);
      cloud_filtered.push_back(pcl::PointXYZ(pos.x, pos.y, pos.z));
    });
  }
}

static void PCL_Downsample(benchmark::State& state) {
  auto cloud = ReadCloud<pcl::PointXYZ>("room_scan.pcd");
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(COMPRESSION_RES, COMPRESSION_RES, COMPRESSION_RES);

  for (auto _ : state) {
    sor.filter(cloud_filtered);
  }
}

// Register the function as a benchmark
// BENCHMARK(Bonxai_Downsample);
BENCHMARK(Bonxai_Downsample_FreshGrid);
BENCHMARK(PCL_Downsample);

// Run the benchmark
BENCHMARK_MAIN();
