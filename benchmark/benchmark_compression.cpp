#include <benchmark/benchmark.h>
#include "benchmark_utils.hpp"
#include "bonxai/bonxai.hpp"
#include "bonxai/serialization.hpp"
#include <pcl/compression/octree_pointcloud_compression.h>

#include "lz4.h"

using namespace Bonxai;

static const double COMPRESSION_RES = 0.0025;

static void Bonxai_Compress(benchmark::State& state)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("data/color_scene.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    exit(-1);
  }

  std::vector<char> compressed_data;

  for (auto _ : state)
  {
    VoxelGrid<uint16_t> grid(COMPRESSION_RES);
    auto accessor = grid.createAccessor();

    for (const auto& point : *cloud)
    {
      auto coord = grid.posToCoord(point.x, point.y, point.z);
      uint16_t col = uint16_t(point.r >> 3) |
                     (uint16_t(point.g >> 2 ) << 5 ) |
                     (uint16_t(point.b >> 3 ) << 11 );
      accessor.setValue(coord, col);
    }

    std::ostringstream ofile(std::ios::binary);
    Bonxai::Serialize(ofile, grid);
    std::string serialized = ofile.str();

    const int src_size(serialized.size() + 1);
    const size_t max_dst_size = LZ4_compressBound(src_size);
    compressed_data.resize(max_dst_size);
    LZ4_compress_default(serialized.data(), compressed_data.data(), src_size, max_dst_size);
  }
}

static void PCL_Compress(benchmark::State& state)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("data/color_scene.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    exit(-1);
  }

  for (auto _ : state)
  {
    std::stringstream compressedData;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> encoder(
           pcl::io::MANUAL_CONFIGURATION, false, COMPRESSION_RES, COMPRESSION_RES, true, 20, true, 6);

    encoder.encodePointCloud (cloud, compressedData);
  }
}


// Register the function as a benchmark
BENCHMARK(Bonxai_Compress);
BENCHMARK(PCL_Compress);

