#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include "benchmark_utils.hpp"
#include "bonxai/bonxai.hpp"

int main(int argc, char** argv) {
  const double VOXEL_RESOLUTION = 0.02;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  if (argc == 2) {
    cloud = ReadCloud(argv[1]);
  } else {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    for (double x = 0; x < 1.0; x += VOXEL_RESOLUTION) {
      for (double y = 0; y < 1.0; y += VOXEL_RESOLUTION) {
        for (double z = 0; z < 1.0; z += VOXEL_RESOLUTION) {
          cloud->push_back(pcl::PointXYZ(x, y, z));
        }
      }
    }
  }

  octomap::OcTree octree(VOXEL_RESOLUTION);
  for (const auto& point : *cloud) {
    octomap::point3d endpoint(point.x, point.y, point.z);
    octree.updateNode(endpoint, true);
  }
  //----------------------
  Bonxai::VoxelGrid<int32_t> grid(VOXEL_RESOLUTION);
  auto t_accessor = grid.createAccessor();

  for (const auto& point : *cloud) {
    auto coord = grid.posToCoord(point.x, point.y, point.z);
    t_accessor.setValue(coord, 42);
  }
  //----------------------

  std::cout << "Octomap: " << octree.memoryUsage() << std::endl;
  std::cout << "Bonxai:  " << grid.memUsage() << std::endl;

  return 0;
}
