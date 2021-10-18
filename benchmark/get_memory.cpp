#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <openvdb/openvdb.h>
#include "benchmark_utils.hpp"
#include "treexy/treexy.hpp"

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  if (argc == 2)
  {
    cloud = ReadCloud(argv[1]);
  }
  else
  {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    for (double x = 0; x < 1.0; x += VOXEL_RESOLUTION)
    {
      for (double y = 0; y < 1.0; y += VOXEL_RESOLUTION)
      {
        for (double z = 0; z < 1.0; z += VOXEL_RESOLUTION)
        {
          cloud->push_back(pcl::PointXYZ(x, y, z));
        }
      }
    }
  }

  octomap::OcTree octree(VOXEL_RESOLUTION);
  for (const auto& point : *cloud)
  {
    octomap::point3d endpoint(point.x, point.y, point.z);
    octree.updateNode(endpoint, true);
  }
  //----------------------

  openvdb::initialize();
  double INV_RES = 1.0 / VOXEL_RESOLUTION;

  using GridType = openvdb::Grid<openvdb::tree::Tree4<int32_t, 2, 2, 3>::Type>;

  GridType::Ptr vdb_grid = GridType::create();
  GridType::Accessor accessor = vdb_grid->getAccessor();
  vdb_grid->setGridClass(openvdb::GRID_LEVEL_SET);

  for (const auto& point : *cloud)
  {
    openvdb::math::Vec3<float> v(
        point.x * INV_RES, point.y * INV_RES, point.z * INV_RES);
    accessor.setValue(openvdb::Coord::floor(v), 42);
  }
  //----------------------
  Treexy::VoxelGrid<int> grid(VOXEL_RESOLUTION);
  auto t_accessor = grid.createAccessor();

  for (const auto& point : *cloud)
  {
    auto coord = grid.posToCoord(point.x, point.y, point.z);
    t_accessor.setValue(coord, 42);
  }
  //----------------------

  std::cout << "Octomap: " << octree.memoryUsage() << std::endl;
  std::cout << "OpenVDB: " << vdb_grid->memUsage() << std::endl;
  std::cout << "Treexy:  " << grid.getMemoryUsage() << std::endl;

  return 0;
}
