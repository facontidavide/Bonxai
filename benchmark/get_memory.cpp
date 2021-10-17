#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <openvdb/openvdb.h>
#include "benchmark_utils.hpp"
#include "treexy/treexy.hpp"


int main(int argc, char** argv)
{
  std::string filename = "data/rgbd.pcd";
  if( argc == 2 )
  {
    filename = argv[1];
  }

  auto cloud = ReadCloud(filename);

  octomap::OcTree octree(VOXEL_RESOLUTION);
  for (const auto& point : *cloud)
  {
    octomap::point3d endpoint(point.x, point.y, point.z);
    octree.updateNode(endpoint, true);
  }
  //----------------------

  openvdb::initialize();
  double INV_RES = 1.0 / VOXEL_RESOLUTION;

  openvdb::Int32Grid::Ptr vdb_grid = openvdb::Int32Grid::create();
  openvdb::Int32Grid::Accessor accessor = vdb_grid->getAccessor();
  vdb_grid->setGridClass(openvdb::GRID_LEVEL_SET);

  for (const auto& point : *cloud)
  {
    openvdb::math::Vec3<float> v(point.x * INV_RES,
                                 point.y * INV_RES,
                                 point.z * INV_RES);
    accessor.setValue(openvdb::Coord::floor(v), 42);
  }
  //----------------------
  Treexy::VoxelGrid<int> grid(VOXEL_RESOLUTION);
  auto t_accessor = grid.createAccessor();

  for (const auto& point : *cloud)
  {
    auto coord = grid.posToCoord( point.x, point.y, point.z );
    t_accessor.setValue(coord, 42);
  }
  //----------------------

  std::cout << "Octomap: " << octree.memoryUsage() << std::endl;
  std::cout << "OpenVDB: " << vdb_grid->memUsage() << std::endl;
  std::cout << "Treexy: " << grid.getMemoryUsage() << std::endl;

  return 0;
}
