#include "treexy/treexy.hpp"
#include "treexy/serialization.hpp"
#include <fstream>

int main()
{
  const double VOXEL_RESOLUTION = 0.1;

  Treexy::VoxelGrid<int> grid(VOXEL_RESOLUTION);
  auto accessor = grid.createAccessor();

  for (double x = 0; x < 0.4; x += VOXEL_RESOLUTION)
  {
    for (double y = 0; y < 0.4; y += VOXEL_RESOLUTION)
    {
      for (double z = 0; z < 0.4; z += VOXEL_RESOLUTION)
      {
        accessor.setValue(grid.posToCoord(x, y, z), 42);
      }
    }
  }

  std::ofstream ofile("box.txy", std::ios::binary);
  Treexy::Serialize(ofile, grid);
  ofile.close();

  std::ifstream ifile("box.txy", std::ios::binary);
  auto new_grid = Treexy::Deserialize<int>(ifile);
  ifile.close();

  return 0;
}
