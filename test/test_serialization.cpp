#include "treexy/treexy.hpp"
#include "treexy/serialization.hpp"
#include <fstream>

int main()
{
  const double VOXEL_RESOLUTION = 0.1;

  Treexy::VoxelGrid<int> grid(VOXEL_RESOLUTION);
  auto accessor = grid.createAccessor();

  int count = 0;
  for (double x = -0.4; x < 0.4; x += VOXEL_RESOLUTION)
  {
    for (double y = -0.4; y < 0.4; y += VOXEL_RESOLUTION)
    {
      for (double z = -0.4; z < 0.4; z += VOXEL_RESOLUTION)
      {
        accessor.setValue(grid.posToCoord(x, y, z), count++);
      }
    }
  }

  std::ofstream ofile("box.txy", std::ios::binary);
  Treexy::Serialize(ofile, grid);
  ofile.close();

  std::ifstream ifile("box.txy", std::ios::binary);
  auto new_grid = Treexy::Deserialize<int>(ifile);
  ifile.close();

  std::cout << "Original grid memory: " << grid.getMemoryUsage() << std::endl;
  std::cout << "New grid memory: " << new_grid.getMemoryUsage() << std::endl;

  auto new_accessor = new_grid.createAccessor();

  bool everything_fine = true;

  count = 0;
  for (double x = -0.4; x < 0.4; x += VOXEL_RESOLUTION)
  {
    for (double y = -0.4; y < 0.4; y += VOXEL_RESOLUTION)
    {
      for (double z = -0.4; z < 0.4; z += VOXEL_RESOLUTION)
      {
        auto value_ptr = new_accessor.value(grid.posToCoord(x, y, z));
        if( !value_ptr || *value_ptr != count )
        {
          std::cout << " Problem at cell " << x << " " << y << " " << z << std::endl;
          everything_fine = false;
        }
        count++;
      }
    }
  }
  if( everything_fine )
  {
    std::cout << "Round trip looks good!" << std::endl;
  }

  return 0;
}
