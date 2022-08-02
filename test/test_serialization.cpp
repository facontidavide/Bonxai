#include "bonxai/bonxai.hpp"
#include "bonxai/serialization.hpp"
#include <sstream>

int main()
{
  const double VOXEL_RESOLUTION = 0.1;

  Bonxai::VoxelGrid<int> grid(VOXEL_RESOLUTION);
  auto accessor = grid.createAccessor();

  auto value_ptr = accessor.value({});
  if(!value_ptr)
  {
    std::cout << "Empty as expected" << std::endl;
  }

  int count = 0;
  for (double x = -0.5; x < 0.5; x += VOXEL_RESOLUTION)
  {
    for (double y = -0.5; y < 0.5; y += VOXEL_RESOLUTION)
    {
      for (double z = -0.5; z < 0.5; z += VOXEL_RESOLUTION)
      {
        accessor.setValue(grid.posToCoord(x, y, z), count++);
      }
    }
  }

  std::ostringstream ofile(std::ios::binary);
  Bonxai::Serialize(ofile, grid);

  std::string msg = ofile.str();

  std::istringstream ifile(msg, std::ios::binary);

  char header[256];
  ifile.getline(header, 256);
  Bonxai::HeaderInfo info = Bonxai::GetHeaderInfo(header);
  auto new_grid = Bonxai::Deserialize<int>(ifile, info);

  std::cout << "Original grid memory: " << grid.memUsage() << std::endl;
  std::cout << "New grid memory: " << new_grid.memUsage() << std::endl;

  auto new_accessor = new_grid.createAccessor();

  bool everything_fine = true;

  count = 0;
  for (double x = -0.5; x < 0.5; x += VOXEL_RESOLUTION)
  {
    for (double y = -0.5; y < 0.5; y += VOXEL_RESOLUTION)
    {
      for (double z = -0.5; z < 0.5; z += VOXEL_RESOLUTION)
      {
        Bonxai::CoordT coord = grid.posToCoord(x, y, z);
        int* value_ptr = new_accessor.value(coord);
        if (!value_ptr || *value_ptr != count)
        {
          std::cout << " Problem at cell " << x << " " << y << " " << z << std::endl;
          everything_fine = false;
        }
        count++;
      }
    }
  }
  if (everything_fine)
  {
    std::cout << "Round trip looks good!" << std::endl;
  }

  return 0;
}
