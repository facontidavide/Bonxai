#include "bonxai/bonxai.hpp"
#include <iostream>

int main()
{
  const double VOXEL_RESOLUTION = 0.1;

  Bonxai::VoxelGrid<int> grid(VOXEL_RESOLUTION);
  // to modify a grid, we need a mutable accessor
  auto accessor = grid.createAccessor();

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

  // To iterate throught all the cells, we can use visitors.
  auto mutableVisitor = [](int& value, const Bonxai::CoordT&)
  {
    value = 1;
  };
  auto& gridRef = grid;
  gridRef.forEachCell(mutableVisitor);

  // Const version of the visitor
  auto constVisitor = [](const int& value, const Bonxai::CoordT&)
  {
    if(value != 1)
    {
      throw std::runtime_error("unexpected");
    }
  };
  const auto& gridConstRef = grid;
  gridConstRef.forEachCell(constVisitor);

  auto const_accessor = gridConstRef.createConstAccessor();
  std::cout << "value at origing: "<< *(const_accessor.value({0, 0, 0})) << "\n";

  std::cout << "DONE\n";
  return 0;
}
