#include "bonxai/bonxai.hpp"
#include <iostream>

int main()
{
  const double VOXEL_RESOLUTION = 0.1;

  // when using Bonxai, you can pretend that you have an "infinite"
  // 3D matrix of voxels, where each voxel as a certain size (0.1 in this case)
  Bonxai::VoxelGrid<int> grid(VOXEL_RESOLUTION);

  // You can not access directly the voxels using methods of VoxelGrid<>.
  // You should create an "accessor" object instead.
  // IMPORTANT: reuse this object as much as possible, in particular inside loops!
  auto accessor = grid.createAccessor();

  // We will densily fill the voxels inside a cube with dimention 1.0 X 1.0 X 1.0
  // centered at the origin
  int count = 0;
  for (double x = -0.5; x < 0.5; x += VOXEL_RESOLUTION)
  {
    for (double y = -0.5; y < 0.5; y += VOXEL_RESOLUTION)
    {
      for (double z = -0.5; z < 0.5; z += VOXEL_RESOLUTION)
      {
        // convert a position in the 3D (double) space into
        // coordinates inside the grid (integers)
        const Bonxai::CoordT coord = grid.posToCoord(x, y, z);
        // set the value
        accessor.setValue(coord, count++);
      }
    }
  }
  //-------------------------------------------------
  // You can read the value of a voxel doing:
  int* origin_ptr = accessor.value(grid.posToCoord(0, 0, 0));
  // And you can modify it.
  *origin_ptr = 500;
  std::cout << "Value at (0, 0, 0): " << *origin_ptr << std::endl;

  // you can also use the method value to create voxels and get their pointer in a single step
  bool create_voxel_if_missing = true;
  int* far_voxel = accessor.value(grid.posToCoord(10, 10, 10), create_voxel_if_missing);
  (*far_voxel)++;
  // The value is initialized to int(), in this case 0
  std::cout << "Value at (10, 10, 10): " << *far_voxel << std::endl;

  //-------------------------------------------------
  // We need a way to iterate through all the voxels.
  // In Bonxai it is done with lambdas and calling  VoxelGrid::forEachCell
  auto mutableVisitor = [&grid, &accessor](int& value, const Bonxai::CoordT& coord)
  {
    // this visitor will:
    // - remove all the voxels with Z < -0.1
    // - set to 1 all the voxels with Z >= -0.1
    Bonxai::Point3D pos = grid.coordToPos(coord);
    if(pos.z < -0.1) {
      accessor.setCellOff(coord); // disable the voxel
    }
    else {
      value = 1; // overwite the value (mutable reference).
    }
  };
  grid.forEachCell(mutableVisitor);

  //-------------------------------------------------

  // When you have a const reference to the VoxelGrid or/and you want
  // non-mutable interfaces, you can do instead
  auto constVisitor = [](const int& value, const Bonxai::CoordT& coord)
  {
    if(value < 0)
    {
      throw std::runtime_error("unexpected");
    }
  };
  grid.forEachCell(constVisitor);

  auto constAccessor = grid.createConstAccessor();
  const int* cell = constAccessor.value(grid.posToCoord(0, 0, 0));

  std::cout << "\nValue at (0, 0, 0): "
            << ((cell == nullptr) ? std::string("nullptr") : std::to_string(*cell))
            << std::endl;

  cell = constAccessor.value(grid.posToCoord(0, 0, -0.2));
  std::cout << "Value at (0, 0, -0.2): "
            << ((cell == nullptr) ? std::string("nullptr") : std::to_string(*cell))
            << std::endl;

  return 0;
}
