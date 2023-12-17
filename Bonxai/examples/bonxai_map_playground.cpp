#include <octomap/OcTree.h>
#include "pcl/point_types.h"

#include "bonxai_map/probabilistic_map.hpp"
#include "bonxai_map/pcl_utils.hpp"

int main()
{
  using namespace Bonxai;

  // constants
  const double voxel_res = 0.02;
  const double inv_resolution = 1.0 / voxel_res;
  const Point3D test_point = Eigen::Vector3d{0.25, 0.12, 1.6};

  // run bonxai
  std::vector<Bonxai::CoordT> bonxai_ray;
  const auto coord = Bonxai::PosToCoord(test_point, inv_resolution);
  ComputeRay({0, 0, 0}, coord, bonxai_ray);
  std::cout << "points bonxai: " << bonxai_ray.size() << std::endl;

  std::vector<Point3D> points;
  for(const auto& ray_coord : bonxai_ray){
    points.push_back(CoordToPos(ray_coord, voxel_res));
  }
  WritePointsFromPCD("point_to_point.pcd", points);

  // run octree
  octomap::OcTree octree(voxel_res);
  octomap::KeyRay ray;
  octree.computeRayKeys({0,0,0}, {(float)test_point.x, (float)test_point.y, (float)test_point.z}, ray);
  std::cout << "points octree: " << ray.size() << std::endl;

  return 0;
}
