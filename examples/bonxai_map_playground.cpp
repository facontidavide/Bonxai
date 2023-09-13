#include <octomap/OcTree.h>
#include "bonxai_map/pcl_utils.hpp"
#include "bonxai_map/pointcloud.hpp"
#include "pcl/point_types.h"

int main()
{
  using namespace Bonxai;

  Point3D testA = Eigen::Vector3d(1,2,3);
//  Point3D testB = pcl::PointXYZ(1,2,3);

  std::vector<CoordT> points;
  ComputeRay({0, 0, 0}, {0.25, 0.12, 1.6}, 0.02, points);
  WritePointsFromPCD("point_to_point.pcd", points);

  std::cout << "points bonxai: " << points.size() << std::endl;

  octomap::OcTree octree(0.02);
  octomap::KeyRay ray;
  octree.computeRayKeys({0,0,0}, {0.25, 0.12, 1.6}, ray);

  std::cout << "points octree: " << points.size() << std::endl;

  return 0;
}
