#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include "bonxai_map/pcl_utils.hpp"
#include "bonxai_map/pointcloud.hpp"

int main()
{
  using namespace Bonxai;


  std::vector<CoordT> points;
  ComputeRay({0, 0, 0}, {1,2,3}, 0.025, points);
  WritePointsFromPCD("point_to_point.pcd", points);

  return 0;
}
