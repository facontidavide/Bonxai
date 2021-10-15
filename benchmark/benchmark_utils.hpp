#ifndef BENCHMARK_UTILS_HPP
#define BENCHMARK_UTILS_HPP

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

const double VOXEL_RESOLUTION = 0.05;

inline pcl::PointCloud<pcl::PointXYZ>::Ptr
ReadCloud(const std::string& filename = "data/rgbd.pcd")
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    exit(-1);
  }
  return cloud;
}

#endif  // BENCHMARK_UTILS_HPP
