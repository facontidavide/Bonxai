#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <filesystem>

const double VOXEL_RESOLUTION = 0.01;

template <typename PointT = pcl::PointXYZ> inline
typename pcl::PointCloud<PointT>::Ptr
ReadCloud(const std::string& filename = "room_scan.pcd")
{
  auto path = std::filesystem::path(DATA_PATH);
  auto full_path = (path / filename).generic_string();

  if(!std::filesystem::exists(full_path))
  {
    std::cout << "File not found: " << full_path << std::endl;
    exit(-1);
  }
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT>(full_path, *cloud) == -1)
  {
    std::cout << "Problem loading file " << full_path << std::endl;
    exit(-1);
  }
  return cloud;
}

