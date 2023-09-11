#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <filesystem>

const double VOXEL_RESOLUTION = 0.01;

inline pcl::PointCloud<pcl::PointXYZ>::Ptr
ReadCloud(const std::string& filename = "room_scan.pcd")
{
  auto path = std::filesystem::path(DATA_PATH);
  auto full_path = (path / filename).generic_string();

  if(!std::filesystem::exists(full_path))
  {
    std::cout << "File not found: " << full_path << std::endl;
    exit(-1);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(full_path, *cloud) == -1)
  {
    std::cout << "Problem loading file " << full_path << std::endl;
    exit(-1);
  }
  return cloud;
}

