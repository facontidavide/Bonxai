#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <filesystem>

struct Parameters {
  double voxel_size;
  std::string filename;
};

static Parameters TestParameters[] = {
    {0.01, "table_scene_mug_rgb.pcd"},
    {0.05, "room_scan.pcd"},
};

//----------------------------------------

template <typename PointT = pcl::PointXYZ>
inline typename pcl::PointCloud<PointT>::Ptr ReadCloud(const std::string& filename) {
  auto path = std::filesystem::path(DATA_PATH);
  auto full_path = (path / filename).generic_string();

  if (!std::filesystem::exists(full_path)) {
    std::cout << "File not found: " << full_path << std::endl;
    exit(-1);
  }
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT>(full_path, *cloud) == -1) {
    std::cout << "Problem loading file " << full_path << std::endl;
    exit(-1);
  }
  return cloud;
}
