#include "bonxai_map/pcl_utils.hpp"

#include <memory>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

template <typename PointT>
bool ReadPointsFromPCD_Impl(const std::string& filepath, std::vector<PointT>& points)
{
  if (!std::filesystem::exists(filepath))
  {
    throw std::runtime_error("File doesn't exist");
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud) == -1)
  {
    throw std::runtime_error("Problem loading file");
  }
  points.clear();
  points.reserve(cloud->points.size());
  for (const auto& p : cloud->points)
  {
    points.push_back({ p.x, p.y, p.z });
  }
  return true;
}

bool Bonxai::ReadPointsFromPCD(const std::string& filepath,
                               std::vector<Eigen::Vector3d>& points)
{
  return ReadPointsFromPCD_Impl(filepath, points);
}

bool Bonxai::ReadPointsFromPCD(const std::string& filepath,
                               std::vector<Bonxai::Point3D>& points)
{
  return ReadPointsFromPCD_Impl(filepath, points);
}

//-----------------------------------------------

inline pcl::PointXYZ toPointXYZ(const Eigen::Vector3d& p)
{
  return { float(p.x()), float(p.y()), float(p.z()) };
}

inline pcl::PointXYZ toPointXYZ(const Bonxai::Point3D& p)
{
  return { float(p.x), float(p.y), float(p.z) };
}

inline pcl::PointXYZ toPointXYZ(const Bonxai::CoordT& p)
{
  return { float(p.x), float(p.y), float(p.z) };
}

template <typename PointT>
void WritePointsFromPCD_Impl(const std::string& filepath,
                             const std::vector<PointT>& points)
{
  static pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.clear();
  cloud.points.reserve(points.size());
  for (const auto& p : points)
  {
    cloud.push_back(toPointXYZ(p));
  }
  pcl::io::savePCDFile(filepath, cloud);
}

void Bonxai::WritePointsFromPCD(const std::string& filepath,
                                const std::vector<Eigen::Vector3d>& points)
{
  WritePointsFromPCD_Impl(filepath, points);
}

void Bonxai::WritePointsFromPCD(const std::string& filepath,
                                const std::vector<Point3D>& points)
{
  WritePointsFromPCD_Impl(filepath, points);
}

void Bonxai::WritePointsFromPCD(const std::string& filepath,
                                const std::vector<CoordT>& points)
{
  WritePointsFromPCD_Impl(filepath, points);
}
