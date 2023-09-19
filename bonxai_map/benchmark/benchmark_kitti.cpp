#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <set>
#include <filesystem>
#include <chrono>
#include <Eigen/Geometry>

#include "bonxai_map/pcl_utils.hpp"
#include "bonxai_map/probabilistic_map.hpp"
#include "cxxopt/cxxopts.hpp"

namespace fs = std::filesystem;

long ToMsec(std::chrono::system_clock::duration const& dur)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
}

Eigen::Isometry3f ReadCalibration(const std::string& calibration_file)
{
  if(!std::filesystem::exists(calibration_file))
  {
    throw std::runtime_error("Calibration file not found");
  }

  std::ifstream input(calibration_file);
  std::string line;

  std::string header;

  Eigen::Isometry3f calib;

  while(std::getline(input, line))
  {
    Eigen::Matrix3f rot;
    Eigen::Vector3f pos;

    std::istringstream ss(line);
    ss  >> header
        >> rot(0,0) >> rot(0,1) >> rot(0,2)  >> pos(0)
        >> rot(1,0) >> rot(1,1) >> rot(1,2)  >> pos(1)
        >> rot(2,0) >> rot(2,1) >> rot(2,2)  >> pos(2);

    if(header == "Tr:")
    {
      calib = Eigen::Translation3f(pos) * Eigen::Quaternionf(rot);
      return calib;
    }
  }

  throw std::runtime_error("Calibration value not found");
}

std::vector<Eigen::Isometry3f> ReadPoses(const std::string& poses_file)
{
  if(!std::filesystem::exists(poses_file))
  {
    throw std::runtime_error("Calibration file not found");
  }

  std::ifstream input(poses_file);
  std::string line;

  std::vector<Eigen::Isometry3f> poses;

  while(std::getline(input, line))
  {
    Eigen::Matrix3f rot;
    Eigen::Vector3f pos;

    std::istringstream ss(line);
    ss  >> rot(0,0) >> rot(0,1) >> rot(0,2)  >> pos(0)
        >> rot(1,0) >> rot(1,1) >> rot(1,2)  >> pos(1)
        >> rot(2,0) >> rot(2,1) >> rot(2,2)  >> pos(2);

    poses.emplace_back(Eigen::Translation3f(pos) * Eigen::Quaternionf(rot));
  }
  return poses;
}

template <class PointCloudT>
void ReadPointcloud(const std::string& cloud_file,
                    const Eigen::Isometry3f& transform,
                    PointCloudT& points)
{
  std::fstream input(cloud_file, std::ios::in | std::ios::binary);

  points.clear();
  while (input.good() && !input.eof())
  {
    Eigen::Vector3f point;
    float intensity;
    input.read((char *) &point.x(), sizeof(float));
    input.read((char *) &point.y(), sizeof(float));
    input.read((char *) &point.z(), sizeof(float));
    input.read((char *) &intensity, sizeof(float));

    // apply transform first
    const Eigen::Vector3f p = transform * point;
    points.push_back( {p.x(), p.y(), p.z()} );
  }
}


//-----------------------------------------------------------
int main(int argc, char** argv)
{
  cxxopts::Options options("Kitti benchmarks", "ctomap VS Bonxai");

  options.add_options()
      ("clouds", "Pointcloud folder path", cxxopts::value<std::string>())
      ("calib", "Calibration file path", cxxopts::value<std::string>())
      ("poses", "Poses file path", cxxopts::value<std::string>())
      ("max_files", "Max files to process", cxxopts::value<size_t>()->default_value("1000"))
      ("max_dist", "Max distance in meters", cxxopts::value<double>()->default_value("25.0"))
      ("voxel_size", "Voxel size in meters", cxxopts::value<double>()->default_value("0.2"))
      ("skip_octree", "Do not compute the octree", cxxopts::value<bool>()->default_value("false"))
      ;
  const auto options_res = options.parse(argc, argv);

  const auto velodyne_path = options_res["clouds"].as<std::string>();
  const auto poses_file = options_res["poses"].as<std::string>();
  const auto calibration_file = options_res["calib"].as<std::string>();
  const auto max_distance = options_res["max_dist"].as<double>();
  auto voxel_size = options_res["voxel_size"].as<double>();
  auto max_pointclouds = options_res["max_files"].as<size_t>();
  const auto skip_octree = options_res["skip_octree"].as<bool>();

  if (options_res.count("pc") || velodyne_path.empty() ||
      poses_file.empty() || calibration_file.empty())
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  const auto calibration_transform = ReadCalibration(calibration_file);
  const auto poses = ReadPoses(poses_file);

  long total_time_octree = 0;
  long total_time_bonxai = 0;

  std::vector<std::string> cloud_filenames;
  for (const auto& entry : fs::directory_iterator(velodyne_path))
  {
    cloud_filenames.push_back(entry.path().generic_string());
  }

  max_pointclouds = std::min(max_pointclouds, cloud_filenames.size());

  std::sort(cloud_filenames.begin(), cloud_filenames.end());
  cloud_filenames.resize(max_pointclouds);

  //---------------------------------------
  octomap::OcTree octree(voxel_size);
  Bonxai::ProbabilisticMap bonxai_map(voxel_size);

  for (size_t count = 0; count < cloud_filenames.size(); count++)
  {
    const auto& filename = cloud_filenames[count];
    const Eigen::Isometry3f transform = poses[count] * calibration_transform;

    const Eigen::Vector3f origin(transform.translation());
    std::vector<Eigen::Vector3f> pointcloud;
    ReadPointcloud(filename, transform, pointcloud);

    const auto t1 = std::chrono::system_clock::now();

    bonxai_map.insertPointCloud(pointcloud, origin, max_distance);

    const auto diff = ToMsec(std::chrono::system_clock::now() - t1);
    std::cout << "[" << filename << "] bonxai time: " << diff << " ms" << std::endl;
    total_time_bonxai += diff;
  }

  printf("average time bonxai: %.1f ms\n",
         double(total_time_bonxai) / double(max_pointclouds));

  std::vector<Eigen::Vector3d> bonxai_result;
  bonxai_map.getOccupiedVoxels(bonxai_result);
  std::vector<Bonxai::CoordT> free_coords;
  bonxai_map.getFreeVoxels(free_coords);
  std::cout << "free cells: " << free_coords.size() << std::endl;
  std::cout << "bonxai_result.pcd contains " << bonxai_result.size()
            << " points\n" << std::endl;

  if(!bonxai_result.empty())
  {
    Bonxai::WritePointsFromPCD("bonxai_result.pcd", bonxai_result);
  }

  std::cout << "//-----------------------------------------\n";
  //----------------------------------------------------

  if(!skip_octree)
  {
    for (size_t count = 0; count < cloud_filenames.size(); count++)
    {
      const auto& filename = cloud_filenames[count];
      const Eigen::Isometry3f transform = poses[count] * calibration_transform;

      const octomap::point3d origin(transform.translation().x(),
                                    transform.translation().y(),
                                    transform.translation().z());
      octomap::Pointcloud pointcloud;
      ReadPointcloud(filename, transform, pointcloud);

      const auto t1 = std::chrono::system_clock::now();

      octree.insertPointCloud(pointcloud, origin, max_distance, false, true);

      const auto diff = ToMsec(std::chrono::system_clock::now() - t1);
      std::cout << "[" << filename << "] octree time: " << diff << " ms" << std::endl;
      total_time_octree += diff;
    }

    printf("average time octree: %.1f ms\n",
           double(total_time_octree) / double(max_pointclouds));

    std::vector<Eigen::Vector3d> octree_result;
    int free_cell_count = 0;
    for (auto it = octree.begin(), end = octree.end(); it != end; ++it)
    {
      if (octree.isNodeOccupied(*it)){
        octree_result.push_back( {it.getX(), it.getY(), it.getZ()} );
      }
      else {
        free_cell_count++;
      }
    }
    std::cout << "free cells: " << free_cell_count << std::endl;
    std::cout << "octomap_result.pcd contains " << octree_result.size()
              << " points\n" << std::endl;
    if(!octree_result.empty())
    {
      Bonxai::WritePointsFromPCD("octomap_result.pcd", octree_result);
    }

    std::cout << "\nMemory used. octree: " << int(octree.memoryUsage() / 1000)
              << " Kb / bonxai: " << int(bonxai_map.grid().memUsage() / 1000)
              << " Kb" << std::endl;
    printf("speed up: %.1f X\n", double(total_time_octree) / double(total_time_bonxai));
  }

  return 0;
}
