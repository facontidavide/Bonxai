#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <set>
#include <filesystem>
#include <chrono>
#include <Eigen/Geometry>

#include "bonxai_map/pcl_utils.hpp"

namespace fs = std::filesystem;

long ToMsec(std::chrono::system_clock::duration const& dur)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
}

std::vector<Eigen::Vector3d> ComputeOctomap(const std::vector<std::string>& filenames,
                                            const std::vector<Eigen::Matrix4f>& poses,
                                            double max_dist,
                                            double voxel_size)
{
  std::vector<Eigen::Vector3d> out;

  octomap::OcTree octree(voxel_size);

  octomap::Pointcloud pointcloud;

  size_t count = 0;

  Eigen::Matrix4f calib;
  calib <<
      4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
      -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
      9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
      0, 0, 0, 1;

//  calib <<
//      -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
//      -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
//      9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
//      0, 0,0, 1;

  for (const auto& filename : filenames)
  {
    const Eigen::Matrix4f transform = poses[count++] * calib;
    const octomap::point3d origin( transform(0,3), transform(1,3), transform(2,3) );

    std::cout << "Parsing: " << filename << std::endl
              << transform << std::endl;

    const auto t1 = std::chrono::system_clock::now();
    std::fstream input(filename.c_str(),
                       std::ios::in | std::ios::binary);

    pointcloud.clear();
    while (input.good() && !input.eof()) {
      Eigen::Vector4f point = {0, 0 , 0, 1};
      float intensity;
      input.read((char *) &point.x(), sizeof(float));
      input.read((char *) &point.y(), sizeof(float));
      input.read((char *) &point.z(), sizeof(float));
      input.read((char *) &intensity, sizeof(float));

      // apply transform first
      const Eigen::Vector4f p = transform * point;
      pointcloud.push_back(p.x(), p.y(), p.z());
    }
    input.close();
    const auto t2 = std::chrono::system_clock::now();

    octree.insertPointCloud(pointcloud, origin, max_dist, false, false);
    const auto t3 = std::chrono::system_clock::now();

    std::cout << "Time: " << ToMsec(t2-t1) << " / " << ToMsec(t3-t2) << std::endl;
  }

  // now, traverse all leafs in the tree:
  for (auto it = octree.begin(), end = octree.end(); it != end; ++it)
  {
    if (octree.isNodeOccupied(*it)){
      out.push_back( {it.getX(), it.getY(), it.getZ()} );
    }
  }
  return out;
}

int main(int argc, char** argv)
{
  if( argc != 3 ) {
    std::cout << " Usage: " << argv[0] << " velodyne_path pose_file" << std::endl;
    return -1;
  }

  const std::string velodyne_path = argv[1];
  const std::string pose_file = argv[2];

  std::vector<std::string> cloud_filenames;
  for (const auto& entry : fs::directory_iterator(velodyne_path))
  {
    cloud_filenames.push_back(entry.path().generic_string());
  }

  const size_t max_samples = 1000;
  const double max_dist = 25;
  const double voxel_size = 0.2;

  std::sort(cloud_filenames.begin(), cloud_filenames.end());
  cloud_filenames.resize(max_samples);

  std::vector<Eigen::Matrix4f> poses;
  {
    /*
     * r11 r12 r13 tx
     * r21 r22 r23 ty
     * r31 r32 r33 tz
     * 0   0   0   1
     *
     * r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
     */
    std::ifstream input(pose_file);
    std::string line;
    while(std::getline(input, line) && poses.size() < max_samples)
    {
      std::istringstream ss(line);
      Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
      ss  >> pose(0,0) >> pose(0,1) >> pose(0,2)  >> pose(0,3)
          >> pose(1,0) >> pose(1,1) >> pose(1,2)  >> pose(1,3)
          >> pose(2,0) >> pose(2,1) >> pose(2,2)  >> pose(2,3);
      poses.push_back(pose);
    }
  }


  auto computed_octomap = ComputeOctomap(cloud_filenames, poses, max_dist, voxel_size);

  Bonxai::WritePointsFromPCD("ocotomap_result.pcd", computed_octomap);

  return 0;
}
