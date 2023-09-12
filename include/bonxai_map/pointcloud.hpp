#pragma once

#include "bonxai/bonxai.hpp"
#include <eigen3/Eigen/Core>


namespace Bonxai
{

inline Bonxai::Point3D ToPoint3D(const Eigen::Vector3d& v)
{
  return {v.x(), v.y(), v.z()};
}

inline Eigen::Vector3d FromPoint3D(const Bonxai::Point3D& p)
{
  return {p.x, p.y, p.z};
}

bool ComputeRay(const Eigen::Vector3d &origin,
                const Eigen::Vector3d &end,
                const double resoulution,
                std::vector<CoordT>& ray);

}
