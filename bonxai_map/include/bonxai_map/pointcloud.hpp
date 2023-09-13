#pragma once

#include "bonxai/bonxai.hpp"
#include <eigen3/Eigen/Geometry>

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

class ProbabilisticMap
{
public:

  static constexpr float prob(float logods)
  {
    return 1.0 - 1.0 / (1.0 + std::exp(logods));
  }

  static constexpr float logods(float prob)
  {
    return std::log(prob / (1.0 - prob));
  }

  // same default values as OctoMap
  struct Thresholds {
    float miss_decrement = logods(0.4f);
    float hit_increment = logods(0.7f);

    float clamp_min = logods(0.12f);
    float clamp_max = logods(0.97f);

    float occupancy_threshold = logods(0.5);
  };

  static const float UnknownProbability;

  struct CellT {
    float probability;
    uint8_t update_count = 0;
  };

  ProbabilisticMap(double resolution): _grid(resolution) {}

  VoxelGrid<CellT>& grid();

  const VoxelGrid<CellT>& grid() const;

  const Thresholds& thresholds() const;

  void setThresholds(const Thresholds& options);

  void insertPointCloud(const std::vector<Eigen::Vector3d> &points,
                        const Eigen::Vector3d &origin);

  bool isOccupied(const Bonxai::CoordT& coord) const;

  bool isUnknown(const Bonxai::CoordT& coord) const;

  bool isFree(const Bonxai::CoordT& coord) const;

  void getOccupiedVoxels(std::vector<Bonxai::CoordT>& coords);

private:
  VoxelGrid<CellT> _grid;
  Thresholds _options;
  uint8_t _update_count = 1;
};

}
