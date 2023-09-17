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

bool ComputeRay(const CoordT &key_origin,
                const CoordT &key_end,
                std::vector<CoordT> &ray);


class ProbabilisticMap
{
public:

  [[nodiscard]] static constexpr float prob(int32_t logods_fixed)
  {
    float logods = float(logods_fixed) * 1e-3;
    return (1.0 - 1.0 / (1.0 + std::exp(logods)));
  }

  [[nodiscard]] static constexpr int32_t logods(float prob)
  {
    return int32_t(1e3 * std::log(prob / (1.0 - prob)));
  }

  // The default values are the same as OctoMap
  struct Options {
    int32_t prob_miss_log = logods(0.4f);
    int32_t prob_hit_log = logods(0.7f);

    int32_t clamp_min = logods(0.12f);
    int32_t clamp_max = logods(0.97f);

    float occupancy_threshold = logods(0.5);
  };

  static const int32_t UnknownProbability;

  struct CellT {
    int32_t update_count : 8;
    int32_t probability : 24;
    CellT(): update_count(0), probability(UnknownProbability) {};
  };

  ProbabilisticMap(double resolution): _grid(resolution) {}

  VoxelGrid<CellT>& grid();

  const VoxelGrid<CellT>& grid() const;

  const Options& options() const;

  void setOptions(const Options& options);

  void insertPointCloud(const std::vector<Eigen::Vector3f> &points,
                        const Eigen::Vector3f &origin,
                        double max_range);

  bool isOccupied(const Bonxai::CoordT& coord) const;

  bool isUnknown(const Bonxai::CoordT& coord) const;

  bool isFree(const Bonxai::CoordT& coord) const;

  void getOccupiedVoxels(std::vector<Bonxai::CoordT>& coords);

  void getFreeVoxels(std::vector<Bonxai::CoordT>& coords);

  template <typename PointT>
  void getOccupiedVoxels(std::vector<PointT>& points)
  {
    thread_local std::vector<Bonxai::CoordT> coords;
    coords.clear();
    getOccupiedVoxels(coords);
    for(const auto& coord: coords)
    {
      const auto p = _grid.coordToPos(coord);
      points.emplace_back( p.x, p.y, p.z );
    }
  }

private:
  VoxelGrid<CellT> _grid;
  Options _options;
  uint8_t _update_count = 1;
};

}
