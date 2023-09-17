#pragma once

#include "bonxai/bonxai.hpp"
#include <eigen3/Eigen/Geometry>

namespace Bonxai
{

bool ComputeRay(const CoordT &key_origin,
                const CoordT &key_end,
                std::vector<CoordT> &ray);

/**
 * @brief The ProbabilisticMap class is meant to behave as much as possible as
 * octomap::Octree, given the same voxel size.
 *
 * Insert a point cloud to update the current probability
 */
class ProbabilisticMap
{
public:

  /// Compute the logds, but return the result as an integer,
  /// The real number is represented as a fixed precision
  /// integer (6 decimals after the comma)
  [[nodiscard]] static constexpr int32_t logods(float prob)
  {
    return int32_t(1e6 * std::log(prob / (1.0 - prob)));
  }

  /// Expect the fixed comma value returned by logods()
  [[nodiscard]] static constexpr float prob(int32_t logods_fixed)
  {
    float logods = float(logods_fixed) * 1e-6;
    return (1.0 - 1.0 / (1.0 + std::exp(logods)));
  }

  struct CellT {
    // variable used to check if a cell was already updated in this loop
    int32_t update_id : 4;
    // the probability of the cell to be occupied
    int32_t probability_log : 28;

    CellT(): update_id(0), probability_log(UnknownProbability) {};
  };

  /// These default values are the same as OctoMap
  struct Options {
    int32_t prob_miss_log = logods(0.4f);
    int32_t prob_hit_log = logods(0.7f);

    int32_t clamp_min_log = logods(0.12f);
    int32_t clamp_max_log = logods(0.97f);

    int32_t occupancy_threshold_log = logods(0.5);
  };

  static const int32_t UnknownProbability;

  ProbabilisticMap(double resolution): _grid(resolution) {}

  [[nodiscard]] VoxelGrid<CellT>& grid();

  [[nodiscard]] const VoxelGrid<CellT>& grid() const;

  [[nodiscard]] const Options& options() const;

  void setOptions(const Options& options);

  void insertPointCloud(const std::vector<Eigen::Vector3f> &points,
                        const Eigen::Vector3f &origin,
                        double max_range);

  [[nodiscard]] bool isOccupied(const Bonxai::CoordT& coord) const;

  [[nodiscard]] bool isUnknown(const Bonxai::CoordT& coord) const;

  [[nodiscard]] bool isFree(const Bonxai::CoordT& coord) const;

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


  void clearRay(const Bonxai::CoordT& from, const Bonxai::CoordT& to);
};

}
