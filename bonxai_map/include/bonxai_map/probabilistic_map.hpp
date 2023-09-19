#pragma once

#include "bonxai/bonxai.hpp"
#include <eigen3/Eigen/Geometry>
#include <unordered_set>

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

  ProbabilisticMap(double resolution);

  [[nodiscard]] VoxelGrid<CellT>& grid();

  [[nodiscard]] const VoxelGrid<CellT>& grid() const;

  [[nodiscard]] const Options& options() const;

  void setOptions(const Options& options);

  /**
   * @brief insertPointCloud will update the probability map
   * with a new set of detections.
   * The template function can accept points of different types,
   * such as pcl:Point, Eigen::Vector or Bonxai::Point3d
   *
   * Both origin and points must be in word coordinates
   *
   * @param points   a vector of points which represent detected obstacles
   * @param origin   origin of the point cloud
   * @param max_range  max range of the ray, if exceeded, we will use that
   * to compute a free space
   */
  template <typename PointT>
  void insertPointCloud(const std::vector<PointT>& points,
                        const PointT& origin,
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

  std::unordered_set<CoordT> _miss_coords;
  std::vector<CoordT> _hit_coords;

  Bonxai::VoxelGrid<CellT>::Accessor _accessor;

  void addPoint(const Eigen::Vector3f &origin, const Eigen::Vector3f &point, const float &max_range,
                const float &max_sqr);
  void updateFreeCells(const Eigen::Vector3f& origin);
};

//--------------------------------------------------
template <typename T> inline
    Eigen::Vector3f ToEigenVector3f(const T& v)
{
  static_assert(type_has_method_x<T>::value ||
                    type_has_member_x<T>::value ||
                    type_has_operator<T>::value,
                "Can't assign to Eigen::Vector3f");

  if constexpr(type_has_method_x<T>::value) {
    return {v.x(), v.y(), v.z()};
  }
  if constexpr(type_has_member_x<T>::value) {
    return {v.x, v.y, v.z};
  }
  if constexpr(type_has_operator<T>::value){
    return {v[0], v[1], v[2]};
  }
}


template<typename PointT> inline
void ProbabilisticMap::insertPointCloud(const std::vector<PointT> &points,
                                        const PointT &origin,
                                        double max_range)
{
  const auto from = ToEigenVector3f(origin);
  const double max_range_sqr = max_range*max_range;
  for(const auto& point: points)
  {
    const auto to = ToEigenVector3f(point);
    addPoint(from, to, max_range, max_range_sqr);
  }
  updateFreeCells(origin);
}

}
