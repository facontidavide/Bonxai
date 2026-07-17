#pragma once

#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <limits>
#include <utility>

#include "bonxai/bonxai.hpp"

namespace Bonxai {

// Integer Bresenham line between two voxels (one step per cell, 26-connected).
// Fast, but it does NOT visit every voxel crossed by the continuous segment:
// see ProbabilisticMap::Options::RayMode.
template <class Functor>
void RayIterator(const CoordT& key_origin, const CoordT& key_end, const Functor& func);

// Exact voxel traversal (Amanatides & Woo, "A Fast Voxel Traversal Algorithm for
// Ray Tracing", 1987). Visits every voxel crossed by the segment going from `from`
// (continuous world coordinates, inside voxel `coord_from`) to the CENTER of voxel
// `coord_to`; `coord_from` is included, `coord_to` is excluded. Voxels are the
// half-open boxes [coord * resolution, (coord + 1) * resolution).
template <class Functor>
void ExactRayIterator(
    const Eigen::Vector3d& from, const CoordT& coord_from, const CoordT& coord_to,
    double resolution, const Functor& func);

inline void ComputeRay(const CoordT& key_origin, const CoordT& key_end, std::vector<CoordT>& ray) {
  ray.clear();
  RayIterator(key_origin, key_end, [&ray](const CoordT& coord) {
    ray.push_back(coord);
    return true;
  });
}

inline void ComputeExactRay(
    const Eigen::Vector3d& from, const CoordT& coord_from, const CoordT& coord_to,
    double resolution, std::vector<CoordT>& ray) {
  ray.clear();
  ExactRayIterator(from, coord_from, coord_to, resolution, [&ray](const CoordT& coord) {
    ray.push_back(coord);
    return true;
  });
}

/**
 * @brief The ProbabilisticMap class is meant to behave as much as possible as
 * octomap::Octree, given the same voxel size.
 *
 * Insert a point cloud to update the current probability
 */
class ProbabilisticMap {
 public:
  using Vector3D = Eigen::Vector3d;

  /// Compute the logds, but return the result as an integer,
  /// The real number is represented as a fixed precision
  /// integer (6 decimals after the comma)
  [[nodiscard]] static constexpr int32_t logods(float prob) {
    return int32_t(1e6 * std::log(prob / (1.0 - prob)));
  }

  /// Expect the fixed comma value returned by logods()
  [[nodiscard]] static constexpr float prob(int32_t logods_fixed) {
    float logods = float(logods_fixed) * 1e-6;
    return (1.0 - 1.0 / (1.0 + std::exp(logods)));
  }

  struct CellT {
    // transient per-scan state of `flags`; always kUnseen outside updateFreeCells()
    enum : int32_t { kUnseen = 0, kFree = 1, kHit = 2 };

    int32_t flags : 4;
    // the probability of the cell to be occupied
    int32_t probability_log : 28;

    CellT()
        : flags(kUnseen),
          probability_log(UnknownProbability){};
  };

  /// These default values are the same as OctoMap
  struct Options {
    int32_t prob_miss_log = logods(0.4f);
    int32_t prob_hit_log = logods(0.7f);

    int32_t clamp_min_log = logods(0.12f);
    int32_t clamp_max_log = logods(0.97f);

    int32_t occupancy_threshold_log = logods(0.5);

    // Ray traversal used for free-space carving.
    // Exact (default, octomap-equivalent): visit every voxel crossed by the
    // segment from the sensor origin to the center of the endpoint voxel.
    // Approximate: the legacy integer Bresenham. Faster, but it skips part of
    // the crossed voxels, so carving is weaker and depends on the direction of
    // the ray relative to the grid axes.
    enum class RayMode : uint8_t { Exact, Approximate };
    RayMode ray_mode = RayMode::Exact;
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
   * Both origin and points must be in world coordinates
   *
   * @param points   a vector of points which represent detected obstacles
   * @param origin   origin of the point cloud
   * @param max_range  max range of the ray, if exceeded, we will use that
   * to compute a free space
   */
  template <typename PointT, typename Allocator>
  void insertPointCloud(
      const std::vector<PointT, Allocator>& points, const PointT& origin, double max_range);

  // This function is usually called by insertPointCloud
  // We expose it here to add more control to the user.
  // The probability update is deferred: once finished adding points,
  // you must call updateFreeCells()
  void addHitPoint(const Vector3D& point);

  // This function is usually called by insertPointCloud
  // We expose it here to add more control to the user.
  // The probability update is deferred: once finished adding points,
  // you must call updateFreeCells()
  void addMissPoint(const Vector3D& point);

  // Carves the free space between the origin and the endpoints added with
  // addHitPoint / addMissPoint, then applies exactly one probability update
  // per touched cell (a hit always wins over a miss within the same scan).
  // Called automatically by insertPointCloud.
  void updateFreeCells(const Vector3D& origin);

  [[nodiscard]] bool isOccupied(const Bonxai::CoordT& coord) const;

  [[nodiscard]] bool isUnknown(const Bonxai::CoordT& coord) const;

  [[nodiscard]] bool isFree(const Bonxai::CoordT& coord) const;

  void getOccupiedVoxels(std::vector<Bonxai::CoordT>& coords);

  void getFreeVoxels(std::vector<Bonxai::CoordT>& coords);

  template <typename PointT>
  void getOccupiedVoxels(std::vector<PointT>& points) {
    thread_local std::vector<Bonxai::CoordT> coords;
    coords.clear();
    getOccupiedVoxels(coords);
    for (const auto& coord : coords) {
      const auto p = _grid.coordToPos(coord);
      points.emplace_back(p.x, p.y, p.z);
    }
  }

 private:
  VoxelGrid<CellT> _grid;
  Options _options;

  // unique endpoint voxels (hits and misses) of the current scan: the targets of
  // the free-space carving rays. Cell pointers stay valid for the whole scan
  // (leaf blocks never move once allocated), so the update pass needs no lookups.
  std::vector<std::pair<CoordT, CellT*>> _ray_targets;
  // non-endpoint voxels traversed by the rays of the current scan
  std::vector<CellT*> _traversed_cells;

  mutable Bonxai::VoxelGrid<CellT>::Accessor _accessor;
};

//--------------------------------------------------

template <typename PointT, typename Alloc>
inline void ProbabilisticMap::insertPointCloud(
    const std::vector<PointT, Alloc>& points, const PointT& origin, double max_range) {
  const auto from = ConvertPoint<Vector3D>(origin);
  const double max_range_sqr = max_range * max_range;
  for (const auto& point : points) {
    const auto to = ConvertPoint<Vector3D>(point);
    Vector3D vect(to - from);
    const double squared_norm = vect.squaredNorm();
    // points that exceed the max_range will create a cleaning ray
    if (squared_norm >= max_range_sqr) {
      // The new point will have distance == max_range from origin
      const Vector3D new_point = from + ((vect / std::sqrt(squared_norm)) * max_range);
      addMissPoint(new_point);
    } else {
      addHitPoint(to);
    }
  }
  updateFreeCells(from);
}

template <class Functor>
inline void RayIterator(const CoordT& key_origin, const CoordT& key_end, const Functor& func) {
  if (key_origin == key_end) {
    return;
  }
  if (!func(key_origin)) {
    return;
  }

  CoordT error = {0, 0, 0};
  CoordT coord = key_origin;
  CoordT delta = (key_end - coord);
  const CoordT step = {delta.x < 0 ? -1 : 1, delta.y < 0 ? -1 : 1, delta.z < 0 ? -1 : 1};

  delta = {
      delta.x < 0 ? -delta.x : delta.x, delta.y < 0 ? -delta.y : delta.y,
      delta.z < 0 ? -delta.z : delta.z};

  const int max = std::max(std::max(delta.x, delta.y), delta.z);

  // maximum change of any coordinate
  for (int i = 0; i < max - 1; ++i) {
    // update errors
    error = error + delta;
    // manual loop unrolling
    if ((error.x << 1) >= max) {
      coord.x += step.x;
      error.x -= max;
    }
    if ((error.y << 1) >= max) {
      coord.y += step.y;
      error.y -= max;
    }
    if ((error.z << 1) >= max) {
      coord.z += step.z;
      error.z -= max;
    }
    if (!func(coord)) {
      return;
    }
  }
}

template <class Functor>
inline void ExactRayIterator(
    const Eigen::Vector3d& from, const CoordT& coord_from, const CoordT& coord_to,
    double resolution, const Functor& func) {
  if (coord_from == coord_to) {
    return;
  }
  if (!func(coord_from)) {
    return;
  }

  const Eigen::Vector3d to(
      (coord_to.x + 0.5) * resolution, (coord_to.y + 0.5) * resolution,
      (coord_to.z + 0.5) * resolution);
  const Eigen::Vector3d delta = to - from;

  CoordT coord = coord_from;
  int32_t step[3];
  double t_max[3];
  double t_delta[3];
  // parametrized along the unnormalized segment: t = 1 at the endpoint center
  for (int i = 0; i < 3; i++) {
    if (delta[i] != 0.0) {
      const double inv_delta = 1.0 / delta[i];
      step[i] = (delta[i] > 0.0) ? 1 : -1;
      const double boundary = (coord[i] + (step[i] > 0 ? 1 : 0)) * resolution;
      t_max[i] = (boundary - from[i]) * inv_delta;
      t_delta[i] = resolution * std::abs(inv_delta);
    } else {
      step[i] = 0;
      t_max[i] = std::numeric_limits<double>::infinity();
      t_delta[i] = std::numeric_limits<double>::infinity();
    }
  }
  while (true) {
    const int axis = (t_max[0] < t_max[1]) ? ((t_max[0] < t_max[2]) ? 0 : 2)
                                           : ((t_max[1] < t_max[2]) ? 1 : 2);
    if (t_max[axis] > 1.0) {
      return;  // no boundary crossing left before the end of the segment
    }
    coord[axis] += step[axis];
    t_max[axis] += t_delta[axis];
    if (coord == coord_to) {
      return;  // the endpoint voxel is excluded
    }
    if (!func(coord)) {
      return;
    }
  }
}

}  // namespace Bonxai
