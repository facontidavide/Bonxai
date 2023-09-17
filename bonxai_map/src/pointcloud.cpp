#include "bonxai_map/pointcloud.hpp"
#include <eigen3/Eigen/Geometry>
#include <unordered_set>

namespace Bonxai
{

const int32_t ProbabilisticMap::UnknownProbability = ProbabilisticMap::logods(0.5f);

bool ComputeRay(const CoordT &key_origin,
                const CoordT &key_end,
                std::vector<CoordT> &ray)
{
  ray.clear();
  if (key_origin == key_end)
  {
    return true;
  }
  ray.push_back( key_origin );

  CoordT error = {0,0,0};
  CoordT coord = key_origin;
  CoordT delta = (key_end - coord);
  const CoordT step = { delta.x < 0 ? -1: 1,
                        delta.y < 0 ? -1: 1,
                        delta.z < 0 ? -1: 1 };

  delta = { delta.x < 0 ? -delta.x: delta.x,
            delta.y < 0 ? -delta.y: delta.y,
            delta.z < 0 ? -delta.z: delta.z };

  const int max = std::max(std::max(delta.x, delta.y), delta.z);

  // maximum change of any coordinate
  for (int i = 0; i < max-1; ++i){
    // update errors
    error = error + delta;

    if ((error.x << 1) >= max)
    {
      coord.x += step.x;
      error.x -= max;
    }
    if ((error.y << 1) >= max)
    {
      coord.y += step.y;
      error.y -= max;
    }
    if ((error.z << 1) >= max)
    {
      coord.z += step.z;
      error.z -= max;
    }
    ray.push_back( coord );
  }
  return true;
}

VoxelGrid<ProbabilisticMap::CellT> &ProbabilisticMap::grid()
{
  return _grid;
}

const VoxelGrid<ProbabilisticMap::CellT> &ProbabilisticMap::grid() const
{
  return _grid;
}

const ProbabilisticMap::Options &ProbabilisticMap::options() const
{
  return _options;
}

void ProbabilisticMap::setOptions(const Options &options)
{
  _options = options;
}

void ProbabilisticMap::insertPointCloud(const std::vector<Eigen::Vector3f> &points,
                                        const Eigen::Vector3f& origin,
                                        double max_range)
{
  auto accessor = _grid.createAccessor();

  const double max_range_sqr = max_range*max_range;

  thread_local std::vector<CoordT> ray_endpoints;
  CoordT prev_coord ={0,0,0};
  ray_endpoints.clear();

  // first: mark the hits
  for(const auto& p: points)
  {
    if(max_range > 0)
    {
      const Eigen::Vector3f vect(p - origin);
      const double squared_norm = vect.squaredNorm();
      if( squared_norm >= max_range_sqr)
      {
        // this will be considered a "miss".
        // Compute the end point to cast a cleaning ray
        const Eigen::Vector3f new_point = (vect / std::sqrt(squared_norm)) * max_range;
        const auto end_coord = _grid.posToCoord( new_point );
        if(end_coord != prev_coord)
        {
          ray_endpoints.push_back(end_coord);
          prev_coord = end_coord;
        }
        continue;
      }
    }
    const auto coord = _grid.posToCoord( {p.x(), p.y(), p.z()} );

    CellT* cell = accessor.value(coord, true);
    if(cell->update_count != _update_count)
    {
      cell->probability = std::min(cell->probability + _options.prob_hit_log,
                                   _options.clamp_max);

      // don't visit twice the same cell
      cell->update_count = _update_count;
      ray_endpoints.push_back(coord);
    }
  }

  const auto coord_origin = _grid.posToCoord( origin );

  for(const auto& coord_end: ray_endpoints)
  {
    // clean space with ray casting
    thread_local std::vector<Bonxai::CoordT> ray;
    ComputeRay(coord_origin, coord_end, ray);

    for(const auto& coord: ray)
    {
      CellT* cell = accessor.value(coord, true);
      if(cell->update_count != _update_count)
      {
        cell->probability = std::max(cell->probability + _options.prob_miss_log,
                                     _options.clamp_min);

        // don't visit twice the same cell
        cell->update_count = _update_count;
      }
    }
  }
  if(++_update_count == 255)
  {
    _update_count = 1;
  }
}

void ProbabilisticMap::getOccupiedVoxels(std::vector<CoordT> &coords)
{
  coords.clear();
  auto visitor = [&](CellT& cell, const CoordT& coord) {
    if(cell.probability > _options.occupancy_threshold)
    {
      coords.push_back(coord);
    }
  };
  _grid.forEachCell(visitor);
}

void ProbabilisticMap::getFreeVoxels(std::vector<CoordT> &coords)
{
  coords.clear();
  auto visitor = [&](CellT& cell, const CoordT& coord) {
    if(cell.probability < _options.occupancy_threshold)
    {
      coords.push_back(coord);
    }
  };
  _grid.forEachCell(visitor);
}

}

