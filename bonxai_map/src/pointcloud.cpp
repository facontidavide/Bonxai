#include "bonxai_map/pointcloud.hpp"
#include <eigen3/Eigen/Geometry>

namespace Bonxai
{

const float ProbabilisticMap::UnknownProbability = ProbabilisticMap::logods(0.5);


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

  using Eigen::Vector3i;

  Vector3i error = Vector3i::Zero();
  Vector3i coord = Vector3i(key_origin.x, key_origin.y, key_origin.z);
  const Vector3i delta = (Vector3i(key_end.x, key_end.y, key_end.z) - coord).array().abs();
  const Vector3i step = (delta.array() < 0).select(-1, Vector3i::Ones());

  const int max = delta.maxCoeff();

  // maximum change of any coordinate
  for (int i = 0; i < max-1; ++i){
    // update errors
    error += delta;

    for (int index = 0; index < 3; ++index)
    {
      if ((error(index) << 1) < max)
      {
        continue;
      }

      coord(index) += step(index);
      error(index) -= max;
    }
    ray.push_back( {coord.x(), coord.y(), coord.z()} );
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

void ProbabilisticMap::insertPointCloud(const std::vector<Eigen::Vector3d> &points,
                                        const Eigen::Vector3d& origin,
                                        double max_range,
                                        bool discretize)
{
  auto accessor = _grid.createAccessor();

  // first: mark the hits
  for(const auto& p: points)
  {
    auto cell = accessor.getLeafGrid(_grid.posToCoord( {p.x(), p.y(), p.z()} ), true )->data;

    if(cell->update_count != _update_count)
    {
      // new cell
      if(cell->update_count == 0) {
        cell->probability = UnknownProbability + _options.hit_increment;
      }
      else {
        cell->probability = std::min(cell->probability + _options.hit_increment,
                                     _options.clamp_max);
      }
      // we don't want to visit twice the same cell
      cell->update_count = _update_count;
    }
  }

  thread_local std::vector<Bonxai::CoordT> ray;

  for(const auto& p: points)
  {
    const Eigen::Vector3d point = {p.x(), p.y(), p.z()};
    // clean space with ray casting
    ComputeRay(origin, point, _grid.resolution, ray);
    for(const auto& coord: ray)
    {
      auto cell = accessor.getLeafGrid(coord, true )->data;
      if(cell->update_count != _update_count)
      {
        // new cell
        if(cell->update_count == 0) {
          cell->probability = UnknownProbability + _options.miss_decrement;
        }
        else {
          cell->probability = std::max(cell->probability + _options.miss_decrement,
                                       _options.clamp_min);
        }
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

}

