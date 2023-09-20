#include "bonxai_map/probabilistic_map.hpp"
#include <eigen3/Eigen/Geometry>
#include <unordered_set>

namespace Bonxai
{

const int32_t ProbabilisticMap::UnknownProbability = ProbabilisticMap::logods(0.5f);

bool ComputeRay(const CoordT& key_origin,
                const CoordT& key_end,
                std::vector<CoordT>& ray)
{
  ray.clear();
  if (key_origin == key_end)
  {
    return true;
  }
  ray.push_back(key_origin);

  CoordT error = { 0, 0, 0 };
  CoordT coord = key_origin;
  CoordT delta = (key_end - coord);
  const CoordT step = { delta.x < 0 ? -1 : 1,
                        delta.y < 0 ? -1 : 1,
                        delta.z < 0 ? -1 : 1 };

  delta = { delta.x < 0 ? -delta.x : delta.x,
            delta.y < 0 ? -delta.y : delta.y,
            delta.z < 0 ? -delta.z : delta.z };

  const int max = std::max(std::max(delta.x, delta.y), delta.z);

  // maximum change of any coordinate
  for (int i = 0; i < max - 1; ++i)
  {
    // update errors
    error = error + delta;
    // manual loop unrolling
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
    ray.push_back(coord);
  }

  return true;
}

VoxelGrid<ProbabilisticMap::CellT>& ProbabilisticMap::grid()
{
  return _grid;
}

ProbabilisticMap::ProbabilisticMap(double resolution)
  : _grid(resolution)
  , _accessor(_grid.createAccessor())
{}

const VoxelGrid<ProbabilisticMap::CellT>& ProbabilisticMap::grid() const
{
  return _grid;
}

const ProbabilisticMap::Options& ProbabilisticMap::options() const
{
  return _options;
}

void ProbabilisticMap::setOptions(const Options& options)
{
  _options = options;
}

void ProbabilisticMap::addPoint(const Eigen::Vector3f& origin,
                                const Eigen::Vector3f& point,
                                const float& max_range,
                                const float& max_range_sqr)
{
  Eigen::Vector3f vect(point - origin);
  const double squared_norm = vect.squaredNorm();
  if (squared_norm >= max_range_sqr)
  {
    // this will be considered a "miss".
    // Compute the end point to cast a cleaning ray
    vect /= std::sqrt(squared_norm);
    const Eigen::Vector3f new_point = origin + (vect * max_range);
    const auto coord = _grid.posToCoord(new_point);

    // for very dense pointclouds, this MIGHT be true.
    // worth checking, to avoid calling unordered_set::insert
    _miss_coords.insert(coord);
    return;
  }

  const auto coord = _grid.posToCoord(point);
  CellT* cell = _accessor.value(coord, true);

  if (cell->update_id != _update_count)
  {
    cell->probability_log = std::min(cell->probability_log + _options.prob_hit_log,
                                     _options.clamp_max_log);

    cell->update_id = _update_count;
    _hit_coords.push_back(coord);
  }
}

void Bonxai::ProbabilisticMap::updateFreeCells(const Eigen::Vector3f& origin)
{
  auto clearRay = [this](const CoordT& from, const CoordT& to) {
    auto accessor = _grid.createAccessor();
    // clean space with ray casting
    thread_local std::vector<Bonxai::CoordT> ray;
    ComputeRay(from, to, ray);

    for (const auto& coord : ray)
    {
      CellT* cell = accessor.value(coord, true);
      if (cell->update_id != _update_count)
      {
        cell->probability_log = std::max(
            cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);
        cell->update_id = _update_count;
      }
    }
  };

  const auto coord_origin = _grid.posToCoord(origin);

  for (const auto& coord_end : _hit_coords)
  {
    clearRay(coord_origin, coord_end);
  }
  _hit_coords.clear();

  for (const auto& coord_end : _miss_coords)
  {
    clearRay(coord_origin, coord_end);
  }
  _miss_coords.clear();

  if (++_update_count == 4)
  {
    _update_count = 1;
  }
}

void ProbabilisticMap::getOccupiedVoxels(std::vector<CoordT>& coords)
{
  coords.clear();
  auto visitor = [&](CellT& cell, const CoordT& coord) {
    if (cell.probability_log > _options.occupancy_threshold_log)
    {
      coords.push_back(coord);
    }
  };
  _grid.forEachCell(visitor);
}

void ProbabilisticMap::getFreeVoxels(std::vector<CoordT>& coords)
{
  coords.clear();
  auto visitor = [&](CellT& cell, const CoordT& coord) {
    if (cell.probability_log < _options.occupancy_threshold_log)
    {
      coords.push_back(coord);
    }
  };
  _grid.forEachCell(visitor);
}

}  // namespace Bonxai
