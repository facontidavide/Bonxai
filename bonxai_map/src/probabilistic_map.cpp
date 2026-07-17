#include "bonxai_map/probabilistic_map.hpp"

#include <eigen3/Eigen/Geometry>

namespace Bonxai {

const int32_t ProbabilisticMap::UnknownProbability = ProbabilisticMap::logods(0.5f);

VoxelGrid<ProbabilisticMap::CellT>& ProbabilisticMap::grid() {
  return _grid;
}

ProbabilisticMap::ProbabilisticMap(double resolution)
    : _grid(resolution),
      _accessor(_grid.createAccessor()) {}

const VoxelGrid<ProbabilisticMap::CellT>& ProbabilisticMap::grid() const {
  return _grid;
}

const ProbabilisticMap::Options& ProbabilisticMap::options() const {
  return _options;
}

void ProbabilisticMap::setOptions(const Options& options) {
  _options = options;
}

void ProbabilisticMap::addHitPoint(const Vector3D& point) {
  const auto coord = _grid.posToCoord(point);
  CellT* cell = _accessor.value(coord, true);

  if (cell->flags == CellT::kUnseen) {
    _ray_targets.emplace_back(coord, cell);
  }
  // a hit always wins over a miss within the same scan
  cell->flags = CellT::kHit;
}

void ProbabilisticMap::addMissPoint(const Vector3D& point) {
  const auto coord = _grid.posToCoord(point);
  CellT* cell = _accessor.value(coord, true);

  if (cell->flags == CellT::kUnseen) {
    cell->flags = CellT::kFree;
    _ray_targets.emplace_back(coord, cell);
  }
}

bool ProbabilisticMap::isOccupied(const CoordT& coord) const {
  if (auto* cell = _accessor.value(coord, false)) {
    return cell->probability_log > _options.occupancy_threshold_log;
  }
  return false;
}

bool ProbabilisticMap::isUnknown(const CoordT& coord) const {
  if (auto* cell = _accessor.value(coord, false)) {
    return cell->probability_log == _options.occupancy_threshold_log;
  }
  return true;
}

bool ProbabilisticMap::isFree(const CoordT& coord) const {
  if (auto* cell = _accessor.value(coord, false)) {
    return cell->probability_log < _options.occupancy_threshold_log;
  }
  return false;
}

void Bonxai::ProbabilisticMap::updateFreeCells(const Vector3D& origin) {
  auto accessor = _grid.createAccessor();

  auto applyHit = [this](CellT* cell) {
    cell->probability_log =
        std::min(cell->probability_log + _options.prob_hit_log, _options.clamp_max_log);
    cell->flags = CellT::kUnseen;
  };
  auto applyMiss = [this](CellT* cell) {
    cell->probability_log =
        std::max(cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);
    cell->flags = CellT::kUnseen;
  };

  // mark the voxels traversed by the rays; endpoints keep their hit/miss state
  auto visitFreeCell = [this, &accessor](const CoordT& coord) {
    CellT* cell = accessor.value(coord, true);
    if (cell->flags == CellT::kUnseen) {
      cell->flags = CellT::kFree;
      _traversed_cells.push_back(cell);
    }
    return true;
  };

  const auto coord_origin = _grid.posToCoord(origin);
  const bool exact = (_options.ray_mode == Options::RayMode::Exact);
  const double resolution = _grid.voxelSize();

  for (const auto& [coord_end, cell] : _ray_targets) {
    if (exact) {
      ExactRayIterator(origin, coord_origin, coord_end, resolution, visitFreeCell);
    } else {
      RayIterator(coord_origin, coord_end, visitFreeCell);
    }
  }

  // apply exactly one probability update per touched cell and clear its flag
  for (const auto& [coord, cell] : _ray_targets) {
    if (cell->flags == CellT::kHit) {
      applyHit(cell);
    } else {
      applyMiss(cell);
    }
  }
  _ray_targets.clear();

  for (CellT* cell : _traversed_cells) {
    applyMiss(cell);
  }
  _traversed_cells.clear();
}

void ProbabilisticMap::getOccupiedVoxels(std::vector<CoordT>& coords) {
  coords.clear();
  auto visitor = [&](CellT& cell, const CoordT& coord) {
    if (cell.probability_log > _options.occupancy_threshold_log) {
      coords.push_back(coord);
    }
  };
  _grid.forEachCell(visitor);
}

void ProbabilisticMap::getFreeVoxels(std::vector<CoordT>& coords) {
  coords.clear();
  auto visitor = [&](CellT& cell, const CoordT& coord) {
    if (cell.probability_log < _options.occupancy_threshold_log) {
      coords.push_back(coord);
    }
  };
  _grid.forEachCell(visitor);
}

}  // namespace Bonxai
