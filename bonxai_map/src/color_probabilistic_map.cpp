#include "bonxai_map/color_probabilistic_map.hpp"

#include <eigen3/Eigen/Geometry>
#include <limits>
#include <unordered_set>

namespace Bonxai {

const int32_t ColorProbabilisticMap::UnknownProbability = ColorProbabilisticMap::logods(0.5f);

VoxelGrid<ColorProbabilisticMap::CellT>& ColorProbabilisticMap::grid() {
  return _grid;
}

ColorProbabilisticMap::ColorProbabilisticMap(double resolution)
    : _grid(resolution),
      _accessor(_grid.createAccessor()) {}

const VoxelGrid<ColorProbabilisticMap::CellT>& ColorProbabilisticMap::grid() const {
  return _grid;
}

const ColorProbabilisticMap::Options& ColorProbabilisticMap::options() const {
  return _options;
}

void ColorProbabilisticMap::setOptions(const Options& options) {
  _options = options;
}

void ColorProbabilisticMap::addHitPoint(const Vector3D& point) {
  const auto coord = _grid.posToCoord(point);
  CellT* cell = _accessor.value(coord, true);

  if (cell->update_id != _update_count) {
    cell->probability_log =
        std::min(cell->probability_log + _options.prob_hit_log, _options.clamp_max_log);

    cell->update_id = _update_count;
    _hit_coords.push_back(coord);
  }
}

void ColorProbabilisticMap::addMissPoint(const Vector3D& point) {
  const auto coord = _grid.posToCoord(point);
  CellT* cell = _accessor.value(coord, true);

  if (cell->update_id != _update_count) {
    cell->probability_log =
        std::max(cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);

    cell->update_id = _update_count;
    _miss_coords.push_back(coord);
  }
}

void ColorProbabilisticMap::updateColor(const Vector3D& point, const Color& color) {
  const auto coord = _grid.posToCoord(point);
  CellT* cell = _accessor.value(coord, true);

  // running per-channel average; 32-bit intermediate avoids overflow
  const uint32_t n = cell->color_count;
  cell->color.r = static_cast<uint8_t>((cell->color.r * n + color.r) / (n + 1));
  cell->color.g = static_cast<uint8_t>((cell->color.g * n + color.g) / (n + 1));
  cell->color.b = static_cast<uint8_t>((cell->color.b * n + color.b) / (n + 1));

  // clamp the count so the average keeps converging instead of wrapping
  if (cell->color_count < std::numeric_limits<uint16_t>::max()) {
    cell->color_count++;
  }
}

bool ColorProbabilisticMap::isOccupied(const CoordT& coord) const {
  if (auto* cell = _accessor.value(coord, false)) {
    return cell->probability_log > _options.occupancy_threshold_log;
  }
  return false;
}

bool ColorProbabilisticMap::isUnknown(const CoordT& coord) const {
  if (auto* cell = _accessor.value(coord, false)) {
    return cell->probability_log == _options.occupancy_threshold_log;
  }
  return true;
}

bool ColorProbabilisticMap::isFree(const CoordT& coord) const {
  if (auto* cell = _accessor.value(coord, false)) {
    return cell->probability_log < _options.occupancy_threshold_log;
  }
  return false;
}

void Bonxai::ColorProbabilisticMap::updateFreeCells(const Vector3D& origin) {
  auto accessor = _grid.createAccessor();

  // same as addMissPoint, but using lambda will force inlining
  auto clearPoint = [this, &accessor](const CoordT& coord) {
    CellT* cell = accessor.value(coord, true);
    if (cell->update_id != _update_count) {
      cell->probability_log =
          std::max(cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);
      cell->update_id = _update_count;
    }
    return true;
  };

  const auto coord_origin = _grid.posToCoord(origin);

  for (const auto& coord_end : _hit_coords) {
    RayIterator(coord_origin, coord_end, clearPoint);
  }
  _hit_coords.clear();

  for (const auto& coord_end : _miss_coords) {
    RayIterator(coord_origin, coord_end, clearPoint);
  }
  _miss_coords.clear();

  if (++_update_count == 4) {
    _update_count = 1;
  }
}

void ColorProbabilisticMap::getOccupiedVoxels(std::vector<CoordT>& coords) {
  coords.clear();
  auto visitor = [&](CellT& cell, const CoordT& coord) {
    if (cell.probability_log > _options.occupancy_threshold_log) {
      coords.push_back(coord);
    }
  };
  _grid.forEachCell(visitor);
}

void ColorProbabilisticMap::getOccupiedVoxels(
    std::vector<CoordT>& coords, std::vector<Color>& colors) {
  coords.clear();
  colors.clear();
  auto visitor = [&](CellT& cell, const CoordT& coord) {
    if (cell.probability_log > _options.occupancy_threshold_log) {
      coords.push_back(coord);
      colors.push_back(cell.color);
    }
  };
  _grid.forEachCell(visitor);
}

void ColorProbabilisticMap::getFreeVoxels(std::vector<CoordT>& coords) {
  coords.clear();
  auto visitor = [&](CellT& cell, const CoordT& coord) {
    if (cell.probability_log < _options.occupancy_threshold_log) {
      coords.push_back(coord);
    }
  };
  _grid.forEachCell(visitor);
}

}  // namespace Bonxai
