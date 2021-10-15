#ifndef TREEXY_HPP
#define TREEXY_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <functional>
#include <cmath>
#include <iostream>
#include "treexy/node_mask.hpp"

namespace Treexy
{
struct Point3D
{
  double x;
  double y;
  double z;
};

struct CoordT
{
  int32_t x;
  int32_t y;
  int32_t z;
};

inline bool operator==(const CoordT& a, const CoordT& b)
{
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

inline bool operator!=(const CoordT& a, const CoordT& b)
{
  return !(a == b);
}

}  // end namespace Treexy

namespace std
{
template <>
struct hash<Treexy::CoordT>
{
  std::size_t operator()(const Treexy::CoordT& p) const
  {
    //    same as boost
    //    size_t seed = 0.0;
    //    std::hash<int32_t> hasher;
    //    seed ^= hasher(p.x) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    //    seed ^= hasher(p.y) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    //    seed ^= hasher(p.z) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    //    return seed;
    //    same as OpenVDB
    return ((1 << 20) - 1) & (p.x * 73856093 ^ p.y * 19349663 ^ p.z * 83492791);
  }
};
}  // namespace std

namespace Treexy
{
template <typename DataT, int Log2DIM>
struct Grid
{
  constexpr static int DIM = 1 << Log2DIM;
  constexpr static int SIZE = DIM * DIM * DIM;
  std::array<DataT, SIZE> data;
  Treexy::Mask<Log2DIM> mask;
};

template <typename DataT, int Log2DIM_INNER = 2, int Log2DIM_LEAF = 3>
struct VoxelGrid
{
  /// @brief Return a hash key derived from the existing coordinates.
  /// @details For details on this hash function please see the VDB paper.
  constexpr static int32_t Log2N = Log2DIM_INNER + Log2DIM_LEAF;

  using LeafGrid = Grid<DataT, Log2DIM_LEAF>;
  using InnerGrid = Grid<std::unique_ptr<LeafGrid>, Log2DIM_INNER>;
  using RootMap = std::unordered_map<CoordT, InnerGrid>;

  RootMap root_map;

  const double resolution;
  const double inv_resolution;
  const double half_resolution;

  VoxelGrid(double voxel_size)
    : resolution(voxel_size)
    , inv_resolution(1.0 / voxel_size)
    , half_resolution(0.5 * voxel_size)
  {
  }

  CoordT posToCoord(const Point3D& pos)
  {
    return { static_cast<int32_t>(pos.x * inv_resolution) - std::signbit(pos.x),
             static_cast<int32_t>(pos.y * inv_resolution) - std::signbit(pos.y),
             static_cast<int32_t>(pos.z * inv_resolution) - std::signbit(pos.z) };
  }

  Point3D coordToPos(const CoordT& coord)
  {
    return { half_resolution + static_cast<double>(coord.x * resolution),
             half_resolution + static_cast<double>(coord.y * resolution),
             half_resolution + static_cast<double>(coord.z * resolution) };
  }

  class Accessor
  {
  public:
    Accessor(RootMap& root) : root_(root)
    {
    }

    bool setValue(const CoordT& coord, const DataT& value);

    const DataT* value(const CoordT& coord) const;

    template <class VisitorFunction>
    void forEachCell(const VisitorFunction& func)
    {
      constexpr static int32_t MASK_INNER = ((1 << Log2DIM_INNER) - 1);
      constexpr static int32_t MASK_LEAF = ((1 << Log2DIM_LEAF) - 1);

      for (auto& map_it : root_)
      {
        const auto& A = (map_it.first);
        InnerGrid& inner_grid = map_it.second;
        auto& mask2 = inner_grid.mask;

        for (auto inner_it = mask2.beginOn(); inner_it; ++inner_it)
        {
          auto inner_index = *inner_it;
          auto& leaf_grid = inner_grid.data[inner_index];
          // clang-format off
          int32_t xB = A.x + ((inner_index & MASK_INNER) << Log2DIM_LEAF);
          int32_t yB = A.y + (((inner_index >> Log2DIM_INNER) & MASK_INNER) << Log2DIM_LEAF);
          int32_t zB = A.z + (((inner_index >> (Log2DIM_INNER * 2)) & MASK_INNER) << Log2DIM_LEAF);
          // clang-format on
          auto& mask1 = leaf_grid->mask;

          for (auto leaf_it = mask1.beginOn(); leaf_it; ++leaf_it)
          {
            auto leaf_index = *leaf_it;

            int32_t pX = xB + ((leaf_index)&MASK_LEAF);
            int32_t pY = yB + ((leaf_index >> Log2DIM_LEAF) & MASK_LEAF);
            int32_t pZ = zB + ((leaf_index >> (Log2DIM_LEAF * 2)) & MASK_LEAF);

            CoordT pos = { pX, pY, pZ };
            func(leaf_grid->data[leaf_index], pos);
          }
        }
      }
    }

  private:
    RootMap& root_;
    CoordT prev_root_coord_;
    CoordT prev_inner_coord_;
    InnerGrid* prev_inner_ptr_ = nullptr;
    LeafGrid* prev_leaf_ptr_ = nullptr;

    static inline CoordT getRootCoord(const CoordT& coord)
    {
      constexpr static int32_t MASK = ~((1 << Log2N) - 1);
      return { coord.x & MASK, coord.y & MASK, coord.z & MASK };
    }

    static inline uint32_t getInnerIndex(const CoordT& coord)
    {
      constexpr static int32_t MASK = ((1 << Log2DIM_INNER) - 1);
      // clang-format off
      return ((coord.x >> Log2DIM_LEAF) & MASK) +
            (((coord.y >> Log2DIM_LEAF) & MASK) <<  Log2DIM_INNER) +
            (((coord.z >> Log2DIM_LEAF) & MASK) << (Log2DIM_INNER * 2));
      // clang-format on
    }

    static inline uint32_t getLeafIndex(const CoordT& coord)
    {
      constexpr static int32_t MASK = ((1 << Log2DIM_LEAF) - 1);
      // clang-format off
      return (coord.x & MASK) +
            ((coord.y & MASK) <<  Log2DIM_LEAF) +
            ((coord.z & MASK) << (Log2DIM_LEAF * 2));
      // clang-format on
    }
  };

  Accessor createAccessor()
  {
    return Accessor(root_map);
  }
};

template <typename DataT, int Log2DIM_INNER, int Log2DIM_LEAF>
inline bool VoxelGrid<DataT, Log2DIM_INNER, Log2DIM_LEAF>::Accessor::setValue(
    const CoordT& coord, const DataT& value)
{
  LeafGrid* leaf_ptr = prev_leaf_ptr_;

  const CoordT inner_key = { coord.x & ~((1 << Log2DIM_LEAF) - 1),
                             coord.y & ~((1 << Log2DIM_LEAF) - 1),
                             coord.z & ~((1 << Log2DIM_LEAF) - 1) };

  if( inner_key != prev_inner_coord_ || !prev_leaf_ptr_ )
  {
      InnerGrid* inner_ptr = prev_inner_ptr_;
      const CoordT root_key = getRootCoord(coord);

      // check if the key is the same as prev_inner_ptr_
      if (root_key != prev_root_coord_ || !prev_inner_ptr_)
      {
        auto root_it = root_.find(root_key);
        // Not found: create a new entry in the map
        if (root_it == root_.end())
        {
          root_it = root_.insert({ root_key, InnerGrid() }).first;
        }
        inner_ptr = &(root_it->second);
        // update the cache
        prev_root_coord_ = root_key;
        prev_inner_ptr_ = inner_ptr;
      }

      const uint32_t inner_index = getInnerIndex(coord);

      auto& inner_data = inner_ptr->data[inner_index];
      if (inner_ptr->mask.setOn(inner_index) == false)
      {
        inner_data = std::make_unique<LeafGrid>();
      }

      leaf_ptr = inner_data.get();
      prev_inner_coord_ = inner_key;
      prev_leaf_ptr_ = leaf_ptr;
  }

  const uint32_t leaf_index = getLeafIndex(coord);

  bool was_on = leaf_ptr->mask.setOn(leaf_index);
  leaf_ptr->data[leaf_index] = value;
  return !was_on;
}

template <typename DataT, int Log2DIM_INNER, int Log2DIM_LEAF>
inline const DataT*
VoxelGrid<DataT, Log2DIM_INNER, Log2DIM_LEAF>::Accessor::value(const CoordT& coord) const
{
  InnerGrid* inner_ptr = prev_inner_ptr_;

  const CoordT root_key = getRootCoord(coord);

  if (root_key != prev_root_coord_ || !prev_inner_ptr_)
  {
    auto it = root_.find(root_key);
    if (it == root_.end())
    {
      return nullptr;
    }
    inner_ptr = it->second;
    // update the cache
    prev_root_coord_ = root_key;
    prev_inner_ptr_ = inner_ptr;
  }

  const uint32_t inner_index = getInnerIndex(coord);

  auto& inner_data = inner_ptr->data[inner_index];

  if (!inner_ptr->mask.isOn(inner_index))
  {
    return nullptr;
  }

  LeafGrid* leaf_ptr = &(inner_ptr->data[inner_index]);
  const uint32_t leaf_index = getLeafIndex(coord);

  if (!leaf_ptr->mask.isOn(leaf_index))
  {
    return nullptr;
  }
  return &(leaf_ptr->data[leaf_index]);
}

}  // namespace Treexy

#endif  // TREEXY_HPP
