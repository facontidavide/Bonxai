#ifndef TREEXY_HPP
#define TREEXY_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <functional>
#include <cmath>
#include <iostream>
#include <atomic>
#include <mutex>
#include <shared_mutex>

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

template <typename DataT, int Log2DIM>
struct Grid
{
  Grid() = default;
  Grid(Grid&& other):
    data(std::move(other.data)),
    mask(std::move(other.mask))
  {}

  constexpr static int DIM = 1 << Log2DIM;
  constexpr static int SIZE = DIM * DIM * DIM;
  std::array<DataT, SIZE> data;
  Treexy::Mask<Log2DIM> mask;
  std::shared_mutex mutex;
};

template <typename DataT, int INNER_BITS = 2, int LEAF_BITS = 3>
struct VoxelGrid
{
  constexpr static int32_t Log2N = INNER_BITS + LEAF_BITS;

  using LeafGrid = Grid<DataT, LEAF_BITS>;
  using InnerGrid = Grid<std::shared_ptr<LeafGrid>, INNER_BITS>;
  using RootMap = std::unordered_map<CoordT, InnerGrid>;

  RootMap root_map;
  mutable std::mutex root_mutex;

  const double resolution;
  const double inv_resolution;
  const double half_resolution;

  /**
   * @brief VoxelGrid constructor
   *
   * @param voxel_size  dimension of the voxel. Used to convert between Point3D and
   * CoordT
   */
  VoxelGrid(double voxel_size)
    : resolution(voxel_size)
    , inv_resolution(1.0 / resolution)
    , half_resolution(0.5 * resolution)
  {
  }

  VoxelGrid(VoxelGrid&& other)
    : resolution(other.resolution)
    , inv_resolution(1.0 / resolution)
    , half_resolution(0.5 * resolution)
  {
    root_map = std::move(other.root_map);
  }

  /**
   * @brief getMemoryUsage returns the amount of bytes used by this data structure
   */
  size_t getMemoryUsage() const;

  /**
   * @brief posToCoord MUST be used to convert real coordinates to CoordT indexes.
   */
  inline CoordT posToCoord(float x, float y, float z);

  inline CoordT posToCoord(double x, double y, double z);

  inline CoordT posToCoord(const Point3D& pos)
  {
    return posToCoord(pos.x, pos.y, pos.z);
  }

  /**
   * @brief coordToPos converts CoordT indexes to Point3D.
   */
  Point3D coordToPos(const CoordT& coord);

  /**
   *  @brief forEachCell apply a function of type:
   *
   *      void(DataT*, const CoordT&)
   *
   * to each active element of the grid.
   */
  template <class VisitorFunction>
  void forEachCell(VisitorFunction func);

  struct CellInfo
  {
    LeafGrid* leaf_grid;
    uint32_t index;
  };

  /** Class to be used to set and get values of a cell of the Grid.
   *  It uses caching to speed up computation.
   *
   *  Create an instance of this object with the method VoxelGrid::greateAccessor()
   */
  class Accessor
  {
  public:
    Accessor(VoxelGrid& grid) : grid_(grid)
    {
    }

    /**
     * @brief setValue of a cell. If the cell did not exist, it is created.
     *
     * @param coord   coordinate of the cell
     * @param value   value to set.
     */
    void setValue(const CoordT& coord, const DataT& value);

    /** @brief value getter.
     *
     * @param coord   coordinate of the cell
     * @return        return the pointer to the value or nullptr if not set.
     */
    DataT* value(const CoordT& coord);

    /**
     * @brief lastInnerdGrid returns the pointer to the InnerGrid in the cache.
     */
    const InnerGrid* lastInnerdGrid() const
    {
      return prev_inner_ptr_;
    }

    /**
     * @brief lastLeafGrid returns the pointer to the LeafGrid in the cache.
     */
    const LeafGrid* lastLeafGrid() const
    {
      return prev_leaf_ptr_;
    }

    /**
     * @brief getCell gets the point to the LeafGrid containing the cell
     *        and its index. It is the basic class used by setValue() and value().
     *
     * @param coord               Coordinate of the cell.
     * @param create_if_missing   if true, create the Root, Inner and Leaf, if not present.
     */
    CellInfo getCell(const CoordT& coord, bool create_if_missing = false);

  private:
    VoxelGrid& grid_;
    CoordT prev_root_coord_;
    CoordT prev_inner_coord_;
    InnerGrid* prev_inner_ptr_ = nullptr;
    LeafGrid* prev_leaf_ptr_ = nullptr;
  };

  Accessor createAccessor()
  {
    return Accessor(*this);
  }

  static inline CoordT getRootKey(const CoordT& coord)
  {
    constexpr static int32_t MASK = ~((1 << Log2N) - 1);
    return { coord.x & MASK, coord.y & MASK, coord.z & MASK };
  }

  static inline CoordT getInnerKey(const CoordT& coord)
  {
    constexpr static int32_t MASK = ~((1 << LEAF_BITS) - 1);
    return { coord.x & MASK, coord.y & MASK, coord.z & MASK };
  }

  static inline uint32_t getInnerIndex(const CoordT& coord)
  {
    constexpr static int32_t MASK = ((1 << INNER_BITS) - 1);
    // clang-format off
    return ((coord.x >> LEAF_BITS) & MASK) +
          (((coord.y >> LEAF_BITS) & MASK) <<  INNER_BITS) +
          (((coord.z >> LEAF_BITS) & MASK) << (INNER_BITS * 2));
    // clang-format on
  }

  static inline uint32_t getLeafIndex(const CoordT& coord)
  {
    constexpr static int32_t MASK = ((1 << LEAF_BITS) - 1);
    // clang-format off
    return (coord.x & MASK) +
          ((coord.y & MASK) <<  LEAF_BITS) +
          ((coord.z & MASK) << (LEAF_BITS * 2));
    // clang-format on
  }
};

}  // namespace Treexy

#include "treexy_impl.hpp"

#endif  // TREEXY_HPP
