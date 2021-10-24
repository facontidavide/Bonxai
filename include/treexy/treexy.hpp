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

template <typename DataT>
struct Grid
{
  Grid(size_t log2dim)
    : LOG2DIM(log2dim), DIM(1 << LOG2DIM), SIZE(DIM * DIM * DIM), mask(LOG2DIM)
  {
    data = new DataT[SIZE];
  }

  Grid(const Grid& other) = delete;

  Grid(Grid&& other)
    : LOG2DIM(other.LOG2DIM)
    , DIM(other.DIM)
    , SIZE(other.SIZE)
    , mask(std::move(other.mask))
  {
    std::swap(data, other.data);
  }

  ~Grid()
  {
    if (data)
    {
      delete[] data;
    }
  }

  size_t memUsage() const
  {
    return mask.memUsage() + sizeof(int32_t) * 3 + sizeof(DataT*) +
           sizeof(DataT) * SIZE;
  }

  const int32_t LOG2DIM;
  const int32_t DIM;
  const int32_t SIZE;
  DataT* data = nullptr;
  Treexy::Mask mask;
};

template <typename DataT>
class VoxelGrid
{
public:
  const int32_t INNER_BITS;
  const int32_t LEAF_BITS;
  const int32_t Log2N;
  const double resolution;
  const double inv_resolution;

  using LeafGrid = Grid<DataT>;
  using InnerGrid = Grid<std::shared_ptr<LeafGrid>>;
  using RootMap = std::unordered_map<CoordT, InnerGrid>;

  RootMap root_map;

  /**
   * @brief VoxelGrid constructor
   *
   * @param voxel_size  dimension of the voxel. Used to convert between Point3D and
   * CoordT
   */
  VoxelGrid(double voxel_size, uint8_t inner_bits = 2, uint8_t leaf_bits = 3)
    : INNER_BITS(inner_bits)
    , LEAF_BITS(leaf_bits)
    , Log2N(INNER_BITS + LEAF_BITS)
    , resolution(voxel_size)
    , inv_resolution(1.0 / resolution)
  {
  }

  /**
   * @brief getMemoryUsage returns the amount of bytes used by this data structure
   */
  size_t memUsage() const;

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
     * @param create_if_missing   if true, create the Root, Inner and Leaf, if not
     * present.
     */
    LeafGrid* getLeafGrid(const CoordT& coord, bool create_if_missing = false);

  private:
    VoxelGrid& grid_;
    CoordT prev_root_coord_ = { std::numeric_limits<int32_t>::max(), 0, 0 };
    CoordT prev_inner_coord_ = { std::numeric_limits<int32_t>::max(), 0, 0 };
    InnerGrid* prev_inner_ptr_ = nullptr;
    LeafGrid* prev_leaf_ptr_ = nullptr;
  };

  Accessor createAccessor()
  {
    return Accessor(*this);
  }

  inline CoordT getRootKey(const CoordT& coord)
  {
    const int32_t MASK = ~((1 << Log2N) - 1);
    return { coord.x & MASK, coord.y & MASK, coord.z & MASK };
  }

  inline CoordT getInnerKey(const CoordT& coord)
  {
    const int32_t MASK = ~((1 << LEAF_BITS) - 1);
    return { coord.x & MASK, coord.y & MASK, coord.z & MASK };
  }

  inline uint32_t getInnerIndex(const CoordT& coord)
  {
    const int32_t MASK = ((1 << INNER_BITS) - 1);
    // clang-format off
    return ((coord.x >> LEAF_BITS) & MASK) |
          (((coord.y >> LEAF_BITS) & MASK) <<  INNER_BITS) |
          (((coord.z >> LEAF_BITS) & MASK) << (INNER_BITS * 2));
    // clang-format on
  }

  inline uint32_t getLeafIndex(const CoordT& coord)
  {
    const int32_t MASK = ((1 << LEAF_BITS) - 1);
    // clang-format off
    return (coord.x & MASK) |
          ((coord.y & MASK) <<  LEAF_BITS) |
          ((coord.z & MASK) << (LEAF_BITS * 2));
    // clang-format on
  }
};

}  // namespace Treexy

#include "treexy_impl.hpp"

#endif  // TREEXY_HPP
