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
    : LOG2DIM(log2dim)
    , DIM(1 << LOG2DIM)
    , SIZE(DIM * DIM * DIM)
    , mask(LOG2DIM)
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

  const uint32_t LOG2DIM;
  const uint32_t DIM;
  const uint32_t SIZE;
  DataT* data = nullptr;
  Treexy::Mask mask;
};

template <typename DataT>
class VoxelGrid
{
public:
  const uint32_t INNER_BITS;
  const uint32_t LEAF_BITS;
  const uint32_t Log2N;
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
  VoxelGrid(double voxel_size, uint8_t inner_bits = 2, uint8_t leaf_bits = 3);

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
    Accessor(VoxelGrid& grid)
      : grid_(grid)
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
     * @param coord   coordinate of the cell.
     * @return        return the pointer to the value or nullptr if not set.
     */
    DataT* value(const CoordT& coord);

    /** @brief setCellOn is similar to setValue, but the value is changes only if the
     * cell has been created, otherwise the previous value is used.
     *
     * @param coord   coordinate of the cell.
     * @param default_value   default value of the cell. Use only if the cell did not exist before.
     * @return        return the previous state of the cell (ON = true).
     */
    bool setCellOn(const CoordT& coord, const DataT& default_value = DataT());

    /** @brief setCellOff will disable a cell without releting its content.
     *
     * @param coord   coordinate of the cell.
     * @return        return the previous state of the cell (ON = true).
     */
    bool setCellOff(const CoordT& coord);

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

  inline uint32_t getInnerIndex(const CoordT& coord);

  inline uint32_t getLeafIndex(const CoordT& coord);
};


}  // namespace Treexy

//----------------- Implementations ------------------

namespace std
{
template <>
struct hash<Treexy::CoordT>
{
  std::size_t operator()(const Treexy::CoordT& p) const
  {
    // same a OpenVDB
    return ((1 << 20) - 1) & (p.x * 73856093 ^ p.y * 19349663 ^ p.z * 83492791);
  }
};
}  // namespace std

namespace Treexy
{

template<typename DataT> inline
    VoxelGrid<DataT>::VoxelGrid(double voxel_size, uint8_t inner_bits, uint8_t leaf_bits)
  : INNER_BITS(inner_bits)
  , LEAF_BITS(leaf_bits)
  , Log2N(INNER_BITS + LEAF_BITS)
  , resolution(voxel_size)
  , inv_resolution(1.0 / resolution)
{
  if (LEAF_BITS < 1 || INNER_BITS < 1)
  {
    throw std::runtime_error(
        "The minimum value of the inner_bits and leaf_bits should be 1");
  }
}

template <typename DataT>
inline CoordT VoxelGrid<DataT>::posToCoord(float x, float y, float z)
{
  return { static_cast<int32_t>(x * inv_resolution) - std::signbit(x),
           static_cast<int32_t>(y * inv_resolution) - std::signbit(y),
           static_cast<int32_t>(z * inv_resolution) - std::signbit(z) };
}

template <typename DataT>
inline CoordT VoxelGrid<DataT>::posToCoord(double x, double y, double z)
{
  return { static_cast<int32_t>(x * inv_resolution) - std::signbit(x),
           static_cast<int32_t>(y * inv_resolution) - std::signbit(y),
           static_cast<int32_t>(z * inv_resolution) - std::signbit(z) };
}

template<typename DataT> inline
Point3D VoxelGrid<DataT>::coordToPos(const CoordT &coord)
{
  const double half_resolution = 0.5 * resolution;
  return { half_resolution + static_cast<double>(coord.x * resolution),
        half_resolution + static_cast<double>(coord.y * resolution),
        half_resolution + static_cast<double>(coord.z * resolution) };
}

template<typename DataT> inline
uint32_t VoxelGrid<DataT>::getInnerIndex(const CoordT &coord)
{
  const uint32_t MASK = ((1 << INNER_BITS) - 1);
  // clang-format off
  return ((coord.x >> LEAF_BITS) & MASK) |
      (((coord.y >> LEAF_BITS) & MASK) <<  INNER_BITS) |
      (((coord.z >> LEAF_BITS) & MASK) << (INNER_BITS * 2));
  // clang-format on
}

template<typename DataT> inline
uint32_t VoxelGrid<DataT>::getLeafIndex(const CoordT &coord)
{
  const uint32_t MASK = ((1 << LEAF_BITS) - 1);
  // clang-format off
  return (coord.x & MASK) |
      ((coord.y & MASK) <<  LEAF_BITS) |
      ((coord.z & MASK) << (LEAF_BITS * 2));
  // clang-format on
}

//----------------------------------
template <typename DataT>
inline void VoxelGrid<DataT>::Accessor::setValue(const CoordT& coord,
                                                 const DataT& value)
{
  const CoordT inner_key = grid_.getInnerKey(coord);
  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, true);
    prev_inner_coord_ = inner_key;
  }

  uint32_t index = grid_.getLeafIndex(coord);
  prev_leaf_ptr_->mask.setOn(index);
  prev_leaf_ptr_->data[index] = value;
}

//----------------------------------
template <typename DataT>
inline DataT* VoxelGrid<DataT>::Accessor::value(const CoordT& coord)
{
  const CoordT inner_key = grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, false);
    prev_inner_coord_ = inner_key;
  }
  if (prev_leaf_ptr_)
  {
    uint32_t index = grid_.getLeafIndex(coord);
    if (prev_leaf_ptr_->mask.isOn(index))
    {
      return &(prev_leaf_ptr_->data[index]);
    }
  }
  return nullptr;
}

//----------------------------------
template<typename DataT>
inline bool VoxelGrid<DataT>::Accessor::setCellOn(const CoordT &coord, const DataT& default_value)
{
  const CoordT inner_key = grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, true);
    prev_inner_coord_ = inner_key;
  }
  uint32_t index = grid_.getLeafIndex(coord);
  bool was_on = prev_leaf_ptr_->mask.setOn(index);
  if(!was_on)
  {
    prev_leaf_ptr_->data[index] = default_value;
  }
  return was_on;
}

//----------------------------------
template<typename DataT>
inline bool VoxelGrid<DataT>::Accessor::setCellOff(const CoordT &coord)
{
  const CoordT inner_key = grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, false);
    prev_inner_coord_ = inner_key;
  }
  if (prev_leaf_ptr_)
  {
    uint32_t index = grid_.getLeafIndex(coord);
    return prev_leaf_ptr_->mask.setOff(index);
  }
  return false;
}

//----------------------------------
template <typename DataT>
inline typename VoxelGrid<DataT>::LeafGrid*
VoxelGrid<DataT>::Accessor::getLeafGrid(const CoordT& coord, bool create_if_missing)
{
  InnerGrid* inner_ptr = prev_inner_ptr_;
  const CoordT root_key = grid_.getRootKey(coord);

  if (root_key != prev_root_coord_ || !inner_ptr)
  {
    auto it = grid_.root_map.find(root_key);
    if (it == grid_.root_map.end())
    {
      if (!create_if_missing)
      {
        return nullptr;
      }
      it = grid_.root_map.insert({ root_key, InnerGrid(grid_.INNER_BITS) }).first;
    }
    inner_ptr = &(it->second);

    // update the cache
    prev_root_coord_ = root_key;
    prev_inner_ptr_ = inner_ptr;
  }

  const uint32_t inner_index = grid_.getInnerIndex(coord);
  auto& inner_data = inner_ptr->data[inner_index];

  if (create_if_missing)
  {
    if (!inner_ptr->mask.setOn(inner_index))
    {
      inner_data = std::make_shared<LeafGrid>(grid_.LEAF_BITS);
    }
  }
  else
  {
    if (!inner_ptr->mask.isOn(inner_index))
    {
      return nullptr;
    }
  }

  return inner_data.get();
}


template<typename DataT>
inline size_t VoxelGrid<DataT>::memUsage() const
{
  size_t total_size = 0;

  for (unsigned i = 0; i < root_map.bucket_count(); ++i)
  {
    size_t bucket_size = root_map.bucket_size(i);
    if (bucket_size == 0)
    {
      total_size++;
    }
    else
    {
      total_size += bucket_size;
    }
  }

  total_size += root_map.size() * (sizeof(CoordT) + sizeof(void*));

  for (const auto& [key, inner_grid] : root_map)
  {
    total_size += inner_grid.memUsage();
    for (auto inner_it = inner_grid.mask.beginOn(); inner_it; ++inner_it)
    {
      const int32_t inner_index = *inner_it;
      auto& leaf_grid = inner_grid.data[inner_index];
      total_size += leaf_grid->memUsage();
    }
  }
  return total_size;
}

//----------------------------------
template <typename DataT>
template <class VisitorFunction>
inline void VoxelGrid<DataT>::forEachCell(VisitorFunction func)
{
  const int32_t MASK_LEAF = ((1 << LEAF_BITS) - 1);
  const int32_t MASK_INNER = ((1 << INNER_BITS) - 1);

  for (auto& map_it : root_map)
  {
    const auto& [xA, yA, zA] = (map_it.first);
    InnerGrid& inner_grid = map_it.second;
    auto& mask2 = inner_grid.mask;

    for (auto inner_it = mask2.beginOn(); inner_it; ++inner_it)
    {
      const int32_t inner_index = *inner_it;
      const int32_t INNER_BITS_2 = INNER_BITS * 2;
      // clang-format off
      int32_t xB = xA | ((inner_index & MASK_INNER) << LEAF_BITS);
      int32_t yB = yA | (((inner_index >> INNER_BITS) & MASK_INNER) << LEAF_BITS);
      int32_t zB = zA | (((inner_index >> (INNER_BITS_2)) & MASK_INNER) << LEAF_BITS);
      // clang-format on

      auto& leaf_grid = inner_grid.data[inner_index];
      auto& mask1 = leaf_grid->mask;

      for (auto leaf_it = mask1.beginOn(); leaf_it; ++leaf_it)
      {
        const int32_t leaf_index = *leaf_it;
        const int32_t LEAF_BITS_2 = LEAF_BITS * 2;
        CoordT pos = { xB | (leaf_index & MASK_LEAF),
                       yB | ((leaf_index >> LEAF_BITS) & MASK_LEAF),
                       zB | ((leaf_index >> (LEAF_BITS_2)) & MASK_LEAF) };
        // apply the visitor
        func(leaf_grid->data[leaf_index], pos);
      }
    }
  }
}

}  // namespace Treexy

#endif  // TREEXY_HPP
