#pragma once

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
#include <type_traits>

#include "bonxai/node_mask.hpp"

namespace Bonxai
{

// Type traits used in Point3D::operator=
template<class T, class = void>
struct type_has_method_x : std::false_type { };
template<class T>
struct type_has_method_x<T, std::void_t<decltype(T().x())>> : std::true_type { };

template<class T, class = void>
struct type_has_member_x : std::false_type { };
template<class T>
struct type_has_member_x<T, std::void_t<decltype(T::x)>> : std::true_type { };

template<class T, class = void>
struct type_has_operator : std::false_type { };
template<class T>
struct type_has_operator<T, std::void_t<decltype(T().operator[])>> : std::true_type { };


struct Point3D
{
  double x;
  double y;
  double z;

  Point3D() {}

  Point3D(double _x, double _y, double _z): x(_x), y(_y), z(_z) {}

  // This copy operator accepts types like
  // Eigen::Vector3d, std::array<double,3>, std::vector<double>, pcl::PointXYZ
  // of Point3D itself
  template <typename T>
  Point3D& operator =(const T& v) {

    static_assert(type_has_method_x<T>::value ||
                  type_has_member_x<T>::value ||
                  type_has_operator<T>::value,
                  "Can't assign values automatically to Point3D");

    if constexpr(type_has_method_x<T>::value) {
      x = v.x();
      y = v.y();
      z = v.z();
    }
    if constexpr(type_has_member_x<T>::value) {
      x = v.x;
      y = v.y;
      z = v.z;
    }
    if constexpr(type_has_operator<T>::value){
      x = v[0];
      y = v[1];
      z = v[2];
    }
    return *this;
  }

  template <typename T> Point3D(const T& v) {
    *this = v;
  }
};

struct CoordT
{
  int32_t x;
  int32_t y;
  int32_t z;

  int32_t& operator[](int index) {
    switch(index) {
      case 0: return x;
      case 1: return y;
      case 2: return z;
      default: throw std::runtime_error("out of bound index");
    }
  }

  bool operator==(const CoordT& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator!=(const CoordT& other) const
  {
    return !(*this == other);
  }
  CoordT operator+(const CoordT& other)
  {
    return {x + other.x, y + other.y, z + other.z};
  }
  CoordT operator-(const CoordT& other)
  {
    return {x - other.x, y - other.y, z - other.z};
  }
};

inline CoordT PosToCoord(const Point3D& point, double inv_resolution)
{
  return { int32_t(point.x * inv_resolution) - std::signbit(point.x),
           int32_t(point.y * inv_resolution) - std::signbit(point.y),
           int32_t(point.z * inv_resolution) - std::signbit(point.z) };
}

inline Point3D CoordToPos(const CoordT& coord, double resolution)
{
  const double half_resolution = 0.5 * resolution;
  return { half_resolution + double(coord.x * resolution),
           half_resolution + double(coord.y * resolution),
           half_resolution + double(coord.z * resolution) };
}

//--------------------------------------------

/**
 * @brief The Grid class is used to store data in a
 * cube. the size (DIM) of the cube can only be a power of 2
 *
 * For instance, given Grid(3),
 * DIM will be 8 and SIZE 520 (8³)
 */
template <typename DataT>
struct Grid
{
  // number of bits used to represent DIM
  uint32_t LOG2DIM;
  // dimension of the grid (side of the cube)
  uint32_t DIM;
  // total number of elements in the cube
  uint32_t SIZE;

  Grid(size_t log2dim)
    : LOG2DIM(log2dim)
    , DIM(1 << LOG2DIM)
    , SIZE(DIM * DIM * DIM)
    , mask(LOG2DIM)
  {
    data = new DataT[SIZE];
  }

  Grid(const Grid& other) = delete;
  Grid& operator=(const Grid& other) = delete;

  Grid(Grid&& other)
    : LOG2DIM(other.LOG2DIM)
    , DIM(other.DIM)
    , SIZE(other.SIZE)
    , mask(std::move(other.mask))
  {
    std::swap(data, other.data);
  }

  Grid& operator=(Grid&& other)
  {
    LOG2DIM = other.LOG2DIM;
    DIM = other.DIM;
    SIZE = other.SIZE;
    std::swap(data, other.data);
    return *this;
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

  DataT* data = nullptr;
  Bonxai::Mask mask;
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
  const double half_resolution;
  const uint32_t INNER_MASK;
  const uint32_t LEAF_MASK;

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

  size_t activeCellsCount() const;

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
     * @return        the previous state of the cell (ON = true).
     */
    bool setValue(const CoordT& coord, const DataT& value);

    /** @brief value getter.
     *
     * @param coord   coordinate of the cell.
     * @return        return the pointer to the value or nullptr if not set.
     */
    DataT* value(const CoordT& coord);

    /** @brief setCellOn is similar to setValue, but the value is changed only if the
     * cell has been created, otherwise, the previous value is used.
     *
     * @param coord           coordinate of the cell.
     * @param default_value   default value of the cell. Use only if the cell did not
     * exist before.
     * @return                the previous state of the cell (ON = true).
     */
    bool setCellOn(const CoordT& coord, const DataT& default_value = DataT());

    /** @brief setCellOff will disable a cell without deleting its content.
     *
     * @param coord   coordinate of the cell.
     * @return        the previous state of the cell (ON = true).
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
     * @brief getLeafGrid gets the pointer to the LeafGrid containing the cell.
     * It is the basic class used by setValue() and value().
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

}  // namespace Bonxai

//----------------------------------------------------
//----------------- Implementations ------------------
//----------------------------------------------------

namespace std
{
template <>
struct hash<Bonxai::CoordT>
{
  std::size_t operator()(const Bonxai::CoordT& p) const
  {
    // same a OpenVDB
    return ((1 << 20) - 1) & (p.x * 73856093 ^ p.y * 19349663 ^ p.z * 83492791);
  }
};
}  // namespace std

namespace Bonxai
{
template <typename DataT>
inline VoxelGrid<DataT>::VoxelGrid(double voxel_size,
                                   uint8_t inner_bits,
                                   uint8_t leaf_bits)
  : INNER_BITS(inner_bits)
  , LEAF_BITS(leaf_bits)
  , Log2N(INNER_BITS + LEAF_BITS)
  , resolution(voxel_size)
  , inv_resolution(1.0 / resolution)
  , half_resolution(0.5 * resolution)
  , INNER_MASK((1 << INNER_BITS) - 1)
  , LEAF_MASK((1 << LEAF_BITS) - 1)
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
  return { static_cast<int32_t>(x * inv_resolution - std::signbit(x)),
           static_cast<int32_t>(y * inv_resolution - std::signbit(y)),
           static_cast<int32_t>(z * inv_resolution - std::signbit(z)) };
}

template <typename DataT>
inline CoordT VoxelGrid<DataT>::posToCoord(double x, double y, double z)
{
  return { static_cast<int32_t>(x * inv_resolution) - std::signbit(x),
           static_cast<int32_t>(y * inv_resolution) - std::signbit(y),
           static_cast<int32_t>(z * inv_resolution) - std::signbit(z) };
}

template <typename DataT>
inline Point3D VoxelGrid<DataT>::coordToPos(const CoordT& coord)
{
  return { half_resolution + static_cast<double>(coord.x * resolution),
           half_resolution + static_cast<double>(coord.y * resolution),
           half_resolution + static_cast<double>(coord.z * resolution) };
}

template <typename DataT>
inline uint32_t VoxelGrid<DataT>::getInnerIndex(const CoordT& coord)
{
  return ((coord.x >> LEAF_BITS) & INNER_MASK) |
         (((coord.y >> LEAF_BITS) & INNER_MASK) << INNER_BITS) |
         (((coord.z >> LEAF_BITS) & INNER_MASK) << (INNER_BITS * 2));
}

template <typename DataT>
inline uint32_t VoxelGrid<DataT>::getLeafIndex(const CoordT& coord)
{
  return (coord.x & LEAF_MASK) |
         ((coord.y & LEAF_MASK) << LEAF_BITS) |
         ((coord.z & LEAF_MASK) << (LEAF_BITS * 2));
}

//----------------------------------
template <typename DataT>
inline bool VoxelGrid<DataT>::Accessor::setValue(const CoordT& coord,
                                                 const DataT& value)
{
  const CoordT inner_key = grid_.getInnerKey(coord);
  if (inner_key != prev_inner_coord_ || prev_leaf_ptr_ == nullptr)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, true);
    prev_inner_coord_ = inner_key;
  }

  uint32_t index = grid_.getLeafIndex(coord);
  bool was_on = prev_leaf_ptr_->mask.setOn(index);
  prev_leaf_ptr_->data[index] = value;
  return was_on;
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
template <typename DataT>
inline bool VoxelGrid<DataT>::Accessor::setCellOn(const CoordT& coord,
                                                  const DataT& default_value)
{
  const CoordT inner_key = grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, true);
    prev_inner_coord_ = inner_key;
  }
  uint32_t index = grid_.getLeafIndex(coord);
  bool was_on = prev_leaf_ptr_->mask.setOn(index);
  if (!was_on)
  {
    prev_leaf_ptr_->data[index] = default_value;
  }
  return was_on;
}

//----------------------------------
template <typename DataT>
inline bool VoxelGrid<DataT>::Accessor::setCellOff(const CoordT& coord)
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

template <typename DataT>
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

template <typename DataT>
inline size_t VoxelGrid<DataT>::activeCellsCount() const
{
  size_t total_size = 0;

  for (const auto& [key, inner_grid] : root_map)
  {
    for (auto inner_it = inner_grid.mask.beginOn(); inner_it; ++inner_it)
    {
      const int32_t inner_index = *inner_it;
      auto& leaf_grid = inner_grid.data[inner_index];
      total_size += leaf_grid->mask.countOn();
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

}  // namespace Bonxai
