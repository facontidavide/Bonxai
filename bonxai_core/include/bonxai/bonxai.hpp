/*
 * Copyright Contributors to the Bonxai Project
 * Copyright Contributors to the OpenVDB Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Bonxai
{

// Magically converts any representation of a point in 3D
// (type with x, y and z) to another one. Works with:
//
// - pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGB, etc
// - Eigen::Vector3d, Eigen::Vector3f
// - custom type with x,y,z fields. In this case Bonxai::Point3D
// - arrays or vectors with 3 elements.

template <typename PointOut, typename PointIn>
PointOut ConvertPoint(const PointIn& v);

struct Point3D
{
  double x;
  double y;
  double z;

  Point3D() = default;

  Point3D(const Point3D& v) = default;
  Point3D(Point3D&& v) = default;

  Point3D& operator=(const Point3D& v) = default;
  Point3D& operator=(Point3D&& v) = default;

  Point3D(double x, double y, double z);

  template <typename T>
  Point3D(const T& v)
  {
    *this = ConvertPoint<Point3D>(v);
  }

  template <typename T>
  Point3D& operator=(const T& v)
  {
    *this = ConvertPoint<Point3D>(v);
    return *this;
  }

  // Access to x, y, z, using index 0, 1, 2
  [[nodiscard]] double& operator[](size_t index);
};

struct CoordT
{
  int32_t x;
  int32_t y;
  int32_t z;

  // Access to x, y, z, using index 0, 1, 2
  [[nodiscard]] int32_t& operator[](size_t index);

  [[nodiscard]] bool operator==(const CoordT& other) const;
  [[nodiscard]] bool operator!=(const CoordT& other) const;

  [[nodiscard]] CoordT operator+(const CoordT& other) const;
  [[nodiscard]] CoordT operator-(const CoordT& other) const;

  CoordT& operator+=(const CoordT& other);
  CoordT& operator-=(const CoordT& other);
};

[[nodiscard]] inline CoordT PosToCoord(const Point3D& point, double inv_resolution)
{
  return { static_cast<int32_t>(std::nearbyint(point.x * inv_resolution)),
           static_cast<int32_t>(std::nearbyint(point.y * inv_resolution)),
           static_cast<int32_t>(std::nearbyint(point.z * inv_resolution)) };
}

[[nodiscard]] inline Point3D CoordToPos(const CoordT& coord, double resolution)
{
  return { (static_cast<double>(coord.x)) * resolution,
           (static_cast<double>(coord.y)) * resolution,
           (static_cast<double>(coord.z)) * resolution };
}

//----------------------------------------------------------

/// Bit-mask to encode active states and facilitate sequential iterators.
class Mask
{
  uint64_t* words_ = nullptr;
  // small object optimization, that will be used when
  // SIZE <= 512 bits, i.e LOG2DIM <= 3
  uint64_t static_words_[8];

public:
  // Number of bits in mask
  const uint32_t SIZE;
  // Number of 64 bit words
  const uint32_t WORD_COUNT;

  /// Initialize all bits to zero.
  Mask(size_t log2dim);
  /// Initialize all bits to a given value.
  Mask(size_t log2dim, bool on);

  Mask(const Mask& other);
  Mask(Mask&& other);

  ~Mask();

  /// Return the memory footprint in bytes of this Mask
  size_t memUsage() const;

  /// Return the number of bits available in this Mask
  uint32_t bitCount() const { return SIZE; }

  /// Return the number of machine words used by this Mask
  uint32_t wordCount() const { return WORD_COUNT; }

  uint64_t getWord(size_t n) const { return words_[n]; }

  void setWord(size_t n, uint64_t v) { words_[n] = v; }

  uint32_t countOn() const;

  class Iterator
  {
  public:
    Iterator(const Mask* parent)
      : pos_(parent->SIZE)
      , parent_(parent)
    {}
    Iterator(uint32_t pos, const Mask* parent)
      : pos_(pos)
      , parent_(parent)
    {}
    Iterator& operator=(const Iterator&) = default;

    uint32_t operator*() const { return pos_; }

    operator bool() const { return pos_ != parent_->SIZE; }

    Iterator& operator++()
    {
      pos_ = parent_->findNextOn(pos_ + 1);
      return *this;
    }

  private:
    uint32_t pos_;
    const Mask* parent_;
  };

  bool operator==(const Mask& other) const;

  bool operator!=(const Mask& other) const { return !((*this) == other); }

  Iterator beginOn() const { return Iterator(this->findFirstOn(), this); }

  /// Return true if the given bit is set.
  bool isOn(uint32_t n) const;
  /// Return true if any bit is set.
  bool isOn() const;

  bool isOff() const;

  bool setOn(uint32_t n);

  bool setOff(uint32_t n);

  void set(uint32_t n, bool On);

  /// Set all bits on
  void setOn();

  /// Set all bits off
  void setOff();

  /// Set all bits to the value "on"
  void set(bool on);

  /// Toggle the state of all bits in the mask
  void toggle();
  /// Toggle the state of one bit in the mask
  void toggle(uint32_t n);

private:
  uint32_t findFirstOn() const;
  uint32_t findNextOn(uint32_t start) const;

  static uint32_t FindLowestOn(uint64_t v);
  static uint32_t CountOn(uint64_t v);
};

// empty data structure used exclusively in BinaryVoxelGrid
struct EmptyVoxel
{
};

//----------------------------------------------------------
/**
 * @brief The Grid class is used to store data in a cube.
 * The size (DIM) of the cube can only be a power of 2.
 *
 * For instance, given Grid(3),
 * DIM will be 8 and SIZE 512 (8³)
 */
template <typename DataT>
class Grid
{
private:
  uint8_t dim_ = 0;
  // total number of elements in the cube
  uint32_t size_ = 0;

  DataT* data_ = nullptr;
  Bonxai::Mask mask_;
  bool external_memory_ = false;

public:
  Grid(size_t log2dim)
    : dim_(1 << log2dim)
    , mask_(log2dim)
  {
    if constexpr (!std::is_same_v<DataT, EmptyVoxel>)
    {
      size_ = dim_ * dim_ * dim_;
      data_ = new DataT[size_];
    }
  }

  Grid(size_t log2dim, DataT* preAllocatedMemory)
    : dim_(1 << log2dim)
    , data_(preAllocatedMemory)
    , mask_(log2dim)
    , external_memory_(true)
  {}

  Grid(const Grid& other) = delete;
  Grid& operator=(const Grid& other) = delete;

  Grid(Grid&& other);
  Grid& operator=(Grid&& other);

  ~Grid()
  {
    if (!external_memory_)
    {
      delete[] data_;
    }
  }

  [[nodiscard]] size_t memUsage() const;

  [[nodiscard]] size_t size() const { return size_; }

  [[nodiscard]] Bonxai::Mask& mask() { return mask_; };

  [[nodiscard]] const Bonxai::Mask& mask() const { return mask_; }

  [[nodiscard]] DataT& cell(size_t index)
  {
    static_assert(!std::is_same_v<DataT, EmptyVoxel>,
                  "Cells have no value, when DataT == EmptyVoxel");
    return data_[index];
  }

  [[nodiscard]] const DataT& cell(size_t index) const
  {
    static_assert(!std::is_same_v<DataT, EmptyVoxel>,
                  "Cells have no value, when DataT == EmptyVoxel");
    return data_[index];
  }
};

//----------------------------------------------------------

enum ClearOption
{
  // reset the entire grid, freeing all the memory allocated so far
  CLEAR_MEMORY,
  // keep the memory allocated, but set all the cells to OFF
  SET_ALL_CELLS_OFF
};

// This is a simplistic allocator that can only grow and will never
// release memory. Good option in many cases
template <typename DataT>
class SimpleBlockAllocator
{
public:
  SimpleBlockAllocator(size_t block_bytes, int blocks_per_chunk = 1024);
  DataT* allocateBlock();
  void clear();

private:
  size_t blocks_per_chunk_ = 0;
  size_t block_bytes_ = 0;
  size_t capacity_ = 0;
  size_t size_ = 0;
  std::vector<std::shared_ptr<std::vector<char>>> chunks_;

  void addNewChunk();
};

template <typename DataT>
class VoxelGrid
{
private:
  uint32_t INNER_BITS;
  uint32_t LEAF_BITS;
  uint32_t Log2N;
  double resolution;
  double inv_resolution;
  uint32_t INNER_MASK;
  uint32_t LEAF_MASK;

  SimpleBlockAllocator<DataT> leaf_block_allocator_;

public:
  using LeafGrid = Grid<DataT>;
  using InnerGrid = Grid<std::shared_ptr<LeafGrid>>;
  using RootMap = std::unordered_map<CoordT, InnerGrid>;

  /**
   * @brief VoxelGrid constructor
   *
   * @param voxel_size  dimension of the voxel. Used to convert between Point3D and
   * CoordT
   */
  explicit VoxelGrid(double voxel_size,
                     uint8_t inner_bits = 2,
                     uint8_t leaf_bits = 3);

  VoxelGrid(const VoxelGrid&) = delete;
  VoxelGrid& operator=(const VoxelGrid&) = delete;

  VoxelGrid(VoxelGrid&& other) = default;
  VoxelGrid& operator=(VoxelGrid&& other) = default;

  uint32_t innetBits() const { return INNER_BITS; }
  uint32_t leafBits() const { return LEAF_BITS; }
  double voxelSize() const { return resolution; }

  const RootMap& rootMap() const { return root_map; }
  RootMap& rootMap() { return root_map; }

  /// @brief getMemoryUsage returns the amount of bytes used by this data structure
  [[nodiscard]] size_t memUsage() const;

  /// @brief Return the total number of active cells
  [[nodiscard]] size_t activeCellsCount() const;

  /// @brief posToCoord is used to convert real coordinates to CoordT indices.
  [[nodiscard]] CoordT posToCoord(double x, double y, double z) const;

  /// @brief posToCoord is used to convert real coordinates to CoordT indices.
  [[nodiscard]] CoordT posToCoord(const Point3D& pos) const
  {
    return posToCoord(pos.x, pos.y, pos.z);
  }

  /// @brief coordToPos converts CoordT indices to Point3D.
  [[nodiscard]] Point3D coordToPos(const CoordT& coord) const;

  /**
   *  @brief forEachCell apply a function of type:
   *
   *      void(const DataT&, const CoordT&)
   *
   * to each active element of the grid.
   */
  template <class VisitorFunction>
  void forEachCell(VisitorFunction func) const;

  /**
   *  @brief forEachCell apply a function of type:
   *
   *      void(DataT&, const CoordT&)
   *
   * to each active element of the grid.
   */
  template <class VisitorFunction>
  void forEachCell(VisitorFunction func)
  {
    static_cast<const VoxelGrid*>(this)->forEachCell(func);
  }

  void clear(ClearOption opt);

  class ConstAccessor
  {
  public:
    ConstAccessor(const VoxelGrid& grid)
      : grid_(grid)
    {}

    /** @brief value getter.
     *
     * @param coord   coordinate of the cell.
     * @return        return the pointer to the value or nullptr if not set.
     */
    [[nodiscard]] const DataT* value(const CoordT& coord) const;

    /**
     * @brief isCellOn only check if a cell is in "On" state
     * @param coordinate of the cell.
     */
    [[nodiscard]] bool isCellOn(const CoordT& coord) const;

    /// @brief lastInnerGrid returns the pointer to the InnerGrid in the cache.
    [[nodiscard]] const InnerGrid* lastInnerGrid() const { return prev_inner_ptr_; }

    /// @brief lastLeafGrid returns the pointer to the LeafGrid in the cache.
    [[nodiscard]] const LeafGrid* lastLeafGrid() const { return prev_leaf_ptr_; }

    [[nodiscard]] const LeafGrid* getLeafGrid(const CoordT& coord) const;

  protected:
    const VoxelGrid& grid_;
    mutable CoordT prev_root_coord_ = { std::numeric_limits<int32_t>::max(), 0, 0 };
    mutable CoordT prev_inner_coord_ = { std::numeric_limits<int32_t>::max(), 0, 0 };
    mutable const InnerGrid* prev_inner_ptr_ = nullptr;
    mutable const LeafGrid* prev_leaf_ptr_ = nullptr;
  };

  /** Class to be used to set and get values of a cell of the Grid.
   *  It uses caching to speed up computation.
   *
   *  Create an instance of this object with the method VoxelGrid::createAccessor()
   */
  class Accessor : public ConstAccessor
  {
  public:
    Accessor(VoxelGrid& grid)
      : ConstAccessor(grid)
      , mutable_grid_(grid)
    {}

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
    [[nodiscard]] DataT* value(const CoordT& coord, bool create_if_missing = false);

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
     * @brief getLeafGrid gets the pointer to the LeafGrid containing the cell.
     * It is the basic class used by setValue() and value().
     *
     * @param coord               Coordinate of the cell.
     * @param create_if_missing   if true, create the Root, Inner and Leaf, if not
     * present.
     */
    [[nodiscard]] LeafGrid* getLeafGrid(const CoordT& coord,
                                        bool create_if_missing = false);

  private:
    VoxelGrid& mutable_grid_;
    CoordT prev_root_coord_ = { std::numeric_limits<int32_t>::max(), 0, 0 };
    CoordT prev_inner_coord_ = { std::numeric_limits<int32_t>::max(), 0, 0 };
    InnerGrid* prev_inner_ptr_ = nullptr;
    LeafGrid* prev_leaf_ptr_ = nullptr;
  };

  Accessor createAccessor() { return Accessor(*this); }

  ConstAccessor createConstAccessor() const { return ConstAccessor(*this); }

  [[nodiscard]] CoordT getRootKey(const CoordT& coord) const;

  [[nodiscard]] CoordT getInnerKey(const CoordT& coord) const;

  [[nodiscard]] uint32_t getInnerIndex(const CoordT& coord) const;

  [[nodiscard]] uint32_t getLeafIndex(const CoordT& coord) const;

private:
  RootMap root_map;
};

using BinaryVoxelGrid = VoxelGrid<EmptyVoxel>;
//----------------------------------------------------
//----------------- Implementations ------------------
//----------------------------------------------------

inline Point3D::Point3D(double _x, double _y, double _z)
  : x(_x)
  , y(_y)
  , z(_z)
{}

inline double& Point3D::operator[](size_t index)
{
  switch (index)
  {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      throw std::runtime_error("out of bound index");
  }
}

// clang-format off
template <class T, class = void>
struct type_has_method_x : std::false_type {};
template <class T>
struct type_has_method_x<T, std::void_t<decltype(T().x())>> : std::true_type {};

template <class T, class = void>
struct type_has_member_x : std::false_type {};
template <class T>
struct type_has_member_x<T, std::void_t<decltype(T::x)>> : std::true_type {};

template<typename>
struct type_is_vector : std::false_type {};
template<typename T, typename A>
struct type_is_vector<std::vector<T, A>> : std::true_type {};
template<typename T>
struct type_is_vector<std::array<T, 3>> : std::true_type {};
// clang-format on

template <typename PointOut, typename PointIn>
inline PointOut ConvertPoint(const PointIn& v)
{
  // clang-format off
  static_assert(std::is_same_v<PointIn, PointOut> ||
                type_has_method_x<PointIn>::value ||
                type_has_member_x<PointIn>::value ||
                type_is_vector<PointIn>::value,
                "Can't convert from the specified type");

  static_assert(std::is_same_v<PointIn, PointOut> ||
                type_has_method_x<PointOut>::value ||
                type_has_member_x<PointOut>::value ||
                type_is_vector<PointOut>::value,
                "Can't convert to the specified type");

  // clang-format on
  if constexpr (std::is_same_v<PointIn, PointOut>)
  {
    return v;
  }
  if constexpr (type_has_method_x<PointIn>::value)
  {
    return { v.x(), v.y(), v.z() };
  }
  if constexpr (type_has_member_x<PointIn>::value)
  {
    return { v.x, v.y, v.z };
  }
  if constexpr (type_is_vector<PointIn>::value)
  {
    return { v[0], v[1], v[2] };
  }
}

inline int32_t& CoordT::operator[](size_t index)
{
  switch (index)
  {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      throw std::runtime_error("out of bound index");
  }
}

inline bool CoordT::operator==(const CoordT& other) const
{
  return x == other.x && y == other.y && z == other.z;
}

inline bool CoordT::operator!=(const CoordT& other) const
{
  return !(*this == other);
}

inline CoordT CoordT::operator+(const CoordT& other) const
{
  return { x + other.x, y + other.y, z + other.z };
}

inline CoordT CoordT::operator-(const CoordT& other) const
{
  return { x - other.x, y - other.y, z - other.z };
}

inline CoordT& CoordT::operator+=(const CoordT& other)
{
  x += other.x;
  y += other.y;
  z += other.z;
  return *this;
}

inline CoordT& CoordT::operator-=(const CoordT& other)
{
  x -= other.x;
  y -= other.y;
  z -= other.z;
  return *this;
}

template <typename DataT>
inline Grid<DataT>::Grid(Grid&& other)
  : dim_(other.dim_)
  , size_(other.size_)
  , mask_(std::move(other.mask_))
{
  std::swap(data_, other.data_);
}

template <typename DataT>
inline Grid<DataT>& Grid<DataT>::operator=(Grid&& other)
{
  dim_ = other.dim_;
  size_ = other.size_;
  mask_ = std::move(other.mask_);
  std::swap(data_, other.data_);
  return *this;
}

template <typename DataT>
inline size_t Grid<DataT>::memUsage() const
{
  auto mem = mask_.memUsage() + sizeof(uint8_t) + sizeof(uint32_t) + sizeof(DataT*);
  if constexpr (!std::is_same_v<DataT, EmptyVoxel>)
  {
    mem += sizeof(DataT) * size_;
  }
  return mem;
}

template <typename DataT>
inline VoxelGrid<DataT>::VoxelGrid(double voxel_size,
                                   uint8_t inner_bits,
                                   uint8_t leaf_bits)
  : INNER_BITS(inner_bits)
  , LEAF_BITS(leaf_bits)
  , Log2N(INNER_BITS + LEAF_BITS)
  , resolution(voxel_size)
  , inv_resolution(1.0 / resolution)
  , INNER_MASK((1 << INNER_BITS) - 1)
  , LEAF_MASK((1 << LEAF_BITS) - 1)
  , leaf_block_allocator_((1 << LEAF_BITS) * (1 << LEAF_BITS) * (1 << LEAF_BITS) *
                          sizeof(DataT))
{
  if (LEAF_BITS < 1 || INNER_BITS < 1)
  {
    throw std::runtime_error(
        "The minimum value of the inner_bits and leaf_bits should be 1");
  }
}

template <typename DataT>
inline CoordT VoxelGrid<DataT>::posToCoord(double x, double y, double z) const
{
  return { static_cast<int32_t>(std::nearbyint(x * inv_resolution)),
           static_cast<int32_t>(std::nearbyint(y * inv_resolution)),
           static_cast<int32_t>(std::nearbyint(z * inv_resolution)) };
}

template <typename DataT>
inline Point3D VoxelGrid<DataT>::coordToPos(const CoordT& coord) const
{
  return { (static_cast<double>(coord.x)) * resolution,
           (static_cast<double>(coord.y)) * resolution,
           (static_cast<double>(coord.z)) * resolution };
}

template <typename DataT>
inline CoordT VoxelGrid<DataT>::getRootKey(const CoordT& coord) const
{
  const int32_t MASK = ~((1 << Log2N) - 1);
  return { coord.x & MASK, coord.y & MASK, coord.z & MASK };
}

template <typename DataT>
inline CoordT VoxelGrid<DataT>::getInnerKey(const CoordT& coord) const
{
  const int32_t MASK = ~((1 << LEAF_BITS) - 1);
  return { coord.x & MASK, coord.y & MASK, coord.z & MASK };
}

template <typename DataT>
inline uint32_t VoxelGrid<DataT>::getInnerIndex(const CoordT& coord) const
{
  // clang-format off
  return ((coord.x >> LEAF_BITS) & INNER_MASK) |
         (((coord.y >> LEAF_BITS) & INNER_MASK) << INNER_BITS) |
         (((coord.z >> LEAF_BITS) & INNER_MASK) << (INNER_BITS * 2));
  // clang-format on
}

template <typename DataT>
inline uint32_t VoxelGrid<DataT>::getLeafIndex(const CoordT& coord) const
{
  // clang-format off
  return (coord.x & LEAF_MASK) |
         ((coord.y & LEAF_MASK) << LEAF_BITS) |
         ((coord.z & LEAF_MASK) << (LEAF_BITS * 2));
  // clang-format on
}

template <typename DataT>
inline bool VoxelGrid<DataT>::Accessor::setValue(const CoordT& coord,
                                                 const DataT& value)
{
  static_assert(!std::is_same_v<DataT, EmptyVoxel>,
                "You can not access a value when using type EmptyVoxel. Use "
                "setCellOn / setCellOff");

  const CoordT inner_key = mutable_grid_.getInnerKey(coord);
  if (inner_key != prev_inner_coord_ || prev_leaf_ptr_ == nullptr)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, true);
    prev_inner_coord_ = inner_key;
  }

  const uint32_t index = mutable_grid_.getLeafIndex(coord);
  const bool was_on = prev_leaf_ptr_->mask().setOn(index);
  prev_leaf_ptr_->cell(index) = value;
  return was_on;
}

//----------------------------------
template <typename DataT>
inline DataT* VoxelGrid<DataT>::Accessor::value(const CoordT& coord,
                                                bool create_if_missing)
{
  static_assert(!std::is_same_v<DataT, EmptyVoxel>,
                "You can not access a value when using type EmptyVoxel. Use "
                "isCellOn / setCellOn / setCellOff");

  const CoordT inner_key = mutable_grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, create_if_missing);
    prev_inner_coord_ = inner_key;
  }

  if (prev_leaf_ptr_)
  {
    const uint32_t index = mutable_grid_.getLeafIndex(coord);
    if (prev_leaf_ptr_->mask().isOn(index))
    {
      return &(prev_leaf_ptr_->cell(index));
    }
    else if (create_if_missing)
    {
      prev_leaf_ptr_->mask().setOn(index);
      prev_leaf_ptr_->cell(index) = {};
      return &(prev_leaf_ptr_->cell(index));
    }
  }
  return nullptr;
}

template <typename DataT>
inline const DataT* VoxelGrid<DataT>::ConstAccessor::value(const CoordT& coord) const
{
  static_assert(!std::is_same_v<DataT, EmptyVoxel>,
                "You can not access a value when using type EmptyVoxel. Use "
                "isCellOn");

  const CoordT inner_key = grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord);
    prev_inner_coord_ = inner_key;
  }

  if (prev_leaf_ptr_)
  {
    const uint32_t index = grid_.getLeafIndex(coord);
    if (prev_leaf_ptr_->mask().isOn(index))
    {
      return &(prev_leaf_ptr_->cell(index));
    }
  }
  return nullptr;
}

template <typename DataT>
inline bool VoxelGrid<DataT>::ConstAccessor::isCellOn(const CoordT& coord) const
{
  const CoordT inner_key = grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord);
    prev_inner_coord_ = inner_key;
  }

  if (prev_leaf_ptr_)
  {
    const uint32_t index = grid_.getLeafIndex(coord);
    if (prev_leaf_ptr_->mask().isOn(index))
    {
      return true;
    }
  }
  return false;
}

//----------------------------------
template <typename DataT>
inline bool VoxelGrid<DataT>::Accessor::setCellOn(const CoordT& coord,
                                                  const DataT& default_value)
{
  const CoordT inner_key = mutable_grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, true);
    prev_inner_coord_ = inner_key;
  }
  uint32_t index = mutable_grid_.getLeafIndex(coord);
  bool was_on = prev_leaf_ptr_->mask().setOn(index);
  if constexpr (!std::is_same_v<DataT, EmptyVoxel>)
  {
    if (!was_on)
    {
      prev_leaf_ptr_->cell(index) = default_value;
    }
  }
  return was_on;
}

//----------------------------------
template <typename DataT>
inline bool VoxelGrid<DataT>::Accessor::setCellOff(const CoordT& coord)
{
  const CoordT inner_key = mutable_grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, false);
    prev_inner_coord_ = inner_key;
  }
  if (prev_leaf_ptr_)
  {
    uint32_t index = mutable_grid_.getLeafIndex(coord);
    return prev_leaf_ptr_->mask().setOff(index);
  }
  return false;
}

//----------------------------------
template <typename DataT>
inline typename VoxelGrid<DataT>::LeafGrid*
VoxelGrid<DataT>::Accessor::getLeafGrid(const CoordT& coord, bool create_if_missing)
{
  InnerGrid* inner_ptr = prev_inner_ptr_;
  const CoordT root_key = mutable_grid_.getRootKey(coord);

  if (root_key != prev_root_coord_ || !inner_ptr)
  {
    auto it = mutable_grid_.root_map.find(root_key);
    if (it == mutable_grid_.root_map.end())
    {
      if (!create_if_missing)
      {
        return nullptr;
      }
      it = mutable_grid_.root_map
               .insert({ root_key, InnerGrid(mutable_grid_.INNER_BITS) })
               .first;
    }
    inner_ptr = &(it->second);
    // update the cache
    prev_root_coord_ = root_key;
    prev_inner_ptr_ = inner_ptr;
  }

  const uint32_t inner_index = mutable_grid_.getInnerIndex(coord);
  auto& inner_data = inner_ptr->cell(inner_index);

  if (create_if_missing)
  {
    if (!inner_ptr->mask().setOn(inner_index))
    {
      if constexpr (std::is_trivial_v<DataT>)
      {
        DataT* preAllocatedMemory =
            mutable_grid_.leaf_block_allocator_.allocateBlock();
        inner_data =
            std::make_shared<LeafGrid>(mutable_grid_.LEAF_BITS, preAllocatedMemory);
      }
      else
      {
        inner_data = std::make_shared<LeafGrid>(mutable_grid_.LEAF_BITS);
      }
    }
  }
  else
  {
    if (!inner_ptr->mask().isOn(inner_index))
    {
      return nullptr;
    }
  }
  return inner_data.get();
}

template <typename DataT>
inline const typename VoxelGrid<DataT>::LeafGrid*
VoxelGrid<DataT>::ConstAccessor::getLeafGrid(const CoordT& coord) const
{
  const InnerGrid* inner_ptr = prev_inner_ptr_;
  const CoordT root_key = grid_.getRootKey(coord);

  if (root_key != prev_root_coord_ || !inner_ptr)
  {
    auto it = grid_.root_map.find(root_key);
    if (it == grid_.root_map.end())
    {
      return nullptr;
    }
    inner_ptr = &(it->second);
    // update the cache
    prev_root_coord_ = root_key;
    prev_inner_ptr_ = inner_ptr;
  }

  const uint32_t inner_index = grid_.getInnerIndex(coord);
  const auto& inner_data = inner_ptr->cell(inner_index);

  if (!inner_ptr->mask().isOn(inner_index))
  {
    return nullptr;
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
    for (auto inner_it = inner_grid.mask().beginOn(); inner_it; ++inner_it)
    {
      const int32_t inner_index = *inner_it;
      auto& leaf_grid = inner_grid.cell(inner_index);
      total_size += leaf_grid->memUsage();
    }
  }
  return total_size;
}

template <typename DataT>
inline void VoxelGrid<DataT>::clear(ClearOption opt)
{
  if (opt == CLEAR_MEMORY)
  {
    root_map.clear();
    leaf_block_allocator_.clear();
    return;
  }
  auto accessor = createAccessor();
  forEachCell([&accessor, this](DataT&, const CoordT& coord) {
    accessor.setCellOff(coord);
  });
}

template <typename DataT>
inline size_t VoxelGrid<DataT>::activeCellsCount() const
{
  size_t total_size = 0;

  for (const auto& [key, inner_grid] : root_map)
  {
    for (auto inner_it = inner_grid.mask().beginOn(); inner_it; ++inner_it)
    {
      const int32_t inner_index = *inner_it;
      auto& leaf_grid = inner_grid.cell(inner_index);
      total_size += leaf_grid->mask().countOn();
    }
  }
  return total_size;
}

//----------------------------------
template <typename DataT>
template <class VisitorFunction>
inline void VoxelGrid<DataT>::forEachCell(VisitorFunction func) const
{
  const int32_t MASK_LEAF = ((1 << LEAF_BITS) - 1);
  const int32_t MASK_INNER = ((1 << INNER_BITS) - 1);

  for (auto& map_it : root_map)
  {
    const auto& [xA, yA, zA] = (map_it.first);
    const InnerGrid& inner_grid = map_it.second;
    const auto& mask2 = inner_grid.mask();

    for (auto inner_it = mask2.beginOn(); inner_it; ++inner_it)
    {
      const int32_t inner_index = *inner_it;
      const int32_t INNER_BITS_2 = INNER_BITS * 2;
      // clang-format off
      int32_t xB = xA | ((inner_index & MASK_INNER) << LEAF_BITS);
      int32_t yB = yA | (((inner_index >> INNER_BITS) & MASK_INNER) << LEAF_BITS);
      int32_t zB = zA | (((inner_index >> (INNER_BITS_2)) & MASK_INNER) << LEAF_BITS);
      // clang-format on

      const auto& leaf_grid = inner_grid.cell(inner_index);
      const auto& mask1 = leaf_grid->mask();

      for (auto leaf_it = mask1.beginOn(); leaf_it; ++leaf_it)
      {
        const int32_t leaf_index = *leaf_it;
        const int32_t LEAF_BITS_2 = LEAF_BITS * 2;
        CoordT pos = { xB | (leaf_index & MASK_LEAF),
                       yB | ((leaf_index >> LEAF_BITS) & MASK_LEAF),
                       zB | ((leaf_index >> (LEAF_BITS_2)) & MASK_LEAF) };
        // apply the visitor
        if constexpr (std::is_same_v<DataT, EmptyVoxel>)
        {
          EmptyVoxel dummy{};
          func(dummy, pos);
        }
        else
        {
          func(leaf_grid->cell(leaf_index), pos);
        }
      }
    }
  }
}

//----------------------------------------------------------

#define BONXAI_USE_INTRINSICS

/// Returns the index of the lowest, i.e. least significant, on bit in
/// the specified 64 bit word
///
/// @warning Assumes that at least one bit is set in the word, i.e. @a v !=
/// uint32_t(0)!

inline uint32_t Mask::FindLowestOn(uint64_t v)
{
#if defined(_MSC_VER) && defined(BONXAI_USE_INTRINSICS)
  unsigned long index;
  _BitScanForward64(&index, v);
  return static_cast<uint32_t>(index);
#elif (defined(__GNUC__) || defined(__clang__)) && defined(BONXAI_USE_INTRINSICS)
  return static_cast<uint32_t>(__builtin_ctzll(v));
#else
  static const unsigned char DeBruijn[64] = {
    0,  1,  2,  53, 3,  7,  54, 27, 4,  38, 41, 8,  34, 55, 48, 28,
    62, 5,  39, 46, 44, 42, 22, 9,  24, 35, 59, 56, 49, 18, 29, 11,
    63, 52, 6,  26, 37, 40, 33, 47, 61, 45, 43, 21, 23, 58, 17, 10,
    51, 25, 36, 32, 60, 20, 57, 16, 50, 31, 19, 15, 30, 14, 13, 12,
  };
// disable unary minus on unsigned warning
#if defined(_MSC_VER) && !defined(__NVCC__)
#pragma warning(push)
#pragma warning(disable : 4146)
#endif
  /// @warning evil bit twiddling ahead!
  return DeBruijn[uint64_t((v & -v) * UINT64_C(0x022FDD63CC95386D)) >> 58];
#if defined(_MSC_VER) && !defined(__NVCC__)
#pragma warning(pop)
#endif

#endif
}

/// @return Number of bits that are on in the specified 64-bit word

inline uint32_t Mask::CountOn(uint64_t v)
{
#if defined(_MSC_VER) && defined(_M_X64)
  v = __popcnt64(v);
#elif (defined(__GNUC__) || defined(__clang__))
  v = __builtin_popcountll(v);
#else
  // Software Implementation
  /// @warning evil bit twiddling ahead!
  v = v - ((v >> 1) & uint64_t(0x5555555555555555));
  v = (v & uint64_t(0x3333333333333333)) + ((v >> 2) & uint64_t(0x3333333333333333));
  v = (((v + (v >> 4)) & uint64_t(0xF0F0F0F0F0F0F0F)) *
       uint64_t(0x101010101010101)) >>
      56;
#endif
  return static_cast<uint32_t>(v);
}

inline void Mask::setOn()
{
  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = ~uint64_t(0);
  }
}

inline void Mask::setOff()
{
  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = uint64_t(0);
  }
}

inline void Mask::set(bool on)
{
  const uint64_t v = on ? ~uint64_t(0) : uint64_t(0);
  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = v;
  }
}

inline void Mask::toggle()
{
  uint32_t n = WORD_COUNT;
  for (auto* w = words_; n--; ++w)
  {
    *w = ~*w;
  }
}

inline void Mask::toggle(uint32_t n)
{
  words_[n >> 6] ^= uint64_t(1) << (n & 63);
}

inline uint32_t Mask::findFirstOn() const
{
  const uint64_t* w = words_;
  uint32_t n = 0;
  while (n < WORD_COUNT && !*w)
  {
    ++w;
    ++n;
  }
  return n == WORD_COUNT ? SIZE : (n << 6) + FindLowestOn(*w);
}

inline uint32_t Mask::findNextOn(uint32_t start) const
{
  uint32_t n = start >> 6;  // initiate
  if (n >= WORD_COUNT)
  {
    return SIZE;  // check for out of bounds
  }
  uint32_t m = start & 63;
  uint64_t b = words_[n];
  if (b & (uint64_t(1) << m))
  {
    return start;  // simple case: start is on
  }
  b &= ~uint64_t(0) << m;  // mask out lower bits
  while (!b && ++n < WORD_COUNT)
  {
    b = words_[n];
  }                                                 // find next non-zero word
  return (!b ? SIZE : (n << 6) + FindLowestOn(b));  // catch last word=0
}

inline Mask::Mask(size_t log2dim)
  : SIZE(1U << (3 * log2dim))
  , WORD_COUNT(std::max(SIZE >> 6, 1u))
{
  words_ = (WORD_COUNT <= 8) ? static_words_ : new uint64_t[WORD_COUNT];

  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = 0;
  }
}

inline Mask::Mask(size_t log2dim, bool on)
  : SIZE(1U << (3 * log2dim))
  , WORD_COUNT(std::max(SIZE >> 6, 1u))
{
  words_ = (WORD_COUNT <= 8) ? static_words_ : new uint64_t[WORD_COUNT];

  const uint64_t v = on ? ~uint64_t(0) : uint64_t(0);
  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = v;
  }
}

inline Mask::Mask(const Mask& other)
  : SIZE(other.SIZE)
  , WORD_COUNT(other.WORD_COUNT)
{
  words_ = (WORD_COUNT <= 8) ? static_words_ : new uint64_t[WORD_COUNT];

  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = other.words_[i];
  }
}

inline Mask::Mask(Mask&& other)
  : SIZE(other.SIZE)
  , WORD_COUNT(other.WORD_COUNT)
{
  if (WORD_COUNT <= 8)
  {
    words_ = static_words_;
    for (uint32_t i = 0; i < WORD_COUNT; ++i)
    {
      words_[i] = other.words_[i];
    }
  }
  else
  {
    std::swap(words_, other.words_);
  }
}

inline Mask::~Mask()
{
  if (WORD_COUNT > 8)
  {
    delete[] words_;
  }
}

inline size_t Mask::memUsage() const
{
  if (WORD_COUNT > 8)
  {
    return sizeof(Mask) + sizeof(uint64_t) * WORD_COUNT;
  }
  return sizeof(Mask);
}

inline uint32_t Mask::countOn() const
{
  uint32_t sum = 0, n = WORD_COUNT;
  for (const uint64_t* w = words_; n--; ++w)
  {
    sum += CountOn(*w);
  }
  return sum;
}

inline bool Mask::operator==(const Mask& other) const
{
  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    if (words_[i] != other.words_[i])
    {
      return false;
    }
  }
  return true;
}

inline bool Mask::isOn(uint32_t n) const
{
  return 0 != (words_[n >> 6] & (uint64_t(1) << (n & 63)));
}

inline bool Mask::isOn() const
{
  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    if (words_[i] != ~uint64_t(0))
    {
      return false;
    }
  }
  return true;
}

inline bool Mask::isOff() const
{
  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    if (words_[i] != uint64_t(0))
    {
      return false;
    }
  }
  return true;
}

inline bool Mask::setOn(uint32_t n)
{
  uint64_t& word = words_[n >> 6];
  const uint64_t on_bit = (uint64_t(1) << (n & 63));
  bool was_on = word & on_bit;
  word |= on_bit;
  return was_on;
}

inline bool Mask::setOff(uint32_t n)
{
  uint64_t& word = words_[n >> 6];
  const uint64_t on_bit = (uint64_t(1) << (n & 63));
  bool was_on = word & on_bit;
  word &= ~(on_bit);
  return was_on;
}

inline void Mask::set(uint32_t n, bool On)
{
#if 1  // switch between branchless
  auto& word = words_[n >> 6];
  n &= 63;
  word &= ~(uint64_t(1) << n);
  word |= uint64_t(On) << n;
#else
  On ? this->setOn(n) : this->setOff(n);
#endif
}

template <typename DataT>
inline SimpleBlockAllocator<DataT>::SimpleBlockAllocator(size_t block_bytes,
                                                         int blocks_per_chunk)
  : blocks_per_chunk_(blocks_per_chunk)
  , block_bytes_(block_bytes)
{
  addNewChunk();
}

template <typename DataT>
inline DataT* SimpleBlockAllocator<DataT>::allocateBlock()
{
  if (size_ >= capacity_)
  {
    addNewChunk();
  }
  const size_t chunck_index = size_ / blocks_per_chunk_;
  const size_t item_index = size_ % blocks_per_chunk_;
  size_++;
  return reinterpret_cast<DataT*>(
      &(chunks_.at(chunck_index)->at(block_bytes_ * item_index)));
}

template <typename DataT>
inline void SimpleBlockAllocator<DataT>::clear()
{
  chunks_.clear();
}

template <typename DataT>
inline void SimpleBlockAllocator<DataT>::addNewChunk()
{
  chunks_.push_back(
      std::make_shared<std::vector<char>>(blocks_per_chunk_ * block_bytes_));
  capacity_ += blocks_per_chunk_;
}

}  // namespace Bonxai

namespace std
{
template <>
struct hash<Bonxai::CoordT>
{
  std::size_t operator()(const Bonxai::CoordT& p) const
  {
    // same as OpenVDB
    return ((1 << 20) - 1) & (static_cast<int64_t>(p.x) * 73856093 ^
                              static_cast<int64_t>(p.y) * 19349663 ^
                              static_cast<int64_t>(p.z) * 83492791);
  }
};
}  // namespace std
