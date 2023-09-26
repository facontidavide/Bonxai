/*
 * Copyright Contributors to the Bonxai Project
 * Copyright Contributors to the OpenVDB Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <type_traits>
#include <unordered_map>

namespace Bonxai
{

struct Point2D
{
  double x;
  double y;

  Point2D() = default;

  Point2D(const Point2D& v) = default;
  Point2D(Point2D&& v) = default;

  Point2D& operator=(const Point2D& v) = default;
  Point2D& operator=(Point2D&& v) = default;

  Point2D(double x, double y);
};

struct Coord2D
{
  int32_t x;
  int32_t y;

  [[nodiscard]] bool operator==(const Coord2D& other) const;
  [[nodiscard]] bool operator!=(const Coord2D& other) const;

  [[nodiscard]] Coord2D operator+(const Coord2D& other) const;
  [[nodiscard]] Coord2D operator-(const Coord2D& other) const;

  Coord2D& operator+=(const Coord2D& other);
  Coord2D& operator-=(const Coord2D& other);
};

[[nodiscard]] inline Coord2D PosToCoord(const Point2D& point, double inv_resolution)
{
  return { int32_t(point.x * inv_resolution) - std::signbit(point.x),
           int32_t(point.y * inv_resolution) - std::signbit(point.y) };
}

[[nodiscard]] inline Point2D Coord2DoPos(const Coord2D& coord, double resolution)
{
  return { (double(coord.x) + 0.5) * resolution,
           (double(coord.y) + 0.5) * resolution};
}

//----------------------------------------------------------

/// Bit-mask to encode active states and facilitate sequential iterators.
class Mask2D
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
  Mask2D(size_t log2dim);
  /// Initialize all bits to a given value.
  Mask2D(size_t log2dim, bool on);

  Mask2D(const Mask2D& other);
  Mask2D(Mask2D&& other);

  ~Mask2D();

  /// Return the memory footprint in bytes of this Mask2D
  size_t memUsage() const;

  /// Return the number of bits available in this Mask2D
  uint32_t bitCount() const { return SIZE; }

  /// Return the number of machine words used by this Mask2D
  uint32_t wordCount() const { return WORD_COUNT; }

  uint64_t getWord(size_t n) const { return words_[n]; }

  void setWord(size_t n, uint64_t v) { words_[n] = v; }

  uint32_t countOn() const;

  class Iterator
  {
  public:
    Iterator(const Mask2D* parent)
      : pos_(parent->SIZE)
      , parent_(parent)
    {}
    Iterator(uint32_t pos, const Mask2D* parent)
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
    const Mask2D* parent_;
  };

  bool operator==(const Mask2D& other) const;

  bool operator!=(const Mask2D& other) const { return !((*this) == other); }

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

  /// Set all bits too the value "on"
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

//----------------------------------------------------------
/**
 * @brief The Grid2D class is used to store data in a 2D
 * square. The size (DIM) of the cube can only be a power of 2
 *
 * For instance, given Grid(3),
 * DIM will be 8 and SIZE 64 (8*8)
 */
template <typename DataT>
class Grid2D
{
private:
  uint8_t dim_;
  // total number of elements in the cube
  uint32_t size_;

  DataT* data_ = nullptr;
  Bonxai::Mask2D mask_;

public:
  Grid2D(size_t log2dim)
    : dim_(1 << log2dim)
    , size_(dim_ * dim_)
    , mask_(log2dim)
  {
    data_ = new DataT[size_];
  }

  Grid2D(const Grid2D& other) = delete;
  Grid2D& operator=(const Grid2D& other) = delete;

  Grid2D(Grid2D&& other);
  Grid2D& operator=(Grid2D&& other);

  ~Grid2D();

  [[nodiscard]] size_t memUsage() const;

  [[nodiscard]] size_t size() const { return size_; }

  [[nodiscard]] Bonxai::Mask2D& mask() { return mask_; };

  [[nodiscard]] const Bonxai::Mask2D& mask() const { return mask_; };

  [[nodiscard]] DataT& cell(size_t index) { return data_[index]; };

  [[nodiscard]] const DataT& cell(size_t index) const { return data_[index]; };
};

//----------------------------------------------------------
template <typename DataT>
class PixelGrid
{
public:
  const uint32_t LEAF_BITS;
  const uint32_t Log2N;
  const double resolution;
  const double inv_resolution;
  const uint32_t LEAF_MASK;

  using LeafGrid = Grid2D<DataT>;
  using RootMap = std::unordered_map<Coord2D, LeafGrid>;

  RootMap root_map;

  /**
   * @brief VoxelGrid constructor
   *
   * @param voxel_size  dimension of the voxel. Used to convert between Point2D and
   * Coord2D
   */
  PixelGrid(double voxel_size, uint8_t leaf_bits = 3);

  /// @brief getMemoryUsage returns the amount of bytes used by this data structure
  [[nodiscard]] size_t memUsage() const;

  /// @brief Return the total number of active cells
  [[nodiscard]] size_t activeCellsCount() const;

  /// @brief posToCoord is used to convert real coordinates to Coord2D indexes.
  [[nodiscard]] Coord2D posToCoord(double x, double y);

  /// @brief posToCoord is used to convert real coordinates to Coord2D indexes.
  [[nodiscard]] Coord2D posToCoord(const Point2D& pos)
  {
    return posToCoord(pos.x, pos.y);
  }

  /// @brief Coord2DoPos converts Coord2D indexes to Point2D.
  [[nodiscard]] Point2D Coord2DoPos(const Coord2D& coord);

  /**
   *  @brief forEachCell apply a function of type:
   *
   *      void(DataT*, const Coord2D&)
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
    Accessor(PixelGrid& grid)
      : grid_(grid)
    {}

    /**
     * @brief setValue of a cell. If the cell did not exist, it is created.
     *
     * @param coord   coordinate of the cell
     * @param value   value to set.
     * @return        the previous state of the cell (ON = true).
     */
    bool setValue(const Coord2D& coord, const DataT& value);

    /** @brief value getter.
     *
     * @param coord   coordinate of the cell.
     * @return        return the pointer to the value or nullptr if not set.
     */
    [[nodiscard]] DataT* value(const Coord2D& coord, bool create_if_missing = false);

    /** @brief setCellOn is similar to setValue, but the value is changed only if the
     * cell has been created, otherwise, the previous value is used.
     *
     * @param coord           coordinate of the cell.
     * @param default_value   default value of the cell. Use only if the cell did not
     * exist before.
     * @return                the previous state of the cell (ON = true).
     */
    bool setCellOn(const Coord2D& coord, const DataT& default_value = DataT());

    /** @brief setCellOff will disable a cell without deleting its content.
     *
     * @param coord   coordinate of the cell.
     * @return        the previous state of the cell (ON = true).
     */
    bool setCellOff(const Coord2D& coord);

    /// @brief lastLeafGrid returns the pointer to the LeafGrid in the cache.
    [[nodiscard]] const LeafGrid* lastLeafGrid() const { return prev_leaf_ptr_; }

    /**
     * @brief getLeafGrid gets the pointer to the LeafGrid containing the cell.
     * It is the basic class used by setValue() and value().
     *
     * @param coord               Coordinate of the cell.
     * @param create_if_missing   if true, create the Root, Inner and Leaf, if not
     * present.
     */
    [[nodiscard]] LeafGrid* getLeafGrid(const Coord2D& coord,
                                        bool create_if_missing = false);

  private:
    PixelGrid& grid_;
    Coord2D prev_root_coord_ = { std::numeric_limits<int32_t>::max(), 0 };
    LeafGrid* prev_leaf_ptr_ = nullptr;
  };

  Accessor createAccessor() { return Accessor(*this); }

  [[nodiscard]] Coord2D getRootKey(const Coord2D& coord);

  [[nodiscard]] uint32_t getLeafIndex(const Coord2D& coord);
};

//----------------------------------------------------
//----------------- Implementations ------------------
//----------------------------------------------------

inline Point2D::Point2D(double _x, double _y)
  : x(_x)
  , y(_y)
{}


inline bool Coord2D::operator==(const Coord2D& other) const
{
  return x == other.x && y == other.y;
}

inline bool Coord2D::operator!=(const Coord2D& other) const
{
  return !(*this == other);
}

inline Coord2D Coord2D::operator+(const Coord2D& other) const
{
  return { x + other.x, y + other.y };
}

inline Coord2D Coord2D::operator-(const Coord2D& other) const
{
  return { x - other.x, y - other.y };
}

inline Coord2D& Coord2D::operator+=(const Coord2D& other)
{
  x += other.x;
  y += other.y;
  return *this;
}

inline Coord2D& Coord2D::operator-=(const Coord2D& other)
{
  x -= other.x;
  y -= other.y;
  return *this;
}

template <typename DataT>
inline Grid2D<DataT>::Grid2D(Grid2D&& other)
  : dim_(other.dim_)
  , size_(other.size_)
  , mask_(std::move(other.mask_))
{
  std::swap(data_, other.data_);
}

template <typename DataT>
inline Grid2D<DataT>& Grid2D<DataT>::operator=(Grid2D&& other)
{
  dim_ = other.dim_;
  size_ = other.size_;
  mask_ = std::move(other.mask_);
  std::swap(data_, other.data_);
  return *this;
}

template <typename DataT>
inline Grid2D<DataT>::~Grid2D()
{
  if (data_)
  {
    delete[] data_;
  }
}

template <typename DataT>
inline size_t Grid2D<DataT>::memUsage() const
{
  return mask_.memUsage() + sizeof(uint8_t) + sizeof(uint32_t) + sizeof(DataT*) +
         sizeof(DataT) * size_;
}

template <typename DataT>
inline PixelGrid<DataT>::PixelGrid(double voxel_size,
                                   uint8_t leaf_bits)
  : LEAF_BITS(leaf_bits)
  , Log2N(LEAF_BITS)
  , resolution(voxel_size)
  , inv_resolution(1.0 / resolution)
  , LEAF_MASK((1 << LEAF_BITS) - 1)
{
  if (LEAF_BITS < 1)
  {
    throw std::runtime_error(
        "The minimum value of the inner_bits and leaf_bits should be 1");
  }
}

template <typename DataT>
inline Coord2D PixelGrid<DataT>::posToCoord(double x, double y)
{
  return { static_cast<int32_t>(x * inv_resolution - std::signbit(x)),
           static_cast<int32_t>(y * inv_resolution - std::signbit(y)) };
}

template <typename DataT>
inline Point2D PixelGrid<DataT>::Coord2DoPos(const Coord2D& coord)
{
  return { (double(coord.x) + 0.5) * resolution,
           (double(coord.y) + 0.5) * resolution };
}

template <typename DataT>
inline Coord2D PixelGrid<DataT>::getRootKey(const Coord2D& coord)
{
  const int32_t MASK = ~((1 << Log2N) - 1);
  return { coord.x & MASK, coord.y & MASK };
}


template <typename DataT>
inline uint32_t PixelGrid<DataT>::getLeafIndex(const Coord2D& coord)
{
  // clang-format off
  return (coord.x & LEAF_MASK) |
         ((coord.y & LEAF_MASK) << LEAF_BITS);
  // clang-format on
}

template <typename DataT>
inline bool PixelGrid<DataT>::Accessor::setValue(const Coord2D& coord,
                                                 const DataT& value)
{
  getLeafGrid(coord, true);

  const uint32_t index = grid_.getLeafIndex(coord);
  const bool was_on = prev_leaf_ptr_->mask().setOn(index);
  prev_leaf_ptr_->cell(index) = value;
  return was_on;
}

//----------------------------------
template <typename DataT>
inline DataT* PixelGrid<DataT>::Accessor::value(const Coord2D& coord,
                                                bool create_if_missing)
{
  const Coord2D inner_key = grid_.getInnerKey(coord);
prev_leaf_ptr_ = getLeafGrid(coord, create_if_missing);

  if (prev_leaf_ptr_)
  {
    const uint32_t index = grid_.getLeafIndex(coord);
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

//----------------------------------
template <typename DataT>
inline bool PixelGrid<DataT>::Accessor::setCellOn(const Coord2D& coord,
                                                  const DataT& default_value)
{
  const Coord2D inner_key = grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, true);
    prev_inner_coord_ = inner_key;
  }
  uint32_t index = grid_.getLeafIndex(coord);
  bool was_on = prev_leaf_ptr_->mask.setOn(index);
  if (!was_on)
  {
    prev_leaf_ptr_->cell(index) = default_value;
  }
  return was_on;
}

//----------------------------------
template <typename DataT>
inline bool PixelGrid<DataT>::Accessor::setCellOff(const Coord2D& coord)
{
  const Coord2D inner_key = grid_.getInnerKey(coord);

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
inline typename PixelGrid<DataT>::LeafGrid*
PixelGrid<DataT>::Accessor::getLeafGrid(const Coord2D& coord, bool create_if_missing)
{
  InnerGrid* inner_ptr = prev_inner_ptr_;
  const Coord2D root_key = grid_.getRootKey(coord);

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
  auto& inner_data = inner_ptr->cell(inner_index);

  if (create_if_missing)
  {
    if (!inner_ptr->mask().setOn(inner_index))
    {
      inner_data = std::make_shared<LeafGrid>(grid_.LEAF_BITS);
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
inline size_t PixelGrid<DataT>::memUsage() const
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

  total_size += root_map.size() * (sizeof(Coord2D) + sizeof(void*));

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
inline size_t PixelGrid<DataT>::activeCellsCount() const
{
  size_t total_size = 0;

  for (const auto& [key, inner_grid] : root_map)
  {
    for (auto inner_it = inner_grid.mask().beginOn(); inner_it; ++inner_it)
    {
      const int32_t inner_index = *inner_it;
      auto& leaf_grid = inner_grid.cell(inner_index);
      total_size += leaf_grid->mask.countOn();
    }
  }
  return total_size;
}

//----------------------------------
template <typename DataT>
template <class VisitorFunction>
inline void PixelGrid<DataT>::forEachCell(VisitorFunction func)
{
  const int32_t MASK_LEAF = ((1 << LEAF_BITS) - 1);

  for (auto& map_it : root_map)
  {
    const auto& [xA, yA] = (map_it.first);
    auto& inner_grid = map_it.second;
    auto& mask2 = inner_grid.mask();

    for (auto inner_it = mask2.beginOn(); inner_it; ++inner_it)
    {
      const int32_t inner_index = *inner_it;
      // clang-format off
      int32_t xB = xA | ((inner_index & MASK_INNER) << LEAF_BITS);
      int32_t yB = yA | ((inner_index & MASK_INNER) << LEAF_BITS);
      // clang-format on

      auto& leaf_grid = inner_grid.cell(inner_index);
      auto& mask1 = leaf_grid->mask();

      for (auto leaf_it = mask1.beginOn(); leaf_it; ++leaf_it)
      {
        const int32_t leaf_index = *leaf_it;
        Coord2D pos = { xB | (leaf_index & MASK_LEAF),
                        yB | ((leaf_index >> LEAF_BITS) & MASK_LEAF)};
        // apply the visitor
        func(leaf_grid->cell(leaf_index), pos);
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

inline uint32_t Mask2D::FindLowestOn(uint64_t v)
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
  return DeBruijn[uint64_t((v & -v) * UINT64_C(0x022FDD63CC95386D)) >> 58];
#if defined(_MSC_VER) && !defined(__NVCC__)
#pragma warning(pop)
#endif

#endif
}

/// @return Number of bits that are on in the specified 64-bit word

inline uint32_t Mask2D::CountOn(uint64_t v)
{
#if defined(_MSC_VER) && defined(_M_X64)
  v = __popcnt64(v);
#elif (defined(__GNUC__) || defined(__clang__))
  v = __builtin_popcountll(v);
#else
  // Software Implementation
  v = v - ((v >> 1) & uint64_t(0x5555555555555555));
  v = (v & uint64_t(0x3333333333333333)) + ((v >> 2) & uint64_t(0x3333333333333333));
  v = (((v + (v >> 4)) & uint64_t(0xF0F0F0F0F0F0F0F)) *
       uint64_t(0x101010101010101)) >>
      56;
#endif
  return static_cast<uint32_t>(v);
}

inline void Mask2D::setOn()
{
  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = ~uint64_t(0);
  }
}

inline void Mask2D::setOff()
{
  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = uint64_t(0);
  }
}

inline void Mask2D::set(bool on)
{
  const uint64_t v = on ? ~uint64_t(0) : uint64_t(0);
  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = v;
  }
}

inline void Mask2D::toggle()
{
  uint32_t n = WORD_COUNT;
  for (auto* w = words_; n--; ++w)
  {
    *w = ~*w;
  }
}

inline void Mask2D::toggle(uint32_t n)
{
  words_[n >> 6] ^= uint64_t(1) << (n & 63);
}

inline uint32_t Mask2D::findFirstOn() const
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

inline uint32_t Mask2D::findNextOn(uint32_t start) const
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

inline Mask2D::Mask2D(size_t log2dim)
  : SIZE(1U << (2 * log2dim))
  , WORD_COUNT(std::max(SIZE >> 6, 1u))
{
  words_ = (WORD_COUNT <= 8) ? static_words_ : new uint64_t[WORD_COUNT];

  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = 0;
  }
}

inline Mask2D::Mask2D(size_t log2dim, bool on)
  : SIZE(1U << (2 * log2dim))
  , WORD_COUNT(std::max(SIZE >> 6, 1u))
{
  words_ = (WORD_COUNT <= 8) ? static_words_ : new uint64_t[WORD_COUNT];

  const uint64_t v = on ? ~uint64_t(0) : uint64_t(0);
  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = v;
  }
}

inline Mask2D::Mask2D(const Mask2D& other)
  : SIZE(other.SIZE)
  , WORD_COUNT(other.WORD_COUNT)
{
  words_ = (WORD_COUNT <= 8) ? static_words_ : new uint64_t[WORD_COUNT];

  for (uint32_t i = 0; i < WORD_COUNT; ++i)
  {
    words_[i] = other.words_[i];
  }
}

inline Mask2D::Mask2D(Mask2D&& other)
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

inline Mask2D::~Mask2D()
{
  if (words_ && WORD_COUNT > 8)
  {
    delete[] words_;
  }
}

inline size_t Mask2D::memUsage() const
{
  if (WORD_COUNT > 8)
  {
    return sizeof(Mask2D) + sizeof(uint64_t) * WORD_COUNT;
  }
  return sizeof(Mask2D);
}

inline uint32_t Mask2D::countOn() const
{
  uint32_t sum = 0, n = WORD_COUNT;
  for (const uint64_t* w = words_; n--; ++w)
  {
    sum += CountOn(*w);
  }
  return sum;
}

inline bool Mask2D::operator==(const Mask2D& other) const
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

inline bool Mask2D::isOn(uint32_t n) const
{
  return 0 != (words_[n >> 6] & (uint64_t(1) << (n & 63)));
}

inline bool Mask2D::isOn() const
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

inline bool Mask2D::isOff() const
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

inline bool Mask2D::setOn(uint32_t n)
{
  uint64_t& word = words_[n >> 6];
  const uint64_t on_bit = (uint64_t(1) << (n & 63));
  bool was_on = word & on_bit;
  word |= on_bit;
  return was_on;
}

inline bool Mask2D::setOff(uint32_t n)
{
  uint64_t& word = words_[n >> 6];
  const uint64_t on_bit = (uint64_t(1) << (n & 63));
  bool was_on = word & on_bit;
  word &= ~(on_bit);
  return was_on;
}

inline void Mask2D::set(uint32_t n, bool On)
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

}  // namespace Bonxai

namespace std
{
template <>
struct hash<Bonxai::Coord2D>
{
  std::size_t operator()(const Bonxai::Coord2D& p) const
  {
    // same a OpenVDB
    return ((1 << 20) - 1) & (p.x * 73856093 ^ p.y * 19349663);
  }
};
}  // namespace std
