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
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "grid_allocator.hpp"
#include "grid_coord.hpp"
#include "mask.hpp"

namespace Bonxai {

// empty data structure used exclusively in BinaryVoxelGrid
struct EmptyVoxel {};

/**
 * @brief The Grid class is used to store data in a cube.
 * The size (DIM) of the cube can only be a power of 2.
 *
 * For instance, given Grid(3),
 * DIM will be 8 and SIZE 512 (8³)
 */
template <typename DataT>
class Grid {
 private:
  uint8_t dim_ = 0;
  // total number of elements in the cube
  uint32_t size_ = 0;

  DataT* data_ = nullptr;
  Bonxai::Mask mask_;
  bool external_memory_ = false;

 public:
  Grid(size_t log2dim)
      : dim_(1 << log2dim),
        mask_(log2dim) {
    if constexpr (!std::is_same_v<DataT, EmptyVoxel>) {
      size_ = dim_ * dim_ * dim_;
      data_ = new DataT[size_];
    }
  }

  Grid(size_t log2dim, DataT* preAllocatedMemory)
      : dim_(1 << log2dim),
        data_(preAllocatedMemory),
        mask_(log2dim),
        external_memory_(true) {}

  Grid(const Grid& other) = delete;
  Grid& operator=(const Grid& other) = delete;

  Grid(Grid&& other);
  Grid& operator=(Grid&& other);

  ~Grid() {
    if (data_ != nullptr && !external_memory_) {
      delete[] data_;
    }
  }

  [[nodiscard]] size_t memUsage() const;

  [[nodiscard]] size_t size() const {
    return size_;
  }

  [[nodiscard]] Bonxai::Mask& mask() {
    return mask_;
  };

  [[nodiscard]] const Bonxai::Mask& mask() const {
    return mask_;
  }

  [[nodiscard]] DataT& cell(size_t index) {
    static_assert(
        !std::is_same_v<DataT, EmptyVoxel>, "Cells have no value, when DataT == EmptyVoxel");
    return data_[index];
  }

  [[nodiscard]] const DataT& cell(size_t index) const {
    static_assert(
        !std::is_same_v<DataT, EmptyVoxel>, "Cells have no value, when DataT == EmptyVoxel");
    return data_[index];
  }
};

//----------------------------------------------------------

/// Branching factors as a policy. StaticShape lets the index arithmetic fold
/// into constant shifts; DynamicShape is for shapes only known at runtime,
/// such as a grid read back by Deserialize().
template <uint32_t INNER = 2, uint32_t LEAF = 3>
struct StaticShape {
  static_assert(INNER >= 1, "The minimum value of inner_bits is 1");
  static_assert(LEAF >= 1, "The minimum value of leaf_bits is 1");

  static constexpr uint32_t INNER_BITS = INNER;
  static constexpr uint32_t LEAF_BITS = LEAF;
  static constexpr uint32_t LOG2_N = INNER_BITS + LEAF_BITS;
  static constexpr uint32_t INNER_MASK = (uint32_t(1) << INNER_BITS) - 1;
  static constexpr uint32_t LEAF_MASK = (uint32_t(1) << LEAF_BITS) - 1;

  StaticShape() = default;

  /// Lets VoxelGrid keep one constructor; rejects a shape it cannot represent.
  StaticShape(uint32_t inner_bits, uint32_t leaf_bits) {
    if (inner_bits != INNER_BITS || leaf_bits != LEAF_BITS) {
      throw std::runtime_error(
          "This VoxelGrid has a compile-time shape that does not match the requested "
          "inner_bits / leaf_bits. Use VoxelGrid<DataT, DynamicShape> instead.");
    }
  }
};

struct DynamicShape {
  uint32_t INNER_BITS = 2;
  uint32_t LEAF_BITS = 3;
  uint32_t LOG2_N = INNER_BITS + LEAF_BITS;
  uint32_t INNER_MASK = (uint32_t(1) << INNER_BITS) - 1;
  uint32_t LEAF_MASK = (uint32_t(1) << LEAF_BITS) - 1;

  DynamicShape() = default;

  DynamicShape(uint32_t inner_bits, uint32_t leaf_bits)
      : INNER_BITS(inner_bits),
        LEAF_BITS(leaf_bits),
        LOG2_N(inner_bits + leaf_bits),
        INNER_MASK((uint32_t(1) << inner_bits) - 1),
        LEAF_MASK((uint32_t(1) << leaf_bits) - 1) {
    if (INNER_BITS < 1 || LEAF_BITS < 1) {
      throw std::runtime_error("The minimum value of the inner_bits and leaf_bits should be 1");
    }
  }
};

//----------------------------------------------------------

enum ClearOption {
  // reset the entire grid, freeing all the memory allocated so far
  CLEAR_MEMORY,
  // keep the memory allocated, but set all the cells to OFF
  SET_ALL_CELLS_OFF
};

template <typename DataT, typename Shape = StaticShape<>>
class VoxelGrid {
 private:
  Shape shape_;  // empty for StaticShape
  double resolution;
  double inv_resolution;

  GridBlockAllocator<DataT> leaf_block_allocator_;

  // Bumped whenever inner/leaf nodes are released, so that the Accessors can tell that the
  // pointers they cached do not refer to live nodes anymore.
  uint32_t cache_epoch_ = 0;

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
  explicit VoxelGrid(double voxel_size, uint8_t inner_bits = 2, uint8_t leaf_bits = 3);

  VoxelGrid(const VoxelGrid&) = delete;
  VoxelGrid& operator=(const VoxelGrid&) = delete;

  VoxelGrid(VoxelGrid&& other) = default;
  VoxelGrid& operator=(VoxelGrid&& other) = default;

  uint32_t innetBits() const {
    return shape_.INNER_BITS;
  }
  uint32_t leafBits() const {
    return shape_.LEAF_BITS;
  }
  double voxelSize() const {
    return resolution;
  }

  const RootMap& rootMap() const {
    return root_map;
  }
  RootMap& rootMap() {
    return root_map;
  }

  /// @brief getMemoryUsage returns the amount of bytes used by this data structure
  [[nodiscard]] size_t memUsage() const;

  /**
   *  Try freeing memory;  this will discard grids where all the cells are OFF.
   *  Note that the memory release is NOT guaranteed, since we are using a memory pool too.
   *  Note that this invalidates the cache of the existing Accessors, but they will detect it
   *  and refresh themselves transparently on the next access.
   */
  void releaseUnusedMemory();

  /// @brief Return the total number of active cells
  [[nodiscard]] size_t activeCellsCount() const;

  /// @brief posToCoord is used to convert real coordinates to CoordT indices.
  [[nodiscard]] CoordT posToCoord(double x, double y, double z) const;

  /// @brief posToCoord is used to convert real coordinates to CoordT indices.
  [[nodiscard]] CoordT posToCoord(const Point3D& pos) const {
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
  void forEachCell(VisitorFunction func) {
    static_cast<const VoxelGrid*>(this)->forEachCell(func);
  }

  void clear(ClearOption opt);

  // You should never use this function directly. It is exposed in the public API only
  // for special cases, such as serialization or testing
  std::shared_ptr<LeafGrid> allocateLeafGrid();

  class ConstAccessor {
   public:
    ConstAccessor(const VoxelGrid& grid)
        : grid_(grid),
          cache_epoch_(grid.cache_epoch_) {}

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
    [[nodiscard]] const InnerGrid* lastInnerGrid() const {
      refreshCache();
      return prev_inner_ptr_;
    }

    /// @brief lastLeafGrid returns the pointer to the LeafGrid in the cache.
    [[nodiscard]] const LeafGrid* lastLeafGrid() const {
      refreshCache();
      return prev_leaf_ptr_;
    }

    [[nodiscard]] const LeafGrid* getLeafGrid(const CoordT& coord) const;

   protected:
    /// Drop the cache if the grid released the nodes that these pointers refer to.
    void refreshCache() const {
      if (cache_epoch_ == grid_.cache_epoch_) {
        return;
      }
      cache_epoch_ = grid_.cache_epoch_;
      prev_root_coord_ = {std::numeric_limits<int32_t>::max(), 0, 0};
      prev_inner_coord_ = {std::numeric_limits<int32_t>::max(), 0, 0};
      prev_inner_ptr_ = nullptr;
      prev_leaf_ptr_ = nullptr;
    }

    const VoxelGrid& grid_;
    mutable uint32_t cache_epoch_;
    mutable CoordT prev_root_coord_ = {std::numeric_limits<int32_t>::max(), 0, 0};
    mutable CoordT prev_inner_coord_ = {std::numeric_limits<int32_t>::max(), 0, 0};
    mutable const InnerGrid* prev_inner_ptr_ = nullptr;
    mutable const LeafGrid* prev_leaf_ptr_ = nullptr;
  };

  /** Class to be used to set and get values of a cell of the Grid.
   *  It uses caching to speed up computation.
   *
   *  Create an instance of this object with the method VoxelGrid::createAccessor()
   */
  class Accessor : public ConstAccessor {
   public:
    Accessor(VoxelGrid& grid)
        : ConstAccessor(grid),
          mutable_grid_(grid),
          cache_epoch_(grid.cache_epoch_) {}

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
    [[nodiscard]] LeafGrid* getLeafGrid(const CoordT& coord, bool create_if_missing = false);

   private:
    // Accessor shadows the cache of the base class with non-const pointers,
    // therefore it must track its own epoch.
    void refreshCache() {
      if (cache_epoch_ == mutable_grid_.cache_epoch_) {
        return;
      }
      cache_epoch_ = mutable_grid_.cache_epoch_;
      prev_root_coord_ = {std::numeric_limits<int32_t>::max(), 0, 0};
      prev_inner_coord_ = {std::numeric_limits<int32_t>::max(), 0, 0};
      prev_inner_ptr_ = nullptr;
      prev_leaf_ptr_ = nullptr;
    }

    VoxelGrid& mutable_grid_;
    uint32_t cache_epoch_;
    CoordT prev_root_coord_ = {std::numeric_limits<int32_t>::max(), 0, 0};
    CoordT prev_inner_coord_ = {std::numeric_limits<int32_t>::max(), 0, 0};
    InnerGrid* prev_inner_ptr_ = nullptr;
    LeafGrid* prev_leaf_ptr_ = nullptr;
  };

  Accessor createAccessor() {
    return Accessor(*this);
  }

  ConstAccessor createConstAccessor() const {
    return ConstAccessor(*this);
  }

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

template <typename DataT>
inline Grid<DataT>::Grid(Grid&& other)
    : dim_(other.dim_),
      size_(other.size_),
      mask_(std::move(other.mask_)) {
  std::swap(data_, other.data_);
}

template <typename DataT>
inline Grid<DataT>& Grid<DataT>::operator=(Grid&& other) {
  dim_ = other.dim_;
  size_ = other.size_;
  mask_ = std::move(other.mask_);
  std::swap(data_, other.data_);
  return *this;
}

template <typename DataT>
inline size_t Grid<DataT>::memUsage() const {
  auto mem = mask_.memUsage() + sizeof(uint8_t) + sizeof(uint32_t) + sizeof(DataT*);
  if (!std::is_same_v<DataT, EmptyVoxel> && !external_memory_) {
    mem += sizeof(DataT) * size_;
  }
  return mem;
}

template <typename DataT, typename Shape>
inline void VoxelGrid<DataT, Shape>::releaseUnusedMemory() {
  std::vector<CoordT> keys_to_delete;
  for (auto& [key, inner_grid] : root_map) {
    for (auto inner_it = inner_grid.mask().beginOn(); inner_it; ++inner_it) {
      const int32_t inner_index = *inner_it;
      auto& leaf_grid = inner_grid.cell(inner_index);
      if (leaf_grid->mask().isOff()) {
        inner_grid.mask().setOff(inner_index);
        leaf_grid.reset();
      }
    }
    if (inner_grid.mask().isOff()) {
      keys_to_delete.push_back(key);
    }
  }
  for (const auto& key : keys_to_delete) {
    root_map.erase(key);
  }
  leaf_block_allocator_.releaseUnusedMemory();
  ++cache_epoch_;
}

template <typename DataT, typename Shape>
inline VoxelGrid<DataT, Shape>::VoxelGrid(double voxel_size, uint8_t inner_bits, uint8_t leaf_bits)
    : shape_(inner_bits, leaf_bits),
      resolution(voxel_size),
      inv_resolution(1.0 / resolution),
      leaf_block_allocator_(shape_.LEAF_BITS) {}

template <typename DataT, typename Shape>
inline CoordT VoxelGrid<DataT, Shape>::posToCoord(double x, double y, double z) const {
  return PosToCoord({x, y, z}, inv_resolution);
}

template <typename DataT, typename Shape>
inline Point3D VoxelGrid<DataT, Shape>::coordToPos(const CoordT& coord) const {
  return CoordToPos(coord, resolution);
}

template <typename DataT, typename Shape>
inline CoordT VoxelGrid<DataT, Shape>::getRootKey(const CoordT& coord) const {
  const int32_t MASK = ~((1 << shape_.LOG2_N) - 1);
  return {coord.x & MASK, coord.y & MASK, coord.z & MASK};
}

template <typename DataT, typename Shape>
inline CoordT VoxelGrid<DataT, Shape>::getInnerKey(const CoordT& coord) const {
  const int32_t MASK = ~((1 << shape_.LEAF_BITS) - 1);
  return {coord.x & MASK, coord.y & MASK, coord.z & MASK};
}

template <typename DataT, typename Shape>
inline uint32_t VoxelGrid<DataT, Shape>::getInnerIndex(const CoordT& coord) const {
  // clang-format off
  const uint32_t leaf_bits = shape_.LEAF_BITS;
  const uint32_t inner_bits = shape_.INNER_BITS;
  const uint32_t inner_mask = shape_.INNER_MASK;
  return ((coord.x >> leaf_bits) & inner_mask) |
         (((coord.y >> leaf_bits) & inner_mask) << inner_bits) |
         (((coord.z >> leaf_bits) & inner_mask) << (inner_bits * 2));
  // clang-format on
}

template <typename DataT, typename Shape>
inline uint32_t VoxelGrid<DataT, Shape>::getLeafIndex(const CoordT& coord) const {
  // clang-format off
  const uint32_t leaf_bits = shape_.LEAF_BITS;
  const uint32_t leaf_mask = shape_.LEAF_MASK;
  return (coord.x & leaf_mask) |
         ((coord.y & leaf_mask) << leaf_bits) |
         ((coord.z & leaf_mask) << (leaf_bits * 2));
  // clang-format on
}

template <typename DataT, typename Shape>
inline bool VoxelGrid<DataT, Shape>::Accessor::setValue(const CoordT& coord, const DataT& value) {
  static_assert(
      !std::is_same_v<DataT, EmptyVoxel>,
      "You can not access a value when using type EmptyVoxel. Use "
      "setCellOn / setCellOff");

  refreshCache();
  const CoordT inner_key = mutable_grid_.getInnerKey(coord);
  if (inner_key != prev_inner_coord_ || prev_leaf_ptr_ == nullptr) {
    prev_leaf_ptr_ = getLeafGrid(coord, true);
    prev_inner_coord_ = inner_key;
  }

  const uint32_t index = mutable_grid_.getLeafIndex(coord);
  const bool was_on = prev_leaf_ptr_->mask().setOn(index);
  prev_leaf_ptr_->cell(index) = value;
  return was_on;
}

//----------------------------------
template <typename DataT, typename Shape>
inline DataT* VoxelGrid<DataT, Shape>::Accessor::value(
    const CoordT& coord, bool create_if_missing) {
  static_assert(
      !std::is_same_v<DataT, EmptyVoxel>,
      "You can not access a value when using type EmptyVoxel. Use "
      "isCellOn / setCellOn / setCellOff");

  refreshCache();
  const CoordT inner_key = mutable_grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_ || prev_leaf_ptr_ == nullptr) {
    prev_leaf_ptr_ = getLeafGrid(coord, create_if_missing);
    prev_inner_coord_ = inner_key;
  }

  if (prev_leaf_ptr_) {
    const uint32_t index = mutable_grid_.getLeafIndex(coord);
    if (prev_leaf_ptr_->mask().isOn(index)) {
      return &(prev_leaf_ptr_->cell(index));
    } else if (create_if_missing) {
      prev_leaf_ptr_->mask().setOn(index);
      prev_leaf_ptr_->cell(index) = {};
      return &(prev_leaf_ptr_->cell(index));
    }
  }
  return nullptr;
}

template <typename DataT, typename Shape>
inline const DataT* VoxelGrid<DataT, Shape>::ConstAccessor::value(const CoordT& coord) const {
  static_assert(
      !std::is_same_v<DataT, EmptyVoxel>,
      "You can not access a value when using type EmptyVoxel. Use isCellOn");

  refreshCache();
  const CoordT inner_key = grid_.getInnerKey(coord);

  // a cached nullptr must not be trusted: the leaf may have been created since then
  if (inner_key != prev_inner_coord_ || prev_leaf_ptr_ == nullptr) {
    prev_leaf_ptr_ = getLeafGrid(coord);
    prev_inner_coord_ = inner_key;
  }

  if (prev_leaf_ptr_) {
    const uint32_t index = grid_.getLeafIndex(coord);
    if (prev_leaf_ptr_->mask().isOn(index)) {
      return &(prev_leaf_ptr_->cell(index));
    }
  }
  return nullptr;
}

template <typename DataT, typename Shape>
inline bool VoxelGrid<DataT, Shape>::ConstAccessor::isCellOn(const CoordT& coord) const {
  refreshCache();
  const CoordT inner_key = grid_.getInnerKey(coord);

  // a cached nullptr must not be trusted: the leaf may have been created since then
  if (inner_key != prev_inner_coord_ || prev_leaf_ptr_ == nullptr) {
    prev_leaf_ptr_ = getLeafGrid(coord);
    prev_inner_coord_ = inner_key;
  }

  if (prev_leaf_ptr_) {
    const uint32_t index = grid_.getLeafIndex(coord);
    if (prev_leaf_ptr_->mask().isOn(index)) {
      return true;
    }
  }
  return false;
}

//----------------------------------
template <typename DataT, typename Shape>
inline bool VoxelGrid<DataT, Shape>::Accessor::setCellOn(
    const CoordT& coord, const DataT& default_value) {
  refreshCache();
  const CoordT inner_key = mutable_grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_ || prev_leaf_ptr_ == nullptr) {
    prev_leaf_ptr_ = getLeafGrid(coord, true);
    prev_inner_coord_ = inner_key;
  }
  uint32_t index = mutable_grid_.getLeafIndex(coord);
  bool was_on = prev_leaf_ptr_->mask().setOn(index);
  if constexpr (!std::is_same_v<DataT, EmptyVoxel>) {
    if (!was_on) {
      prev_leaf_ptr_->cell(index) = default_value;
    }
  }
  return was_on;
}

//----------------------------------
template <typename DataT, typename Shape>
inline bool VoxelGrid<DataT, Shape>::Accessor::setCellOff(const CoordT& coord) {
  refreshCache();
  const CoordT inner_key = mutable_grid_.getInnerKey(coord);

  if (inner_key != prev_inner_coord_) {
    prev_leaf_ptr_ = getLeafGrid(coord, false);
    prev_inner_coord_ = inner_key;
  }
  if (prev_leaf_ptr_) {
    uint32_t index = mutable_grid_.getLeafIndex(coord);
    return prev_leaf_ptr_->mask().setOff(index);
  }
  return false;
}

//----------------------------------
template <typename DataT, typename Shape>
inline typename std::shared_ptr<Grid<DataT>> VoxelGrid<DataT, Shape>::allocateLeafGrid() {
  if constexpr (std::is_trivial_v<DataT> && !std::is_same_v<DataT, EmptyVoxel>) {
    auto allocated = leaf_block_allocator_.allocateBlock();
    DataT* memory_block = allocated.first;
    auto deleter = [deleter_impl = std::move(allocated.second)](LeafGrid* ptr) {
      // `delete` runs the destructor: calling it explicitly too destroyed the
      // LeafGrid twice, double-freeing Mask::words_ when leaf_bits >= 4
      deleter_impl();
      delete ptr;
    };
    return std::shared_ptr<LeafGrid>(
        new LeafGrid(shape_.LEAF_BITS, memory_block), std::move(deleter));
  } else {
    return std::make_shared<LeafGrid>(shape_.LEAF_BITS);
  }
}

template <typename DataT, typename Shape>
inline typename VoxelGrid<DataT, Shape>::LeafGrid* VoxelGrid<DataT, Shape>::Accessor::getLeafGrid(
    const CoordT& coord, bool create_if_missing) {
  refreshCache();
  InnerGrid* inner_ptr = prev_inner_ptr_;
  const CoordT root_key = mutable_grid_.getRootKey(coord);

  if (root_key != prev_root_coord_ || !inner_ptr) {
    auto it = mutable_grid_.root_map.find(root_key);
    if (it == mutable_grid_.root_map.end()) {
      if (!create_if_missing) {
        return nullptr;
      }
      it = mutable_grid_.root_map.insert({root_key, InnerGrid(mutable_grid_.shape_.INNER_BITS)})
               .first;
    }
    inner_ptr = &(it->second);
    // update the cache
    prev_root_coord_ = root_key;
    prev_inner_ptr_ = inner_ptr;
  }

  const uint32_t inner_index = mutable_grid_.getInnerIndex(coord);
  auto& inner_data = inner_ptr->cell(inner_index);

  if (create_if_missing) {
    if (!inner_ptr->mask().setOn(inner_index)) {
      inner_data = mutable_grid_.allocateLeafGrid();
    }
  } else {
    if (!inner_ptr->mask().isOn(inner_index)) {
      return nullptr;
    }
  }
  return inner_data.get();
}

template <typename DataT, typename Shape>
inline const typename VoxelGrid<DataT, Shape>::LeafGrid*
VoxelGrid<DataT, Shape>::ConstAccessor::getLeafGrid(const CoordT& coord) const {
  refreshCache();
  const InnerGrid* inner_ptr = prev_inner_ptr_;
  const CoordT root_key = grid_.getRootKey(coord);

  if (root_key != prev_root_coord_ || !inner_ptr) {
    auto it = grid_.root_map.find(root_key);
    if (it == grid_.root_map.end()) {
      return nullptr;
    }
    inner_ptr = &(it->second);
    // update the cache
    prev_root_coord_ = root_key;
    prev_inner_ptr_ = inner_ptr;
  }

  const uint32_t inner_index = grid_.getInnerIndex(coord);
  const auto& inner_data = inner_ptr->cell(inner_index);

  if (!inner_ptr->mask().isOn(inner_index)) {
    return nullptr;
  }
  return inner_data.get();
}

template <typename DataT, typename Shape>
inline size_t VoxelGrid<DataT, Shape>::memUsage() const {
  size_t total_size = 0;

  for (unsigned i = 0; i < root_map.bucket_count(); ++i) {
    size_t bucket_size = root_map.bucket_size(i);
    if (bucket_size == 0) {
      total_size++;
    } else {
      total_size += bucket_size;
    }
  }

  total_size += root_map.size() * (sizeof(CoordT) + sizeof(void*));

  for (const auto& [key, inner_grid] : root_map) {
    total_size += inner_grid.memUsage();
    for (auto inner_it = inner_grid.mask().beginOn(); inner_it; ++inner_it) {
      const int32_t inner_index = *inner_it;
      auto& leaf_grid = inner_grid.cell(inner_index);
      total_size += leaf_grid->memUsage();
    }
  }
  // if we are using the memory pool, leaf_grid->memUsage() did not return the right
  // value
  total_size += leaf_block_allocator_.memUsage();
  return total_size;
}

template <typename DataT, typename Shape>
inline void VoxelGrid<DataT, Shape>::clear(ClearOption opt) {
  if (opt == CLEAR_MEMORY) {
    root_map.clear();
    leaf_block_allocator_.clear();
    ++cache_epoch_;
    return;
  }
  auto accessor = createAccessor();
  forEachCell([&accessor, this](DataT&, const CoordT& coord) { accessor.setCellOff(coord); });
}

template <typename DataT, typename Shape>
inline size_t VoxelGrid<DataT, Shape>::activeCellsCount() const {
  size_t total_size = 0;

  for (const auto& [key, inner_grid] : root_map) {
    for (auto inner_it = inner_grid.mask().beginOn(); inner_it; ++inner_it) {
      const int32_t inner_index = *inner_it;
      auto& leaf_grid = inner_grid.cell(inner_index);
      total_size += leaf_grid->mask().countOn();
    }
  }
  return total_size;
}

//----------------------------------
template <typename DataT, typename Shape>
template <class VisitorFunction>
inline void VoxelGrid<DataT, Shape>::forEachCell(VisitorFunction func) const {
  const int32_t LEAF_BITS = int32_t(shape_.LEAF_BITS);
  const int32_t INNER_BITS = int32_t(shape_.INNER_BITS);
  const int32_t MASK_LEAF = ((1 << LEAF_BITS) - 1);
  const int32_t MASK_INNER = ((1 << INNER_BITS) - 1);

  for (auto& map_it : root_map) {
    const auto& [xA, yA, zA] = (map_it.first);
    const InnerGrid& inner_grid = map_it.second;
    const auto& mask2 = inner_grid.mask();

    for (auto inner_it = mask2.beginOn(); inner_it; ++inner_it) {
      const int32_t inner_index = *inner_it;
      const int32_t INNER_BITS_2 = INNER_BITS * 2;
      // clang-format off
      int32_t xB = xA | ((inner_index & MASK_INNER) << LEAF_BITS);
      int32_t yB = yA | (((inner_index >> INNER_BITS) & MASK_INNER) << LEAF_BITS);
      int32_t zB = zA | (((inner_index >> (INNER_BITS_2)) & MASK_INNER) << LEAF_BITS);
      // clang-format on

      const auto& leaf_grid = inner_grid.cell(inner_index);
      const auto& mask1 = leaf_grid->mask();

      for (auto leaf_it = mask1.beginOn(); leaf_it; ++leaf_it) {
        const int32_t leaf_index = *leaf_it;
        const int32_t LEAF_BITS_2 = LEAF_BITS * 2;
        CoordT pos = {
            xB | (leaf_index & MASK_LEAF), yB | ((leaf_index >> LEAF_BITS) & MASK_LEAF),
            zB | ((leaf_index >> (LEAF_BITS_2)) & MASK_LEAF)};
        // apply the visitor
        if constexpr (std::is_same_v<DataT, EmptyVoxel>) {
          EmptyVoxel dummy{};
          func(dummy, pos);
        } else {
          func(leaf_grid->cell(leaf_index), pos);
        }
      }
    }
  }
}

}  // namespace Bonxai
