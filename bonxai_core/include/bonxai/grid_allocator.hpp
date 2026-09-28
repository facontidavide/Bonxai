/*
 * Copyright Contributors to the Bonxai Project
 * Copyright Contributors to the OpenVDB Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <cassert>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "mask.hpp"

namespace Bonxai {

/**
 * @brief The GridBlockAllocator is used to pre-allocate the meory of multiple Grids
 * in "chunks". It is a very simple memory pool.
 *
 * Each chunk allocates memory for 512 Grids
 */
template <typename DataT>
class GridBlockAllocator {
 public:
  // use log2dim of the block to be allocated
  GridBlockAllocator(size_t log2dim);

  GridBlockAllocator(const GridBlockAllocator&) = delete;
  GridBlockAllocator(GridBlockAllocator&&) = default;

  GridBlockAllocator& operator=(const GridBlockAllocator& other) = delete;
  GridBlockAllocator& operator=(GridBlockAllocator&& other) = default;

  /// What the leaves need when they give their block back. It does not live in the
  /// allocator, which moves with its VoxelGrid: every chunk points to it, and so every
  /// leaf, through the chunk its Deleter holds.
  struct State {
    std::mutex mutex;
    size_t size = 0;  // blocks in use
  };

  struct Chunk {
    explicit Chunk(std::shared_ptr<State> s)
        : mask(3, true),
          state(std::move(s)) {}
    Mask mask;
    std::shared_ptr<State> state;
    // not a std::vector: resize() would memset the whole chunk, and no cell is
    // read before it is written
    std::unique_ptr<char[]> data;
  };

  /// Gives a block back to the pool. A concrete type rather than a
  /// std::function, whose capture would not fit the small-object buffer and so
  /// cost one heap allocation per leaf.
  class Deleter {
   public:
    Deleter() = default;
    Deleter(std::shared_ptr<Chunk> chunk, uint32_t index)
        : chunk_(std::move(chunk)),
          index_(index) {}

    void operator()() const {
      assert(index_ < blocks_per_chunk);
      State& state = *chunk_->state;
      std::unique_lock lock(state.mutex);
      chunk_->mask.setOn(index_);
      state.size--;
    }

   private:
    std::shared_ptr<Chunk> chunk_;
    uint32_t index_ = 0;
  };

  std::pair<DataT*, Deleter> allocateBlock();

  /// Leaves still alive give their blocks back to the chunks they came from, which they
  /// keep alive, and not to the new ones.
  void clear() {
    chunks_.clear();
    state_ = std::make_shared<State>();
    capacity_ = 0;
  }

  void releaseUnusedMemory();

  size_t capacity() const {
    return capacity_;
  }

  size_t size() const {
    return state_->size;
  }

  size_t memUsage() const;

  // specific size to use for Mask(3)
  static constexpr size_t blocks_per_chunk = 512;

 protected:
  size_t log2dim_ = 0;
  size_t block_bytes_ = 0;
  size_t capacity_ = 0;
  std::vector<std::shared_ptr<Chunk>> chunks_;
  std::shared_ptr<State> state_;

  void addNewChunk();
};

//----------------------------------------------------
//----------------- Implementations ------------------
//----------------------------------------------------

template <typename DataT>
inline GridBlockAllocator<DataT>::GridBlockAllocator(size_t log2dim)
    : log2dim_(log2dim),
      block_bytes_(std::pow((1 << log2dim), 3) * sizeof(DataT)),
      state_(std::make_shared<State>()) {}

template <typename DataT>
inline std::pair<DataT*, typename GridBlockAllocator<DataT>::Deleter>
GridBlockAllocator<DataT>::allocateBlock() {
  std::unique_lock lock(state_->mutex);
  if (state_->size >= capacity_) {
    // Need more memory. Create a new chunk
    addNewChunk();
    // first index of new chunk is available
    std::shared_ptr<Chunk> chunk = chunks_.back();
    DataT* ptr = reinterpret_cast<DataT*>(chunk->data.get());
    chunk->mask.setOff(0);
    state_->size++;
    return {ptr, Deleter(chunk, 0)};
  }

  // There must be available memory, somewhere. Search in reverse order
  for (auto it = chunks_.rbegin(); it != chunks_.rend(); it++) {
    std::shared_ptr<Chunk>& chunk = (*it);
    auto mask_index = chunk->mask.findFirstOn();
    if (mask_index < chunk->mask.size()) {
      // found in this chunk
      size_t data_index = block_bytes_ * mask_index;
      DataT* ptr = reinterpret_cast<DataT*>(&chunk->data[data_index]);
      chunk->mask.setOff(mask_index);
      state_->size++;
      return {ptr, Deleter(chunk, mask_index)};
    }
  }
  throw std::logic_error("Unexpected end of GridBlockAllocator::allocateBlock");
}

template <typename DataT>
inline void GridBlockAllocator<DataT>::releaseUnusedMemory() {
  std::unique_lock lock(state_->mutex);
  int to_be_erased_count = 0;
  auto remove_if = std::remove_if(chunks_.begin(), chunks_.end(), [&](const auto& chunk) -> bool {
    bool notUsed = chunk->mask.isOn();
    to_be_erased_count += (notUsed) ? 1 : 0;
    return notUsed;
  });
  chunks_.erase(remove_if, chunks_.end());
  capacity_ -= to_be_erased_count * blocks_per_chunk;
}

template <typename DataT>
inline size_t GridBlockAllocator<DataT>::memUsage() const {
  return chunks_.size() * (sizeof(Chunk) + block_bytes_ * blocks_per_chunk);
}

template <typename DataT>
inline void GridBlockAllocator<DataT>::addNewChunk() {
  auto chunk = std::make_shared<Chunk>(state_);
  chunk->data.reset(new char[blocks_per_chunk * block_bytes_]);
  chunks_.push_back(chunk);
  capacity_ += blocks_per_chunk;
}

}  // namespace Bonxai
