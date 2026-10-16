/*
 * Copyright Contributors to the Bonxai Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "mask.hpp"

namespace Bonxai {

struct EmptyVoxel;

namespace details {

#if defined(__POPCNT__) || defined(__aarch64__) || (defined(_MSC_VER) && defined(_M_X64))
inline constexpr bool HARDWARE_POPCOUNT = true;
#else
// Mask::CountOn is then a function call, such as libgcc's without -mpopcnt
inline constexpr bool HARDWARE_POPCOUNT = false;
#endif

/// The number of bits on in v; without a popcount instruction, a SWAR sum rather than a call.
template <bool HARDWARE = HARDWARE_POPCOUNT>
inline uint32_t CountOn(uint64_t v) {
  if constexpr (HARDWARE) {
    return Mask::CountOn(v);
  } else {
    v -= (v >> 1) & 0x5555555555555555;
    v = (v & 0x3333333333333333) + ((v >> 2) & 0x3333333333333333);
    v = (v + (v >> 4)) & 0x0F0F0F0F0F0F0F0F;
    return static_cast<uint32_t>((v * 0x0101010101010101) >> 56);
  }
}

/// The number of bits on in the words before each word of a mask.
template <uint32_t WORD_COUNT>
struct WordOffsets {
  uint16_t before[WORD_COUNT] = {};
};

template <>
struct WordOffsets<1> {};

}  // namespace details

/// A mask of (2^LOG2DIM)^3 bits, sized at compile time, that counts the bits below any bit in
/// O(1): it keeps the number of bits on before each of its words.
template <uint32_t LOG2DIM>
class StaticMask : private details::WordOffsets<((uint32_t(1) << (3 * LOG2DIM)) + 63) / 64> {
 public:
  static constexpr uint32_t WORD_COUNT = ((uint32_t(1) << (3 * LOG2DIM)) + 63) / 64;
  static_assert(LOG2DIM <= 5, "The offsets of the words are 16 bits");

  [[nodiscard]] bool isOn(uint32_t n) const {
    return (words_[n >> 6] >> (n & 63)) & 1;
  }

  [[nodiscard]] bool isOff() const {
    for (uint32_t i = 0; i < WORD_COUNT; ++i) {
      if (words_[i] != 0) {
        return false;
      }
    }
    return true;
  }

  /// Turns bit n on; returns its previous state.
  bool setOn(uint32_t n) {
    const uint64_t bit = uint64_t(1) << (n & 63);
    const bool was_on = (words_[n >> 6] & bit) != 0;
    if (!was_on) {
      words_[n >> 6] |= bit;
      shiftOffsets(n >> 6, 1);
    }
    return was_on;
  }

  void setOff(uint32_t n) {
    const uint64_t bit = uint64_t(1) << (n & 63);
    if (words_[n >> 6] & bit) {
      words_[n >> 6] &= ~bit;
      shiftOffsets(n >> 6, -1);
    }
  }

  [[nodiscard]] uint32_t countOn() const {
    return offset(WORD_COUNT - 1) + details::CountOn(words_[WORD_COUNT - 1]);
  }

  /// The number of bits on below n: where the element of bit n sits in a packed array.
  [[nodiscard]] uint32_t rank(uint32_t n) const {
    const uint32_t word = n >> 6;
    return offset(word) + details::CountOn(words_[word] & ((uint64_t(1) << (n & 63)) - 1));
  }

  /// Calls func(n) for every bit n that is on, in increasing order.
  template <class Func>
  void forEachOn(Func&& func) const {
    for (uint32_t i = 0; i < WORD_COUNT; ++i) {
      for (uint64_t bits = words_[i]; bits != 0; bits &= bits - 1) {
        func(i * 64 + Mask::FindLowestOn(bits));
      }
    }
  }

 private:
  [[nodiscard]] uint32_t offset(uint32_t word) const {
    if constexpr (WORD_COUNT == 1) {
      return 0;
    } else {
      return this->before[word];
    }
  }

  void shiftOffsets(uint32_t word, int delta) {
    if constexpr (WORD_COUNT > 1) {
      for (uint32_t i = word + 1; i < WORD_COUNT; ++i) {
        this->before[i] = static_cast<uint16_t>(this->before[i] + delta);
      }
    }
  }

  uint64_t words_[WORD_COUNT] = {};
};

/// A leaf of CompactStorage: its mask, and the values of its cells on, packed in bit order.
template <typename DataT, uint32_t LOG2DIM>
class CompactLeaf {
  static constexpr bool HAS_VALUES = !std::is_same_v<DataT, EmptyVoxel>;
  static_assert(
      !HAS_VALUES || std::is_nothrow_move_assignable_v<DataT>,
      "CompactStorage moves the values of a leaf when a cell is turned on or off: DataT needs "
      "a noexcept move assignment");

 public:
  using Mask = StaticMask<LOG2DIM>;

  [[nodiscard]] const Mask& mask() const {
    return mask_;
  }

  /// The value of cell n, or nullptr if it is off.
  [[nodiscard]] DataT* find(uint32_t n) {
    return mask_.isOn(n) ? &values_[mask_.rank(n)] : nullptr;
  }

  [[nodiscard]] const DataT* find(uint32_t n) const {
    return mask_.isOn(n) ? &values_[mask_.rank(n)] : nullptr;
  }

  /**
   * @brief Turns cell n on with value, if it was off, which moves the values of the others.
   *
   * @return  the cell (nullptr without values), and whether it was on already, in which case
   * its value is left as it was.
   */
  std::pair<DataT*, bool> insert(uint32_t n, const DataT& value) {
    if constexpr (!HAS_VALUES) {
      return {nullptr, mask_.setOn(n)};
    } else {
      if (DataT* cell = find(n)) {
        return {cell, true};
      }
      const uint32_t count = mask_.countOn();
      const uint32_t rank = mask_.rank(n);
      std::unique_ptr<DataT[]> values(new DataT[count + 1]);
      // before the others move: value may be one of them
      values[rank] = value;
      std::move(values_.get(), values_.get() + rank, values.get());
      std::move(values_.get() + rank, values_.get() + count, values.get() + rank + 1);
      values_ = std::move(values);
      mask_.setOn(n);
      return {&values_[rank], false};
    }
  }

  /// Turns cell n on with value, whether it was on or off; returns whether it was on.
  bool insertOrAssign(uint32_t n, const DataT& value) {
    const auto [cell, was_on] = insert(n, value);
    if (was_on) {
      *cell = value;
    }
    return was_on;
  }

  /// Turns cell n off and destroys its value, moving the others; returns whether it was on.
  bool erase(uint32_t n) {
    if (!mask_.isOn(n)) {
      return false;
    }
    if constexpr (HAS_VALUES) {
      const uint32_t count = mask_.countOn();
      const uint32_t rank = mask_.rank(n);
      std::unique_ptr<DataT[]> values(count > 1 ? new DataT[count - 1] : nullptr);
      std::move(values_.get(), values_.get() + rank, values.get());
      std::move(values_.get() + rank + 1, values_.get() + count, values.get() + rank);
      values_ = std::move(values);
    }
    mask_.setOff(n);
    return true;
  }

  /// Turns every cell off.
  void clear() {
    values_.reset();
    mask_ = Mask{};
  }

  /// Calls func(n, value) for every cell n on, in order; as in a dense leaf, values are mutable.
  template <class Func>
  void forEachOn(Func&& func) const {
    if constexpr (HAS_VALUES) {
      uint32_t rank = 0;
      mask_.forEachOn([&](uint32_t n) { func(n, values_[rank++]); });
    } else {
      mask_.forEachOn([&](uint32_t n) {
        DataT none{};
        func(n, none);
      });
    }
  }

  /// The heap this leaf holds: its values, not the leaf itself.
  [[nodiscard]] size_t valuesMemUsage() const {
    return HAS_VALUES ? mask_.countOn() * sizeof(DataT) : 0;
  }

 private:
  Mask mask_;
  std::unique_ptr<DataT[]> values_;
};

/// An inner node of CompactStorage: its mask, and its leaves by value, packed in bit order.
template <typename LeafT, uint32_t LOG2DIM>
class CompactInner {
 public:
  using Mask = StaticMask<LOG2DIM>;

  [[nodiscard]] const Mask& mask() const {
    return mask_;
  }

  /// The leaf at position n, or nullptr if there is none.
  [[nodiscard]] LeafT* find(uint32_t n) {
    return mask_.isOn(n) ? &leaves_[mask_.rank(n)] : nullptr;
  }

  [[nodiscard]] const LeafT* find(uint32_t n) const {
    return mask_.isOn(n) ? &leaves_[mask_.rank(n)] : nullptr;
  }

  /// The leaf at position n, created empty if there is none, which moves the leaves after it.
  LeafT* findOrCreate(uint32_t n) {
    const uint32_t rank = mask_.rank(n);
    if (!mask_.isOn(n)) {
      // one more element, not twice as many: the leaves are most of the memory of the node
      leaves_.reserve(leaves_.size() + 1);
      leaves_.emplace(leaves_.begin() + rank);
      mask_.setOn(n);
    }
    return &leaves_[rank];
  }

  /// Removes the leaves that have no cell on.
  void eraseEmptyLeaves() {
    const auto is_empty = [](const LeafT& leaf) { return leaf.mask().isOff(); };
    forEachLeaf([&](uint32_t n, const LeafT& leaf) {
      if (is_empty(leaf)) {
        mask_.setOff(n);
      }
    });
    leaves_.erase(std::remove_if(leaves_.begin(), leaves_.end(), is_empty), leaves_.end());
    leaves_.shrink_to_fit();
  }

  /// Calls func(n, leaf) for every leaf n, in increasing order.
  template <class Func>
  void forEachLeaf(Func&& func) {
    uint32_t rank = 0;
    mask_.forEachOn([&](uint32_t n) { func(n, leaves_[rank++]); });
  }

  template <class Func>
  void forEachLeaf(Func&& func) const {
    uint32_t rank = 0;
    mask_.forEachOn([&](uint32_t n) { func(n, leaves_[rank++]); });
  }

  /// The memory of this node, its leaves and their values.
  [[nodiscard]] size_t memUsage() const {
    size_t total = sizeof(*this) + leaves_.capacity() * sizeof(LeafT);
    for (const auto& leaf : leaves_) {
      total += leaf.valuesMemUsage();
    }
    return total;
  }

 private:
  Mask mask_;
  std::vector<LeafT> leaves_;
};

}  // namespace Bonxai
