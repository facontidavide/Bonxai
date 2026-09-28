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
#include <iterator>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "grid_coord.hpp"

#ifndef BONXAI_NOINLINE
#if defined(__GNUC__) || defined(__clang__)
#define BONXAI_NOINLINE __attribute__((noinline))
#elif defined(_MSC_VER)
#define BONXAI_NOINLINE __declspec(noinline)
#else
#define BONXAI_NOINLINE
#endif
#endif

namespace Bonxai {

/**
 * @brief A hash map from CoordT to ValueT: the root map of a VoxelGrid.
 *
 * It has the interface of a std::unordered_map, reduced to what Bonxai uses, and is
 * built differently:
 *
 * - Every value is allocated on its own and never moves, whatever is inserted or erased
 *   afterwards: the accessors of a VoxelGrid cache pointers to them.
 * - The index is open addressing with linear probing over buckets of 16 bytes, holding the
 *   upper half of the hash, the position of the value and a pointer to it. A lookup goes
 *   from its bucket straight to the value, where std::unordered_map reads two other nodes
 *   first; a key that is not there is almost always rejected without leaving the buckets;
 *   and growing the index moves buckets only, never values.
 * - It is iterated in insertion order, which is also the order in which the values were
 *   allocated: a traversal walks the memory forward. Erasing a key moves the last value
 *   into its place.
 *
 * Its hash is internal and only good in its upper bits, which are the only ones it uses:
 * std::hash<CoordT> is the one to use in other containers.
 *
 * Inserting or erasing invalidates the iterators. References and pointers to a value stay
 * valid until that very value is erased.
 */
template <typename ValueT>
class CoordMap {
 public:
  using key_type = CoordT;
  using mapped_type = ValueT;
  using value_type = std::pair<const CoordT, ValueT>;
  using size_type = std::size_t;

  template <bool IS_CONST>
  class Iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = CoordMap::value_type;
    using difference_type = std::ptrdiff_t;
    using reference = std::conditional_t<IS_CONST, const value_type&, value_type&>;
    using pointer = std::conditional_t<IS_CONST, const value_type*, value_type*>;

    Iterator() = default;

    /// an iterator converts to a const_iterator
    template <bool OTHER, typename = std::enable_if_t<IS_CONST && !OTHER>>
    Iterator(const Iterator<OTHER>& other)
        : values_(other.values_),
          pos_(other.pos_),
          value_(other.value_) {}

    reference operator*() const {
      return *value_;
    }
    pointer operator->() const {
      return value_;
    }

    Iterator& operator++() {
      ++pos_;
      value_ = pos_ < values_->size() ? (*values_)[pos_] : nullptr;
      return *this;
    }
    Iterator operator++(int) {
      Iterator previous = *this;
      ++(*this);
      return previous;
    }

    // every position holds a distinct value, and end() holds none
    friend bool operator==(const Iterator& a, const Iterator& b) {
      return a.value_ == b.value_;
    }
    friend bool operator!=(const Iterator& a, const Iterator& b) {
      return a.value_ != b.value_;
    }

   private:
    friend class CoordMap;
    template <bool>
    friend class Iterator;

    // the value is carried along with its position, so that dereferencing the result of
    // find() does not read the vector of values again
    Iterator(const std::vector<value_type*>* values, size_t pos, value_type* value)
        : values_(values),
          pos_(pos),
          value_(value) {}

    const std::vector<value_type*>* values_ = nullptr;
    size_t pos_ = 0;
    value_type* value_ = nullptr;
  };

  using iterator = Iterator<false>;
  using const_iterator = Iterator<true>;

  CoordMap() = default;

  CoordMap(const CoordMap&) = delete;
  CoordMap& operator=(const CoordMap&) = delete;

  CoordMap(CoordMap&& other) noexcept {
    swap(other);
  }
  CoordMap& operator=(CoordMap&& other) noexcept {
    CoordMap(std::move(other)).swap(*this);
    return *this;
  }

  ~CoordMap() {
    clear();
  }

  void swap(CoordMap& other) noexcept {
    std::swap(values_, other.values_);
    std::swap(buckets_, other.buckets_);
    std::swap(mask_, other.mask_);
    std::swap(shift_, other.shift_);
    std::swap(bits_, other.bits_);
  }

  [[nodiscard]] iterator begin() {
    return iteratorAt(0);
  }
  [[nodiscard]] iterator end() {
    return {&values_, values_.size(), nullptr};
  }
  [[nodiscard]] const_iterator begin() const {
    return const_cast<CoordMap*>(this)->begin();
  }
  [[nodiscard]] const_iterator end() const {
    return const_cast<CoordMap*>(this)->end();
  }
  [[nodiscard]] const_iterator cbegin() const {
    return begin();
  }
  [[nodiscard]] const_iterator cend() const {
    return end();
  }

  [[nodiscard]] size_type size() const {
    return values_.size();
  }
  [[nodiscard]] bool empty() const {
    return values_.empty();
  }

  [[nodiscard]] iterator find(const CoordT& key) {
    const Bucket& bucket = buckets_[findBucket(key, tagOf(key))];
    return {&values_, bucket.pos, bucket.value};
  }
  [[nodiscard]] const_iterator find(const CoordT& key) const {
    return const_cast<CoordMap*>(this)->find(key);
  }

  /// Inserts {key, ValueT(args...)} if the key is missing, and does nothing otherwise.
  template <typename... Args>
  std::pair<iterator, bool> try_emplace(const CoordT& key, Args&&... args) {
    const uint32_t tag = tagOf(key);
    const Bucket& bucket = buckets_[findBucket(key, tag)];
    if (bucket.value) {
      return {iterator(&values_, bucket.pos, bucket.value), false};
    }
    return {emplaceNew(key, tag, std::forward<Args>(args)...), true};
  }

  std::pair<iterator, bool> insert(value_type&& value) {
    return try_emplace(value.first, std::move(value.second));
  }

  size_type erase(const CoordT& key) {
    size_t hole = findBucket(key, tagOf(key));
    value_type* erased = buckets_[hole].value;
    if (!erased) {
      return 0;
    }
    const uint32_t pos = buckets_[hole].pos;

    // Backward shift deletion: pull back the buckets that follow in the same cluster,
    // so that the hole does not cut the probe sequence of any of them.
    const size_t mask = mask_;
    for (size_t i = (hole + 1) & mask; buckets_[i].value; i = (i + 1) & mask) {
      const size_t home = homeOf(buckets_[i].tag);
      // bucket i can move to the hole, unless its home is in (hole, i]
      if (((i - home) & mask) >= ((i - hole) & mask)) {
        buckets_[hole] = buckets_[i];
        hole = i;
      }
    }
    buckets_[hole] = Bucket{};

    // fill the gap in the vector with the last value, and update its bucket
    value_type* last = values_.back();
    if (last != erased) {
      values_[pos] = last;
      for (size_t i = homeOf(tagOf(last->first));; i = (i + 1) & mask) {
        if (buckets_[i].value == last) {
          buckets_[i].pos = pos;
          break;
        }
      }
    }
    values_.pop_back();
    delete erased;
    return 1;
  }

  /// Destroys all the values, and releases all the memory.
  void clear() {
    // Last to first. Front to back frees the memory in increasing addresses: glibc keeps
    // the first few blocks of each size in its thread cache and merges all the others into
    // the top of the heap, which then goes back to the kernel, to be faulted in again by
    // the next grid. Rebuilding a small grid in a loop took 3.5 times as long.
    // (ankerl::unordered_dense's segmented_map had the same issue, fixed in 5.1.0.)
    for (auto it = values_.rbegin(); it != values_.rend(); ++it) {
      delete *it;
    }
    // not `values_ = {}`: assigning an initializer list keeps the capacity
    std::vector<value_type*>().swap(values_);
    setBuckets(emptyBuckets(), 0);
  }

  void reserve(size_type count) {
    uint32_t bits = std::max(bits_, kMinBits);
    while (bits <= kMaxBits && (size_t(1) << bits) / 2 < count) {
      ++bits;
    }
    if (bits != bits_) {
      rehash(bits);
    }
    values_.reserve(count);
  }

  /// Bytes used by the index: the buckets, and the vector of pointers to the values.
  /// The values themselves are not counted.
  [[nodiscard]] size_t memUsage() const {
    const size_t buckets = bits_ == 0 ? 0 : mask_ + 1;
    return buckets * sizeof(Bucket) + values_.capacity() * sizeof(value_type*);
  }

 private:
  struct Bucket {
    uint32_t tag = 0;             // the upper 32 bits of the hash
    uint32_t pos = 0;             // position of the value in values_
    value_type* value = nullptr;  // nullptr: the bucket is empty
  };

  static constexpr uint32_t kMinBits = 4;
  // positions are 32 bits, and a size_t of 32 bits cannot be shifted by 32
  static constexpr uint32_t kMaxBits =
      std::min<uint32_t>(32, std::numeric_limits<size_t>::digits - 1);

  std::vector<value_type*> values_;  // in insertion order
  // Until the first insertion, the map points at a single, static, empty bucket, rather
  // than at nullptr: a lookup then needs no special case. Lookups are latency bound, and
  // every instruction added to their path lets the CPU overlap fewer of them.
  // bits_ == 0 tells that bucket apart; its address is never compared, since a copy of
  // the static may exist in each shared library.
  Bucket* buckets_ = emptyBuckets();
  size_t mask_ = 0;      // number of buckets - 1
  uint32_t shift_ = 32;  // the home bucket of a tag is tag >> shift_
  uint32_t bits_ = 0;    // log2 of the number of buckets, 0 before the first insertion

  static Bucket* emptyBuckets() {
    static Bucket empty[1];
    return empty;
  }

  /// One multiplication of the packed coordinates: its upper bits, the only ones used
  /// here, depend on every bit of x, y and z.
  static uint32_t tagOf(const CoordT& key) {
    const uint64_t packed = (uint64_t(uint32_t(key.x)) | (uint64_t(uint32_t(key.y)) << 32)) ^
                            (uint64_t(uint32_t(key.z)) * UINT64_C(0x9e3779b97f4a7c15));
    return uint32_t((packed * UINT64_C(0xd6e8feb86659fd93)) >> 32);
  }

  /// at most half full: a hit reads 1.5 buckets on average, a miss 2.5
  [[nodiscard]] size_t maxLoad() const {
    return bits_ == 0 ? 0 : (mask_ + 1) / 2;
  }
  [[nodiscard]] size_t homeOf(uint32_t tag) const {
    // 64 bits, so that the shift by 32 of the empty map is defined
    return size_t(uint64_t(tag) >> shift_);
  }

  /// the bucket holding the key, or the empty bucket where its probe sequence ends
  [[nodiscard]] size_t findBucket(const CoordT& key, uint32_t tag) const {
    for (size_t i = homeOf(tag);; i = (i + 1) & mask_) {
      const Bucket& bucket = buckets_[i];
      if (!bucket.value || (bucket.tag == tag && bucket.value->first == key)) {
        return i;
      }
    }
  }

  void setBuckets(Bucket* buckets, uint32_t bits) {
    if (bits_ != 0) {
      delete[] buckets_;
    }
    buckets_ = buckets;
    bits_ = bits;
    mask_ = (size_t(1) << bits) - 1;
    shift_ = 32 - bits;
  }

  // The cold path of try_emplace, and never inlined into it. Inlined, the address of the
  // key escapes into the construction of the pair, and the compiler keeps the caller's
  // key in memory, written as three 32 bits stores. VoxelGrid's accessors then copy that
  // key into their cache with a 64 bits load, which cannot be forwarded from those stores
  // and has to wait for them to retire: lookups that should overlap run one after the
  // other, and updating a large grid got 2.5 times slower. Taken by value, the key reaches
  // this function in registers.
  template <typename... Args>
  BONXAI_NOINLINE iterator emplaceNew(CoordT key, uint32_t tag, Args&&... args) {
    // make room first: from here on, nothing can throw once the value exists
    if (values_.size() + 1 > maxLoad()) {
      rehash(std::max(bits_ + 1, kMinBits));
    }
    if (values_.size() == values_.capacity()) {
      values_.reserve(values_.empty() ? size_t(1) << kMinBits : 2 * values_.capacity());
    }
    auto* value = new value_type(
        std::piecewise_construct, std::forward_as_tuple(key),
        std::forward_as_tuple(std::forward<Args>(args)...));
    const uint32_t pos = uint32_t(values_.size());
    values_.push_back(value);
    buckets_[findBucket(key, tag)] = {tag, pos, value};
    return iterator(&values_, pos, value);
  }

  void rehash(uint32_t bits) {
    if (bits > kMaxBits) {
      throw std::length_error("CoordMap: too many elements");
    }
    Bucket* buckets = new Bucket[size_t(1) << bits];
    const size_t mask = (size_t(1) << bits) - 1;
    for (size_t i = 0; i <= mask_; ++i) {
      if (buckets_[i].value) {
        size_t b = buckets_[i].tag >> (32 - bits);
        while (buckets[b].value) {
          b = (b + 1) & mask;
        }
        buckets[b] = buckets_[i];
      }
    }
    setBuckets(buckets, bits);
  }

  [[nodiscard]] iterator iteratorAt(size_t pos) {
    return {&values_, pos, pos < values_.size() ? values_[pos] : nullptr};
  }
};

}  // namespace Bonxai
