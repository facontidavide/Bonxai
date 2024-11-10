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
#include <utility>

namespace Bonxai {

class Mask {
  uint64_t* words_ = nullptr;
  // small object optimization, that will be used when
  // SIZE <= 512 bits, i.e LOG2DIM <= 3
  uint64_t static_words_[8];

 public:
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
  uint32_t bitCount() const {
    return SIZE;
  }

  /// Return the number of machine words used by this Mask
  uint32_t wordCount() const {
    return WORD_COUNT;
  }

  uint64_t getWord(size_t n) const {
    return words_[n];
  }

  void setWord(size_t n, uint64_t v) {
    words_[n] = v;
  }

  uint32_t countOn() const;

  class Iterator {
   public:
    Iterator(const Mask* parent)
        : pos_(parent->SIZE),
          parent_(parent) {}
    Iterator(uint32_t pos, const Mask* parent)
        : pos_(pos),
          parent_(parent) {}
    Iterator& operator=(const Iterator&) = default;

    uint32_t operator*() const {
      return pos_;
    }

    operator bool() const {
      return pos_ != parent_->SIZE;
    }

    Iterator& operator++() {
      pos_ = parent_->findNextOn(pos_ + 1);
      return *this;
    }

   private:
    uint32_t pos_;
    const Mask* parent_;
  };

  bool operator==(const Mask& other) const;

  bool operator!=(const Mask& other) const {
    return !((*this) == other);
  }

  Iterator beginOn() const {
    return Iterator(this->findFirstOn(), this);
  }

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

  uint32_t findFirstOn() const;

  uint32_t size() const {
    return SIZE;
  }

 private:
  uint32_t findNextOn(uint32_t start) const;

  static uint32_t FindLowestOn(uint64_t v);
  static uint32_t CountOn(uint64_t v);

  // Number of bits in mask
  uint32_t SIZE;
  // Number of 64 bit words
  uint32_t WORD_COUNT;
};

//----------------------------------------------------
//----------------- Implementations ------------------
//----------------------------------------------------

#define BONXAI_USE_INTRINSICS

/// Returns the index of the lowest, i.e. least significant, on bit in
/// the specified 64 bit word
///
/// @warning Assumes that at least one bit is set in the word, i.e. @a v !=
/// uint32_t(0)!

inline uint32_t Mask::FindLowestOn(uint64_t v) {
#if defined(_MSC_VER) && defined(BONXAI_USE_INTRINSICS)
  unsigned long index;
  _BitScanForward64(&index, v);
  return static_cast<uint32_t>(index);
#elif (defined(__GNUC__) || defined(__clang__)) && defined(BONXAI_USE_INTRINSICS)
  return static_cast<uint32_t>(__builtin_ctzll(v));
#else
  static const unsigned char DeBruijn[64] = {
      0,  1,  2,  53, 3,  7,  54, 27, 4,  38, 41, 8,  34, 55, 48, 28, 62, 5,  39, 46, 44, 42,
      22, 9,  24, 35, 59, 56, 49, 18, 29, 11, 63, 52, 6,  26, 37, 40, 33, 47, 61, 45, 43, 21,
      23, 58, 17, 10, 51, 25, 36, 32, 60, 20, 57, 16, 50, 31, 19, 15, 30, 14, 13, 12,
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

inline uint32_t Mask::CountOn(uint64_t v) {
#if defined(_MSC_VER) && defined(_M_X64)
  v = __popcnt64(v);
#elif (defined(__GNUC__) || defined(__clang__))
  v = __builtin_popcountll(v);
#else
  // Software Implementation
  /// @warning evil bit twiddling ahead!
  v = v - ((v >> 1) & uint64_t(0x5555555555555555));
  v = (v & uint64_t(0x3333333333333333)) + ((v >> 2) & uint64_t(0x3333333333333333));
  v = (((v + (v >> 4)) & uint64_t(0xF0F0F0F0F0F0F0F)) * uint64_t(0x101010101010101)) >> 56;
#endif
  return static_cast<uint32_t>(v);
}

inline void Mask::setOn() {
  for (uint32_t i = 0; i < WORD_COUNT; ++i) {
    words_[i] = ~uint64_t(0);
  }
}

inline void Mask::setOff() {
  for (uint32_t i = 0; i < WORD_COUNT; ++i) {
    words_[i] = uint64_t(0);
  }
}

inline void Mask::set(bool on) {
  const uint64_t v = on ? ~uint64_t(0) : uint64_t(0);
  for (uint32_t i = 0; i < WORD_COUNT; ++i) {
    words_[i] = v;
  }
}

inline void Mask::toggle() {
  uint32_t n = WORD_COUNT;
  for (auto* w = words_; n--; ++w) {
    *w = ~*w;
  }
}

inline void Mask::toggle(uint32_t n) {
  words_[n >> 6] ^= uint64_t(1) << (n & 63);
}

inline uint32_t Mask::findFirstOn() const {
  const uint64_t* w = words_;
  uint32_t n = 0;
  while (n < WORD_COUNT && !*w) {
    ++w;
    ++n;
  }
  return n == WORD_COUNT ? SIZE : (n << 6) + FindLowestOn(*w);
}

inline uint32_t Mask::findNextOn(uint32_t start) const {
  uint32_t n = start >> 6;  // initiate
  if (n >= WORD_COUNT) {
    return SIZE;  // check for out of bounds
  }
  uint32_t m = start & 63;
  uint64_t b = words_[n];
  if (b & (uint64_t(1) << m)) {
    return start;  // simple case: start is on
  }
  b &= ~uint64_t(0) << m;  // mask out lower bits
  while (!b && ++n < WORD_COUNT) {
    b = words_[n];
  }                                                 // find next non-zero word
  return (!b ? SIZE : (n << 6) + FindLowestOn(b));  // catch last word=0
}

inline Mask::Mask(size_t log2dim)
    : SIZE(1U << (3 * log2dim)),
      WORD_COUNT(std::max(SIZE >> 6, 1u)) {
  words_ = (WORD_COUNT <= 8) ? static_words_ : new uint64_t[WORD_COUNT];

  for (uint32_t i = 0; i < WORD_COUNT; ++i) {
    words_[i] = 0;
  }
}

inline Mask::Mask(size_t log2dim, bool on)
    : SIZE(1U << (3 * log2dim)),
      WORD_COUNT(std::max(SIZE >> 6, 1u)) {
  words_ = (WORD_COUNT <= 8) ? static_words_ : new uint64_t[WORD_COUNT];

  const uint64_t v = on ? ~uint64_t(0) : uint64_t(0);
  for (uint32_t i = 0; i < WORD_COUNT; ++i) {
    words_[i] = v;
  }
}

inline Mask::Mask(const Mask& other)
    : SIZE(other.SIZE),
      WORD_COUNT(other.WORD_COUNT) {
  words_ = (WORD_COUNT <= 8) ? static_words_ : new uint64_t[WORD_COUNT];

  for (uint32_t i = 0; i < WORD_COUNT; ++i) {
    words_[i] = other.words_[i];
  }
}

inline Mask::Mask(Mask&& other)
    : SIZE(other.SIZE),
      WORD_COUNT(other.WORD_COUNT) {
  if (WORD_COUNT <= 8) {
    words_ = static_words_;
    for (uint32_t i = 0; i < WORD_COUNT; ++i) {
      words_[i] = other.words_[i];
    }
  } else {
    std::swap(words_, other.words_);
  }
}

inline Mask::~Mask() {
  if (WORD_COUNT > 8) {
    delete[] words_;
  }
}

inline size_t Mask::memUsage() const {
  if (WORD_COUNT > 8) {
    return sizeof(Mask) + sizeof(uint64_t) * WORD_COUNT;
  }
  return sizeof(Mask);
}

inline uint32_t Mask::countOn() const {
  uint32_t sum = 0, n = WORD_COUNT;
  for (const uint64_t* w = words_; n--; ++w) {
    sum += CountOn(*w);
  }
  return sum;
}

inline bool Mask::operator==(const Mask& other) const {
  for (uint32_t i = 0; i < WORD_COUNT; ++i) {
    if (words_[i] != other.words_[i]) {
      return false;
    }
  }
  return true;
}

inline bool Mask::isOn(uint32_t n) const {
  return 0 != (words_[n >> 6] & (uint64_t(1) << (n & 63)));
}

inline bool Mask::isOn() const {
  for (uint32_t i = 0; i < WORD_COUNT; ++i) {
    if (words_[i] != ~uint64_t(0)) {
      return false;
    }
  }
  return true;
}

inline bool Mask::isOff() const {
  for (uint32_t i = 0; i < WORD_COUNT; ++i) {
    if (words_[i] != uint64_t(0)) {
      return false;
    }
  }
  return true;
}

inline bool Mask::setOn(uint32_t n) {
  uint64_t& word = words_[n >> 6];
  const uint64_t on_bit = (uint64_t(1) << (n & 63));
  bool was_on = word & on_bit;
  word |= on_bit;
  return was_on;
}

inline bool Mask::setOff(uint32_t n) {
  uint64_t& word = words_[n >> 6];
  const uint64_t on_bit = (uint64_t(1) << (n & 63));
  bool was_on = word & on_bit;
  word &= ~(on_bit);
  return was_on;
}

inline void Mask::set(uint32_t n, bool On) {
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
