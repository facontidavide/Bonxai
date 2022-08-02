// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

#ifndef NODEMASK_HPP
#define NODEMASK_HPP

#include <array>
#include <cstdint>

#define BONXAI_USE_INTRINSICS

namespace Bonxai
{
/// @brief Returns the index of the lowest, i.e. least significant, on bit in
/// the specified 64 bit word
///
/// @warning Assumes that at least one bit is set in the word, i.e. @a v !=
/// uint32_t(0)!

static inline uint32_t FindLowestOn(uint64_t v)
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

// ----------------------------> CountOn <--------------------------------------

/// @return Number of bits that are on in the specified 64-bit word

inline uint32_t CountOn(uint64_t v)
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

// ----------------------------> Mask <--------------------------------------

/// @brief Bit-mask to encode active states and facilitate sequential iterators
/// and a fast codec for I/O compression.
class Mask
{
  uint64_t* mWords = nullptr;
  uint64_t staticWords[8];

public:
  const uint8_t LOG2DIM;
  const uint32_t SIZE;        // Number of bits in mask
  const uint32_t WORD_COUNT;  // Number of 64 bit words

  /// @brief Initialize all bits to zero.
  Mask(size_t log2dim)
    : LOG2DIM(log2dim)
    , SIZE(1U << (3 * LOG2DIM))
    , WORD_COUNT(std::max(SIZE >> 6, 1u))
  {
    mWords = (WORD_COUNT <= 8) ? staticWords : new uint64_t[WORD_COUNT];

    for (uint32_t i = 0; i < WORD_COUNT; ++i)
    {
      mWords[i] = 0;
    }
  }
  Mask(size_t log2dim, bool on)
    : LOG2DIM(log2dim)
    , SIZE(1U << (3 * LOG2DIM))
    , WORD_COUNT(std::max(SIZE >> 6, 1u))
  {
    mWords = (WORD_COUNT <= 8) ? staticWords : new uint64_t[WORD_COUNT];

    const uint64_t v = on ? ~uint64_t(0) : uint64_t(0);
    for (uint32_t i = 0; i < WORD_COUNT; ++i)
    {
      mWords[i] = v;
    }
  }

  /// @brief Copy constructor
  Mask(const Mask& other)
    : LOG2DIM(other.LOG2DIM)
    , SIZE(other.SIZE)
    , WORD_COUNT(other.WORD_COUNT)
  {
    mWords = (WORD_COUNT <= 8) ? staticWords : new uint64_t[WORD_COUNT];

    for (uint32_t i = 0; i < WORD_COUNT; ++i)
    {
      mWords[i] = other.mWords[i];
    }
  }

  Mask(Mask&& other)
    : LOG2DIM(other.LOG2DIM)
    , SIZE(other.SIZE)
    , WORD_COUNT(other.WORD_COUNT)
  {
    if (WORD_COUNT <= 8)
    {
      mWords = staticWords;
      for (uint32_t i = 0; i < WORD_COUNT; ++i)
      {
        mWords[i] = other.mWords[i];
      }
    }
    else
    {
      std::swap(mWords, other.mWords);
    }
  }

  ~Mask()
  {
    if (mWords && WORD_COUNT > 8)
    {
      delete[] mWords;
    }
  }

  /// @brief Return the memory footprint in bytes of this Mask
  size_t memUsage() const
  {
    if (WORD_COUNT > 8)
    {
      return sizeof(Mask) + sizeof(uint64_t) * WORD_COUNT;
    }
    return sizeof(Mask);
  }

  /// @brief Return the number of bits available in this Mask
  uint32_t bitCount() const
  {
    return SIZE;
  }

  /// @brief Return the number of machine words used by this Mask
  uint32_t wordCount() const
  {
    return WORD_COUNT;
  }

  uint64_t getWord(size_t n) const
  {
    return mWords[n];
  }

  void setWord(size_t n, uint64_t v)
  {
    mWords[n] = v;
  }

  uint32_t countOn() const
  {
    uint32_t sum = 0, n = WORD_COUNT;
    for (const uint64_t* w = mWords; n--; ++w)
    {
      sum += CountOn(*w);
    }
    return sum;
  }

  class Iterator
  {
  public:
    Iterator(const Mask* parent)
      : mPos(parent->SIZE)
      , mParent(parent)
    {
    }
    Iterator(uint32_t pos, const Mask* parent)
      : mPos(pos)
      , mParent(parent)
    {
    }
    Iterator& operator=(const Iterator&) = default;
    uint32_t operator*() const
    {
      return mPos;
    }
    operator bool() const
    {
      return mPos != mParent->SIZE;
    }
    Iterator& operator++()
    {
      mPos = mParent->findNextOn(mPos + 1);
      return *this;
    }

  private:
    uint32_t mPos;
    const Mask* mParent;
  };  // Member class MaskIterator

  /// @brief Return the <i>n</i>th word of the bit mask, for a word of arbitrary
  /// size.
  template <typename WordT>
  WordT getWord(int n) const
  {
    return reinterpret_cast<const WordT*>(mWords)[n];
  }

  /// @brief Assignment operator that works with openvdb::util::NodeMask
  template <typename MaskT>
  Mask& operator=(const MaskT& other)
  {
    static_assert(sizeof(Mask) == sizeof(MaskT), "Mismatching sizeof");
    static_assert(WORD_COUNT == MaskT::WORD_COUNT, "Mismatching word count");
    static_assert(LOG2DIM == MaskT::LOG2DIM, "Mismatching LOG2DIM");
    auto* src = reinterpret_cast<const uint64_t*>(&other);
    uint64_t* dst = mWords;
    for (uint32_t i = 0; i < WORD_COUNT; ++i)
    {
      *dst++ = *src++;
    }
    return *this;
  }

  bool operator==(const Mask& other) const
  {
    for (uint32_t i = 0; i < WORD_COUNT; ++i)
    {
      if (mWords[i] != other.mWords[i])
        return false;
    }
    return true;
  }

  bool operator!=(const Mask& other) const
  {
    return !((*this) == other);
  }

  Iterator beginOn() const
  {
    return Iterator(this->findFirstOn(), this);
  }

  /// @brief Return true if the given bit is set.
  bool isOn(uint32_t n) const
  {
    return 0 != (mWords[n >> 6] & (uint64_t(1) << (n & 63)));
  }

  bool isOn() const
  {
    for (uint32_t i = 0; i < WORD_COUNT; ++i)
      if (mWords[i] != ~uint64_t(0))
        return false;
    return true;
  }

  bool isOff() const
  {
    for (uint32_t i = 0; i < WORD_COUNT; ++i)
      if (mWords[i] != uint64_t(0))
        return false;
    return true;
  }

  /// @brief Set the given bit on.
  bool setOn(uint32_t n)
  {
    uint64_t& word = mWords[n >> 6];
    const uint64_t on_bit = (uint64_t(1) << (n & 63));
    bool was_on = word & on_bit;
    word |= on_bit;
    return was_on;
  }

  bool setOff(uint32_t n)
  {
    uint64_t& word = mWords[n >> 6];
    const uint64_t on_bit = (uint64_t(1) << (n & 63));
    bool was_on = word & on_bit;
    word &= ~(on_bit);
    return was_on;
  }

  void set(uint32_t n, bool On)
  {
#if 1  // switch between branchless
    auto& word = mWords[n >> 6];
    n &= 63;
    word &= ~(uint64_t(1) << n);
    word |= uint64_t(On) << n;
#else
    On ? this->setOn(n) : this->setOff(n);
#endif
  }

  /// @brief Set all bits on
  void setOn()
  {
    for (uint32_t i = 0; i < WORD_COUNT; ++i)
    {
      mWords[i] = ~uint64_t(0);
    }
  }

  /// @brief Set all bits off
  void setOff()
  {
    for (uint32_t i = 0; i < WORD_COUNT; ++i)
    {
      mWords[i] = uint64_t(0);
    }
  }

  /// @brief Set all bits off
  void set(bool on)
  {
    const uint64_t v = on ? ~uint64_t(0) : uint64_t(0);
    for (uint32_t i = 0; i < WORD_COUNT; ++i)
    {
      mWords[i] = v;
    }
  }
  /// brief Toggle the state of all bits in the mask
  void toggle()
  {
    uint32_t n = WORD_COUNT;
    for (auto* w = mWords; n--; ++w)
    {
      *w = ~*w;
    }
  }
  void toggle(uint32_t n)
  {
    mWords[n >> 6] ^= uint64_t(1) << (n & 63);
  }

private:
  uint32_t findFirstOn() const
  {
    const uint64_t* w = mWords;
    uint32_t n = 0;
    while (n < WORD_COUNT && !*w)
    {
      ++w;
      ++n;
    }
    return n == WORD_COUNT ? SIZE : (n << 6) + FindLowestOn(*w);
  }

  uint32_t findNextOn(uint32_t start) const
  {
    uint32_t n = start >> 6;  // initiate
    if (n >= WORD_COUNT)
    {
      return SIZE;  // check for out of bounds
    }
    uint32_t m = start & 63;
    uint64_t b = mWords[n];
    if (b & (uint64_t(1) << m))
    {
      return start;  // simple case: start is on
    }
    b &= ~uint64_t(0) << m;  // mask out lower bits
    while (!b && ++n < WORD_COUNT)
    {
      b = mWords[n];
    }                                                 // find next non-zero word
    return (!b ? SIZE : (n << 6) + FindLowestOn(b));  // catch last word=0
  }
};  // Mask class

}  // namespace Bonxai

#endif  // NODEMASK_HPP
