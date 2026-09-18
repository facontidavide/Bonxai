#include "bonxai/mask.hpp"

#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "bonxai/bonxai.hpp"

namespace {

// Reference: the ON positions, found by testing every bit one at a time.
std::vector<uint32_t> BitsOnByScan(const Bonxai::Mask& mask) {
  std::vector<uint32_t> out;
  for (uint32_t i = 0; i < mask.size(); ++i) {
    if (mask.isOn(i)) {
      out.push_back(i);
    }
  }
  return out;
}

std::vector<uint32_t> BitsOnByIterator(const Bonxai::Mask& mask) {
  std::vector<uint32_t> out;
  for (auto it = mask.beginOn(); it; ++it) {
    out.push_back(*it);
  }
  return out;
}

void ExpectIteratorMatchesScan(const Bonxai::Mask& mask) {
  const auto expected = BitsOnByScan(mask);
  EXPECT_EQ(BitsOnByIterator(mask), expected);
  EXPECT_EQ(mask.countOn(), expected.size());
  EXPECT_EQ(mask.findFirstOn(), expected.empty() ? mask.size() : expected.front());
}

}  // namespace

TEST(Mask, EmptyIteratesNothing) {
  for (size_t log2dim : {1, 2, 3, 4}) {
    Bonxai::Mask mask(log2dim);
    EXPECT_TRUE(mask.isOff());
    EXPECT_EQ(mask.findFirstOn(), mask.size());
    EXPECT_TRUE(BitsOnByIterator(mask).empty());
    ExpectIteratorMatchesScan(mask);
  }
}

TEST(Mask, FullIteratesEveryBit) {
  for (size_t log2dim : {1, 2, 3, 4}) {
    Bonxai::Mask mask(log2dim, true);
    EXPECT_TRUE(mask.isOn());
    EXPECT_EQ(mask.countOn(), mask.size());
    ExpectIteratorMatchesScan(mask);
  }
}

// The iterator advances word by word, so the ends of a word and the ends of the
// mask are where an off-by-one would hide.
TEST(Mask, SingleBitAtEveryBoundary) {
  for (size_t log2dim : {2, 3, 4}) {
    Bonxai::Mask reference(log2dim);
    const uint32_t size = reference.size();
    std::vector<uint32_t> positions{0, 1, 62, 63, 64, 65, size - 1};
    if (size > 128) {
      positions.insert(positions.end(), {126, 127, 128, 129, size - 65, size - 64, size - 2});
    }
    for (uint32_t pos : positions) {
      if (pos >= size) {
        continue;
      }
      Bonxai::Mask mask(log2dim);
      EXPECT_FALSE(mask.setOn(pos));
      EXPECT_TRUE(mask.isOn(pos));
      const std::vector<uint32_t> expected{pos};
      EXPECT_EQ(BitsOnByIterator(mask), expected) << "log2dim=" << log2dim << " pos=" << pos;
      ExpectIteratorMatchesScan(mask);
    }
  }
}

TEST(Mask, AdjacentAndSparsePatterns) {
  Bonxai::Mask mask(4);  // 4096 bits, 64 words
  for (uint32_t pos : {0u, 1u, 2u, 63u, 64u, 65u, 127u, 128u, 1000u, 4094u, 4095u}) {
    mask.setOn(pos);
  }
  ExpectIteratorMatchesScan(mask);

  mask.setOff(64);
  mask.setOff(0);
  ExpectIteratorMatchesScan(mask);
}

TEST(Mask, RandomPatternsMatchScan) {
  std::mt19937 rng(1234);
  for (size_t log2dim : {1, 2, 3, 4}) {
    for (int trial = 0; trial < 50; ++trial) {
      Bonxai::Mask mask(log2dim);
      std::bernoulli_distribution on(trial / 50.0);
      for (uint32_t i = 0; i < mask.size(); ++i) {
        if (on(rng)) {
          mask.setOn(i);
        }
      }
      ExpectIteratorMatchesScan(mask);
    }
  }
}

// beginOn() builds the iterator from a position; a mid-mask start must yield
// exactly the ON bits from there on.
TEST(Mask, IteratorConstructedAtAPosition) {
  Bonxai::Mask mask(3);  // 512 bits
  for (uint32_t pos : {5u, 63u, 64u, 200u, 511u}) {
    mask.setOn(pos);
  }
  const auto all = BitsOnByScan(mask);

  for (uint32_t start = 0; start <= mask.size(); ++start) {
    std::vector<uint32_t> expected;
    for (uint32_t pos : all) {
      if (pos >= start) {
        expected.push_back(pos);
      }
    }
    std::vector<uint32_t> got;
    Bonxai::Mask::Iterator it(start, &mask);
    // a position handed in is reported as-is, matching the previous behaviour
    if (it && mask.isOn(start)) {
      got.push_back(*it);
    }
    for (++it; it; ++it) {
      got.push_back(*it);
    }
    EXPECT_EQ(got, expected) << "start=" << start;
  }
}

TEST(Mask, SetOnAndSetOffReportThePreviousState) {
  Bonxai::Mask mask(2);
  EXPECT_FALSE(mask.setOn(7));
  EXPECT_TRUE(mask.setOn(7));
  EXPECT_TRUE(mask.setOff(7));
  EXPECT_FALSE(mask.setOff(7));
  EXPECT_TRUE(mask.isOff());
}

TEST(Mask, CopyAndMovePreserveContent) {
  for (size_t log2dim : {2, 4}) {  // inline storage, then heap storage
    Bonxai::Mask original(log2dim);
    for (uint32_t i = 0; i < original.size(); i += 7) {
      original.setOn(i);
    }
    const auto expected = BitsOnByScan(original);

    Bonxai::Mask copy(original);
    EXPECT_EQ(BitsOnByIterator(copy), expected);
    EXPECT_TRUE(copy == original);

    Bonxai::Mask moved(std::move(copy));
    EXPECT_EQ(BitsOnByIterator(moved), expected);
    EXPECT_TRUE(moved == original);
  }
}

// log2dim == 1 gives 8 bits inside a 64 bit word; the padding must stay off.
TEST(Mask, PaddingBitsStayOff) {
  Bonxai::Mask mask(1);
  EXPECT_EQ(mask.size(), 8u);

  mask.setOn();
  EXPECT_EQ(mask.countOn(), 8u);
  EXPECT_TRUE(mask.isOn());
  EXPECT_EQ(BitsOnByIterator(mask), (std::vector<uint32_t>{0, 1, 2, 3, 4, 5, 6, 7}));

  mask.toggle();
  EXPECT_EQ(mask.countOn(), 0u);
  EXPECT_TRUE(mask.isOff());

  EXPECT_EQ(Bonxai::Mask(1, true).countOn(), 8u);
}

// Mask declares a move constructor, which suppresses the implicit assignment
// operators. Without an explicit one, Grid<T>::operator=(Grid&&) fails to compile as
// soon as it is instantiated, which is what any container storing the InnerGrid by
// value ends up doing when it erases an element.
TEST(MaskTest, MoveAssignmentStaticWords) {
  Bonxai::Mask source(3);  // 512 bits: stored inside the object
  source.setOn(5);
  source.setOn(500);

  Bonxai::Mask destination(3);
  destination.setOn(1);
  destination = std::move(source);

  EXPECT_FALSE(destination.isOn(1));
  EXPECT_TRUE(destination.isOn(5));
  EXPECT_TRUE(destination.isOn(500));
  EXPECT_EQ(destination.countOn(), 2u);
}

TEST(MaskTest, MoveAssignmentHeapWords) {
  Bonxai::Mask source(4);  // 4096 bits: allocated on the heap
  source.setOn(7);
  source.setOn(4000);

  Bonxai::Mask destination(4);
  destination.setOn(1);
  destination = std::move(source);

  EXPECT_FALSE(destination.isOn(1));
  EXPECT_TRUE(destination.isOn(7));
  EXPECT_TRUE(destination.isOn(4000));
  EXPECT_EQ(destination.countOn(), 2u);
}

TEST(MaskTest, GridMoveAssignment) {
  Bonxai::Grid<int> source(3);
  source.mask().setOn(9);
  source.cell(9) = 42;

  Bonxai::Grid<int> destination(3);
  destination = std::move(source);

  EXPECT_TRUE(destination.mask().isOn(9));
  EXPECT_EQ(destination.cell(9), 42);
}
