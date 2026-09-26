#include "bonxai/coord_map.hpp"

#include <gtest/gtest.h>

#include <map>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

using Bonxai::CoordMap;
using Bonxai::CoordT;

namespace {

// counts the live instances, and cannot be copied: like an InnerGrid
struct Tracked {
  static int alive;
  int value = 0;

  explicit Tracked(int v)
      : value(v) {
    ++alive;
  }
  Tracked(Tracked&& other) noexcept
      : value(other.value) {
    ++alive;
  }
  Tracked(const Tracked&) = delete;
  Tracked& operator=(const Tracked&) = delete;
  ~Tracked() {
    --alive;
  }
};
int Tracked::alive = 0;

using Key = std::tuple<int32_t, int32_t, int32_t>;

Key toKey(const CoordT& c) {
  return {c.x, c.y, c.z};
}

// the map holds exactly the reference: same size, every key found with its value,
// and an iteration that visits each key once
void expectSameContent(const CoordMap<Tracked>& map, const std::map<Key, int>& reference) {
  ASSERT_EQ(map.size(), reference.size());
  ASSERT_EQ(map.empty(), reference.empty());
  for (const auto& [key, value] : reference) {
    const auto [x, y, z] = key;
    auto it = map.find({x, y, z});
    ASSERT_NE(it, map.end()) << "missing " << x << " " << y << " " << z;
    EXPECT_EQ(it->second.value, value);
  }
  std::map<Key, int> visited;
  for (const auto& [coord, tracked] : map) {
    EXPECT_TRUE(visited.emplace(toKey(coord), tracked.value).second) << "visited twice";
  }
  EXPECT_EQ(visited, reference);
}

}  // namespace

TEST(CoordMap, EmptyMap) {
  CoordMap<Tracked> map;
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 0u);
  EXPECT_EQ(map.begin(), map.end());
  EXPECT_EQ(map.find({1, 2, 3}), map.end());
  EXPECT_EQ(map.erase({1, 2, 3}), 0u);
  map.clear();
  EXPECT_TRUE(map.empty());
}

TEST(CoordMap, TryEmplaceInsertsOnlyMissingKeys) {
  Tracked::alive = 0;
  {
    CoordMap<Tracked> map;
    auto [it, inserted] = map.try_emplace({1, -2, 3}, 42);
    EXPECT_TRUE(inserted);
    EXPECT_EQ(it->first, (CoordT{1, -2, 3}));
    EXPECT_EQ(it->second.value, 42);

    auto [again, inserted_again] = map.try_emplace({1, -2, 3}, 7);
    EXPECT_FALSE(inserted_again);
    EXPECT_EQ(again, it);
    EXPECT_EQ(again->second.value, 42) << "an existing value must not be replaced";
    EXPECT_EQ(Tracked::alive, 1) << "no value must be built for a key that exists";

    auto [third, inserted_third] = map.insert({CoordT{0, 0, 0}, Tracked(5)});
    EXPECT_TRUE(inserted_third);
    EXPECT_EQ(third->second.value, 5);
    EXPECT_EQ(map.size(), 2u);
  }
  EXPECT_EQ(Tracked::alive, 0) << "the destructor must destroy every value";
}

TEST(CoordMap, IteratesInInsertionOrder) {
  CoordMap<Tracked> map;
  std::vector<CoordT> keys;
  for (int i = 0; i < 1000; ++i) {
    keys.push_back({i * 32, -i * 64, (i % 7) * 32});
    map.try_emplace(keys.back(), i);
  }
  size_t i = 0;
  for (const auto& [coord, tracked] : map) {
    ASSERT_LT(i, keys.size());
    EXPECT_EQ(coord, keys[i]);
    EXPECT_EQ(tracked.value, int(i));
    ++i;
  }
  EXPECT_EQ(i, keys.size());
}

// VoxelGrid's accessors cache pointers to the values: neither growing the map nor
// erasing other keys may move them
TEST(CoordMap, ValuesNeverMove) {
  CoordMap<Tracked> map;
  std::vector<std::pair<CoordT, const Tracked*>> kept;
  for (int i = 0; i < 20000; ++i) {
    const CoordT key{(i % 50 - 25) * 32, (i / 50 % 20 - 10) * 32, (i / 1000 - 10) * 32};
    auto* value = &map.try_emplace(key, i).first->second;
    if (i % 3 == 0) {
      kept.emplace_back(key, value);
    }
  }
  // erase everything else
  for (int i = 0; i < 20000; ++i) {
    if (i % 3 != 0) {
      const CoordT key{(i % 50 - 25) * 32, (i / 50 % 20 - 10) * 32, (i / 1000 - 10) * 32};
      EXPECT_EQ(map.erase(key), 1u);
    }
  }
  map.reserve(100000);
  ASSERT_EQ(map.size(), kept.size());
  for (const auto& [key, pointer] : kept) {
    auto it = map.find(key);
    ASSERT_NE(it, map.end());
    EXPECT_EQ(&it->second, pointer) << "a value moved";
  }
}

// random operations against std::map, on keys packed in a small volume that crosses
// zero: long clusters in the index, and a lot of backward shifting on erase
TEST(CoordMap, RandomOperationsMatchStdMap) {
  Tracked::alive = 0;
  std::mt19937 rng(42);
  std::uniform_int_distribution<int32_t> coord(-12, 12);
  std::uniform_int_distribution<int> operation(0, 9);
  {
    CoordMap<Tracked> map;
    std::map<Key, int> reference;
    for (int step = 0; step < 200000; ++step) {
      const CoordT key{coord(rng) * 32, coord(rng) * 32, coord(rng) * 32};
      const int op = operation(rng);
      if (op < 5) {
        const bool inserted = map.try_emplace(key, step).second;
        EXPECT_EQ(inserted, reference.emplace(toKey(key), step).second);
      } else if (op < 8) {
        EXPECT_EQ(map.erase(key), reference.erase(toKey(key)));
      } else {
        auto it = map.find(key);
        auto ref = reference.find(toKey(key));
        ASSERT_EQ(it == map.end(), ref == reference.end());
        if (ref != reference.end()) {
          EXPECT_EQ(it->second.value, ref->second);
        }
      }
      if (step % 20000 == 0) {
        expectSameContent(map, reference);
      }
    }
    expectSameContent(map, reference);
    EXPECT_EQ(Tracked::alive, int(reference.size()));

    // erase all, in random order
    std::vector<Key> keys;
    for (const auto& [key, value] : reference) {
      keys.push_back(key);
    }
    std::shuffle(keys.begin(), keys.end(), rng);
    for (const auto& [x, y, z] : keys) {
      ASSERT_EQ(map.erase({x, y, z}), 1u);
    }
    EXPECT_TRUE(map.empty());
    EXPECT_EQ(map.begin(), map.end());
    EXPECT_EQ(Tracked::alive, 0);
  }
  EXPECT_EQ(Tracked::alive, 0);
}

TEST(CoordMap, ClearReleasesEverythingAndTheMapIsReusable) {
  Tracked::alive = 0;
  CoordMap<Tracked> map;
  for (int i = 0; i < 5000; ++i) {
    map.try_emplace({i, -i, i * 3}, i);
  }
  EXPECT_GT(map.memUsage(), 0u);
  map.clear();
  EXPECT_EQ(Tracked::alive, 0);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.memUsage(), 0u);
  EXPECT_EQ(map.find({1, -1, 3}), map.end());

  map.try_emplace({1, -1, 3}, 1);
  ASSERT_NE(map.find({1, -1, 3}), map.end());
  EXPECT_EQ(map.size(), 1u);
}

// Last to first: front to back frees the memory in increasing addresses, and glibc then
// hands the top of the heap back to the kernel every time a grid is destroyed
TEST(CoordMap, ClearDestroysTheValuesLastToFirst) {
  struct Logged {
    std::vector<int>* log;
    int id;
    Logged(std::vector<int>* l, int i)
        : log(l),
          id(i) {}
    ~Logged() {
      log->push_back(id);
    }
  };
  std::vector<int> destroyed;
  {
    CoordMap<Logged> map;
    for (int i = 0; i < 100; ++i) {
      map.try_emplace({i, 0, 0}, &destroyed, i);
    }
  }
  ASSERT_EQ(destroyed.size(), 100u);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(destroyed[i], 99 - i);
  }
}

TEST(CoordMap, MoveLeavesAnEmptyUsableMap) {
  Tracked::alive = 0;
  CoordMap<Tracked> source;
  for (int i = 0; i < 100; ++i) {
    source.try_emplace({i, i, i}, i);
  }
  const Tracked* pointer = &source.find({7, 7, 7})->second;

  CoordMap<Tracked> moved(std::move(source));
  EXPECT_EQ(moved.size(), 100u);
  EXPECT_EQ(&moved.find({7, 7, 7})->second, pointer) << "moving the map moves no value";
  EXPECT_TRUE(source.empty());
  source.try_emplace({1, 2, 3}, 0);
  EXPECT_EQ(source.size(), 1u);

  CoordMap<Tracked> assigned;
  assigned.try_emplace({-9, 9, 9}, 9);
  assigned = std::move(moved);
  EXPECT_EQ(assigned.size(), 100u);
  EXPECT_EQ(assigned.find({-9, 9, 9}), assigned.end()) << "the previous content must be gone";
  EXPECT_EQ(Tracked::alive, 101);
}

TEST(CoordMap, ConstAccess) {
  CoordMap<Tracked> map;
  map.try_emplace({-1, -1, -1}, 3);
  const auto& const_map = map;
  auto it = const_map.find({-1, -1, -1});
  ASSERT_NE(it, const_map.end());
  EXPECT_EQ(it->second.value, 3);
  CoordMap<Tracked>::const_iterator from_mutable = map.begin();
  EXPECT_EQ(from_mutable, const_map.cbegin());
  int sum = 0;
  for (const auto& [coord, tracked] : const_map) {
    sum += tracked.value + coord.x;
  }
  EXPECT_EQ(sum, 2);
}
