// Root map policies: every candidate behind the same small interface, so that the
// accessors of VoxelGrid (exp2/include/bonxai/bonxai.hpp) run the same code for all.
//
//   Inner*       find(const CoordT&)            nullptr when missing
//   Inner*       emplace(const CoordT&, bits)   the key is known to be missing
//   void         forEach(f) / forEachMutable(f) f(const CoordT&, Inner&)
//   void         erase(key), clear(), reserve(n)
//   size_t       size(), buckets()
//   bool         takeMoved()                    values moved since the last call
//   kMovesOnInsert                              values may move when inserting
//
// Three families:
//   Node<M, H>    M<CoordT, Inner, H>: the map never moves its values (node maps)
//   Ptr<M, H>     M<CoordT, unique_ptr<Inner>, H>: any map, values behind a pointer
//   Inline<M, H>  M<CoordT, Inner, H> for a map that moves its values when it grows:
//                 VoxelGrid then drops the caches of its accessors (a rehash bumps
//                 cache_epoch_). A different contract from std::unordered_map's.
#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include "bonxai/coord_map.hpp"
#include "bonxai/grid_coord.hpp"

#ifdef WITH_ANKERL
#include <ankerl/unordered_dense.h>
#endif
#ifdef WITH_BOOST
#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/unordered/unordered_map.hpp>
#include <boost/unordered/unordered_node_map.hpp>
#endif
#ifdef WITH_ABSL
#include <absl/container/flat_hash_map.h>
#include <absl/container/node_hash_map.h>
#include <absl/hash/hash.h>
namespace Bonxai {
template <typename H>
H AbslHashValue(H h, const CoordT& c) {
  return H::combine(std::move(h), c.x, c.y, c.z);
}
}  // namespace Bonxai
#endif
#ifdef WITH_PHMAP
#include <parallel_hashmap/phmap.h>
#endif
#ifdef WITH_TSL
#include <tsl/hopscotch_map.h>
#include <tsl/robin_map.h>
#include <tsl/sparse_map.h>
#endif
#ifdef WITH_SKA
#include <bytell_hash_map.hpp>
#include <flat_hash_map.hpp>
#endif
#ifdef WITH_EMHASH5
#include <emhash/hash_table5.hpp>
#endif
#ifdef WITH_EMHASH6
#include <emhash/hash_table6.hpp>
#endif
#ifdef WITH_EMHASH7
#include <emhash/hash_table7.hpp>
#endif
#ifdef WITH_EMHASH8
#include <emhash/hash_table8.hpp>
#endif
#ifdef WITH_ROBINHOOD
#include <robin_hood.h>
#endif
#ifdef WITH_FPH
#include <fph/dynamic_fph_table.h>
#include <fph/meta_fph_table.h>
#endif

namespace mx {
using Bonxai::CoordT;

//------------------------------------------------------------------ hashes ---
// A root key is computed field by field. When it has to go through memory (to a find()
// that is not inlined, or spilled), it is written as 32 bits stores, and a 64 bits load of
// x and y cannot be forwarded from them: the load waits for the stores to retire, and the
// lookups stop overlapping. The empty asm keeps the compiler from merging the loads of x
// and y into one; it costs nothing when the key is in registers. CoordMap needs none of
// this, being inlined; without it, some libraries were 2.7 times slower here.
#define MX_SEPARATE(v) asm("" : "+r"(v))
inline uint64_t packed(const CoordT& p) noexcept {
  uint32_t x = uint32_t(p.x), y = uint32_t(p.y);
  MX_SEPARATE(y);
  return (uint64_t(x) | (uint64_t(y) << 32)) ^
         uint64_t(uint32_t(p.z)) * UINT64_C(0x9e3779b97f4a7c15);
}
// the same for operator==, which Clang turns into a 64 bits compare of x and y
struct KeyEq {
  bool operator()(const CoordT& a, const CoordT& b) const noexcept {
    int32_t ay = a.y, by = b.y;
    MX_SEPARATE(ay);
    MX_SEPARATE(by);
    return a.x == b.x && ay == by && a.z == b.z;
  }
};
inline uint64_t fmix64(uint64_t k) noexcept {
  k ^= k >> 33;
  k *= UINT64_C(0xff51afd7ed558ccd);
  k ^= k >> 33;
  k *= UINT64_C(0xc4ceb9fe1a85ec53);
  k ^= k >> 33;
  return k;
}

// the packed coordinates, not mixed: only for the maps that mix the hash themselves
struct HPack {
  size_t operator()(const CoordT& p) const noexcept {
    return packed(p);
  }
};
// one multiplication, folded: every bit depends on every coordinate
struct HPmx {
  size_t operator()(const CoordT& p) const noexcept {
    const uint64_t h = packed(p) * UINT64_C(0xd6e8feb86659fd93);
    return h ^ (h >> 32);
  }
};
// murmur3's finalizer, as std::hash<CoordT> now
struct HFmix {
  size_t operator()(const CoordT& p) const noexcept {
    return fmix64(packed(p));
  }
};
// the same, declared as good in every bit: boost and ankerl then use them as they are
struct HPmxA : HPmx {
  using is_avalanching = void;
};
struct HFmixA : HFmix {
  using is_avalanching = void;
};
// Bonxai's std::hash<CoordT>: murmur3, not noexcept (libstdc++ then caches it in the nodes)
using HStd = std::hash<CoordT>;
// a candidate for it: one multiplication per coordinate, which no compiler can merge
// into a 64 bits load of x and y, then murmur3; not noexcept either
struct HSep {
  size_t operator()(const CoordT& p) const {
    return fmix64(
        uint64_t(uint32_t(p.x)) * UINT64_C(0x9E3779B97F4A7C15) ^
        uint64_t(uint32_t(p.y)) * UINT64_C(0xC2B2AE3D27D4EB4F) ^
        uint64_t(uint32_t(p.z)) * UINT64_C(0x165667B19E3779F9));
  }
};

//------------------------------------------------------------- the wrapper ---
#ifdef MX_NO_FORCE_INLINE
#define MX_INLINE inline
#else
#define MX_INLINE __attribute__((always_inline)) inline
#endif

// SegNode: never moved when inserting, but erase moves the last value into the hole
// Raw: the map holds an Inner*, the wrapper owns it (trivially copyable, unlike unique_ptr)
enum class Kind { Node, SegNode, Ptr, Raw, Inline };

template <class It, class = void>
struct has_value_fn : std::false_type {};
template <class It>
struct has_value_fn<It, std::void_t<decltype(std::declval<It&>().value())>> : std::true_type {};

template <class M, class = void>
struct has_try_emplace : std::false_type {};
template <class M>
struct has_try_emplace<
    M, std::void_t<decltype(std::declval<M&>().try_emplace(std::declval<const CoordT&>()))>>
    : std::true_type {};

template <class M, class = void>
struct has_bucket_count : std::false_type {};
template <class M>
struct has_bucket_count<M, std::void_t<decltype(std::declval<const M&>().bucket_count())>>
    : std::true_type {};

template <class M, class = void>
struct has_values_fn : std::false_type {};
template <class M>
struct has_values_fn<M, std::void_t<decltype(std::declval<const M&>().values().data())>>
    : std::true_type {};

template <class Inner, class MapT, Kind K>
class Wrap {
 public:
  static constexpr bool kMovesOnInsert = (K == Kind::Inline);
  // a value never moves while it is in the map, as with std::unordered_map
  static constexpr bool kStable = (K == Kind::Node || K == Kind::Ptr || K == Kind::Raw);
  using map_type = MapT;

  Wrap() = default;
  Wrap(const Wrap&) = delete;
  Wrap& operator=(const Wrap&) = delete;
  Wrap(Wrap&& o) noexcept {
    std::swap(m_, o.m_);
  }
  Wrap& operator=(Wrap&& o) noexcept {
    std::swap(m_, o.m_);
    return *this;
  }
  ~Wrap() {
    destroyValues();
  }

  MX_INLINE Inner* find(const CoordT& k) {
    auto it = m_.find(k);
    if (it == m_.end()) {
      return nullptr;
    }
    return ptrOf(it);
  }
  MX_INLINE const Inner* find(const CoordT& k) const {
    auto it = m_.find(k);
    if (it == m_.end()) {
      return nullptr;
    }
    return cptrOf(it);
  }

  MX_INLINE Inner* emplace(const CoordT& k, uint32_t bits) {
    if constexpr (K == Kind::Raw) {
      Inner* raw = new Inner(bits);
      if constexpr (has_try_emplace<MapT>::value) {
        m_.try_emplace(k, raw);
      } else {
        m_.emplace(k, raw);
      }
      return raw;
    } else if constexpr (K == Kind::Ptr) {
      auto p = std::make_unique<Inner>(bits);
      Inner* raw = p.get();
      if constexpr (has_try_emplace<MapT>::value) {
        m_.try_emplace(k, std::move(p));
      } else {
        m_.emplace(k, std::move(p));
      }
      return raw;
    } else {
      [[maybe_unused]] const auto before = signature();
      auto it = tryEmplace(k, bits);
      if constexpr (K == Kind::Inline) {
        if (signature() != before) {
          moved_ = true;
        }
      }
      return ptrOf(it);
    }
  }

  bool takeMoved() {
    const bool m = moved_;
    moved_ = false;
    return m;
  }

  template <class F>
  void forEach(F&& f) const {
    for (auto it = m_.begin(); it != m_.end(); ++it) {
      f(it->first, *cptrOf(it));
    }
  }
  template <class F>
  void forEachMutable(F&& f) {
    for (auto it = m_.begin(); it != m_.end(); ++it) {
      f(it->first, *ptrOf(it));
    }
  }

  void erase(const CoordT& k) {
    if constexpr (K == Kind::Raw) {
      auto it = m_.find(k);
      if (it != m_.end()) {
        Inner* p = it->second;
        m_.erase(it);
        delete p;
      }
    } else {
      m_.erase(k);
    }
  }
  // CLEAR_MEMORY: give everything back, as CoordMap::clear() does
  void clear() {
    destroyValues();
    MapT empty;
    std::swap(m_, empty);
  }
  void reserve(size_t n) {
    m_.reserve(n);
  }
  size_t size() const {
    return m_.size();
  }
  size_t buckets() const {
    if constexpr (has_bucket_count<MapT>::value) {
      return m_.bucket_count();
    } else {
      return 0;
    }
  }
  size_t memUsage() const {
    return 0;
  }
  MapT& raw() {
    return m_;
  }

 private:
  MapT m_;
  bool moved_ = false;

  void destroyValues() {
    if constexpr (K == Kind::Raw) {
      for (auto it = m_.begin(); it != m_.end(); ++it) {
        delete it->second;
      }
    }
  }

  template <class It>
  static Inner* ptrOf(It& it) {
    if constexpr (K == Kind::Raw) {
      return it->second;
    } else if constexpr (K == Kind::Ptr) {
      return it->second.get();
    } else if constexpr (has_value_fn<It>::value) {
      return &it.value();  // tsl: it->second is const
    } else {
      return &it->second;
    }
  }
  template <class It>
  static const Inner* cptrOf(It& it) {
    if constexpr (K == Kind::Raw) {
      return it->second;
    } else if constexpr (K == Kind::Ptr) {
      return it->second.get();
    } else {
      return &it->second;
    }
  }
  template <class... A>
  auto tryEmplace(const CoordT& k, A&&... a) {
    if constexpr (has_try_emplace<MapT>::value) {
      return m_.try_emplace(k, std::forward<A>(a)...).first;
    } else {
      return m_.emplace(k, std::forward<A>(a)...).first;
    }
  }
  // changes whenever the values may have moved
  std::pair<size_t, const void*> signature() const {
    size_t b = 0;
    const void* d = nullptr;
    if constexpr (has_bucket_count<MapT>::value) {
      b = m_.bucket_count();
    }
    if constexpr (has_values_fn<MapT>::value) {
      d = m_.values().data();
    }
    return {b, d};
  }
};

template <template <class, class, class> class M, class H>
struct Node {
  template <class Inner>
  using Map = Wrap<Inner, M<CoordT, Inner, H>, Kind::Node>;
};
template <template <class, class, class> class M, class H>
struct SegNode {
  template <class Inner>
  using Map = Wrap<Inner, M<CoordT, Inner, H>, Kind::SegNode>;
};
template <template <class, class, class> class M, class H>
struct Ptr {
  template <class Inner>
  using Map = Wrap<Inner, M<CoordT, std::unique_ptr<Inner>, H>, Kind::Ptr>;
};
template <template <class, class, class> class M, class H>
struct Raw {
  template <class Inner>
  using Map = Wrap<Inner, M<CoordT, Inner*, H>, Kind::Raw>;
};
template <template <class, class, class> class M, class H>
struct Inline {
  template <class Inner>
  using Map = Wrap<Inner, M<CoordT, Inner, H>, Kind::Inline>;
};

//---------------------------------------------------------------- CoordMap ---
template <class Inner>
class CoordMapWrap {
 public:
  static constexpr bool kMovesOnInsert = false;
  static constexpr bool kStable = true;
  using map_type = Bonxai::CoordMap<Inner>;

  Inner* find(const CoordT& k) {
    auto it = m_.find(k);
    return it == m_.end() ? nullptr : &it->second;
  }
  const Inner* find(const CoordT& k) const {
    auto it = m_.find(k);
    return it == m_.end() ? nullptr : &it->second;
  }
  Inner* emplace(const CoordT& k, uint32_t bits) {
    return &m_.try_emplace(k, bits).first->second;
  }
  bool takeMoved() {
    return false;
  }
  template <class F>
  void forEach(F&& f) const {
    for (const auto& kv : m_) {
      f(kv.first, kv.second);
    }
  }
  template <class F>
  void forEachMutable(F&& f) {
    for (auto& kv : m_) {
      f(kv.first, kv.second);
    }
  }
  void erase(const CoordT& k) {
    m_.erase(k);
  }
  void clear() {
    m_.clear();
  }
  void reserve(size_t n) {
    m_.reserve(n);
  }
  size_t size() const {
    return m_.size();
  }
  size_t buckets() const {
    return 0;
  }
  size_t memUsage() const {
    return m_.memUsage();
  }
  map_type& raw() {
    return m_;
  }

 private:
  map_type m_;
};

struct CoordMapPolicy {
  template <class Inner>
  using Map = CoordMapWrap<Inner>;
};

//------------------------------------------------------ the maps themselves ---
template <class K, class V, class H>
using StdMap = std::unordered_map<K, V, H, KeyEq>;

#ifdef WITH_ANKERL
template <class K, class V, class H>
using AnkerlMap = ankerl::unordered_dense::map<K, V, H, KeyEq>;
template <class K, class V, class H>
using AnkerlSeg = ankerl::unordered_dense::segmented_map<K, V, H, KeyEq>;
#endif
#ifdef WITH_BOOST
template <class K, class V, class H>
using BoostFca = boost::unordered_map<K, V, H, KeyEq>;
template <class K, class V, class H>
using BoostNode = boost::unordered_node_map<K, V, H, KeyEq>;
template <class K, class V, class H>
using BoostFlat = boost::unordered_flat_map<K, V, H, KeyEq>;
#endif
#ifdef WITH_ABSL
template <class K, class V, class H>
using AbslNode = absl::node_hash_map<K, V, H, KeyEq>;
template <class K, class V, class H>
using AbslFlat = absl::flat_hash_map<K, V, H, KeyEq>;
using HAbsl = absl::Hash<CoordT>;
#endif
#ifdef WITH_PHMAP
template <class K, class V, class H>
using PhNode = phmap::node_hash_map<K, V, H, KeyEq>;
template <class K, class V, class H>
using PhFlat = phmap::flat_hash_map<K, V, H, KeyEq>;
#endif
#ifdef WITH_TSL
template <class K, class V, class H>
using TslRobin = tsl::robin_map<K, V, H, KeyEq, std::allocator<std::pair<K, V>>, false>;
template <class K, class V, class H>
using TslRobinSH = tsl::robin_map<K, V, H, KeyEq, std::allocator<std::pair<K, V>>, true>;
template <class K, class V, class H>
using TslHop = tsl::hopscotch_map<K, V, H, KeyEq>;
template <class K, class V, class H>
using TslHopSH = tsl::hopscotch_map<K, V, H, KeyEq, std::allocator<std::pair<K, V>>, 30, true>;
template <class K, class V, class H>
using TslSparse = tsl::sparse_map<K, V, H, KeyEq>;
#endif
#ifdef WITH_SKA
template <class K, class V, class H>
using SkaFlat = ska::flat_hash_map<K, V, H, KeyEq>;
template <class K, class V, class H>
using SkaBytell = ska::bytell_hash_map<K, V, H, KeyEq>;
#endif
#ifdef WITH_EMHASH5
template <class K, class V, class H>
using Em5 = emhash5::HashMap<K, V, H, KeyEq>;
#endif
#ifdef WITH_EMHASH6
template <class K, class V, class H>
using Em6 = emhash6::HashMap<K, V, H, KeyEq>;
#endif
#ifdef WITH_EMHASH7
template <class K, class V, class H>
using Em7 = emhash7::HashMap<K, V, H, KeyEq>;
#endif
#ifdef WITH_EMHASH8
template <class K, class V, class H>
using Em8 = emhash8::HashMap<K, V, H, KeyEq>;
#endif
#ifdef WITH_ROBINHOOD
template <class K, class V, class H>
using RhNode = robin_hood::unordered_node_map<K, V, H, KeyEq>;
template <class K, class V, class H>
using RhFlat = robin_hood::unordered_flat_map<K, V, H, KeyEq>;
#endif
#ifdef WITH_FPH
// fph wants a seeded hash that is injective on the keys: 21 bits per coordinate is
// injective for the root keys of any map smaller than 2^20 roots of 1.6 m per side
inline uint64_t pack21(const CoordT& p) noexcept {
  return (uint64_t(uint32_t(p.x)) & 0x1FFFFF) | ((uint64_t(uint32_t(p.y)) & 0x1FFFFF) << 21) |
         ((uint64_t(uint32_t(p.z)) & 0x1FFFFF) << 42);
}
struct FphSeedHash {
  size_t operator()(const CoordT& p, size_t seed) const noexcept {
    return fph::MixSeedHash<uint64_t>{}(pack21(p), seed);
  }
};
struct FphStrongSeedHash {
  size_t operator()(const CoordT& p, size_t seed) const noexcept {
    return fph::StrongSeedHash<uint64_t>{}(pack21(p), seed);
  }
};
struct FphKeyGen {
  std::mt19937_64 rng{12345};
  CoordT operator()() {
    const uint64_t r = rng();
    return {
        int32_t(r & 0x1FFFFF) - (1 << 20), int32_t((r >> 21) & 0x1FFFFF) - (1 << 20),
        int32_t((r >> 42) & 0x1FFFFF) - (1 << 20)};
  }
};
template <class K, class V, class H>
using FphDyn =
    fph::DynamicFphMap<K, V, H, KeyEq, std::allocator<std::pair<const K, V>>, uint32_t, FphKeyGen>;
template <class K, class V, class H>
using FphMeta = fph::MetaFphMap<K, V, H, KeyEq, std::allocator<std::pair<const K, V>>, uint32_t>;
#endif

}  // namespace mx
