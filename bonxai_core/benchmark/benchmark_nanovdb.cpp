/*
 * Copyright Contributors to the Bonxai Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Bonxai against NanoVDB on an identical workload.
//
// NanoVDB is two data structures, and the distinction matters when reading the
// numbers:
//
//   tools::build::Grid  mutable, host side, 5-4-3 tree (32^3 -> 16^3 -> 8^3
//                       nodes under a std::map root). This is the one that can
//                       be compared with Bonxai's VoxelGrid.
//   NanoGrid            read only, produced by createNanoGrid(), with the whole
//                       tree linearised into a single buffer. Bonxai has no
//                       equivalent, so it is reported separately as the best
//                       NanoVDB can do for lookups.
//
// Both libraries are given the exact same integer coordinates, so no
// coordinate-conversion difference leaks into the comparison.

#include <benchmark/benchmark.h>
#include <nanovdb/NanoVDB.h>
#include <nanovdb/tools/CreateNanoGrid.h>
#include <nanovdb/tools/GridBuilder.h>

#include <algorithm>
#include <random>
#include <vector>

#include "bonxai/bonxai.hpp"
#include "synthetic_scan.hpp"

using namespace Bonxai;

static constexpr double kVoxelSize = 0.05;

static const std::vector<CoordT>& ScanCoords() {
  static const std::vector<CoordT> coords = [] {
    const auto points = MakeSyntheticScan({0.0, 0.0, 1.0});
    std::vector<CoordT> out;
    out.reserve(points.size());
    for (const auto& p : points) {
      out.push_back(PosToCoord(p, 1.0 / kVoxelSize));
    }
    return out;
  }();
  return coords;
}

/// A spatially incoherent permutation: defeats both accessor caches and shows
/// the cost of a cold descent through the tree.
static const std::vector<CoordT>& ShuffledScanCoords() {
  static const std::vector<CoordT> coords = [] {
    auto out = ScanCoords();
    std::shuffle(out.begin(), out.end(), std::mt19937(42));
    return out;
  }();
  return coords;
}

static nanovdb::Coord ToNano(const CoordT& c) {
  return nanovdb::Coord(c.x, c.y, c.z);
}

/// Both libraries are filled from the same coordinates by these two helpers, so
/// the workload can only ever drift in one place.
template <typename Shape>
static VoxelGrid<float, Shape> MakeBonxaiGrid() {
  VoxelGrid<float, Shape> grid(kVoxelSize);
  auto accessor = grid.createAccessor();
  for (const auto& coord : ScanCoords()) {
    accessor.setValue(coord, 1.0f);
  }
  return grid;
}

// build::Grid is neither copyable nor movable, so it is filled in place
static void FillFromScan(nanovdb::tools::build::Grid<float>& builder) {
  auto accessor = builder.getAccessor();
  for (const auto& coord : ScanCoords()) {
    accessor.setValue(ToNano(coord), 1.0f);
  }
}

//---------------------------------------------------------------- create -----

template <typename Shape>
static void Bonxai_NV_Create(benchmark::State& state) {
  const auto& coords = ScanCoords();
  for (auto _ : state) {
    VoxelGrid<float, Shape> grid(kVoxelSize);
    auto accessor = grid.createAccessor();
    for (const auto& coord : coords) {
      accessor.setValue(coord, 1.0f);
    }
    benchmark::DoNotOptimize(grid);
  }
  state.SetItemsProcessed(state.iterations() * coords.size());
}

static void NanoVDB_Create(benchmark::State& state) {
  const auto& coords = ScanCoords();
  for (auto _ : state) {
    nanovdb::tools::build::Grid<float> grid(0.0f);
    auto accessor = grid.getAccessor();
    for (const auto& coord : coords) {
      accessor.setValue(ToNano(coord), 1.0f);
    }
    benchmark::DoNotOptimize(grid);
  }
  state.SetItemsProcessed(state.iterations() * coords.size());
}

//---------------------------------------------------------------- update -----

template <typename Shape>
static void Bonxai_NV_Update(benchmark::State& state) {
  const auto& coords = ScanCoords();
  VoxelGrid<float, Shape> grid(kVoxelSize);
  {
    auto accessor = grid.createAccessor();
    for (const auto& coord : coords) {
      accessor.setValue(coord, 1.0f);
    }
  }
  for (auto _ : state) {
    auto accessor = grid.createAccessor();
    for (const auto& coord : coords) {
      accessor.setValue(coord, 2.0f);
    }
  }
  state.SetItemsProcessed(state.iterations() * coords.size());
}

static void NanoVDB_Update(benchmark::State& state) {
  const auto& coords = ScanCoords();
  nanovdb::tools::build::Grid<float> grid(0.0f);
  {
    auto accessor = grid.getAccessor();
    for (const auto& coord : coords) {
      accessor.setValue(ToNano(coord), 1.0f);
    }
  }
  for (auto _ : state) {
    auto accessor = grid.getAccessor();
    for (const auto& coord : coords) {
      accessor.setValue(ToNano(coord), 2.0f);
    }
  }
  state.SetItemsProcessed(state.iterations() * coords.size());
}

//----------------------------------------------------------------- query -----
// Arg(0): scan order, spatially coherent.  Arg(1): shuffled.

template <typename Shape>
static void Bonxai_NV_Query(benchmark::State& state) {
  const auto& coords = state.range(0) ? ShuffledScanCoords() : ScanCoords();
  auto grid = MakeBonxaiGrid<Shape>();
  float sum = 0;
  for (auto _ : state) {
    auto accessor = grid.createConstAccessor();
    for (const auto& coord : coords) {
      if (const float* value = accessor.value(coord)) {
        sum += *value;
      }
    }
    benchmark::DoNotOptimize(sum);
  }
  state.SetItemsProcessed(state.iterations() * coords.size());
}

static void NanoVDB_Query(benchmark::State& state) {
  const auto& coords = state.range(0) ? ShuffledScanCoords() : ScanCoords();
  nanovdb::tools::build::Grid<float> grid(0.0f);
  FillFromScan(grid);
  float sum = 0;
  for (auto _ : state) {
    auto accessor = grid.getAccessor();
    for (const auto& coord : coords) {
      sum += accessor.getValue(ToNano(coord));
    }
    benchmark::DoNotOptimize(sum);
  }
  state.SetItemsProcessed(state.iterations() * coords.size());
}

static void NanoVDBReadOnly_Query(benchmark::State& state) {
  const auto& coords = state.range(0) ? ShuffledScanCoords() : ScanCoords();
  nanovdb::tools::build::Grid<float> builder(0.0f);
  FillFromScan(builder);
  auto handle = nanovdb::tools::createNanoGrid(builder);
  const auto* grid = handle.grid<float>();
  float sum = 0;
  for (auto _ : state) {
    auto accessor = grid->getAccessor();
    for (const auto& coord : coords) {
      sum += accessor.getValue(ToNano(coord));
    }
    benchmark::DoNotOptimize(sum);
  }
  state.SetItemsProcessed(state.iterations() * coords.size());
}

//--------------------------------------------------------------- iterate -----

template <typename Shape>
static void Bonxai_NV_Iterate(benchmark::State& state) {
  auto grid = MakeBonxaiGrid<Shape>();
  double sum = 0;
  for (auto _ : state) {
    grid.forEachCell([&](float& value, const CoordT& coord) { sum += value + coord.x; });
    benchmark::DoNotOptimize(sum);
  }
  state.counters["cells"] = double(grid.activeCellsCount());
}

static void NanoVDBReadOnly_Iterate(benchmark::State& state) {
  nanovdb::tools::build::Grid<float> builder(0.0f);
  FillFromScan(builder);
  auto handle = nanovdb::tools::createNanoGrid(builder);
  const auto* grid = handle.grid<float>();
  const auto& tree = grid->tree();
  double sum = 0;
  size_t count = 0;
  for (auto _ : state) {
    count = 0;
    for (uint32_t i = 0; i < tree.nodeCount(0); ++i) {
      const auto* leaf = tree.getFirstLeaf() + i;
      for (auto it = leaf->cbeginValueOn(); it; ++it) {
        sum += *it + leaf->offsetToGlobalCoord(it.pos()).x();
        ++count;
      }
    }
    benchmark::DoNotOptimize(sum);
  }
  state.counters["cells"] = double(count);
}

//----------------------------------------------- a large, sparse map --------
// The scan above fits in a room, that is, a handful of root nodes. Mapping a
// building or an outdoor area gives the root map hundreds of thousands of
// entries, which is the regime where its hash function matters.
//
// The two libraries are not built the same way below the root, so both shapes
// worth comparing are measured:
//
//   NanoVDB            root map -> upper 32^3 -> lower 16^3 -> leaf 8^3
//   StaticShape<2, 3>  root map ->               inner  4^3 -> leaf 8^3
//   StaticShape<4, 3>  root map ->               inner 16^3 -> leaf 8^3
//
// StaticShape<4, 3> is the fair comparison: same 8^3 leaf and same 16^3 node
// above it as NanoVDB, the only difference being NanoVDB's extra 32^3 level,
// whose job the root map does in Bonxai. It covers 128 voxels per side instead
// of the default's 32, so it needs far fewer roots, and pays for it with dense
// 16^3 inner nodes.

static const std::vector<CoordT>& WideCoords() {
  static const std::vector<CoordT> coords = [] {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> spread(-100.0, 100.0);
    std::vector<CoordT> out;
    out.reserve(300000);
    for (size_t i = 0; i < out.capacity(); ++i) {
      out.push_back(PosToCoord({spread(rng), spread(rng), spread(rng)}, 1.0 / kVoxelSize));
    }
    return out;
  }();
  return coords;
}

template <typename Shape>
static void Bonxai_NV_WideCreate(benchmark::State& state) {
  const auto& coords = WideCoords();
  size_t roots = 0;
  double megabytes = 0;
  for (auto _ : state) {
    // the bits must be passed explicitly: the constructor defaults to 2 and 3, which
    // a StaticShape with different branching factors rejects
    VoxelGrid<float, Shape> grid(kVoxelSize, Shape::INNER_BITS, Shape::LEAF_BITS);
    auto accessor = grid.createAccessor();
    for (const auto& coord : coords) {
      accessor.setValue(coord, 1.0f);
    }
    roots = grid.rootMap().size();
    megabytes = double(grid.memUsage()) / 1e6;
    benchmark::DoNotOptimize(grid);
  }
  state.counters["roots"] = double(roots);
  state.counters["MB"] = megabytes;
  state.SetItemsProcessed(state.iterations() * coords.size());
}

static void NanoVDB_WideCreate(benchmark::State& state) {
  const auto& coords = WideCoords();
  for (auto _ : state) {
    nanovdb::tools::build::Grid<float> grid(0.0f);
    auto accessor = grid.getAccessor();
    for (const auto& coord : coords) {
      accessor.setValue(ToNano(coord), 1.0f);
    }
    benchmark::DoNotOptimize(grid);
  }
  state.SetItemsProcessed(state.iterations() * coords.size());
}

template <typename Shape>
static void Bonxai_NV_WideQuery(benchmark::State& state) {
  const auto& coords = WideCoords();
  VoxelGrid<float, Shape> grid(kVoxelSize, Shape::INNER_BITS, Shape::LEAF_BITS);
  {
    auto accessor = grid.createAccessor();
    for (const auto& coord : coords) {
      accessor.setValue(coord, 1.0f);
    }
  }
  float sum = 0;
  for (auto _ : state) {
    auto accessor = grid.createConstAccessor();
    for (const auto& coord : coords) {
      if (const float* value = accessor.value(coord)) {
        sum += *value;
      }
    }
    benchmark::DoNotOptimize(sum);
  }
  state.counters["roots"] = double(grid.rootMap().size());
  state.SetItemsProcessed(state.iterations() * coords.size());
}

static void NanoVDB_WideQuery(benchmark::State& state) {
  const auto& coords = WideCoords();
  nanovdb::tools::build::Grid<float> grid(0.0f);
  {
    auto accessor = grid.getAccessor();
    for (const auto& coord : coords) {
      accessor.setValue(ToNano(coord), 1.0f);
    }
  }
  float sum = 0;
  for (auto _ : state) {
    auto accessor = grid.getAccessor();
    for (const auto& coord : coords) {
      sum += accessor.getValue(ToNano(coord));
    }
    benchmark::DoNotOptimize(sum);
  }
  state.SetItemsProcessed(state.iterations() * coords.size());
}

//------------------------------------------------------------ one-offs -------

/// Cost of linearising the mutable tree into the read-only NanoGrid. Bonxai has
/// no equivalent step, so this is what the NanoVDB lookup numbers cost up front.
static void NanoVDB_Convert(benchmark::State& state) {
  nanovdb::tools::build::Grid<float> builder(0.0f);
  FillFromScan(builder);
  for (auto _ : state) {
    auto handle = nanovdb::tools::createNanoGrid(builder);
    benchmark::DoNotOptimize(handle);
  }
}

static void MemoryUsage(benchmark::State& state) {
  auto bonxai_grid = MakeBonxaiGrid<StaticShape<>>();
  nanovdb::tools::build::Grid<float> builder(0.0f);
  FillFromScan(builder);
  auto handle = nanovdb::tools::createNanoGrid(builder);
  for (auto _ : state) {}
  state.counters["Bonxai_MB"] = double(bonxai_grid.memUsage()) / 1e6;
  state.counters["NanoVDB_ReadOnly_MB"] = double(handle.bufferSize()) / 1e6;
  state.counters["cells"] = double(bonxai_grid.activeCellsCount());
}

// StaticShape is the default: the branching factors are compile-time constants.
// DynamicShape holds them as members, which is what a grid read back by
// Deserialize() gets, and is the only behaviour available before this change.
BENCHMARK_TEMPLATE(Bonxai_NV_Create, StaticShape<>)->MinTime(1);
BENCHMARK_TEMPLATE(Bonxai_NV_Create, DynamicShape)->MinTime(1);
BENCHMARK(NanoVDB_Create)->MinTime(1);
BENCHMARK_TEMPLATE(Bonxai_NV_Update, StaticShape<>)->MinTime(1);
BENCHMARK_TEMPLATE(Bonxai_NV_Update, DynamicShape)->MinTime(1);
BENCHMARK(NanoVDB_Update)->MinTime(1);
BENCHMARK_TEMPLATE(Bonxai_NV_Query, StaticShape<>)->Arg(0)->Arg(1)->MinTime(1);
BENCHMARK_TEMPLATE(Bonxai_NV_Query, DynamicShape)->Arg(0)->Arg(1)->MinTime(1);
BENCHMARK(NanoVDB_Query)->Arg(0)->Arg(1)->MinTime(1);
BENCHMARK(NanoVDBReadOnly_Query)->Arg(0)->Arg(1)->MinTime(1);
BENCHMARK_TEMPLATE(Bonxai_NV_Iterate, StaticShape<>)->MinTime(1);
BENCHMARK_TEMPLATE(Bonxai_NV_Iterate, DynamicShape)->MinTime(1);
BENCHMARK(NanoVDBReadOnly_Iterate)->MinTime(1);
BENCHMARK_TEMPLATE(Bonxai_NV_WideCreate, StaticShape<>)->MinTime(1);
BENCHMARK_TEMPLATE(Bonxai_NV_WideCreate, StaticShape<4, 3>)->MinTime(1);
BENCHMARK(NanoVDB_WideCreate)->MinTime(1);
BENCHMARK_TEMPLATE(Bonxai_NV_WideQuery, StaticShape<>)->MinTime(1);
BENCHMARK_TEMPLATE(Bonxai_NV_WideQuery, StaticShape<4, 3>)->MinTime(1);
BENCHMARK(NanoVDB_WideQuery)->MinTime(1);
BENCHMARK(NanoVDB_Convert)->MinTime(1);
BENCHMARK(MemoryUsage);

BENCHMARK_MAIN();
