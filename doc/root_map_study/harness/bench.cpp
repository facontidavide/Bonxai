// bench <workload>  -> one JSON line. The root map is compiled in: -DBONXAI_DEFAULT_POLICY=...
// With -DREAL, the headers of a Bonxai checkout are used as they are (no policy).
#include <malloc.h>
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#ifndef REAL
#include "maps.hpp"
#endif
#include "bonxai/bonxai.hpp"
#include "synthetic_scan.hpp"
#ifdef WITH_PMAP
#include "bonxai_map/probabilistic_map.hpp"
#include "probabilistic_map.cpp"
#endif

#ifndef VARIANT_NAME
#define VARIANT_NAME "unnamed"
#endif

using namespace Bonxai;
using Clock = std::chrono::steady_clock;
using VGrid = VoxelGrid<float, StaticShape<>>;

static long minorFaults() {
  struct rusage r;
  getrusage(RUSAGE_SELF, &r);
  return r.ru_minflt;
}
static double ms_since(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}
static size_t heapBytes() {
  struct mallinfo2 mi = mallinfo2();
  return mi.uordblks + mi.hblkhd;
}

struct Out {
  std::map<std::string, double> v;
  void print(const std::string& variant, const std::string& workload) {
    printf("{\"variant\":\"%s\",\"workload\":\"%s\"", variant.c_str(), workload.c_str());
    for (auto& [k, x] : v) {
      printf(",\"%s\":%.6g", k.c_str(), x);
    }
    printf("}\n");
  }
};

struct KeyHash {
  size_t operator()(const CoordT& c) const {
    uint64_t h = (uint64_t(uint32_t(c.x)) | (uint64_t(uint32_t(c.y)) << 32)) ^
                 uint64_t(uint32_t(c.z)) * 0x9e3779b97f4a7c15ull;
    h ^= h >> 31;
    h *= 0xbf58476d1ce4e5b9ull;
    return h ^ (h >> 29);
  }
};
using KeySet = std::unordered_set<CoordT, KeyHash>;

//------------------------------------------------------ key distributions ---
// n distinct ROOT keys. `miss` gives n distinct keys of the same kind that are not in
// the first set.
static std::vector<CoordT> rootKeys(
    const std::string& dist, size_t n, bool miss, uint32_t seed = 1) {
  std::vector<CoordT> out;
  out.reserve(n);
  std::mt19937 rng(seed + (miss ? 1000 : 0));
  if (dist == "rand" || dist == "stride") {
    // uniform in a cube 8 times as large as the set, centred on the origin. The hits are
    // the first n distinct draws, the misses the next n distinct draws not among them
    const int side = int(std::ceil(std::cbrt(8.0 * n)));
    const int s = dist == "stride" ? 16 : 1;
    std::uniform_int_distribution<int> u(-side / 2, side - side / 2 - 1);
    std::mt19937 r(seed);
    KeySet hits;
    std::vector<CoordT> hv;
    while (hv.size() < n) {
      const CoordT c{u(r) * s, u(r) * s, u(r) * s};
      if (hits.insert(c).second) {
        hv.push_back(c);
      }
    }
    if (!miss) {
      return hv;
    }
    KeySet seen;
    while (out.size() < n) {
      const CoordT c{u(r) * s, u(r) * s, u(r) * s};
      if (!hits.count(c) && seen.insert(c).second) {
        out.push_back(c);
      }
    }
    return out;
  }
  if (dist == "dense") {
    // every key of a cube; the misses are the cube next to it
    const int side = int(std::ceil(std::cbrt(double(n))));
    const int off = miss ? side : 0;
    for (int x = 0; x < side && out.size() < n; ++x) {
      for (int y = 0; y < side && out.size() < n; ++y) {
        for (int z = 0; z < side && out.size() < n; ++z) {
          out.push_back({x - side / 2 + off, y - side / 2, z - side / 2});
        }
      }
    }
  } else if (dist == "plane") {
    // a rolling terrain: one layer of keys, z a smooth function of x and y; the misses
    // are the keys right above it
    const int side = int(std::ceil(std::sqrt(double(n))));
    for (int x = 0; x < side && out.size() < n; ++x) {
      for (int y = 0; y < side && out.size() < n; ++y) {
        const int z = int(std::lround(3.0 * std::sin(x / 17.0) * std::cos(y / 23.0)));
        out.push_back({x - side / 2, y - side / 2, z + (miss ? 1 : 0)});
      }
    }
  } else if (dist == "line") {
    // a long corridor, two keys wide and high: the misses are next to it
    for (size_t i = 0; out.size() < n; ++i) {
      const int x = int(i / 4) - int(n / 8);
      const int y = int(i % 2) + (miss ? 2 : 0);
      const int z = int((i / 2) % 2);
      out.push_back({x, y, z});
    }
  } else {
    fprintf(stderr, "unknown distribution %s\n", dist.c_str());
    std::exit(2);
  }
  std::shuffle(out.begin(), out.end(), rng);
  return out;
}

// one cell in each root key, at a random place inside it
static std::vector<CoordT> cellsIn(const std::vector<CoordT>& roots, uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> u(0, 31);
  std::vector<CoordT> out;
  out.reserve(roots.size());
  for (const auto& r : roots) {
    out.push_back({r.x * 32 + u(rng), r.y * 32 + u(rng), r.z * 32 + u(rng)});
  }
  return out;
}

static std::vector<CoordT> wideCoords(size_t n, double half, double offset, uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> spread(-half, half);
  std::vector<CoordT> out;
  out.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    // same draw order as benchmark_nanovdb's WideCoords
    const double x = spread(rng), y = spread(rng), z = spread(rng);
    out.push_back(PosToCoord({x + offset, y + offset, z + offset}, 1.0 / 0.05));
  }
  return out;
}

// at least `atleast` queries, drawn from `from` in a random order
static std::vector<CoordT> sampleQueries(
    const std::vector<CoordT>& from, size_t atleast, uint32_t seed) {
  std::vector<CoordT> out;
  out.reserve(std::max(atleast, from.size()));
  std::mt19937 rng(seed);
  while (out.size() < atleast) {
    auto chunk = from;
    std::shuffle(chunk.begin(), chunk.end(), rng);
    out.insert(out.end(), chunk.begin(), chunk.end());
  }
  return out;
}

//--------------------------------------------------------- VoxelGrid level ---
template <class Grid>
static double queryPass(
    const Grid& grid, const std::vector<CoordT>& coords, double& sum, size_t& hits) {
  auto t0 = Clock::now();
  auto acc = grid.createConstAccessor();
  double s = 0;
  size_t h = 0;
  for (const auto& c : coords) {
    if (const float* v = acc.value(c)) {
      s += *v;
      ++h;
    }
  }
  const double t = ms_since(t0);
  sum = s;
  hits = h;
  return t;
}

template <class Grid>
static double bestQuery(
    const Grid& grid, const std::vector<CoordT>& coords, int passes, double& sum, size_t& hits) {
  double best = 1e300;
  for (int i = 0; i < passes; ++i) {
    best = std::min(best, queryPass(grid, coords, sum, hits));
  }
  return best;
}

template <class Grid>
static double iteratePass(const Grid& grid, double& sum) {
  auto t0 = Clock::now();
  double s = 0;
  grid.forEachCell([&](const float& v, const CoordT& c) { s += v + c.x; });
  sum = s;
  return ms_since(t0);
}

template <class Grid>
static void fill(Grid& grid, const std::vector<CoordT>& cells) {
  auto acc = grid.createAccessor();
  for (const auto& c : cells) {
    acc.setValue(c, 1.0f);
  }
}

// cells: the grid; hitq / missq: the queries, in the order they are made
static void runCells(
    Out& out, const std::vector<CoordT>& cells, const std::vector<CoordT>& hitq,
    const std::vector<CoordT>& missq, int passes) {
  {
    // the first build of the process: includes the page faults of fresh memory
    const long f0 = minorFaults();
    auto t0 = Clock::now();
    auto grid0 = std::make_unique<VGrid>(0.05);
    fill(*grid0, cells);
    out.v["coldbuild_ms"] = ms_since(t0);
    out.v["coldbuild_faults"] = double(minorFaults() - f0);
  }
  // warm builds, into memory the allocator already has
  const int builds = std::clamp(int(300000 / cells.size()), 1, 50);
  double best = 1e300;
  const long fw = minorFaults();
  for (int b = 0; b < builds; ++b) {
    auto t0 = Clock::now();
    auto g = std::make_unique<VGrid>(0.05);
    fill(*g, cells);
    best = std::min(best, ms_since(t0));
  }
  out.v["build_ms"] = best;
  out.v["build_faults"] = double(minorFaults() - fw) / builds;

  const size_t heap0 = heapBytes();
  VGrid grid(0.05);
  fill(grid, cells);
  out.v["heap_mb"] = double(heapBytes() - heap0) / 1e6;
  out.v["roots"] = double(grid.rootMap().size());

  double sum;
  size_t hits;
  out.v["hit_ms"] = bestQuery(grid, hitq, passes, sum, hits);
  out.v["hit_sum"] = sum;
  out.v["miss_ms"] = bestQuery(grid, missq, passes, sum, hits);
  out.v["miss_hits"] = double(hits);
  {
    // update existing cells: the find path of the mutable accessor
    double b = 1e300;
    for (int p = 0; p < passes; ++p) {
      auto t1 = Clock::now();
      auto acc = grid.createAccessor();
      for (const auto& c : hitq) {
        acc.setValue(c, 2.0f);
      }
      b = std::min(b, ms_since(t1));
    }
    out.v["update_ms"] = b;
  }
  best = 1e300;
  const int iters = std::clamp(int(1000000 / cells.size()), 1, 200);
  for (int p = 0; p < passes; ++p) {
    auto t1 = Clock::now();
    for (int i = 0; i < iters; ++i) {
      iteratePass(grid, sum);
    }
    best = std::min(best, ms_since(t1) / iters);
  }
  out.v["iter_ms"] = best;
  out.v["iter_sum"] = sum;
  auto t2 = Clock::now();
  grid.clear(CLEAR_MEMORY);
  out.v["clear_ms"] = ms_since(t2);
}

static void runWide(Out& out, size_t n, double half, double offset, int passes) {
  const auto pts = wideCoords(n, half, offset, 42);
  const auto miss = wideCoords(n, half, offset, 4242);
  runCells(out, pts, pts, miss, passes);
}

// one cell per root key of the distribution; queries in a random order
static void runDist(Out& out, const std::string& dist, size_t n, int passes) {
  const auto roots = rootKeys(dist, n, false);
  const auto mroots = rootKeys(dist, n, true);
  const auto cells = cellsIn(roots, 5);
  const size_t q = std::max<size_t>(n, 1000000);
  const auto hitq = sampleQueries(cells, q, 6);
  const auto missq = sampleQueries(cellsIn(mroots, 7), q, 8);
  runCells(out, cells, hitq, missq, passes);
}

// growth<N>k: build the wide map of N thousand cells three times, the first one cold, timing
// every insertion: the worst one is the stall of the largest growth of the map, which a
// flat map pays by moving its values and CoordMap by moving its buckets
static void runGrowth(Out& out, size_t n, double half) {
  const auto pts = wideCoords(n, half, 0.0, 42);
  for (int b = 0; b < 3; ++b) {
    const std::string tag = b == 0 ? "cold_" : "warm" + std::to_string(b) + "_";
    const long f0 = minorFaults();
    double worst = 0, over1 = 0, second = 0;
    size_t n1 = 0;
    auto t0 = Clock::now();
    auto g = std::make_unique<VGrid>(0.05);
    {
      auto acc = g->createAccessor();
      for (const auto& c : pts) {
        const auto a = Clock::now();
        acc.setValue(c, 1.0f);
        const double us = std::chrono::duration<double, std::micro>(Clock::now() - a).count();
        if (us > worst) {
          second = worst;
          worst = us;
        } else if (us > second) {
          second = us;
        }
        if (us > 1000) {
          over1 += us;
          ++n1;
        }
      }
    }
    out.v[tag + "build_ms"] = ms_since(t0);
    out.v[tag + "faults"] = double(minorFaults() - f0);
    out.v[tag + "worst_ms"] = worst / 1000;
    out.v[tag + "second_ms"] = second / 1000;
    out.v[tag + "over1ms_n"] = double(n1);
    out.v[tag + "over1ms_ms"] = over1 / 1000;
    out.v["roots"] = double(g->rootMap().size());
    const long f1 = minorFaults();
    auto t1 = Clock::now();
    g.reset();
    out.v[tag + "destroy_ms"] = ms_since(t1);
    out.v[tag + "destroy_faults"] = double(minorFaults() - f1);
  }
}

// the room scan of benchmark_nanovdb: 354 roots
static void runRoom(Out& out, int passes) {
  const auto points = MakeSyntheticScan({0.0, 0.0, 1.0});
  std::vector<CoordT> coords;
  for (const auto& p : points) {
    coords.push_back(PosToCoord(p, 1.0 / 0.05));
  }
  auto shuffled = coords;
  std::shuffle(shuffled.begin(), shuffled.end(), std::mt19937(42));

  double best = 1e300;
  for (int p = 0; p < passes * 3; ++p) {
    auto t0 = Clock::now();
    VGrid grid(0.05);
    fill(grid, coords);
    best = std::min(best, ms_since(t0));
  }
  out.v["build_ms"] = best;
  VGrid grid(0.05);
  fill(grid, coords);
  out.v["roots"] = double(grid.rootMap().size());
  double sum;
  size_t hits;
  out.v["scan_ms"] = bestQuery(grid, coords, passes * 3, sum, hits);
  out.v["shuffled_ms"] = bestQuery(grid, shuffled, passes * 3, sum, hits);
  out.v["hit_sum"] = sum;
  best = 1e300;
  for (int p = 0; p < passes * 3; ++p) {
    auto t1 = Clock::now();
    auto acc = grid.createAccessor();
    for (const auto& c : coords) {
      acc.setValue(c, 2.0f);
    }
    best = std::min(best, ms_since(t1));
  }
  out.v["update_ms"] = best;
}

// create and destroy the room scan's grid in a loop, under glibc's default settings:
// what the allocator does with the memory handed back is part of the timing
static void runRoomCreate(Out& out) {
  const auto points = MakeSyntheticScan({0.0, 0.0, 1.0});
  std::vector<CoordT> coords;
  for (const auto& p : points) {
    coords.push_back(PosToCoord(p, 1.0 / 0.05));
  }
  std::vector<double> t;
  for (int i = 0; i < 60; ++i) {
    auto t0 = Clock::now();
    {
      VGrid grid(0.05);
      fill(grid, coords);
    }
    t.push_back(ms_since(t0));
  }
  std::sort(t.begin(), t.end());
  out.v["create_ms"] = t[t.size() / 2];
  out.v["create_min_ms"] = t[0];
}

// lidar-like ray casting along a trajectory: coherent accesses, many new roots
struct RayParams {
  double voxel = 0.05;
  int scans = 40;
  int rings = 16;
  int columns = 256;
  double max_range = 30.0;
  double scan_step = 10.0;
};

static void runRays(Out& out, const RayParams& rp, int passes) {
  const size_t heap0 = heapBytes();
  VGrid grid(rp.voxel);
  std::vector<CoordT> endpoints;
  auto t0 = Clock::now();
  size_t steps = 0;
  {
    auto acc = grid.createAccessor();
    for (int s = 0; s < rp.scans; ++s) {
      const double ox = -0.5 * rp.scans * rp.scan_step + s * rp.scan_step;
      const double oy = 40.0 * std::sin(s * 0.07);
      const double oz = 1.8;
      for (int r = 0; r < rp.rings; ++r) {
        const double pitch = (-15.0 + 30.0 * r / (rp.rings - 1)) * M_PI / 180.0;
        for (int c = 0; c < rp.columns; ++c) {
          const double yaw = 2.0 * M_PI * (c + 0.37 * s) / rp.columns;
          const double dx = std::cos(pitch) * std::cos(yaw);
          const double dy = std::cos(pitch) * std::sin(yaw);
          const double dz = std::sin(pitch);
          double range = rp.max_range;
          if (dz < -1e-6) {
            range = std::min(range, oz / -dz);
          } else {
            // a pseudo random obstacle
            uint32_t h =
                uint32_t(s * 73856093u) ^ uint32_t(r * 19349669u) ^ uint32_t(c * 83492791u);
            h ^= h >> 13;
            h *= 0x5bd1e995u;
            h ^= h >> 15;
            range = std::min(range, 4.0 + (h % 1000) * (rp.max_range - 4.0) / 1000.0);
          }
          const int n = int(range / rp.voxel);
          for (int i = 0; i < n; ++i) {
            const double t = i * rp.voxel;
            const CoordT coord = grid.posToCoord(ox + dx * t, oy + dy * t, oz + dz * t);
            float* v = acc.value(coord, true);
            *v -= 0.1f;
          }
          steps += n;
          const CoordT end = grid.posToCoord(ox + dx * range, oy + dy * range, oz + dz * range);
          *acc.value(end, true) += 0.9f;
          endpoints.push_back(end);
        }
      }
    }
  }
  out.v["build_ms"] = ms_since(t0);
  out.v["steps_M"] = steps / 1e6;
  out.v["heap_mb"] = double(heapBytes() - heap0) / 1e6;
  out.v["roots"] = double(grid.rootMap().size());

  // random queries in the mapped volume: hits and misses
  std::vector<CoordT> random_q;
  {
    std::mt19937 rng(7);
    const double half_x = 0.5 * rp.scans * rp.scan_step + rp.max_range;
    std::uniform_real_distribution<double> ux(-half_x, half_x), uy(-80, 80), uz(-2, 8);
    for (int i = 0; i < 1000000; ++i) {
      random_q.push_back(grid.posToCoord(ux(rng), uy(rng), uz(rng)));
    }
  }
  double sum;
  size_t hits;
  out.v["randq_ms"] = bestQuery(grid, random_q, passes, sum, hits);
  out.v["randq_hits"] = double(hits);
  auto shuffled = endpoints;
  std::shuffle(shuffled.begin(), shuffled.end(), std::mt19937(3));
  out.v["endq_ms"] = bestQuery(grid, shuffled, passes, sum, hits);
  out.v["endq_sum"] = sum;
  double best = 1e300;
  for (int p = 0; p < passes; ++p) {
    best = std::min(best, iteratePass(grid, sum));
  }
  out.v["iter_ms"] = best;
  out.v["iter_sum"] = sum;
}

#ifdef WITH_PMAP
// ProbabilisticMap end to end: a car driving down a street lined with buildings and
// poles, a 32 beam lidar, 10 cm voxels. Everything Bonxai's own mapping code does.
static void runPmap(Out& out, double voxel, int scans) {
  using Eigen::Vector3d;
  std::vector<std::vector<Vector3d>> clouds;
  std::vector<Vector3d> origins;
  const int rings = 32, columns = 720;
  const double max_range = 40.0;
  for (int s = 0; s < scans; ++s) {
    const Vector3d o(s * 1.0, 1.5 * std::sin(s * 0.05), 1.8);
    origins.push_back(o);
    std::vector<Vector3d> cloud;
    for (int r = 0; r < rings; ++r) {
      const double pitch = (-25.0 + 40.0 * r / (rings - 1)) * M_PI / 180.0;
      for (int c = 0; c < columns; ++c) {
        const double yaw = 2.0 * M_PI * (c + 0.5 * (s % 2)) / columns;
        const Vector3d d(
            std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch));
        double t = 1e9;
        if (d.z() < -1e-9) {
          t = std::min(t, -o.z() / d.z());  // the road
        }
        // facades on both sides, set back by a few meters every 25 m
        for (double side : {-1.0, 1.0}) {
          if (d.y() * side > 1e-9) {
            const double block = std::floor((o.x() + d.x() * 10.0) / 25.0);
            const double setback = 9.0 + 3.0 * std::fmod(std::abs(block) * 7.0, 3.0);
            const double tt = (side * setback - o.y()) / d.y();
            const double zz = o.z() + d.z() * tt;
            if (zz < 15.0) {
              t = std::min(t, tt);
            }
          }
        }
        // poles every 12 m on both sidewalks
        for (double side : {-1.0, 1.0}) {
          const double py = side * 6.5;
          for (int k = -4; k <= 4; ++k) {
            const double px = std::round(o.x() / 12.0) * 12.0 + k * 12.0;
            // distance of the ray (2D) to the pole
            const double ex = px - o.x(), ey = py - o.y();
            const double dd = std::hypot(d.x(), d.y());
            if (dd < 1e-9) {
              continue;
            }
            const double proj = (ex * d.x() + ey * d.y()) / (dd * dd);
            if (proj <= 0) {
              continue;
            }
            const double cx = o.x() + d.x() * proj - px, cy = o.y() + d.y() * proj - py;
            if (cx * cx + cy * cy < 0.15 * 0.15 && o.z() + d.z() * proj < 6.0) {
              t = std::min(t, proj);
            }
          }
        }
        if (t > max_range * 1.5) {
          t = max_range * 1.5;  // sky: a ray that only clears space
        }
        cloud.push_back(o + d * t);
      }
    }
    clouds.push_back(std::move(cloud));
  }
  const size_t heap0 = heapBytes();
  ProbabilisticMap map(voxel);
  auto t0 = Clock::now();
  for (int s = 0; s < scans; ++s) {
    map.insertPointCloud(clouds[s], origins[s], max_range);
  }
  out.v["insert_ms"] = ms_since(t0);
  out.v["heap_mb"] = double(heapBytes() - heap0) / 1e6;
  out.v["roots"] = double(map.grid().rootMap().size());
  // occupancy queries: random points of the mapped street
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> ux(-20, scans + 20), uy(-25, 25), uz(-1, 12);
  std::vector<CoordT> q;
  for (int i = 0; i < 2000000; ++i) {
    q.push_back(map.grid().posToCoord(ux(rng), uy(rng), uz(rng)));
  }
  double best = 1e300;
  size_t occ = 0;
  for (int p = 0; p < 3; ++p) {
    auto t1 = Clock::now();
    occ = 0;
    for (const auto& c : q) {
      occ += map.isOccupied(c) ? 1 : 0;
    }
    best = std::min(best, ms_since(t1));
  }
  out.v["query_ms"] = best;
  out.v["occupied"] = double(occ);
  std::vector<CoordT> occupied;
  auto t2 = Clock::now();
  map.getOccupiedVoxels(occupied);
  out.v["getocc_ms"] = ms_since(t2);
}
#endif

//--------------------------------------------------------------- map level ---
#ifndef REAL
using Inner = Grid<std::shared_ptr<Grid<float>>>;
using MapW = BONXAI_DEFAULT_POLICY::template Map<Inner>;
// an InnerGrid of 8 cells: the same object as in a VoxelGrid, a smaller array
constexpr uint32_t kBits = 1;

static inline uint64_t touch(const Inner* in) {
  return in->mask().getWord(0) + in->size();
}

static void runMapOps(Out& out, const std::string& dist, size_t n) {
  const auto keys = rootKeys(dist, n, false);
  const auto miss = rootKeys(dist, n, true);
  const size_t nq = std::max<size_t>(n, 2000000);
  const auto hitq = sampleQueries(keys, nq, 11);
  const auto missq = sampleQueries(miss, nq, 12);
  std::vector<CoordT> mixq;
  {
    std::mt19937 rng(13);
    for (size_t i = 0; i < nq; ++i) {
      mixq.push_back((rng() & 1) ? hitq[i] : missq[i]);
    }
  }
  const int reps = std::clamp(int(2000000 / n), 1, 100);

  auto build = [&](bool reserve) {
    auto m = std::make_unique<MapW>();
    if (reserve) {
      m->reserve(n);
    }
    for (const auto& k : keys) {
      m->emplace(k, kBits);
    }
    return m;
  };
  for (bool reserve : {false, true}) {
    double best = 1e300;
    for (int r = 0; r < reps; ++r) {
      auto t0 = Clock::now();
      auto m = build(reserve);
      const double t = ms_since(t0);
      best = std::min(best, t);
      if (r + 1 == reps) {
        const size_t heap0 = heapBytes();
        m.reset();
        const size_t heap1 = heapBytes();
        // bytes per key, the Inner and its array included
        out.v[reserve ? "mem_reserved_B" : "mem_B"] = double(heap0 - heap1) / n;
      }
    }
    out.v[reserve ? "insert_reserved_ns" : "insert_ns"] = best * 1e6 / n;
  }

  auto m = build(false);
  out.v["buckets"] = double(m->buckets());
  auto lookups = [&](const std::vector<CoordT>& q, const char* name) {
    double best = 1e300;
    uint64_t sum = 0;
    size_t found = 0;
    for (int p = 0; p < 3; ++p) {
      auto t0 = Clock::now();
      sum = 0;
      found = 0;
      for (const auto& k : q) {
        if (const Inner* in = m->find(k)) {
          sum += touch(in);
          ++found;
        }
      }
      best = std::min(best, ms_since(t0));
    }
    out.v[std::string(name) + "_ns"] = best * 1e6 / q.size();
    out.v[std::string(name) + "_found"] = double(found) / q.size();
    out.v[std::string(name) + "_sum"] = double(sum);
  };
  lookups(hitq, "hit");
  lookups(missq, "miss");
  lookups(mixq, "mix");

  {
    // iterate
    const int iters = std::clamp(int(2000000 / n), 1, 200);
    double best = 1e300;
    uint64_t sum = 0;
    for (int p = 0; p < 3; ++p) {
      auto t0 = Clock::now();
      for (int i = 0; i < iters; ++i) {
        m->forEach([&](const CoordT& k, const Inner& in) { sum += touch(&in) + k.x; });
      }
      best = std::min(best, ms_since(t0) / iters);
    }
    out.v["iter_ns"] = best * 1e6 / n;
    out.v["iter_sum"] = double(sum);
  }
  {
    // erase one key, insert another: the size stays the same
    const size_t ops = std::min<size_t>(n, 500000);
    const int r2 = std::clamp(int(500000 / ops), 1, 50);
    double best = 1e300;
    for (int r = 0; r < r2; ++r) {
      auto mm = build(false);
      auto t0 = Clock::now();
      for (size_t i = 0; i < ops; ++i) {
        mm->erase(keys[i]);
        mm->emplace(miss[i], kBits);
      }
      best = std::min(best, ms_since(t0));
      if (mm->size() != n) {
        out.v["errors"] = 1;
      }
    }
    out.v["churn_ns"] = best * 1e6 / ops;
  }
  {
    double best = 1e300;
    for (int r = 0; r < reps; ++r) {
      auto mm = build(false);
      auto t0 = Clock::now();
      mm->clear();
      best = std::min(best, ms_since(t0));
    }
    out.v["clear_ns"] = best * 1e6 / n;
  }
}

// build, then hit and miss lookups, at sizes spread over whole doublings: each map grows
// at its own thresholds, so any single size favours some of them
static void runSweep(Out& out, const std::string& dist) {
  for (size_t n :
       {10000, 20000, 35000, 50000, 70000, 100000, 140000, 200000, 250000, 280000, 360000, 500000,
        700000, 1000000}) {
    const auto keys = rootKeys(dist, n, false);
    const auto miss = rootKeys(dist, n, true);
    const size_t nq = 2000000;
    const auto hitq = sampleQueries(keys, nq, 11);
    const auto missq = sampleQueries(miss, nq, 12);
    const std::string tag = std::to_string(n / 1000) + "k";
    auto t0 = Clock::now();
    auto m = std::make_unique<MapW>();
    for (const auto& k : keys) {
      m->emplace(k, kBits);
    }
    out.v["insert_" + tag + "_ns"] = ms_since(t0) * 1e6 / n;
    for (int which = 0; which < 2; ++which) {
      const auto& q = which ? missq : hitq;
      double best = 1e300;
      uint64_t sum = 0;
      for (int p = 0; p < 2; ++p) {
        auto t1 = Clock::now();
        sum = 0;
        for (const auto& k : q) {
          if (const Inner* in = m->find(k)) {
            sum += touch(in);
          }
        }
        best = std::min(best, ms_since(t1));
      }
      out.v[(which ? "miss_" : "hit_") + tag + "_ns"] = best * 1e6 / nq;
      out.v[(which ? "misssum_" : "hitsum_") + tag] = double(sum);
    }
  }
}
#endif

//---------------------------------------------------------------- selftest ---
// random operations against a reference, with several accessors alive across growth
static void runSelfTest(Out& out) {
  std::mt19937 rng(123);
  std::map<std::tuple<int, int, int>, float> ref;
  VGrid grid(0.05);
  auto a1 = grid.createAccessor();
  auto a2 = grid.createAccessor();
  auto c1 = grid.createConstAccessor();
  std::uniform_int_distribution<int> small(-3000, 3000), pick(0, 99);
  size_t errors = 0;
  for (int round = 0; round < 4; ++round) {
    for (int i = 0; i < 200000; ++i) {
      const CoordT c{small(rng), small(rng), small(rng) / 10};
      const int op = pick(rng);
      auto key = std::make_tuple(c.x, c.y, c.z);
      if (op < 40) {
        const float v = float(i);
        (op & 1 ? a1 : a2).setValue(c, v);
        ref[key] = v;
      } else if (op < 50) {
        (op & 1 ? a1 : a2).setCellOff(c);
        ref.erase(key);
      } else {
        const float* v = (op & 1) ? c1.value(c) : a1.value(c);
        auto it = ref.find(key);
        if ((v == nullptr) != (it == ref.end()) || (v && *v != it->second)) {
          ++errors;
        }
      }
      // re-read an old key through the other accessor: catches stale cached pointers
      if (!ref.empty() && (i % 97) == 0) {
        auto it = ref.begin();
        std::advance(it, rng() % std::min<size_t>(ref.size(), 50));
        const auto [x, y, z] = it->first;
        const float* v = c1.value({x, y, z});
        if (!v || *v != it->second) {
          ++errors;
        }
      }
    }
    // switch off a third of the cells, so that whole roots become empty
    for (auto it = ref.begin(); it != ref.end();) {
      if (rng() % 3 == 0) {
        const auto [x, y, z] = it->first;
        a2.setCellOff({x, y, z});
        it = ref.erase(it);
      } else {
        ++it;
      }
    }
    const size_t roots_before = grid.rootMap().size();
    grid.releaseUnusedMemory();
    out.v["erased_roots_" + std::to_string(round)] = double(roots_before - grid.rootMap().size());
    {
      std::set<std::tuple<int, int, int>> root_keys;
      for (auto& [k, val] : ref) {
        const auto [x, y, z] = k;
        const CoordT rk = grid.getRootKey({x, y, z});
        root_keys.insert({rk.x, rk.y, rk.z});
      }
      if (root_keys.size() != grid.rootMap().size()) {
        ++errors;
      }
    }
    for (auto& [k, val] : ref) {
      const auto [x, y, z] = k;
      const float* v = c1.value({x, y, z});
      if (!v || *v != val) {
        ++errors;
      }
    }
    size_t count = 0;
    grid.forEachCell([&](const float&, const CoordT&) { ++count; });
    if (count != ref.size()) {
      ++errors;
    }
  }
  grid.clear(CLEAR_MEMORY);
  if (grid.rootMap().size() != 0 || a1.value({1, 2, 3}) != nullptr) {
    ++errors;
  }
#ifndef REAL
  {
    // the map alone: random operations against std::map, every distribution
    for (const char* dist : {"rand", "dense", "plane", "line", "stride"}) {
      const auto keys = rootKeys(dist, 20000, false);
      const auto miss = rootKeys(dist, 20000, true);
      MapW m;
      std::map<std::tuple<int, int, int>, const Inner*> r;
      std::mt19937 g(7);
      for (int i = 0; i < 200000; ++i) {
        const auto& k = (g() & 1) ? keys[g() % keys.size()] : miss[g() % miss.size()];
        const auto t = std::make_tuple(k.x, k.y, k.z);
        const int op = g() % 10;
        if (op < 5) {
          if (!m.find(k)) {
            r[t] = m.emplace(k, kBits);
          }
        } else if (op < 7) {
          m.erase(k);
          r.erase(t);
        } else {
          const Inner* in = m.find(k);
          auto it = r.find(t);
          // node maps: the pointer must not have changed since insertion
          if ((in == nullptr) != (it == r.end()) || (in && MapW::kStable && in != it->second)) {
            ++errors;
          }
        }
        if (m.size() != r.size()) {
          ++errors;
        }
      }
      size_t cnt = 0;
      m.forEach(
          [&](const CoordT& k, const Inner&) { cnt += r.count(std::make_tuple(k.x, k.y, k.z)); });
      if (cnt != r.size()) {
        ++errors;
      }
      m.clear();
      if (m.size() != 0 || m.find(keys[0])) {
        ++errors;
      }
    }
  }
#endif
  out.v["errors"] = double(errors);
  out.v["cells"] = double(ref.size());
}

//-------------------------------------------------------------------- main ---
int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: bench <workload>\n");
    return 2;
  }
  const std::string w = argv[1];
  const bool default_malloc = w.size() > 8 && w.substr(w.size() - 8) == "_default";
  if (!default_malloc) {
    // keep freed memory mapped: page faults are very expensive on this VM, and would
    // otherwise measure the kernel rather than the map. M_MMAP_THRESHOLD alone cannot do
    // it: glibc caps it at 32 MB, and the arrays of flat maps above that went back to the
    // kernel between builds, to be faulted in again
    mallopt(M_MMAP_MAX, 0);
    mallopt(M_TRIM_THRESHOLD, 1 << 30);
    mallopt(M_MMAP_THRESHOLD, 32 << 20);
    mallopt(M_TOP_PAD, 64 << 20);
  }
  Out out;
  auto startsWith = [&](const char* p) { return w.rfind(p, 0) == 0; };
  if (w == "selftest") {
    runSelfTest(out);
  } else if (w == "wide") {
    runWide(out, 300000, 100.0, 0.0, 3);
  } else if (w == "widepos") {
    runWide(out, 300000, 100.0, 100.0, 3);
  } else if (startsWith("growth")) {
    // growth<N>k[_default]
    const size_t k = std::stoul(w.substr(6));
    runGrowth(out, k * 1000, k >= 1000 ? 200.0 : 100.0);
  } else if (w == "wide1m") {
    runWide(out, 1000000, 200.0, 0.0, 2);
  } else if (w == "wide_default") {
    runWide(out, 300000, 100.0, 0.0, 1);
  } else if (w == "room") {
    runRoom(out, 5);
  } else if (w == "roomcreate_default") {
    runRoomCreate(out);
  } else if (w == "rays10") {
    RayParams rp;
    rp.voxel = 0.1;
    rp.scans = 80;
    runRays(out, rp, 3);
  } else if (startsWith("vg_")) {
    // vg_<dist>_<roots>
    const auto a = w.find('_', 3);
    runDist(out, w.substr(3, a - 3), std::stoul(w.substr(a + 1)), 3);
#ifdef WITH_PMAP
  } else if (w == "pmap") {
    runPmap(out, 0.1, 60);
  } else if (w == "pmap5") {
    runPmap(out, 0.05, 25);
#endif
#ifndef REAL
  } else if (startsWith("sweep_")) {
    runSweep(out, w.substr(6));
  } else if (startsWith("map_")) {
    // map_<dist>_<n>
    const auto a = w.find('_', 4);
    runMapOps(out, w.substr(4, a - 4), std::stoul(w.substr(a + 1)));
#endif
  } else {
    fprintf(stderr, "unknown workload %s\n", w.c_str());
    return 2;
  }
  out.print(VARIANT_NAME, w);
  return 0;
}
