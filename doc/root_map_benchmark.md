# The root map: what was tested and learned

`VoxelGrid` keeps its roots in a hash map, one root per 32 voxels per side by default (1.6 m
at 5 cm). A room needs a few hundred, an outdoor map hundreds of thousands, and every lookup
that leaves an accessor's cached leaf goes through that map. PR #70 replaced
`std::unordered_map` with `CoordMap` (`bonxai/coord_map.hpp`) and fixed `std::hash<CoordT>`.

## What changed

- **`std::hash<CoordT>`**: #69 dropped its truncation to 20 bits, which made keys collide
  past ~100k roots. It also sign-extended the coordinates, so grids crossing zero
  collided (the 46656 root keys of a 36³ grid centred on the origin gave 29226 hashes), and
  OpenVDB's formula left the low bits of root keys at zero. It now casts to `uint32_t`,
  multiplies each coordinate by its own constant and mixes with murmur3's finalizer.
  `VoxelGrid` no longer uses it; it stays for users' containers.
- **`CoordMap<ValueT>`**, 388 lines, standard library only:
  - open addressing with linear probing, at most half full, over 16-byte buckets
    `{upper 32 bits of the hash, position, pointer to the value}`: a hit goes from its bucket
    straight to the value, a miss is almost always rejected by the tag;
  - each value is allocated on its own, next to its InnerGrid's data, and never moves: the
    accessors cache pointers to InnerGrids, as with `std::unordered_map`;
  - iteration follows a vector of pointers in insertion order; `clear()` frees last to first.
- **Accessors**: the slow paths take three integers; the mutable one is never inlined; keys
  are copied into the cache field by field (see *Lessons*).

## How it was chosen

1. A first comparison against std, boost, absl and unordered_dense on 5 workloads favoured
   CoordMap. Asked whether that was cherry-picked, it was redone on a VM: 98 configurations
   of 20 header-only maps (hashtable-bench's, plus boost, absl, phmap, fph), each holding
   the InnerGrids every way it can, on 21 workloads, with a decision rule written before
   the results: switch to a library scoring ≤ 0.95 of CoordMap (geometric mean of every
   time) unless it is more than 5% slower end to end, and the code built into Bonxai must
   confirm the harness.
2. On the VM the rule picked unordered_dense's `map`. CoordMap was kept at first because
   that map took 2.7× as long to build 1M roots, which turned out to be the benchmark's own
   page faults (see *Lessons*); retracted, the branch switched to unordered_dense. But the
   VM's noise (one binary, three runs 10–20% apart) was as large as the differences.
3. The finalists ran on bare metal: an i7-13700H, turbo off, GCC 15, each run a process of
   its own pinned to a performance core, variants in a random order, two runs of 8 rounds.
   In the harness every contender scored 0.74–0.80 of CoordMap. Built into Bonxai,
   unordered_dense's `map` was 1.06–1.08 end to end and its `segmented_map` 1.08: the rule
   keeps CoordMap. With unordered_dense, the queries of the ray casting and probabilistic
   map workloads took 6–28% longer in 46 of 48 paired runs.

## Results against `main` (bare metal, medians, ms)

| | `main` | this | ratio |
|---|---|---|---|
| 278k roots: build | 181 | 91.8 | 0.51 |
| 278k roots: query a cell that is there | 41.6 | 27.5 | 0.66 |
| 278k roots: query a cell that is not | 29.8 | 8.15 | 0.27 |
| 278k roots: `forEachCell` / `clear()` | 45.7 / 150 | 26.7 / 60.3 | 0.58 / 0.40 |
| 1M roots: build | 710 | 326 | 0.46 |
| 1M roots: query a cell that is there / is not | 149 / 95.1 | 92.2 / 40.8 | 0.62 / 0.43 |
| 970k roots: worst single insertion | 93.9 | 11.2 | 0.12 |
| room scan: shuffled queries | 1.91 | 1.31 | 0.69 |
| rays: cast / random queries / endpoint queries | 2090 / 90.1 / 47.2 | 2002 / 46.7 / 33.6 | 0.96 / 0.52 / 0.71 |
| probabilistic map: insert clouds / query | 4430 / 167 | 4073 / 120 | 0.92 / 0.72 |
| memory, 278k roots (MB) | 999 | 1013 | 1.01 |

Over the 21 workloads: 0.56 of `main`'s time, 0.75 end to end. The alternatives, relative to
`main` the same way:

| | overall | end to end | notes |
|---|---|---|---|
| CoordMap (this) | 0.56 | **0.75** | best at finding cells that are there |
| unordered_dense `map` | **0.50** | 0.81 | misses 1.1–2.7× faster than CoordMap, builds 5–15% faster |
| unordered_dense `segmented_map` | 0.52 | 0.81 | hits slightly worse than `map` |
| boost `unordered_flat_map` (harness estimate) | 0.50 | 0.75 | iterates and clears 2× slower; worst insertion 21 ms (96 on a first build) |

## Against NanoVDB

`benchmark_nanovdb`, same integer coordinates, NanoVDB 13.1.0 (`tools::build::Grid`; the
read-only `NanoGrid` in brackets), each benchmark in its own process, medians of 5, ms:

| | `main` | this | NanoVDB |
|---|---|---|---|
| room: create | 1.12 | **1.05** | 4.97 |
| room: update | 0.416 | **0.387** | 0.609 |
| room: query in scan order | 0.456 | 0.415 | **0.317** (0.323) |
| room: query shuffled | 1.90 | **1.29** | 1.48 (1.34) |
| room: iterate | 0.265 | **0.237** | (0.371) |
| wide: create | 423 | **162** | 1041 |
| wide: query a cell that is there | 42.1 | 26.7 | **22.7** |
| wide: query a cell that is not | 29.5 | **8.41** | 14.5 |

Bonxai builds and updates grids 1.6–6× faster and answers misses 1.7× faster. NanoVDB
stays 1.2–1.3× faster on hits: its top node covers 4096 voxels per side, so its root table
stays in cache, where the wide map needs 278k Bonxai roots; and its accessor caches every
level. `StaticShape<4, 3>` (16× fewer roots, 64 KB inner grids) was slower, not faster.

## Lessons

- **A `CoordT` must not go through memory whole.** Computed field by field, it is stored
  as three 32-bit writes; copied or passed by value, x and y are then read back with one
  64-bit load, which store forwarding cannot serve: each lookup waits for the previous one
  to retire. It cost 1.2–3.5× in five places, including `std::hash<CoordT>` packing x and y
  (2.3× on updates through a `std::unordered_map`). Hence three-integer slow paths, a
  field-by-field `cacheKey()`, and a hash reading each coordinate on its own.
- **Every instruction on the lookup counts.** Lookups on a large map are latency bound and
  overlap; 20 extra instructions (an empty-map check, a mask computed per lookup) made them
  25% slower. An empty map points at a static empty bucket instead.
- **Keep the slow path out of the fast one.** With creation and allocation inlined into
  `setValue()`, it stopped being inlined into callers' loops: scan-order updates got 1.7×
  slower. `findOrCreateLeaf` is `BONXAI_NOINLINE`; the read-only `findLeaf` stays inline.
- **Free large structures last to first.** Freed front to back, glibc merges the blocks
  into the top of the heap and hands it back to the kernel; the next grid faults it all in
  again (3.5× slower to rebuild a room; 2× to create the wide grid with unordered_dense).
- **Cache-line alignment of nodes gained nothing** and made building 1M roots 25% slower.
- **Benchmarking traps, each of which changed a conclusion:**
  - `mallopt(M_MMAP_THRESHOLD)` is capped at 32 MB: larger arrays still go back to the
    kernel between builds. `M_MMAP_MAX = 0` keeps them. This produced the retracted 2.7×.
  - A VM's noise (10–20%) exceeded the differences being decided (5–12%).
  - In one process, the allocator's state left by a previous benchmark moved creation
    timings by up to 2.5×: run each benchmark in its own process.
  - Runners on 4 cores sharing the L3 changed memory-bound results, not only their noise
    (random hits went from 23% faster with CoordMap to 6% slower); 2 cores were fine.
  - Averages hide trade-offs: unordered_dense led every summary score yet lost end to end.
    Measuring the code built into Bonxai, not only a harness, settled it.

## Limits

- Misses cost CoordMap 1.1–2.7× what unordered_dense's maps take (the most on large maps
  of scattered roots).
- `rootMap()` offers the part of `std::unordered_map` that Bonxai uses (`begin`, `end`,
  `find`, `try_emplace`, `insert`, `erase(key)`, `clear`, `size`, `empty`, `swap`,
  `reserve`); `at`, `count`, `operator[]`, `emplace`, erase by iterator and the bucket
  interface are gone. Inserting or erasing invalidates every iterator, and erasing moves
  the last element into the hole.
- Erasing, through `rootMap()`, a root that an accessor cached leaves that accessor dangling:
  create accessors again afterwards (#71).

The benchmark harness, the decision rule and the raw results of every run are in PR #70's
history (`git show 95540fc:doc/root_map_study/README.md`), not in the repository.
