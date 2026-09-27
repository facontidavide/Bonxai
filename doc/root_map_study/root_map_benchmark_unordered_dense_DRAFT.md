> **Draft**, for the case where unordered_dense's map stays the root map after the bare
> metal run (see `README.md`). Its numbers come from the VM; the sections marked `??` wait
> for that run.

# The root map

`VoxelGrid` keeps its root nodes in a hash map, one root per 2^(inner_bits + leaf_bits)
voxels per side: 32 by default, 1.6 m at 5 cm. A room needs a few hundred of them, a
building or an outdoor map hundreds of thousands, and every lookup that leaves the
accessor's cached inner node goes through that map.

This records why the map is now Martin Ankerl's
[unordered_dense](https://github.com/martinus/unordered_dense) `map`, vendored in
`bonxai/detail`, rather than a `std::unordered_map`; how it is wired into the accessors;
and how it was chosen, against 20 hash maps on 21 workloads. That choice went through a
wrong turn worth keeping on record: a map written for Bonxai, `CoordMap`, was first kept
against the rule written down to decide, on the strength of a number that turned out to
be an artifact of the benchmark. *How it was chosen* tells that part.

## What was wrong

**The hash.** `std::hash<CoordT>` was OpenVDB's `x * 73856093 ^ y * 19349669 ^ z * 83492791`,
truncated to 20 bits until #69. It also sign extended the coordinates, which made it
collide on any grid that crosses zero: the 46656 root keys of a 36³ grid centred on the
origin gave only 29226 distinct values. Martin Ankerl found this one. A robot's map
nearly always crosses zero, since the origin is usually where the robot started.

**The container.** A lookup in libstdc++'s `std::unordered_map` divides the hash by the
prime number of buckets, reads the bucket, then the node *before* the first node of that
bucket, and only then the node itself: three dependent memory accesses, each one a cache
miss on a large map. A key that is missing walks its bucket's chain too. Iterating or
clearing the map follows the chain of nodes, which is in no particular order in memory.

Fixing the hash alone changes little: the container costs far more than the collisions.

## The root map now

`VoxelGrid::RootMap` is `Bonxai::unordered_dense::map<CoordT, InnerGrid, RootKeyHash>`,
unordered_dense 5.1.0, MIT licence.

- **Index**: groups of 16 buckets, each group 16 one byte fingerprints, 8 overflow
  counters and 16 positions in the array of values, 88 bytes. A lookup compares the 16
  fingerprints of the key's group at once, then reads the value whose fingerprint matches;
  a key that is not there almost never leaves its group. At most 80% of the buckets are
  used.
- **Values**: one `std::vector<std::pair<CoordT, InnerGrid>>`, in insertion order, 120
  bytes per root. Each `InnerGrid` still allocates its own array of pointers to leaves,
  and the leaves never move.
- **What moves**: growing the vector moves every `InnerGrid` to the new array, and erasing
  a root moves the last one into its place. The accessors cope with both (*The accessors*
  below). Code that walks `rootMap()` or calls `rootMap().find(key)->second` works
  unchanged, but a reference to an `InnerGrid` is valid only until the next insertion or
  erasure. With `std::unordered_map`, it stayed valid until its own root was erased.
- **Hash**: `RootKeyHash` multiplies each coordinate by its own odd constant and xors the
  products; unordered_dense mixes the result further.
- **Vendoring**: `bonxai/detail/unordered_dense.h` and `stl.h` are the upstream headers
  with the namespace and the macros renamed, so that they cannot clash with another copy
  of the library in the same program. `bonxai/detail/update_unordered_dense.sh` does the
  renaming from a checkout of the upstream repository, for the next update.

`std::hash<CoordT>` is no longer used by Bonxai, but stays for the containers of the users.
It now multiplies each coordinate by its own constant and mixes the result with murmur3's
finalizer: every bit of it is good, where OpenVDB's left the low bits of root keys at zero,
which ruins any table with a power of two buckets.

## The code

| File | What changed |
|---|---|
| `bonxai_core/include/bonxai/detail/unordered_dense.h`, `stl.h` | New: unordered_dense 5.1.0, renamed into `Bonxai::unordered_dense`. |
| `bonxai_core/include/bonxai/detail/update_unordered_dense.sh` | New: the renaming, from an upstream checkout. |
| `bonxai_core/include/bonxai/bonxai.hpp` | `VoxelGrid::RootMap` is unordered_dense's map. The accessors check the root they cached before using it. Their slow paths take three integers, and the mutable one is never inlined. They copy keys into their cache field by field. `memUsage()` counts the index in bytes. `clear()` destroys the roots last to first. |
| `bonxai_core/include/bonxai/grid_coord.hpp` | `std::hash<CoordT>`: no truncation, no sign extension, every bit mixed, each coordinate read on its own. |
| `bonxai_core/test/voxel_grid_test.cpp` | The accessors survive the growth of the map, changes made through `rootMap()`, and the release of many roots. The test of `std::hash<CoordT>` covers grids that cross zero, and its low bits. |
| `bonxai_core/benchmark/benchmark_nanovdb.cpp` | The wide queries also measure cells that are not in the grid. |

### The accessors

Each accessor caches the key and the pointer of the last leaf it used, and the key and the
place of the last root. Their fast path is the same as on `main`: `value()`, `setValue()`,
`isCellOn()` and the others compare the inner key of the coordinates with the cached one,
and use the cached leaf when it matches. Leaves never move, so that pointer stays valid
until the grid releases leaves, which `releaseUnusedMemory()` and `clear()` announce by
bumping the grid's `cache_epoch_`, as on `main`.

When the coordinates leave the cached leaf, the accessor goes through the root it cached,
and that one may have moved. `cachedRoot()` uses it only if the array of the map is the one
the accessor saw, the cached position is still inside it, and the key there is still the
cached one; otherwise the accessor looks the root up again. This catches every way the map
can move a root: its growth, an erasure moving the last root, another accessor, or code
going through `rootMap()` directly, which no epoch would see. It costs three comparisons,
on the slow path only.

- `ConstAccessor::findLeaf(x, y, z)`: the root, through `cachedRoot()` or a lookup, then
  the leaf in the `InnerGrid`. It is inline: random queries are 10 to 20% faster with it
  inlined.
- `Accessor::findOrCreateLeaf(x, y, z, create_if_missing)`: the same, creating the root
  and the leaf when they are missing. It is never inlined (`BONXAI_NOINLINE`): it carries
  the allocations, which would otherwise make `setValue()` too big to be inlined into the
  loops that call it.

Both take three integers rather than a `const CoordT&`, so that the coordinates reach
them in registers, and both update the cache with `cacheKey()`, which copies a key field
by field. The reasons are in *What bit along the way*.

**`memUsage()`.** `main` summed, over the buckets of the unordered map, 1 for an empty bucket
and its number of nodes for the others: a count, not bytes. It now adds the bytes of the
index, the keys, and the spare capacity of the array of values. The `InnerGrid`s are
counted, as before, by `InnerGrid::memUsage()`.

**`clear()`** destroys the roots last to first. Front to back frees their arrays in
increasing addresses: glibc merges them into the top of the heap and hands it back to the
kernel, and the next grid faults it in again.

### `std::hash<CoordT>`

Four commits changed it:

1. #69 dropped its truncation to 20 bits.
2. It then cast the coordinates to `uint32_t` before multiplying them. Sign extended, the
   negative ones filled the upper bits with ones, and grids crossing zero collided.
3. It became murmur3's finalizer over x and y packed in one word and z multiplied by the
   golden ratio: the low bits of OpenVDB's hash were zero on root keys, whose coordinates
   are multiples of 32.
4. It now multiplies each coordinate by its own constant before murmur3's finalizer. Packed
   in one word, x and y were read with one 64 bits load, the trap of *What bit along the
   way*: updating 300k random cells through a `std::unordered_map<CoordT, ...>` took 2.3
   times as long. GCC 13 and Clang 18 now read the coordinates with three 32 bits loads,
   at every optimisation level.

### Tests

In `voxel_grid_test.cpp`:

- `VoxelGridRootMap.AccessorsSurviveGrowth`: a reader and a writer cache a root, 10k more
  roots make the map move it, then both go through that root to another of its leaves.
- `VoxelGridRootMap.AccessorsSurviveChangesThroughRootMap`: a reader caches the last root;
  `rootMap().erase()` of another root moves it, `rootMap().try_emplace()` puts a new root
  where it was; the reader must see each change.
- `VoxelGridRootMap.AccessorsSurviveReleasingManyRoots`: a reader caches the last of 100
  roots, `releaseUnusedMemory()` releases half of them, and the reader must find every
  remaining cell and none of the released ones.
- `CoordHash.DoesNotCollapseOnLargeGrids`: `std::hash<CoordT>` on three grids of root keys,
  two of them crossing zero, and on its low 20 bits alone.

Without the check of `cachedRoot()`, the first two read freed memory, which the address
sanitizer reports. CI runs every test with and without the address and undefined behaviour
sanitizers.

`benchmark_nanovdb`'s wide queries take an argument: `0` queries the cells of the grid, `1`
other random cells in the same volume, nearly all of them in roots that do not exist.

## How it was chosen

In three steps, the second of them wrong.

1. **A first comparison**, against `std::unordered_map`, `std::map`, and the maps of
   boost, absl and unordered_dense on five workloads, made a map written for Bonxai,
   `CoordMap`, look best: open addressing over 16 bytes buckets pointing to nodes that
   never move.
2. **Asked whether that was a special case**, the comparison was redone with every header
   only map of [hashtable-bench](https://github.com/renzibei/hashtable-bench) and a few
   more, 98 configurations of 20 maps, on 21 workloads. unordered_dense led every average,
   and the rule written down before the final results said to replace `CoordMap` with its
   flat `map`. `CoordMap` was kept anyway: built into Bonxai, the flat map had taken 2.7
   times as long to build a map of 1M roots.
3. **That number was an artifact** of the benchmark, found when asked whether the
   preference for `CoordMap` was biased. The harness capped glibc's mmap threshold at
   32 MB, so the flat map's arrays of 63 and 126 MB went back to the kernel between two
   builds and were faulted in again, and a page fault on the machine of these tests costs
   from 2 to 30 µs, depending on what its host did with the memory since: the same build
   took 1620 ms or 642 ms depending on the order of the runs. With the harness fixed, the
   contenders were run again, and the rule was applied as written, with no exception. It
   picks unordered_dense's `map`.

### The candidates

| Map | Version | Licence |
|---|---|---|
| `std::unordered_map` | libstdc++ 13 | |
| `boost::unordered_map`, `unordered_node_map`, `unordered_flat_map` | 1.83 and 1.92 | BSL |
| `absl::node_hash_map`, `flat_hash_map` | 20220623 and 20260817 | Apache 2.0 |
| `phmap::node_hash_map`, `flat_hash_map` | 2.0 | Apache 2.0 |
| `ankerl::unordered_dense::map`, `segmented_map` | 5.1.0 | MIT |
| `tsl::robin_map`, `hopscotch_map`, `sparse_map` | 1.4.1, 2.4.0, 0.7.0 | MIT |
| `ska::flat_hash_map`, `bytell_hash_map` | 2018 | BSL |
| `emhash5`, `6`, `7`, `8` | 1.1.0 | MIT |
| `robin_hood::unordered_node_map`, `unordered_flat_map` | 3.11 | MIT |
| `fph::DynamicFphMap`, `MetaFphMap` | 2026 | Apache 2.0 |
| `CoordMap` | written for Bonxai | |

Each map held the `InnerGrid`s every way it allows: in its nodes; by `std::unique_ptr`;
by a raw pointer; or in the map itself, with the accessors dropping their cache whenever
the map moved its values. Each had every hash it can work with. 98 configurations in all.

### How

A copy of `VoxelGrid` took its root map as a template parameter and ran the same accessor
code with every map. Each configuration was a binary of its own, built by GCC 13 at `-O3`,
and every run a process of its own, pinned to one core, in a different order every round.
A pilot of 4 workloads picked each library's best configuration; the final suite ran them
on 21 workloads. The two leading ways of using unordered_dense were then built into Bonxai
itself.

After the fix, only the contenders ran again, 3 rounds, 5 for the end to end workloads:
the other maps were beaten on every summary by one of them, or were at least 14% slower
than `CoordMap` on all of them. The contenders: unordered_dense's `map` holding the
`InnerGrid`s, by `unique_ptr` or by raw pointer; its `segmented_map`; boost's
`unordered_flat_map` and `unordered_node_map`; `CoordMap`; and, built into Bonxai, both
unordered_dense maps and `CoordMap`.

### The workloads

| Category | Workloads |
|---|---|
| large maps | 300k cells at random in a cube of 200 m centred on the origin, 278k roots (`benchmark_nanovdb`'s); the same shifted to positive coordinates; 1M cells in 400 m, 970k roots; one cell in each of 250k random roots |
| key patterns, 250k roots | a dense cube; a rolling terrain, one layer of roots over 800 × 800 m; a corridor 100 km long; random roots whose coordinates are multiples of 16 |
| small maps | 1k and 10k random roots; `benchmark_nanovdb`'s room scan, 354 roots |
| the map alone | insertion with and without `reserve()`, hits, misses, 50% hits, erase and insert, iteration and clear: 1k, 100k and 1M random keys, and 100k keys of a dense cube |
| end to end | lidar-like rays cast at 10 cm along 800 m, then random queries and queries of the endpoints; `ProbabilisticMap::insertPointCloud()` of 60 scans of a street at 10 cm and of 25 at 5 cm, then `isOccupied()` at random points |
| allocator | creating and destroying the room's grid, and the wide map, with glibc's default settings |
| size | insertion, hits and misses at 14 sizes, 10k to 1M roots |

Every operation is timed: building, a query of a cell that is there (a *hit*) or not (a
*miss*), updating every cell, `forEachCell` and `clear()`. The wide workloads query the
cells in the order they were inserted, as `benchmark_nanovdb` does, the others in a random
order. Every run but the allocator ones keeps the memory it frees
(`mallopt(M_MMAP_MAX, 0)` and a trim threshold of 1 GB), so that it measures the maps
rather than the kernel.

### Traps

- **The key through memory.** The first wrappers passed the maps a reference to a key that
  GCC had written as three 32 bits stores, and the maps whose `find()` is not inlined read
  it back with one 64 bits load: 13 configurations updated cells 2.2 to 2.8 times slower
  than they can. See *What bit along the way*.
- **The mmap cap.** `mallopt(M_MMAP_THRESHOLD)` cannot go above 32 MB: every allocation
  above it goes back to the kernel when freed, whatever the other settings. Only the flat
  maps that hold the `InnerGrid`s have arrays that large, at 1M roots: their builds were
  charged for the page faults, up to 2.7 times `CoordMap`'s time. This one decided the
  second step above.
- **Page faults vary.** The same page fault costs 2 µs or 30 µs on this machine, depending
  on what its host did with the memory. The first build of a process, which faults in
  everything, and the allocator category measure it as much as the maps: the same
  allocator run of the same binary went from 354 to 192 ms from one run to the next.
- **Sizes.** Every map grows at its own thresholds, so any single size favours some of
  them; the sweep of 14 sizes covers whole doublings.
- **Two binaries of the same code** differ by up to 15% on a single time, 1 to 2% on the
  summaries: code layout.

### Results

Relative to `CoordMap`, lower is better. *Every time* is the geometric mean of all the
times at once, the score that the rule names; 72 of the harness's 126 times are the map
alone and the size sweep. *Per category* is the geometric mean of the categories, each the
geometric mean of its times. *Total time* is the geometric mean over the workloads of the
sum of all the times of each, which weighs every operation by what it costs.

The harness, after the fix:

| | large maps | key patterns | small maps | map alone | end to end | allocator | size | **per category** | **total time** | every time |
|---|---|---|---|---|---|---|---|---|---|---|
| unordered_dense `map`, `InnerGrid`s in the map | 0.90 | 0.82 | 0.88 | 0.65 | 1.09 | 0.77 | 0.77 | **0.83** | 0.89 | 0.79 |
| unordered_dense `segmented_map` | 0.99 | 0.88 | 0.91 | 0.71 | 1.11 | 0.90 | 0.72 | **0.88** | 0.91 | 0.81 |
| unordered_dense `map`, `unique_ptr` | 1.00 | 0.84 | 0.87 | 0.79 | 1.10 | 0.80 | 0.81 | **0.88** | 0.93 | 0.85 |
| unordered_dense `map`, raw pointer | 1.01 | 0.83 | 0.86 | 0.81 | 1.09 | 0.83 | 0.80 | **0.89** | 0.93 | 0.86 |
| boost `unordered_flat_map`, in the map | 1.07 | 0.74 | 0.87 | 0.92 | 1.02 | 0.98 | 0.80 | **0.91** | 1.03 | 0.88 |
| boost `unordered_node_map` | 1.24 | 0.79 | 0.89 | 0.99 | 1.05 | 1.07 | 0.77 | **0.96** | 1.05 | 0.92 |
| `CoordMap` | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 |

Built into Bonxai:

| | large maps | key patterns | small maps | end to end | allocator | **per category** | **total time** | every time |
|---|---|---|---|---|---|---|---|---|
| unordered_dense `map` | 0.86 | 0.79 | 0.84 | 1.05 | 0.69 | **0.84** | 0.88 | 0.85 |
| unordered_dense `segmented_map` | 0.97 | 0.94 | 0.92 | 1.12 | 0.77 | **0.94** | 0.98 | 0.95 |
| `CoordMap` | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 |

Without the allocator category, the least reliable, the `map` built into Bonxai is at
0.88 per category, 0.95 in total time and 0.86 on every time; `segmented_map` at 0.98, 1.05
and 0.97.

Per operation, on the ten large, patterned and small maps, built into Bonxai:

| relative to `CoordMap` | unordered_dense `map` | unordered_dense `segmented_map` |
|---|---|---|
| build | 0.74 to 1.03 | 0.86 to 0.96 |
| hit | 0.98 to 1.24 | 1.11 to 1.54 |
| miss | 0.36 to 0.73 | 0.40 to 1.01 |
| update every cell | 1.03 to 1.09 | 1.05 to 1.22 |
| `forEachCell` | 0.79 to 1.00 | 0.90 to 1.04 |
| `clear()` | 0.78 | 0.81 |

End to end, built into Bonxai, medians of 5:

| relative to `CoordMap` | unordered_dense `map` | `CoordMap`, ms |
|---|---|---|
| rays: cast | 0.995 | 2335 |
| rays: random queries | 1.07 | 94 |
| rays: queries of the endpoints | 1.11 | 76 |
| `insertPointCloud()`, 10 cm | 0.96 | 4929 |
| `isOccupied()`, 10 cm | 1.13 | 217 |
| `insertPointCloud()`, 5 cm | 0.985 | 5739 |
| `isOccupied()`, 5 cm | 1.11 | 279 |

Building the map of 970k roots, every insertion timed, medians of 3:

| | build, ms | worst insertion, ms | first build of the process: worst insertion, ms |
|---|---|---|---|
| `CoordMap`, in Bonxai | 675 | 13 | 22 |
| unordered_dense `map`, in Bonxai | 635 | 20 | 37 |
| unordered_dense `segmented_map`, in Bonxai | 621 | 30 | 36 |
| unordered_dense `map`, `unique_ptr` | 660 | 11 | 13 |
| boost `unordered_flat_map`, in the map | 832 | 51 | 107 |
| boost `unordered_node_map` | 799 | 74 | 72 |

The worst insertion is the one that grows the map: the flat map moves 116 MB of
`InnerGrid`s, `CoordMap` rebuilt a 32 MB index.

### The decision

The rule was written down before the final results were looked at, and applied unchanged
to the runs after the fix:

1. Score each map by the geometric mean of all its times relative to `CoordMap`.
2. Switch to a library that scores 0.95 or less and is at most 5% slower end to end.
   Prefer `segmented_map`, which never moves an `InnerGrid` when inserting, unless the
   flat `map` is another 5% better, measured built into Bonxai.
3. Within 5% either way, prefer the established library.
4. Keep `CoordMap` only if it is at least 5% better than the others.

Every contender scores 0.79 to 0.92: `CoordMap` goes. Built into Bonxai, the `map` scores
0.85 and is 5.0% slower end to end, `segmented_map` 0.95 and 12% slower: the `map` is the
one that passes, and it is 11% better than `segmented_map`.

Two things are close enough to say. The end to end check passes by a hair, 1.0499 against
1.05, and only because of queries: in the time the end to end workloads take, the `map` is
1.5% faster, since casting rays and inserting point clouds, which take nearly all of it,
are as fast or faster. In the harness, every unordered_dense variant failed that check,
1.09 to 1.11, and boost's two maps passed it; they are 3 to 5% slower than `CoordMap` in
total time, and stall up to 107 ms when they grow. And the `map` gives up a guarantee:
`InnerGrid`s move when the map grows or loses a root. Bonxai's own code copes with it; code
that keeps references into `rootMap()` must not keep them across insertions.

### Where unordered_dense's map loses

- **Queries of cells that are there**, through `VoxelGrid`: 0.98 to 1.24 times
  `CoordMap`'s time, and the queries of the end to end workloads 7 to 13% slower.
- **The worst insertion** of a large map, when it grows: 20 ms at 970k roots, where
  `CoordMap` took 13.
- **References into `rootMap()`**, as above.

## What bit along the way

**A CoordT must not go through memory whole.** Computed field by field, a `CoordT` that
GCC has to put in memory is written as three 32 bits stores. Copied, or passed by value,
it is then read back with one 64 bits load for x and y, which cannot be forwarded from two
stores: that load waits for them to retire, that is, for the previous lookup to finish,
and lookups that should overlap run one after the other. It turned up in five places,
each time making updates or random queries 1.2 to 3.5 times slower: the insertion path of
the root map inlined into the accessor, a `CoordT` passed by value to a function kept out
of line, the accessors copying a key into their cache with `prev_root_coord_ = root_key`,
which `main` does too, `std::hash<CoordT>` packing x and y into one word, which a `find()`
that is not inlined then reads from memory, and the harness that compared the maps, where
13 of them paid for it. Hence the out of line `Accessor::findOrCreateLeaf`, which takes
three integers, `ConstAccessor::cacheKey`, which copies field by field, and hashes that
read each coordinate on their own.

**The slow path must stay out of the fast one.** A loop of `setValue()` in scan order
barely touches the root map, and runs fast only if `setValue()` is inlined into it. With
the lookup, the creation of roots and leaves and their allocation inlined into
`setValue()`, it no longer was, and updating the room scan got 1.7 times slower than on
`main`. `Accessor::findOrCreateLeaf`, taken when the coordinates leave the cached leaf,
is never inlined. Its read-only counterpart is: it has no allocation to carry, and random
queries are 10 to 20% faster with it inlined.

**Destroying the roots front to back hands the heap back to the kernel.** Front to back
frees the memory in increasing addresses: glibc keeps the first few blocks of each size in
its thread cache, merges all the others into the top of the heap, and that top then goes
back to the kernel, to be faulted in again by the next grid. Creating the room scan in a
loop took 1581 page faults and 3.5 times as long per grid. `clear()` destroys the roots
last to first. Martin Ankerl found and fixed the same thing in
`unordered_dense::segmented_map` 5.1.0.

**A benchmark can measure the kernel instead of the code.** The 32 MB cap of
`M_MMAP_THRESHOLD` and the variable cost of a page fault on a virtual machine made one map
look 2.7 times slower on one operation, and that number overrode a rule written down to
avoid exactly that kind of decision. Numbers that decide should be reproduced in another
order, and explained, before they are trusted.

??BEFORE_AFTER??

??NANOVDB??
