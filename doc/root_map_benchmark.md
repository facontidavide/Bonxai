# The root map

`VoxelGrid` keeps its root nodes in a hash map, one root per 2^(inner_bits + leaf_bits)
voxels per side: 32 by default, 1.6 m at 5 cm. A room needs a few hundred of them, a
building or an outdoor map hundreds of thousands, and every lookup that leaves the
accessor's cached inner node goes through that map.

This records why the map is now a `CoordMap` (`bonxai/coord_map.hpp`) rather than a
`std::unordered_map`, and how it was chosen.

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

## CoordMap

- **Index**: open addressing with linear probing, at most half full, over 16 bytes
  buckets `{upper 32 bits of the hash, position of the value, pointer to it}`. A hit goes
  from its bucket straight to the value. A miss almost never leaves the buckets, since the
  32 bits tag rejects the keys that are not there. Growing moves buckets, never values.
- **Values**: each `std::pair<const CoordT, InnerGrid>` is allocated on its own, right
  before the InnerGrid allocates its own data, and never moves, whatever is inserted or
  erased afterwards. That is the guarantee of `std::unordered_map` that the accessors rely
  on: they cache pointers to the InnerGrids.
- **Iteration**: a vector of pointers to the values, in insertion order, which is also the
  order in which their memory was allocated. `forEachCell`, `memUsage`, serialization and
  `clear()` walk the memory forward.
- **Hash**: one multiplication of the packed coordinates. Only its upper 32 bits are used,
  as the tag and to pick the bucket, and those depend on every bit of x, y and z.
- **Interface**: the subset of `std::unordered_map` that Bonxai uses: `begin`, `end`,
  `find`, `try_emplace`, `insert`, `erase`, `clear`, `size`, `empty`, plus `reserve` and
  `memUsage`. Code that walks `rootMap()` or calls `rootMap().find(key)->second` compiles
  unchanged.

`std::hash<CoordT>` is no longer used by Bonxai, but stays for the containers of the users.
It is now murmur3's finalizer over the packed coordinates: every bit of it is good, where
OpenVDB's left the low bits of root keys at zero, which ruins any table with a power of
two buckets.

## How it was chosen

A copy of `VoxelGrid` took its root map as a template parameter, and every candidate ran
the same workloads in its own process, pinned to one core, in a different order every
round; the tables give medians. The workloads:

| | cells | roots | what it exercises |
|---|---|---|---|
| wide | 300k random, cube of 200 m centred on the origin | 278k | random access to a large map (`benchmark_nanovdb`'s) |
| wide, positive | the same, shifted to positive coordinates | 278k | the hash, when nothing crosses zero |
| 1M | 1M random, cube of 400 m | 970k | a map far larger than the caches |
| rays | lidar-like scans along 800 m, rays cast at 10 cm | 18k | coherent inserts, as `ProbabilisticMap` does |
| room | `benchmark_nanovdb`'s synthetic scan | 354 | a map that fits in the caches |

The candidates, on the wide workload, relative to the `std::unordered_map` measured in
the same round:

| | build | hit | miss | iterate | clear |
|---|---|---|---|---|---|
| `std::unordered_map`, with every hash and setting tried | 0.92–1.00 | 0.92–1.16 | 0.80–1.49 | 0.95–1.07 | 0.90–1.04 |
| `std::map`, as the roots of OpenVDB and NanoVDB | 1.54 | 5.5 | 10.4 | 1.71 | 1.16 |
| `boost::unordered_node_map` | 0.49 | 0.63 | 0.36 | 1.30 | 1.30 |
| `absl::node_hash_map` | 0.55 | 0.73 | 0.44 | 1.14 | 1.29 |
| `ankerl::unordered_dense::segmented_map` 5.1 | 0.43 | 0.76 | 0.33 | 0.63 | 0.42 |
| `ankerl::unordered_dense::map` of `unique_ptr` | 0.48 | 0.64 | 0.42 | 0.59 | 0.52 |
| **`CoordMap`** | **0.46** | **0.60** | **0.29** | **0.65** | **0.53** |

Every open addressing map beats `std::unordered_map` by about as much, as long as its
values sit next to their InnerGrid's data: the variants that kept their values in a pool
of their own, or inside the buckets, were 15 to 25% slower on hits. The node based maps
iterate and clear in hash order, 1.3 times slower than `std::unordered_map`.
`CoordMap` is as fast as the best of them everywhere, without a dependency.

The hashes were measured in the same way, and with a simulation of the probe sequences
on grids, planes, lines and the clouds above. When only its upper bits are used, the one
multiplication spreads the keys as well as murmur3's finalizer or `mum`, in about half the
time: lookups in the room, where the whole map fits in the caches, are 12% faster with it.
A cheaper linear hash, `x * a + y * b + z * c`, was rejected: its probe sequences grew
long on some of the grids centred on the origin.

## What bit along the way

Five things, found by measuring the code that ships rather than a model of it, and each
one worth remembering beyond this map.

**Every instruction on the lookup counts.** Lookups on a large map are latency bound, and
the CPU hides that latency by running several of them at once: the more instructions each
one takes, the fewer are in flight. A first version tested for the empty map, computed its
mask from the number of buckets and compared `find()` to `end()` through `size()`: 20
instructions more than now, and 25% slower lookups. An empty map now points at a static
empty bucket, so that no lookup needs a special case.

**A CoordT must not go through memory whole.** Computed field by field, a `CoordT` that
GCC has to put in memory is written as three 32 bits stores. Copied, or passed by value,
it is then read back with one 64 bits load for x and y, which cannot be forwarded from two
stores: that load waits for them to retire, that is, for the previous lookup to finish,
and lookups that should overlap run one after the other. It turned up in three places,
each time making updates or random queries 1.2 to 3.5 times slower: the insertion path of
the map inlined into the accessor, a `CoordT` passed by value to a function kept out of
line, and the accessors copying a key into their cache with `prev_root_coord_ =
root_key`, which `main` does too. Hence `CoordMap::emplaceNew`, which is never inlined and
takes its key by value, the out of line `Accessor::findOrCreateLeaf`, which takes three
integers, and `ConstAccessor::cacheKey`, which copies field by field.

**The slow path must stay out of the fast one.** A loop of `setValue()` in scan order
barely touches the root map, and runs fast only if `setValue()` is inlined into it. With
the lookup, the creation of roots and leaves and their allocation inlined into
`setValue()`, it no longer was, and updating the room scan got 1.7 times slower than on
`main`. `Accessor::findOrCreateLeaf`, taken when the coordinates leave the cached leaf,
is never inlined. Its read-only counterpart is: it has no allocation to carry, and random
queries are 10 to 20% faster with it inlined.

**Destroying the values front to back hands the heap back to the kernel.** Front to back
frees the memory in increasing addresses: glibc keeps the first few blocks of each size in
its thread cache, merges all the others into the top of the heap, and that top then goes
back to the kernel, to be faulted in again by the next grid. Creating the room scan in a
loop took 1581 page faults and 3.5 times as long per grid. `CoordMap::clear()` destroys
the values last to first. Martin Ankerl found and fixed the same thing in
`unordered_dense::segmented_map` 5.1.0.

**Cache line alignment does not matter.** Half of the nodes have their key and the fields
of the InnerGrid that a lookup reads on two cache lines. Aligning the nodes to 64 bytes
removed that, gained nothing measurable, and made building 1M roots 25% slower.

## Before and after

The code of `main` against this one, each compiled from its own headers into the same
harness. All in ms, medians, with the ratio to `main` in brackets.

| | main | this |
|---|---|---|
| wide: build | 359 | 181 (0.50) |
| wide: query a cell that is there | 86.1 | 55.2 (0.64) |
| wide: query a cell that is not | 58.0 | 18.7 (0.32) |
| wide: update every cell | 110 | 65.5 (0.60) |
| wide: `forEachCell` | 91.5 | 59.3 (0.65) |
| wide: `clear()` | 383 | 209 (0.55) |
| 1M: build, first one in the process | 5238 | 1821 (0.35) |
| 1M: query a cell that is there | 337 | 174 (0.52) |
| 1M: query a cell that is not | 265 | 61.3 (0.23) |
| 1M: update every cell | 458 | 196 (0.43) |
| 1M: `forEachCell` | 359 | 172 (0.48) |
| rays: cast all the rays | 2982 | 2635 (0.88) |
| rays: random queries | 139 | 96.1 (0.69) |
| room: build | 0.96 | 0.73 (0.77) |
| room: queries in scan order | 0.48 | 0.45 (0.93) |
| room: shuffled queries | 1.68 | 1.22 (0.72) |

Casting rays changes the least: a ray walks from voxel to voxel, and the accessor's cache
answers nearly every step. What it gains comes from the cache being copied field by
field. The build of 1M roots is the first of each process: later ones measure how the
allocator hands 3 GB back and forth with the kernel.

These numbers come from a 4 core Xeon (Skylake-SP) virtual machine, where the 64 bits
division that `std::unordered_map` does on every lookup is slow and page faults are very
expensive: the ratios may differ on another CPU, the ordering should not.

## Bonxai against NanoVDB

`benchmark_nanovdb` gives both libraries the exact same integer coordinates. Build it
with:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBONXAI_BENCHMARK_NANOVDB=ON
cmake --build build
./build/bonxai_core/benchmark/benchmark_nanovdb
```

NanoVDB 13.1.0 here, and `tools::build::Grid`, its mutable structure. Each benchmark ran
in a process of its own: in one process, the allocator's state left by the previous
benchmark moves the timings of creation by up to 2.5 times, NanoVDB's included. Median of
3, in ms.

| | main | this | NanoVDB |
|---|---|---|---|
| room: create | 1.73 | 1.21 (0.70) | 4.87 |
| room: update | 0.437 | 0.403 (0.92) | 0.787 |
| room: query, in scan order | 0.529 | 0.486 (0.92) | 0.322 |
| room: query, shuffled | 1.87 | 1.44 (0.77) | 1.31 |
| room: `forEachCell` | 0.285 | 0.269 (0.94) | 0.355 (`NanoGrid`) |
| wide: create | 1099 | 406 (0.37) | 1683 |
| wide: query a cell that is there | 87.3 | 55.1 (0.63) | 53.1 |
| wide: query a cell that is not | 65.9 | 19.6 (0.30) | 37.9 |
| wide, `StaticShape<4, 3>`: query a cell that is there | 109 | 82.3 (0.76) | |
| wide, `StaticShape<4, 3>`: query a cell that is not | 71.7 | 38.8 (0.54) | |

On random access, Bonxai's default shape is now level with NanoVDB when the cell is there,
and twice as fast when it is not; on this machine, `main` was 1.5 to 1.6 times slower than
NanoVDB on the first. The wider `StaticShape<4, 3>` no longer pays off anywhere: fewer
roots, but inner nodes of 64 KB.

Comparing the creation timings of the two libraries says little here: they measure how
glibc trades memory with the kernel, which each library's pattern of allocation triggers
differently. Setting `GLIBC_TUNABLES=glibc.malloc.trim_threshold=4294967295`, the room
takes NanoVDB 1.5 ms instead of 4.4, and Bonxai 5.1 ms instead of 1.1: that setting also
freezes the threshold above which glibc maps memory directly, which then applies to every
1 MB block of leaves. Between `main` and this, under the same allocator, the comparison
holds.

The memory used by the wide map goes from 959 MB to 977 MB: the buckets of the index, and
a slightly larger node.
