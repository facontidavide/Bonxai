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
It now multiplies each coordinate by its own constant and mixes the result with murmur3's
finalizer: every bit of it is good, where OpenVDB's left the low bits of root keys at zero,
which ruins any table with a power of two buckets.

## The code

| File | What changed |
|---|---|
| `bonxai_core/include/bonxai/coord_map.hpp` | New: `CoordMap<ValueT>`, 383 lines, standard library only. |
| `bonxai_core/include/bonxai/bonxai.hpp` | `VoxelGrid::RootMap` is a `CoordMap<InnerGrid>`. The slow paths of the accessors take three integers, and the mutable one is never inlined. The accessors copy keys into their cache field by field. `memUsage()` counts the index in bytes. |
| `bonxai_core/include/bonxai/grid_coord.hpp` | `std::hash<CoordT>`: no truncation, no sign extension, every bit mixed, each coordinate read on its own. |
| `bonxai_core/test/coord_map_test.cpp` | New: 9 tests of `CoordMap` on its own. |
| `bonxai_core/test/voxel_grid_test.cpp` | The test of `std::hash<CoordT>` also covers grids that cross zero, and its low bits. |
| `bonxai_core/benchmark/benchmark_nanovdb.cpp` | The wide queries also measure cells that are not in the grid. |

### Inside CoordMap

```text
buckets_: 2^bits of 16 bytes, at most half full        values_: 8 bytes each, in insertion order
   i   tag         pos  value                            [0]  [1]  [2]  ...  [7]  ...
  ...                                                                          |
  41   0x52e3a1c0   7   ------------------> node 7 <---------------------------+
  42   0x52f9001b   2   ------------------> node 2
  43   0            0   nullptr    (empty: ends the probe sequences that reach it)
  ...

node: one std::pair<const CoordT, InnerGrid> per allocation, 120 bytes, allocated just
      before the InnerGrid allocates its own array of 64 pointers to leaves (1 KB)
```

The map has five members: `values_`, the vector of pointers to the nodes; `buckets_`;
`bits_`, the log2 of the number of buckets, 0 before the first insertion; and `mask_` and
`shift_`, derived from `bits_` and stored because every lookup uses them.

**The hash.** `tagOf()` packs x and y into one 64 bits word, folds z in with a
multiplication by the golden ratio, multiplies the lot by an odd constant, and keeps the
upper 32 bits: the *tag*. A key's home bucket is the top `bits_` bits of its tag. Carries
only move upwards in a multiplication, so those upper bits depend on every bit of x, y and
z, and they are the only ones used. The tag is used twice more. A bucket is compared with
a key through its tag first, and through the node only when the whole tag matches: a
lookup reads the node of another key at most once in 2^(32 - bits) comparisons, 1 in 4096
with 1M buckets. And growing needs no hashing, since the new home of a bucket is in its tag.

**Lookup.** `find()` walks from the home bucket to the first bucket that is empty or that
holds the key, and builds its result from that bucket alone: the iterator carries the
bucket's position and pointer, and an empty bucket's pointer is null, as `end()`'s is. The
empty map needs no special case either. Until the first insertion, `buckets_` points at a
single static empty bucket and `shift_` is 32, so every key's home is bucket 0 and every
lookup stops there. The index grows when it would become more than half full, so it is
between a quarter and half full: with linear probing, a hit reads 1.5 buckets on average
at worst, and a miss 2.5.

**Insertion.** `try_emplace()` is the same walk. A key that is missing goes to
`emplaceNew()`, never inlined, which:

1. doubles the number of buckets if one more value would make them more than half full
   (the first insertion allocates 16);
2. doubles the capacity of `values_` if it is full;
3. allocates the node, and constructs the key and then the value in place, from
   `try_emplace()`'s arguments. For an `InnerGrid`, whose constructor allocates its array,
   the array usually comes right after the node in memory;
4. appends the pointer to `values_`, and writes `{tag, position, pointer}` into the empty
   bucket that ends the key's probe sequence. That bucket is looked up again, since step 1
   may have moved everything.

Nothing can throw once the node exists: if an allocation or the value's constructor
throws, the map holds exactly what it held before, with more capacity at most.

**Erasure.** `erase()` empties the key's bucket with backward shift deletion. The
buckets that follow, up to the empty bucket that ends the cluster, are visited in order:
each one moves into the hole unless its home lies after the hole, and the hole then moves
to where it was. Nothing is marked deleted, so lookups do not get slower as keys are
erased. The last pointer of `values_` then fills the erased one's position, and its bucket's
`pos` is updated: the bucket is found by walking from its home, comparing pointers rather
than keys. Finally the node is freed. No value moves; only pointers to them do. This changes
the order of iteration, which `std::unordered_map` does not promise to keep either.

**Growth.** `rehash()` allocates the new buckets, and reinserts each old one at the top
bits of its tag with linear probing: no hash is computed, no key compared, no node read.
Going beyond 2^32 buckets throws `std::length_error`: that is 2^31 roots, a cube of
2000 km at 5 cm. `reserve()` grows the buckets and `values_` for a given number of values.

**Destruction.** `clear()`, which the destructor calls, deletes the nodes last to first,
frees `values_` and the buckets, and points at the static empty bucket again. Why last to
first is in *What bit along the way*.

**Iteration.** The iterators are forward iterators over `values_` that carry the position
and the pointer, so dereferencing the result of `find()` reads neither `values_` nor the
buckets again. Two iterators are equal when they point at the same node. `end()` points at
none, so comparing with it is a test of that pointer.

**Invariants**, which the tests exercise through the interface: every node is in exactly
one bucket, and `values_[bucket.pos] == bucket.value`; no bucket between a node's home and
its own bucket is empty; at most half of the buckets are full.

**Memory.** A node is 120 bytes, 128 with malloc's header, where libstdc++'s node is 136,
144 with the header: it also holds a pointer to the next node, and the hash, cached. The
index costs 16 bytes per bucket, 2 to 4 buckets per root, and 8 bytes per root in `values_`,
up to 16 with its spare capacity: 40 to 80 bytes per root, against 8 to 16 for
`std::unordered_map`'s array of buckets. Each root also has its InnerGrid's 1 KB array,
and its leaves: the wide map below takes 977 MB instead of 959.

**What it does not do.** `CoordMap` offers the part of `std::unordered_map`'s interface that
Bonxai uses: `begin`, `end`, `cbegin`, `cend`, `size`, `empty`, `find`, `try_emplace`,
`insert(value_type&&)`, `erase(key)`, `clear`, `swap`, and moves; plus `reserve` and
`memUsage`. There is no `operator[]`, `at`, `count`, `contains`, `emplace`,
`insert(const value_type&)`, erase by iterator, bucket interface, `max_load_factor`,
`hash_function` or copy. Code that uses one of them on `rootMap()` no longer compiles; it
could not copy the map before either, since an `InnerGrid` cannot be copied. Inserting
or erasing invalidates every iterator, where `std::unordered_map` only invalidates them
when it rehashes, or when their own element is erased. A reference to a value stays valid
until that value is erased, as with `std::unordered_map`: this is the guarantee that
matters to the accessors.

### VoxelGrid's side

**The type.** `VoxelGrid::RootMap` is `CoordMap<InnerGrid>`, and `rootMap()` returns it as
before. A missing root is created with `try_emplace(root_key, inner_bits)`, which builds the
`InnerGrid` in its node, right before its array. `main` built a temporary `InnerGrid`, then
moved it into a node that `insert()` allocated after its array.

**The accessors.** Each accessor caches the key and the pointer of the last leaf it used,
and the key and the pointer of the last root. Their fast path is the same as before:
`value()`, `setValue()`, `isCellOn()` and the others compare the inner key of the
coordinates with the cached one, and only update the cache differently. When the
coordinates leave the cached leaf, they call `getLeafGrid()`, which now only forwards x,
y and z to one of two functions:

- `ConstAccessor::findLeaf(x, y, z)`: computes the root key, compares it with the cached
  one, looks it up in the root map only if they differ, then finds the leaf in the
  `InnerGrid`. It is inline: random queries are 10 to 20% faster with it inlined.
- `Accessor::findOrCreateLeaf(x, y, z, create_if_missing)`: the same, creating the root
  and the leaf when they are missing. It is never inlined (`BONXAI_NOINLINE`): it carries
  the allocations, which would otherwise make `setValue()` too big to be inlined into the
  loops that call it.

Both take three integers rather than a `const CoordT&`, so that the coordinates reach
them in registers. Both update the cache with `cacheKey()`, which copies a key field by
field. The reasons are in *What bit along the way*.

**`memUsage()`.** `main` summed, over the buckets of the unordered map, 1 for an empty bucket
and its number of nodes for the others: a count, not bytes. It now adds
`CoordMap::memUsage()`, the bytes of the buckets and of `values_`, and the keys. The
`InnerGrid` in each node is counted, as before, by `InnerGrid::memUsage()`.

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

`CoordMap` keeps its own hash, which does pack x and y: inside Bonxai, its lookups are
inlined where the root key is computed, so that key is in registers, never in memory.

### Tests

`coord_map_test.cpp` tests `CoordMap` on its own, with a value that counts its live
instances and can only be moved, like an `InnerGrid`:

- `EmptyMap`: lookups, erasures and iteration on a map that never held anything.
- `TryEmplaceInsertsOnlyMissingKeys`: a second `try_emplace()` of a key neither inserts nor
  constructs anything.
- `IteratesInInsertionOrder`.
- `ValuesNeverMove`: the address of every value stays the same across growth and the
  erasure of other keys.
- `RandomOperationsMatchStdMap`: 200k random insertions, erasures and lookups against a
  `std::map`, on root keys of a small grid, so that clusters are long and erasures shift
  them; the contents are compared in full every 20k operations, then everything is erased
  in random order, and no instance may be left alive.
- `ClearReleasesEverythingAndTheMapIsReusable`.
- `ClearDestroysTheValuesLastToFirst`.
- `MoveLeavesAnEmptyUsableMap`.
- `ConstAccess`.

In `voxel_grid_test.cpp`, `VoxelGridRootMap.InnerGridsAreNotMovedByInsertion` still pins
the guarantee that the accessors rely on. `CoordHash.DoesNotCollapseOnLargeGrids` checks
`std::hash<CoordT>` on three grids of root keys, two of them crossing zero, and on its low
20 bits alone. CI runs all of them with and without the address and undefined behaviour
sanitizers.

`benchmark_nanovdb`'s wide queries take an argument: `0` queries the cells of the grid, `1`
other random cells in the same volume, nearly all of them in roots that do not exist.

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
and lookups that should overlap run one after the other. It turned up in four places,
each time making updates or random queries 1.2 to 3.5 times slower: the insertion path of
the map inlined into the accessor, a `CoordT` passed by value to a function kept out of
line, the accessors copying a key into their cache with `prev_root_coord_ = root_key`,
which `main` does too, and `std::hash<CoordT>` packing x and y into one word, which a
`find()` that is not inlined then reads from memory. Hence `CoordMap::emplaceNew`, which is
never inlined and takes its key by value, the out of line `Accessor::findOrCreateLeaf`,
which takes three integers, `ConstAccessor::cacheKey`, which copies field by field, and a
`std::hash<CoordT>` that reads each coordinate on its own.

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
