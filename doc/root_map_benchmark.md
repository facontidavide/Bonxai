# The root map

`VoxelGrid` keeps its root nodes in a hash map, one root per 2^(inner_bits + leaf_bits)
voxels per side: 32 by default, 1.6 m at 5 cm. A room needs a few hundred of them, a
building or an outdoor map hundreds of thousands, and every lookup that leaves the
accessor's cached inner node goes through that map.

This records why the map is now a `CoordMap` (`bonxai/coord_map.hpp`) rather than a
`std::unordered_map`, how it works, and how it was chosen: against 20 hash maps, each at
its best, on 21 workloads, then on bare metal against the finalists built into Bonxai. It
is not the fastest map on every operation, the ones it loses are in *Where CoordMap
loses*; it stays because, built into Bonxai, none of the alternatives is as fast end to
end, which the rule written down before the comparison requires: see *The decision*.

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
| `bonxai_core/test/voxel_grid_test.cpp` | The test of `std::hash<CoordT>` also covers grids that cross zero, and its low bits. New: the accessors survive the release of many roots. |
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
and its leaves: the wide map of *Before and after* takes 1013 MB instead of 999.

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
the guarantee that the accessors rely on. `VoxelGridRootMap.AccessorsSurviveReleasingManyRoots`
has a reader cache the last of 100 roots, releases half of them, and checks that the reader
then finds every remaining cell, none of the released ones, and a write made through
another accessor: a map that fills the holes of erased roots with other roots, as
unordered_dense's do, must pass it too. `VoxelGridRootMap.AccessorsSurviveChangesToOtherRoots`
erases and inserts, through `rootMap()`, roots other than the one a reader cached, and
checks what the reader finds afterwards. `CoordHash.DoesNotCollapseOnLargeGrids` checks
`std::hash<CoordT>` on three grids of root keys, two of them crossing zero, and on its low
20 bits alone. CI runs all of them with and without the address and undefined behaviour
sanitizers.

`benchmark_nanovdb`'s wide queries take an argument: `0` queries the cells of the grid, `1`
other random cells in the same volume, nearly all of them in roots that do not exist.

## How it was chosen

`CoordMap` was chosen twice. First against `std::unordered_map`, `std::map`, and the maps
of boost, absl and unordered_dense, on five workloads. Asked whether that was a special
case, the comparison was redone with every header only map of
[hashtable-bench](https://github.com/renzibei/hashtable-bench) and a few more, each at its
best, on 21 workloads. It partly was: `CoordMap` is not the fastest map on every operation,
nor by the usual summaries of them, where Martin Ankerl's
[unordered_dense](https://github.com/martinus/unordered_dense) is ahead. It stays because
finding cells that are there, what ray casting and the queries of a map spend their time
on, is faster with it: built into Bonxai, unordered_dense's maps are 8% slower end to end,
and the rule written down before the final results does not let them in. The details
follow; the tables of this section come from a virtual machine, *On bare metal* gives the
final numbers.

### The candidates

| Map | Version | Licence | How the InnerGrids were held |
|---|---|---|---|
| `std::unordered_map` | libstdc++ 13 | | in the nodes |
| `boost::unordered_map`, `unordered_node_map`, `unordered_flat_map` | 1.83 and 1.92 | BSL | in the nodes; flat: `unique_ptr`, raw pointer, or in the map |
| `absl::node_hash_map`, `flat_hash_map` | 20220623 and 20260817 | Apache 2.0 | the same |
| `phmap::node_hash_map`, `flat_hash_map` | 2.0 | Apache 2.0 | the same |
| `ankerl::unordered_dense::map`, `segmented_map` | 5.1.0 | MIT | `unique_ptr`, raw pointer, or in the map |
| `tsl::robin_map`, `hopscotch_map`, `sparse_map` | 1.4.1, 2.4.0, 0.7.0 | MIT | `unique_ptr` or raw pointer; robin: in the map too |
| `ska::flat_hash_map`, `bytell_hash_map` | 2018 | BSL | `unique_ptr` or raw pointer |
| `emhash5`, `6`, `7`, `8` | 1.1.0 | MIT | `unique_ptr` or raw pointer |
| `robin_hood::unordered_node_map`, `unordered_flat_map` | 3.11 | MIT | in the nodes; flat: `unique_ptr` or raw pointer |
| `fph::DynamicFphMap`, `MetaFphMap` | 2026 | Apache 2.0 | `unique_ptr` or raw pointer |

A map that moves its values held a `std::unique_ptr<InnerGrid>`, or a raw pointer that
the harness owned, which lets it copy them as plain bytes; or the `InnerGrid` itself, with
every accessor dropping its cache whenever the map moved its values. Each map had every
hash it can work with: the packed coordinates for the maps that mix hashes themselves, a
multiplication and a fold or murmur3's finalizer for the others, their own where they have
one. 98 configurations in all.

### How

A copy of `VoxelGrid` took its root map as a template parameter and ran the same accessor
code with every map. Each configuration was a binary of its own, built by GCC 13 at `-O3`,
and every run a process of its own, pinned to one core, in a different order every round.
A pilot of 4 workloads and 3 rounds picked each library's best configuration. The final
suite ran 13 configurations, the leading ones and `CoordMap`'s own, on 21 workloads, 3
rounds, and the other libraries' best once.
Then the two ways of using unordered_dense that led were built into Bonxai itself, and
measured against `CoordMap` from the repository's headers, 3 rounds again.

Two things in the harness had to be fixed before its numbers meant anything:

- **The key through memory**, see *What bit along the way*. The wrappers first passed the
  maps a reference to a key that GCC had written as three 32 bits stores, and the maps
  whose `find()` is not inlined read it back with one 64 bits load: 13 configurations
  updated cells 2.2 to 2.8 times slower than they can. `CoordMap`, inlined, never paid for
  it. The first comparison had the same flaw.
- **Sizes.** Every map grows at its own thresholds, so any single size favours some of
  them: at 278k roots, `CoordMap`'s index had just doubled to 16 MB, a quarter full, where
  the others were half full. A sweep of 14 sizes from 10k to 1M roots covers whole
  doublings.

Two binaries of the same code differ too: `CoordMap` built as the harness and from the
repository's headers were up to 15% apart on a single time, 1 to 2% on the summaries. A
few percent on one time means nothing here.

### The workloads

| Category | Workloads |
|---|---|
| large maps | 300k cells at random in a cube of 200 m centred on the origin, 278k roots (`benchmark_nanovdb`'s); the same shifted to positive coordinates; 1M cells in 400 m, 970k roots; one cell in each of 250k random roots |
| key patterns, 250k roots | a dense cube; a rolling terrain, one layer of roots over 800 × 800 m; a corridor 100 km long; random roots whose coordinates are multiples of 16 |
| small maps | 1k and 10k random roots; `benchmark_nanovdb`'s room scan, 354 roots |
| the map alone | insertion with and without `reserve()`, hits, misses, 50% hits, erase and insert, iteration and clear: 1k, 100k and 1M random keys, and 100k keys of a dense cube |
| end to end | lidar-like rays cast at 10 cm along 800 m, then random queries and queries of the endpoints; `ProbabilisticMap::insertPointCloud()` of 60 scans of a street at 10 cm and of 25 at 5 cm, then `isOccupied()` at random points |
| allocator | creating and destroying the room's grid, and the wide map, without the `mallopt()` of every other run |
| size | insertion, hits and misses at 14 sizes, 10k to 1M roots |

Every operation is timed: building, a query of a cell that is there (a *hit*) or not (a
*miss*), updating every cell, `forEachCell` and `clear()`. The wide workloads query the
cells in the order they were inserted, as `benchmark_nanovdb` does, the others in a random
order.

### Results

Relative to `CoordMap`, lower is better. *Every time*, the last column, is the geometric
mean of all 126 times at once: the score that the decision rule named. 72 of those times
are the map alone and the size sweep, where `CoordMap` is weakest, so that score weighs the
map on its own more than its use by `VoxelGrid`. Hence two more summaries: *per category*,
the geometric mean of the seven categories, each the geometric mean of its times, and
*total time*, the geometric mean over the workloads of the sum of all the times of each,
which weighs every operation by what it costs. The rows are in the order of *per
category*.

| | large maps | key patterns | small maps | map alone | end to end | allocator | size | **per category** | **total time** | every time |
|---|---|---|---|---|---|---|---|---|---|---|
| ankerl::unordered_dense::map 5.1.0, values inline | 0.92 | 0.83 | 0.85 | 0.66 | 1.08 | 0.93 | 0.77 | **0.85** | 0.92 | 0.79 |
| ankerl::unordered_dense::segmented_map 5.1.0 | 0.97 | 0.97 | 0.92 | 0.73 | 1.05 | 0.93 | 0.76 | **0.90** | 0.95 | 0.83 |
| ankerl::unordered_dense::map 5.1.0, raw pointer | 1.05 | 0.83 | 0.89 | 0.80 | 1.09 | 0.99 | 0.78 | **0.91** | 0.98 | 0.86 |
| ankerl::unordered_dense::map 5.1.0, unique_ptr | 1.05 | 0.87 | 0.92 | 0.81 | 1.10 | 1.04 | 0.78 | **0.93** | 0.99 | 0.87 |
| boost::unordered_flat_map 1.92, values inline | 1.04 | 0.79 | 0.91 | 0.96 | 1.02 | 1.18 | 0.80 | **0.95** | 1.09 | 0.90 |
| boost::unordered_node_map 1.92 ¹ | 1.28 | 0.87 | 0.93 | 0.94 | 0.95 | 1.26 | 0.73 | **0.98** | 1.07 | 0.91 |
| CoordMap | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 |
| boost::unordered_node_map 1.83 | 1.20 | 0.84 | 0.92 | 1.05 | 1.02 | 1.28 | 0.78 | **1.00** | 1.08 | 0.94 |
| phmap::flat_hash_map 2.0, raw pointer ¹ | 1.28 | 0.80 | 1.02 | 1.03 | 1.02 | 1.17 | 0.79 | **1.00** | 1.11 | 0.96 |
| phmap::node_hash_map 2.0 ¹ | 1.34 | 1.00 | 0.98 | 0.98 | 0.96 | 1.22 | 0.87 | **1.04** | 1.09 | 1.00 |
| emhash8::HashMap 1.1.0, raw pointer | 1.09 | 1.07 | 1.03 | 0.98 | 1.09 | 1.02 | 1.08 | **1.05** | 1.07 | 1.05 |
| robin_hood::unordered_flat_map 3.11, raw pointer ¹ | 1.32 | 0.77 | 1.03 | 1.17 | 1.05 | 1.17 | 0.97 | **1.06** | 1.11 | 1.06 |
| absl::node_hash_map 20260817 ¹ | 1.26 | 1.10 | 1.00 | 1.03 | 1.01 | 1.22 | 0.90 | **1.07** | 1.11 | 1.02 |
| robin_hood::unordered_node_map 3.11 ¹ | 1.17 | 0.85 | 1.04 | 1.10 | 1.02 | 1.32 | 1.03 | **1.07** | 1.08 | 1.06 |
| absl::flat_hash_map 20260817, values inline ¹ | 1.22 | 0.97 | 1.06 | 1.03 | 1.10 | 1.23 | 1.03 | **1.09** | 1.12 | 1.06 |
| emhash5::HashMap 1.1.0, raw pointer ¹ | 1.29 | 1.02 | 1.10 | 1.11 | 0.98 | 1.15 | 0.99 | **1.09** | 1.09 | 1.08 |
| emhash6::HashMap 1.1.0, unique_ptr ¹ | 1.30 | 1.08 | 1.12 | 1.06 | 0.95 | 1.25 | 0.96 | **1.10** | 1.10 | 1.06 |
| emhash7::HashMap 1.1.0, raw pointer ¹ | 1.20 | 0.98 | 1.11 | 1.10 | 1.04 | 1.27 | 1.00 | **1.10** | 1.11 | 1.07 |
| ska::bytell_hash_map 2018, raw pointer ¹ | 1.27 | 1.05 | 1.12 | 1.28 | 1.04 | 1.26 | 1.11 | **1.16** | 1.14 | 1.17 |
| tsl::robin_map 1.4.1, raw pointer ¹ | 1.34 | 1.18 | 1.12 | 1.28 | 1.06 | 1.19 | 1.13 | **1.18** | 1.16 | 1.20 |
| tsl::hopscotch_map 2.4.0, unique_ptr ¹ | 1.29 | 1.13 | 1.19 | 1.26 | 1.02 | 1.22 | 1.23 | **1.19** | 1.14 | 1.22 |
| ska::flat_hash_map 2018, unique_ptr ¹ | 1.25 | 1.14 | 1.07 | 1.30 | 1.01 | 1.48 | 1.13 | **1.19** | 1.19 | 1.18 |
| boost::unordered_map 1.92 ¹ | 1.65 | 1.77 | 1.22 | 1.49 | 1.05 | 1.34 | 1.67 | **1.43** | 1.27 | 1.53 |
| tsl::sparse_map 0.7.0, unique_ptr ¹ | 1.88 | 1.68 | 1.39 | 1.81 | 1.21 | 1.49 | 1.56 | **1.56** | 1.56 | 1.63 |
| fph::MetaFphMap 2026, raw pointer ¹ | 1.79 | 1.26 | 1.17 | 2.11 | 1.06 | 2.26 | 1.68 | **1.56** | 2.16 | 1.66 |
| fph::DynamicFphMap 2026, unique_ptr ¹ | 1.97 | 1.64 | 1.14 | 2.47 | 0.94 | 2.49 | 2.23 | **1.74** | 2.27 | 1.95 |
| main: std::unordered_map, old hash | 2.15 | 2.86 | 1.46 |  | 1.21 | 1.48 |  | **1.74** | 1.60 | 1.90 |
| std::unordered_map libstdc++ 13 ¹ | 2.23 | 2.82 | 1.36 | 2.19 | 1.17 | 1.39 | 2.49 | **1.85** | 1.60 | 2.13 |

¹ One round, against the `CoordMap` runs of the three rounds: a few percent either way.

The code as it would ship: the two ways of using unordered_dense that led, vendored into
Bonxai, against `CoordMap`, on the 54 times of the workloads that go through `VoxelGrid`:

| | large maps | key patterns | small maps | end to end | allocator | **per category** | **total time** | every time |
|---|---|---|---|---|---|---|---|---|
| unordered_dense `map`, vendored | 0.89 | 0.83 | 0.86 | 1.02 | 0.94 | **0.90** | 0.96 | 0.89 |
| unordered_dense `segmented_map`, vendored | 0.98 | 0.94 | 0.91 | 1.08 | 0.91 | **0.96** | 1.03 | 0.96 |
| CoordMap | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 |
| CoordMap, built as the harness | 1.00 | 1.04 | 1.00 | 1.01 | 1.03 | **1.01** | 1.02 | 1.01 |

Per operation, on the ten large, patterned and small maps of the code as it would ship:

| relative to `CoordMap` | unordered_dense `segmented_map` | unordered_dense `map` |
|---|---|---|
| build | 0.86 to 1.03 | 0.77 to **2.70** |
| hit | **1.19 to 1.54** | 0.99 to 1.27 |
| miss | 0.39 to 0.93 | 0.37 to 0.80 |
| update every cell | 1.04 to 1.23 | 0.95 to 1.06 |
| `forEachCell` | 0.95 to 1.07 | 0.93 to 1.03 |
| `clear()` | 0.73 to 1.17 | 0.73 to 1.02 |

End to end, casting rays and `ProbabilisticMap::insertPointCloud()` take the same time
with every map to within 3%: a ray walks from voxel to voxel, and the accessors' cache
answers nearly every step. Only the queries at random points differ: `segmented_map` takes
6 to 22% longer, `map` 0 to 10%.

### Where CoordMap loses

- **Misses.** A key that is not there costs it 1.1 to 2.7 times what unordered_dense
  takes in `VoxelGrid`, and 1.8 to 4.2 times what `boost::unordered_flat_map` takes, on
  every workload. Both check one group of 15 or 16 one byte fingerprints; `CoordMap` walks
  16 bytes buckets, 32 to 64 bytes per root, an index that falls out of the caches sooner.
  With 2 MB pages, which remove most TLB misses, the gap shrank only a little. On the large
  maps a miss costs `CoordMap` 0.1 to 0.36 times what a hit does: in total time, the hits
  that it wins against unordered_dense weigh more than the misses it loses.
- **Creating and destroying small grids** in a loop with glibc's default settings: 1.8 ms
  per room grid, the median of 60, where unordered_dense takes 0.9 and `main` 1.1. The
  fastest runs are about the same, 0.9 to 1 ms. The 60 grids cost `CoordMap` 60k page
  faults, `main` 12k and `segmented_map` 8k: glibc hands part of the heap back to the
  kernel after some of them, and the next grid faults it in again. Destroying the values
  last to first made this rarer, not impossible.
- **The map alone**, with an `InnerGrid` of 8 cells, where a hit reads little more than
  the map: `boost::unordered_flat_map` holding the `InnerGrid`s finds them in 0.53 to 0.71
  of `CoordMap`'s time, and is ahead at 13 of the 14 sizes of the sweep, where `CoordMap`
  ranks 2nd to 10th of 30. Against unordered_dense, inserting costs `CoordMap` up to 1.4
  times as much, and 1.3 to 2.2 times after `reserve()`; erasing a key and inserting
  another 1.4 to 1.7 times; clearing 1.1 to 2.6 times: unordered_dense's dense storage of
  values is built for these. Through `VoxelGrid`, where a hit goes on to the `InnerGrid`'s array and to a
  leaf, and an insertion is only part of creating a root, this changes: on each of the 19
  times of finding or updating cells, the fastest map takes 0.84 to 1.06 of `CoordMap`'s
  time, boost's node map most often, 6 times, and none is faster on the 1M map.

### The decision

The rule was written down before the final results were looked at: score each map by the
geometric mean of all its times relative to `CoordMap`, the *every time* column; switch to
a library that scores 0.95 or less and is at most 5% slower end to end; within 5% either
way, prefer the established library; keep `CoordMap` only if it is at least 5% better.
Between unordered_dense's two maps, prefer `segmented_map`, which, like
`std::unordered_map`, never moves an `InnerGrid` when inserting, unless the flat `map` is
another 5% better. The code built into Bonxai must confirm what the harness says.

On the virtual machine, the harness picked unordered_dense's `map`, and the branch
switched to it. That machine turned out as noisy as the differences being decided (one
binary, three runs: 631, 729 and 776 ms), and an earlier finding against that map, 2.7
times as long to build 1M roots, was the benchmark's: glibc caps the threshold above
which it maps memory directly at 32 MB, the harness's `mallopt(M_MMAP_THRESHOLD)` did not
keep the flat maps' larger arrays, and each build paid for page faults that the next one
would not have. The final comparison ran on bare metal, *On bare metal* below.

There, in the harness, every contender scores 0.74 to 0.80: by the rule `CoordMap` goes.
Built into Bonxai, the rule's last clause does not confirm it: unordered_dense's `map` is
0.87 to 0.89 every time but 1.06 to 1.08 end to end, over the 5% allowed, in both runs;
`segmented_map`, 0.92 every time and 1.08 end to end, fails the same way. The loss is
consistent: the random and endpoint queries of `rays10` and the queries of the
probabilistic map took unordered_dense 6 to 28% longer in 46 of their 48 pairs of runs
(eight rounds, two runs), the two others in the same round.

`CoordMap` therefore stays. If misses, building and clearing, or the map on its own matter
more to your use of Bonxai than finding cells, unordered_dense's `map` is the one to try:
2 to 3 times faster on misses, 5 to 15% faster to build, and half of `main`'s time
overall. The change to `VoxelGrid` is in this branch's history: commit `adc6c23`, and
`0564a50`, a destructor that frees the nodes last to first, without which creating and
destroying a large grid takes twice as long.

## On bare metal

An Intel Core i7-13700H laptop, GCC 15, turbo off, the performance governor. Every run a
process of its own, pinned to a performance core whose hyperthread sibling stays idle,
two cores sharing the rounds, the variants in a new random order every round; medians of
eight rounds. The scripts and the raw results are in `doc/root_map_study`. The first run
compared every contender, in the harness and built into Bonxai; the second, the three
built into Bonxai, unordered_dense's `map` with the destructor of `0564a50`. The numbers
below are the second's, relative to `main`: lower is better.

| | `CoordMap` (this) | unordered_dense `map` | unordered_dense `segmented_map` |
|---|---|---|---|
| large maps | 0.48 | **0.44** | 0.45 |
| key patterns | 0.45 | **0.36** | 0.37 |
| small maps | 0.73 | **0.61** | 0.62 |
| **end to end** | **0.75** | 0.81 | 0.81 |
| allocator | 0.58 | **0.55** | 0.67 |
| every time | 0.56 | **0.50** | 0.52 |

| | `main`, ms | `CoordMap` (this) | `map` | `segmented_map` |
|---|---|---|---|---|
| wide: build | 181 | 0.51 | 0.47 | 0.43 |
| wide: query a cell that is there | 41.6 | 0.66 | 0.68 | 0.71 |
| wide: query a cell that is not | 29.8 | 0.27 | 0.24 | 0.27 |
| wide: `forEachCell` | 45.7 | 0.58 | 0.60 | 0.63 |
| wide: `clear()` | 150 | 0.40 | 0.36 | 0.37 |
| 1M: build | 710 | 0.46 | 0.40 | 0.39 |
| 1M: query a cell that is there | 149 | 0.62 | 0.71 | 0.76 |
| 1M: query a cell that is not | 95.1 | 0.43 | 0.25 | 0.29 |
| 1M: update every cell | 184 | 0.55 | 0.60 | 0.61 |
| 250k random roots: query a cell that is there | 166 | 0.75 | 0.92 | 0.93 |
| 250k random roots: query a cell that is not | 83.6 | 0.35 | 0.13 | 0.14 |
| room: shuffled queries | 1.91 | 0.69 | 0.84 | 0.86 |
| rays: cast all the rays | 2090 | 0.96 | 0.96 | 0.96 |
| rays: random queries | 90.1 | 0.52 | 0.59 | 0.60 |
| rays: query the endpoints | 47.2 | 0.71 | 0.86 | 0.85 |
| probabilistic map: insert the clouds | 4430 | 0.92 | 0.94 | 0.95 |
| probabilistic map: query | 167 | 0.72 | 0.83 | 0.81 |
| wide, glibc's defaults: build again | 319 | 0.38 | 0.34 | 0.76 |
| 970k roots: build, memory kept | 775 | 0.50 | 0.44 | 0.43 |
| 970k roots: worst insertion, ms | 93.9 | 11.2 | 12.8 | 14.7 |

The two alternatives trade finding cells for missing them: 2 to 3 times faster on misses,
faster to build and clear, 3 to 25% slower to find a cell that is there. In the harness,
where the maps are compared on their own as well, the other finalists were estimated the
same way: `boost::unordered_flat_map` at 0.50 of `main` overall and 0.75 end to end, but
2 times slower to iterate and clear than the others, and with a worst insertion of 21 ms,
96 on the first build.

The spread of a measurement over the eight rounds, max over min, has a median of 4% and a
90th percentile of 10% in the second run; the first, with a language server busy during
part of it, 11% and 28%. The end to end differences held in 46 of 48 pairs of runs.

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
and lookups that should overlap run one after the other. It turned up in five places,
each time making updates or random queries 1.2 to 3.5 times slower: the insertion path of
the map inlined into the accessor, a `CoordT` passed by value to a function kept out of
line, the accessors copying a key into their cache with `prev_root_coord_ = root_key`,
which `main` does too, `std::hash<CoordT>` packing x and y into one word, which a `find()`
that is not inlined then reads from memory, and the harness that compared the maps, where
13 of them paid for it and `CoordMap` did not. Hence `CoordMap::emplaceNew`, which is never
inlined and takes its key by value, the out of line `Accessor::findOrCreateLeaf`, which
takes three integers, `ConstAccessor::cacheKey`, which copies field by field, and a
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

The code of `main` against this one, each compiled from its own headers into the harness
of *How it was chosen*. All in ms, medians of 3, with the ratio to `main` in brackets.

| | main | this |
|---|---|---|
| wide: build | 349 | 178 (0.51) |
| wide: query a cell that is there | 83.4 | 54.2 (0.65) |
| wide: query a cell that is not | 52.2 | 13.5 (0.26) |
| wide: update every cell | 98.5 | 60.7 (0.62) |
| wide: `forEachCell` | 81.5 | 55.8 (0.69) |
| wide: `clear()` | 355 | 197 (0.55) |
| 1M: build, first one in the process | 2683 | 1709 (0.64) |
| 1M: build, later ones | 1436 | 579 (0.40) |
| 1M: query a cell that is there | 324 | 177 (0.55) |
| 1M: query a cell that is not | 224 | 59.0 (0.26) |
| 1M: update every cell | 420 | 211 (0.50) |
| 1M: `forEachCell` | 337 | 172 (0.51) |
| rays: cast all the rays | 2509 | 2261 (0.90) |
| rays: random queries | 132 | 98.0 (0.74) |
| rays: query the endpoints | 85.2 | 70.9 (0.83) |
| room: build | 0.809 | 0.69 (0.85) |
| room: queries in scan order | 0.423 | 0.375 (0.89) |
| room: shuffled queries | 1.58 | 1.08 (0.69) |
| room: update every cell | 0.392 | 0.327 (0.83) |
| wide: memory, MB | 999 | 1013 |
| 1M: memory, MB | 3382 | 3397 |

Casting rays changes the least: a ray walks from voxel to voxel, and the accessor's cache
answers nearly every step. What it gains comes from the cache being copied field by
field. The first build of the 1M map in a process faults in 3 GB of fresh memory; the
later ones reuse it, and show the map itself.

*On bare metal* has the final numbers. These come from a 4 core Xeon (Skylake-SP) virtual
machine, where the 64 bits
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
benchmark moves the timings of creation by up to 2.5 times, NanoVDB's included.
`doc/root_map_study/harness/nanovdb.py` builds it against `main` and this branch and runs
it that way. On the machine of *On bare metal*, medians of 5, in ms, with the ratio to
`main` in brackets.

| | main | this | NanoVDB |
|---|---|---|---|
| room: create | 1.12 | 1.05 (0.94) | 4.97 |
| room: update | 0.416 | 0.387 (0.93) | 0.609 |
| room: query, in scan order | 0.456 | 0.415 (0.91) | 0.317 |
| room: query, shuffled | 1.90 | 1.29 (0.68) | 1.48 |
| room: `forEachCell` | 0.265 | 0.237 (0.89) | 0.371 (`NanoGrid`) |
| wide: create | 423 | 162 (0.38) | 1041 |
| wide: query a cell that is there | 42.1 | 26.7 (0.63) | 22.7 |
| wide: query a cell that is not | 29.5 | 8.41 (0.28) | 14.5 |
| wide, `StaticShape<4, 3>`: create | 1667 | 1543 (0.93) | |
| wide, `StaticShape<4, 3>`: query a cell that is there | 49.8 | 29.9 (0.60) | |
| wide, `StaticShape<4, 3>`: query a cell that is not | 33.6 | 13.3 (0.40) | |

On random access to the wide map, Bonxai's default shape now takes 1.2 times as long as
NanoVDB when the cell is there, where `main` took 1.9 times as long, and 0.6 times as long
when it is not. On the room, shuffled queries are now faster than NanoVDB's; in scan
order, where the accessors' caches answer nearly every query and the root map hardly
matters, NanoVDB stays 1.3 times faster. The wider `StaticShape<4, 3>` does not pay off,
with `main` or with this: fewer roots, but inner grids of 64 KB.

Comparing the creation timings of the two libraries says little here: they measure how
glibc trades memory with the kernel, which each library's pattern of allocation triggers
differently. On the virtual machine, setting
`GLIBC_TUNABLES=glibc.malloc.trim_threshold=4294967295`, the room
takes NanoVDB 1.5 ms instead of 4.3, and Bonxai 4.8 ms instead of 1.3, `main` 5.0 instead
of 1.2: that setting also freezes the threshold above which glibc maps memory directly,
which then applies to every chunk of 512 leaves, 1 MB of `float` cells. Between `main` and
this, under the same allocator, the comparison holds.

The wide map takes 1013 MB of heap instead of 999: the buckets of the index, and a
slightly larger node.
