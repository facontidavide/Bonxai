# The hash of the root map

`VoxelGrid` keeps its root nodes in a `std::unordered_map<CoordT, InnerGrid>`. This
records why that map's *hash* changed, and why the map itself did not.

## The problem

`std::hash<CoordT>` used to end with `((1 << 20) - 1) &`, which capped the number of
distinct hash values at ~1M. A Bonxai root node covers only `2^(inner_bits + leaf_bits)`
= 32 voxels per side by default, so a grid reaches hundreds of thousands of roots very
quickly: a cloud spread over 200 m at 5 cm already produces 278k of them. Past ~100k
roots the truncation makes keys collide, and every root lookup starts walking a chain.

Dropping the truncation costs nothing to compute — the multiplications are the same —
and removes the ceiling. `VoxelGridRootMap.HashDoesNotCollapseOnLargeGrids` pins it.

## Grids that cross zero

The hash also collided on any grid crossing zero: each coordinate was sign extended to
64 bits before being multiplied, and the 46656 root keys of a 36³ grid centred on the
origin gave only 29226 distinct values. Robot maps nearly always cross zero, since the
origin is usually where the robot started. Martin Ankerl found this while looking at
`segmented_map` (see below). Taking the coordinates as `uint32_t` before widening them
keeps the same multiplications and removes the collisions: 1 key in ~10⁵ still
collides on grids of up to 8M roots, which is harmless.

A fully mixed hash (x and y packed in one word, z folded in, murmur3's `fmix64`) was
measured too. It has no collisions at all, but `std::unordered_map` is faster with the
plain multiplications. On a lattice, a linear hash taken modulo libstdc++'s prime bucket
count fills the buckets more evenly than a random-looking hash, so chains are shorter.
Root keys only, `std::unordered_map<CoordT, int>`, 4M random finds, gcc 13, mean of
7, build / find in ms:

| root keys | sign extended | `fmix64` | **`uint32_t`** |
|---|---|---|---|
| 36³, centred | 4.6 / 122 | 3.9 / 85 | **3.5 / 78** |
| 36³, positive | 3.7 / 79 | 3.8 / 86 | **3.4 / 77** |
| 100×100×30, centred | 55 / 236 | 40 / 189 | **40 / 192** |
| 100×100×30, positive | 46 / 196 | 48 / 230 | **45 / 188** |
| 200×200×20, centred | 362 / 421 | 287 / 376 | **261 / 325** |

`HashDoesNotCollapseOnLargeGrids` now covers grids centred on the origin too.

The benchmarks below were run before this change, with the sign extended hash, on
positive coordinates only.

## Bonxai against NanoVDB

`benchmark_nanovdb` gives both libraries the exact same integer coordinates. Build it
with:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBONXAI_BENCHMARK_NANOVDB=ON
cmake --build build
./build/bonxai_core/benchmark/benchmark_nanovdb
```

### A large, sparse map: 300k coordinates over 200 m at 5 cm

Median of 3 repetitions, with the standard deviation, on an idle machine.

The two libraries are not built the same way below the root, so Bonxai is measured with
two shapes:

| | structure | root covers | roots here |
|---|---|---|---|
| NanoVDB | root map → upper 32³ → lower 16³ → leaf 8³ | 4096 voxels | a handful |
| `StaticShape<2, 3>` (default) | root map → inner 4³ → leaf 8³ | 32 voxels | 278k |
| `StaticShape<4, 3>` | root map → inner 16³ → leaf 8³ | 128 voxels | 33k |

`StaticShape<4, 3>` is the like-for-like one: the same 8³ leaf and the same 16³ node
above it as NanoVDB, the only difference being NanoVDB's extra 32³ level, whose job the
root map does in Bonxai.

**Random queries**

| | truncated hash | untruncated hash |
|---|---|---|
| Bonxai `StaticShape<2, 3>` | 134 ms ± 7 | **57.8 ms ± 2.9** |
| Bonxai `StaticShape<4, 3>` | 76.8 ms ± 3.7 | **53.2 ms ± 1.0** |
| NanoVDB | 30.8 ms ± 0.6 | 31.0 ms ± 0.7 |

Two things to read here. The truncated hash punished the default shape for having many
roots, which is why the wider shape used to be 1.7x faster; with the hash fixed, the two
shapes perform the same (58 vs 53 ms), so the default shape no longer has to be traded
away — and it uses 959 MB against 2809 MB for the same 300k cells.

And NanoVDB stays ~1.7x faster on fully random access, down from 2.5x. That part is
structural: its extra level means its root container is consulted about once, while
Bonxai hashes a root key on every lookup that leaves the cached inner node. Fixing the
hash makes that hash cheap; it cannot make it disappear.

**Building the map**

| | truncated hash | untruncated hash |
|---|---|---|
| Bonxai `StaticShape<2, 3>` | 926 ms ± 212 | **581 ms ± 47** |
| Bonxai `StaticShape<4, 3>` | 828 ms ± 506 | 844 ms ± 382 |
| NanoVDB | 447 ms ± 6 | 411 ms ± 1 |

Only the first row says anything: filling a grid allocates it, and at 1 to 3 GB per
iteration these timings are dominated by the kernel handing out and zeroing pages. The
`StaticShape<4, 3>` rows, which allocate 2.8 GB of dense 16³ inner nodes, have a
standard deviation of half their own value and should be ignored.

### The room scan (48k cells, 72 roots), unchanged

| | truncated | untruncated | NanoVDB |
|---|---|---|---|
| create | 1218 µs | 1233 µs | 1525–1597 µs |
| update | 418 µs | 420 µs | 648–652 µs |
| query, in scan order | 412 µs | 404 µs | 269–277 µs |
| query, shuffled | 1711 µs | 1612 µs | 1199–1202 µs |

With 72 roots the hash is irrelevant, and the numbers only confirm that nothing
regressed. Note that these are `tools::build::Grid`, NanoVDB's mutable structure; its
read-only `NanoGrid` is faster still, but it has to be built first (`NanoVDB_Convert`)
and Bonxai has no equivalent.

## Why the container was left alone

`ankerl::unordered_dense` was vendored and measured before settling on the hash. Two
things came out of it.

First, most hash maps are not eligible at all. The accessors cache a raw `InnerGrid*`
that points **inside** the root map, so a container that moves its values when it grows
would leave every accessor dangling — that rules out every flat hash map, whose whole
point is to keep values in a contiguous array.
`VoxelGridRootMap.InnerGridsAreNotMovedByInsertion` pins that requirement.

Second, the segmented variant, which is eligible, is not worth a dependency (1M
coordinates, best of 5, update / read in ms):

| | 46k roots | 110k roots | 420k roots | 785k roots |
|---|---|---|---|---|
| `std::unordered_map`, truncated | 149 / 160 | 319 / 310 | 1090 / 1014 | 1590 / 1646 |
| **`std::unordered_map`, untruncated** | **131 / 143** | **161 / 163** | **218 / 216** | **206 / 197** |
| `unordered_dense::segmented_map` | 250 / 265 | 304 / 331 | 545 / 615 | 749 / 814 |
| `segmented_map`, mixed hash | 189 / 219 | 174 / 194 | 178 / 186 | 170 / 179 |

It only draws level in the very sparse regime, where the plain map with a working hash
is already there, and it is clearly worse at the sizes a robot actually maps — 480 ms
against 261 ms to build the 46k-root map. Those rows are kept as a record; the vendored
header is not in the tree.

Martin Ankerl, the author of `unordered_dense`, later tracked `segmented_map`'s poor
showing at 46k roots down to the order in which it destroyed its values: front to back,
which made glibc hand the heap back to the kernel on every teardown, so each rebuild in
the benchmark paged everything in again. Release 5.1.0 fixes it. On his reconstruction
of this benchmark, with a collision free hash, `segmented_map` ties with
`std::unordered_map` at 46k roots and builds 1.9x faster at 787k roots. That makes it
worth measuring again if very large maps become the target.
