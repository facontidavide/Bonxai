# The root map study: notes and instructions

Status on 2026-09-27, branch `claude/issue-69-qvgw41`, PR #70. Everything measured so far
ran on a 4 core virtual machine (Xeon Skylake-SP, GCC 13) whose noise turned out to be as
large as the differences being decided. The decision between the two remaining maps is
therefore **not final**: it waits for a run on bare metal, with the scripts of `harness/`.

## Where the branch stands

| Commit | What |
|---|---|
| `8d5904f` (on main) | #69: the 20 bits truncation of `std::hash<CoordT>` dropped. |
| `dea0b2d` | `std::hash<CoordT>` no longer collides on grids that cross zero. |
| `80e2cbf` | `CoordMap`, a root map written for Bonxai, replaces `std::unordered_map`. |
| `75f04f4` | `std::hash<CoordT>` reads each coordinate on its own (a 2.3x store forwarding stall). |
| `6f0fa7c`, `d6b4d88` | Doc of CoordMap, its comparison against 20 maps, a test of released roots. |
| `adc6c23` | The root map becomes ankerl::unordered_dense's `map`, vendored in `bonxai/detail`. |
| next | CoordMap restored (not used by `VoxelGrid`), this directory. |

The hash fixes are independent of the choice of map and stay whatever it is.

`VoxelGrid` now uses unordered_dense's `map`. `CoordMap` (`bonxai/coord_map.hpp`) and its
tests are kept, unused, until the bare metal run decides. Going back to it means restoring
`bonxai/bonxai.hpp` and `test/voxel_grid_test.cpp` from `d6b4d88`.

## How the choice was made, and unmade

1. A first comparison (std, boost, absl, unordered_dense, 5 workloads) made CoordMap look
   best.
2. Asked whether that was cherry picked, the comparison was redone: 98 configurations of
   20 maps (every header only map of hashtable-bench and a few more), 21 workloads, with a
   rule written down before the results (`DECISION_RULE.md`). The rule picked
   unordered_dense's `map`; CoordMap was kept anyway, because that map had taken 2.7 times
   as long to build 1M roots.
3. **The mmap cap.** That 2.7x was the benchmark's. The harness kept freed memory with
   `mallopt(M_MMAP_THRESHOLD, 32 MB)`, but glibc caps that threshold at 32 MB: larger
   allocations still went back to the kernel between builds, and only the flat maps that
   hold the InnerGrids have arrays that large (63 and 126 MB at 1M roots). A page fault
   costs 2 to 30 us on the VM depending on what its host did with the memory: the same
   build took 1620 or 642 ms depending on the order of the runs. The harness now sets
   `mallopt(M_MMAP_MAX, 0)`. Fixed, the 1M build of unordered_dense's map is 1.03x
   CoordMap's.
4. The contenders ran again, the rule applied with no override: it picked unordered_dense's
   `map`, and the branch switched to it (`adc6c23`).
5. The same machine then showed its limits: one binary, three runs in a row, gave hit
   times of 631, 729 and 776 ms (10 to 20% apart), when the choice turns on 5 to 12%.

The whole story, with every table, is in `root_map_benchmark_unordered_dense_DRAFT.md`, the
draft of the doc for the unordered_dense outcome. `doc/root_map_benchmark.md` still
describes CoordMap as the choice (`d6b4d88`).

## What the VM runs say, and how much to trust it

After the fix (`results_vm/rerun_analysis.md`), relative to CoordMap:

| | per category | total time | every time (the rule) | end to end |
|---|---|---|---|---|
| unordered_dense `map`, built into Bonxai | 0.84 | 0.88 | 0.85 | 1.0499 |
| unordered_dense `segmented_map`, built into Bonxai | 0.94 | 0.98 | 0.95 | 1.12 |
| boost `unordered_flat_map`, harness | 0.91 | 1.03 | 0.88 | 1.02 |
| boost `unordered_node_map`, harness | 0.96 | 1.05 | 0.92 | 1.05 |

- **Solid**: unordered_dense finds missing keys 2 to 4 times faster than CoordMap; both
  are far faster than `std::unordered_map` (0.2 to 0.7 of `main`'s times on large maps).
- **Probably real**: CoordMap finds cells that are there faster, 0 to 24% (every run
  agreed on the direction); the end to end queries are 7 to 13% slower with
  unordered_dense, while ray casting and `insertPointCloud()` are even or faster.
- **Not settled**: the overall verdict. The end to end check passed by 0.01%. Without the
  allocator category, which measures the VM's page faults as much as the maps, the total
  time advantage of unordered_dense's map shrinks from 12% to 5%.

## Open issues

1. **Decide on bare metal**: run `harness/run_all.sh` (below) and apply the rule.
2. **The accessors' check may cost.** unordered_dense moves InnerGrids when it grows or
   erases, so `ConstAccessor::cachedRoot()` checks the cached root (same array, position in
   range, same key) before using it; this also covers code that changes `rootMap()`
   directly, and `AccessorsSurviveGrowth` / `AccessorsSurviveChangesThroughRootMap` catch
   its absence under the address sanitizer. On the VM, hits looked 10 to 25% slower than
   with the prototype measured in the rerun, which bumped an epoch on growth instead, but
   within the VM's noise. The bare metal run measures `real_this`, with the check. If it
   costs, a cheaper design: a generation counter bumped by every change of the map (a thin
   wrapper around it), compared once on the slow path.
3. **The doc**: replace `doc/root_map_benchmark.md` with the draft, completed with the bare
   metal numbers, or keep it if CoordMap wins.
4. **PR #70's description** still gives the retracted 2.7x as the reason to keep CoordMap.
5. **NanoVDB**: `benchmark_nanovdb` has to be rerun with the final code (each benchmark in
   a process of its own: in one process, the state the allocator was left in moves the
   creation timings by up to 2.5x).

## Running the benchmark on a desktop

Needs: Linux, g++ 11 or later, python3, git, Eigen 3 (`libeigen3-dev`), network access for
the setup. About 1 GB of disk, 4 GB of free memory.

Prepare the machine, as far as it allows:

```bash
sudo cpupower frequency-set -g performance          # or the desktop's performance mode
echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo   # Intel; AMD: boost off in BIOS
# close browsers, IDEs, indexers; plug a laptop in
```

Then, from the repository at this branch:

```bash
doc/root_map_study/harness/setup.sh      # dependencies and worktrees, next to the repository
doc/root_map_study/harness/build.py      # 10 binaries
ROOT_MAP_CORES="2 4 6 8" ROOT_MAP_ROUNDS=8 doc/root_map_study/harness/run_all.sh
```

`ROOT_MAP_WORK` moves the working directory (default: `../bonxai_root_map_work` next to the
repository). `ROOT_MAP_CORES` are the cores to run on, one runner each, sharing the rounds:
physical cores with nothing else on them; on a CPU with hyperthreads, leave their siblings
idle; on a hybrid CPU, performance cores only. The runners share the last level cache and
the memory, which adds noise but, every variant meeting the same neighbours, no bias. One
core and 5 rounds take one to two hours; the run resumes where
it stopped if interrupted. It ends with `$ROOT_MAP_WORK/results/analysis.md`: the tables,
the rule applied, `main` against CoordMap against this branch, and the growth of a map of
970k roots. Send back that file with `final.jsonl` and `growth.jsonl` of the same
directory, and the CPU model.

What runs, each in a process of its own, pinned, in a new random order every round:

| Variant | What |
|---|---|
| `coordmap` | the harness's copy of `VoxelGrid` with CoordMap |
| `akmap_inl_pack` | unordered_dense `map`, InnerGrids in the map |
| `akseg_pack` | unordered_dense `segmented_map` |
| `akmap_up_pmxA`, `akmap_raw_pack` | unordered_dense `map` holding `unique_ptr` / raw pointers |
| `bflat192_inl_pack`, `bnode192_pack` | boost 1.92 `unordered_flat_map` (InnerGrids in the map), `unordered_node_map` |
| `real_this` | the headers of this branch: unordered_dense `map` |
| `real_coordmap` | the headers at `d6b4d88`: CoordMap |
| `real_main` | the headers of main at `8d5904f`: `std::unordered_map` |

The `real_*` variants cannot run the map alone and size workloads; their failures there are
expected.

## This directory

| Path | What |
|---|---|
| `DECISION_RULE.md` | the rule, and its history |
| `root_map_benchmark_unordered_dense_DRAFT.md` | the doc for the unordered_dense outcome, VM numbers |
| `harness/bench.cpp` | the workloads; `growth<N>k` times every insertion |
| `harness/maps.hpp`, `harness/include/` | every map behind one interface, the templated copy of `VoxelGrid` |
| `harness/setup.sh`, `build.py`, `run_all.sh`, `run.py`, `analyze.py` | as above |
| `results_vm/` | the VM rerun after the fix: raw runs and analysis |

The full harness of the 98 configurations (13 more libraries) is not here: the report of
the comparison publishes it, with the raw results of every run.
