# The root map study: notes and instructions

Status on 2026-09-28, branch `claude/issue-69-qvgw41`, PR #70: **decided**. `VoxelGrid`
keeps `CoordMap`. The comparison ran on bare metal, and the rule of `DECISION_RULE.md`,
applied without override, does not let unordered_dense's maps in: built into Bonxai, they
are 6 to 8% slower end to end. `doc/root_map_benchmark.md` has the story and the numbers,
*On bare metal* the final ones.

## Where the branch stands

| Commit | What |
|---|---|
| `8d5904f` (on main) | #69: the 20 bits truncation of `std::hash<CoordT>` dropped. |
| `dea0b2d` | `std::hash<CoordT>` no longer collides on grids that cross zero. |
| `80e2cbf` | `CoordMap`, a root map written for Bonxai, replaces `std::unordered_map`. |
| `75f04f4` | `std::hash<CoordT>` reads each coordinate on its own (a 2.3x store forwarding stall). |
| `6f0fa7c`, `d6b4d88` | Doc of CoordMap, its comparison against 20 maps, a test of released roots. |
| `adc6c23` | The root map becomes ankerl::unordered_dense's `map`, vendored in `bonxai/detail`. |
| `48cd980` | This directory, and the draft doc for the unordered_dense outcome. |
| `0564a50` | unordered_dense's `map`: `VoxelGrid`'s destructor frees the nodes last to first. |
| `6866d81` | The harness runs on several cores, and runs `benchmark_nanovdb`. |
| next | Back to `CoordMap`, as at `d6b4d88`; the bare metal results. |

The code of `VoxelGrid` is `d6b4d88`'s, plus `AccessorsSurviveChangesThroughRootMap`, a
test written for unordered_dense that `CoordMap` passes too. The unordered_dense version
stays in the history: `adc6c23` and `0564a50`.

## How the choice was made, unmade, and made

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
   hold the InnerGrids have arrays that large (63 and 126 MB at 1M roots). The harness now
   sets `mallopt(M_MMAP_MAX, 0)`. Fixed, the 1M build of unordered_dense's map is 1.03x
   CoordMap's.
4. The contenders ran again on the VM, the rule applied with no override: it picked
   unordered_dense's `map`, and the branch switched to it (`adc6c23`).
5. The VM then showed its limits: one binary, three runs in a row, gave hit times 10 to
   20% apart, when the choice turns on 5 to 12%.
6. **Bare metal** (i7-13700H, turbo off, 8 rounds): in the harness, every contender scores
   0.74 to 0.80 against CoordMap, but built into Bonxai, unordered_dense's `map` is 1.06
   to 1.08 end to end, over the rule's 5%, in both runs; `segmented_map`, tried as well,
   1.08. The rule's last clause, that the code built into Bonxai confirms the harness,
   fails: CoordMap stays.
7. Along the way, `benchmark_nanovdb` showed unordered_dense's `map` creating the wide grid
   in 2x CoordMap's time: the map's destructor freed the InnerGrids first to last, and glibc
   gave the heap back to the kernel. `0564a50` fixed it, and the second run measured the
   map with the fix.

## What the bare metal runs say

Relative to `main` (`results_baremetal/run2`), lower is better:

| | CoordMap | unordered_dense `map` | `segmented_map` |
|---|---|---|---|
| every time | 0.56 | **0.50** | 0.52 |
| end to end | **0.75** | 0.81 | 0.81 |
| find a cell that is there, 250k random roots | **0.75** | 0.92 | 0.93 |
| find a cell that is not, 250k random roots | 0.35 | **0.13** | 0.14 |

- unordered_dense finds missing keys 2 to 3 times faster than CoordMap, builds 5 to 15%
  faster, and clears faster; CoordMap finds cells that are there 3 to 25% faster.
- End to end, ray casting and `insertPointCloud()` are even; their queries are 6 to 28%
  slower with unordered_dense, in 46 of 48 pairs of runs.
- Run 1 (`results_baremetal/run1`) has every harness contender too: boost's
  `unordered_flat_map`, estimated at 0.50 of `main` and 0.75 end to end, iterates and
  clears 2 times slower than the others.

## Open issues

1. **PR #70's description** still gives the retracted 2.7x as the reason to keep CoordMap.

## Running the benchmark on a desktop

Needs: Linux, g++ 11 or later, python3, git, curl, Eigen 3 (`libeigen3-dev`), Google
Benchmark (`libbenchmark-dev`, for `nanovdb.py`), network access for the setup. About 1 GB
of disk, 4 GB of free memory per core used.

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
ROOT_MAP_CORES="2 6" ROOT_MAP_ROUNDS=8 doc/root_map_study/harness/run_all.sh
doc/root_map_study/harness/nanovdb.py --cores "2 6" --rounds 5
```

`ROOT_MAP_WORK` moves the working directory (default: `../bonxai_root_map_work` next to the
repository). `ROOT_MAP_CORES` are the cores to run on, one runner each, sharing the rounds:
physical cores with nothing else on them; on a CPU with hyperthreads, leave their siblings
idle; on a hybrid CPU, performance cores only. The runners share the last level cache and
the memory: two are fine, but four changed the results of the large maps, not only their
noise (random hits went from 23% faster with CoordMap to 6% slower). Two cores and 8
rounds take about 40 minutes; the run resumes where it stopped if interrupted. It ends with
`$ROOT_MAP_WORK/results/analysis.md`: the tables, the rule applied, `main` against CoordMap
against this branch, and the growth of a map of 970k roots.

What runs, each in a process of its own, pinned, in a new random order every round:

| Variant | What |
|---|---|
| `coordmap` | the harness's copy of `VoxelGrid` with CoordMap |
| `akmap_inl_pack` | unordered_dense `map`, InnerGrids in the map |
| `akseg_pack` | unordered_dense `segmented_map` |
| `akmap_up_pmxA`, `akmap_raw_pack` | unordered_dense `map` holding `unique_ptr` / raw pointers |
| `bflat192_inl_pack`, `bnode192_pack` | boost 1.92 `unordered_flat_map` (InnerGrids in the map), `unordered_node_map` |
| `real_this` | the headers of this branch: CoordMap |
| `real_coordmap` | the headers at `d6b4d88`: CoordMap |
| `real_main` | the headers of main at `8d5904f`: `std::unordered_map` |

The `real_*` variants cannot run the map alone and size workloads; their failures there are
expected. To measure unordered_dense built into Bonxai, check `0564a50` out in a worktree
and add it to `REAL` in `build.py`.

## This directory

| Path | What |
|---|---|
| `DECISION_RULE.md` | the rule, and its history |
| `harness/bench.cpp` | the workloads; `growth<N>k` times every insertion |
| `harness/maps.hpp`, `harness/include/` | every map behind one interface, the templated copy of `VoxelGrid` |
| `harness/setup.sh`, `build.py`, `run_all.sh`, `run.py`, `analyze.py` | as above |
| `harness/nanovdb.py` | `benchmark_nanovdb` against `main`, CoordMap and this branch |
| `results_baremetal/run1` | every variant; `real_this` was unordered_dense's `map` without `0564a50`; `gunzip -k final.jsonl.gz` before `analyze.py` |
| `results_baremetal/run2` | built into Bonxai: `real_this` was unordered_dense's `map` with `0564a50`, `real_seg` its `segmented_map` |
| `results_vm/` | the VM rerun after the mmap fix: raw runs and analysis |

The draft doc for the unordered_dense outcome, with the VM's tables, is in `48cd980`. The
full harness of the 98 configurations (13 more libraries) is not here: the report of the
comparison publishes it, with the raw results of every run.
