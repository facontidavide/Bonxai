# The decision rule

Written before the first final results of the comparison were looked at, and applied
unchanged ever since. Applied to the bare metal run as well, with no override: whatever
else is found (worst insertion, memory, the guarantees each map gives) is reported next to
the result, for the maintainer to weigh, and does not change what the rule picks.

1. **Primary score**: the geometric mean of every timing ratio to CoordMap over all 21
   workloads (medians of the rounds): the *every time* column of `harness/analyze.py`.
   End to end (rays10, pmap, pmap5) is checked separately.
2. **Switch** to a vendorable library when its score is 0.95 or less and it is not more
   than 5% worse than CoordMap end to end. Prefer unordered_dense's `segmented_map`, which
   never moves an InnerGrid when inserting, unless the flat `map` is another 5% better.
3. **Within ±5%**: prefer the established library.
4. **Keep CoordMap** only if it is at least 5% better than every other.

The code built into Bonxai (`real_this` against `real_coordmap`) must confirm the harness.

## Its history

- **First application** (VM, before the fix): it picked unordered_dense's `map`. CoordMap
  was kept anyway, because that map had taken 2.7 times as long to build 1M roots. That
  number turned out to be the benchmark's: see *The mmap cap* in `README.md`.
- **Second application** (VM, after the fix, contenders only): every contender scored 0.79
  to 0.92, so CoordMap goes. Built into Bonxai, unordered_dense's `map` scored 0.85 and
  passed the end to end check by a hair, 1.0499. The branch switched to it.
- **Third application** (bare metal, i7-13700H, two runs of 8 rounds): in the harness,
  every contender scores 0.74 to 0.80, so CoordMap would go. Built into Bonxai,
  unordered_dense's `map` scores 0.87 to 0.89 but is 1.06 to 1.08 end to end, and
  `segmented_map` 0.92 and 1.08: the code built into Bonxai does not confirm the harness.
  CoordMap stays, by the rule this time.
