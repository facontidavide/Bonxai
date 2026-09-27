# Harness, relative to CoordMap

| | large maps | key patterns | small maps | map alone | end to end | allocator | size | **per category** | **total time** | every time | every time, per round |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ankerl::unordered_dense::map 5.1.0, values inline | 0.90 | 0.82 | 0.88 | 0.65 | 1.09 | 0.77 | 0.77 | **0.83** | 0.89 | 0.79 | 0.78, 0.78, 0.80 |
| ankerl::unordered_dense::segmented_map 5.1.0 | 0.99 | 0.88 | 0.91 | 0.71 | 1.11 | 0.90 | 0.72 | **0.88** | 0.91 | 0.81 | 0.82, 0.79, 0.84 |
| ankerl::unordered_dense::map 5.1.0, unique_ptr | 1.00 | 0.84 | 0.87 | 0.79 | 1.10 | 0.80 | 0.81 | **0.88** | 0.93 | 0.85 | 0.85, 0.86, 0.87 |
| ankerl::unordered_dense::map 5.1.0, raw pointer | 1.01 | 0.83 | 0.86 | 0.81 | 1.09 | 0.83 | 0.80 | **0.89** | 0.93 | 0.86 | 0.85, 0.86, 0.88 |
| boost::unordered_flat_map 1.92, values inline | 1.07 | 0.74 | 0.87 | 0.92 | 1.02 | 0.98 | 0.80 | **0.91** | 1.03 | 0.88 | 0.89, 0.85, 0.90 |
| boost::unordered_node_map 1.92 | 1.24 | 0.79 | 0.89 | 0.99 | 1.05 | 1.07 | 0.77 | **0.96** | 1.05 | 0.92 | 0.91, 0.91, 0.93 |
| CoordMap | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 | 1.00, 1.00, 1.00 |

# Built into Bonxai, relative to CoordMap in Bonxai

| | large maps | key patterns | small maps | end to end | allocator | **per category** | **total time** | every time | every time, per round |
|---|---|---|---|---|---|---|---|---|---|
| unordered_dense map, in Bonxai | 0.86 | 0.79 | 0.84 | 1.05 | 0.69 | **0.84** | 0.88 | 0.85 | 0.86, 0.87, 0.84 |
| unordered_dense segmented_map, in Bonxai | 0.97 | 0.94 | 0.92 | 1.12 | 0.77 | **0.94** | 0.98 | 0.95 | 0.96, 0.95, 0.96 |
| CoordMap, in Bonxai | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 | 1.00, 1.00, 1.00 |

| relative to CoordMap | unordered_dense segmented_map, in Bonxai | unordered_dense map, in Bonxai |
|---|---|---|
| build | 0.86 to 0.96 | 0.74 to 1.03 |
| hit | 1.11 to 1.54 | 0.98 to 1.24 |
| miss | 0.40 to 1.01 | 0.36 to 0.73 |
| update | 1.05 to 1.22 | 1.03 to 1.09 |
| iter | 0.90 to 1.04 | 0.79 to 1.00 |
| clear | 0.81 to 0.81 | 0.78 to 0.78 |

## The rule, applied

Harness, every time at once and end to end, relative to CoordMap:
- ankerl::unordered_dense::map 5.1.0, values inline: 0.786, end to end 1.092: score <= 0.95, but end to end > 1.05
- ankerl::unordered_dense::segmented_map 5.1.0: 0.810, end to end 1.107: score <= 0.95, but end to end > 1.05
- ankerl::unordered_dense::map 5.1.0, unique_ptr: 0.854, end to end 1.098: score <= 0.95, but end to end > 1.05
- ankerl::unordered_dense::map 5.1.0, raw pointer: 0.857, end to end 1.086: score <= 0.95, but end to end > 1.05
- boost::unordered_flat_map 1.92, values inline: 0.882, end to end 1.016: switch (<= 0.95, end to end <= 1.05)
- boost::unordered_node_map 1.92: 0.917, end to end 1.049: switch (<= 0.95, end to end <= 1.05)

Rule 4, keep CoordMap only if at least 5% better than every other: no

Built into Bonxai, relative to CoordMap in Bonxai:
- unordered_dense segmented_map, in Bonxai: every time 0.954, per category 0.938, total time 0.975, end to end 1.116
- unordered_dense map, in Bonxai: every time 0.850, per category 0.840, total time 0.881, end to end 1.050

flat map at least 5% better than segmented_map, in Bonxai: yes (0.891)

**The rule picks: unordered_dense map**

## Growth to 970k roots: every insertion timed, medians of the rounds

| | mode | cold build, ms | warm build, ms | worst insertion cold, ms | worst insertion warm, ms | faults, warm build |
|---|---|---|---|---|---|---|
| CoordMap | memory kept | 2068 | 658 | 21.3 | 12.6 | 2 |
| ankerl::unordered_dense::map 5.1.0, values inline | memory kept | 1963 | 670 | 36.2 | 21.7 | 0 |
| ankerl::unordered_dense::segmented_map 5.1.0 | memory kept | 2047 | 611 | 35.1 | 28.1 | 4 |
| ankerl::unordered_dense::map 5.1.0, raw pointer | memory kept | 1787 | 633 | 12.4 | 10.4 | 1 |
| ankerl::unordered_dense::map 5.1.0, unique_ptr | memory kept | 1850 | 660 | 13.3 | 10.6 | 1 |
| boost::unordered_flat_map 1.92, values inline | memory kept | 3128 | 832 | 107.0 | 50.5 | 1 |
| boost::unordered_node_map 1.92 | memory kept | 2219 | 799 | 72.1 | 73.6 | 1 |
| CoordMap, in Bonxai | memory kept | 2079 | 675 | 21.8 | 13.3 | 2 |
| unordered_dense segmented_map, in Bonxai | memory kept | 2510 | 621 | 35.8 | 30.3 | 4 |
| unordered_dense map, in Bonxai | memory kept | 1788 | 635 | 36.5 | 19.5 | 0 |
| CoordMap | glibc defaults | 2020 | 678 | 24.5 | 12.6 | 221 |
| ankerl::unordered_dense::map 5.1.0, values inline | glibc defaults | 1861 | 639 | 42.3 | 19.3 | 221 |
| ankerl::unordered_dense::segmented_map 5.1.0 | glibc defaults | 1877 | 677 | 31.7 | 28.6 | 221 |
| ankerl::unordered_dense::map 5.1.0, raw pointer | glibc defaults | 1783 | 616 | 13.3 | 9.9 | 220 |
| ankerl::unordered_dense::map 5.1.0, unique_ptr | glibc defaults | 1746 | 715 | 13.1 | 10.6 | 220 |
| boost::unordered_flat_map 1.92, values inline | glibc defaults | 2729 | 899 | 110.4 | 99.6 | 58113 |
| boost::unordered_node_map 1.92 | glibc defaults | 2063 | 911 | 85.2 | 75.0 | 220 |
| CoordMap, in Bonxai | glibc defaults | 1958 | 701 | 24.4 | 13.0 | 221 |
| unordered_dense segmented_map, in Bonxai | glibc defaults | 1834 | 668 | 33.4 | 30.8 | 221 |
| unordered_dense map, in Bonxai | glibc defaults | 1871 | 593 | 41.1 | 19.3 | 221 |
