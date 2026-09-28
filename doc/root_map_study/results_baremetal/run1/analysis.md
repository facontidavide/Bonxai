# The harness, relative to CoordMap

| | large maps | key patterns | small maps | map alone | end to end | allocator | size | **per category** | **total time** | every time |
|---|---|---|---|---|---|---|---|---|---|---|
| unordered_dense map, InnerGrids in the map | 0.86 | 0.76 | 0.85 | 0.61 | 1.05 | 0.98 | 0.69 | **0.82** | 0.88 | 0.74 |
| boost unordered_flat_map 1.92, InnerGrids in the map | 1.02 | 0.62 | 0.83 | 0.78 | 1.00 | 1.20 | 0.60 | **0.84** | 0.94 | 0.75 |
| unordered_dense segmented_map | 0.92 | 0.81 | 0.85 | 0.65 | 1.07 | 1.15 | 0.65 | **0.85** | 0.90 | 0.75 |
| unordered_dense map, raw pointer | 0.96 | 0.77 | 0.86 | 0.74 | 1.06 | 1.02 | 0.69 | **0.86** | 0.93 | 0.79 |
| unordered_dense map, unique_ptr | 0.91 | 0.81 | 0.86 | 0.75 | 1.09 | 1.05 | 0.72 | **0.88** | 0.94 | 0.80 |
| boost unordered_node_map 1.92 | 1.06 | 0.73 | 0.89 | 0.84 | 1.02 | 1.35 | 0.62 | **0.90** | 0.97 | 0.80 |
| CoordMap | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 |

Without the allocator category:

| | large maps | key patterns | small maps | map alone | end to end | size | **per category** | **total time** | every time |
|---|---|---|---|---|---|---|---|---|---|
| unordered_dense map, InnerGrids in the map | 0.86 | 0.76 | 0.85 | 0.61 | 1.05 | 0.69 | **0.79** | 0.87 | 0.73 |
| boost unordered_flat_map 1.92, InnerGrids in the map | 1.02 | 0.62 | 0.83 | 0.78 | 1.00 | 0.60 | **0.79** | 0.93 | 0.74 |
| unordered_dense segmented_map | 0.92 | 0.81 | 0.85 | 0.65 | 1.07 | 0.65 | **0.81** | 0.88 | 0.74 |
| unordered_dense map, raw pointer | 0.96 | 0.77 | 0.86 | 0.74 | 1.06 | 0.69 | **0.84** | 0.93 | 0.79 |
| boost unordered_node_map 1.92 | 1.06 | 0.73 | 0.89 | 0.84 | 1.02 | 0.62 | **0.85** | 0.95 | 0.79 |
| unordered_dense map, unique_ptr | 0.91 | 0.81 | 0.86 | 0.75 | 1.09 | 0.72 | **0.85** | 0.93 | 0.80 |
| CoordMap | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 |

# Built into Bonxai, relative to CoordMap in Bonxai

| | large maps | key patterns | small maps | end to end | allocator | **per category** | **total time** | every time |
|---|---|---|---|---|---|---|---|---|
| unordered_dense map, in Bonxai (this branch) | 0.86 | 0.80 | 0.84 | 1.06 | 0.95 | **0.90** | 0.97 | 0.87 |
| CoordMap, in Bonxai (d6b4d88) | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 |
| std::unordered_map, in Bonxai (main) | 1.94 | 2.31 | 1.39 | 1.32 | 1.69 | **1.70** | 1.58 | 1.77 |

Without the allocator category:

| | large maps | key patterns | small maps | end to end | **per category** | **total time** | every time |
|---|---|---|---|---|---|---|---|
| unordered_dense map, in Bonxai (this branch) | 0.86 | 0.80 | 0.84 | 1.06 | **0.88** | 0.96 | 0.87 |
| CoordMap, in Bonxai (d6b4d88) | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 |
| std::unordered_map, in Bonxai (main) | 1.94 | 2.31 | 1.39 | 1.32 | **1.70** | 1.62 | 1.77 |

## The rule, applied

- unordered_dense map, InnerGrids in the map: every time 0.738, end to end 1.0472: passes: score <= 0.95, end to end <= 1.05
- unordered_dense segmented_map: every time 0.750, end to end 1.0682: score <= 0.95, but end to end > 1.05
- boost unordered_flat_map 1.92, InnerGrids in the map: every time 0.754, end to end 0.9973: passes: score <= 0.95, end to end <= 1.05
- unordered_dense map, raw pointer: every time 0.792, end to end 1.0643: score <= 0.95, but end to end > 1.05
- boost unordered_node_map 1.92: every time 0.800, end to end 1.0179: passes: score <= 0.95, end to end <= 1.05
- unordered_dense map, unique_ptr: every time 0.803, end to end 1.0948: score <= 0.95, but end to end > 1.05

Rule 4, keep CoordMap only if at least 5% better than every other: no
unordered_dense map against segmented_map, every time: 0.984 (the map is preferred only at 0.95 or less)

Built into Bonxai, unordered_dense's map against CoordMap: every time 0.871, per category 0.898, total time 0.968, end to end 1.0562
It does NOT confirm a switch to it (score < 1.05 and end to end <= 1.05).

# main, CoordMap and this branch, every time, medians in ms (ratio to main)

| workload | time | std::unordered_map, in Bonxai (main) | CoordMap, in Bonxai (d6b4d88) | unordered_dense map, in Bonxai (this branch) |
|---|---|---|---|---|
| wide | build | 225.2 | 124 (0.55) | 106 (0.47) |
| wide | hit | 47.25 | 33.83 (0.72) | 33.5 (0.71) |
| wide | miss | 34.25 | 12.47 (0.36) | 8.709 (0.25) |
| wide | update | 60.51 | 41.36 (0.68) | 35.97 (0.59) |
| wide | iter | 52.61 | 31.69 (0.60) | 32.39 (0.62) |
| wide | clear | 182.7 | 74.77 (0.41) | 59.86 (0.33) |
| wide1m | build | 784.7 | 361.2 (0.46) | 343 (0.44) |
| wide1m | hit | 155.9 | 110.8 (0.71) | 120.2 (0.77) |
| wide1m | miss | 101.4 | 49.06 (0.48) | 28.35 (0.28) |
| wide1m | update | 195.7 | 111.7 (0.57) | 119.6 (0.61) |
| wide1m | iter | 170.7 | 100.8 (0.59) | 91.03 (0.53) |
| wide1m | clear | 604.1 | 224.4 (0.37) | 175.9 (0.29) |
| vg_rand_250000 | build | 174.7 | 78.5 (0.45) | 58.67 (0.34) |
| vg_rand_250000 | hit | 183.6 | 153.7 (0.84) | 190.4 (1.04) |
| vg_rand_250000 | miss | 92.84 | 31.18 (0.34) | 12.19 (0.13) |
| vg_rand_250000 | iter | 41.61 | 19.65 (0.47) | 20.09 (0.48) |
| widepos | build | 185.5 | 97.33 (0.52) | 88.91 (0.48) |
| widepos | hit | 43.41 | 29.9 (0.69) | 30.23 (0.70) |
| widepos | miss | 27.96 | 9.157 (0.33) | 7.617 (0.27) |
| vg_dense_250000 | build | 200 | 77.32 (0.39) | 65.81 (0.33) |
| vg_dense_250000 | hit | 221.9 | 131.2 (0.59) | 157 (0.71) |
| vg_dense_250000 | miss | 92.68 | 26.86 (0.29) | 11.4 (0.12) |
| vg_plane_250000 | build | 173.8 | 75.14 (0.43) | 66.39 (0.38) |
| vg_plane_250000 | hit | 183.2 | 139.1 (0.76) | 180.4 (0.98) |
| vg_plane_250000 | miss | 91.6 | 27.3 (0.30) | 12.42 (0.14) |
| vg_line_250000 | build | 187.9 | 82.74 (0.44) | 76.93 (0.41) |
| vg_line_250000 | hit | 229.9 | 158.4 (0.69) | 182.5 (0.79) |
| vg_line_250000 | miss | 98.28 | 17.37 (0.18) | 12.13 (0.12) |
| vg_stride_250000 | build | 170.5 | 77.32 (0.45) | 68.54 (0.40) |
| vg_stride_250000 | hit | 180.6 | 138.1 (0.76) | 172.7 (0.96) |
| vg_stride_250000 | miss | 89.1 | 31.3 (0.35) | 12.19 (0.14) |
| vg_rand_1000 | build | 0.26 | 0.2209 (0.85) | 0.214 (0.82) |
| vg_rand_1000 | hit | 28.75 | 18.54 (0.64) | 18.18 (0.63) |
| vg_rand_1000 | miss | 37.43 | 20.13 (0.54) | 6.905 (0.18) |
| vg_rand_1000 | iter | 0.02528 | 0.02471 (0.98) | 0.01591 (0.63) |
| vg_rand_10000 | build | 3.218 | 2.65 (0.82) | 2.442 (0.76) |
| vg_rand_10000 | hit | 76.4 | 57.39 (0.75) | 58.43 (0.76) |
| vg_rand_10000 | miss | 46.39 | 14.75 (0.32) | 7.262 (0.16) |
| vg_rand_10000 | iter | 0.8666 | 0.4699 (0.54) | 0.453 (0.52) |
| room | build | 0.8343 | 0.7712 (0.92) | 0.7886 (0.95) |
| room | scan | 0.4716 | 0.4437 (0.94) | 0.4541 (0.96) |
| room | shuffled | 1.934 | 1.342 (0.69) | 1.708 (0.88) |
| room | update | 0.4664 | 0.4637 (0.99) | 0.4628 (0.99) |
| rays10 | build | 2108 | 2017 (0.96) | 2011 (0.95) |
| rays10 | randq | 89.84 | 47.68 (0.53) | 52.27 (0.58) |
| rays10 | endq | 46.02 | 33.54 (0.73) | 38.79 (0.84) |
| pmap | insert | 4470 | 4090 (0.92) | 4152 (0.93) |
| pmap | query | 174.4 | 118.8 (0.68) | 134.3 (0.77) |
| pmap5 | insert | 4714 | 4347 (0.92) | 4402 (0.93) |
| pmap5 | query | 211.7 | 140.3 (0.66) | 140.1 (0.66) |
| roomcreate_default | create | 1.173 | 1.109 (0.95) | 1.187 (1.01) |
| wide_default | coldbuild | 568.9 | 472.1 (0.83) | 465 (0.82) |
| wide_default | build | 322.9 | 126.3 (0.39) | 109.2 (0.34) |
| wide_default | clear | 150.7 | 59.68 (0.40) | 54.21 (0.36) |

## Growth to 970k roots, every insertion timed, medians

| | malloc | first build, ms | build, ms | worst insertion, first build, ms | worst insertion, ms |
|---|---|---|---|---|---|
| CoordMap | memory kept | 1600 | 380 | 21.1 | 11.1 |
| unordered_dense map, InnerGrids in the map | memory kept | 1535 | 332 | 29.6 | 12.4 |
| unordered_dense segmented_map | memory kept | 1533 | 321 | 17.6 | 14.2 |
| unordered_dense map, unique_ptr | memory kept | 1549 | 322 | 12.5 | 9.0 |
| unordered_dense map, raw pointer | memory kept | 1525 | 317 | 12.6 | 8.9 |
| boost unordered_flat_map 1.92, InnerGrids in the map | memory kept | 1627 | 347 | 96.1 | 21.3 |
| boost unordered_node_map 1.92 | memory kept | 1567 | 341 | 28.4 | 21.5 |
| CoordMap, in Bonxai (d6b4d88) | memory kept | 1584 | 380 | 21.1 | 11.1 |
| unordered_dense map, in Bonxai (this branch) | memory kept | 1538 | 341 | 29.2 | 12.4 |
| std::unordered_map, in Bonxai (main) | memory kept | 2027 | 761 | 97.3 | 93.0 |
| CoordMap | glibc defaults | 1624 | 379 | 23.7 | 10.4 |
| unordered_dense map, InnerGrids in the map | glibc defaults | 1595 | 336 | 32.8 | 12.3 |
| unordered_dense segmented_map | glibc defaults | 1566 | 328 | 17.9 | 14.1 |
| unordered_dense map, unique_ptr | glibc defaults | 1583 | 325 | 12.5 | 9.0 |
| unordered_dense map, raw pointer | glibc defaults | 1550 | 318 | 12.5 | 9.0 |
| boost unordered_flat_map 1.92, InnerGrids in the map | glibc defaults | 1748 | 434 | 102.3 | 95.7 |
| boost unordered_node_map 1.92 | glibc defaults | 1587 | 339 | 29.0 | 21.4 |
| CoordMap, in Bonxai (d6b4d88) | glibc defaults | 1625 | 383 | 23.7 | 10.4 |
| unordered_dense map, in Bonxai (this branch) | glibc defaults | 1608 | 333 | 33.1 | 12.5 |
| std::unordered_map, in Bonxai (main) | glibc defaults | 2056 | 763 | 100.5 | 93.1 |
