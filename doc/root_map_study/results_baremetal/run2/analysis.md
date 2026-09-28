# The harness, relative to CoordMap

| |  | **per category** | **total time** | every time |
|---|---|---|---|

Without the allocator category:

| |  | **per category** | **total time** | every time |
|---|---|---|---|

# Built into Bonxai, relative to CoordMap in Bonxai

| | large maps | key patterns | small maps | end to end | allocator | **per category** | **total time** | every time |
|---|---|---|---|---|---|---|---|---|
| unordered_dense map, in Bonxai (this branch) | 0.91 | 0.80 | 0.84 | 1.08 | 0.94 | **0.91** | 0.97 | 0.89 |
| unordered_dense segmented_map, in Bonxai | 0.94 | 0.82 | 0.85 | 1.08 | 1.15 | **0.96** | 0.99 | 0.92 |
| CoordMap, in Bonxai (d6b4d88) | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 |
| std::unordered_map, in Bonxai (main) | 2.07 | 2.21 | 1.37 | 1.33 | 1.72 | **1.70** | 1.59 | 1.78 |

Without the allocator category:

| | large maps | key patterns | small maps | end to end | **per category** | **total time** | every time |
|---|---|---|---|---|---|---|---|
| unordered_dense map, in Bonxai (this branch) | 0.91 | 0.80 | 0.84 | 1.08 | **0.90** | 0.98 | 0.89 |
| unordered_dense segmented_map, in Bonxai | 0.94 | 0.82 | 0.85 | 1.08 | **0.91** | 0.98 | 0.90 |
| CoordMap, in Bonxai (d6b4d88) | 1.00 | 1.00 | 1.00 | 1.00 | **1.00** | 1.00 | 1.00 |
| std::unordered_map, in Bonxai (main) | 2.07 | 2.21 | 1.37 | 1.33 | **1.70** | 1.64 | 1.79 |

## The rule, applied


Rule 4, keep CoordMap only if at least 5% better than every other: yes

Built into Bonxai, unordered_dense map, in Bonxai (this branch) against CoordMap: every time 0.892, per category 0.910, total time 0.974, end to end 1.0811
It does NOT confirm a switch to it (score < 1.05 and end to end <= 1.05).

Built into Bonxai, unordered_dense segmented_map, in Bonxai against CoordMap: every time 0.919, per category 0.958, total time 0.990, end to end 1.0791
It does NOT confirm a switch to it (score < 1.05 and end to end <= 1.05).

# main, CoordMap and this branch, every time, medians in ms (ratio to main)

| workload | time | std::unordered_map, in Bonxai (main) | CoordMap, in Bonxai (d6b4d88) | unordered_dense map, in Bonxai (this branch) | unordered_dense segmented_map, in Bonxai |
|---|---|---|---|---|---|
| wide | build | 181.2 | 91.78 (0.51) | 85.98 (0.47) | 77.08 (0.43) |
| wide | hit | 41.55 | 27.54 (0.66) | 28.43 (0.68) | 29.57 (0.71) |
| wide | miss | 29.83 | 8.146 (0.27) | 7.06 (0.24) | 7.998 (0.27) |
| wide | update | 49.11 | 29.97 (0.61) | 29.11 (0.59) | 29.78 (0.61) |
| wide | iter | 45.7 | 26.73 (0.58) | 27.5 (0.60) | 28.86 (0.63) |
| wide | clear | 149.6 | 60.3 (0.40) | 53.33 (0.36) | 55.28 (0.37) |
| wide1m | build | 710 | 325.8 (0.46) | 282.6 (0.40) | 273.8 (0.39) |
| wide1m | hit | 148.8 | 92.18 (0.62) | 106.2 (0.71) | 113.2 (0.76) |
| wide1m | miss | 95.12 | 40.8 (0.43) | 23.55 (0.25) | 27.56 (0.29) |
| wide1m | update | 183.5 | 100.8 (0.55) | 109.3 (0.60) | 112 (0.61) |
| wide1m | iter | 166.6 | 81.32 (0.49) | 86.34 (0.52) | 89.92 (0.54) |
| wide1m | clear | 577.3 | 190.8 (0.33) | 172.9 (0.30) | 174.7 (0.30) |
| vg_rand_250000 | build | 150.3 | 69.28 (0.46) | 62.69 (0.42) | 60.84 (0.40) |
| vg_rand_250000 | hit | 165.5 | 123.5 (0.75) | 152.7 (0.92) | 154.1 (0.93) |
| vg_rand_250000 | miss | 83.55 | 29.23 (0.35) | 11.26 (0.13) | 11.95 (0.14) |
| vg_rand_250000 | iter | 36.64 | 18.35 (0.50) | 19.61 (0.54) | 20.18 (0.55) |
| widepos | build | 171.6 | 91.51 (0.53) | 86.14 (0.50) | 76.96 (0.45) |
| widepos | hit | 39.94 | 27.54 (0.69) | 28.51 (0.71) | 29.74 (0.74) |
| widepos | miss | 25.63 | 8.403 (0.33) | 6.982 (0.27) | 8.025 (0.31) |
| vg_dense_250000 | build | 183.1 | 70.36 (0.38) | 62.92 (0.34) | 61.25 (0.33) |
| vg_dense_250000 | hit | 197.2 | 123.9 (0.63) | 151.9 (0.77) | 154.1 (0.78) |
| vg_dense_250000 | miss | 78.57 | 26.54 (0.34) | 11.13 (0.14) | 11.92 (0.15) |
| vg_plane_250000 | build | 158 | 69.59 (0.44) | 62.74 (0.40) | 60.87 (0.39) |
| vg_plane_250000 | hit | 171.5 | 122.5 (0.71) | 150.1 (0.88) | 153.6 (0.90) |
| vg_plane_250000 | miss | 87.26 | 26.09 (0.30) | 11.04 (0.13) | 11.9 (0.14) |
| vg_line_250000 | build | 142.9 | 67.69 (0.47) | 62.71 (0.44) | 60.97 (0.43) |
| vg_line_250000 | hit | 160.5 | 121.7 (0.76) | 151.7 (0.94) | 153.3 (0.96) |
| vg_line_250000 | miss | 72.17 | 15.89 (0.22) | 11.15 (0.15) | 12.08 (0.17) |
| vg_stride_250000 | build | 151 | 69.66 (0.46) | 62.55 (0.41) | 61.09 (0.40) |
| vg_stride_250000 | hit | 164.7 | 124.2 (0.75) | 151.7 (0.92) | 153.7 (0.93) |
| vg_stride_250000 | miss | 83.67 | 29.71 (0.36) | 11.24 (0.13) | 11.81 (0.14) |
| vg_rand_1000 | build | 0.2585 | 0.2203 (0.85) | 0.2168 (0.84) | 0.201 (0.78) |
| vg_rand_1000 | hit | 28.55 | 18.2 (0.64) | 18.04 (0.63) | 18.75 (0.66) |
| vg_rand_1000 | miss | 37.15 | 19.95 (0.54) | 6.944 (0.19) | 6.51 (0.18) |
| vg_rand_1000 | iter | 0.02179 | 0.02406 (1.10) | 0.0161 (0.74) | 0.01733 (0.80) |
| vg_rand_10000 | build | 3.047 | 2.571 (0.84) | 2.247 (0.74) | 2.075 (0.68) |
| vg_rand_10000 | hit | 64.37 | 47.44 (0.74) | 49.7 (0.77) | 50.92 (0.79) |
| vg_rand_10000 | miss | 45.88 | 14.56 (0.32) | 7.194 (0.16) | 6.926 (0.15) |
| vg_rand_10000 | iter | 0.6542 | 0.4395 (0.67) | 0.4373 (0.67) | 0.458 (0.70) |
| room | build | 0.7876 | 0.7544 (0.96) | 0.7321 (0.93) | 0.7807 (0.99) |
| room | scan | 0.4649 | 0.435 (0.94) | 0.4448 (0.96) | 0.4584 (0.99) |
| room | shuffled | 1.905 | 1.309 (0.69) | 1.601 (0.84) | 1.645 (0.86) |
| room | update | 0.4267 | 0.3742 (0.88) | 0.3764 (0.88) | 0.3838 (0.90) |
| rays10 | build | 2090 | 2002 (0.96) | 2001 (0.96) | 1999 (0.96) |
| rays10 | randq | 90.12 | 46.73 (0.52) | 52.88 (0.59) | 53.75 (0.60) |
| rays10 | endq | 47.23 | 33.64 (0.71) | 40.44 (0.86) | 40.25 (0.85) |
| pmap | insert | 4430 | 4073 (0.92) | 4165 (0.94) | 4194 (0.95) |
| pmap | query | 167.2 | 119.7 (0.72) | 139.2 (0.83) | 135.1 (0.81) |
| pmap5 | insert | 4667 | 4253 (0.91) | 4323 (0.93) | 4365 (0.94) |
| pmap5 | query | 202.6 | 130.5 (0.64) | 137 (0.68) | 135.6 (0.67) |
| roomcreate_default | create | 1.169 | 1.074 (0.92) | 1.031 (0.88) | 1.093 (0.93) |
| wide_default | coldbuild | 556.7 | 460.9 (0.83) | 459 (0.82) | 435.2 (0.78) |
| wide_default | build | 319.1 | 121.4 (0.38) | 107.5 (0.34) | 241.9 (0.76) |
| wide_default | clear | 150.1 | 59.59 (0.40) | 55.08 (0.37) | 55.11 (0.37) |

## Growth to 970k roots, every insertion timed, medians

| | malloc | first build, ms | build, ms | worst insertion, first build, ms | worst insertion, ms |
|---|---|---|---|---|---|
| CoordMap, in Bonxai (d6b4d88) | memory kept | 1580 | 384 | 20.8 | 11.2 |
| unordered_dense map, in Bonxai (this branch) | memory kept | 1533 | 340 | 29.4 | 12.8 |
| unordered_dense segmented_map, in Bonxai | memory kept | 1528 | 334 | 18.1 | 14.7 |
| std::unordered_map, in Bonxai (main) | memory kept | 2022 | 775 | 98.4 | 93.9 |
| CoordMap, in Bonxai (d6b4d88) | glibc defaults | 1615 | 384 | 23.6 | 10.3 |
| unordered_dense map, in Bonxai (this branch) | glibc defaults | 1600 | 341 | 33.0 | 12.7 |
| unordered_dense segmented_map, in Bonxai | glibc defaults | 1561 | 333 | 18.5 | 14.8 |
| std::unordered_map, in Bonxai (main) | glibc defaults | 2049 | 790 | 98.4 | 94.4 |
