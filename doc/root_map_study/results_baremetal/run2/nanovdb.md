| benchmark | main | CoordMap | this | segmented | spread (max/min, this) |
|---|---|---|---|---|---|
| Update<StaticShape<>> | 0.4163 | 0.3874 (0.93) | 0.3846 (0.92) | 0.4148 (1.00) | 1.06 |
| WideCreate<StaticShape<4, 3>> | 1667 | 1543 (0.93) | 1528 (0.92) | 1548 (0.93) | 1.05 |
| Query<DynamicShape>/1 | 2.161 | 1.464 (0.68) | 1.611 (0.75) | 1.679 (0.78) | 1.03 |
| NanoVDBReadOnly_Iterate |  |  | 0.3711 |  | 1.09 |
| Create<DynamicShape> | 1.249 | 1.133 (0.91) | 1.118 (0.89) | 1.139 (0.91) | 1.03 |
| WideQuery<StaticShape<4, 3>>/1 | 33.63 | 13.29 (0.40) | 14.81 (0.44) | 14.71 (0.44) | 1.20 |
| Query<DynamicShape>/0 | 0.5047 | 0.4722 (0.94) | 0.4626 (0.92) | 0.4733 (0.94) | 1.02 |
| NanoVDB_WideCreate |  |  | 1041 |  | 1.05 |
| Query<StaticShape<>>/1 | 1.899 | 1.288 (0.68) | 1.539 (0.81) | 1.561 (0.82) | 1.03 |
| Iterate<StaticShape<>> | 0.2653 | 0.2371 (0.89) | 0.2422 (0.91) | 0.2403 (0.91) | 1.03 |
| WideQuery<StaticShape<>>/1 | 29.52 | 8.407 (0.28) | 6.404 (0.22) | 7.277 (0.25) | 1.20 |
| MemoryUsage | 10.24 | 10.25 (1.00) | 10.26 (1.00) | 10.24 (1.00) | 1.00 |
| Update<DynamicShape> | 0.5358 | 0.4433 (0.83) | 0.4572 (0.85) | 0.4683 (0.87) | 1.02 |
| Create<StaticShape<>> | 1.116 | 1.045 (0.94) | 1.039 (0.93) | 1.06 (0.95) | 1.05 |
| Iterate<DynamicShape> | 0.2767 | 0.2471 (0.89) | 0.2506 (0.91) | 0.2484 (0.90) | 1.08 |
| NanoVDB_Query/1 |  |  | 1.482 |  | 1.02 |
| WideCreate<StaticShape<>> | 422.8 | 161.8 (0.38) | 158.9 (0.38) | 143.6 (0.34) | 1.03 |
| NanoVDB_WideQuery/1 |  |  | 14.45 |  | 1.06 |
| NanoVDBReadOnly_Query/1 |  |  | 1.344 |  | 1.02 |
| Query<StaticShape<>>/0 | 0.4563 | 0.4147 (0.91) | 0.4223 (0.93) | 0.4313 (0.95) | 1.04 |
| NanoVDB_Create |  |  | 4.966 |  | 1.12 |
| WideQuery<StaticShape<4, 3>>/0 | 49.83 | 29.88 (0.60) | 32.41 (0.65) | 33.49 (0.67) | 1.06 |
| NanoVDB_WideQuery/0 |  |  | 22.66 |  | 1.09 |
| WideQuery<StaticShape<>>/0 | 42.08 | 26.68 (0.63) | 27.05 (0.64) | 28.53 (0.68) | 1.09 |
| NanoVDB_Convert |  |  | 3.28 |  | 1.02 |
| NanoVDBReadOnly_Query/0 |  |  | 0.328 |  | 1.02 |
| NanoVDB_Update |  |  | 0.6086 |  | 1.01 |
| NanoVDB_Query/0 |  |  | 0.3172 |  | 1.02 |
