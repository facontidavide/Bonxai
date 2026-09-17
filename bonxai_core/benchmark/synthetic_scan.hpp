/*
 * Copyright Contributors to the Bonxai Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "bonxai/grid_coord.hpp"

namespace Bonxai {

/**
 * Synthetic 3D lidar scan, used by the benchmarks that should not depend on the
 * git-lfs .pcd files in data/.
 *
 * A sensor at @a origin inside a closed box room with a few vertical pillars.
 * The ring / column layout and the vertical field of view match a 64 beam
 * spinning lidar, which is the shape of the data bonxai_map is normally fed.
 */
inline std::vector<Point3D> MakeSyntheticScan(
    const Point3D& origin, int rings = 64, int columns = 1024) {
  constexpr double room_x = 10.0;
  constexpr double room_y = 10.0;
  constexpr double room_z = 3.0;

  struct Pillar {
    double x, y, radius;
  };
  constexpr Pillar pillars[] = {
      {2.5, 3.0, 0.4}, {-3.0, 1.5, 0.6}, {1.0, -3.5, 0.3}, {-2.0, -2.5, 0.5}};

  std::vector<Point3D> points;
  points.reserve(static_cast<size_t>(rings) * columns);

  for (int ring = 0; ring < rings; ++ring) {
    // vertical field of view: -22.5 to +22.5 degrees
    const double pitch = (-22.5 + 45.0 * ring / double(rings - 1)) * M_PI / 180.0;
    for (int column = 0; column < columns; ++column) {
      const double yaw = 2.0 * M_PI * column / double(columns);
      const double dx = std::cos(pitch) * std::cos(yaw);
      const double dy = std::cos(pitch) * std::sin(yaw);
      const double dz = std::sin(pitch);

      // distance to the walls of the room
      double range = std::numeric_limits<double>::infinity();
      auto hit_slab = [&](double dir, double from, double lo, double hi) {
        if (std::abs(dir) < 1e-9) {
          return;
        }
        const double t = std::max((lo - from) / dir, (hi - from) / dir);
        if (t > 0.0 && t < range) {
          range = t;
        }
      };
      hit_slab(dx, origin.x, -room_x, room_x);
      hit_slab(dy, origin.y, -room_y, room_y);
      hit_slab(dz, origin.z, 0.0, room_z);

      // closer hits on the pillars (ray against a circle in the XY plane)
      for (const auto& pillar : pillars) {
        const double ox = origin.x - pillar.x;
        const double oy = origin.y - pillar.y;
        const double a = dx * dx + dy * dy;
        if (a < 1e-12) {
          continue;
        }
        const double b = 2.0 * (ox * dx + oy * dy);
        const double c = ox * ox + oy * oy - pillar.radius * pillar.radius;
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) {
          continue;
        }
        const double t = (-b - std::sqrt(discriminant)) / (2.0 * a);
        if (t > 0.0 && t < range) {
          const double z = origin.z + dz * t;
          if (z > 0.0 && z < room_z) {
            range = t;
          }
        }
      }

      if (std::isfinite(range)) {
        points.push_back({origin.x + dx * range, origin.y + dy * range, origin.z + dz * range});
      }
    }
  }
  return points;
}

}  // namespace Bonxai
