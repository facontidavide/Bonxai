#include <gtest/gtest.h>

#include <unordered_set>
#include <vector>

#include "bonxai_map/probabilistic_map.hpp"

using Bonxai::CoordT;
using Bonxai::ProbabilisticMap;

namespace {

const ProbabilisticMap::Options kDefaults{};
const int32_t kHitLog = kDefaults.prob_hit_log;
const int32_t kMissLog = kDefaults.prob_miss_log;

int32_t cellLog(ProbabilisticMap& map, int32_t x, int32_t y, int32_t z) {
  auto accessor = map.grid().createAccessor();
  const auto* cell = accessor.value({x, y, z}, false);
  return cell ? int32_t(cell->probability_log) : ProbabilisticMap::UnknownProbability;
}

using CellSet = std::unordered_set<CoordT>;

// every cell whose probability equals exactly one miss update
CellSet carvedCells(ProbabilisticMap& map) {
  CellSet out;
  map.grid().forEachCell([&](ProbabilisticMap::CellT& cell, const CoordT& coord) {
    if (cell.probability_log == kMissLog) {
      out.insert(coord);
    }
  });
  return out;
}

}  // namespace

// Regression: cells used to store a 4-bit update tag compared against a counter
// cycling 1,2,3. A cell whose last update was exactly 3 scans earlier aliased with
// the current counter and silently lost the new update - and, for hit endpoints,
// the whole carving ray of that endpoint.
TEST(ProbabilisticMap, UpdatesSurviveGapOfThreeScans) {
  ProbabilisticMap map(1.0);
  const Eigen::Vector3d origin(0.5, 0.5, 0.5);
  const std::vector<Eigen::Vector3d> scan = {{5.5, 0.5, 0.5}};
  // unrelated scan, far away from the first one
  const Eigen::Vector3d far_origin(100.5, 0.5, 0.5);
  const std::vector<Eigen::Vector3d> far_scan = {{100.5, 10.5, 0.5}};

  map.insertPointCloud(scan, origin, 999.0);           // scan 1: hit on (5,0,0)
  map.insertPointCloud(far_scan, far_origin, 999.0);   // scan 2
  map.insertPointCloud(far_scan, far_origin, 999.0);   // scan 3
  map.insertPointCloud(scan, origin, 999.0);           // scan 4: (5,0,0) again, gap of 3

  // the endpoint must accumulate both hits ...
  EXPECT_EQ(cellLog(map, 5, 0, 0), 2 * kHitLog);
  // ... and a cell along the carving ray both misses
  EXPECT_EQ(cellLog(map, 2, 0, 0), 2 * kMissLog);
}

// Within one scan a cell that is both a hit endpoint and a miss endpoint must end
// up occupied ("prefer occupied cells over free ones", like octomap), no matter in
// which order the points were added.
TEST(ProbabilisticMap, HitWinsOverMissRegardlessOfOrder) {
  const Eigen::Vector3d origin(0.5, 0.5, 0.5);
  const Eigen::Vector3d point(5.5, 0.5, 0.5);

  ProbabilisticMap miss_first(1.0);
  miss_first.addMissPoint(point);
  miss_first.addHitPoint(point);
  miss_first.updateFreeCells(origin);

  ProbabilisticMap hit_first(1.0);
  hit_first.addHitPoint(point);
  hit_first.addMissPoint(point);
  hit_first.updateFreeCells(origin);

  EXPECT_EQ(cellLog(miss_first, 5, 0, 0), kHitLog);
  EXPECT_EQ(cellLog(hit_first, 5, 0, 0), kHitLog);
}

// Free-space carving must visit EVERY voxel crossed by the segment from the sensor
// origin to the center of the endpoint voxel (endpoint excluded) - not the
// direction-dependent subset visited by an integer Bresenham line.
TEST(ProbabilisticMap, ExactRayVisitsEveryCrossedVoxel) {
  ProbabilisticMap map(1.0);
  const Eigen::Vector3d origin(0.5, 0.5, 0.5);
  const std::vector<Eigen::Vector3d> scan = {{10.5, 5.3, 0.5}};
  map.insertPointCloud(scan, origin, 999.0);

  // brute-force reference: dense sampling of the segment origin -> center of (10,5,0)
  const Eigen::Vector3d target(10.5, 5.5, 0.5);
  CellSet expected;
  const int steps = 200000;
  for (int i = 0; i <= steps; i++) {
    const Eigen::Vector3d p = origin + (target - origin) * (double(i) / steps);
    expected.insert(map.grid().posToCoord(p.x(), p.y(), p.z()));
  }
  expected.erase({10, 5, 0});  // the endpoint gets the hit, not a ray miss

  EXPECT_EQ(carvedCells(map), expected);
  // the legacy Bresenham visited only max(|dx|,|dy|,|dz|) = 10 cells
  EXPECT_GT(expected.size(), 10u);

  // the public helper exposes the same traversal
  std::vector<CoordT> ray;
  Bonxai::ComputeExactRay(origin, {0, 0, 0}, {10, 5, 0}, 1.0, ray);
  EXPECT_EQ(CellSet(ray.begin(), ray.end()), expected);
}

// The legacy integer Bresenham stays available as an explicit opt-in.
TEST(ProbabilisticMap, ApproximateModeUsesLegacyBresenham) {
  ProbabilisticMap map(1.0);
  auto options = map.options();
  options.ray_mode = ProbabilisticMap::Options::RayMode::Approximate;
  map.setOptions(options);

  const Eigen::Vector3d origin(0.5, 0.5, 0.5);
  const std::vector<Eigen::Vector3d> scan = {{10.5, 5.3, 0.5}};
  map.insertPointCloud(scan, origin, 999.0);

  std::vector<CoordT> ray;
  Bonxai::ComputeRay({0, 0, 0}, {10, 5, 0}, ray);
  EXPECT_EQ(carvedCells(map), CellSet(ray.begin(), ray.end()));
}

// Points beyond max_range carve free space up to the clamped endpoint, which gets a
// miss (not a hit); the original endpoint stays untouched.
TEST(ProbabilisticMap, MaxRangePointsCarveWithoutHit) {
  ProbabilisticMap map(1.0);
  const Eigen::Vector3d origin(0.5, 0.5, 0.5);
  const std::vector<Eigen::Vector3d> scan = {{20.5, 0.5, 0.5}};
  map.insertPointCloud(scan, origin, 10.0);

  EXPECT_EQ(cellLog(map, 10, 0, 0), kMissLog);  // clamped endpoint: one miss
  EXPECT_EQ(cellLog(map, 5, 0, 0), kMissLog);   // along the ray
  EXPECT_EQ(cellLog(map, 20, 0, 0), ProbabilisticMap::UnknownProbability);
}
