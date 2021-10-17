#include <benchmark/benchmark.h>
#include <openvdb/openvdb.h>
#include "benchmark_utils.hpp"


inline openvdb::Coord GetCoord(float x, float y, float z)
{
  static float INV_RES = 1.0 / VOXEL_RESOLUTION;

  return openvdb::Coord( static_cast<int32_t>(x * INV_RES) - std::signbit(x),
                         static_cast<int32_t>(y * INV_RES) - std::signbit(y),
                         static_cast<int32_t>(z * INV_RES) - std::signbit(z) );
}

static void OpenVDB_Create(benchmark::State& state)
{
  openvdb::initialize();

  auto cloud = ReadCloud();

  for (auto _ : state)
  {
    openvdb::Int32Grid::Ptr grid = openvdb::Int32Grid::create();
    openvdb::Int32Grid::Accessor accessor = grid->getAccessor();
    grid->setGridClass(openvdb::GRID_LEVEL_SET);

    for (const auto& point : *cloud)
    {
      accessor.setValue(GetCoord(point.x, point.y, point.z), 42);
    }
  }
}

static void OpenVDB_Update(benchmark::State& state)
{
  auto cloud = ReadCloud();

  openvdb::Int32Grid::Ptr grid = openvdb::Int32Grid::create();
  grid->setGridClass(openvdb::GRID_LEVEL_SET);

  {
    openvdb::Int32Grid::Accessor accessor = grid->getAccessor();
    for (const auto& point : *cloud)
    {
      accessor.setValue(GetCoord(point.x, point.y, point.z), 42);
    }
  }

  for (auto _ : state)
  {
    openvdb::Int32Grid::Accessor accessor = grid->getAccessor();
    for (const auto& point : *cloud)
    {
      accessor.setValue(GetCoord(point.x, point.y, point.z), 42);
    }
  }
}

static void OpenVDB_IterateAllCells(benchmark::State& state)
{
  auto cloud = ReadCloud();

  openvdb::Int32Grid::Ptr grid = openvdb::Int32Grid::create();
  grid->setGridClass(openvdb::GRID_LEVEL_SET);
  openvdb::Int32Grid::Accessor accessor = grid->getAccessor();

  for (const auto& point : *cloud)
  {
    accessor.setValue(GetCoord(point.x, point.y, point.z), 42);
  }

  for (auto _ : state)
  {
    for (auto iter = grid->cbeginValueOn(); iter.test(); ++iter)
    {
      benchmark::DoNotOptimize(iter.getCoord());
      benchmark::DoNotOptimize(iter.getValue());
    }
  }
}

// Register the function as a benchmark
BENCHMARK(OpenVDB_Create);
BENCHMARK(OpenVDB_Update);
BENCHMARK(OpenVDB_IterateAllCells);
