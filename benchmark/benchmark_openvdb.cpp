#include <benchmark/benchmark.h>
#include <openvdb/openvdb.h>
#include "benchmark_utils.hpp"

double INV_RES = 1.0 / VOXEL_RESOLUTION;

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
      openvdb::Coord xyz(point.x * INV_RES, point.y * INV_RES, point.z * INV_RES);
      accessor.setValue(xyz, 42);
    }
  }
}

static void OpenVDB_Update(benchmark::State& state)
{
  auto cloud = ReadCloud();

  openvdb::Int32Grid::Ptr grid = openvdb::Int32Grid::create();
  grid->setGridClass(openvdb::GRID_LEVEL_SET);

  for (const auto& point : *cloud)
  {
    openvdb::Int32Grid::Accessor accessor = grid->getAccessor();
    openvdb::Coord xyz(point.x * INV_RES, point.y * INV_RES, point.z * INV_RES);
    accessor.setValueOn(xyz, 42);
  }

  for (auto _ : state)
  {
    openvdb::Int32Grid::Accessor accessor = grid->getAccessor();
    for (const auto& point : *cloud)
    {
      openvdb::Coord xyz(point.x * INV_RES, point.y * INV_RES, point.z * INV_RES);
      accessor.setValueOn(xyz, 42);
    }
  }
}

static void OpenVDB_IterateAllCells(benchmark::State& state)
{
  auto cloud = ReadCloud();

  openvdb::Int32Grid::Ptr grid = openvdb::Int32Grid::create();
  grid->setGridClass(openvdb::GRID_LEVEL_SET);

  for (const auto& point : *cloud)
  {
    openvdb::Int32Grid::Accessor accessor = grid->getAccessor();
    openvdb::Coord xyz(point.x * INV_RES, point.y * INV_RES, point.z * INV_RES);
    accessor.setValueOn(xyz, 42);
  }

  for (auto _ : state)
  {
    openvdb::Int32Grid::Accessor accessor = grid->getAccessor();
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
