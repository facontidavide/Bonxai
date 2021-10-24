#ifdef TREEXY_USE_SSE

#include <xmmintrin.h>
#include <emmintrin.h>
#include <smmintrin.h>

#endif

namespace std
{
template <>
struct hash<Treexy::CoordT>
{
  std::size_t operator()(const Treexy::CoordT& p) const
  {
    // same a OpenVDB
    return ((1 << 20) - 1) & (p.x * 73856093 ^ p.y * 19349663 ^ p.z * 83492791);
  }
};
}  // namespace std

namespace Treexy
{
template <typename DataT>
inline CoordT VoxelGrid<DataT>::posToCoord(float x, float y, float z)
{
#ifdef TREEXY_USE_SSE
  union VI
  {
    __m128i m;
    int32_t i[4];
  };
  static __m128 RES = _mm_set1_ps(inv_resolution);
  __m128 vect = _mm_set_ps(x, y, z, 0.0);
  __m128 res = _mm_mul_ps(vect, RES);
  VI out;
  out.m = _mm_cvttps_epi32(_mm_floor_ps(res));
  return { out.i[3], out.i[2], out.i[1] };
#else
  return { static_cast<int32_t>(x * inv_resolution) - std::signbit(x),
           static_cast<int32_t>(y * inv_resolution) - std::signbit(y),
           static_cast<int32_t>(z * inv_resolution) - std::signbit(z) };
#endif
}

template <typename DataT>
inline CoordT VoxelGrid<DataT>::posToCoord(double x, double y, double z)
{
  return { static_cast<int32_t>(x * inv_resolution) - std::signbit(x),
           static_cast<int32_t>(y * inv_resolution) - std::signbit(y),
           static_cast<int32_t>(z * inv_resolution) - std::signbit(z) };
}

template <typename DataT>
inline Point3D VoxelGrid<DataT>::coordToPos(const CoordT& coord)
{
  const double half_resolution = 0.5 * resolution;
  return { half_resolution + static_cast<double>(coord.x * resolution),
           half_resolution + static_cast<double>(coord.y * resolution),
           half_resolution + static_cast<double>(coord.z * resolution) };
}

//----------------------------------
template <typename DataT>
inline void VoxelGrid<DataT>::Accessor::setValue(const CoordT& coord,
                                                 const DataT& value)
{
  const CoordT inner_key = grid_.getInnerKey(coord);
  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, true);
    prev_inner_coord_ = inner_key;
  }

  uint32_t index = grid_.getLeafIndex(coord);
  prev_leaf_ptr_->mask.setOn(index);
  prev_leaf_ptr_->data[index] = value;
}

//----------------------------------
template <typename DataT>
inline DataT* VoxelGrid<DataT>::Accessor::value(const CoordT& coord)
{
  const CoordT inner_key = grid_.getInnerKey(coord);
  if (inner_key != prev_inner_coord_)
  {
    prev_leaf_ptr_ = getLeafGrid(coord, true);
    prev_inner_coord_ = inner_key;
  }

  uint32_t index = grid_.getLeafIndex(coord);
  if (prev_leaf_ptr_->mask.isOn(index))
  {
    return &(prev_leaf_ptr_->data[index]);
  }
  return nullptr;
}

//----------------------------------
template <typename DataT>
inline typename VoxelGrid<DataT>::LeafGrid*
VoxelGrid<DataT>::Accessor::getLeafGrid(const CoordT& coord, bool create_if_missing)
{
  InnerGrid* inner_ptr = prev_inner_ptr_;
  const CoordT root_key = grid_.getRootKey(coord);

  if (root_key != prev_root_coord_)
  {
    auto it = grid_.root_map.find(root_key);
    if (it == grid_.root_map.end())
    {
      if (!create_if_missing)
      {
        return nullptr;
      }
      it = grid_.root_map.insert({ root_key, InnerGrid(grid_.INNER_BITS) }).first;
    }
    inner_ptr = &(it->second);

    // update the cache
    prev_root_coord_ = root_key;
    prev_inner_ptr_ = inner_ptr;
  }

  const uint32_t inner_index = grid_.getInnerIndex(coord);
  auto& inner_data = inner_ptr->data[inner_index];

  if (create_if_missing)
  {
    if (!inner_ptr->mask.setOn(inner_index))
    {
      inner_data = std::make_shared<LeafGrid>(grid_.LEAF_BITS);
    }
  }
  else
  {
    if (!inner_ptr->mask.isOn(inner_index))
    {
      return nullptr;
    }
  }
  return inner_data.get();
}

//----------------------------------
template <typename DataT>
inline size_t VoxelGrid<DataT>::getMemoryUsage() const
{
  size_t total_size = 0;

  for (unsigned i = 0; i < root_map.bucket_count(); ++i)
  {
    size_t bucket_size = root_map.bucket_size(i);
    if (bucket_size == 0)
    {
      total_size++;
    }
    else
    {
      total_size += bucket_size;
    }
  }

  size_t entry_size = sizeof(CoordT) + sizeof(InnerGrid) + sizeof(void*);
  total_size += root_map.size() * entry_size;

  for (const auto& [key, inner_grid] : root_map)
  {
    total_size += inner_grid.mask.countOn() * sizeof(LeafGrid);
  }
  return total_size;
}

//----------------------------------
template <typename DataT>
template <class VisitorFunction>
inline void VoxelGrid<DataT>::forEachCell(VisitorFunction func)
{
  const int32_t MASK_LEAF = ((1 << LEAF_BITS) - 1);
  const int32_t MASK_INNER = ((1 << INNER_BITS) - 1);

  for (auto& map_it : root_map)
  {
    const auto& [xA, yA, zA] = (map_it.first);
    InnerGrid& inner_grid = map_it.second;
    auto& mask2 = inner_grid.mask;

    for (auto inner_it = mask2.beginOn(); inner_it; ++inner_it)
    {
      const int32_t inner_index = *inner_it;
      const int32_t INNER_BITS_2 = INNER_BITS * 2;
      // clang-format off
      int32_t xB = xA | ((inner_index & MASK_INNER) << LEAF_BITS);
      int32_t yB = yA | (((inner_index >> INNER_BITS) & MASK_INNER) << LEAF_BITS);
      int32_t zB = zA | (((inner_index >> (INNER_BITS_2)) & MASK_INNER) << LEAF_BITS);
      // clang-format on

      auto& leaf_grid = inner_grid.data[inner_index];
      auto& mask1 = leaf_grid->mask;

      for (auto leaf_it = mask1.beginOn(); leaf_it; ++leaf_it)
      {
        const int32_t leaf_index = *leaf_it;
        const int32_t LEAF_BITS_2 = LEAF_BITS * 2;
        CoordT pos = { xB | (leaf_index & MASK_LEAF),
                       yB | ((leaf_index >> LEAF_BITS) & MASK_LEAF),
                       zB | ((leaf_index >> (LEAF_BITS_2)) & MASK_LEAF) };
        // apply the visitor
        func(leaf_grid->data[leaf_index], pos);
      }
    }
  }
}

}  // namespace Treexy
