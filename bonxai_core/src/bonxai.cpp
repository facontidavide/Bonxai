/*
 * Copyright Contributors to the Bonxai Project
 * Copyright Contributors to the OpenVDB Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "bonxai/bonxai.h"

#include <memory>
#include <string>

#include "bonxai/bonxai.hpp"
#include "bonxai/serialization.hpp"
#include "pointer_input_stream.hpp"
#include "vector_output_stream.hpp"

#define BONXAI_DEFINE_GRID_IMPL(t, T)                                                       \
  BONXAI_API bonxai_error_t bonxai_grid_##t##_new(                                          \
      double resolution, bonxai_grid_##t##_handle* grid) {                                  \
    return Bonxai::grid_new<T, bonxai_grid_##t##_handle>(resolution, grid);                 \
  }                                                                                         \
  BONXAI_API bonxai_error_t bonxai_grid_##t##_pos_to_coord(                                 \
      bonxai_grid_##t##_handle grid, double x, double y, double z, bonxai_coord_t* coord) { \
    return Bonxai::grid_pos_to_coord<T, bonxai_grid_##t##_handle>(grid, x, y, z, coord);    \
  }                                                                                         \
  BONXAI_API void bonxai_grid_##t##_delete(bonxai_grid_##t##_handle grid) {                 \
    return Bonxai::grid_delete<T, bonxai_grid_##t##_handle>(grid);                          \
  }                                                                                         \
  BONXAI_API bonxai_error_t bonxai_accessor_##t##_new(                                      \
      bonxai_grid_##t##_handle grid, bonxai_accessor_##t##_handle* accessor) {              \
    return Bonxai::accessor_new<T, bonxai_accessor_##t##_handle, bonxai_grid_##t##_handle>( \
        grid, accessor);                                                                    \
  }                                                                                         \
  BONXAI_API bonxai_error_t bonxai_accessor_##t##_set(                                      \
      bonxai_accessor_##t##_handle accessor, const bonxai_coord_t* coord, T value) {        \
    return Bonxai::accessor_set<T, bonxai_accessor_##t##_handle>(accessor, coord, value);   \
  }                                                                                         \
  BONXAI_API bonxai_error_t bonxai_accessor_##t##_get(                                      \
      bonxai_accessor_##t##_handle accessor, const bonxai_coord_t* coord,                   \
      bonxai_bool_t create_if_missing, T** value) {                                         \
    return Bonxai::accessor_get<T, bonxai_accessor_##t##_handle>(                           \
        accessor, coord, create_if_missing, value);                                         \
  }                                                                                         \
  BONXAI_API void bonxai_accessor_##t##_delete(bonxai_accessor_##t##_handle accessor) {     \
    return Bonxai::accessor_delete<T, bonxai_accessor_##t##_handle>(accessor);              \
  }                                                                                         \
  BONXAI_API bonxai_error_t bonxai_deserialize_##t(                                         \
      bonxai_input_stream_handle stream, bonxai_grid_##t##_handle* grid) {                  \
    return Bonxai::deserialize<T, bonxai_grid_##t##_handle>(stream, grid);                  \
  }                                                                                         \
  BONXAI_API bonxai_error_t bonxai_serialize_##t(                                           \
      bonxai_grid_##t##_handle grid, bonxai_output_stream_handle stream) {                  \
    return Bonxai::serialize<T, bonxai_grid_##t##_handle>(grid, stream);                    \
  }

namespace Bonxai {
template <typename T, typename HANDLE>
[[nodiscard]] bonxai_error_t grid_new(double resolution, HANDLE* grid) {
  if (grid == nullptr) {
    return BONXAI_ERR_INV_ARG;
  }
  *grid = reinterpret_cast<HANDLE>(new VoxelGrid<T>(resolution));
  return BONXAI_OK;
}

template <typename T, typename HANDLE>
[[nodiscard]] bonxai_error_t grid_pos_to_coord(
    HANDLE grid, double x, double y, double z, bonxai_coord_t* coord) {
  if (grid == nullptr || coord == nullptr) {
    return BONXAI_ERR_INV_ARG;
  }
  const auto coord_impl = reinterpret_cast<VoxelGrid<T>*>(grid)->posToCoord(x, y, z);
  *coord = *reinterpret_cast<const bonxai_coord_t*>(&coord_impl);
  return BONXAI_OK;
}

template <typename T, typename HANDLE>
void grid_delete(HANDLE grid) {
  delete reinterpret_cast<VoxelGrid<T>*>(grid);
}

template <typename T, typename ACCESSOR, typename HANDLE>
[[nodiscard]] bonxai_error_t accessor_new(HANDLE grid, ACCESSOR* accessor) {
  if (grid == nullptr || accessor == nullptr) {
    return BONXAI_ERR_INV_ARG;
  }
  *accessor = reinterpret_cast<ACCESSOR>(
      new typename VoxelGrid<T>::Accessor(*reinterpret_cast<VoxelGrid<T>*>(grid)));
  return BONXAI_OK;
}

template <typename T, typename ACCESSOR>
[[nodiscard]] bonxai_error_t accessor_set(ACCESSOR accessor, const bonxai_coord_t* coord, T value) {
  if (accessor == nullptr || coord == nullptr) {
    return BONXAI_ERR_INV_ARG;
  }
  reinterpret_cast<typename VoxelGrid<T>::Accessor*>(accessor)->setValue(
      *reinterpret_cast<const CoordT*>(coord), value);
  return BONXAI_OK;
}

template <typename T, typename ACCESSOR>
[[nodiscard]] bonxai_error_t accessor_get(
    ACCESSOR accessor, const bonxai_coord_t* coord, const bonxai_bool_t create_if_missing,
    T** value) {
  if (accessor == nullptr || coord == nullptr || value == nullptr) {
    return BONXAI_ERR_INV_ARG;
  }
  *value = reinterpret_cast<typename VoxelGrid<T>::Accessor*>(accessor)->value(
      *reinterpret_cast<const CoordT*>(coord), create_if_missing == BONXAI_TRUE);
  return BONXAI_OK;
}

template <typename T, typename ACCESSOR>
void accessor_delete(ACCESSOR accessor) {
  delete reinterpret_cast<typename VoxelGrid<T>::Accessor*>(accessor);
}

template <typename T, typename HANDLE>
[[nodiscard]] bonxai_error_t deserialize(bonxai_input_stream_handle stream, HANDLE* grid) {
  if (grid == nullptr || stream == nullptr) {
    return BONXAI_ERR_INV_ARG;
  }
  auto& stream_impl = *reinterpret_cast<PointerInputStream*>(stream);
  std::string first_line{};
  std::getline(stream_impl, first_line);
  stream_impl.seekg(0);
  const auto header_info = GetHeaderInfo(first_line);
  *reinterpret_cast<VoxelGrid<T>**>(grid) = new VoxelGrid<T>(header_info.resolution);
  **reinterpret_cast<VoxelGrid<T>**>(grid) = Bonxai::Deserialize<T>(stream_impl, header_info);
  return BONXAI_OK;
}

template <typename T, typename HANDLE>
[[nodiscard]] bonxai_error_t serialize(HANDLE grid, bonxai_output_stream_handle stream) {
  if (grid == nullptr || stream == nullptr) {
    return BONXAI_ERR_INV_ARG;
  }
  Bonxai::Serialize(
      *reinterpret_cast<VectorOutputStream*>(stream), *reinterpret_cast<const VoxelGrid<T>*>(grid));
  return BONXAI_OK;
}
}  // namespace Bonxai

BONXAI_API bonxai_error_t bonxai_output_stream_new(bonxai_output_stream_handle* stream) {
  if (stream == nullptr) {
    return BONXAI_ERR_INV_ARG;
  }
  *stream = reinterpret_cast<bonxai_output_stream_handle>(new Bonxai::VectorOutputStream());
  return BONXAI_OK;
}

BONXAI_API bonxai_error_t
bonxai_output_stream_get_data(bonxai_output_stream_handle stream, const void** data) {
  if (stream == nullptr || data == nullptr) {
    return BONXAI_ERR_INV_ARG;
  }
  *data = reinterpret_cast<Bonxai::VectorOutputStream*>(stream)->get_buffer().get_data();
  return BONXAI_OK;
}

BONXAI_API bonxai_error_t
bonxai_output_stream_get_size(bonxai_output_stream_handle stream, size_t* size) {
  if (stream == nullptr || size == nullptr) {
    return BONXAI_ERR_INV_ARG;
  }
  *size = reinterpret_cast<Bonxai::VectorOutputStream*>(stream)->get_buffer().get_size();
  return BONXAI_OK;
}

BONXAI_API void bonxai_output_stream_delete(bonxai_output_stream_handle stream) {
  delete reinterpret_cast<Bonxai::VectorOutputStream*>(stream);
}

BONXAI_API bonxai_error_t
bonxai_input_stream_new(const void* data, size_t size, bonxai_input_stream_handle* stream) {
  if (data == nullptr || stream == nullptr) {
    return BONXAI_ERR_INV_ARG;
  }
  *stream =
      reinterpret_cast<bonxai_input_stream_handle>(new Bonxai::PointerInputStream(data, size));
  return BONXAI_OK;
}

BONXAI_API void bonxai_input_stream_delete(bonxai_input_stream_handle stream) {
  delete reinterpret_cast<Bonxai::PointerInputStream*>(stream);
}

BONXAI_DEFINE_GRID_IMPL(u8, uint8_t)
BONXAI_DEFINE_GRID_IMPL(u16, uint16_t)
BONXAI_DEFINE_GRID_IMPL(u32, uint32_t)
BONXAI_DEFINE_GRID_IMPL(u64, uint64_t)

BONXAI_DEFINE_GRID_IMPL(i8, int8_t)
BONXAI_DEFINE_GRID_IMPL(i16, int16_t)
BONXAI_DEFINE_GRID_IMPL(i32, int32_t)
BONXAI_DEFINE_GRID_IMPL(i64, int64_t)

BONXAI_DEFINE_GRID_IMPL(f32, float)
BONXAI_DEFINE_GRID_IMPL(f64, double)