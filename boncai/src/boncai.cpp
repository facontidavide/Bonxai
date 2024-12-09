/*
 * Copyright Contributors to the Bonxai Project
 * Copyright Contributors to the OpenVDB Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "boncai.h"

#include <bonxai/bonxai.hpp>
#include <bonxai/serialization.hpp>
#include <memory>
#include <string>

#include "pointer_input_stream.hpp"
#include "vector_output_stream.hpp"

#define BONCAI_DEFINE_GRID_IMPL(t, T)                                                       \
  BONCAI_EXPORT boncai_error_t boncai_grid_##t##_new(                                       \
      double resolution, boncai_grid_##t##_handle* grid) {                                  \
    return Boncai::grid_new<T, boncai_grid_##t##_handle>(resolution, grid);                 \
  }                                                                                         \
  BONCAI_EXPORT boncai_error_t boncai_grid_##t##_pos_to_coord(                              \
      boncai_grid_##t##_handle grid, double x, double y, double z, boncai_coord_t* coord) { \
    return Boncai::grid_pos_to_coord<T, boncai_grid_##t##_handle>(grid, x, y, z, coord);    \
  }                                                                                         \
  BONCAI_EXPORT void boncai_grid_##t##_delete(boncai_grid_##t##_handle grid) {              \
    return Boncai::grid_delete<T, boncai_grid_##t##_handle>(grid);                          \
  }                                                                                         \
  BONCAI_EXPORT boncai_error_t boncai_accessor_##t##_new(                                   \
      boncai_grid_##t##_handle grid, boncai_accessor_##t##_handle* accessor) {              \
    return Boncai::accessor_new<T, boncai_accessor_##t##_handle, boncai_grid_##t##_handle>( \
        grid, accessor);                                                                    \
  }                                                                                         \
  BONCAI_EXPORT boncai_error_t boncai_accessor_##t##_set(                                   \
      boncai_accessor_##t##_handle accessor, const boncai_coord_t* coord, T value) {        \
    return Boncai::accessor_set<T, boncai_accessor_##t##_handle>(accessor, coord, value);   \
  }                                                                                         \
  BONCAI_EXPORT boncai_error_t boncai_accessor_##t##_get(                                   \
      boncai_accessor_##t##_handle accessor, const boncai_coord_t* coord,                   \
      boncai_bool_t create_if_missing, T** value) {                                         \
    return Boncai::accessor_get<T, boncai_accessor_##t##_handle>(                           \
        accessor, coord, create_if_missing, value);                                         \
  }                                                                                         \
  BONCAI_EXPORT void boncai_accessor_##t##_delete(boncai_accessor_##t##_handle accessor) {  \
    return Boncai::accessor_delete<T, boncai_accessor_##t##_handle>(accessor);              \
  }                                                                                         \
  BONCAI_EXPORT boncai_error_t boncai_deserialize_##t(                                      \
      boncai_input_stream_handle stream, boncai_grid_##t##_handle* grid) {                  \
    return Boncai::deserialize<T, boncai_grid_##t##_handle>(stream, grid);                  \
  }                                                                                         \
  BONCAI_EXPORT boncai_error_t boncai_serialize_##t(                                        \
      boncai_grid_##t##_handle grid, boncai_output_stream_handle stream) {                  \
    return Boncai::serialize<T, boncai_grid_##t##_handle>(grid, stream);                    \
  }

namespace Boncai {
template <typename T, typename HANDLE>
[[nodiscard]] boncai_error_t grid_new(double resolution, HANDLE* grid) {
  if (grid == nullptr) {
    return BONCAI_ERR_INV_ARG;
  }
  *grid = reinterpret_cast<HANDLE>(new Bonxai::VoxelGrid<T>(resolution));
  return BONCAI_OK;
}

template <typename T, typename HANDLE>
[[nodiscard]] boncai_error_t grid_pos_to_coord(
    HANDLE grid, double x, double y, double z, boncai_coord_t* coord) {
  if (grid == nullptr || coord == nullptr) {
    return BONCAI_ERR_INV_ARG;
  }
  const auto coord_impl = reinterpret_cast<Bonxai::VoxelGrid<T>*>(grid)->posToCoord(x, y, z);
  *coord = *reinterpret_cast<const boncai_coord_t*>(&coord_impl);
  return BONCAI_OK;
}

template <typename T, typename HANDLE>
void grid_delete(HANDLE grid) {
  delete reinterpret_cast<Bonxai::VoxelGrid<T>*>(grid);
}

template <typename T, typename ACCESSOR, typename HANDLE>
[[nodiscard]] boncai_error_t accessor_new(HANDLE grid, ACCESSOR* accessor) {
  if (grid == nullptr || accessor == nullptr) {
    return BONCAI_ERR_INV_ARG;
  }
  *accessor = reinterpret_cast<ACCESSOR>(
      new typename Bonxai::VoxelGrid<T>::Accessor(*reinterpret_cast<Bonxai::VoxelGrid<T>*>(grid)));
  return BONCAI_OK;
}

template <typename T, typename ACCESSOR>
[[nodiscard]] boncai_error_t accessor_set(ACCESSOR accessor, const boncai_coord_t* coord, T value) {
  if (accessor == nullptr || coord == nullptr) {
    return BONCAI_ERR_INV_ARG;
  }
  reinterpret_cast<typename Bonxai::VoxelGrid<T>::Accessor*>(accessor)->setValue(
      *reinterpret_cast<const Bonxai::CoordT*>(coord), value);
  return BONCAI_OK;
}

template <typename T, typename ACCESSOR>
[[nodiscard]] boncai_error_t accessor_get(
    ACCESSOR accessor, const boncai_coord_t* coord, const boncai_bool_t create_if_missing,
    T** value) {
  if (accessor == nullptr || coord == nullptr || value == nullptr) {
    return BONCAI_ERR_INV_ARG;
  }
  *value = reinterpret_cast<typename Bonxai::VoxelGrid<T>::Accessor*>(accessor)->value(
      *reinterpret_cast<const Bonxai::CoordT*>(coord), create_if_missing == BONCAI_TRUE);
  return BONCAI_OK;
}

template <typename T, typename ACCESSOR>
void accessor_delete(ACCESSOR accessor) {
  delete reinterpret_cast<typename Bonxai::VoxelGrid<T>::Accessor*>(accessor);
}

template <typename T, typename HANDLE>
[[nodiscard]] boncai_error_t deserialize(boncai_input_stream_handle stream, HANDLE* grid) {
  if (grid == nullptr || stream == nullptr) {
    return BONCAI_ERR_INV_ARG;
  }
  auto& stream_impl = *reinterpret_cast<PointerInputStream*>(stream);
  std::string first_line{};
  std::getline(stream_impl, first_line);
  stream_impl.seekg(0);
  const auto header_info = Bonxai::GetHeaderInfo(first_line);
  *reinterpret_cast<Bonxai::VoxelGrid<T>**>(grid) =
      new Bonxai::VoxelGrid<T>(header_info.resolution);
  **reinterpret_cast<Bonxai::VoxelGrid<T>**>(grid) =
      Bonxai::Deserialize<T>(stream_impl, header_info);
  return BONCAI_OK;
}

template <typename T, typename HANDLE>
[[nodiscard]] boncai_error_t serialize(HANDLE grid, boncai_output_stream_handle stream) {
  if (grid == nullptr || stream == nullptr) {
    return BONCAI_ERR_INV_ARG;
  }
  Bonxai::Serialize(
      *reinterpret_cast<VectorOutputStream*>(stream),
      *reinterpret_cast<const Bonxai::VoxelGrid<T>*>(grid));
  return BONCAI_OK;
}
}  // namespace Boncai

BONCAI_API_BEGIN

BONCAI_EXPORT boncai_error_t boncai_output_stream_new(boncai_output_stream_handle* stream) {
  if (stream == nullptr) {
    return BONCAI_ERR_INV_ARG;
  }
  *stream = reinterpret_cast<boncai_output_stream_handle>(new Boncai::VectorOutputStream());
  return BONCAI_OK;
}

BONCAI_EXPORT boncai_error_t
boncai_output_stream_get_data(boncai_output_stream_handle stream, const void** data) {
  if (stream == nullptr || data == nullptr) {
    return BONCAI_ERR_INV_ARG;
  }
  *data = reinterpret_cast<Boncai::VectorOutputStream*>(stream)->get_buffer().get_data();
  return BONCAI_OK;
}

BONCAI_EXPORT boncai_error_t
boncai_output_stream_get_size(boncai_output_stream_handle stream, size_t* size) {
  if (stream == nullptr || size == nullptr) {
    return BONCAI_ERR_INV_ARG;
  }
  *size = reinterpret_cast<Boncai::VectorOutputStream*>(stream)->get_buffer().get_size();
  return BONCAI_OK;
}

BONCAI_EXPORT void boncai_output_stream_delete(boncai_output_stream_handle stream) {
  delete reinterpret_cast<Boncai::VectorOutputStream*>(stream);
}

BONCAI_EXPORT boncai_error_t
boncai_input_stream_new(const void* data, size_t size, boncai_input_stream_handle* stream) {
  if (data == nullptr || stream == nullptr) {
    return BONCAI_ERR_INV_ARG;
  }
  *stream =
      reinterpret_cast<boncai_input_stream_handle>(new Boncai::PointerInputStream(data, size));
  return BONCAI_OK;
}

BONCAI_EXPORT void boncai_input_stream_delete(boncai_input_stream_handle stream) {
  delete reinterpret_cast<Boncai::PointerInputStream*>(stream);
}

BONCAI_DEFINE_GRID_IMPL(u8, uint8_t)
BONCAI_DEFINE_GRID_IMPL(u16, uint16_t)
BONCAI_DEFINE_GRID_IMPL(u32, uint32_t)
BONCAI_DEFINE_GRID_IMPL(u64, uint64_t)

BONCAI_API_END