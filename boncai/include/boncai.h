/*
 * Copyright Contributors to the Bonxai Project
 * Copyright Contributors to the OpenVDB Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef BONCAI_H
#define BONCAI_H

#include <stddef.h>
#include <stdint.h>

#ifdef _MSC_VER
#define BONCAI_EXPORT __declspec(dllexport)
#else
#define BONCAI_EXPORT __attribute__((visibility("default")))
#endif

#define BONCAI_API_BEGIN extern "C" {
#define BONCAI_API_END }

#define BONCAI_OK 0
#define BONCAI_ERR_INV_ARG 1

#define BONCAI_FALSE 0
#define BONCAI_TRUE 1

#define BONCAI_DEFINE_GRID_API(t, T)                                                       \
  typedef struct boncai_grid_##t* boncai_grid_##t##_handle;                                \
  typedef struct boncai_accessor_##t* boncai_accessor_##t##_handle;                        \
  BONCAI_EXPORT boncai_error_t boncai_grid_##t##_new(                                      \
      double resolution, boncai_grid_##t##_handle* grid);                                  \
  BONCAI_EXPORT boncai_error_t boncai_grid_##t##_pos_to_coord(                             \
      boncai_grid_##t##_handle grid, double x, double y, double z, boncai_coord_t* coord); \
  BONCAI_EXPORT void boncai_grid_##t##_delete(boncai_grid_##t##_handle grid);              \
  BONCAI_EXPORT boncai_error_t boncai_accessor_##t##_new(                                  \
      boncai_grid_##t##_handle grid, boncai_accessor_##t##_handle* accessor);              \
  BONCAI_EXPORT boncai_error_t boncai_accessor_##t##_set(                                  \
      boncai_accessor_##t##_handle accessor, const boncai_coord_t* coord, T value);        \
  BONCAI_EXPORT boncai_error_t boncai_accessor_##t##_get(                                  \
      boncai_accessor_##t##_handle accessor, const boncai_coord_t* coord,                  \
      boncai_bool_t create_if_missing, T** value);                                         \
  BONCAI_EXPORT void boncai_accessor_##t##_delete(boncai_accessor_##t##_handle accessor);  \
  BONCAI_EXPORT boncai_error_t boncai_serialize_##t(                                       \
      boncai_grid_##t##_handle grid, boncai_output_stream_handle stream);                  \
  BONCAI_EXPORT boncai_error_t boncai_deserialize_##t(                                     \
      boncai_input_stream_handle stream, boncai_grid_##t##_handle* grid);

BONCAI_API_BEGIN

typedef int boncai_error_t;
typedef int boncai_bool_t;
typedef struct boncai_output_stream* boncai_output_stream_handle;
typedef struct boncai_input_stream* boncai_input_stream_handle;

typedef struct boncai_coord {
  int32_t x;
  int32_t y;
  int32_t z;
} boncai_coord_t;

BONCAI_EXPORT boncai_error_t boncai_output_stream_new(boncai_output_stream_handle* stream);
BONCAI_EXPORT boncai_error_t
boncai_output_stream_get_data(boncai_output_stream_handle stream, const void** data);
BONCAI_EXPORT boncai_error_t
boncai_output_stream_get_size(boncai_output_stream_handle stream, size_t* size);
BONCAI_EXPORT void boncai_output_stream_delete(boncai_output_stream_handle stream);

BONCAI_EXPORT boncai_error_t
boncai_input_stream_new(const void* data, size_t size, boncai_input_stream_handle* stream);
BONCAI_EXPORT void boncai_input_stream_delete(boncai_input_stream_handle stream);

BONCAI_DEFINE_GRID_API(u8, uint8_t)
BONCAI_DEFINE_GRID_API(u16, uint16_t)
BONCAI_DEFINE_GRID_API(u32, uint32_t)
BONCAI_DEFINE_GRID_API(u64, uint64_t)

BONCAI_API_END

#endif  // BONCAI_H