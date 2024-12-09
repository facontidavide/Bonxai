/*
 * Copyright Contributors to the Bonxai Project
 * Copyright Contributors to the OpenVDB Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#ifdef _MSC_VER
#define BONCAI_EXPORT __declspec(dllexport)
#else
#define BONCAI_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
#include <cstddef>
#include <cstdint>
#define BONCAI_CALL extern "C"
#else
#include <stddef.h>
#include <stdint.h>
#define BONCAI_CALL
#endif

#define BONCAI_API BONCAI_EXPORT BONCAI_CALL

#define BONCAI_OK 0
#define BONCAI_ERR_INV_ARG 1

#define BONCAI_FALSE 0
#define BONCAI_TRUE 1

typedef int boncai_error_t;
typedef int boncai_bool_t;
typedef struct boncai_output_stream* boncai_output_stream_handle;
typedef struct boncai_input_stream* boncai_input_stream_handle;

typedef struct boncai_grid_u8* boncai_grid_u8_handle;
typedef struct boncai_accessor_u8* boncai_accessor_u8_handle;
typedef struct boncai_grid_u16* boncai_grid_u16_handle;
typedef struct boncai_accessor_u16* boncai_accessor_u16_handle;
typedef struct boncai_grid_u32* boncai_grid_u32_handle;
typedef struct boncai_accessor_u32* boncai_accessor_u32_handle;
typedef struct boncai_grid_u64* boncai_grid_u64_handle;
typedef struct boncai_accessor_u64* boncai_accessor_u64_handle;

typedef struct boncai_coord {
  int32_t x;
  int32_t y;
  int32_t z;
} boncai_coord_t;

BONCAI_API boncai_error_t boncai_output_stream_new(boncai_output_stream_handle* stream);
BONCAI_API boncai_error_t
boncai_output_stream_get_data(boncai_output_stream_handle stream, const void** data);
BONCAI_API boncai_error_t
boncai_output_stream_get_size(boncai_output_stream_handle stream, size_t* size);
BONCAI_API void boncai_output_stream_delete(boncai_output_stream_handle stream);

BONCAI_API boncai_error_t
boncai_input_stream_new(const void* data, size_t size, boncai_input_stream_handle* stream);
BONCAI_API void boncai_input_stream_delete(boncai_input_stream_handle stream);

BONCAI_API boncai_error_t boncai_grid_u8_new(double resolution, boncai_grid_u8_handle* grid);
BONCAI_API boncai_error_t boncai_grid_u8_pos_to_coord(
    boncai_grid_u8_handle grid, double x, double y, double z, boncai_coord_t* coord);
BONCAI_API void boncai_grid_u8_delete(boncai_grid_u8_handle grid);
BONCAI_API boncai_error_t
boncai_accessor_u8_new(boncai_grid_u8_handle grid, boncai_accessor_u8_handle* accessor);
BONCAI_API boncai_error_t boncai_accessor_u8_set(
    boncai_accessor_u8_handle accessor, const boncai_coord_t* coord, uint8_t value);
BONCAI_API boncai_error_t boncai_accessor_u8_get(
    boncai_accessor_u8_handle accessor, const boncai_coord_t* coord,
    boncai_bool_t create_if_missing, uint8_t** value);
BONCAI_API void boncai_accessor_u8_delete(boncai_accessor_u8_handle accessor);
BONCAI_API boncai_error_t
boncai_serialize_u8(boncai_grid_u8_handle grid, boncai_output_stream_handle stream);
BONCAI_API boncai_error_t
boncai_deserialize_u8(boncai_input_stream_handle stream, boncai_grid_u8_handle* grid);

BONCAI_API boncai_error_t boncai_grid_u16_new(double resolution, boncai_grid_u16_handle* grid);
BONCAI_API boncai_error_t boncai_grid_u16_pos_to_coord(
    boncai_grid_u16_handle grid, double x, double y, double z, boncai_coord_t* coord);
BONCAI_API void boncai_grid_u16_delete(boncai_grid_u16_handle grid);
BONCAI_API boncai_error_t
boncai_accessor_u16_new(boncai_grid_u16_handle grid, boncai_accessor_u16_handle* accessor);
BONCAI_API boncai_error_t boncai_accessor_u16_set(
    boncai_accessor_u16_handle accessor, const boncai_coord_t* coord, uint16_t value);
BONCAI_API boncai_error_t boncai_accessor_u16_get(
    boncai_accessor_u16_handle accessor, const boncai_coord_t* coord,
    boncai_bool_t create_if_missing, uint16_t** value);
BONCAI_API void boncai_accessor_u16_delete(boncai_accessor_u16_handle accessor);
BONCAI_API boncai_error_t
boncai_serialize_u16(boncai_grid_u16_handle grid, boncai_output_stream_handle stream);
BONCAI_API boncai_error_t
boncai_deserialize_u16(boncai_input_stream_handle stream, boncai_grid_u16_handle* grid);

BONCAI_API boncai_error_t boncai_grid_u32_new(double resolution, boncai_grid_u32_handle* grid);
BONCAI_API boncai_error_t boncai_grid_u32_pos_to_coord(
    boncai_grid_u32_handle grid, double x, double y, double z, boncai_coord_t* coord);
BONCAI_API void boncai_grid_u32_delete(boncai_grid_u32_handle grid);
BONCAI_API boncai_error_t
boncai_accessor_u32_new(boncai_grid_u32_handle grid, boncai_accessor_u32_handle* accessor);
BONCAI_API boncai_error_t boncai_accessor_u32_set(
    boncai_accessor_u32_handle accessor, const boncai_coord_t* coord, uint32_t value);
BONCAI_API boncai_error_t boncai_accessor_u32_get(
    boncai_accessor_u32_handle accessor, const boncai_coord_t* coord,
    boncai_bool_t create_if_missing, uint32_t** value);
BONCAI_API void boncai_accessor_u32_delete(boncai_accessor_u32_handle accessor);
BONCAI_API boncai_error_t
boncai_serialize_u32(boncai_grid_u32_handle grid, boncai_output_stream_handle stream);
BONCAI_API boncai_error_t
boncai_deserialize_u32(boncai_input_stream_handle stream, boncai_grid_u32_handle* grid);

BONCAI_API boncai_error_t boncai_grid_u64_new(double resolution, boncai_grid_u64_handle* grid);
BONCAI_API boncai_error_t boncai_grid_u64_pos_to_coord(
    boncai_grid_u64_handle grid, double x, double y, double z, boncai_coord_t* coord);
BONCAI_API void boncai_grid_u64_delete(boncai_grid_u64_handle grid);
BONCAI_API boncai_error_t
boncai_accessor_u64_new(boncai_grid_u64_handle grid, boncai_accessor_u64_handle* accessor);
BONCAI_API boncai_error_t boncai_accessor_u64_set(
    boncai_accessor_u64_handle accessor, const boncai_coord_t* coord, uint64_t value);
BONCAI_API boncai_error_t boncai_accessor_u64_get(
    boncai_accessor_u64_handle accessor, const boncai_coord_t* coord,
    boncai_bool_t create_if_missing, uint64_t** value);
BONCAI_API void boncai_accessor_u64_delete(boncai_accessor_u64_handle accessor);
BONCAI_API boncai_error_t
boncai_serialize_u64(boncai_grid_u64_handle grid, boncai_output_stream_handle stream);
BONCAI_API boncai_error_t
boncai_deserialize_u64(boncai_input_stream_handle stream, boncai_grid_u64_handle* grid);