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
#define BONXAI_EXPORT __declspec(dllexport)
#else
#define BONXAI_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
#include <cstddef>
#include <cstdint>
#define BONXAI_CALL extern "C"
#else
#include <stddef.h>
#include <stdint.h>
#define BONXAI_CALL
#endif

#define BONXAI_API BONXAI_EXPORT BONXAI_CALL

#define BONXAI_OK 0
#define BONXAI_ERR_INV_ARG 1

#define BONXAI_FALSE 0
#define BONXAI_TRUE 1

typedef int bonxai_error_t;
typedef int bonxai_bool_t;
typedef struct bonxai_output_stream* bonxai_output_stream_handle;
typedef struct bonxai_input_stream* bonxai_input_stream_handle;

typedef struct bonxai_grid_u8* bonxai_grid_u8_handle;
typedef struct bonxai_accessor_u8* bonxai_accessor_u8_handle;
typedef struct bonxai_grid_u16* bonxai_grid_u16_handle;
typedef struct bonxai_accessor_u16* bonxai_accessor_u16_handle;
typedef struct bonxai_grid_u32* bonxai_grid_u32_handle;
typedef struct bonxai_accessor_u32* bonxai_accessor_u32_handle;
typedef struct bonxai_grid_u64* bonxai_grid_u64_handle;
typedef struct bonxai_accessor_u64* bonxai_accessor_u64_handle;

typedef struct bonxai_grid_i8* bonxai_grid_i8_handle;
typedef struct bonxai_accessor_i8* bonxai_accessor_i8_handle;
typedef struct bonxai_grid_i16* bonxai_grid_i16_handle;
typedef struct bonxai_accessor_i16* bonxai_accessor_i16_handle;
typedef struct bonxai_grid_i32* bonxai_grid_i32_handle;
typedef struct bonxai_accessor_i32* bonxai_accessor_i32_handle;
typedef struct bonxai_grid_i64* bonxai_grid_i64_handle;
typedef struct bonxai_accessor_i64* bonxai_accessor_i64_handle;

typedef struct bonxai_grid_f32* bonxai_grid_f32_handle;
typedef struct bonxai_accessor_f32* bonxai_accessor_f32_handle;
typedef struct bonxai_grid_f64* bonxai_grid_f64_handle;
typedef struct bonxai_accessor_f64* bonxai_accessor_f64_handle;

typedef struct bonxai_coord {
  int32_t x;
  int32_t y;
  int32_t z;
} bonxai_coord_t;

BONXAI_API bonxai_error_t bonxai_output_stream_new(bonxai_output_stream_handle* stream);
BONXAI_API bonxai_error_t
bonxai_output_stream_get_data(bonxai_output_stream_handle stream, const void** data);
BONXAI_API bonxai_error_t
bonxai_output_stream_get_size(bonxai_output_stream_handle stream, size_t* size);
BONXAI_API void bonxai_output_stream_delete(bonxai_output_stream_handle stream);

BONXAI_API bonxai_error_t
bonxai_input_stream_new(const void* data, size_t size, bonxai_input_stream_handle* stream);
BONXAI_API void bonxai_input_stream_delete(bonxai_input_stream_handle stream);

BONXAI_API bonxai_error_t bonxai_grid_u8_new(double resolution, bonxai_grid_u8_handle* grid);
BONXAI_API bonxai_error_t bonxai_grid_u8_pos_to_coord(
    bonxai_grid_u8_handle grid, double x, double y, double z, bonxai_coord_t* coord);
BONXAI_API void bonxai_grid_u8_delete(bonxai_grid_u8_handle grid);
BONXAI_API bonxai_error_t
bonxai_accessor_u8_new(bonxai_grid_u8_handle grid, bonxai_accessor_u8_handle* accessor);
BONXAI_API bonxai_error_t bonxai_accessor_u8_set(
    bonxai_accessor_u8_handle accessor, const bonxai_coord_t* coord, uint8_t value);
BONXAI_API bonxai_error_t bonxai_accessor_u8_get(
    bonxai_accessor_u8_handle accessor, const bonxai_coord_t* coord,
    bonxai_bool_t create_if_missing, uint8_t** value);
BONXAI_API void bonxai_accessor_u8_delete(bonxai_accessor_u8_handle accessor);
BONXAI_API bonxai_error_t
bonxai_serialize_u8(bonxai_grid_u8_handle grid, bonxai_output_stream_handle stream);
BONXAI_API bonxai_error_t
bonxai_deserialize_u8(bonxai_input_stream_handle stream, bonxai_grid_u8_handle* grid);

BONXAI_API bonxai_error_t bonxai_grid_u16_new(double resolution, bonxai_grid_u16_handle* grid);
BONXAI_API bonxai_error_t bonxai_grid_u16_pos_to_coord(
    bonxai_grid_u16_handle grid, double x, double y, double z, bonxai_coord_t* coord);
BONXAI_API void bonxai_grid_u16_delete(bonxai_grid_u16_handle grid);
BONXAI_API bonxai_error_t
bonxai_accessor_u16_new(bonxai_grid_u16_handle grid, bonxai_accessor_u16_handle* accessor);
BONXAI_API bonxai_error_t bonxai_accessor_u16_set(
    bonxai_accessor_u16_handle accessor, const bonxai_coord_t* coord, uint16_t value);
BONXAI_API bonxai_error_t bonxai_accessor_u16_get(
    bonxai_accessor_u16_handle accessor, const bonxai_coord_t* coord,
    bonxai_bool_t create_if_missing, uint16_t** value);
BONXAI_API void bonxai_accessor_u16_delete(bonxai_accessor_u16_handle accessor);
BONXAI_API bonxai_error_t
bonxai_serialize_u16(bonxai_grid_u16_handle grid, bonxai_output_stream_handle stream);
BONXAI_API bonxai_error_t
bonxai_deserialize_u16(bonxai_input_stream_handle stream, bonxai_grid_u16_handle* grid);

BONXAI_API bonxai_error_t bonxai_grid_u32_new(double resolution, bonxai_grid_u32_handle* grid);
BONXAI_API bonxai_error_t bonxai_grid_u32_pos_to_coord(
    bonxai_grid_u32_handle grid, double x, double y, double z, bonxai_coord_t* coord);
BONXAI_API void bonxai_grid_u32_delete(bonxai_grid_u32_handle grid);
BONXAI_API bonxai_error_t
bonxai_accessor_u32_new(bonxai_grid_u32_handle grid, bonxai_accessor_u32_handle* accessor);
BONXAI_API bonxai_error_t bonxai_accessor_u32_set(
    bonxai_accessor_u32_handle accessor, const bonxai_coord_t* coord, uint32_t value);
BONXAI_API bonxai_error_t bonxai_accessor_u32_get(
    bonxai_accessor_u32_handle accessor, const bonxai_coord_t* coord,
    bonxai_bool_t create_if_missing, uint32_t** value);
BONXAI_API void bonxai_accessor_u32_delete(bonxai_accessor_u32_handle accessor);
BONXAI_API bonxai_error_t
bonxai_serialize_u32(bonxai_grid_u32_handle grid, bonxai_output_stream_handle stream);
BONXAI_API bonxai_error_t
bonxai_deserialize_u32(bonxai_input_stream_handle stream, bonxai_grid_u32_handle* grid);

BONXAI_API bonxai_error_t bonxai_grid_u64_new(double resolution, bonxai_grid_u64_handle* grid);
BONXAI_API bonxai_error_t bonxai_grid_u64_pos_to_coord(
    bonxai_grid_u64_handle grid, double x, double y, double z, bonxai_coord_t* coord);
BONXAI_API void bonxai_grid_u64_delete(bonxai_grid_u64_handle grid);
BONXAI_API bonxai_error_t
bonxai_accessor_u64_new(bonxai_grid_u64_handle grid, bonxai_accessor_u64_handle* accessor);
BONXAI_API bonxai_error_t bonxai_accessor_u64_set(
    bonxai_accessor_u64_handle accessor, const bonxai_coord_t* coord, uint64_t value);
BONXAI_API bonxai_error_t bonxai_accessor_u64_get(
    bonxai_accessor_u64_handle accessor, const bonxai_coord_t* coord,
    bonxai_bool_t create_if_missing, uint64_t** value);
BONXAI_API void bonxai_accessor_u64_delete(bonxai_accessor_u64_handle accessor);
BONXAI_API bonxai_error_t
bonxai_serialize_u64(bonxai_grid_u64_handle grid, bonxai_output_stream_handle stream);
BONXAI_API bonxai_error_t
bonxai_deserialize_u64(bonxai_input_stream_handle stream, bonxai_grid_u64_handle* grid);

BONXAI_API bonxai_error_t bonxai_grid_i8_new(double resolution, bonxai_grid_i8_handle* grid);
BONXAI_API bonxai_error_t bonxai_grid_i8_pos_to_coord(
    bonxai_grid_i8_handle grid, double x, double y, double z, bonxai_coord_t* coord);
BONXAI_API void bonxai_grid_i8_delete(bonxai_grid_i8_handle grid);
BONXAI_API bonxai_error_t
bonxai_accessor_i8_new(bonxai_grid_i8_handle grid, bonxai_accessor_i8_handle* accessor);
BONXAI_API bonxai_error_t bonxai_accessor_i8_set(
    bonxai_accessor_i8_handle accessor, const bonxai_coord_t* coord, int8_t value);
BONXAI_API bonxai_error_t bonxai_accessor_i8_get(
    bonxai_accessor_i8_handle accessor, const bonxai_coord_t* coord,
    bonxai_bool_t create_if_missing, int8_t** value);
BONXAI_API void bonxai_accessor_i8_delete(bonxai_accessor_i8_handle accessor);
BONXAI_API bonxai_error_t
bonxai_serialize_i8(bonxai_grid_i8_handle grid, bonxai_output_stream_handle stream);
BONXAI_API bonxai_error_t
bonxai_deserialize_i8(bonxai_input_stream_handle stream, bonxai_grid_i8_handle* grid);

BONXAI_API bonxai_error_t bonxai_grid_i16_new(double resolution, bonxai_grid_i16_handle* grid);
BONXAI_API bonxai_error_t bonxai_grid_i16_pos_to_coord(
    bonxai_grid_i16_handle grid, double x, double y, double z, bonxai_coord_t* coord);
BONXAI_API void bonxai_grid_i16_delete(bonxai_grid_i16_handle grid);
BONXAI_API bonxai_error_t
bonxai_accessor_i16_new(bonxai_grid_i16_handle grid, bonxai_accessor_i16_handle* accessor);
BONXAI_API bonxai_error_t bonxai_accessor_i16_set(
    bonxai_accessor_i16_handle accessor, const bonxai_coord_t* coord, int16_t value);
BONXAI_API bonxai_error_t bonxai_accessor_i16_get(
    bonxai_accessor_i16_handle accessor, const bonxai_coord_t* coord,
    bonxai_bool_t create_if_missing, int16_t** value);
BONXAI_API void bonxai_accessor_i16_delete(bonxai_accessor_i16_handle accessor);
BONXAI_API bonxai_error_t
bonxai_serialize_i16(bonxai_grid_i16_handle grid, bonxai_output_stream_handle stream);
BONXAI_API bonxai_error_t
bonxai_deserialize_i16(bonxai_input_stream_handle stream, bonxai_grid_i16_handle* grid);

BONXAI_API bonxai_error_t bonxai_grid_i32_new(double resolution, bonxai_grid_i32_handle* grid);
BONXAI_API bonxai_error_t bonxai_grid_i32_pos_to_coord(
    bonxai_grid_i32_handle grid, double x, double y, double z, bonxai_coord_t* coord);
BONXAI_API void bonxai_grid_i32_delete(bonxai_grid_i32_handle grid);
BONXAI_API bonxai_error_t
bonxai_accessor_i32_new(bonxai_grid_i32_handle grid, bonxai_accessor_i32_handle* accessor);
BONXAI_API bonxai_error_t bonxai_accessor_i32_set(
    bonxai_accessor_i32_handle accessor, const bonxai_coord_t* coord, int32_t value);
BONXAI_API bonxai_error_t bonxai_accessor_i32_get(
    bonxai_accessor_i32_handle accessor, const bonxai_coord_t* coord,
    bonxai_bool_t create_if_missing, int32_t** value);
BONXAI_API void bonxai_accessor_i32_delete(bonxai_accessor_i32_handle accessor);
BONXAI_API bonxai_error_t
bonxai_serialize_i32(bonxai_grid_i32_handle grid, bonxai_output_stream_handle stream);
BONXAI_API bonxai_error_t
bonxai_deserialize_i32(bonxai_input_stream_handle stream, bonxai_grid_i32_handle* grid);

BONXAI_API bonxai_error_t bonxai_grid_i64_new(double resolution, bonxai_grid_i64_handle* grid);
BONXAI_API bonxai_error_t bonxai_grid_i64_pos_to_coord(
    bonxai_grid_i64_handle grid, double x, double y, double z, bonxai_coord_t* coord);
BONXAI_API void bonxai_grid_i64_delete(bonxai_grid_i64_handle grid);
BONXAI_API bonxai_error_t
bonxai_accessor_i64_new(bonxai_grid_i64_handle grid, bonxai_accessor_i64_handle* accessor);
BONXAI_API bonxai_error_t bonxai_accessor_i64_set(
    bonxai_accessor_i64_handle accessor, const bonxai_coord_t* coord, int64_t value);
BONXAI_API bonxai_error_t bonxai_accessor_i64_get(
    bonxai_accessor_i64_handle accessor, const bonxai_coord_t* coord,
    bonxai_bool_t create_if_missing, int64_t** value);
BONXAI_API void bonxai_accessor_i64_delete(bonxai_accessor_i64_handle accessor);
BONXAI_API bonxai_error_t
bonxai_serialize_i64(bonxai_grid_i64_handle grid, bonxai_output_stream_handle stream);
BONXAI_API bonxai_error_t
bonxai_deserialize_i64(bonxai_input_stream_handle stream, bonxai_grid_i64_handle* grid);

BONXAI_API bonxai_error_t bonxai_grid_f32_new(double resolution, bonxai_grid_f32_handle* grid);
BONXAI_API bonxai_error_t bonxai_grid_f32_pos_to_coord(
    bonxai_grid_f32_handle grid, double x, double y, double z, bonxai_coord_t* coord);
BONXAI_API void bonxai_grid_f32_delete(bonxai_grid_f32_handle grid);
BONXAI_API bonxai_error_t
bonxai_accessor_f32_new(bonxai_grid_f32_handle grid, bonxai_accessor_f32_handle* accessor);
BONXAI_API bonxai_error_t bonxai_accessor_f32_set(
    bonxai_accessor_f32_handle accessor, const bonxai_coord_t* coord, float value);
BONXAI_API bonxai_error_t bonxai_accessor_f32_get(
    bonxai_accessor_f32_handle accessor, const bonxai_coord_t* coord,
    bonxai_bool_t create_if_missing, float** value);
BONXAI_API void bonxai_accessor_f32_delete(bonxai_accessor_f32_handle accessor);
BONXAI_API bonxai_error_t
bonxai_serialize_f32(bonxai_grid_f32_handle grid, bonxai_output_stream_handle stream);
BONXAI_API bonxai_error_t
bonxai_deserialize_f32(bonxai_input_stream_handle stream, bonxai_grid_f32_handle* grid);

BONXAI_API bonxai_error_t bonxai_grid_f64_new(double resolution, bonxai_grid_f64_handle* grid);
BONXAI_API bonxai_error_t bonxai_grid_f64_pos_to_coord(
    bonxai_grid_f64_handle grid, double x, double y, double z, bonxai_coord_t* coord);
BONXAI_API void bonxai_grid_f64_delete(bonxai_grid_f64_handle grid);
BONXAI_API bonxai_error_t
bonxai_accessor_f64_new(bonxai_grid_f64_handle grid, bonxai_accessor_f64_handle* accessor);
BONXAI_API bonxai_error_t bonxai_accessor_f64_set(
    bonxai_accessor_f64_handle accessor, const bonxai_coord_t* coord, double value);
BONXAI_API bonxai_error_t bonxai_accessor_f64_get(
    bonxai_accessor_f64_handle accessor, const bonxai_coord_t* coord,
    bonxai_bool_t create_if_missing, double** value);
BONXAI_API void bonxai_accessor_f64_delete(bonxai_accessor_f64_handle accessor);
BONXAI_API bonxai_error_t
bonxai_serialize_f64(bonxai_grid_f64_handle grid, bonxai_output_stream_handle stream);
BONXAI_API bonxai_error_t
bonxai_deserialize_f64(bonxai_input_stream_handle stream, bonxai_grid_f64_handle* grid);