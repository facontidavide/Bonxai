/*
 * Copyright Contributors to the Bonxai Project
 * Copyright Contributors to the OpenVDB Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace Bonxai {

// Magically converts any representation of a point in 3D
// (type with x, y and z) to another one. Works with:
//
// - pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGB, etc
// - Eigen::Vector3d, Eigen::Vector3f
// - custom type with x,y,z fields. In this case Bonxai::Point3D
// - arrays or vectors with 3 elements.

template <typename PointOut, typename PointIn>
PointOut ConvertPoint(const PointIn& v);

struct Point3D {
  double x;
  double y;
  double z;

  Point3D() = default;

  Point3D(const Point3D& v) = default;
  Point3D(Point3D&& v) = default;

  Point3D& operator=(const Point3D& v) = default;
  Point3D& operator=(Point3D&& v) = default;

  Point3D(double x, double y, double z);

  template <typename T>
  Point3D(const T& v) {
    *this = ConvertPoint<Point3D>(v);
  }

  template <typename T>
  Point3D& operator=(const T& v) {
    *this = ConvertPoint<Point3D>(v);
    return *this;
  }

  // Access to x, y, z, using index 0, 1, 2
  [[nodiscard]] double& operator[](size_t index);
};

struct CoordT {
  int32_t x;
  int32_t y;
  int32_t z;

  // Access to x, y, z, using index 0, 1, 2
  [[nodiscard]] int32_t& operator[](size_t index);

  [[nodiscard]] bool operator==(const CoordT& other) const;
  [[nodiscard]] bool operator!=(const CoordT& other) const;

  [[nodiscard]] CoordT operator+(const CoordT& other) const;
  [[nodiscard]] CoordT operator-(const CoordT& other) const;

  CoordT& operator+=(const CoordT& other);
  CoordT& operator-=(const CoordT& other);
};

[[nodiscard]] inline CoordT PosToCoord(const Point3D& point, double inv_resolution) {
  return {
      static_cast<int32_t>(std::floor(point.x * inv_resolution)),
      static_cast<int32_t>(std::floor(point.y * inv_resolution)),
      static_cast<int32_t>(std::floor(point.z * inv_resolution))};
}

[[nodiscard]] inline Point3D CoordToPos(const CoordT& coord, double resolution) {
  return {
      (static_cast<double>(coord.x)) * resolution, (static_cast<double>(coord.y)) * resolution,
      (static_cast<double>(coord.z)) * resolution};
}

//----------------------------------------------------
//----------------- Implementations ------------------
//----------------------------------------------------

inline Point3D::Point3D(double _x, double _y, double _z)
    : x(_x),
      y(_y),
      z(_z) {}

inline double& Point3D::operator[](size_t index) {
  switch (index) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      throw std::runtime_error("out of bound index");
  }
}

// clang-format off
template <class T, class = void>
struct type_has_method_x : std::false_type {};
template <class T>
struct type_has_method_x<T, std::void_t<decltype(T().x())>> : std::true_type {};

template <class T, class = void>
struct type_has_member_x : std::false_type {};
template <class T>
struct type_has_member_x<T, std::void_t<decltype(T::x)>> : std::true_type {};

template<typename>
struct type_is_vector : std::false_type {};
template<typename T, typename A>
struct type_is_vector<std::vector<T, A>> : std::true_type {};
template<typename T>
struct type_is_vector<std::array<T, 3>> : std::true_type {};
// clang-format on

template <typename PointOut, typename PointIn>
inline PointOut ConvertPoint(const PointIn& v) {
  // clang-format off
  static_assert(std::is_same_v<PointIn, PointOut> ||
                    type_has_method_x<PointIn>::value ||
                    type_has_member_x<PointIn>::value ||
                    type_is_vector<PointIn>::value,
                "Can't convert from the specified type");

  static_assert(std::is_same_v<PointIn, PointOut> ||
                    type_has_method_x<PointOut>::value ||
                    type_has_member_x<PointOut>::value ||
                    type_is_vector<PointOut>::value,
                "Can't convert to the specified type");

  // clang-format on
  if constexpr (std::is_same_v<PointIn, PointOut>) {
    return v;
  }
  if constexpr (type_has_method_x<PointIn>::value) {
    return {v.x(), v.y(), v.z()};
  }
  if constexpr (type_has_member_x<PointIn>::value) {
    return {v.x, v.y, v.z};
  }
  if constexpr (type_is_vector<PointIn>::value) {
    return {v[0], v[1], v[2]};
  }
}

inline int32_t& CoordT::operator[](size_t index) {
  switch (index) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      throw std::runtime_error("out of bound index");
  }
}

inline bool CoordT::operator==(const CoordT& other) const {
  return x == other.x && y == other.y && z == other.z;
}

inline bool CoordT::operator!=(const CoordT& other) const {
  return !(*this == other);
}

inline CoordT CoordT::operator+(const CoordT& other) const {
  return {x + other.x, y + other.y, z + other.z};
}

inline CoordT CoordT::operator-(const CoordT& other) const {
  return {x - other.x, y - other.y, z - other.z};
}

inline CoordT& CoordT::operator+=(const CoordT& other) {
  x += other.x;
  y += other.y;
  z += other.z;
  return *this;
}

inline CoordT& CoordT::operator-=(const CoordT& other) {
  x -= other.x;
  y -= other.y;
  z -= other.z;
  return *this;
}

}  // namespace Bonxai

namespace std {
template <>
struct hash<Bonxai::CoordT> {
  std::size_t operator()(const Bonxai::CoordT& p) const {
    // For the containers of the users: VoxelGrid's root map, a CoordMap, has its own hash.
    //
    // x and y are packed in one 64 bits word, z is folded in with a multiplication by the
    // golden ratio, and murmur3's finalizer mixes the lot: every bit of the result is good,
    // whatever the container does with them. This replaced OpenVDB's
    // `x * 73856093 ^ y * 19349669 ^ z * 83492791`, which collided heavily on any grid
    // crossing zero once the coordinates were sign extended (the 46656 root keys of a 36^3
    // grid centred on the origin gave 29226 distinct values), was truncated to 20 bits on
    // top of that, and left its low bits at zero when the coordinates shared theirs, as
    // root keys do.
    uint64_t k = (uint64_t(uint32_t(p.x)) | (uint64_t(uint32_t(p.y)) << 32)) ^
                 (uint64_t(uint32_t(p.z)) * UINT64_C(0x9e3779b97f4a7c15));
    k ^= k >> 33;
    k *= UINT64_C(0xff51afd7ed558ccd);
    k ^= k >> 33;
    k *= UINT64_C(0xc4ceb9fe1a85ec53);
    k ^= k >> 33;
    return static_cast<std::size_t>(k);
  }
};

}  // namespace std
