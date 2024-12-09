/*
 * Copyright Contributors to the Bonxai Project
 * Copyright Contributors to the OpenVDB Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <streambuf>

namespace Bonxai {
/**
 * An std::streambuf implementation that acts as a view into an
 * existing block of memory specified by an address and a size.
 * In conjunction with {@link PointerInputStream}, this allows
 * passing any C-style slice of data to any function that accepts a std::istream.
 *
 * More information on std::streambuf can be found here:
 * https://cplusplus.com/reference/streambuf/streambuf/
 */
struct PointerStreamBuffer final : public std::streambuf {
  explicit PointerStreamBuffer(const void* data, const size_t size) noexcept {
    auto* base = const_cast<char*>(static_cast<const char*>(data));
    setg(base, base, base + size);
  }

  PointerStreamBuffer(const PointerStreamBuffer&) noexcept = default;
  PointerStreamBuffer(PointerStreamBuffer&&) noexcept = default;
  ~PointerStreamBuffer() noexcept override = default;

  PointerStreamBuffer& operator=(const PointerStreamBuffer&) noexcept = default;
  PointerStreamBuffer& operator=(PointerStreamBuffer&&) noexcept = default;

 protected:
  [[nodiscard]] int_type underflow() override {
    if (gptr() == egptr()) {
      return traits_type::eof();
    }
    return traits_type::to_int_type(*gptr());
  }
};

/**
 * Allows passing a C-style slice of memory into any API
 * that accepts an std::istream to read in data.
 */
class PointerInputStream final : public std::istream {
  PointerStreamBuffer _buffer;

 public:
  explicit PointerInputStream(const void* data, const size_t size) noexcept
      : std::istream(&_buffer),
        _buffer(data, size) {}

  PointerInputStream(const PointerInputStream&) = delete;
  PointerInputStream(PointerInputStream&&) = delete;
  ~PointerInputStream() noexcept override = default;

  PointerInputStream& operator=(const PointerInputStream&) = delete;
  PointerInputStream& operator=(PointerInputStream&&) = delete;
};
}  // namespace Bonxai