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

namespace Boncai {
struct PointerStreamBuffer final : public std::streambuf {
  explicit PointerStreamBuffer(const void* data, size_t size) noexcept {
    auto* base = const_cast<char*>(static_cast<const char*>(data));
    setg(base, base, base + size);
  }

  PointerStreamBuffer(const PointerStreamBuffer&) = delete;
  PointerStreamBuffer(PointerStreamBuffer&&) noexcept = default;
  ~PointerStreamBuffer() noexcept override = default;

  PointerStreamBuffer& operator=(const PointerStreamBuffer&) = delete;
  PointerStreamBuffer& operator=(PointerStreamBuffer&&) noexcept = default;

 protected:
  [[nodiscard]] int_type underflow() override {
    if (gptr() == egptr()) {
      return traits_type::eof();
    }
    return traits_type::to_int_type(*gptr());
  }
};

class PointerInputStream final : public std::istream {
  PointerStreamBuffer _buffer;

 public:
  explicit PointerInputStream(const void* data, const size_t size) noexcept
      : std::istream(&_buffer),
        _buffer(data, size) {}

  PointerInputStream(const PointerInputStream&) = delete;
  PointerInputStream(PointerInputStream&&) noexcept = default;
  ~PointerInputStream() noexcept override = default;

  PointerInputStream& operator=(const PointerInputStream&) = delete;
  PointerInputStream& operator=(PointerInputStream&&) noexcept = default;
};
}  // namespace Boncai