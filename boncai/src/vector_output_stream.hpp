/*
 * Copyright Contributors to the Bonxai Project
 * Copyright Contributors to the OpenVDB Project
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <cstdint>
#include <ostream>
#include <streambuf>
#include <vector>

namespace Boncai {
class VectorStreamBuffer final : public std::streambuf {
  std::vector<uint8_t> _buffer;

 public:
  explicit VectorStreamBuffer(const size_t initial_size = 1024) noexcept {
    _buffer.reserve(initial_size);
  }

  VectorStreamBuffer(const VectorStreamBuffer&) = delete;
  VectorStreamBuffer(VectorStreamBuffer&&) noexcept = default;
  ~VectorStreamBuffer() noexcept override = default;

  auto operator=(const VectorStreamBuffer&) -> VectorStreamBuffer& = delete;
  auto operator=(VectorStreamBuffer&&) noexcept -> VectorStreamBuffer& = default;

  [[nodiscard]] auto get_data() const noexcept -> const uint8_t* {
    return _buffer.data();
  }

  [[nodiscard]] auto get_size() const noexcept -> size_t {
    return _buffer.size();
  }

 protected:
  auto overflow(const int_type ch) -> int_type override {
    if (ch != traits_type::eof()) {
      _buffer.push_back(static_cast<uint8_t>(ch));
      return ch;
    }
    return traits_type::eof();
  }

  auto xsputn(const char* data, const std::streamsize count) -> std::streamsize override {
    _buffer.insert(_buffer.end(), data, data + count);
    return count;
  }
};

class VectorOutputStream final : public std::ostream {
  VectorStreamBuffer _buffer;

 public:
  explicit VectorOutputStream(const size_t initial_size = 1024) noexcept
      : std::ostream(&_buffer),
        _buffer(initial_size) {}

  VectorOutputStream(const VectorOutputStream&) = delete;
  VectorOutputStream(VectorOutputStream&&) = delete;
  ~VectorOutputStream() noexcept override = default;

  auto operator=(const VectorOutputStream&) -> VectorOutputStream& = delete;
  auto operator=(VectorOutputStream&&) -> VectorOutputStream& = delete;

  [[nodiscard]] auto get_buffer() const noexcept -> const VectorStreamBuffer& {
    return _buffer;
  }
};
}  // namespace Boncai