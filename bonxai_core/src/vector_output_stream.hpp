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

namespace Bonxai {
/**
 * A std::streambuf implementation which contains an std::vector
 * as its backing memory and allows write-access to said memory
 * through the standard APIs.
 * In conjunction with {@link VectorOutputStream}, this allows
 * buffering whatever operation that requires an std::ostream
 * into memory and to access the underlying memory in a C-friendly manner.
 *
 * More information on std::streambuf can be found here:
 * https://cplusplus.com/reference/streambuf/streambuf/
 */
class VectorStreamBuffer final : public std::streambuf {
  std::vector<uint8_t> _buffer;

 public:
  explicit VectorStreamBuffer(const size_t initial_size = 1024) noexcept {
    _buffer.reserve(initial_size);
  }

  VectorStreamBuffer(const VectorStreamBuffer&) = delete;
  VectorStreamBuffer(VectorStreamBuffer&&) noexcept = default;
  ~VectorStreamBuffer() noexcept override = default;

  VectorStreamBuffer& operator=(const VectorStreamBuffer&) = delete;
  VectorStreamBuffer& operator=(VectorStreamBuffer&&) noexcept = default;

  [[nodiscard]] const uint8_t* get_data() const noexcept {
    return _buffer.data();
  }

  [[nodiscard]] size_t get_size() const noexcept {
    return _buffer.size();
  }

 protected:
  int_type overflow(const int_type ch) override {
    if (ch != traits_type::eof()) {
      _buffer.push_back(static_cast<uint8_t>(ch));
      return ch;
    }
    return traits_type::eof();
  }

  std::streamsize xsputn(const char* data, const std::streamsize count) override {
    _buffer.insert(_buffer.end(), data, data + count);
    return count;
  }
};

/**
 * Allows buffering any operation that accepts an std::ostream
 * to memory and to access the produced data in a C-friendly manner.
 */
class VectorOutputStream final : public std::ostream {
  VectorStreamBuffer _buffer;

 public:
  explicit VectorOutputStream(const size_t initial_size = 1024) noexcept
      : std::ostream(&_buffer),
        _buffer(initial_size) {}

  VectorOutputStream(const VectorOutputStream&) = delete;
  VectorOutputStream(VectorOutputStream&&) = delete;
  ~VectorOutputStream() noexcept override = default;

  VectorOutputStream& operator=(const VectorOutputStream&) = delete;
  VectorOutputStream& operator=(VectorOutputStream&&) = delete;

  [[nodiscard]] const VectorStreamBuffer& get_buffer() const noexcept {
    return _buffer;
  }
};
}  // namespace Bonxai