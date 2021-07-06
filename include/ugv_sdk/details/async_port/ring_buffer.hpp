/*
 * ring_buffer.hpp
 *
 * Created on: Dec 08, 2019 22:22
 * Description:
 *
 * Requirements:
 *  1. Size of buffer must be a power of 2
 *  2. Maximum buffer size is 2^(number_of_bits_of(size_t)-1)
 *
 * Implementation details:
 *
 * - Initial state (empty)
 * [0][1][2][3]...[N]
 *  ^
 * R/W
 *
 * - Add one element
 * [D][1][2][3]...[N]
 *  ^  ^
 *  R  W
 *
 * - Buffer gets full when last element X is inserted
 * [X][D][D][D]...[D]
 *     ^
 *    W/R (W>R)
 *
 * - Buffer data overwritten by new element Y after getting full
 * [X][Y][D][D]...[D]
 *        ^
 *       W/R (W>R)
 *
 * Reference:
 *  [1] https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/
 *  [2] https://stackoverflow.com/questions/10527581/why-must-a-ring-buffer-size-be-a-power-of-2
 *  [3] https://stackoverflow.com/questions/9718116/improving-c-circular-buffer-efficiency
 *  [4] https://www.snellman.net/blog/archive/2016-12-13-ring-buffers/
 *  [5] tttp://www.trytoprogram.com/c-examples/c-program-to-test-if-a-number-is-a-power-of-2/
 *
 * Copyright (c) 2019 Weston Robot Pte. Ltd.
 */

#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include <cstdint>
#include <cstddef>

#include <array>
#include <iostream>

namespace westonrobot {
template <typename T = uint8_t, std::size_t N = 1024>
class RingBuffer {
 public:
  // Init and reset of buffer
  RingBuffer() {
    // assert size is a power of 2
    static_assert((N != 0) && ((N & (N - 1)) == 0), 
        "Size of ring buffer has to be 2^n, where n is a positive integer");

    size_ = N;
    size_mask_ = size_ - 1;
    write_index_ = 0;
    read_index_ = 0;
  }

  void Reset() {
    write_index_ = 0;
    read_index_ = 0;
  }

  // Buffer size information
  bool IsEmpty() { return (read_index_ == write_index_); }
  bool IsFull() { return (size_ == GetOccupiedSize()); }

  std::size_t GetFreeSize() const { return (size_ - GetOccupiedSize()); }
  std::size_t GetOccupiedSize() const { return (write_index_ - read_index_); };

  std::array<T, N> GetBuffer() const { return buffer_; }

  // Read/Write functions
  std::size_t Read(T data[], std::size_t btr) {
    for (int i = 0; i < btr; ++i) {
      if (ReadByte(&data[i]) == 0) return i;
    }
    return btr;
  }
  std::size_t Peek(T data[], std::size_t btp) {
    for (int i = 0; i < btp; ++i) {
      if (PeekByteAt(&data[i], i) == 0) return i;
    }
    return btp;
  }
  std::size_t Write(const T new_data[], std::size_t btw) {
    for (int i = 0; i < btw; ++i) {
      if (WriteByte(new_data[i]) == 0) return i;
    }
    return btw;
  }

  void PrintBuffer() const {
    std::cout << "read index: " << read_index_
              << " , write index: " << write_index_ << std::endl;
    std::cout << "buffer content: " << std::endl;
    for (int i = 0; i < N; ++i)
      std::cout << "[" << i << "]"
                << " " << static_cast<int>(buffer_[i]) << std::endl;
  }

 private:
  std::array<T, N> buffer_;  // buffer memory to store data
  std::size_t size_;         // size of allocated memory area
  std::size_t size_mask_;    // for internal indexing management
  std::size_t write_index_;  // place new data to be written
  std::size_t read_index_;   // place earliest written data to be read from

  size_t ReadByte(T *data) {
    if (IsEmpty()) return 0;
    *data = buffer_[read_index_++ & size_mask_];
    return 1;
  }

  size_t PeekByteAt(T *data, size_t n) {
    if (n >= GetOccupiedSize()) return 0;
    *data = buffer_[(read_index_ + n) & size_mask_];
    return 1;
  }

  size_t WriteByte(const T &new_data) {
    if (GetOccupiedSize() == size_) return 0;
    buffer_[(write_index_++) & size_mask_] = new_data;
    return 1;
  }
};
}  // namespace westonrobot

#endif /* RING_BUFFER_HPP */
