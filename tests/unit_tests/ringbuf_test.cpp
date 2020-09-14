/*
 * ringbuf_test.cpp
 *
 * Created on: Dec 09, 2019 11:03
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>
#include <iostream>

#include "gtest/gtest.h"

// #define ALLOW_OVERWRITE
#include "asyncio/ring_buffer.hpp"

using namespace westonrobot;

#define RINGBUF_SIZE 8

struct RingBufferTest : testing::Test {
  RingBufferTest() {}

  RingBuffer<uint8_t, RINGBUF_SIZE> buff;
  uint8_t test_data[10] = {0x01, 0x02, 0x03, 0x04, 0x05,
                           0x06, 0x07, 0x08, 0x09, 0x0a};
  uint8_t read_data[10];
};

TEST_F(RingBufferTest, WriteTest) {
  // buff.PrintBuffer();

  // write less elements than buffer size
  ASSERT_EQ(buff.Write(test_data, 4), 4);
  ASSERT_EQ(buff.GetOccupiedSize(), 4);

//   buff.PrintBuffer();

  // write more elements than buffer size
  ASSERT_EQ(buff.Write(test_data, 8), 4);
  auto data = buff.GetBuffer();
  ASSERT_TRUE(data[0] == test_data[0] && data[1] == test_data[1] &&
              data[2] == test_data[2] && data[3] == test_data[3] &&
              data[4] == test_data[0] && data[5] == test_data[1] &&
              data[6] == test_data[2] && data[7] == test_data[3]);
//   buff.PrintBuffer();
}

TEST_F(RingBufferTest, ReadTest) {
  buff.Write(test_data, 4);

  // buff.PrintBuffer();

  // read first two elements
  ASSERT_EQ(buff.Read(read_data, 2), 2);
  ASSERT_TRUE(read_data[0] == test_data[0] && read_data[1] == test_data[1]);

  // read the next two elements
  ASSERT_EQ(buff.Read(&read_data[2], 2), 2);
  ASSERT_TRUE(read_data[2] == test_data[2] && read_data[3] == test_data[3]);

  // std::cout << "read content: " << std::endl;
  // for (int i = 0; i < 4; ++i)
  //     std::cout << "[" << i << "]"
  //               << " " << static_cast<int>(read_data[i]) << std::endl;
  // buff.PrintBuffer();

  // no element to be read
  ASSERT_EQ(buff.Read(read_data, 4), 0);
}

TEST_F(RingBufferTest, PeekTest) {
  buff.Write(test_data, 4);

  // peek less elements than what's available
  ASSERT_EQ(buff.Peek(read_data, 2), 2);
  ASSERT_TRUE(read_data[0] == test_data[0] && read_data[1] == test_data[1]);

  // peek more elements than what's available
  ASSERT_EQ(buff.Peek(read_data, 5), 4);
  ASSERT_TRUE(read_data[0] == test_data[0] && read_data[1] == test_data[1] &&
              read_data[2] == test_data[2] && read_data[3] == test_data[3]);

  // read all elements out
  ASSERT_EQ(buff.Read(read_data, 4), 4);
  ASSERT_TRUE(read_data[0] == test_data[0] && read_data[1] == test_data[1] &&
              read_data[2] == test_data[2] && read_data[3] == test_data[3]);

  // no element to be peeked
  ASSERT_EQ(buff.Peek(read_data, 2), 0);
}

TEST_F(RingBufferTest, OverwriteTest) {
  // expecting overflow after writting 8 elements
  ASSERT_EQ(buff.Write(test_data, 8), 8);

  // buff.PrintBuffer();

  // expecting overflow after writting 2 more elements
  ASSERT_EQ(buff.Write(&test_data[8], 2), 0);

  // buff.PrintBuffer();

  // not expecting overflow after reading 2 elements
  ASSERT_EQ(buff.Read(read_data, 2), 2);
  ASSERT_TRUE(read_data[0] == test_data[0] && read_data[1] == test_data[1]);

  // buff.PrintBuffer();

  // expecting overflow after writting 2 elements again
  ASSERT_EQ(buff.Write(test_data, 2), 2);
}

TEST_F(RingBufferTest, StateTest) {
  ASSERT_TRUE(buff.IsEmpty());

  buff.Write(test_data, 4);
  buff.Read(read_data, 2);

  ASSERT_EQ(buff.GetFreeSize(), 6);
  ASSERT_EQ(buff.GetOccupiedSize(), 2);

  buff.Read(&read_data[2], 2);

  // buff.PrintBuffer();

  ASSERT_TRUE(buff.IsEmpty());

  buff.Write(test_data, 8);

  // buff.PrintBuffer();

  ASSERT_EQ(buff.GetOccupiedSize(), 8);
  ASSERT_TRUE(buff.IsFull());
}