/*
 * stopwatch.hpp
 *
 * Created on: Jul 12, 2020 12:07
 * Description:
 *
 * Source:
 * [1] https://github.com/sailormoon/stopwatch
 * [2] https://github.com/rxdu/stopwatch
 *
 * Copyright (c) 2019 sailormoon <http://unlicense.org>
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 *
 * License: <http://unlicense.org>
 */

#ifndef STOPWATCH_HPP
#define STOPWATCH_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <algorithm>

#include <thread>

namespace westonrobot {
// only supported on x86 processors
#if (defined __x86_64__) || (defined __i386)
// An implementation of the 'TrivialClock' concept using the rdtscp instruction.
struct rdtscp_clock {
  using rep = std::uint64_t;
  using period = std::ratio<1>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<rdtscp_clock, duration>;

  static time_point now() noexcept {
    std::uint32_t hi, lo;
    __asm__ __volatile__("rdtscp" : "=d"(hi), "=a"(lo));
    return time_point(duration((static_cast<std::uint64_t>(hi) << 32) | lo));
  }
};

// A timer using the specified clock.
template <class Clock = std::chrono::system_clock>
struct timer {
  using time_point = typename Clock::time_point;
  using duration = typename Clock::duration;

  timer(const duration duration) : expiry(Clock::now() + duration) {}
  timer(const time_point expiry) : expiry(expiry) {}
  bool done() const { return done(Clock::now()); }
  bool done(const time_point now) const { return now >= expiry; }
  duration remaining() const { return remaining(Clock::now()); };
  duration remaining(const time_point now) const { return expiry - now; }
  const time_point expiry;
};

template <class Clock = std::chrono::system_clock>
constexpr timer<Clock> make_timer(const typename Clock::duration duration) {
  return timer<Clock>(duration);
}

// Times how long it takes a function to execute using the specified clock.
template <class Clock = rdtscp_clock, class Func>
typename Clock::duration time_func(Func &&function) {
  const auto start = Clock::now();
  function();
  return Clock::now() - start;
}

// Samples the given function N times using the specified clock.
template <std::size_t N, class Clock = rdtscp_clock, class Func>
std::array<typename Clock::duration, N> sample(Func &&function) {
  std::array<typename Clock::duration, N> samples;
  for (std::size_t i = 0u; i < N; ++i) {
    samples[i] = time_func<Clock>(function);
  }
  std::sort(samples.begin(), samples.end());
  return samples;
}
#endif /* __x86_64__ or __i386 */

struct StopWatch {
  using Clock = std::chrono::high_resolution_clock;
  using time_point = typename Clock::time_point;
  using duration = typename Clock::duration;

  StopWatch() { tic_point = Clock::now(); };

  time_point tic_point;

  void tic() { tic_point = Clock::now(); };

  double toc() {
    return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() -
                                                                 tic_point)
               .count() /
           1000000.0;
  };

  // toc() in different units
  double stoc() {
    return std::chrono::duration_cast<std::chrono::seconds>(Clock::now() -
                                                            tic_point)
        .count();
  };

  double mtoc() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() -
                                                                 tic_point)
        .count();
  };

  double utoc() {
    return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() -
                                                                 tic_point)
        .count();
  };

  double ntoc() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                                tic_point)
        .count();
  };
};

struct Timer {
  using Clock = std::chrono::high_resolution_clock;
  using time_point = typename Clock::time_point;
  using duration = typename Clock::duration;

  Timer() { tic_point = Clock::now(); };

  time_point tic_point;

  void reset() { tic_point = Clock::now(); };

  // you have to call reset() before calling sleep functions
  void sleep_until_ms(int64_t period_ms) {
    int64_t duration =
        period_ms - std::chrono::duration_cast<std::chrono::milliseconds>(
                        Clock::now() - tic_point)
                        .count();
    if (duration > 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(duration));
  };

  void sleep_until_us(int64_t period_us) {
    int64_t duration =
        period_us - std::chrono::duration_cast<std::chrono::microseconds>(
                        Clock::now() - tic_point)
                        .count();
    if (duration > 0)
      std::this_thread::sleep_for(std::chrono::microseconds(duration));
  };
};
}  // namespace westonrobot

#endif  // STOPWATCH_HPP