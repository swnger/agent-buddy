#pragma once
#include <cmath>
#include <cstdint>

// Shared easing and random helpers for the motion engines.
namespace copilot {
// Quintic smootherstep: zero velocity and acceleration at both ends.
template <typename T>
constexpr T quintic(T t) {
  return t * t * t * (t * (t * 6 - 15) + 10);
}

// Finds t where quintic(t) == value, clamped to [0, 1].
inline double inverseQuintic(double value) {
  if (value <= 0) return 0;
  if (value >= 1) return 1;
  double low = 0, high = 1;
  for (int i = 0; i < 30; ++i) {
    const double middle = (low + high) / 2;
    if (quintic(middle) < value) low = middle;
    else high = middle;
  }
  return (low + high) / 2;
}

// Discrete-friendly soft ease (55% linear + 45% cosine): gently slows to ~2x at endpoints
// without concentrating a ~320 ms dead pause onto frame 1 before landing on center frame 0.
inline double softEase(double t) {
  constexpr double kPi = 3.14159265358979323846;
  return 0.55 * t + 0.45 * (0.5 - 0.5 * std::cos(kPi * t));
}

inline double inverseSoftEase(double value) {
  if (value <= 0) return 0;
  if (value >= 1) return 1;
  double low = 0, high = 1;
  for (int i = 0; i < 30; ++i) {
    const double middle = (low + high) / 2;
    if (softEase(middle) < value) low = middle;
    else high = middle;
  }
  return (low + high) / 2;
}

// Advances a xorshift32 state and returns the new value. The state must not be zero.
inline uint32_t xorshift32(uint32_t& state) {
  state ^= state << 13;
  state ^= state >> 17;
  state ^= state << 5;
  return state;
}
}
