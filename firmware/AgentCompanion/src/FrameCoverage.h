#pragma once
#include "Config.h"
#include "SpriteMotion.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace copilot {
// Source-local 16x16 tiles of a completed character frame. Effects can move or disappear
// independently of the sprite, even while both physical framebuffers hold the same pose.
struct FrameCoverage {
  static constexpr int kColumns = (kCharacterFrameWidth + 15) / 16;
  static_assert(kColumns < 32);
  using Tiles = std::array<uint32_t, (kCharacterFrameHeight + 15) / 16>;
  Tiles art{}, effects{};
  SpritePose pose;
  // Native base/patch frames can change only this rectangle when the blink level changes.
  // Scaled and full-frame renderers leave it at the conservative whole-frame default.
  uint32_t eyeColumns = UINT32_MAX;
  int eyeTop = 0, eyeEnd = (kCharacterFrameHeight + 15) / 16;

  bool sameBase(const SpritePose& other) const {
    return pose.direction == other.direction && pose.index == other.index;
  }
  bool shows(const SpritePose& other) const {
    return sameBase(other) && pose.blinkLevel == other.blinkLevel && pose.blinkBlend == other.blinkBlend;
  }
  static uint32_t columns(int x, int width) {
    return (uint32_t(1) << ((x + width + 15) / 16)) - (uint32_t(1) << (x / 16));
  }
  static void mark(Tiles& tiles, int x, int y, int width, int height) {
    if (!width || !height) return;
    const uint32_t bits = columns(x, width);
    for (int row = y / 16; row < std::min<int>(tiles.size(), (y + height + 15) / 16); ++row)
      tiles[row] |= bits;
  }
  void setEyes(int x, int y, int width, int height) {
    eyeColumns = columns(x, width);
    eyeTop = y / 16;
    eyeEnd = (y + height + 15) / 16;
  }

  // Only the render task writes the two buffers. While finishing one it can compare to the
  // preceding completed image, even if the display task is still reading that image.
  Tiles changesFrom(const FrameCoverage& previous, const uint16_t* pixels,
                    const uint16_t* previousPixels) const {
    Tiles changes{};
    const bool newBase = !sameBase(previous.pose);
    const bool newPose = !shows(previous.pose);
    for (size_t ty = 0; ty < changes.size(); ++ty) {
      if (newPose) {
        changes[ty] = art[ty] | previous.art[ty];
        if (!newBase)
          changes[ty] &= static_cast<int>(ty) >= eyeTop && static_cast<int>(ty) < eyeEnd ? eyeColumns : 0;
      }
      constexpr uint32_t columns = (uint32_t(1) << kColumns) - 1;
      uint32_t candidates = (effects[ty] | previous.effects[ty]) & ~changes[ty] & columns;
      while (candidates) {
        // Compare contiguous rows in bursts. Unchanged usage text normally clears a whole
        // run with sixteen comparisons, regardless of the number of glyphs on the line.
        const int left = __builtin_ctz(candidates);
        const int right = left + __builtin_ctz(~(candidates >> left));
        const uint32_t run = (uint32_t(1) << right) - (uint32_t(1) << left);
        candidates &= ~run;
        const int x = left * 16, y = static_cast<int>(ty) * 16;
        const size_t bytes = (std::min(right * 16, kCharacterFrameWidth) - x) * sizeof(uint16_t);
        for (int row = y; row < std::min(y + 16, kCharacterFrameHeight); ++row) {
          const int offset = row * kCharacterFrameWidth + x;
          if (std::memcmp(pixels + offset, previousPixels + offset, bytes) != 0) {
            changes[ty] |= run;
            break;
          }
        }
      }
    }
    return changes;
  }
};
}
