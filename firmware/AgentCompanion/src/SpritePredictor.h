#pragma once
#include "Config.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#if defined(ARDUINO_ARCH_ESP32) && !defined(__clang__)
#define SPRITE_PREDICTOR_OPT __attribute__((optimize("O3")))
#else
#define SPRITE_PREDICTOR_OPT
#endif
#define SPRITE_PREDICTOR_IRAM

namespace copilot {
// The columns [begin, end) of an output row that hold every pixel that a decode changed; begin >= end
// when the row didn't change.
struct SpriteRowChange {
  uint16_t begin, end;
};

using SpriteRowCallback = void (*)(void* ctx, size_t y, const uint16_t* swappedRow, int begin, int end);

class SpritePredictor {
 public:
  // With `changes` (an entry for each output row), pixels that the output already holds aren't
  // written again, and each row's entry gives the columns that changed.
  bool reset(uint8_t* output, size_t bytes, size_t width, size_t stride = 0,
             SpriteRowChange* changes = nullptr) {
    output_ = nullptr;
    onRow_ = nullptr;
    onRowCtx_ = nullptr;
    if (!stride) stride = width;
    if (!output || reinterpret_cast<uintptr_t>(output) % alignof(uint16_t)
        || !width || width > kCharacterFrameWidth || stride < width || stride > kCharacterFrameWidth
        || !bytes || bytes % (width * 2)) return false;
    output_ = reinterpret_cast<uint16_t*>(output);
    remaining_ = bytes / 2;
    width_ = width;
    stride_ = stride;
    column_ = row_ = 0;
    changes_ = changes;
    rowFirst_ = static_cast<int>(width);
    rowLast_ = 0;
    if (changes) std::fill(changes, changes + bytes / 2 / width, SpriteRowChange{static_cast<uint16_t>(width), 0});
    std::memset(previous_, 0, width * sizeof(previous_[0]));
    return true;
  }

  // Streams completed display-order rows (with tight non-black [begin, end) spans) directly from
  // internal SRAM to `onRow`, avoiding intermediate PSRAM writes and re-scans.
  bool resetRows(size_t width, size_t height, SpriteRowCallback onRow, void* ctx) {
    output_ = nullptr;
    onRow_ = nullptr;
    onRowCtx_ = nullptr;
    if (!onRow || !width || width > kCharacterFrameWidth || !height) return false;
    onRow_ = onRow;
    onRowCtx_ = ctx;
    remaining_ = width * height;
    width_ = width;
    stride_ = width;
    column_ = row_ = 0;
    changes_ = nullptr;
    rowFirst_ = static_cast<int>(width);
    rowLast_ = 0;
    std::memset(previous_, 0, width * sizeof(previous_[0]));
    return true;
  }

  SPRITE_PREDICTOR_IRAM SPRITE_PREDICTOR_OPT
  bool consume(const uint8_t* encoded, size_t bytes) {
    if ((!output_ && !onRow_) || !encoded || (bytes & 1u) || bytes / 2 > remaining_) return false;
    size_t left = bytes / 2;
    while (left) {
      const size_t count = left < width_ - column_ ? left : width_ - column_;
      uint16_t* prev = previous_ + column_;
      uint16_t* out = swapped_ + column_;
      if (onRow_) {
        int first = rowFirst_;
        int last = rowLast_;
        const int baseCol = static_cast<int>(column_);
        const uint8_t* enc = encoded;
        for (size_t i = 0; i < count; ++i, enc += 2) {
          const uint16_t d0 = static_cast<uint16_t>((static_cast<uint16_t>(enc[0]) << 8) | enc[1]);
          const uint16_t p0 = prev[i];
          if ((d0 | p0) == 0u) {
            out[i] = 0;
            continue;
          }
          const uint16_t v0 = static_cast<uint16_t>(p0 + d0);
          prev[i] = v0;
          out[i] = __builtin_bswap16(v0);
          if (v0) {
            const int c0 = baseCol + static_cast<int>(i);
            if (c0 < first) first = c0;
            last = c0 + 1;
          }
        }
        rowFirst_ = first;
        rowLast_ = last;
      } else {
        const uint8_t* enc = encoded;
        for (size_t i = 0; i < count; ++i, enc += 2) {
          const uint16_t d0 = static_cast<uint16_t>((static_cast<uint16_t>(enc[0]) << 8) | enc[1]);
          const uint16_t p0 = prev[i];
          if ((d0 | p0) == 0u) {
            out[i] = 0;
            continue;
          }
          const uint16_t v0 = static_cast<uint16_t>(p0 + d0);
          prev[i] = v0;
          out[i] = __builtin_bswap16(v0);
        }
        // Reconstruct in internal RAM, then burst-copy instead of individual PSRAM stores.
        if (changes_) {
          size_t first = 0, last = count;
          while (first < last && output_[first] == out[first]) ++first;
          while (last > first && output_[last - 1] == out[last - 1]) --last;
          if (first < last) {
            std::memcpy(output_ + first, out + first, (last - first) * sizeof(uint16_t));
            SpriteRowChange& change = changes_[row_];
            change.begin = static_cast<uint16_t>(std::min<size_t>(change.begin, column_ + first));
            change.end = static_cast<uint16_t>(std::max<size_t>(change.end, column_ + last));
          }
        } else {
          std::memcpy(output_, out, count * sizeof(uint16_t));
        }
        output_ += count;
      }
      encoded += count * 2;
      left -= count;
      remaining_ -= count;
      column_ += count;
      if (column_ == width_) {
        if (onRow_) {
          onRow_(onRowCtx_, row_, swapped_, rowFirst_, rowLast_);
          rowFirst_ = static_cast<int>(width_);
          rowLast_ = 0;
        } else if (remaining_) {
          output_ += stride_ - width_;
        }
        column_ = 0;
        ++row_;
      }
    }
    return true;
  }

  bool complete() const { return (output_ || onRow_) && remaining_ == 0; }

 private:
  // Stored in native host byte order so prediction needs only one byte-swap per pixel (into swapped_).
  alignas(4) uint16_t previous_[kCharacterFrameWidth] = {};
  alignas(4) uint16_t swapped_[kCharacterFrameWidth] = {};
  uint16_t* output_ = nullptr;
  SpriteRowCallback onRow_ = nullptr;
  void* onRowCtx_ = nullptr;
  int rowFirst_ = 0, rowLast_ = 0;
  size_t width_ = 0, stride_ = 0, column_ = 0, row_ = 0, remaining_ = 0;
  SpriteRowChange* changes_ = nullptr;
};
}
