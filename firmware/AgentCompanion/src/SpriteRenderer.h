#pragma once
#include "Config.h"
#include "SpriteMotion.h"
#include "SpriteStorage.h"
#include <cstddef>
#include <cstdint>

namespace copilot {
// A blink blend shows the next blink level from the middle row outward. Rows this many rows or more
// from the middle show the next level; blend 255 shows it on about half of the rows.
inline int blinkRevealHalfHeight(uint8_t blend, int height) {
  const int half = static_cast<int>((static_cast<unsigned>(blend) * (height + 1) / 2 + 254) / 255);
  return half > 1 ? half : 1;
}

// A rectangle of output pixels.
struct SpriteArea {
  int x = 0, y = 0, width = 0, height = 0;
  bool empty() const { return width <= 0 || height <= 0; }
};

// Renders base/patch packs: one base image per pose plus blink patches around the eyes.
// secondOutput may be null when one output is enough (CharacterScaler's art buffer).
class SpriteRenderer {
 public:
  static constexpr size_t kMaxBaseCacheSlots = 64;
  struct RowSpan {
    int16_t begin = 0, end = 0;
  };
  using RowCallback = void (*)(void* ctx, size_t y, const uint16_t* swappedRow, int begin, int end);
  using InflateRows = bool (*)(const uint8_t* input, size_t inputSize, size_t width, size_t height,
                               RowCallback onRow, void* ctx);
  struct PrefetchTarget {
    int slot = -1;
    size_t poolOffset = 0;
    uint16_t* poolDst = nullptr;
    size_t poolCapacity = 0;
    size_t nextEvictOffset = 0;
    RowSpan* slotSpans = nullptr;
    SpriteBlock block{};
    uint16_t baseWidth = 0, baseHeight = 0;
  };

  SpriteRenderer(uint16_t* firstOpenPatch, uint16_t* secondOpenPatch, uint16_t* patch,
                 size_t patchCapacity, uint16_t* firstOutput, uint16_t* secondOutput,
                 InflateSprite inflate, int outputWidth = kSpriteWidth,
                 int outputHeight = kSpriteHeight);
  // Optional packed-span ring cache in PSRAM.
  // - `pool` holds `poolPixels` RGB565 pixels (packed non-black spans).
  // - `rowSpans` holds `maxSlots * kSpriteHeight` RowSpan entries.
  // - `staging` (optional internal SRAM scratch) batches PSRAM->SRAM reads on warm hits.
  // - `inflateRows` (optional) enables single-pass cold-miss dual-write directly from SRAM rows.
  void setBaseCache(uint16_t* pool, size_t poolPixels, RowSpan* rowSpans, size_t maxSlots,
                    uint16_t* staging = nullptr, size_t stagingPixels = 0,
                    InflateRows inflateRows = nullptr);
  bool render(const SpritePose& pose, uint16_t* frame);
  void invalidate();
  // Marks 16x16 framebuffer tiles containing non-black art/eyes in `frame`.
  void fillTileMask(const uint16_t* frame, uint32_t* tileMask, int tileRows) const;
  // Prefetch helpers for incremental background pose decoding into the packed PSRAM cache.
  bool poseBaseBlock(uint8_t direction, uint8_t index, SpriteBlock& out) const;
  // Optionally protect this cached slot when reserving space for a later pose in the same lookahead.
  bool hasCachedPose(uint8_t direction, uint8_t index, uint64_t* keep = nullptr) const;
  bool hasBaseForFrame(uint8_t direction, uint8_t index, const uint16_t* frame) const;
  void pinPose(uint8_t direction, uint8_t index);
  bool evictPoolRange(size_t begin, size_t end, size_t& nextOverlap, bool allowPinned = false);
  bool preparePrefetch(uint8_t direction, uint8_t index, PrefetchTarget& out, uint64_t keep = 0);
  void commitPrefetch(const PrefetchTarget& target, size_t writtenPixels);
  void cancelPrefetch(const PrefetchTarget& target);
  const char* error() const { return error_; }
  uint32_t decodeUs = 0, compositeUs = 0, eyesUs = 0;
  uint32_t cacheHits = 0, cacheMisses = 0;
  // The blink patch that the last render wrote: empty when the output already showed the pose.
  SpriteArea eyes;

 private:
  struct BaseCacheSlot {
    uint32_t offset = 0;
    uint32_t size = 0;
    size_t poolOffset = 0;
    size_t pixelCount = 0;
    uint32_t lastUsed = 0;
    bool valid = false;
  };
  bool decode(const SpriteBlock& block, uint16_t* output, size_t pixels, size_t width, size_t stride);
  bool isPinnedSlot(const BaseCacheSlot& slot) const;
  size_t nextOverlapOffset(size_t fromOffset) const;
  int findCachedSlot(const SpriteBlock& base);
  int reserveCacheSlot(size_t maxPixels, size_t& nextOverlap, bool allowPinned, bool& pinnedConflict, uint64_t keep = 0);
  void copyPackedSlotToFrame(int slotIndex, uint16_t* art, int outputIndex,
                             const PackHeader& header, const SpriteFrame& entry);
  void clearPadding(uint16_t* frame, int top, const PackHeader& header);
  void resetBaseCache();

  uint16_t* openPatches_[2];
  uint16_t* patch_;
  size_t patchCapacity_;
  uint16_t* outputs_[2];
  int outputWidth_, outputHeight_;
  uint32_t outputKeys_[2] = {UINT32_MAX, UINT32_MAX};
  uint32_t baseKeys_[2] = {UINT32_MAX, UINT32_MAX};
  bool padded_[2] = {false, false};
  RowSpan shown_[2][kSpriteHeight]{};
  uint16_t* basePool_ = nullptr;
  size_t basePoolPixels_ = 0;
  size_t basePoolHead_ = 0;
  RowSpan* cacheRowSpans_ = nullptr;
  size_t baseCacheSlots_ = 0;
  BaseCacheSlot cacheSlots_[kMaxBaseCacheSlots]{};
  SpriteBlock pinnedBlock_{};
  uint16_t* staging_ = nullptr;
  size_t stagingPixels_ = 0;
  InflateRows inflateRows_ = nullptr;
  uint32_t cacheClock_ = 0;
  const uint8_t* cachePackData_ = nullptr;
  uint16_t cacheBaseWidth_ = 0, cacheBaseHeight_ = 0;
  InflateSprite inflate_;
  const char* error_ = nullptr;
};
}
