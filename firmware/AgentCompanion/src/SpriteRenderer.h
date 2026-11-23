#pragma once
#include "FrameCoverage.h"
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
  static constexpr size_t kSlots = 64, kPageWords = 4096;
  static_assert(kPageWords >= kSpriteWidth + 2);
  struct Page {
    Page* next;
    unsigned used;
    uint16_t words[kPageWords];
  };
  SpriteRenderer(uint16_t* firstOpenPatch, uint16_t* secondOpenPatch, uint16_t* patch,
                 size_t patchCapacity, uint16_t* firstOutput, uint16_t* secondOutput,
                 InflateSprite inflate, int outputWidth = kSpriteWidth,
                 int outputHeight = kSpriteHeight);
  void setBaseCache(Page* pages, size_t count, uint16_t* staging);
  bool cached() const { return pageCount_ != 0; }
  bool render(const SpritePose& pose, uint16_t* frame);
  void invalidate();
  void fillTileMask(const uint16_t* frame, FrameCoverage::Tiles& tiles) const;
  bool hasCachedPose(const SpritePose& pose, uint64_t* keep = nullptr) const;
  bool hasBaseForFrame(const SpritePose& pose, const uint16_t* frame) const;
  SpriteBlock beginPrefetch(const SpritePose& pose, uint64_t keep);
  void appendRow(const uint16_t* row, int begin, int end);
  bool finishPrefetch(bool complete);
  const char* error() const { return error_; }
  uint32_t decodeUs = 0, compositeUs = 0, eyesUs = 0;
  uint32_t cacheHits = 0, cacheMisses = 0;
  // The blink patch that the last render wrote: empty when the output already showed the pose.
  SpriteArea eyes;

 private:
  struct Slot {
    Page* first = nullptr;
    SpriteBlock block{};
    uint32_t used = 0;
  };
  struct RowSpan { int16_t begin = 0, end = 0; };
  int find(const SpritePose& pose) const;
  int oldest(uint64_t keep, bool occupied) const;
  void release(int slot);
  void drawSlot(int slot, uint16_t* art, int outputIndex, const PackHeader& header);
  bool decode(const SpriteBlock& block, uint16_t* output, size_t pixels, size_t width, size_t stride);
  void clearPadding(uint16_t* frame, int top, const PackHeader& header);
  uint16_t* openPatches_[2];
  uint16_t* patch_;
  size_t patchCapacity_;
  uint16_t* outputs_[2];
  int outputWidth_, outputHeight_;
  uint32_t outputKeys_[2] = {UINT32_MAX, UINT32_MAX};
  uint32_t baseKeys_[2] = {UINT32_MAX, UINT32_MAX};
  bool padded_[2]{}, blinking_[2]{};
  RowSpan shown_[2][kSpriteHeight]{};
  Slot slots_[kSlots]{};
  Page *pages_ = nullptr, *free_ = nullptr, *tail_ = nullptr;
  size_t pageCount_ = 0, nextPage_ = 0;
  unsigned written_ = 0;
  int writing_ = -1;
  uint64_t keep_ = 0;
  uint32_t clock_ = 0;
  uint16_t* staging_ = nullptr;
  const uint8_t* packData_ = nullptr;
  InflateSprite inflate_;
  const char* error_ = nullptr;
};
}
