#include "SpriteRenderer.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#ifdef ARDUINO_ARCH_ESP32
#include <esp_timer.h>
#endif

namespace copilot {
static_assert(kSpriteWidth == kCharacterFrameWidth && kSpriteHeight == kFrameHeight,
              "Character packs must match the display frame.");
static_assert(kSpriteWidth <= kDisplaySize, "Character packs must fit the display.");
namespace {
uint64_t microsNow() {
#ifdef ARDUINO_ARCH_ESP32
  return esp_timer_get_time();
#else
  return 0;
#endif
}

bool validPatchBounds(const SpriteFrame& entry, const PackHeader& header) {
  const size_t patchPixels = static_cast<size_t>(entry.patchWidth) * entry.patchHeight;
  return (entry.patchWidth == 0) == (entry.patchHeight == 0)
      && entry.patchX <= kSpriteWidth && entry.patchY <= kSpriteHeight
      && entry.patchWidth <= kSpriteWidth - entry.patchX
      && entry.patchHeight <= kSpriteHeight - entry.patchY
      && patchPixels <= header.maxPatchPixels;
}

// Copies the intersection of a base row's non-black span [begin, end) with the eye patch rectangle
// into `openPatch` (which must have been zero-initialized beforehand so margins stay exact black).
inline void captureOpenPatchRow(uint16_t* openPatch, const SpriteFrame& entry,
                                const PackHeader& header, int baseRowY,
                                const uint16_t* spanPixels, int begin, int end) {
  const int sy = header.baseY + baseRowY;
  if (sy < entry.patchY || sy >= entry.patchY + entry.patchHeight || begin >= end) return;
  const int spanLeft = header.baseX + begin;
  const int spanRight = header.baseX + end;
  const int patchLeft = entry.patchX;
  const int patchRight = entry.patchX + entry.patchWidth;
  const int left = std::max(spanLeft, patchLeft);
  const int right = std::min(spanRight, patchRight);
  if (left >= right) return;
  std::memcpy(openPatch + static_cast<size_t>(sy - entry.patchY) * entry.patchWidth + (left - patchLeft),
              spanPixels + (left - spanLeft),
              static_cast<size_t>(right - left) * sizeof(uint16_t));
}

inline void writeRowSpanToFrame(uint16_t* dst, SpriteRenderer::RowSpan& shown,
                                const uint16_t* spanPixels, int b, int e) {
  const int sb = shown.begin, se = shown.end;
  if (b >= e) {
    if (sb < se) std::memset(dst + sb, 0, static_cast<size_t>(se - sb) * sizeof(uint16_t));
    shown = {0, 0};
  } else {
    if (sb < se) {
      if (sb < b) {
        const int clearEnd = std::min(se, b);
        std::memset(dst + sb, 0, static_cast<size_t>(clearEnd - sb) * sizeof(uint16_t));
      }
      if (se > e) {
        const int clearStart = std::max(sb, e);
        std::memset(dst + clearStart, 0, static_cast<size_t>(se - clearStart) * sizeof(uint16_t));
      }
    }
    std::memcpy(dst + b, spanPixels, static_cast<size_t>(e - b) * sizeof(uint16_t));
    shown = {static_cast<int16_t>(b), static_cast<int16_t>(e)};
  }
}

struct ColdMissContext {
  SpriteRenderer* self;
  size_t poolOffset;
  size_t nextEvictOffset;
  bool allowPinnedEvict;
  uint16_t* poolDst;
  size_t poolRemaining;
  size_t writtenPixels;
  SpriteRenderer::RowSpan* slotSpans;
  uint16_t* art;
  int outputWidth;
  SpriteRenderer::RowSpan* shown;
  uint16_t* openPatch;
  const PackHeader* header;
  const SpriteFrame* entry;
  bool overflow;
};

void onColdMissRow(void* opaque, size_t y, const uint16_t* swappedRow, int begin, int end) {
  auto* ctx = static_cast<ColdMissContext*>(opaque);
  if (ctx->overflow) return;
  if (begin >= end) {
    begin = end = 0;
  }
  const size_t count = static_cast<size_t>(end - begin);
  if (count > ctx->poolRemaining) {
    ctx->overflow = true;
    return;
  }
  if (count && ctx->poolOffset + ctx->writtenPixels + count > ctx->nextEvictOffset) {
    if (!ctx->self->evictPoolRange(ctx->poolOffset, ctx->poolOffset + ctx->writtenPixels + count,
                                   ctx->nextEvictOffset, ctx->allowPinnedEvict)) {
      ctx->overflow = true;
      return;
    }
  }
  ctx->slotSpans[y] = {static_cast<int16_t>(begin), static_cast<int16_t>(end)};
  const uint16_t* spanSrc = swappedRow + begin;
  if (count) {
    std::memcpy(ctx->poolDst, spanSrc, count * sizeof(uint16_t));
    ctx->poolDst += count;
    ctx->poolRemaining -= count;
    ctx->writtenPixels += count;
  }
  uint16_t* dst = ctx->art
      + static_cast<size_t>(ctx->header->baseY + y) * ctx->outputWidth
      + ctx->header->baseX;
  writeRowSpanToFrame(dst, ctx->shown[y], spanSrc, begin, end);
  if (ctx->entry->patchHeight) {
    captureOpenPatchRow(ctx->openPatch, *ctx->entry, *ctx->header,
                        static_cast<int>(y), spanSrc, begin, end);
  }
}
}

SpriteRenderer::SpriteRenderer(uint16_t* firstOpenPatch, uint16_t* secondOpenPatch, uint16_t* patch,
                               size_t patchCapacity, uint16_t* firstOutput, uint16_t* secondOutput,
                               InflateSprite inflate, int outputWidth, int outputHeight)
    : openPatches_{firstOpenPatch, secondOpenPatch}, patch_(patch), patchCapacity_(patchCapacity),
      outputs_{firstOutput, secondOutput}, outputWidth_(outputWidth), outputHeight_(outputHeight), inflate_(inflate) {}

void SpriteRenderer::setBaseCache(uint16_t* pool, size_t poolPixels, RowSpan* rowSpans, size_t maxSlots,
                                  uint16_t* staging, size_t stagingPixels, InflateRows inflateRows) {
  basePool_ = (pool && rowSpans && maxSlots > 0) ? pool : nullptr;
  basePoolPixels_ = basePool_ ? poolPixels : 0;
  cacheRowSpans_ = basePool_ ? rowSpans : nullptr;
  baseCacheSlots_ = basePool_ ? std::min(maxSlots, kMaxBaseCacheSlots) : 0;
  staging_ = staging;
  stagingPixels_ = staging ? stagingPixels : 0;
  inflateRows_ = inflateRows;
  resetBaseCache();
}

void SpriteRenderer::resetBaseCache() {
  for (auto& slot : cacheSlots_) slot = {};
  basePoolHead_ = 0;
  cacheClock_ = 0;
  pinnedBlock_ = {};
  cachePackData_ = nullptr;
  cacheBaseWidth_ = cacheBaseHeight_ = 0;
}

void SpriteRenderer::invalidate() {
  for (unsigned i = 0; i < 2; ++i) {
    baseKeys_[i] = UINT32_MAX;
    outputKeys_[i] = UINT32_MAX;
    padded_[i] = false;
    for (auto& span : shown_[i]) span = {0, static_cast<int16_t>(kSpriteWidth)};
  }
  resetBaseCache();
}

void SpriteRenderer::fillTileMask(const uint16_t* frame, uint32_t* tileMask, int tileRows) const {
  if (!frame || !tileMask || tileRows <= 0) return;
  const int outputIndex = frame == outputs_[0] ? 0 : frame == outputs_[1] ? 1 : -1;
  if (outputIndex < 0) return;
  const CharacterPack* pack = characterPack();
  if (!pack || pack->header.layout != PackLayout::BasePatch) return;
  const PackHeader& header = pack->header;
  const int top = (outputHeight_ - kSpriteHeight) / 2;
  const bool useSpans = basePool_ && cacheRowSpans_ && padded_[outputIndex];
  if (useSpans) {
    for (int y = 0; y < header.baseHeight; ++y) {
      const RowSpan& span = shown_[outputIndex][y];
      if (span.begin >= span.end) continue;
      const int ty = (top + header.baseY + y) >> 4;
      if (ty < 0 || ty >= tileRows) continue;
      const int tx0 = std::clamp((header.baseX + span.begin) >> 4, 0, 31);
      const int tx1 = std::clamp((header.baseX + span.end - 1) >> 4, 0, 31);
      tileMask[ty] |= ((uint32_t(2) << (tx1 - tx0)) - 1u) << tx0;
    }
  } else {
    const int tx0 = std::clamp(static_cast<int>(header.baseX) >> 4, 0, 31);
    const int tx1 = std::clamp((static_cast<int>(header.baseX) + header.baseWidth - 1) >> 4, 0, 31);
    const int ty0 = std::max(0, (top + static_cast<int>(header.baseY)) >> 4);
    const int ty1 = std::min(tileRows - 1, (top + static_cast<int>(header.baseY) + header.baseHeight - 1) >> 4);
    const uint32_t bits = ((uint32_t(2) << (tx1 - tx0)) - 1u) << tx0;
    for (int ty = ty0; ty <= ty1; ++ty) tileMask[ty] |= bits;
  }
  if (baseKeys_[outputIndex] != UINT32_MAX && baseKeys_[outputIndex] < pack->header.frameCount) {
    const SpriteFrame& entry = pack->frames[baseKeys_[outputIndex]];
    if (entry.patchWidth > 0 && entry.patchHeight > 0) {
      const int tx0 = std::clamp(static_cast<int>(entry.patchX) >> 4, 0, 31);
      const int tx1 = std::clamp((static_cast<int>(entry.patchX) + entry.patchWidth - 1) >> 4, 0, 31);
      const int ty0 = std::max(0, (top + static_cast<int>(entry.patchY)) >> 4);
      const int ty1 = std::min(tileRows - 1, (top + static_cast<int>(entry.patchY) + entry.patchHeight - 1) >> 4);
      const uint32_t bits = ((uint32_t(2) << (tx1 - tx0)) - 1u) << tx0;
      for (int ty = ty0; ty <= ty1; ++ty) tileMask[ty] |= bits;
    }
  }
}

bool SpriteRenderer::poseBaseBlock(uint8_t direction, uint8_t index, SpriteBlock& out) const {
  const CharacterPack* pack = characterPack();
  if (!pack || pack->header.layout != PackLayout::BasePatch
      || direction >= kSpriteDirections || index >= kSpriteTrackSteps[direction])
    return false;
  out = pack->frames[kSpriteTrackOffsets[direction] + index].base;
  return out.size > 0;
}

bool SpriteRenderer::hasCachedPose(uint8_t direction, uint8_t index, uint64_t* keep) const {
  static_assert(kMaxBaseCacheSlots <= 64);
  const CharacterPack* pack = characterPack();
  if (!pack || !basePool_ || !cacheRowSpans_ || cachePackData_ != pack->data) return false;
  SpriteBlock base;
  if (!poseBaseBlock(direction, index, base)) return false;
  for (size_t i = 0; i < baseCacheSlots_; ++i) {
    const auto& slot = cacheSlots_[i];
    if (slot.valid && slot.offset == base.offset && slot.size == base.size) {
      if (keep) *keep |= uint64_t(1) << i;
      return true;
    }
  }
  return false;
}

bool SpriteRenderer::hasBaseForFrame(uint8_t direction, uint8_t index, const uint16_t* frame) const {
  const int outputIndex = frame == outputs_[0] ? 0 : frame == outputs_[1] ? 1 : -1;
  if (outputIndex < 0 || baseKeys_[outputIndex] == UINT32_MAX) return false;
  const CharacterPack* pack = characterPack();
  if (!pack || pack->header.layout != PackLayout::BasePatch
      || direction >= kSpriteDirections || index >= kSpriteTrackSteps[direction]
      || baseKeys_[outputIndex] >= pack->header.frameCount)
    return false;
  const SpriteFrame& cached = pack->frames[baseKeys_[outputIndex]];
  const SpriteFrame& entry = pack->frames[kSpriteTrackOffsets[direction] + index];
  return cached.base.offset == entry.base.offset && cached.base.size == entry.base.size
      && cached.patchX == entry.patchX && cached.patchY == entry.patchY
      && cached.patchWidth == entry.patchWidth && cached.patchHeight == entry.patchHeight;
}

void SpriteRenderer::pinPose(uint8_t direction, uint8_t index) {
  SpriteBlock block{};
  if (poseBaseBlock(direction, index, block)) pinnedBlock_ = block;
  else pinnedBlock_ = {};
}

bool SpriteRenderer::isPinnedSlot(const BaseCacheSlot& slot) const {
  if (!slot.valid) return false;
  if (pinnedBlock_.size > 0 && slot.offset == pinnedBlock_.offset && slot.size == pinnedBlock_.size)
    return true;
  const CharacterPack* pack = characterPack();
  if (!pack || pack->header.layout != PackLayout::BasePatch) return false;
  for (unsigned i = 0; i < 2; ++i) {
    if (baseKeys_[i] == UINT32_MAX || baseKeys_[i] >= pack->header.frameCount) continue;
    const SpriteBlock& b = pack->frames[baseKeys_[i]].base;
    if (b.offset == slot.offset && b.size == slot.size) return true;
  }
  return false;
}

size_t SpriteRenderer::nextOverlapOffset(size_t fromOffset) const {
  size_t nearest = basePoolPixels_;
  for (size_t i = 0; i < baseCacheSlots_; ++i) {
    const auto& s = cacheSlots_[i];
    if (!s.valid || s.pixelCount == 0) continue;
    const size_t sEnd = s.poolOffset + s.pixelCount;
    if (sEnd <= fromOffset) continue;
    if (s.poolOffset <= fromOffset) return fromOffset;
    if (s.poolOffset < nearest) nearest = s.poolOffset;
  }
  return nearest;
}

bool SpriteRenderer::evictPoolRange(size_t begin, size_t end, size_t& nextOverlap, bool allowPinned) {
  for (size_t i = 0; i < baseCacheSlots_; ++i) {
    auto& s = cacheSlots_[i];
    if (!s.valid || s.pixelCount == 0) continue;
    const size_t sEnd = s.poolOffset + s.pixelCount;
    if (s.poolOffset < end && sEnd > begin) {
      if (!allowPinned && isPinnedSlot(s)) return false;
      s.valid = false;
    }
  }
  nextOverlap = nextOverlapOffset(end);
  return true;
}

bool SpriteRenderer::preparePrefetch(uint8_t direction, uint8_t index, PrefetchTarget& out, uint64_t keep) {
  out = {};
  const CharacterPack* pack = characterPack();
  if (!pack || pack->header.layout != PackLayout::BasePatch
      || !basePool_ || !cacheRowSpans_ || baseCacheSlots_ < 2)
    return false;
  const PackHeader& header = pack->header;
  const size_t basePixels = static_cast<size_t>(header.baseWidth) * header.baseHeight;
  if (basePixels == 0 || basePoolPixels_ < basePixels) return false;
  if (cachePackData_ != pack->data || cacheBaseWidth_ != header.baseWidth
      || cacheBaseHeight_ != header.baseHeight) {
    invalidate();
    cachePackData_ = pack->data;
    cacheBaseWidth_ = header.baseWidth;
    cacheBaseHeight_ = header.baseHeight;
  }
  SpriteBlock base;
  if (!poseBaseBlock(direction, index, base)) return false;
  for (size_t i = 0; i < baseCacheSlots_; ++i) {
    const auto& slot = cacheSlots_[i];
    if (slot.valid && slot.offset == base.offset && slot.size == base.size) return false;
  }
  size_t nextOverlap = basePoolPixels_;
  bool pinnedConflict = false;
  const int chosen = reserveCacheSlot(basePixels, nextOverlap, false, pinnedConflict, keep);
  if (chosen < 0) return false;
  out.slot = chosen;
  out.poolOffset = basePoolHead_;
  out.poolDst = basePool_ + basePoolHead_;
  out.poolCapacity = basePixels;
  out.nextEvictOffset = nextOverlap;
  out.slotSpans = cacheRowSpans_ + static_cast<size_t>(chosen) * kSpriteHeight;
  out.block = base;
  out.baseWidth = header.baseWidth;
  out.baseHeight = header.baseHeight;
  return true;
}

void SpriteRenderer::commitPrefetch(const PrefetchTarget& target, size_t writtenPixels) {
  if (target.slot < 0 || static_cast<size_t>(target.slot) >= baseCacheSlots_
      || writtenPixels > target.poolCapacity)
    return;
  auto& slot = cacheSlots_[target.slot];
  slot.offset = target.block.offset;
  slot.size = target.block.size;
  slot.poolOffset = target.poolOffset;
  slot.pixelCount = writtenPixels;
  slot.lastUsed = ++cacheClock_;
  slot.valid = true;
  basePoolHead_ = target.poolOffset + writtenPixels;
}

void SpriteRenderer::cancelPrefetch(const PrefetchTarget& target) {
  if (target.slot >= 0 && static_cast<size_t>(target.slot) < baseCacheSlots_) {
    cacheSlots_[target.slot].valid = false;
  }
}

bool SpriteRenderer::decode(const SpriteBlock& block, uint16_t* output, size_t pixels, size_t width, size_t stride) {
  constexpr SpriteBlockErrors kErrors{"Sprite block lies outside the installed character pack.",
                                      "Sprite asset decompression failed."};
  error_ = inflateSpriteBlock(inflate_, block, output, pixels, width, stride, kErrors);
  return !error_;
}

int SpriteRenderer::findCachedSlot(const SpriteBlock& base) {
  for (size_t i = 0; i < baseCacheSlots_; ++i) {
    auto& slot = cacheSlots_[i];
    if (slot.valid && slot.offset == base.offset && slot.size == base.size) {
      slot.lastUsed = ++cacheClock_;
      ++cacheHits;
      return static_cast<int>(i);
    }
  }
  return -1;
}

int SpriteRenderer::reserveCacheSlot(size_t maxPixels, size_t& nextOverlap,
                                     bool allowPinned, bool& pinnedConflict, uint64_t keep) {
  pinnedConflict = false;
  if (maxPixels > basePoolPixels_) return -1;
  size_t candidate = basePoolHead_;
  if (candidate + maxPixels > basePoolPixels_) candidate = 0;
  bool foundUnpinned = false;
  for (size_t attempt = 0; attempt <= baseCacheSlots_ + 2; ++attempt) {
    const size_t rangeEnd = candidate + maxPixels;
    size_t blockerEnd = 0;
    for (size_t i = 0; i < baseCacheSlots_; ++i) {
      const auto& s = cacheSlots_[i];
      if (!s.valid || s.pixelCount == 0 || (!isPinnedSlot(s) && !(keep & (uint64_t(1) << i)))) continue;
      const size_t sEnd = s.poolOffset + s.pixelCount;
      if (s.poolOffset < rangeEnd && sEnd > candidate && sEnd > blockerEnd) blockerEnd = sEnd;
    }
    if (blockerEnd == 0) {
      foundUnpinned = true;
      break;
    }
    candidate = blockerEnd;
    if (candidate + maxPixels > basePoolPixels_) candidate = 0;
  }
  if (!foundUnpinned) {
    if (!allowPinned) return -1;
    pinnedConflict = true;
    candidate = basePoolHead_;
    if (candidate + maxPixels > basePoolPixels_) candidate = 0;
  }
  basePoolHead_ = candidate;
  nextOverlap = nextOverlapOffset(candidate);

  int chosen = -1;
  uint32_t oldest = UINT32_MAX;
  for (size_t i = 0; i < baseCacheSlots_; ++i) {
    if (!cacheSlots_[i].valid) {
      chosen = static_cast<int>(i);
      break;
    }
    if (isPinnedSlot(cacheSlots_[i]) || (keep & (uint64_t(1) << i))) continue;
    if (chosen < 0 || cacheSlots_[i].lastUsed < oldest) {
      oldest = cacheSlots_[i].lastUsed;
      chosen = static_cast<int>(i);
    }
  }
  if (chosen < 0 && allowPinned) {
    pinnedConflict = true;
    for (size_t i = 0; i < baseCacheSlots_; ++i) {
      if (chosen < 0 || cacheSlots_[i].lastUsed < oldest) {
        oldest = cacheSlots_[i].lastUsed;
        chosen = static_cast<int>(i);
      }
    }
  }
  if (chosen >= 0) cacheSlots_[chosen].valid = false;
  return chosen;
}

void SpriteRenderer::copyPackedSlotToFrame(int slotIndex, uint16_t* art, int outputIndex,
                                           const PackHeader& header, const SpriteFrame& entry) {
  const auto& slot = cacheSlots_[slotIndex];
  const RowSpan* spans = cacheRowSpans_ + static_cast<size_t>(slotIndex) * kSpriteHeight;
  const uint16_t* poolPtr = basePool_ + slot.poolOffset;
  const size_t patchPixels = static_cast<size_t>(entry.patchWidth) * entry.patchHeight;
  if (patchPixels) {
    std::memset(openPatches_[outputIndex], 0, patchPixels * sizeof(uint16_t));
  }

  // Stage contiguous packed PSRAM bursts through internal SRAM when a staging buffer is available
  // so OPI PSRAM streams in one direction per burst instead of interleaving reads and writes.
  if (staging_ && stagingPixels_ >= static_cast<size_t>(header.baseWidth)) {
    size_t poolOffset = 0;
    int y = 0;
    while (y < header.baseHeight) {
      int batchEnd = y;
      size_t batchPixels = 0;
      while (batchEnd < header.baseHeight) {
        const size_t rowLen = static_cast<size_t>(std::max(0, spans[batchEnd].end - spans[batchEnd].begin));
        if (batchPixels > 0 && batchPixels + rowLen > stagingPixels_) break;
        batchPixels += rowLen;
        ++batchEnd;
      }
      if (batchPixels) {
        std::memcpy(staging_, poolPtr + poolOffset, batchPixels * sizeof(uint16_t));
        poolOffset += batchPixels;
      }
      const uint16_t* src = staging_;
      for (; y < batchEnd; ++y) {
        const int b = spans[y].begin, e = spans[y].end;
        uint16_t* dst = art + static_cast<size_t>(header.baseY + y) * outputWidth_ + header.baseX;
        writeRowSpanToFrame(dst, shown_[outputIndex][y], src, b, e);
        if (patchPixels) {
          captureOpenPatchRow(openPatches_[outputIndex], entry, header, y, src, b, e);
        }
        if (e > b) src += static_cast<size_t>(e - b);
      }
    }
    return;
  }

  const uint16_t* src = poolPtr;
  for (int y = 0; y < header.baseHeight; ++y) {
    const int b = spans[y].begin, e = spans[y].end;
    uint16_t* dst = art + static_cast<size_t>(header.baseY + y) * outputWidth_ + header.baseX;
    writeRowSpanToFrame(dst, shown_[outputIndex][y], src, b, e);
    if (patchPixels) {
      captureOpenPatchRow(openPatches_[outputIndex], entry, header, y, src, b, e);
    }
    if (e > b) src += static_cast<size_t>(e - b);
  }
}

void SpriteRenderer::clearPadding(uint16_t* frame, int top, const PackHeader& header) {
  const int baseTop = top + header.baseY;
  std::memset(frame, 0, static_cast<size_t>(baseTop) * outputWidth_ * sizeof(uint16_t));
  std::memset(frame + static_cast<size_t>(baseTop + header.baseHeight) * outputWidth_, 0,
              static_cast<size_t>(outputHeight_ - baseTop - header.baseHeight) * outputWidth_ * sizeof(uint16_t));
  for (int y = baseTop; y < baseTop + header.baseHeight; ++y) {
    std::memset(frame + static_cast<size_t>(y) * outputWidth_, 0, header.baseX * sizeof(uint16_t));
    std::memset(frame + static_cast<size_t>(y) * outputWidth_ + header.baseX + header.baseWidth, 0,
                static_cast<size_t>(outputWidth_ - header.baseX - header.baseWidth) * sizeof(uint16_t));
  }
}

bool SpriteRenderer::render(const SpritePose& pose, uint16_t* frame) {
  error_ = nullptr;
  decodeUs = compositeUs = eyesUs = 0;
  eyes = {};
  const CharacterPack* pack = characterPack();
  if (!pack || pack->header.layout != PackLayout::BasePatch) {
    error_ = "Installed character pack does not use the base/patch layout.";
    return false;
  }
  const PackHeader& header = pack->header;
  if (!openPatches_[0] || !openPatches_[1] || !patch_ || !outputs_[0]
      || outputs_[0] == outputs_[1] || openPatches_[0] == openPatches_[1]
      || patch_ == openPatches_[0] || patch_ == openPatches_[1] || !inflate_ || !frame) {
    error_ = "Sprite renderer requires distinct persistent buffers.";
    return false;
  }
  if (header.maxPatchPixels > patchCapacity_) {
    error_ = "Character pack blink patches exceed the allocated patch buffers.";
    return false;
  }
  if (pose.direction >= kSpriteDirections || pose.index >= kSpriteTrackSteps[pose.direction]
      || pose.blinkLevel >= kSpriteBlinkLevels
      || (pose.blinkBlend && pose.blinkLevel + 1 >= kSpriteBlinkLevels)) {
    error_ = "Invalid discrete sprite pose.";
    return false;
  }
  if (outputWidth_ != kSpriteWidth || outputHeight_ < kSpriteHeight || outputHeight_ > kDisplaySize) {
    error_ = "Sprite output must match the exported stride and fit the artwork and display.";
    return false;
  }
  const int outputIndex = frame == outputs_[0] ? 0 : frame == outputs_[1] ? 1 : -1;
  if (outputIndex < 0) {
    error_ = "Sprite output buffer is not registered.";
    return false;
  }

  const size_t basePixels = static_cast<size_t>(header.baseWidth) * header.baseHeight;
  const bool useCache = basePool_ && cacheRowSpans_ && baseCacheSlots_ >= 1 && basePoolPixels_ >= basePixels;
  if (useCache && (cachePackData_ != pack->data || cacheBaseWidth_ != header.baseWidth
                   || cacheBaseHeight_ != header.baseHeight)) {
    invalidate();
    cachePackData_ = pack->data;
    cacheBaseWidth_ = header.baseWidth;
    cacheBaseHeight_ = header.baseHeight;
  }

  const unsigned index = kSpriteTrackOffsets[pose.direction] + pose.index;
  const uint32_t key = (index * kSpriteBlinkLevels + pose.blinkLevel) * 256 + pose.blinkBlend;
  if (outputKeys_[outputIndex] == key) return true;

  const SpriteFrame& entry = pack->frames[index];
  const int top = (outputHeight_ - kSpriteHeight) / 2;
  uint16_t* art = frame + static_cast<size_t>(top) * outputWidth_;
  const size_t patchPixels = static_cast<size_t>(entry.patchWidth) * entry.patchHeight;
  const SpriteArea patchArea{entry.patchX, top + entry.patchY, entry.patchWidth, entry.patchHeight};
  if (!validPatchBounds(entry, header)) {
    error_ = "Sprite blink patch lies outside its frame or buffer.";
    return false;
  }

  // Each physical framebuffer owns its cache; the other core may be transmitting the other one.
  outputKeys_[outputIndex] = UINT32_MAX;
  bool sameBase = baseKeys_[outputIndex] != UINT32_MAX;
  if (sameBase) {
    const SpriteFrame& cached = pack->frames[baseKeys_[outputIndex]];
    sameBase = cached.base.offset == entry.base.offset && cached.base.size == entry.base.size
        && cached.patchX == entry.patchX && cached.patchY == entry.patchY
        && cached.patchWidth == entry.patchWidth && cached.patchHeight == entry.patchHeight;
  }
  if (!sameBase) {
    baseKeys_[outputIndex] = UINT32_MAX;
    if (useCache) {
      if (!padded_[outputIndex]) {
        clearPadding(frame, top, header);
        for (int y = 0; y < header.baseHeight; ++y)
          shown_[outputIndex][y] = {0, static_cast<int16_t>(header.baseWidth)};
        padded_[outputIndex] = true;
      }
      const int hit = findCachedSlot(entry.base);
      if (hit >= 0) {
        const uint64_t started = microsNow();
        copyPackedSlotToFrame(hit, art, outputIndex, header, entry);
        compositeUs = static_cast<uint32_t>(microsNow() - started);
      } else {
        size_t nextOverlap = basePoolPixels_;
        bool pinnedConflict = false;
        const int chosen = reserveCacheSlot(basePixels, nextOverlap, true, pinnedConflict);
        if (chosen < 0) {
          error_ = "Sprite base cache allocation failed.";
          return false;
        }
        const uint8_t* source = spriteBlock(entry.base);
        if (!source) {
          error_ = "Sprite block lies outside the installed character pack.";
          return false;
        }
        if (inflateRows_) {
          if (patchPixels) std::memset(openPatches_[outputIndex], 0, patchPixels * sizeof(uint16_t));
          ColdMissContext ctx{
              this,
              basePoolHead_,
              nextOverlap,
              pinnedConflict,
              basePool_ + basePoolHead_,
              basePixels,
              0,
              cacheRowSpans_ + static_cast<size_t>(chosen) * kSpriteHeight,
              art,
              outputWidth_,
              shown_[outputIndex],
              openPatches_[outputIndex],
              &header,
              &entry,
              false};
          const uint64_t started = microsNow();
          const bool ok = inflateRows_(source, entry.base.size, header.baseWidth, header.baseHeight,
                                       onColdMissRow, &ctx);
          decodeUs = static_cast<uint32_t>(microsNow() - started);
          if (!ok || ctx.overflow) {
            padded_[outputIndex] = false;
            error_ = "Sprite asset decompression failed.";
            return false;
          }
          auto& slot = cacheSlots_[chosen];
          slot.offset = entry.base.offset;
          slot.size = entry.base.size;
          slot.poolOffset = basePoolHead_;
          slot.pixelCount = ctx.writtenPixels;
          slot.lastUsed = ++cacheClock_;
          slot.valid = true;
          basePoolHead_ += ctx.writtenPixels;
          ++cacheMisses;
        } else {
          // Fallback path when no row-streaming inflater is attached (e.g. legacy host inflate_).
          evictPoolRange(basePoolHead_, basePoolHead_ + basePixels, nextOverlap, true);
          uint16_t* dst = basePool_ + basePoolHead_;
          const uint64_t started = microsNow();
          if (!decode(entry.base, dst, basePixels, header.baseWidth, header.baseWidth)) {
            padded_[outputIndex] = false;
            return false;
          }
          RowSpan* spans = cacheRowSpans_ + static_cast<size_t>(chosen) * kSpriteHeight;
          size_t packed = 0;
          for (int y = 0; y < header.baseHeight; ++y) {
            const uint16_t* row = dst + static_cast<size_t>(y) * header.baseWidth;
            int begin = 0, end = header.baseWidth;
            while (begin < end && row[begin] == 0) ++begin;
            while (end > begin && row[end - 1] == 0) --end;
            spans[y] = {static_cast<int16_t>(begin), static_cast<int16_t>(end)};
            if (end > begin) {
              const size_t len = static_cast<size_t>(end - begin);
              if (dst + packed != row + begin)
                std::memmove(dst + packed, row + begin, len * sizeof(uint16_t));
              packed += len;
            }
          }
          decodeUs = static_cast<uint32_t>(microsNow() - started);
          auto& slot = cacheSlots_[chosen];
          slot.offset = entry.base.offset;
          slot.size = entry.base.size;
          slot.poolOffset = basePoolHead_;
          slot.pixelCount = packed;
          slot.lastUsed = ++cacheClock_;
          slot.valid = true;
          basePoolHead_ += packed;
          ++cacheMisses;
          const uint64_t copyStart = microsNow();
          copyPackedSlotToFrame(chosen, art, outputIndex, header, entry);
          compositeUs = static_cast<uint32_t>(microsNow() - copyStart);
        }
      }
    } else {
      uint64_t started = microsNow();
      if (!decode(entry.base, art + static_cast<size_t>(header.baseY) * outputWidth_ + header.baseX,
                  basePixels, header.baseWidth, outputWidth_))
        return false;
      decodeUs = static_cast<uint32_t>(microsNow() - started);
      started = microsNow();
      if (!padded_[outputIndex]) {
        clearPadding(frame, top, header);
        padded_[outputIndex] = true;
      }
      for (int y = 0; y < entry.patchHeight; ++y) {
        std::memcpy(openPatches_[outputIndex] + static_cast<size_t>(y) * entry.patchWidth,
                    art + static_cast<size_t>(entry.patchY + y) * outputWidth_ + entry.patchX,
                    entry.patchWidth * sizeof(uint16_t));
      }
      compositeUs = static_cast<uint32_t>(microsNow() - started);
    }
  }
  baseKeys_[outputIndex] = index;
  eyes = patchArea;

  const uint64_t started = microsNow();
  if (patchPixels && (sameBase || pose.blinkLevel || pose.blinkBlend)) {
    const uint16_t* pixels = openPatches_[outputIndex];
    if (pose.blinkLevel) {
      if (!decode(entry.blinks[pose.blinkLevel - 1], patch_, patchPixels, entry.patchWidth, entry.patchWidth))
        return false;
      pixels = patch_;
    }
    for (int y = 0; y < entry.patchHeight; ++y) {
      const int sy = entry.patchY + y;
      std::memcpy(art + static_cast<size_t>(sy) * outputWidth_ + entry.patchX,
                  pixels + static_cast<size_t>(y) * entry.patchWidth, entry.patchWidth * sizeof(uint16_t));
      if (useCache && sy >= header.baseY && sy < header.baseY + header.baseHeight) {
        const int pb = std::max(0, static_cast<int>(entry.patchX) - header.baseX);
        const int pe = std::min<int>(header.baseWidth, static_cast<int>(entry.patchX + entry.patchWidth) - header.baseX);
        if (pb < pe) {
          RowSpan& shown = shown_[outputIndex][sy - header.baseY];
          if (shown.begin >= shown.end) {
            shown = {static_cast<int16_t>(pb), static_cast<int16_t>(pe)};
          } else {
            shown.begin = static_cast<int16_t>(std::min<int>(shown.begin, pb));
            shown.end = static_cast<int16_t>(std::max<int>(shown.end, pe));
          }
        }
      }
    }
    if (pose.blinkBlend) {
      if (!decode(entry.blinks[pose.blinkLevel], patch_, patchPixels,
                  entry.patchWidth, entry.patchWidth))
        return false;
      const int revealHalfHeight = blinkRevealHalfHeight(pose.blinkBlend, entry.patchHeight);
      const int center = entry.patchHeight / 2;
      for (int y = 0; y < entry.patchHeight; ++y) {
        uint16_t* output = art + static_cast<size_t>(entry.patchY + y) * outputWidth_ + entry.patchX;
        const uint16_t* next = patch_ + static_cast<size_t>(y) * entry.patchWidth;
        if (std::abs(y - center) >= revealHalfHeight)
          std::memcpy(output, next, entry.patchWidth * sizeof(uint16_t));
      }
    }
  }
  eyesUs = static_cast<uint32_t>(microsNow() - started);
  outputKeys_[outputIndex] = key;
  return true;
}
}
