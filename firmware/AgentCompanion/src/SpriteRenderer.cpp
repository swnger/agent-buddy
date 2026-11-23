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

}

SpriteRenderer::SpriteRenderer(uint16_t* firstOpenPatch, uint16_t* secondOpenPatch, uint16_t* patch,
                               size_t patchCapacity, uint16_t* firstOutput, uint16_t* secondOutput,
                               InflateSprite inflate, int outputWidth, int outputHeight)
    : openPatches_{firstOpenPatch, secondOpenPatch}, patch_(patch), patchCapacity_(patchCapacity),
      outputs_{firstOutput, secondOutput}, outputWidth_(outputWidth), outputHeight_(outputHeight), inflate_(inflate) {}

void SpriteRenderer::setBaseCache(Page* pages, size_t count, uint16_t* staging) {
  pages_ = pages;
  pageCount_ = pages && staging ? count : 0;
  staging_ = staging;
  invalidate();
}

void SpriteRenderer::invalidate() {
  for (unsigned i = 0; i < 2; ++i) {
    baseKeys_[i] = outputKeys_[i] = UINT32_MAX;
    padded_[i] = blinking_[i] = false;
  }
  for (auto& slot : slots_) slot = {};
  free_ = tail_ = nullptr;
  nextPage_ = clock_ = 0;
  writing_ = -1;
  packData_ = characterPack() ? characterPack()->data : nullptr;
}

void SpriteRenderer::fillTileMask(const uint16_t* frame, FrameCoverage::Tiles& tiles) const {
  if (!frame || (frame != outputs_[0] && frame != outputs_[1])) return;
  const int output = frame == outputs_[0] ? 0 : 1;
  if (baseKeys_[output] == UINT32_MAX) return;
  const auto& pack = *characterPack();
  const auto& h = pack.header;
  const int top = (outputHeight_ - kSpriteHeight) / 2;
  if (cached()) {
    for (int y = 0; y < h.baseHeight; ++y) {
      const auto span = shown_[output][y];
      FrameCoverage::mark(tiles, h.baseX + span.begin, top + h.baseY + y, span.end - span.begin, 1);
    }
  } else FrameCoverage::mark(tiles, h.baseX, top + h.baseY, h.baseWidth, h.baseHeight);
  const auto& eye = pack.frames[baseKeys_[output]];
  FrameCoverage::mark(tiles, eye.patchX, top + eye.patchY, eye.patchWidth, eye.patchHeight);
}

int SpriteRenderer::find(const SpritePose& pose) const {
  const auto* pack = characterPack();
  if (!cached() || !pack || packData_ != pack->data || pose.direction >= kSpriteDirections
      || pose.index >= kSpriteTrackSteps[pose.direction]) return -1;
  const auto& base = pack->frames[kSpriteTrackOffsets[pose.direction] + pose.index].base;
  for (size_t i = 0; i < kSlots; ++i)
    if (slots_[i].used && slots_[i].block.offset == base.offset && slots_[i].block.size == base.size) return i;
  return -1;
}

bool SpriteRenderer::hasCachedPose(const SpritePose& pose, uint64_t* keep) const {
  const int slot = find(pose);
  if (slot >= 0 && keep) *keep |= uint64_t(1) << slot;
  return slot >= 0;
}

bool SpriteRenderer::hasBaseForFrame(const SpritePose& pose, const uint16_t* frame) const {
  if (!frame || (frame != outputs_[0] && frame != outputs_[1]) || pose.direction >= kSpriteDirections
      || pose.index >= kSpriteTrackSteps[pose.direction]) return false;
  const auto key = baseKeys_[frame == outputs_[0] ? 0 : 1];
  if (key == UINT32_MAX) return false;
  const auto* entries = characterPack()->frames;
  const auto& old = entries[key];
  const auto& next = entries[kSpriteTrackOffsets[pose.direction] + pose.index];
  return old.base.offset == next.base.offset && old.base.size == next.base.size
      && old.patchX == next.patchX && old.patchY == next.patchY
      && old.patchWidth == next.patchWidth && old.patchHeight == next.patchHeight;
}

int SpriteRenderer::oldest(uint64_t keep, bool occupied) const {
  int victim = -1;
  for (size_t i = 0; i < kSlots; ++i) {
    if ((keep & (uint64_t(1) << i)) || (occupied && !slots_[i].first)) continue;
    if (victim < 0 || slots_[i].used < slots_[victim].used) victim = i;
  }
  return victim;
}

void SpriteRenderer::release(int slot) {
  Page* page = slots_[slot].first;
  while (page) {
    Page* next = page->next;
    page->next = free_;
    free_ = page;
    page = next;
  }
  slots_[slot] = {};
}

SpriteBlock SpriteRenderer::beginPrefetch(const SpritePose& pose, uint64_t keep) {
  if (!cached() || writing_ >= 0 || pose.direction >= kSpriteDirections
      || pose.index >= kSpriteTrackSteps[pose.direction] || hasCachedPose(pose)) return {};
  writing_ = oldest(keep, false);
  if (writing_ < 0) return {};
  release(writing_);
  keep_ = keep | (uint64_t(1) << writing_);
  tail_ = nullptr;
  return slots_[writing_].block = characterPack()->frames[kSpriteTrackOffsets[pose.direction] + pose.index].base;
}

// Each page holds whole rows: [first column, end column, pixels ...].
void SpriteRenderer::appendRow(const uint16_t* row, int begin, int end) {
  if (writing_ < 0) return;
  const size_t length = end - begin;
  if (!tail_ || written_ + 2 + length > kPageWords) {
    if (!free_ && nextPage_ == pageCount_) {
      const int victim = oldest(keep_, true);
      if (victim < 0) { finishPrefetch(false); return; }
      release(victim);
    }
    Page* page = free_ ? free_ : &pages_[nextPage_++];
    if (free_) free_ = free_->next;
    if (tail_) {
      tail_->used = written_;
      tail_->next = page;
    } else slots_[writing_].first = page;
    tail_ = page;
    page->next = nullptr;
    written_ = 0;
  }
  uint16_t* dst = tail_->words + written_;
  dst[0] = begin;
  dst[1] = end;
  std::memcpy(dst + 2, row + begin, length * sizeof(uint16_t));
  written_ += 2 + length;
}

bool SpriteRenderer::finishPrefetch(bool complete) {
  if (writing_ < 0) return false;
  if (complete && tail_) {
    tail_->used = written_;
    slots_[writing_].used = ++clock_;
  } else {
    release(writing_);
    complete = false;
  }
  writing_ = -1;
  return complete;
}

void SpriteRenderer::drawSlot(int slot, uint16_t* art, int output, const PackHeader& h) {
  slots_[slot].used = ++clock_;
  keep_ |= uint64_t(1) << slot;  // Protect a newly visible pose during an in-flight decode.
  int y = 0;
  for (const Page* page = slots_[slot].first; page; page = page->next) {
    const unsigned used = page->used;
    std::memcpy(staging_, page->words, used * sizeof(uint16_t));
    for (unsigned pos = 0; pos < used; ++y) {
      const int b = staging_[pos++], e = staging_[pos++];
      uint16_t* dst = art + (h.baseY + y) * outputWidth_ + h.baseX;
      const RowSpan old = shown_[output][y];
      if (old.begin < b) std::fill(dst + old.begin, dst + std::min<int>(old.end, b), 0);
      if (old.end > e) std::fill(dst + std::max<int>(old.begin, e), dst + old.end, 0);
      std::memcpy(dst + b, staging_ + pos, (e - b) * sizeof(uint16_t));
      shown_[output][y] = {static_cast<int16_t>(b), static_cast<int16_t>(e)};
      pos += e - b;
    }
  }
}

bool SpriteRenderer::decode(const SpriteBlock& block, uint16_t* output, size_t pixels, size_t width, size_t stride) {
  constexpr SpriteBlockErrors kErrors{"Sprite block lies outside the installed character pack.",
                                      "Sprite asset decompression failed."};
  error_ = inflateSpriteBlock(inflate_, block, output, pixels, width, stride, kErrors);
  return !error_;
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
  if (cached() && packData_ != pack->data) invalidate();
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
  const bool sameBase = hasBaseForFrame(pose, frame);
  if (!sameBase) {
    if (cached() && blinking_[outputIndex]) {
      // A blink can extend beyond both the old and the incoming base spans.
      const auto& old = pack->frames[baseKeys_[outputIndex]];
      for (int y = 0; y < old.patchHeight; ++y)
        std::memset(art + (old.patchY + y) * outputWidth_ + old.patchX, 0, old.patchWidth * 2);
    }
    blinking_[outputIndex] = false;
    baseKeys_[outputIndex] = UINT32_MAX;
    const int slot = find(pose);
    if (cached() && (!padded_[outputIndex] || slot < 0))
      std::fill_n(shown_[outputIndex], header.baseHeight, RowSpan{0, static_cast<int16_t>(header.baseWidth)});
    if (!padded_[outputIndex]) {
      clearPadding(frame, top, header);
      padded_[outputIndex] = true;
    }
    const uint64_t started = microsNow();
    if (slot >= 0) {
      drawSlot(slot, art, outputIndex, header);
      ++cacheHits;
    } else {
      if (!decode(entry.base, art + header.baseY * outputWidth_ + header.baseX,
                  basePixels, header.baseWidth, outputWidth_)) return false;
      decodeUs = microsNow() - started;
      ++cacheMisses;
    }
    compositeUs = microsNow() - started - decodeUs;
  }
  baseKeys_[outputIndex] = index;
  eyes = patchArea;

  const uint64_t started = microsNow();
  const bool blink = pose.blinkLevel || pose.blinkBlend;
  if (patchPixels && blink && !blinking_[outputIndex]) {
    for (int y = 0; y < entry.patchHeight; ++y)
      std::memcpy(openPatches_[outputIndex] + y * entry.patchWidth,
                  art + (entry.patchY + y) * outputWidth_ + entry.patchX, entry.patchWidth * 2);
    blinking_[outputIndex] = true;
  }
  if (blinking_[outputIndex]) {
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
  blinking_[outputIndex] = patchPixels && blink;
  eyesUs = static_cast<uint32_t>(microsNow() - started);
  outputKeys_[outputIndex] = key;
  return true;
}
}
