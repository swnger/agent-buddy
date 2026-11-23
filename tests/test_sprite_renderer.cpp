#include "../firmware/AgentCompanion/src/SpriteRenderer.h"
#include "../firmware/AgentCompanion/src/SpriteStorage.h"
#include "../tools/HostSpriteInflate.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

extern "C" const uint8_t kCopilotPack[], kCopilotPackEnd[];

using namespace copilot;
static unsigned calls = 0;
static bool failInflate = false;
static bool inflate(uint8_t* output, size_t size, const uint8_t* input, size_t compressed, size_t width, size_t stride) {
  ++calls;
  if (failInflate) {
    std::memset(output, 0xa5, std::min<size_t>(128, size));
    return false;
  }
  return inflateSpriteHost(output, size, input, compressed, width, stride);
}

int main() {
  assert(!initializeSpriteStorage() && spriteStorageError());
  assert(loadCharacterPack(kCopilotPack, kCopilotPackEnd - kCopilotPack));
  assert(initializeSpriteStorage());
  const CharacterPack& pack = *characterPack();
  const PackHeader& header = pack.header;
  const uint32_t kSpriteDataSize = header.dataBytes;
  const uint8_t* kSpriteData = pack.data;
  const SpriteFrame* kSpriteFrames = pack.frames;
  const int kSpriteBaseX = header.baseX, kSpriteBaseY = header.baseY;
  const int kSpriteBaseWidth = header.baseWidth, kSpriteBaseHeight = header.baseHeight;
  assert(std::string(header.id) == "copilot" && header.layout == PackLayout::BasePatch);
  assert(spriteBlock({0, kSpriteDataSize}) == kSpriteData);
  assert(!spriteBlock({kSpriteDataSize, 1}));
  assert(!spriteBlock({0, 0}));
  assert(!spriteBlock({1, kSpriteDataSize}));
  constexpr size_t sourcePixels = kSpriteWidth * kSpriteHeight;
  constexpr size_t outputPixels = sourcePixels;
  std::vector<uint16_t> expected(sourcePixels);
  const size_t patchPixels = header.maxPatchPixels;
  const size_t guardedPatch = patchPixels + 2;
  std::vector<uint16_t> firstOpen(guardedPatch, 0xa55a), secondOpen(guardedPatch, 0xa55a);
  std::vector<uint16_t> patch(guardedPatch, 0xa55a);
  std::vector<uint16_t> guarded(outputPixels + 2, 0xa55a), second(outputPixels);
  uint16_t* frame = guarded.data() + 1;
  SpriteRenderer renderer(firstOpen.data() + 1, secondOpen.data() + 1, patch.data() + 1,
                          patchPixels, frame, second.data(), inflate);
  constexpr size_t canvasPixels = kCharacterFrameWidth * kCharacterFrameHeight;
  std::vector<uint16_t> canvas(canvasPixels + 2, 0xa55a), otherCanvas(canvasPixels);
  std::vector<uint16_t> expectedCanvas(canvasPixels);
  std::vector<uint16_t> canvasOpen(guardedPatch), otherOpen(guardedPatch), canvasPatch(guardedPatch);
  uint16_t* canvasFrame = canvas.data() + 1;
  SpriteRenderer expanded(canvasOpen.data(), otherOpen.data(), canvasPatch.data(),
                          patchPixels, canvasFrame, otherCanvas.data(), inflate,
                          kCharacterFrameWidth, kCharacterFrameHeight);
  // One output, as CharacterScaler uses it. Over the same base image only the eyes change.
  std::vector<uint16_t> singleOutput(outputPixels + 2, 0xa55a), singleBefore(outputPixels);
  std::vector<uint16_t> singleOpen(guardedPatch), unusedOpen(guardedPatch), singlePatch(guardedPatch);
  uint16_t* singleFrame = singleOutput.data() + 1;
  SpriteRenderer single(singleOpen.data(), unusedOpen.data(), singlePatch.data(), patchPixels,
                        singleFrame, nullptr, inflate);
  auto checkSingle = [&](const SpritePose& pose, bool sameBase) {
    std::copy_n(singleFrame, outputPixels, singleBefore.begin());
    assert(single.render(pose, singleFrame));
    const SpriteArea area = single.eyes;
    assert(area.empty() || (area.x >= 0 && area.y >= 0 && area.x + area.width <= kSpriteWidth
                            && area.y + area.height <= kSpriteHeight));
    for (size_t i = 0; sameBase && i < outputPixels; ++i) {
      if (singleFrame[i] == singleBefore[i]) continue;
      const int x = static_cast<int>(i % kSpriteWidth), y = static_cast<int>(i / kSpriteWidth);
      assert(x >= area.x && x < area.x + area.width && y >= area.y && y < area.y + area.height);
    }
    assert(single.render(pose, singleFrame) && single.eyes.empty());
    assert(singleOutput.front() == 0xa55a && singleOutput.back() == 0xa55a);
  };
  std::ofstream hashes("build/sprite-renderer-pixels.fnv");
  for (int index = 0; index < kSpriteFrameCount; ++index) {
    const auto& metadata = kSpriteFrames[index];
    unsigned visit = 0;
    for (int level : {0, 1, 2, 3, 4, 3, 2, 1, 0}) {
      std::fill(expected.begin(), expected.end(), 0);
      auto* base = expected.data() + kSpriteBaseY * kSpriteWidth + kSpriteBaseX;
      assert(inflateSpriteHost(reinterpret_cast<uint8_t*>(base), kSpriteBaseWidth * kSpriteBaseHeight * 2,
                              kSpriteData + metadata.base.offset, metadata.base.size, kSpriteBaseWidth, kSpriteWidth));
      if (level && metadata.patchWidth) {
        std::vector<uint16_t> changes(metadata.patchWidth * metadata.patchHeight);
        const auto& block = metadata.blinks[level - 1];
        assert(inflateSpriteHost(reinterpret_cast<uint8_t*>(changes.data()), changes.size() * 2,
                                kSpriteData + block.offset, block.size, metadata.patchWidth, metadata.patchWidth));
        for (int y = 0; y < metadata.patchHeight; ++y) {
          std::copy_n(changes.data() + y * metadata.patchWidth, metadata.patchWidth,
                      expected.data() + (metadata.patchY + y) * kSpriteWidth + metadata.patchX);
        }
      }
      uint8_t direction = 0;
      while (direction + 1 < kSpriteDirections && kSpriteTrackOffsets[direction + 1] <= index) ++direction;
      SpritePose pose{direction, static_cast<uint8_t>(index - kSpriteTrackOffsets[direction]),
                      static_cast<uint8_t>(level)};
      const auto previousSecond = second;
      assert(renderer.render(pose, frame));
      assert(second == previousSecond);
      uint64_t hash = UINT64_C(14695981039346656037);
      const auto* bytes = reinterpret_cast<const uint8_t*>(frame);
      for (size_t i = 0; i < outputPixels * 2; ++i) {
        hash = (hash ^ bytes[i]) * UINT64_C(1099511628211);
      }
      if (visit++ < kSpriteBlinkLevels) hashes << index << " " << level << " " << hash << "\n";
      assert(std::memcmp(frame, expected.data(), outputPixels * 2) == 0);
      checkSingle(pose, visit > 1);
      assert(std::memcmp(singleFrame, expected.data(), outputPixels * 2) == 0);
      for (int y = 0; y < kSpriteHeight; ++y)
        std::copy_n(expected.data() + y * kSpriteWidth, kSpriteWidth,
                    expectedCanvas.data() + (y + kFrameY) * kCharacterFrameWidth);
      assert(expanded.render(pose, canvasFrame));
      assert(std::equal(expectedCanvas.begin(), expectedCanvas.end(), canvasFrame));
      assert(expanded.render(pose, otherCanvas.data()));
      assert(expectedCanvas == otherCanvas);
      assert(canvas.front() == 0xa55a && canvas.back() == 0xa55a);
      assert(guarded.front() == 0xa55a && guarded.back() == 0xa55a);
      const unsigned before = calls;
      assert(renderer.render(pose, frame));
      assert(calls == before);
      assert(renderer.render(pose, second.data()));
      assert(std::memcmp(frame, second.data(), outputPixels * 2) == 0);
      const unsigned after = calls;
      assert(renderer.render(pose, second.data()) && calls == after);
      for (const auto* buffer : {&firstOpen, &secondOpen, &patch}) {
        assert(buffer->front() == 0xa55a && buffer->back() == 0xa55a);
      }
    }
  }
  for (int level = 0; level < kSpriteBlinkLevels; ++level) {
    assert(renderer.render({0, 0, static_cast<uint8_t>(level)}, frame));
    for (int direction = 1; direction < kSpriteDirections; ++direction) {
      assert(renderer.render({static_cast<uint8_t>(direction), 0, static_cast<uint8_t>(level)}, second.data()));
      assert(std::memcmp(frame, second.data(), outputPixels * 2) == 0);
    }
  }
  assert(renderer.render({0, 0, 0}, frame));
  const unsigned centerCalls = calls;
  assert(renderer.render({1, 0, 0}, frame));
  assert(calls == centerCalls);
  assert(!renderer.render({static_cast<uint8_t>(kSpriteDirections), 0, 0}, frame) && renderer.error());
  assert(!renderer.render({0, 24, 0}, frame) && renderer.error());
  for (uint8_t direction : {2, 3}) {
    assert(kSpriteTrackSteps[direction] == 12);
    assert(renderer.render({direction, 11, 0}, frame));
    assert(!renderer.render({direction, 12, 0}, frame) && renderer.error());
  }
  assert(!renderer.render({0, 0, 5}, frame) && renderer.error());
  assert(!renderer.render({0, 0, 0}, nullptr) && renderer.error());
  assert(!renderer.render({0, 0, 0}, expected.data()) && renderer.error());
  assert(renderer.render({0, 0, 0}, frame));
  const std::vector<uint16_t> previous(frame, frame + outputPixels);
  failInflate = true;
  assert(!renderer.render({4, 12, 0}, frame) && renderer.error());
  failInflate = false;
  assert(renderer.render({0, 0, 0}, frame) && !renderer.error());
  assert(std::equal(previous.begin(), previous.end(), frame));
  assert(renderer.render({4, 12, 0}, frame) && !renderer.error());
  const std::vector<uint16_t> open(frame, frame + outputPixels);
  failInflate = true;
  assert(!renderer.render({4, 12, 2}, frame) && renderer.error());
  failInflate = false;
  assert(renderer.render({4, 12, 0}, frame) && !renderer.error());
  assert(std::equal(open.begin(), open.end(), frame));
  // A single output takes only its own buffer.
  assert(!single.render({0, 0, 0}, frame) && single.error());
  SpriteRenderer noOutput(singleOpen.data(), unusedOpen.data(), singlePatch.data(), patchPixels,
                          nullptr, frame, inflate);
  assert(!noOutput.render({0, 0, 0}, frame) && noOutput.error());
  SpriteRenderer aliased(firstOpen.data() + 1, firstOpen.data() + 1, patch.data() + 1,
                         patchPixels, frame, second.data(), inflate);
  SpriteRenderer undersized(firstOpen.data() + 1, secondOpen.data() + 1, patch.data() + 1,
                            patchPixels - 1, frame, second.data(), inflate);
  assert(!undersized.render({0, 0, 0}, frame) && undersized.error());
  assert(!aliased.render({0, 0, 0}, frame) && aliased.error());
  assert(expanded.render({0, 0, 0}, canvasFrame));
  const std::vector<uint16_t> previousCanvas(canvasFrame, canvasFrame + canvasPixels);
  failInflate = true;
  assert(!expanded.render({4, 12, 0}, canvasFrame) && expanded.error());
  failInflate = false;
  assert(expanded.render({0, 0, 0}, canvasFrame));
  assert(std::equal(previousCanvas.begin(), previousCanvas.end(), canvasFrame));
  SpriteRenderer tooSmall(canvasOpen.data(), otherOpen.data(), canvasPatch.data(),
                          patchPixels, canvasFrame, otherCanvas.data(), inflate, 399, 466);
  assert(!tooSmall.render({0, 0, 0}, canvasFrame) && tooSmall.error());
  SpriteRenderer wrongStride(canvasOpen.data(), otherOpen.data(), canvasPatch.data(),
                             patchPixels, canvasFrame, otherCanvas.data(), inflate, 414, 466);
  assert(!wrongStride.render({0, 0, 0}, canvasFrame) && wrongStride.error());

  // Packed-span ring cache + single-pass cold-miss + prefetch + tile-mask tests.
  {
    const size_t basePixels = static_cast<size_t>(kSpriteBaseWidth) * kSpriteBaseHeight;
    std::vector<uint16_t> poolBuf(3 * basePixels, 0);
    std::vector<SpriteRenderer::RowSpan> rowSpans(4 * kSpriteHeight);
    std::vector<uint16_t> cOpen0(guardedPatch), cOpen1(guardedPatch), cPatch(guardedPatch);
    std::vector<uint16_t> cFrame0(outputPixels, 0), cFrame1(outputPixels, 0);
    static SpritePredictor hostPredictor;
    static unsigned rowInflateCalls = 0;
    auto inflateRowsHost = [](const uint8_t* input, size_t inputSize, size_t width, size_t height,
                              SpriteRenderer::RowCallback onRow, void* ctx) -> bool {
      ++rowInflateCalls;
      std::vector<uint8_t> raw(width * height * 2);
      uLongf destLen = static_cast<uLongf>(raw.size());
      if (uncompress(raw.data(), &destLen, input, static_cast<uLong>(inputSize)) != Z_OK
          || destLen != raw.size()) return false;
      return hostPredictor.resetRows(width, height, onRow, ctx)
          && hostPredictor.consume(raw.data(), raw.size())
          && hostPredictor.complete();
    };
    SpriteRenderer cached(cOpen0.data(), cOpen1.data(), cPatch.data(), patchPixels,
                          cFrame0.data(), cFrame1.data(), inflate);
    cached.setBaseCache(poolBuf.data(), poolBuf.size(), rowSpans.data(), 4,
                        nullptr, 0, inflateRowsHost);
    rowInflateCalls = 0;
    assert(cached.render({0, 5, 0}, cFrame0.data()));
    assert(cached.cacheMisses == 1 && cached.cacheHits == 0 && rowInflateCalls == 1);
    assert(cached.hasCachedPose(0, 5));
    // Second framebuffer hits packed cache instead of re-inflating.
    assert(cached.render({0, 5, 0}, cFrame1.data()));
    assert(cached.cacheMisses == 1 && cached.cacheHits == 1 && rowInflateCalls == 1);
    assert(cFrame0 == cFrame1);
    // Verify preparePrefetch / commitPrefetch makes subsequent render an immediate cache hit.
    SpriteRenderer::PrefetchTarget pt{};
    assert(cached.preparePrefetch(0, 6, pt));
    struct PrefetchAcc {
      SpriteRenderer* r;
      SpriteRenderer::PrefetchTarget* t;
      size_t written = 0;
      bool overflow = false;
    } acc{&cached, &pt, 0, false};
    assert(inflateRowsHost(kSpriteData + pt.block.offset, pt.block.size, pt.baseWidth, pt.baseHeight,
                           [](void* o, size_t y, const uint16_t* row, int b, int e) {
                             auto* a = static_cast<PrefetchAcc*>(o);
                             if (a->overflow) return;
                             if (b >= e) b = e = 0;
                             const size_t count = static_cast<size_t>(e - b);
                             if (count && a->t->poolOffset + a->written + count > a->t->nextEvictOffset) {
                               if (!a->r->evictPoolRange(a->t->poolOffset, a->t->poolOffset + a->written + count,
                                                         a->t->nextEvictOffset, false)) {
                                 a->overflow = true;
                                 return;
                               }
                             }
                             a->t->slotSpans[y] = {static_cast<int16_t>(b), static_cast<int16_t>(e)};
                             if (count) {
                               std::memcpy(a->t->poolDst + a->written, row + b, count * sizeof(uint16_t));
                               a->written += count;
                             }
                           },
                           &acc) && !acc.overflow);
    cached.commitPrefetch(pt, acc.written);
    assert(cached.hasCachedPose(0, 6));
    assert(cached.hasBaseForFrame(0, 5, cFrame0.data()));
    assert(!cached.hasBaseForFrame(0, 6, cFrame0.data()));
    const unsigned hitsBefore = cached.cacheHits;
    assert(cached.render({0, 6, 0}, cFrame0.data()));
    assert(cached.cacheHits == hitsBefore + 1);
    assert(cached.hasBaseForFrame(0, 6, cFrame0.data()));
    assert(renderer.render({0, 6, 0}, frame));
    assert(std::memcmp(frame, cFrame0.data(), outputPixels * 2) == 0);

    // Verify pinPose protects a pose across ring wraps and lazy eviction preserves untouched slots on cancel.
    cached.pinPose(0, 6);
    for (uint8_t idx : {7, 8, 9, 10, 11}) {
      SpriteRenderer::PrefetchTarget p{};
      if (cached.preparePrefetch(0, idx, p)) {
        PrefetchAcc pAcc{&cached, &p, 0, false};
        assert(inflateRowsHost(kSpriteData + p.block.offset, p.block.size, p.baseWidth, p.baseHeight,
                               [](void* o, size_t y, const uint16_t* row, int b, int e) {
                                 auto* a = static_cast<PrefetchAcc*>(o);
                                 if (a->overflow) return;
                                 if (b >= e) b = e = 0;
                                 const size_t count = static_cast<size_t>(e - b);
                                 if (count && a->t->poolOffset + a->written + count > a->t->nextEvictOffset) {
                                   if (!a->r->evictPoolRange(a->t->poolOffset, a->t->poolOffset + a->written + count,
                                                             a->t->nextEvictOffset, false)) {
                                     a->overflow = true;
                                     return;
                                   }
                                 }
                                 a->t->slotSpans[y] = {static_cast<int16_t>(b), static_cast<int16_t>(e)};
                                 if (count) {
                                   std::memcpy(a->t->poolDst + a->written, row + b, count * sizeof(uint16_t));
                                   a->written += count;
                                 }
                               },
                               &pAcc) && !pAcc.overflow);
        cached.commitPrefetch(p, pAcc.written);
      }
      assert(cached.hasCachedPose(0, 6));
    }
    // Cancelling a prefetch before writing preserves existing slots in its reserved range.
    const bool elevenWasCached = cached.hasCachedPose(0, 11);
    SpriteRenderer::PrefetchTarget cancelled{};
    if (cached.preparePrefetch(0, 12, cancelled)) {
      cached.cancelPrefetch(cancelled);
    }
    assert(cached.hasCachedPose(0, 6));
    if (elevenWasCached) assert(cached.hasCachedPose(0, 11));
    // Verify tile mask covers every non-black pixel in cFrame0.
    constexpr int kTileRows = (kSpriteHeight + 15) / 16;
    uint32_t mask[kTileRows]{};
    cached.fillTileMask(cFrame0.data(), mask, kTileRows);
    for (int y = 0; y < kSpriteHeight; ++y) {
      for (int x = 0; x < kSpriteWidth; ++x) {
        if (cFrame0[y * kSpriteWidth + x] != 0) {
          assert((mask[y >> 4] & (uint32_t(1) << (x >> 4))) != 0);
        }
      }
    }
    // Verify pixel-exact match against uncached renderer across ring wraps, evictions, and blinks.
    for (const SpritePose& pose : {SpritePose{0, 23, 0}, SpritePose{0, 23, 3}, SpritePose{0, 23, 0},
                                   SpritePose{1, 18, 0}, SpritePose{2, 8, 2}, SpritePose{0, 5, 0},
                                   SpritePose{4, 15, 0, 128}, SpritePose{9, 12, 0}, SpritePose{11, 20, 4},
                                   SpritePose{11, 20, 0}, SpritePose{0, 5, 0}}) {
      assert(renderer.render(pose, frame));
      assert(cached.render(pose, cFrame0.data()));
      assert(std::memcmp(frame, cFrame0.data(), outputPixels * 2) == 0);
      assert(cached.render(pose, cFrame1.data()));
      assert(std::memcmp(frame, cFrame1.data(), outputPixels * 2) == 0);
    }

    // Fill three equal ranges to force a wrap. The next two poses still need their slots
    // and pixels. Reservation must skip both and reuse the third range, even when all
    // metadata slots are occupied.
    cached.setBaseCache(poolBuf.data(), poolBuf.size(), rowSpans.data(), 3);
    for (uint8_t index : {5, 6, 7}) {
      assert(cached.preparePrefetch(0, index, pt));
      cached.commitPrefetch(pt, basePixels);
    }
    uint64_t keep = 0;
    assert(cached.hasCachedPose(0, 5, &keep));
    assert(cached.hasCachedPose(0, 6, &keep));
    assert(cached.preparePrefetch(0, 8, pt, keep) && pt.poolOffset == 2 * basePixels);
    cached.commitPrefetch(pt, basePixels);
    assert(cached.hasCachedPose(0, 8, &keep));
    assert(!cached.preparePrefetch(0, 9, pt, keep));
    for (uint8_t index : {5, 6, 8}) assert(cached.hasCachedPose(0, index));
  }

  std::ofstream image("build/sprite-host-frame.ppm", std::ios::binary);
  image << "P6\n" << kSpriteWidth << " " << kSpriteHeight << "\n255\n";
  for (size_t i = 0; i < outputPixels; ++i) {
    const uint16_t color = __builtin_bswap16(frame[i]);
    const char rgb[] = {static_cast<char>((color >> 11) * 255 / 31),
                        static_cast<char>(((color >> 5) & 63) * 255 / 63),
                        static_cast<char>((color & 31) * 255 / 31)};
    image.write(rgb, 3);
  }
  assert(image.good());
  assert(hashes.good());
  std::cout << "PASS: all " << kSpriteFrameCount * kSpriteBlinkLevels
            << " states, every output pixel, reopening, shared centers, independent caches, single outputs"
               " and recovery\n";
}
