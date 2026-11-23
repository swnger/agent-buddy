#pragma once
#include "FrameCoverage.h"
#include "ScreenOrientation.h"
#include "DisplayDma.h"
#include <Arduino_GFX_Library.h>
#include <cstddef>
#include <cstdint>

namespace copilot {
class OrientedDisplay : public Arduino_Canvas {
 public:
  // setBrightness is the board's brightness control: a panel command or a backlight.
  OrientedDisplay(board::DisplayPanel* panel, Arduino_DataBus* bus, const DisplayScanout& scanout,
                  void (*setBrightness)(uint8_t));
  bool begin(int32_t speed = GFX_NOT_DEFINED) override;
  void flush(bool forceFlush = false) override;
  bool flushFrame(const uint16_t* pixels, int x, int y, int width, int height);
  bool flushAnimation(const uint16_t* pixels, const FrameCoverage::Tiles& changes);
  bool setAngle(float radians);
  bool setTrim(float radians);
  float angle() const { return ScreenOrientation::shortestAngle(orientation_.angle() + trim_); }
  bool setTransferBuffer(uint8_t* buffer, size_t bytes);
  void setBrightness(uint8_t brightness) { setBrightness_(brightness); }
  const char* error() const { return error_; }
  uint32_t composeUs() const { return composeUs_; }
  uint32_t writeUs() const { return writeUs_; }

 private:
  bool flushSource(const ScreenPixels& source);
  bool writeWindow(const ScreenPixels& source, const ScreenTransform& transform,
                   int x, int y, int width, int height, const uint16_t* direct = nullptr);
  static constexpr int kTileColumns = (kPanelWidth + 15) / 16;
  static constexpr int kTileRows = (kPanelHeight + 15) / 16;
  static_assert(kTileColumns < 32 && (kCharacterFrameWidth + 15) / 16 < 32);
  ScreenTransform::TileRange samples_[kTileRows][kTileColumns];

  bool hasShown_ = false, samplesValid_ = false;
  board::DisplayPanel* panel_;
  void (*setBrightness_)(uint8_t);
  uint8_t* transferBuffer_ = nullptr;
  size_t transferBytes_ = 0;
  DisplayOrientation orientation_;
  float trim_ = 0.0f;
  bool frameMarginsDirty_ = true;
  const char* error_ = nullptr;
  uint32_t composeUs_ = 0, writeUs_ = 0;
  DisplayDma dma_;
};
}
