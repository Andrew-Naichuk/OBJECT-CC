// ILI9341 panel model: a 240x320 RGB565 framebuffer with the same
// address-window / writePixels interface the firmware uses via Adafruit_SPITFT.
#pragma once
#include "Adafruit_GFX.h"

class Adafruit_ILI9341 : public Adafruit_GFX {
 public:
  static const int W = 240, H = 320;
  uint16_t fb[W * H];
  bool inverted = false;

  Adafruit_ILI9341(int8_t, int8_t, int8_t) : Adafruit_GFX(W, H) { memset(fb, 0, sizeof fb); }
  void begin(uint32_t = 0) {}
  void invertDisplay(bool i) override { inverted = i; }

  void drawPixel(int16_t x, int16_t y, uint16_t c) override {
    if (x < 0 || y < 0 || x >= _width || y >= _height) return;
    // rotation 0 only (firmware calls setRotation(0))
    fb[y * W + x] = c;
  }

  // Adafruit_SPITFT-style raw window writes
  void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    wx_ = x; wy_ = y; ww_ = w; wh_ = h; wi_ = 0;
  }
  void writePixels(uint16_t *c, uint32_t n, bool = true, bool = false) {
    for (uint32_t k = 0; k < n; k++, wi_++) {
      if (ww_ == 0) return;
      int x = wx_ + (int)(wi_ % ww_);
      int y = wy_ + (int)(wi_ / ww_);
      if (y >= wy_ + wh_) return;
      drawPixel((int16_t)x, (int16_t)y, c[k]);
    }
  }

 private:
  int wx_ = 0, wy_ = 0, ww_ = 0, wh_ = 0;
  uint32_t wi_ = 0;
};
