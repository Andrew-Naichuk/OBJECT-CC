/*
 * 3.2" IPS SPI TFT (ILI9341, 240x320) on Seeed XIAO nRF52840 Sense.
 * Cycling computer: reed-switch wheel speed + trip metrics + GPS status.
 *
 * Wiring (display module -> XIAO):
 *   VCC     -> 3V3
 *   GND     -> GND
 *   LCD_CS  -> D1
 *   LCD_RS  -> D2   (D/C)
 *   LCD_RST -> 7k -> 3.3V (held out of reset; not a GPIO)
 *   MOSI    -> D10
 *   SCK     -> D8
 *   MISO    -> D9
 *   LED     -> D3   (backlight PWM: short press cycles bright / dim / off)
 *   SD_CS   -> D5
 *
 * Reed switch (one pulse per wheel revolution):
 *   One side -> D0, other side -> GND
 *   INPUT_PULLUP; FALLING edge = magnet closed the switch.
 *
 * HGLRC M100 Pro GPS (Serial1, 115200 8N1):
 *   GPS GND -> XIAO GND
 *   GPS TX  -> D7 (XIAO RX)
 *   GPS RX  -> D6 (XIAO TX)
 *   GPS 5V  -> 5V
 *   Compass SCL/SDA unused.
 *
 * microSD slot on the display module (SPI shared with the TFT):
 *   CLK/MOSI/MISO are the LCD bus above, CS = SD_CS = D5.
 *
 * Tact button + battery sense (shared D4 / A4):
 *   Button D4 -> GND. LiPo BAT+ -> 100k -> D4 -> 100k -> GND, 100 nF D4 -> GND.
 *   Released D4 = Vbat/2, which is not a valid digital level, so D4 is only
 *   read with analogRead: raw < 400 (12-bit) = pressed.
 *   Short press (release before 2 s): backlight bright -> dim -> off
 *   (or confirm a pending phone connection).
 *   Hold 4 s while stopped: save the ride and start a new one.
 *
 * BLE file download (no extra wiring):
 *   Advertises as CONFIG ble_name (default "OBJECT-001"). On connect the
 *   device asks for a short press within 10 s; timeout disconnects with no
 *   file access. docs/index.html lists root *.GPX files, saves them on the
 *   phone, and can delete archived rides. CURRENT.GPX cannot be deleted over
 *   BLE. TRIP.DAT is not offered. Its Settings view reads and writes
 *   CONFIG.TXT; saved settings apply at once (a new BLE name on the next
 *   connection). Open that page over HTTPS (Android Chrome, or a Web
 *   Bluetooth browser on iPhone).
 *
 * BLE firmware update:
 *   The page reads FW_VERSION over BLE and compares it with the build
 *   published at docs/firmware/manifest.json. Accepting an update sends the
 *   new image over this same OBJECT service (Chrome blocklists the Nordic
 *   bootloader DFU UUID, so the image cannot go through AdaDFU from the
 *   browser). The sketch stages the image in unused flash, then copies it
 *   over itself and resets.
 *
 * microSD CONFIG.TXT (created with defaults on first boot if missing):
 *   wheel_circ_mm, timezone_offset_min, backlight, ble_name, units,
 *   backlight_dim, max_speed_kmh, stopped_ms, animations. See
 *   docs/CONFIG.TXT.example.
 *
 * Libraries (Arduino Library Manager):
 *   Adafruit ILI9341, Adafruit GFX Library, Adafruit BusIO
 * SdFat and Bluefruit are bundled with the Seeeduino nRF52 core
 * (do not install SdFat 2.3.x).
 *
 * Wheel default: 700x32C (ISO 32-622) -> circumference 2155 mm.
 *
 * TFT layout (portrait 240x320), monochrome to match the OBJECT ride page:
 *   OBJECT | GPS state, km/h or mph | Avg, dot-matrix speed, 24-dot gauge,
 *   Distance / Time / Moving / Max rows, Alt | card, phone, recording.
 * The gauge shows speed (2 km/h a dot), the new-ride hold, phone-connect
 * confirm countdown, or a phone download in progress.
 * Matrix animations (animations=on): boot self-test, wheel heartbeat, new-max
 * comet, 10 km / 10 mi milestones, a 0.0 face when stopped mid-ride, draining
 * digits on the new-ride hold, a firework on Ride saved, press-to-allow
 * chevrons and the Bluetooth mark on phone connect.
 * Rotation 0. If the image is upside down relative to the pin header, use 2.
 */

#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <SdFat.h>
#include <bluefruit.h>
#if defined(NRF52840_XXAA)
#include <InternalFileSystem.h>
#include "flash/flash_nrf5x.h"
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

// Build identity. The CI workflow writes xiao_oled/fw_version.h before it
// compiles; a build from this computer has no such file and reports "dev",
// which the hub always offers to replace with the published build.
#if defined(__has_include)
#if __has_include("fw_version.h")
#include "fw_version.h"
#endif
#endif
#ifndef FW_VERSION
#define FW_VERSION "dev"
#endif

static const int PIN_LCD_CS = D1;
static const int PIN_LCD_DC = D2;
static const int PIN_LCD_BL = D3;
static const int PIN_REED   = D0;
static const int PIN_BTN    = A4;
static const int PIN_SD_CS  = D5;

static const int SCREEN_W = 240;
static const int SCREEN_H = 320;
static const int PAD      = 12;

// Monochrome palette, the same one the OBJECT ride-transfer page uses.
static const uint16_t COL_BG   = 0x0000;  // black
static const uint16_t COL_FG   = 0xFFFF;  // white
static const uint16_t COL_DIM  = 0x8C71;  // #8f8f8f labels and captions
static const uint16_t COL_RULE = 0x2124;  // #242424 hairlines
static const uint16_t COL_OFF  = 0x18E3;  // #1c1c1c unlit matrix dots
static const uint16_t COL_LO   = 0x39E7;  // #3c3c3c animation trails

// Portrait stack, top to bottom.
static const int STATUS_BASE = 19;   // status-bar text baseline
static const int BATT_ICON_Y = 8;
static const int CAPTION_Y   = 34;
static const int CAPTION_H   = 20;
static const int HERO_Y      = 60;   // speed matrix top
static const int HERO_PITCH  = 10;   // matrix cell
static const int HERO_DOT    = 8;    // lit square inside the cell
// The speed is always "%4.1f": blocks of 5, 5, 1 and 5 columns, gaps between.
static const int MAT_COLS    = 19;
static const int MAT_ROWS    = 7;
static const int GAUGE_Y     = 142;  // dot centre line
static const int GAUGE_DOTS  = 24;
static const int GAUGE_PITCH = 9;
static const int GAUGE_R     = 2;
static const float GAUGE_KMH_PER_DOT = 2.0f;
static const int ROWS_Y      = 156;
static const int ROW_H       = 34;
static const int ROW_COUNT   = 4;
static const int VALUE_X     = 96;
static const int FOOTER_Y    = 296;
static const int FOOTER_H    = 22;

// Contact-bounce floor. Real minimum gap is also derived from max_speed_kmh
// so a double-fire after the floor cannot invent absurd km/h and lock Max.
static const unsigned long DEBOUNCE_MS = 15;

// Max only advances when two consecutive rev intervals agree within this
// ratio (min/max). Stops a single pothole / wobble / wire glitch locking Max.
static const float MAX_CONFIRM_RATIO = 0.85f;

static const int BTN_ADC_PRESSED = 400;   // 12-bit raw, ~0.35 V
static const unsigned long BTN_SAMPLE_MS = 10;
static const uint8_t BTN_STABLE_SAMPLES = 3;
static const unsigned long BATT_SETTLE_MS = 50;  // 100 nF recharge after release
static const float BATT_EMA_ALPHA = 0.02f;
static const unsigned long BTN_ARM_MS = 2000;
static const unsigned long BTN_EXEC_MS = 4000;
static const unsigned long BTN_FLASH_MS = 1500;
static const unsigned long BLE_AUTH_MS = 10000;

enum {
  BL_BRIGHT = 0,
  BL_DIM,
  BL_OFF,
  BL_MODE_COUNT
};

enum {
  UNITS_METRIC = 0,
  UNITS_IMPERIAL
};

static const char CFG_NAME[] = "CONFIG.TXT";
static const size_t CFG_BLE_NAME_MAX = 20;

// Ride settings from CONFIG.TXT (defaults match a 700x32C build).
struct Cfg {
  float wheelCircMm;
  int16_t timezoneOffsetMin;
  uint8_t backlight;       // BL_BRIGHT / BL_DIM / BL_OFF
  char bleName[CFG_BLE_NAME_MAX + 1];
  uint8_t units;           // UNITS_METRIC / UNITS_IMPERIAL
  uint8_t backlightDim;    // PWM duty for BL_DIM
  float maxSpeedKmh;
  unsigned long stoppedMs;
  uint8_t animations;      // 0 off, 1 on
};

static Cfg cfg = {
  2155.0f,
  0,
  BL_BRIGHT,
  "OBJECT-001",
  UNITS_METRIC,
  40,
  100.0f,
  3000UL,
  1
};

// Dim is ~16% by default so night use keeps the digits readable and cuts most
// of the backlight current. Off leaves the panel updating with the LEDs dark.
// Index BL_DIM is overwritten from cfg.backlightDim after CONFIG.TXT load.
static uint8_t BL_DUTY[BL_MODE_COUNT] = {255, 40, 0};

// Derived from wheel_circ_mm and max_speed_kmh; read by the reed ISR.
static volatile unsigned long g_minRevMs =
    (unsigned long)((2155.0f * 3.6f) / 100.0f + 0.5f);

static const unsigned long GPS_BAUD = 115200;
static const unsigned long GPS_STALE_MS = 2000;
static const unsigned long GPS_POLL_MS = 2000;
static const unsigned long GPS_DEBUG_MS = 2000;
static const unsigned long GPS_CFG_MS = 5000;
static const int PIN_GPS_RX = D7;
static const int PIN_GPS_TX = D6;

static const unsigned long GPX_MIN_MS = 1000;
static const unsigned long TRIP_DAT_MS = 5000;
// ~2 m of latitude in NAV-PVT 1e-7 deg units.
static const int32_t GPX_MIN_E7 = 180;

static const char GPX_NAME[] = "CURRENT.GPX";
static const char DAT_NAME[] = "TRIP.DAT";
static const char FIX_NAME[] = "LASTFIX.DAT";
static const uint32_t TRIP_MAGIC = 0x50495254UL;  // "TRIP"
static const uint16_t TRIP_VERSION = 1;
static const uint32_t FIX_MAGIC = 0x5849464CUL;  // "LFIX"
static const uint16_t FIX_VERSION = 1;
// Generous uncertainty so a regional last fix never misleads the search.
static const uint32_t FIX_POS_ACC_CM = 10000000UL;  // 100 km
// No RTC across power-off: claim the full U2 range (~18 h) so a same-day
// stale UTC still satisfies u-blox's accuracy rule.
static const uint16_t FIX_TIME_ACC_S = 65535;

static const char GPX_HEADER[] =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<gpx version=\"1.1\" creator=\"XIAO cycling computer\""
    " xmlns=\"http://www.topografix.com/GPX/1/1\""
    " xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\""
    " xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1"
    " http://www.topografix.com/GPX/1/1/gpx.xsd\">\n"
    "<trk>\n<name>Ride</name>\n<type>cycling</type>\n<trkseg>\n";
static const char GPX_FOOTER[] = "</trkseg>\n</trk>\n</gpx>\n";

// RST is tied to 3.3V. Hardware SPI: SCK D8, MOSI D10, MISO D9.
Adafruit_ILI9341 display(PIN_LCD_CS, PIN_LCD_DC, -1);

SdFat sd;
File32 gpxFile;

// --- Shared with ISR (keep volatile; never do heavy work in the ISR) ---
volatile unsigned long g_revCount = 0;       // total wheel revolutions
volatile unsigned long g_lastPulseMs = 0;    // millis() of most recent pulse
volatile unsigned long g_prevPulseMs = 0;    // millis() of pulse before that
volatile unsigned long g_olderPulseMs = 0;   // millis() of pulse before prev
volatile unsigned long g_lastIsrMs = 0;      // for debounce only

// --- Trip state (loop only) ---
static bool tripStarted = false;
static unsigned long elapsedBaseMs = 0;      // elapsed at last power-on / first pulse
static unsigned long elapsedAnchorMs = 0;    // millis() when elapsedBaseMs was set
static unsigned long movingMs = 0;
static unsigned long lastLoopMs = 0;
static float maxSpeedKmh = 0.0f;

static bool sdReady = false;
static uint32_t gpxBodyEnd = 0;
static unsigned long lastGpxMs = 0;
static unsigned long lastDatMs = 0;
static unsigned long gpxPointCount = 0;
static int32_t lastGpxLatE7 = 0;
static int32_t lastGpxLonE7 = 0;
static bool lastGpxPosValid = false;
static uint32_t lastGpxStamp = 0;
static bool lastGpxStampValid = false;

static bool btnStable = false;
static uint8_t btnAdcCount = 0;
static unsigned long btnAdcLastMs = 0;
static unsigned long btnReleaseMs = 0;
static float battVolts = -1.0f;
static bool btnHeld = false;
static bool btnArmed = false;
static bool btnDidExec = false;
static bool btnReleased = false;
static bool btnReleaseDidExec = false;
static unsigned long btnHoldStartMs = 0;
static unsigned long btnReleasedHeldMs = 0;
static uint8_t blMode = BL_BRIGHT;
static unsigned long overlayFlashUntilMs = 0;
static const char *overlayFlashMsg = NULL;

struct TextCache {
  char text[40];
  uint16_t color;
  uint8_t mark;
  bool valid;
};

struct Slot {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  int16_t base;
};

struct DotGlyph {
  char ch;
  uint8_t cols;
  uint8_t rows[7];
};

static TextCache txtStatus;
static TextCache txtBatt;
static int battBarsShown = -1;
static TextCache txtCaption;
static TextCache txtRow[ROW_COUNT];
static TextCache txtFootL;
static TextCache txtFootR;
// Matrix cells and gauge dots hold a level; flushes repaint only changes.
enum {
  LV_BG = 0,  // panel black, the gaps between digit blocks
  LV_OFF,
  LV_LO,
  LV_MID,
  LV_FG,
  LV_KEEP = 0xFE,
  LV_STALE = 0xFF
};
static uint8_t matNext[MAT_ROWS][MAT_COLS];
static uint8_t matShown[MAT_ROWS][MAT_COLS];
static uint8_t gaugeNext[GAUGE_DOTS];
static uint8_t gaugeShown[GAUGE_DOTS];
static bool uiChromeDrawn = false;

// Matrix animation triggers (see "Matrix animations").
struct AnimClip {
  bool on;
  unsigned long startMs;
};
static AnimClip clipBoot, clipBeat, clipFirework, clipAuth, clipRune;
static AnimClip clipNewMax, clipMsLive, clipMilestone;
static bool animBootPending = false;
static float animMaxBaseline = 0.0f;
static long msBaseline = 0;
static bool msBaselineValid = false;
static uint16_t msShowValue = 0;
static unsigned long msEndMs = 0;
static unsigned long animRevSeen = 0;
static unsigned long zeroSinceMs = 0;

#pragma pack(push, 1)
struct TripDat {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t revCount;
  uint32_t movingMs;
  uint32_t elapsedMs;
  float maxSpeedKmh;
  uint32_t gpxBodyEnd;
  uint32_t crc;
};

// Survives new-ride resets; used only as UBX-MGA-INI aid (never as a track point).
struct LastFixDat {
  uint32_t magic;
  uint16_t version;
  uint16_t flags;       // bit0 pos, bit1 time, bit2 alt
  int32_t latE7;
  int32_t lonE7;
  int32_t altCm;        // approx WGS84; posAcc covers error
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t sec;
  uint32_t crc;
};
#pragma pack(pop)

enum {
  FIX_FLAG_POS = 0x0001,
  FIX_FLAG_TIME = 0x0002,
  FIX_FLAG_ALT = 0x0004
};

static LastFixDat lastFix;
static bool lastFixValid = false;
static bool lastFixDirty = false;
static unsigned long lastFixSaveMs = 0;

enum {
  GPS_IDLE = 0,
  GPS_NMEA,
  GPS_UBX_SYNC2,
  GPS_UBX_HDR,
  GPS_UBX_PAYLOAD,
  GPS_UBX_CKA,
  GPS_UBX_CKB,
  GPS_UBX_SKIP
};

struct GpsState {
  bool valid;
  bool satsKnown;
  bool altKnown;
  bool posKnown;
  bool timeKnown;
  uint8_t sats;
  uint8_t fixType;
  float altM;
  int32_t latE7;
  int32_t lonE7;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t sec;
  unsigned long lastFrameMs;
  unsigned long lastRxMs;
  unsigned long lastPollMs;
  unsigned long lastDebugMs;
  unsigned long lastCfgMs;
  unsigned long rxBytes;
  unsigned long ubxOk;
  unsigned long nmeaOk;
  unsigned long ckFail;
  const char *src;
  uint8_t parseState;
  char nmea[120];
  uint8_t nmeaLen;
  uint8_t ubxHdr[4];
  uint8_t ubxHdrLen;
  uint8_t ubxPayload[96];
  uint16_t ubxLen;
  uint16_t ubxGot;
  uint16_t ubxSkipLeft;
  uint8_t ubxCkA;
  uint8_t ubxCkB;
  uint8_t ubxCkACalc;
  uint8_t ubxCkBCalc;
  uint8_t dump[32];
  uint8_t dumpLen;
};

static GpsState gps;

static void lastFixCaptureFromGps();
static bool lastFixSave();
static bool lastFixLoad();
static void gpsInjectAid();
static bool blePhoneConnected();
static bool bleAwaitingAuth();
static void bleConfirmAuth(unsigned long now);
static void backlightApply();

static void clipStart(AnimClip &c, unsigned long now) {
  c.on = true;
  c.startMs = now;
}

// Ride celebrations belong to one ride; a new ride starts their baselines over.
static void animResetRide() {
  animMaxBaseline = 0.0f;
  msBaselineValid = false;
  clipNewMax.on = false;
  clipNewMax.startMs = 0;
  clipMsLive.on = false;
  clipMilestone.on = false;
}

void reedIsr() {
  unsigned long now = millis();
  // Drop edges closer than bounce floor or the interval at max_speed_kmh.
  unsigned long minGap = g_minRevMs;
  if (minGap < DEBOUNCE_MS) {
    minGap = DEBOUNCE_MS;
  }
  if (now - g_lastIsrMs < minGap) {
    return;
  }
  g_lastIsrMs = now;

  g_olderPulseMs = g_prevPulseMs;
  g_prevPulseMs = g_lastPulseMs;
  g_lastPulseMs = now;
  g_revCount++;
}

static uint32_t leU32(const uint8_t *p) {
  return (uint32_t)p[0]
       | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16)
       | ((uint32_t)p[3] << 24);
}

static uint16_t leU16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int32_t leI32(const uint8_t *p) {
  return (int32_t)leU32(p);
}

static void ubxChecksumAdd(uint8_t b, uint8_t *ckA, uint8_t *ckB) {
  *ckA = (uint8_t)(*ckA + b);
  *ckB = (uint8_t)(*ckB + *ckA);
}

static void gpsSendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;
  uint8_t lenLo = (uint8_t)(len & 0xFF);
  uint8_t lenHi = (uint8_t)(len >> 8);

  Serial1.write(0xB5);
  Serial1.write(0x62);
  ubxChecksumAdd(cls, &ckA, &ckB);
  Serial1.write(cls);
  ubxChecksumAdd(id, &ckA, &ckB);
  Serial1.write(id);
  ubxChecksumAdd(lenLo, &ckA, &ckB);
  Serial1.write(lenLo);
  ubxChecksumAdd(lenHi, &ckA, &ckB);
  Serial1.write(lenHi);
  for (uint16_t i = 0; i < len; i++) {
    ubxChecksumAdd(payload[i], &ckA, &ckB);
    Serial1.write(payload[i]);
  }
  Serial1.write(ckA);
  Serial1.write(ckB);
}

static void gpsSetMsgRate(uint8_t cls, uint8_t id, uint8_t rate) {
  const uint8_t payload[] = {cls, id, 0x00, rate, 0x00, 0x00, 0x00, 0x00};
  gpsSendUbx(0x06, 0x01, payload, sizeof(payload));
}

// Quiet the M100 Pro: 1 Hz, UBX NAV-PVT + NMEA GGA only.
// Serial1's RX ring is 64 bytes; 10 Hz PVT (~102 B) plus NMEA overruns it
// during TFT refresh, so we never see another valid frame after the first.
static void gpsConfigure() {
  const uint8_t rate[] = {0xE8, 0x03, 0x01, 0x00, 0x01, 0x00};  // 1000 ms
  gpsSendUbx(0x06, 0x08, rate, sizeof(rate));

  // CFG-PRT UART1: 115200 8N1, in UBX+NMEA, out UBX.
  const uint8_t prt[] = {
    0x01, 0x00, 0x00, 0x00,
    0xD0, 0x08, 0x00, 0x00,
    0x00, 0xC2, 0x01, 0x00,
    0x07, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00
  };
  gpsSendUbx(0x06, 0x00, prt, sizeof(prt));

  gpsSetMsgRate(0x01, 0x07, 1);  // NAV-PVT
  gpsSetMsgRate(0xF0, 0x00, 1);  // GGA
  gpsSetMsgRate(0xF0, 0x01, 0);  // GLL
  gpsSetMsgRate(0xF0, 0x02, 0);  // GSA
  gpsSetMsgRate(0xF0, 0x03, 0);  // GSV
  gpsSetMsgRate(0xF0, 0x04, 0);  // RMC
  gpsSetMsgRate(0xF0, 0x05, 0);  // VTG

  // M10 RAM: NAV-PVT on UART1, 1 Hz meas rate, NMEA output off.
  const uint8_t pvtOn[]   = {0x00, 0x01, 0x00, 0x00, 0x07, 0x00, 0x91, 0x20, 0x01};
  const uint8_t nmeaOff[] = {0x00, 0x01, 0x00, 0x00, 0x02, 0x00, 0x74, 0x10, 0x00};
  const uint8_t meas1hz[] = {0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0xE8, 0x03};
  gpsSendUbx(0x06, 0x8A, pvtOn, sizeof(pvtOn));
  gpsSendUbx(0x06, 0x8A, nmeaOff, sizeof(nmeaOff));
  gpsSendUbx(0x06, 0x8A, meas1hz, sizeof(meas1hz));

  gps.lastCfgMs = millis();
}

static void gpsPollNavPvt() {
  gpsSendUbx(0x01, 0x07, NULL, 0);
  gps.lastPollMs = millis();
}

static void putU16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static void putU32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void putI32(uint8_t *p, int32_t v) {
  putU32(p, (uint32_t)v);
}

// Seed the M10 search from LASTFIX.DAT. Does not mark gps.valid / posKnown
// and is never written into CURRENT.GPX.
static void gpsInjectAid() {
  if (!lastFixValid) {
    return;
  }

  if (lastFix.flags & FIX_FLAG_POS) {
    uint8_t pos[20];
    memset(pos, 0, sizeof(pos));
    pos[0] = 0x01;  // MGA-INI-POS_LLH
    pos[1] = 0x00;
    putI32(pos + 4, lastFix.latE7);
    putI32(pos + 8, lastFix.lonE7);
    putI32(pos + 12, (lastFix.flags & FIX_FLAG_ALT) ? lastFix.altCm : 0);
    putU32(pos + 16, FIX_POS_ACC_CM);
    gpsSendUbx(0x13, 0x40, pos, sizeof(pos));
  }

  if (lastFix.flags & FIX_FLAG_TIME) {
    uint8_t tim[24];
    memset(tim, 0, sizeof(tim));
    tim[0] = 0x10;  // MGA-INI-TIME_UTC
    tim[1] = 0x00;
    tim[2] = 0x00;  // source = on receipt
    tim[3] = 0x80;  // leap seconds unknown
    putU16(tim + 4, lastFix.year);
    tim[6] = lastFix.month;
    tim[7] = lastFix.day;
    tim[8] = lastFix.hour;
    tim[9] = lastFix.minute;
    tim[10] = lastFix.sec;
    // bitfield0 / ns already 0
    putU16(tim + 16, FIX_TIME_ACC_S);
    // reserved + tAccNs already 0
    gpsSendUbx(0x13, 0x40, tim, sizeof(tim));
  }

  Serial.print("GPS aid injected");
  if (lastFix.flags & FIX_FLAG_POS) {
    Serial.print(" pos");
  }
  if (lastFix.flags & FIX_FLAG_TIME) {
    Serial.print(" time");
  }
  Serial.println();
}

static void gpsResetParser() {
  gps.parseState = GPS_IDLE;
  gps.nmeaLen = 0;
  gps.ubxHdrLen = 0;
  gps.ubxGot = 0;
  gps.ubxSkipLeft = 0;
  gps.ubxCkACalc = 0;
  gps.ubxCkBCalc = 0;
}

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return -1;
}

static bool nmeaChecksumOk(const char *s) {
  const char *star = strrchr(s, '*');
  if (star == NULL || star <= s) {
    return true;  // some clones omit checksum
  }

  int hi = hexNibble(star[1]);
  int lo = hexNibble(star[2]);
  if (hi < 0 || lo < 0) {
    return false;
  }

  uint8_t ck = 0;
  for (const char *p = s + 1; p < star; p++) {
    ck ^= (uint8_t)*p;
  }
  return ck == (uint8_t)((hi << 4) | lo);
}

static bool nmeaField(const char *s, int idx, char *out, size_t outlen) {
  const char *p = s;
  if (*p == '$') {
    p++;
  }

  int f = 0;
  while (*p && *p != '*' && *p != '\r' && *p != '\n') {
    const char *start = p;
    while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n') {
      p++;
    }
    if (f == idx) {
      size_t n = (size_t)(p - start);
      if (n >= outlen) {
        n = outlen - 1;
      }
      memcpy(out, start, n);
      out[n] = '\0';
      return true;
    }
    if (*p == ',') {
      p++;
    }
    f++;
  }
  return false;
}

static int32_t nmeaDmToE7(const char *dm, char hemi) {
  if (dm == NULL || dm[0] == '\0') {
    return 0;
  }
  double v = atof(dm);
  int ideg = (int)(v / 100.0);
  double minutes = v - (double)ideg * 100.0;
  double deg = (double)ideg + minutes / 60.0;
  if (hemi == 'S' || hemi == 's' || hemi == 'W' || hemi == 'w') {
    deg = -deg;
  }
  if (deg >= 0.0) {
    return (int32_t)(deg * 10000000.0 + 0.5);
  }
  return (int32_t)(deg * 10000000.0 - 0.5);
}

static void gpsApplyFix(bool valid, bool satsKnown, uint8_t sats,
                        bool altKnown, float altM, const char *src) {
  gps.valid = valid;
  if (satsKnown) {
    gps.satsKnown = true;
    gps.sats = sats;
  }
  if (valid && altKnown) {
    gps.altKnown = true;
    gps.altM = altM;
  } else if (!valid) {
    gps.altKnown = false;
  }
  gps.src = src;
  gps.lastFrameMs = millis();
}

static void gpsApplyPos(bool valid, int32_t latE7, int32_t lonE7) {
  if (valid) {
    gps.posKnown = true;
    gps.latE7 = latE7;
    gps.lonE7 = lonE7;
  } else {
    gps.posKnown = false;
  }
}

static void gpsApplyTime(bool valid, uint16_t year, uint8_t month, uint8_t day,
                         uint8_t hour, uint8_t minute, uint8_t sec) {
  gps.timeKnown = valid;
  if (!valid) {
    return;
  }
  gps.year = year;
  gps.month = month;
  gps.day = day;
  gps.hour = hour;
  gps.minute = minute;
  gps.sec = sec;
}

static bool gpsFrameFresh(unsigned long now) {
  return gps.lastFrameMs != 0 && (now - gps.lastFrameMs) < GPS_STALE_MS;
}

static void gpsHandleGga(const char *s) {
  char qualBuf[8];
  char satBuf[8];
  char altBuf[16];
  char latBuf[16];
  char nsBuf[4];
  char lonBuf[16];
  char ewBuf[4];

  if (!nmeaField(s, 6, qualBuf, sizeof(qualBuf))) {
    return;
  }
  nmeaField(s, 7, satBuf, sizeof(satBuf));
  nmeaField(s, 9, altBuf, sizeof(altBuf));
  nmeaField(s, 2, latBuf, sizeof(latBuf));
  nmeaField(s, 3, nsBuf, sizeof(nsBuf));
  nmeaField(s, 4, lonBuf, sizeof(lonBuf));
  nmeaField(s, 5, ewBuf, sizeof(ewBuf));

  int quality = atoi(qualBuf);
  bool satsKnown = satBuf[0] != '\0';
  uint8_t sats = (uint8_t)atoi(satBuf);
  bool altKnown = altBuf[0] != '\0';
  float altM = (float)atof(altBuf);
  bool valid = quality > 0;
  gpsApplyFix(valid, satsKnown, sats, altKnown, altM, "NMEA");
  if (valid && latBuf[0] && nsBuf[0] && lonBuf[0] && ewBuf[0]) {
    gpsApplyPos(true, nmeaDmToE7(latBuf, nsBuf[0]), nmeaDmToE7(lonBuf, ewBuf[0]));
  } else {
    gpsApplyPos(false, 0, 0);
  }
}

static void gpsHandleNmeaLine(char *s) {
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == '\r' || s[n - 1] == '\n' || s[n - 1] == ' ')) {
    s[--n] = '\0';
  }
  if (n < 6 || s[0] != '$' || !nmeaChecksumOk(s)) {
    return;
  }
  if (memcmp(s + 3, "GGA", 3) != 0) {
    return;
  }
  gps.nmeaOk++;
  gpsHandleGga(s);
}

static void gpsHandleNavPvt(const uint8_t *p, uint16_t len) {
  if (len < 92) {
    return;
  }
  uint16_t year = leU16(p + 4);
  uint8_t month = p[6];
  uint8_t day = p[7];
  uint8_t hour = p[8];
  uint8_t minute = p[9];
  uint8_t sec = p[10];
  uint8_t validFlags = p[11];
  uint8_t fixType = p[20];
  uint8_t numSV = p[23];
  int32_t lonE7 = leI32(p + 24);
  int32_t latE7 = leI32(p + 28);
  int32_t hMslMm = leI32(p + 36);
  (void)p[21];
  bool valid = (fixType >= 2) && (fixType <= 4);
  gps.ubxOk++;
  gps.fixType = fixType;
  gpsApplyFix(valid, true, numSV, valid, (float)hMslMm / 1000.0f, "UBX");
  gpsApplyPos(valid, latE7, lonE7);
  bool timeOk = ((validFlags & 0x03) == 0x03) && year >= 2020 && year < 2100
                && month >= 1 && month <= 12 && day >= 1 && day <= 31;
  gpsApplyTime(timeOk, year, month, day, hour, minute, sec);
  if (valid && timeOk) {
    lastFixCaptureFromGps();
  }
}

static void gpsHandleUbx() {
  uint8_t cls = gps.ubxHdr[0];
  uint8_t id = gps.ubxHdr[1];
  if (cls == 0x01 && id == 0x07) {
    gpsHandleNavPvt(gps.ubxPayload, gps.ubxLen);
  }
}

static void gpsFeedByte(uint8_t b) {
  gps.lastRxMs = millis();
  gps.rxBytes++;
  if (gps.dumpLen < sizeof(gps.dump)) {
    gps.dump[gps.dumpLen++] = b;
  }

  switch (gps.parseState) {
    case GPS_IDLE:
      if (b == '$') {
        gps.nmea[0] = '$';
        gps.nmeaLen = 1;
        gps.parseState = GPS_NMEA;
      } else if (b == 0xB5) {
        gps.parseState = GPS_UBX_SYNC2;
      }
      break;

    case GPS_NMEA:
      if (gps.nmeaLen < sizeof(gps.nmea) - 1) {
        gps.nmea[gps.nmeaLen++] = (char)b;
      }
      if (b == '\n' || gps.nmeaLen >= sizeof(gps.nmea) - 1) {
        gps.nmea[gps.nmeaLen] = '\0';
        gpsHandleNmeaLine(gps.nmea);
        gpsResetParser();
      }
      break;

    case GPS_UBX_SYNC2:
      if (b == 0x62) {
        gps.ubxHdrLen = 0;
        gps.ubxCkACalc = 0;
        gps.ubxCkBCalc = 0;
        gps.parseState = GPS_UBX_HDR;
      } else if (b == 0xB5) {
        gps.parseState = GPS_UBX_SYNC2;
      } else if (b == '$') {
        gps.nmea[0] = '$';
        gps.nmeaLen = 1;
        gps.parseState = GPS_NMEA;
      } else {
        gps.parseState = GPS_IDLE;
      }
      break;

    case GPS_UBX_HDR:
      gps.ubxHdr[gps.ubxHdrLen++] = b;
      ubxChecksumAdd(b, &gps.ubxCkACalc, &gps.ubxCkBCalc);
      if (gps.ubxHdrLen >= 4) {
        gps.ubxLen = (uint16_t)gps.ubxHdr[2] | ((uint16_t)gps.ubxHdr[3] << 8);
        gps.ubxGot = 0;
        if (gps.ubxLen == 0) {
          gps.parseState = GPS_UBX_CKA;
        } else if (gps.ubxLen > 512) {
          gpsResetParser();
        } else if (gps.ubxLen > sizeof(gps.ubxPayload)) {
          gps.ubxSkipLeft = gps.ubxLen;
          gps.parseState = GPS_UBX_SKIP;
        } else {
          gps.parseState = GPS_UBX_PAYLOAD;
        }
      }
      break;

    case GPS_UBX_PAYLOAD:
      gps.ubxPayload[gps.ubxGot++] = b;
      ubxChecksumAdd(b, &gps.ubxCkACalc, &gps.ubxCkBCalc);
      if (gps.ubxGot >= gps.ubxLen) {
        gps.parseState = GPS_UBX_CKA;
      }
      break;

    case GPS_UBX_SKIP:
      ubxChecksumAdd(b, &gps.ubxCkACalc, &gps.ubxCkBCalc);
      gps.ubxSkipLeft--;
      if (gps.ubxSkipLeft == 0) {
        gps.parseState = GPS_UBX_CKA;
      }
      break;

    case GPS_UBX_CKA:
      gps.ubxCkA = b;
      gps.parseState = GPS_UBX_CKB;
      break;

    case GPS_UBX_CKB:
      gps.ubxCkB = b;
      if (gps.ubxCkA == gps.ubxCkACalc && gps.ubxCkB == gps.ubxCkBCalc &&
          gps.ubxLen <= sizeof(gps.ubxPayload)) {
        gpsHandleUbx();
      } else {
        gps.ckFail++;
      }
      gpsResetParser();
      break;

    default:
      gpsResetParser();
      break;
  }
}

static void gpsDrain() {
  while (Serial1.available() > 0) {
    gpsFeedByte((uint8_t)Serial1.read());
  }
}

// D4 is button and battery divider at once. Pressed pulls it to ~0 V; the
// released level is only a valid battery sample once the 100 nF has settled.
static void btnAdcSample(unsigned long now) {
  if ((now - btnAdcLastMs) < BTN_SAMPLE_MS) {
    return;
  }
  btnAdcLastMs = now;

  int raw = analogRead(PIN_BTN);
  bool pressed = raw < BTN_ADC_PRESSED;
  if (pressed != btnStable) {
    if (++btnAdcCount >= BTN_STABLE_SAMPLES) {
      btnStable = pressed;
      btnAdcCount = 0;
      if (!pressed) {
        btnReleaseMs = now;
      }
    }
  } else {
    btnAdcCount = 0;
  }

  if (!btnStable && !pressed && (now - btnReleaseMs) >= BATT_SETTLE_MS) {
    float v = raw * 3.6f / 4096.0f * 2.0f;
    if (battVolts < 0.0f) {
      battVolts = v;
    } else {
      battVolts += BATT_EMA_ALPHA * (v - battVolts);
    }
  }
}

static void gpsWait(unsigned long ms) {
  unsigned long start = millis();
  do {
    gpsDrain();
    btnAdcSample(millis());
    yield();
  } while (millis() - start < ms);
}

static bool gpsIsLive(unsigned long now) {
  return gps.valid && gpsFrameFresh(now);
}

static uint32_t crc32Update(uint32_t c, const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    c ^= data[i];
    for (int b = 0; b < 8; b++) {
      uint32_t mask = (uint32_t)-(int32_t)(c & 1UL);
      c = (c >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return c;
}

static uint32_t crc32(const uint8_t *data, size_t len) {
  return ~crc32Update(0xFFFFFFFFUL, data, len);
}

static void formatE7(int32_t e7, char *buf, size_t buflen) {
  char sign = 0;
  uint32_t v;
  if (e7 < 0) {
    sign = '-';
    v = (uint32_t)(-e7);
  } else {
    v = (uint32_t)e7;
  }
  uint32_t ip = v / 10000000UL;
  uint32_t fp = v % 10000000UL;
  if (sign) {
    snprintf(buf, buflen, "-%lu.%07lu", (unsigned long)ip, (unsigned long)fp);
  } else {
    snprintf(buf, buflen, "%lu.%07lu", (unsigned long)ip, (unsigned long)fp);
  }
}

static bool tripDatSave(uint32_t revCount, uint32_t elapsedMsNow) {
  if (!sdReady) {
    return false;
  }

  TripDat d;
  memset(&d, 0, sizeof(d));
  d.magic = TRIP_MAGIC;
  d.version = TRIP_VERSION;
  d.revCount = revCount;
  d.movingMs = (uint32_t)movingMs;
  d.elapsedMs = elapsedMsNow;
  d.maxSpeedKmh = maxSpeedKmh;
  d.gpxBodyEnd = gpxBodyEnd;
  d.crc = crc32((const uint8_t *)&d, offsetof(TripDat, crc));

  gpsDrain();
  File32 f;
  if (!f.open(DAT_NAME, O_RDWR | O_CREAT | O_TRUNC)) {
    gpsDrain();
    return false;
  }
  bool ok = f.write(&d, sizeof(d)) == sizeof(d);
  f.flush();
  f.close();
  gpsDrain();
  lastDatMs = millis();
  lastFixSave();
  return ok;
}

static bool tripDatLoad(TripDat *out) {
  File32 f;
  if (!f.open(DAT_NAME, O_RDONLY)) {
    return false;
  }
  TripDat d;
  int n = f.read(&d, sizeof(d));
  f.close();
  if (n != (int)sizeof(d)) {
    return false;
  }
  if (d.magic != TRIP_MAGIC || d.version != TRIP_VERSION) {
    return false;
  }
  if (crc32((const uint8_t *)&d, offsetof(TripDat, crc)) != d.crc) {
    return false;
  }
  *out = d;
  return true;
}

static void lastFixCaptureFromGps() {
  if (!gps.posKnown && !gps.timeKnown) {
    return;
  }
  uint16_t prevFlags = lastFix.flags;
  int32_t prevLat = lastFix.latE7;
  int32_t prevLon = lastFix.lonE7;
  if (gps.posKnown) {
    lastFix.latE7 = gps.latE7;
    lastFix.lonE7 = gps.lonE7;
    lastFix.flags = (uint16_t)((lastFix.flags & ~FIX_FLAG_POS) | FIX_FLAG_POS);
    if (gps.altKnown) {
      lastFix.altCm = (int32_t)(gps.altM * 100.0f);
      lastFix.flags = (uint16_t)(lastFix.flags | FIX_FLAG_ALT);
    }
  }
  if (gps.timeKnown) {
    lastFix.year = gps.year;
    lastFix.month = gps.month;
    lastFix.day = gps.day;
    lastFix.hour = gps.hour;
    lastFix.minute = gps.minute;
    lastFix.sec = gps.sec;
    lastFix.flags = (uint16_t)(lastFix.flags | FIX_FLAG_TIME);
  }
  lastFix.magic = FIX_MAGIC;
  lastFix.version = FIX_VERSION;
  lastFixValid = (lastFix.flags & (FIX_FLAG_POS | FIX_FLAG_TIME)) != 0;
  if (lastFixValid &&
      (prevFlags != lastFix.flags || prevLat != lastFix.latE7 || prevLon != lastFix.lonE7)) {
    lastFixDirty = true;
  }
}

static bool lastFixSave() {
  if (!sdReady || !lastFixValid) {
    return false;
  }

  LastFixDat d = lastFix;
  d.magic = FIX_MAGIC;
  d.version = FIX_VERSION;
  d.crc = crc32((const uint8_t *)&d, offsetof(LastFixDat, crc));

  gpsDrain();
  File32 f;
  if (!f.open(FIX_NAME, O_RDWR | O_CREAT | O_TRUNC)) {
    gpsDrain();
    return false;
  }
  bool ok = f.write(&d, sizeof(d)) == sizeof(d);
  f.flush();
  f.close();
  gpsDrain();
  if (ok) {
    lastFixDirty = false;
    lastFixSaveMs = millis();
  }
  return ok;
}

static bool lastFixLoad() {
  File32 f;
  if (!f.open(FIX_NAME, O_RDONLY)) {
    return false;
  }
  LastFixDat d;
  int n = f.read(&d, sizeof(d));
  f.close();
  if (n != (int)sizeof(d)) {
    return false;
  }
  if (d.magic != FIX_MAGIC || d.version != FIX_VERSION) {
    return false;
  }
  if (crc32((const uint8_t *)&d, offsetof(LastFixDat, crc)) != d.crc) {
    return false;
  }
  if ((d.flags & (FIX_FLAG_POS | FIX_FLAG_TIME)) == 0) {
    return false;
  }
  lastFix = d;
  lastFixValid = true;
  Serial.println("GPS LASTFIX.DAT loaded");
  return true;
}

static uint32_t gpxTimeStamp() {
  uint32_t days = ((uint32_t)(gps.year - 2020) * 12u + (gps.month - 1)) * 31u
                  + gps.day;
  uint32_t tod = (uint32_t)gps.hour * 3600u
                 + (uint32_t)gps.minute * 60u
                 + (uint32_t)gps.sec;
  return days * 86400u + tod;
}

static bool gpxWriteHeader() {
  if (!gpxFile.seekSet(0)) {
    return false;
  }
  size_t n = strlen(GPX_HEADER);
  if (gpxFile.write(GPX_HEADER, n) != n) {
    return false;
  }
  gpxBodyEnd = (uint32_t)gpxFile.curPosition();
  size_t fsz = strlen(GPX_FOOTER);
  if (gpxFile.write(GPX_FOOTER, fsz) != fsz) {
    return false;
  }
  gpxFile.flush();
  return true;
}

static bool gpxOpenOrCreate() {
  if (gpxFile.isOpen()) {
    return true;
  }
  if (!gpxFile.open(GPX_NAME, O_RDWR | O_CREAT)) {
    Serial.println("SD CURRENT.GPX open failed");
    return false;
  }
  uint32_t sz = (uint32_t)gpxFile.fileSize();
  if (sz == 0) {
    if (!gpxWriteHeader()) {
      Serial.println("SD GPX header write failed");
      return false;
    }
    Serial.println("SD new CURRENT.GPX");
    return true;
  }
  if (gpxBodyEnd == 0 || gpxBodyEnd > sz) {
    size_t foot = strlen(GPX_FOOTER);
    if (sz > foot) {
      gpxBodyEnd = sz - (uint32_t)foot;
    } else {
      if (!gpxWriteHeader()) {
        return false;
      }
    }
  }
  Serial.print("SD CURRENT.GPX bodyEnd=");
  Serial.println(gpxBodyEnd);
  return true;
}

static bool gpxFinalize() {
  if (!sdReady) {
    return false;
  }
  if (!gpxFile.isOpen() && !gpxOpenOrCreate()) {
    return false;
  }

  gpsDrain();
  if (!gpxFile.seekSet(gpxBodyEnd)) {
    gpsDrain();
    return false;
  }
  size_t fsz = strlen(GPX_FOOTER);
  if (gpxFile.write(GPX_FOOTER, fsz) != fsz) {
    gpsDrain();
    return false;
  }
  uint32_t finalSize = gpxBodyEnd + (uint32_t)fsz;
  if (!gpxFile.truncate(finalSize)) {
    gpsDrain();
    return false;
  }
  gpxFile.flush();
  gpsDrain();
  Serial.print("SD GPX finalized size=");
  Serial.println(finalSize);
  return true;
}

static bool gpxAppendPoint() {
  if (!sdReady || !gpxFile.isOpen()) {
    return false;
  }

  char latBuf[16];
  char lonBuf[16];
  formatE7(gps.latE7, latBuf, sizeof(latBuf));
  formatE7(gps.lonE7, lonBuf, sizeof(lonBuf));

  char line[192];
  int n;
  if (gps.altKnown) {
    n = snprintf(line, sizeof(line),
                 "<trkpt lat=\"%s\" lon=\"%s\"><ele>%.1f</ele>"
                 "<time>%04u-%02u-%02uT%02u:%02u:%02uZ</time></trkpt>\n",
                 latBuf, lonBuf, (double)gps.altM,
                 (unsigned)gps.year, (unsigned)gps.month, (unsigned)gps.day,
                 (unsigned)gps.hour, (unsigned)gps.minute, (unsigned)gps.sec);
  } else {
    n = snprintf(line, sizeof(line),
                 "<trkpt lat=\"%s\" lon=\"%s\">"
                 "<time>%04u-%02u-%02uT%02u:%02u:%02uZ</time></trkpt>\n",
                 latBuf, lonBuf,
                 (unsigned)gps.year, (unsigned)gps.month, (unsigned)gps.day,
                 (unsigned)gps.hour, (unsigned)gps.minute, (unsigned)gps.sec);
  }
  if (n <= 0 || n >= (int)sizeof(line)) {
    return false;
  }

  gpsDrain();
  if (!gpxFile.seekSet(gpxBodyEnd)) {
    gpsDrain();
    return false;
  }
  if (gpxFile.write(line, (size_t)n) != (size_t)n) {
    gpsDrain();
    return false;
  }
  gpxBodyEnd = (uint32_t)gpxFile.curPosition();
  size_t fsz = strlen(GPX_FOOTER);
  bool ok = gpxFile.write(GPX_FOOTER, fsz) == fsz;
  gpxFile.flush();
  gpsDrain();
  if (ok) {
    gpxPointCount++;
    lastGpxLatE7 = gps.latE7;
    lastGpxLonE7 = gps.lonE7;
    lastGpxPosValid = true;
    lastGpxStamp = gpxTimeStamp();
    lastGpxStampValid = true;
    lastFixCaptureFromGps();
  }
  return ok;
}

static void tripLogGps(unsigned long now, uint32_t revCount, uint32_t elapsedMsNow) {
  if (!sdReady || !tripStarted || !gpsIsLive(now) || !gps.posKnown || !gps.timeKnown) {
    return;
  }
  if (now - lastGpxMs < GPX_MIN_MS) {
    return;
  }
  if (lastGpxPosValid) {
    int32_t dlat = gps.latE7 - lastGpxLatE7;
    int32_t dlon = gps.lonE7 - lastGpxLonE7;
    if (dlat < 0) {
      dlat = -dlat;
    }
    if (dlon < 0) {
      dlon = -dlon;
    }
    if (dlat < GPX_MIN_E7 && dlon < GPX_MIN_E7) {
      return;
    }
  }
  if (lastGpxStampValid && gpxTimeStamp() <= lastGpxStamp) {
    return;
  }
  lastGpxMs = now;
  if (gpxAppendPoint()) {
    tripDatSave(revCount, elapsedMsNow);
  }
}

static void tripMaybeSave(unsigned long now, uint32_t revCount, uint32_t elapsedMsNow) {
  if (!sdReady || !tripStarted) {
    return;
  }
  if (now - lastDatMs < TRIP_DAT_MS) {
    return;
  }
  tripDatSave(revCount, elapsedMsNow);
}

static const char *tripResumeSd() {
  TripDat d;
  bool haveDat = tripDatLoad(&d);
  lastFixLoad();
  if (haveDat) {
    gpxBodyEnd = d.gpxBodyEnd;
  }

  if (!gpxOpenOrCreate()) {
    sdReady = false;
    gpxFile.close();
    return "No card";
  }

  if (!haveDat) {
    tripDatSave(0, 0);
    Serial.println("SD new TRIP.DAT");
    return "Card ready";
  }

  noInterrupts();
  g_revCount = d.revCount;
  g_lastPulseMs = 0;
  g_prevPulseMs = 0;
  g_olderPulseMs = 0;
  interrupts();
  movingMs = d.movingMs;
  // Drop a previously saved noise spike so resume does not keep 200+ km/h Max.
  maxSpeedKmh = (d.maxSpeedKmh > 0.0f && d.maxSpeedKmh <= cfg.maxSpeedKmh)
                    ? d.maxSpeedKmh
                    : 0.0f;
  animMaxBaseline = maxSpeedKmh;
  if (d.revCount >= 1 || d.elapsedMs > 0 || d.movingMs > 0) {
    tripStarted = true;
    elapsedBaseMs = d.elapsedMs;
    elapsedAnchorMs = millis();
  }

  Serial.print("SD resume revs=");
  Serial.print(d.revCount);
  Serial.print(" moveMs=");
  Serial.print(d.movingMs);
  Serial.print(" elapsedMs=");
  Serial.print(d.elapsedMs);
  Serial.print(" max=");
  Serial.println(d.maxSpeedKmh, 1);
  return tripStarted ? "Resuming ride" : "Card ready";
}

static void tripResetRam() {
  noInterrupts();
  g_revCount = 0;
  g_lastPulseMs = 0;
  g_prevPulseMs = 0;
  g_olderPulseMs = 0;
  g_lastIsrMs = 0;
  interrupts();
  tripStarted = false;
  elapsedBaseMs = 0;
  elapsedAnchorMs = 0;
  movingMs = 0;
  maxSpeedKmh = 0.0f;
  lastGpxMs = 0;
  lastDatMs = 0;
  gpxPointCount = 0;
  lastGpxLatE7 = 0;
  lastGpxLonE7 = 0;
  lastGpxPosValid = false;
  lastGpxStamp = 0;
  lastGpxStampValid = false;
  animResetRide();
}

static int cfgMonthDays(int year, int month) {
  static const uint8_t kDim[] = {
      0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
  };
  bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  return (month == 2 && leap) ? 29 : kDim[month];
}

static bool tripLocalCivilFromGps(unsigned *yy, unsigned *mo, unsigned *dd,
                                  unsigned *hh) {
  if (!gps.timeKnown) {
    return false;
  }
  // Civil date from GPS UTC + timezone_offset_min (archive names only).
  int y = (int)gps.year;
  int m = (int)gps.month;
  int d = (int)gps.day;
  long minutes = (long)gps.hour * 60L + (long)gps.minute
                 + (long)cfg.timezoneOffsetMin;

  while (minutes < 0) {
    minutes += 24L * 60L;
    d--;
    if (d < 1) {
      m--;
      if (m < 1) {
        m = 12;
        y--;
      }
      d = cfgMonthDays(y, m);
    }
  }
  while (minutes >= 24L * 60L) {
    minutes -= 24L * 60L;
    d++;
    if (d > cfgMonthDays(y, m)) {
      d = 1;
      m++;
      if (m > 12) {
        m = 1;
        y++;
      }
    }
  }
  *yy = (unsigned)(y % 100);
  *mo = (unsigned)m;
  *dd = (unsigned)d;
  *hh = (unsigned)(minutes / 60L);
  return true;
}

static bool tripPickArchiveName(char *buf, size_t buflen) {
  if (buflen < 13) {
    return false;
  }

  unsigned yy, mo, dd, hh;
  if (tripLocalCivilFromGps(&yy, &mo, &dd, &hh)) {
    snprintf(buf, buflen, "%02u%02u%02u%02u.GPX", yy, mo, dd, hh);
    if (!sd.exists(buf)) {
      return true;
    }
    for (unsigned seq = 0; seq < 100; seq++) {
      snprintf(buf, buflen, "%02u%02u%02u%02u.GPX", yy, mo, dd, seq);
      if (!sd.exists(buf)) {
        return true;
      }
    }
  }

  for (unsigned n = 1; n <= 9999; n++) {
    snprintf(buf, buflen, "RIDE%04u.GPX", n);
    if (!sd.exists(buf)) {
      return true;
    }
    if ((n & 0x1F) == 0) {
      gpsDrain();
    }
  }
  return false;
}

// NEW_RIDE_SAVED / NEW_RIDE_RESET (no card) / NEW_RIDE_FAIL (keep RAM).
enum {
  NEW_RIDE_FAIL = 0,
  NEW_RIDE_RESET,
  NEW_RIDE_SAVED
};

static void bleStopForNewRide();

static int tripStartNewRide() {
  bleStopForNewRide();
  if (!sdReady) {
    tripResetRam();
    gpxBodyEnd = 0;
    Serial.println("New ride — no SD, stats reset");
    return NEW_RIDE_RESET;
  }

  gpsDrain();
  if (!gpxFinalize()) {
    Serial.println("SD GPX finalize failed");
    return NEW_RIDE_FAIL;
  }

  bool hasPoints = gpxBodyEnd > (uint32_t)strlen(GPX_HEADER);
  gpxFile.close();
  gpsDrain();

  if (hasPoints) {
    char dest[13];
    if (!tripPickArchiveName(dest, sizeof(dest))) {
      Serial.println("SD archive name failed");
      if (!gpxOpenOrCreate()) {
        sdReady = false;
      }
      return NEW_RIDE_FAIL;
    }

    gpsDrain();
    if (!sd.rename(GPX_NAME, dest)) {
      gpsDrain();
      Serial.print("SD rename failed -> ");
      Serial.println(dest);
      if (!gpxOpenOrCreate()) {
        sdReady = false;
      }
      return NEW_RIDE_FAIL;
    }
    gpsDrain();
    Serial.print("SD archived ");
    Serial.println(dest);
  } else {
    Serial.println("SD empty GPX — skip archive");
  }

  gpxBodyEnd = 0;
  if (!hasPoints) {
    gpsDrain();
    if (!gpxFile.open(GPX_NAME, O_RDWR | O_CREAT | O_TRUNC)) {
      gpsDrain();
      sdReady = false;
      Serial.println("SD CURRENT.GPX reset failed");
      return NEW_RIDE_FAIL;
    }
    if (!gpxWriteHeader()) {
      gpsDrain();
      Serial.println("SD CURRENT.GPX header reset failed");
      return NEW_RIDE_FAIL;
    }
    gpsDrain();
  } else if (!gpxOpenOrCreate()) {
    sdReady = false;
    Serial.println("SD new CURRENT.GPX failed after archive");
    return NEW_RIDE_FAIL;
  }

  tripResetRam();
  if (!tripDatSave(0, 0)) {
    Serial.println("SD TRIP.DAT reset failed — GPX archived, stats cleared");
  }
  return NEW_RIDE_SAVED;
}

static bool bikeStopped(unsigned long now, unsigned long lastPulseMs) {
  return lastPulseMs == 0 || (now - lastPulseMs) >= cfg.stoppedMs;
}

static void backlightApply() {
  analogWrite(PIN_LCD_BL, BL_DUTY[blMode]);
}

static void backlightNext() {
  blMode = (uint8_t)((blMode + 1) % BL_MODE_COUNT);
  backlightApply();
}

static void btnPoll(unsigned long now) {
  btnAdcSample(now);
  bool raw = btnStable;

  if (raw && !btnHeld) {
    btnHeld = true;
    btnDidExec = false;
    btnHoldStartMs = now;
  } else if (!raw && btnHeld) {
    btnReleased = true;
    btnReleaseDidExec = btnDidExec;
    btnReleasedHeldMs = now - btnHoldStartMs;
    btnHeld = false;
    btnArmed = false;
    btnDidExec = false;
    btnHoldStartMs = 0;
  }
}

static void btnFlash(const char *msg, unsigned long now) {
  overlayFlashMsg = msg;
  overlayFlashUntilMs = now + BTN_FLASH_MS;
}

static void btnHandle(unsigned long now, unsigned long lastPulseMs) {
  btnPoll(now);

  bool stopped = bikeStopped(now, lastPulseMs);

  if (btnHeld && !btnDidExec) {
    if (!btnArmed) {
      if (stopped && (now - btnHoldStartMs) < BTN_ARM_MS) {
        btnArmed = true;
      }
    } else if (!stopped) {
      btnArmed = false;
    }
  }

  if (btnArmed && btnHeld && !btnDidExec &&
      (now - btnHoldStartMs) >= BTN_EXEC_MS) {
    btnDidExec = true;
    btnArmed = false;
    int result = tripStartNewRide();
    if (result == NEW_RIDE_SAVED) {
      btnFlash("Ride saved", now);
      clipStart(clipFirework, now);
    } else if (result == NEW_RIDE_RESET) {
      btnFlash("Stats reset", now);
      clipStart(clipFirework, now);
    } else {
      btnFlash("Save failed", now);
    }
  }

  // Release before the new-ride countdown starts. A cancelled hold, or a
  // hold that already saved the ride, leaves the backlight alone.
  // While a phone is waiting for confirm, a short press allows the link
  // instead of cycling the backlight.
  if (btnReleased) {
    bool didExec = btnReleaseDidExec;
    unsigned long held = btnReleasedHeldMs;
    btnReleased = false;
    if (!didExec && held < BTN_ARM_MS) {
      if (bleAwaitingAuth()) {
        bleConfirmAuth(now);
      } else {
        backlightNext();
      }
    }
  }
}

static void sdBeginShared() {
  pinMode(PIN_LCD_CS, OUTPUT);
  digitalWrite(PIN_LCD_CS, HIGH);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  gpsDrain();
  SdSpiConfig cfgSpi(PIN_SD_CS, SHARED_SPI, SD_SCK_MHZ(4), &SPI);
  sdReady = sd.begin(cfgSpi);
  gpsDrain();
  if (!sdReady) {
    Serial.println("SD begin failed — riding without log");
  }
}

// The hub's Settings rewrite the whole file, so hand-added comments are lost.
static const char CFG_TEXT_FORMAT[] =
    "# OBJECT CONFIG.TXT — change these from Settings in the OBJECT hub,\n"
    "# or edit on any computer and reinsert the card. Saving from the hub\n"
    "# rewrites this file. Missing file is created with defaults on first boot.\n"
    "#\n"
    "# wheel_circ_mm: measured rollout in millimetres (700x32C ~2155)\n"
    "wheel_circ_mm=%u\n"
    "#\n"
    "# timezone_offset_min: minutes from UTC for archive filenames only\n"
    "# (GPX timestamps stay UTC). Example: 120 = UTC+2, -300 = UTC-5\n"
    "timezone_offset_min=%d\n"
    "#\n"
    "# backlight at boot: bright | dim | off\n"
    "backlight=%s\n"
    "#\n"
    "# BLE advertise name (1-20 chars, no spaces)\n"
    "ble_name=%s\n"
    "#\n"
    "# units: metric | imperial  (display only; storage stays metric)\n"
    "units=%s\n"
    "#\n"
    "# dim PWM duty 1-254 (bright is always 255)\n"
    "backlight_dim=%u\n"
    "#\n"
    "# max_speed_kmh: treat faster reed intervals as noise\n"
    "max_speed_kmh=%u\n"
    "#\n"
    "# stopped_ms: no pulse for this long => bike stopped (moving time)\n"
    "stopped_ms=%lu\n"
    "#\n"
    "# animations: on | off  (face when stopped, celebrations, transitions)\n"
    "animations=%s\n";

static const char CFG_TMP_NAME[] = "CONFIG.NEW";
static char cfgText[1280];

static bool cfgWheelOk(float v) {
  return v >= 1000.0f && v <= 3000.0f;
}

static bool cfgTimezoneOk(long v) {
  return v >= -720L && v <= 840L;
}

static bool cfgDimOk(long v) {
  return v >= 1L && v <= 254L;
}

static bool cfgMaxSpeedOk(float v) {
  return v >= 20.0f && v <= 200.0f;
}

static bool cfgStoppedOk(long v) {
  return v >= 1000L && v <= 10000L;
}

static bool cfgBleNameOk(const char *v, size_t n) {
  if (n < 1 || n > CFG_BLE_NAME_MAX) {
    return false;
  }
  for (size_t i = 0; i < n; i++) {
    char c = v[i];
    if (c <= 0x20 || c >= 0x7F) {
      return false;
    }
  }
  return true;
}

static const char *cfgBacklightName(uint8_t mode) {
  return mode == BL_DIM ? "dim" : (mode == BL_OFF ? "off" : "bright");
}

static size_t configFormat(const Cfg &c) {
  int n = snprintf(cfgText, sizeof(cfgText), CFG_TEXT_FORMAT,
                   (unsigned)(c.wheelCircMm + 0.5f),
                   (int)c.timezoneOffsetMin,
                   cfgBacklightName(c.backlight),
                   c.bleName,
                   c.units == UNITS_IMPERIAL ? "imperial" : "metric",
                   (unsigned)c.backlightDim,
                   (unsigned)(c.maxSpeedKmh + 0.5f),
                   (unsigned long)c.stoppedMs,
                   c.animations ? "on" : "off");
  if (n <= 0 || (size_t)n >= sizeof(cfgText)) {
    return 0;
  }
  return (size_t)n;
}

static bool configWriteFile(const char *path, const Cfg &c) {
  size_t n = configFormat(c);
  if (n == 0) {
    return false;
  }
  gpsDrain();
  File32 f;
  if (!f.open(path, O_WRONLY | O_CREAT | O_TRUNC)) {
    gpsDrain();
    return false;
  }
  bool ok = f.write(cfgText, n) == n;
  f.flush();
  f.close();
  if (!ok) {
    sd.remove(path);
  }
  gpsDrain();
  return ok;
}

// Writes CONFIG.NEW first, so a power cut leaves either the old file or a
// complete new one that configEnsureFile renames at the next boot.
static bool configSave(const Cfg &c) {
  if (!sdReady) {
    return false;
  }
  if (!configWriteFile(CFG_TMP_NAME, c)) {
    Serial.println("CFG save write failed");
    return false;
  }
  gpsDrain();
  if (sd.exists(CFG_NAME)) {
    sd.remove(CFG_NAME);
  }
  bool ok = sd.rename(CFG_TMP_NAME, CFG_NAME);
  gpsDrain();
  if (!ok) {
    Serial.println("CFG save rename failed");
  }
  return ok;
}

static void configApplyDerived(bool setBacklightMode) {
  if (cfg.maxSpeedKmh < 1.0f) {
    cfg.maxSpeedKmh = 1.0f;
  }
  g_minRevMs = (unsigned long)((cfg.wheelCircMm * 3.6f) / cfg.maxSpeedKmh + 0.5f);
  BL_DUTY[BL_DIM] = cfg.backlightDim;
  if (setBacklightMode) {
    blMode = cfg.backlight;
    if (blMode >= BL_MODE_COUNT) {
      blMode = BL_BRIGHT;
    }
  }
  backlightApply();
}

// Settings saved from the phone take effect without a reboot. The boot
// backlight level only changes the panel when that setting itself changed.
static void configApplyChange(const Cfg &old) {
  if (cfg.wheelCircMm != old.wheelCircMm) {
    // Distance is revolutions x circumference: rescale the count so the
    // distance already ridden stays put and only new revolutions use the
    // new wheel size.
    noInterrupts();
    g_revCount = (unsigned long)((double)g_revCount * old.wheelCircMm
                                 / cfg.wheelCircMm + 0.5);
    interrupts();
  }
  if (cfg.units != old.units) {
    uiChromeDrawn = false;
    msBaselineValid = false;
  }
  configApplyDerived(cfg.backlight != old.backlight);
}

static void configLog() {
  Serial.print("CFG wheel_circ_mm=");
  Serial.println(cfg.wheelCircMm, 0);
  Serial.print("CFG timezone_offset_min=");
  Serial.println((int)cfg.timezoneOffsetMin);
  Serial.print("CFG backlight=");
  Serial.println(cfgBacklightName(cfg.backlight));
  Serial.print("CFG ble_name=");
  Serial.println(cfg.bleName);
  Serial.print("CFG units=");
  Serial.println(cfg.units == UNITS_IMPERIAL ? "imperial" : "metric");
  Serial.print("CFG backlight_dim=");
  Serial.println((unsigned)cfg.backlightDim);
  Serial.print("CFG max_speed_kmh=");
  Serial.println(cfg.maxSpeedKmh, 0);
  Serial.print("CFG stopped_ms=");
  Serial.println(cfg.stoppedMs);
  Serial.print("CFG animations=");
  Serial.println(cfg.animations ? "on" : "off");
}

static char *cfgTrim(char *s) {
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
    s++;
  }
  char *end = s + strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t'
                     || end[-1] == '\r' || end[-1] == '\n')) {
    end--;
  }
  *end = '\0';
  return s;
}

static bool cfgEq(const char *a, const char *b) {
  return strcmp(a, b) == 0;
}

static bool cfgParseBacklight(const char *v, uint8_t *out) {
  if (cfgEq(v, "bright")) {
    *out = BL_BRIGHT;
    return true;
  }
  if (cfgEq(v, "dim")) {
    *out = BL_DIM;
    return true;
  }
  if (cfgEq(v, "off")) {
    *out = BL_OFF;
    return true;
  }
  return false;
}

static bool cfgParseBleName(const char *v, char *out, size_t outlen) {
  size_t n = strlen(v);
  if (n >= outlen || !cfgBleNameOk(v, n)) {
    return false;
  }
  memcpy(out, v, n);
  out[n] = '\0';
  return true;
}

static void configApplyKey(const char *key, const char *val) {
  if (cfgEq(key, "wheel_circ_mm")) {
    float v = (float)atof(val);
    if (cfgWheelOk(v)) {
      cfg.wheelCircMm = v;
    } else {
      Serial.println("CFG reject wheel_circ_mm");
    }
    return;
  }
  if (cfgEq(key, "timezone_offset_min")) {
    long v = atol(val);
    if (cfgTimezoneOk(v)) {
      cfg.timezoneOffsetMin = (int16_t)v;
    } else {
      Serial.println("CFG reject timezone_offset_min");
    }
    return;
  }
  if (cfgEq(key, "backlight")) {
    uint8_t mode;
    if (cfgParseBacklight(val, &mode)) {
      cfg.backlight = mode;
    } else {
      Serial.println("CFG reject backlight");
    }
    return;
  }
  if (cfgEq(key, "ble_name")) {
    if (!cfgParseBleName(val, cfg.bleName, sizeof(cfg.bleName))) {
      Serial.println("CFG reject ble_name");
    }
    return;
  }
  if (cfgEq(key, "units")) {
    if (cfgEq(val, "metric")) {
      cfg.units = UNITS_METRIC;
    } else if (cfgEq(val, "imperial")) {
      cfg.units = UNITS_IMPERIAL;
    } else {
      Serial.println("CFG reject units");
    }
    return;
  }
  if (cfgEq(key, "backlight_dim")) {
    long v = atol(val);
    if (cfgDimOk(v)) {
      cfg.backlightDim = (uint8_t)v;
    } else {
      Serial.println("CFG reject backlight_dim");
    }
    return;
  }
  if (cfgEq(key, "max_speed_kmh")) {
    float v = (float)atof(val);
    if (cfgMaxSpeedOk(v)) {
      cfg.maxSpeedKmh = v;
    } else {
      Serial.println("CFG reject max_speed_kmh");
    }
    return;
  }
  if (cfgEq(key, "stopped_ms")) {
    long v = atol(val);
    if (cfgStoppedOk(v)) {
      cfg.stoppedMs = (unsigned long)v;
    } else {
      Serial.println("CFG reject stopped_ms");
    }
    return;
  }
  if (cfgEq(key, "animations")) {
    if (cfgEq(val, "on")) {
      cfg.animations = 1;
    } else if (cfgEq(val, "off")) {
      cfg.animations = 0;
    } else {
      Serial.println("CFG reject animations");
    }
    return;
  }
}

static void configEnsureFile() {
  if (!sdReady) {
    return;
  }
  gpsDrain();
  bool haveCfg = sd.exists(CFG_NAME);
  bool haveTmp = sd.exists(CFG_TMP_NAME);
  gpsDrain();
  if (haveCfg) {
    if (haveTmp) {
      sd.remove(CFG_TMP_NAME);
      gpsDrain();
    }
    return;
  }
  if (haveTmp) {
    bool ok = sd.rename(CFG_TMP_NAME, CFG_NAME);
    gpsDrain();
    if (ok) {
      Serial.println("CONFIG.TXT restored from CONFIG.NEW");
      return;
    }
  }
  // cfg still holds the compiled-in defaults here (configLoad runs next).
  if (configWriteFile(CFG_NAME, cfg)) {
    Serial.println("CONFIG.TXT created");
  } else {
    Serial.println("CFG write failed");
  }
}

static void configLoad() {
  if (!sdReady) {
    configApplyDerived(true);
    Serial.println("CFG defaults (no card)");
    configLog();
    return;
  }

  File32 f;
  if (!f.open(CFG_NAME, O_RDONLY)) {
    Serial.println("CFG open failed — defaults");
    configApplyDerived(true);
    configLog();
    return;
  }

  char line[96];
  size_t len = 0;
  bool overflow = false;
  while (f.available()) {
    int c = f.read();
    if (c < 0) {
      break;
    }
    if (c == '\n' || c == '\r') {
      if (len == 0 || overflow) {
        len = 0;
        overflow = false;
        continue;
      }
      line[len] = '\0';
      len = 0;
      char *s = cfgTrim(line);
      if (s[0] == '\0' || s[0] == '#') {
        continue;
      }
      char *eq = strchr(s, '=');
      if (eq == NULL) {
        continue;
      }
      *eq = '\0';
      char *key = cfgTrim(s);
      char *val = cfgTrim(eq + 1);
      if (key[0] == '\0') {
        continue;
      }
      configApplyKey(key, val);
    } else if (!overflow && len + 1 < sizeof(line)) {
      line[len++] = (char)c;
    } else {
      overflow = true;
      len = 0;
    }
  }
  if (!overflow && len > 0) {
    line[len] = '\0';
    char *s = cfgTrim(line);
    if (s[0] != '\0' && s[0] != '#') {
      char *eq = strchr(s, '=');
      if (eq != NULL) {
        *eq = '\0';
        char *key = cfgTrim(s);
        char *val = cfgTrim(eq + 1);
        if (key[0] != '\0') {
          configApplyKey(key, val);
        }
      }
    }
  }
  f.close();

  configApplyDerived(true);
  Serial.println("CFG loaded");
  configLog();
}

static void gpsPoll() {
  unsigned long now = millis();
  gpsDrain();

  if (!gpsFrameFresh(now) && (now - gps.lastCfgMs >= GPS_CFG_MS)) {
    gpsConfigure();
  }

  if (!gpsFrameFresh(now) && (now - gps.lastPollMs >= GPS_POLL_MS)) {
    gpsPollNavPvt();
  }

  if (lastFixDirty && sdReady && (now - lastFixSaveMs >= TRIP_DAT_MS)) {
    lastFixSave();
  }

  if (now - gps.lastDebugMs >= GPS_DEBUG_MS) {
    gps.lastDebugMs = now;
    bool live = gps.valid && gpsFrameFresh(now);
    unsigned long age = (gps.lastFrameMs == 0) ? 0 : (now - gps.lastFrameMs);
    Serial.print("GPS ");
    Serial.print(GPS_BAUD);
    Serial.print(" src=");
    Serial.print(gps.src ? gps.src : "-");
    Serial.print(live ? " LIVE" : " CONNECTING");
    Serial.print(" sats=");
    if (gpsFrameFresh(now) && gps.satsKnown) {
      Serial.print(gps.sats);
    } else {
      Serial.print("--");
    }
    Serial.print(" alt=");
    if (live && gps.altKnown) {
      Serial.print(gps.altM, 1);
    } else {
      Serial.print("--");
    }
    Serial.print(" age=");
    if (gps.lastFrameMs == 0) {
      Serial.print("never");
    } else {
      Serial.print(age);
      Serial.print("ms");
    }
    Serial.print(" rx=");
    if (gps.lastRxMs == 0) {
      Serial.print("none");
    } else {
      Serial.print(now - gps.lastRxMs);
      Serial.print("ms");
    }
    Serial.print(" bytes=");
    Serial.print(gps.rxBytes);
    Serial.print(" ubx=");
    Serial.print(gps.ubxOk);
    Serial.print(" nmea=");
    Serial.print(gps.nmeaOk);
    Serial.print(" fix=");
    Serial.print(gps.fixType);
    Serial.print(" ckfail=");
    Serial.print(gps.ckFail);
    Serial.print(" sd=");
    Serial.print(sdReady ? "ok" : "no");
    Serial.print(" gpx=");
    Serial.print(gpxPointCount);
    if (gps.dumpLen > 0) {
      Serial.print(" hex=");
      for (uint8_t i = 0; i < gps.dumpLen; i++) {
        if (gps.dump[i] < 16) {
          Serial.print('0');
        }
        Serial.print(gps.dump[i], HEX);
      }
    }
    Serial.println();
  }
}

// ---------------------------------------------------------------------------
// Drawing. Values that change are rendered into a 1-bit canvas and pushed to
// the panel as whole rows, background included, so nothing is cleared first
// and nothing blinks. Each burst is short so the GPS UART ring is drained.

static bool blePhoneConnected();
static bool bleAwaitingAuth();
static bool bleAuthProgress(unsigned long now, unsigned long *remainSec, int *lit);
static void bleConfirmAuth(unsigned long now);
static bool bleSendProgress(uint8_t *pct);

static GFXcanvas1 textCanvas(SCREEN_W, 32);
static uint16_t blitLine[SCREEN_W * 4];

enum {
  MARK_NONE = 0,
  MARK_RING,
  MARK_DOT,
  MARK_ALERT,
  MARK_PILL,
  MARK_SAT
};

static const int SAT_ICON_W = 17;
static const int SAT_ICON_H = 9;
static const uint8_t SAT_ICON[] = {
  0x00, 0x80, 0x00,
  0x00, 0x80, 0x00,
  0xF8, 0x8F, 0x80,
  0xA9, 0xCA, 0x80,
  0xA9, 0xCA, 0x80,
  0xFF, 0xFF, 0x80,
  0xA9, 0xCA, 0x80,
  0xA9, 0xCA, 0x80,
  0xF8, 0x0F, 0x80,
};

static const Slot SLOT_STATUS  = {108, 4, SCREEN_W - PAD - 108, 22, 15};
static const Slot SLOT_BATT    = {PAD + 30, 4, 108 - PAD - 30 - 4, 22, 15};
static const Slot SLOT_CAPTION = {PAD, CAPTION_Y, SCREEN_W - 2 * PAD, CAPTION_H, 15};
static const Slot SLOT_FOOT_L  = {PAD, FOOTER_Y, 122, FOOTER_H, 15};
static const Slot SLOT_FOOT_R  = {PAD + 122, FOOTER_Y, SCREEN_W - 2 * PAD - 122, FOOTER_H, 15};

static const char *const ROW_LABELS[ROW_COUNT] = {"Distance", "Time", "Moving", "Max"};

static bool cfgImperial() {
  return cfg.units == UNITS_IMPERIAL;
}

static const char *cfgDistUnit() {
  return cfgImperial() ? "mi" : "km";
}

static const char *cfgSpeedUnit() {
  return cfgImperial() ? "mph" : "km/h";
}

static float cfgSpeedShown(float kmh) {
  return cfgImperial() ? (kmh * 0.621371f) : kmh;
}

static float cfgDistShown(float km) {
  return cfgImperial() ? (km * 0.621371f) : km;
}

static float cfgAltShown(float m) {
  return cfgImperial() ? (m * 3.28084f) : m;
}

static const char *cfgAltUnit() {
  return cfgImperial() ? "ft" : "m";
}

static const char *rowUnit(int i) {
  if (i == 0) {
    return cfgDistUnit();
  }
  if (i == 3) {
    return cfgSpeedUnit();
  }
  return "";
}

// 5x7 matrix; bit (cols - 1) is the left column. ' ' is an unlit digit cell.
static const DotGlyph DOT_FONT[] = {
  {' ', 5, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
  {'0', 5, {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
  {'1', 5, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}},
  {'2', 5, {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}},
  {'3', 5, {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E}},
  {'4', 5, {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}},
  {'5', 5, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}},
  {'6', 5, {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}},
  {'7', 5, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
  {'8', 5, {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}},
  {'9', 5, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}},
  {'-', 5, {0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00}},
  {'.', 1, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}},
  {':', 1, {0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00}},
};

static const DotGlyph *dotGlyph(char c) {
  for (size_t i = 0; i < sizeof(DOT_FONT) / sizeof(DOT_FONT[0]); i++) {
    if (DOT_FONT[i].ch == c) {
      return &DOT_FONT[i];
    }
  }
  return &DOT_FONT[0];
}

static bool dotOn(const DotGlyph *g, int row, int col) {
  return (g->rows[row] >> (g->cols - 1 - col)) & 1;
}

// Strip the fill so a NAV-PVT burst can be read out of the 64-byte UART ring.
static void fillRectDrained(int x, int y, int w, int h, uint16_t color) {
  if (w <= 0 || h <= 0) {
    return;
  }
  const int strip = 8;
  for (int yy = y; yy < y + h; yy += strip) {
    int hh = strip;
    if (yy + hh > y + h) {
      hh = y + h - yy;
    }
    display.fillRect(x, yy, w, hh, color);
    gpsDrain();
  }
}

static void fillScreenDrained(uint16_t color) {
  const int band = 16;
  for (int y = 0; y < SCREEN_H; y += band) {
    int h = band;
    if (y + h > SCREEN_H) {
      h = SCREEN_H - y;
    }
    display.fillRect(0, y, SCREEN_W, h, color);
    gpsDrain();
  }
}

static void printDrained(const char *s) {
  if (s == NULL) {
    return;
  }
  while (*s) {
    gpsDrain();
    display.write((uint8_t)*s++);
  }
  gpsDrain();
}

static void drawLabel(const GFXfont *font, int x, int base, uint16_t color, const char *s) {
  display.setFont(font);
  display.setTextColor(color);
  display.setCursor(x, base);
  printDrained(s);
  display.setFont(NULL);
}

static int trackedWidth(const GFXfont *font, int track, const char *s) {
  int w = 0;
  for (const char *p = s; *p; p++) {
    uint8_t c = (uint8_t)*p;
    if (c < font->first || c > font->last) {
      continue;
    }
    w += font->glyph[c - font->first].xAdvance;
    if (p[1]) {
      w += track;
    }
  }
  return w;
}

// The wordmark is set with extra letter spacing, like the page header.
static void drawTracked(const GFXfont *font, int x, int base, int track,
                        const char *s, uint16_t color) {
  display.setFont(font);
  display.setTextColor(color);
  for (const char *p = s; *p; p++) {
    display.setCursor(x, base);
    display.write((uint8_t)*p);
    x = display.getCursorX() + track;
    gpsDrain();
  }
  display.setFont(NULL);
}

// Push a w x h region of textCanvas to the panel, 4 rows per SPI burst.
static void blitCanvas(int x, int y, int w, int h, uint16_t fg, uint16_t bg) {
  const uint8_t *buf = textCanvas.getBuffer();
  const int stride = (SCREEN_W + 7) / 8;
  for (int r0 = 0; r0 < h; r0 += 4) {
    int rows = h - r0;
    if (rows > 4) {
      rows = 4;
    }
    uint16_t *dst = blitLine;
    for (int r = r0; r < r0 + rows; r++) {
      const uint8_t *row = buf + r * stride;
      for (int c = 0; c < w; c++) {
        *dst++ = (row[c >> 3] & (0x80 >> (c & 7))) ? fg : bg;
      }
    }
    display.startWrite();
    display.setAddrWindow(x, y + r0, w, rows);
    display.writePixels(blitLine, (uint32_t)(w * rows));
    display.endWrite();
    gpsDrain();
  }
}

static void invalidateText(TextCache *c) {
  c->valid = false;
  c->text[0] = '\0';
}

// One text slot: optional left text, optional right-aligned text with a
// status mark before it (ring, dot, alert) or a filled pill around it.
static void drawSlot(const Slot &s, TextCache *cache, const GFXfont *font,
                     const char *left, const char *right, uint8_t mark, uint16_t fg) {
  char key[sizeof(cache->text)];
  snprintf(key, sizeof(key), "%s\x1f%s", left, right);
  if (cache->valid && cache->color == fg && cache->mark == mark &&
      strcmp(cache->text, key) == 0) {
    return;
  }

  textCanvas.fillRect(0, 0, s.w, s.h, 0);
  textCanvas.setTextWrap(false);
  textCanvas.setFont(font);

  if (left[0]) {
    textCanvas.setTextColor(1);
    textCanvas.setCursor(0, s.base);
    textCanvas.print(left);
  }

  if (right[0]) {
    int16_t bx, by;
    uint16_t bw, bh;
    textCanvas.getTextBounds(right, 0, s.base, &bx, &by, &bw, &bh);
    int inkLeft = s.w - (int)bw;
    uint16_t ink = 1;
    const int cy = s.base - 6;

    if (mark == MARK_PILL) {
      const int padX = 6;
      int pillW = (int)bw + 2 * padX;
      int pillX = s.w - pillW;
      textCanvas.fillRoundRect(pillX, s.base - 14, pillW, 19, 9, 1);
      inkLeft = pillX + padX;
      ink = 0;
    } else if (mark == MARK_ALERT) {
      int cx = inkLeft - 7 - 7;
      textCanvas.fillCircle(cx, cy, 7, 1);
      textCanvas.setFont(NULL);
      textCanvas.setTextColor(0);
      textCanvas.setCursor(cx - 2, cy - 3);
      textCanvas.print('!');
      textCanvas.setFont(font);
    } else if (mark == MARK_SAT) {
      textCanvas.drawBitmap(inkLeft - 6 - SAT_ICON_W, cy - SAT_ICON_H / 2,
                            SAT_ICON, SAT_ICON_W, SAT_ICON_H, 1);
    } else if (mark == MARK_DOT || mark == MARK_RING) {
      int cx = inkLeft - 8 - 4;
      if (mark == MARK_DOT) {
        textCanvas.fillCircle(cx, cy, 4, 1);
      } else {
        textCanvas.drawCircle(cx, cy, 4, 1);
        textCanvas.drawCircle(cx, cy, 3, 1);
      }
    }

    textCanvas.setTextColor(ink);
    textCanvas.setCursor(inkLeft - bx, s.base);
    textCanvas.print(right);
  }

  blitCanvas(s.x, s.y, s.w, s.h, fg, COL_BG);

  strncpy(cache->text, key, sizeof(cache->text) - 1);
  cache->text[sizeof(cache->text) - 1] = '\0';
  cache->color = fg;
  cache->mark = mark;
  cache->valid = true;
}

static uint16_t levelColor(uint8_t lv) {
  switch (lv) {
    case LV_OFF: return COL_OFF;
    case LV_LO: return COL_LO;
    case LV_MID: return COL_DIM;
    case LV_FG: return COL_FG;
    default: return COL_BG;
  }
}

static bool matDigitCol(int c) {
  return c != 5 && c != 11 && c != 13;
}

// Unlit digit cells, black gaps: the matrix with nothing on it.
static void matGrid() {
  for (int r = 0; r < MAT_ROWS; r++) {
    for (int c = 0; c < MAT_COLS; c++) {
      matNext[r][c] = matDigitCol(c) ? LV_OFF : LV_BG;
    }
  }
}

static void matSet(int r, int c, uint8_t lv) {
  if (r >= 0 && r < MAT_ROWS && c >= 0 && c < MAT_COLS) {
    matNext[r][c] = lv;
  }
}

// Brightest wins, so particle trails never dim a head drawn earlier.
static void matMax(int r, int c, uint8_t lv) {
  if (r >= 0 && r < MAT_ROWS && c >= 0 && c < MAT_COLS && matNext[r][c] < lv) {
    matNext[r][c] = lv;
  }
}

static int dotTextCols(const char *s) {
  int cols = 0;
  int n = 0;
  for (const char *p = s; *p; p++, n++) {
    cols += dotGlyph(*p)->cols;
  }
  return n ? cols + n - 1 : 0;
}

// Glyph cells get `on` or `off`; LV_KEEP leaves what is underneath.
static void matText(const char *s, int col0, int row0, uint8_t on, uint8_t off) {
  int x = col0;
  for (const char *p = s; *p; p++) {
    const DotGlyph *g = dotGlyph(*p);
    for (int r = 0; r < 7; r++) {
      for (int c = 0; c < g->cols; c++) {
        uint8_t lv = dotOn(g, r, c) ? on : off;
        if (lv != LV_KEEP) {
          matSet(row0 + r, x + c, lv);
        }
      }
    }
    x += g->cols + 1;
  }
}

static bool matTextLit(const char *s, int col0, int r, int c) {
  if (r < 0 || r >= 7) {
    return false;
  }
  int x = col0;
  for (const char *p = s; *p; p++) {
    const DotGlyph *g = dotGlyph(*p);
    if (c >= x && c < x + g->cols) {
      return dotOn(g, r, c - x);
    }
    x += g->cols + 1;
  }
  return false;
}

static void matDigits(const char *s, int row0) {
  matGrid();
  matText(s, 0, row0, LV_FG, LV_OFF);
}

static void flushMatrix() {
  const int x0 = (SCREEN_W - (MAT_COLS * HERO_PITCH - (HERO_PITCH - HERO_DOT))) / 2;
  for (int r = 0; r < MAT_ROWS; r++) {
    for (int c = 0; c < MAT_COLS; c++) {
      uint8_t lv = matNext[r][c];
      if (lv == matShown[r][c]) {
        continue;
      }
      display.fillRect(x0 + c * HERO_PITCH, HERO_Y + r * HERO_PITCH,
                       HERO_DOT, HERO_DOT, levelColor(lv));
      matShown[r][c] = lv;
    }
    gpsDrain();
  }
}

static void gaugeBase(int lit) {
  for (int i = 0; i < GAUGE_DOTS; i++) {
    gaugeNext[i] = i < lit ? LV_FG : LV_OFF;
  }
}

static void flushGauge() {
  const int x0 = (SCREEN_W - (GAUGE_DOTS - 1) * GAUGE_PITCH) / 2;
  bool any = false;
  for (int i = 0; i < GAUGE_DOTS; i++) {
    uint8_t lv = gaugeNext[i];
    if (lv == gaugeShown[i]) {
      continue;
    }
    display.fillCircle(x0 + i * GAUGE_PITCH, GAUGE_Y, GAUGE_R, levelColor(lv));
    gaugeShown[i] = lv;
    any = true;
  }
  if (any) {
    gpsDrain();
  }
}

static void rowLabel(int i, char *buf, size_t buflen) {
  const char *u = rowUnit(i);
  snprintf(buf, buflen, u[0] ? "%s %s" : "%s", ROW_LABELS[i], u);
}

static int rowTop(int i) {
  return ROWS_Y + i * ROW_H;
}

// Values end at the right margin; the slot starts clear of the label.
static Slot rowSlot(int i) {
  char label[24];
  rowLabel(i, label, sizeof(label));
  int x = PAD + trackedWidth(&FreeSans9pt7b, 0, label) + 8;
  if (x < VALUE_X) {
    x = VALUE_X;
  }
  Slot s;
  s.x = x;
  s.y = rowTop(i) + 4;
  s.w = SCREEN_W - PAD - x;
  s.h = 28;
  s.base = 21;
  return s;
}

static void drawStaticChrome() {
  for (int i = 0; i <= ROW_COUNT; i++) {
    display.drawFastHLine(PAD, rowTop(i), SCREEN_W - 2 * PAD, COL_RULE);
  }
  gpsDrain();

  for (int i = 0; i < ROW_COUNT; i++) {
    const int base = rowTop(i) + 4 + 21;
    char label[24];
    rowLabel(i, label, sizeof(label));
    drawLabel(&FreeSans9pt7b, PAD, base, COL_DIM, label);
  }
}

static void invalidateAllFields() {
  invalidateText(&txtBatt);
  battBarsShown = -1;
  invalidateText(&txtStatus);
  invalidateText(&txtCaption);
  for (int i = 0; i < ROW_COUNT; i++) {
    invalidateText(&txtRow[i]);
  }
  invalidateText(&txtFootL);
  invalidateText(&txtFootR);
  memset(matShown, LV_STALE, sizeof(matShown));
  memset(gaugeShown, LV_STALE, sizeof(gaugeShown));
}

// Use the same `now` as drawRideScreen / animCompose. Starting the boot clip
// with a later millis() after the chrome SPI paint makes clipAt underflow and
// drop the self-test on the first frame.
static void ensureChrome(unsigned long now) {
  if (uiChromeDrawn) {
    return;
  }
  fillScreenDrained(COL_BG);
  display.setTextWrap(false);
  drawStaticChrome();
  invalidateAllFields();
  uiChromeDrawn = true;
  if (animBootPending) {
    animBootPending = false;
    clipStart(clipBoot, now);
  }
}

// H:MM:SS always (matches Figma samples like 7:34:12).
static void formatHms(unsigned long ms, char *buf, size_t buflen) {
  unsigned long totalSec = ms / 1000UL;
  unsigned long h = totalSec / 3600UL;
  unsigned long m = (totalSec / 60UL) % 60UL;
  unsigned long s = totalSec % 60UL;
  snprintf(buf, buflen, "%lu:%02lu:%02lu", h, m, s);
}

enum {
  OVL_NONE = 0,
  OVL_FLASH,
  OVL_HOLD,
  OVL_BLE_AUTH
};

// Hold-for-new-ride countdown, phone-connect confirm, or a short flash.
static uint8_t overlayState(unsigned long now, unsigned long *remainSec, int *holdLit) {
  if (overlayFlashMsg && (long)(overlayFlashUntilMs - now) > 0) {
    return OVL_FLASH;
  }
  overlayFlashMsg = NULL;

  if (btnArmed && btnHeld && !btnDidExec) {
    unsigned long held = now - btnHoldStartMs;
    if (held >= BTN_ARM_MS && held < BTN_EXEC_MS) {
      unsigned long remainMs = BTN_EXEC_MS - held;
      *remainSec = (remainMs + 999UL) / 1000UL;
      *holdLit = (int)((long)(held - BTN_ARM_MS) * GAUGE_DOTS /
                       (long)(BTN_EXEC_MS - BTN_ARM_MS)) + 1;
      return OVL_HOLD;
    }
  }

  if (bleAuthProgress(now, remainSec, holdLit)) {
    return OVL_BLE_AUTH;
  }
  return OVL_NONE;
}

// --- Matrix animations -------------------------------------------------
// Each frame is a pure function of the time since its trigger (and a hash
// seeded by that time), so the emulator captures the same pixels every run.
// While the wheel turns the digits stay digits: only the heartbeat, the
// gauge effects and the distance milestone play.

static const unsigned long ANIM_BOOT_MS = 700;
static const unsigned long ANIM_BEAT_MS = 120;
static const unsigned long ANIM_FIREWORK_MS = BTN_FLASH_MS;
static const unsigned long ANIM_RUNE_MS = 1300;
static const unsigned long ANIM_NEWMAX_MS = 600;
static const unsigned long ANIM_MS_LIVE_MS = 700;
static const unsigned long ANIM_MILESTONE_MS = 2800;
static const unsigned long NEWMAX_MOVING_MS = 120000;
static const unsigned long NEWMAX_GAP_MS = 60000;
static const float NEWMAX_STEP_KMH = 0.5f;
static const unsigned long FACE_AFTER_MS = 8000;
static const unsigned long FACE_SLEEPY_MS = 120000;
static const unsigned long FACE_ASLEEP_MS = 20000;  // counted from sleepy
static const int MILESTONE_STEP = 10;               // km or mi

static bool animOn() {
  return cfg.animations && blMode != BL_OFF;
}

static bool clipAt(AnimClip &c, unsigned long now, unsigned long durMs, unsigned long *t) {
  if (!c.on) {
    return false;
  }
  unsigned long el = now - c.startMs;
  if (el >= durMs) {
    c.on = false;
    return false;
  }
  *t = el;
  return true;
}

static uint32_t animHash(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7FEB352DUL;
  x ^= x >> 15;
  x *= 0x846CA68BUL;
  x ^= x >> 16;
  return x;
}

static void animNoteMax(unsigned long now, float kmh) {
  if (movingMs < NEWMAX_MOVING_MS) {
    animMaxBaseline = kmh;
    return;
  }
  if (kmh < animMaxBaseline + NEWMAX_STEP_KMH) {
    return;
  }
  if (clipNewMax.startMs != 0 && now - clipNewMax.startMs < NEWMAX_GAP_MS) {
    return;
  }
  animMaxBaseline = kmh;
  if (animOn()) {
    clipStart(clipNewMax, now);
  }
}

static void animTrackDistance(unsigned long now, float distanceKm) {
  long idx = (long)(cfgDistShown(distanceKm) / (float)MILESTONE_STEP);
  if (!msBaselineValid || idx < msBaseline) {
    msBaseline = idx;
    msBaselineValid = true;
    return;
  }
  if (idx == msBaseline) {
    return;
  }
  msBaseline = idx;
  if (!animOn()) {
    return;
  }
  long value = idx * MILESTONE_STEP;
  if (value < 1000) {
    msShowValue = (uint16_t)value;
    clipStart(clipMilestone, now);
  } else {
    clipStart(clipMsLive, now);
  }
}

// Boot self-test: a diagonal band lights every cell and dot once.
static void fxBoot(unsigned long t) {
  int head = (int)(t * 28 / 600) - 1;
  for (int r = 0; r < MAT_ROWS; r++) {
    for (int c = 0; c < MAT_COLS; c++) {
      int d = head - (r + c);
      if (d < 0) {
        matNext[r][c] = LV_BG;
      } else if (d < 3) {
        matNext[r][c] = (uint8_t)(LV_FG - d);
      }
    }
  }
  for (int i = 0; i < GAUGE_DOTS; i++) {
    int d = head - (i + 1);
    if (d < 0) {
      gaugeNext[i] = LV_BG;
    } else if (d < 3) {
      gaugeNext[i] = (uint8_t)(LV_FG - d);
    }
  }
}

static void fxHeartbeat(unsigned long now) {
  unsigned long t;
  if (!clipAt(clipBeat, now, ANIM_BEAT_MS, &t)) {
    return;
  }
  for (int r = 0; r < 6; r++) {
    if (matNext[r][12] == LV_OFF) {
      matNext[r][12] = LV_LO;
    }
  }
}

// New ride hold: the digits drain from the top as the gauge fills.
static void fxDrain(unsigned long now) {
  long held = (long)(now - btnHoldStartMs) - (long)BTN_ARM_MS;
  float e = (float)held * MAT_ROWS / (float)(BTN_EXEC_MS - BTN_ARM_MS);
  if (e <= 0.0f) {
    return;
  }
  if (e > MAT_ROWS) {
    e = MAT_ROWS;
  }
  int gone = (int)e;
  for (int r = 0; r < MAT_ROWS; r++) {
    for (int c = 0; c < MAT_COLS; c++) {
      if (matNext[r][c] != LV_FG) {
        continue;
      }
      if (r < gone) {
        matNext[r][c] = LV_OFF;
      } else if (r == gone) {
        matNext[r][c] = LV_MID;
      }
    }
  }
}

// Ride saved / Stats reset: burst, particles, then the new 0.0 drops in.
static void fxFirework(unsigned long t, const char *digits, uint32_t seed) {
  if (t >= 1400) {
    return;
  }
  if (t >= 900) {
    unsigned long u = t - 900;
    int row0 = 0;
    if (u < 250) {
      float f = u / 250.0f;
      row0 = (int)lroundf(-7.0f + 8.0f * f * f);
    } else if (u < 330) {
      row0 = 1;
    }
    matDigits(digits, row0);
    return;
  }
  matGrid();
  if (t < 150) {
    matSet(3, 9, LV_FG);
    if (t >= 75) {
      matMax(2, 9, LV_MID);
      matMax(4, 9, LV_MID);
      matMax(3, 8, LV_MID);
      matMax(3, 10, LV_MID);
    }
    return;
  }
  float tau = (t - 150) / 1000.0f;
  bool fading = tau > 0.5f;
  for (int i = 0; i < 14; i++) {
    uint32_t h = animHash(seed + (uint32_t)i * 0x9E3779B9UL);
    float ang = i * (6.2831853f / 14.0f) + ((h & 0xFF) / 255.0f - 0.5f) * 0.4f;
    float v = 16.0f + ((h >> 8) & 0xFF) / 255.0f * 10.0f;  // cells per second
    for (int k = 2; k >= 0; k--) {
      float tt = tau - k * 0.04f;
      if (tt < 0.0f) {
        continue;
      }
      float x = 9.0f + cosf(ang) * v * tt;
      float y = 3.0f + sinf(ang) * v * tt * 0.5f + 10.0f * tt * tt;
      int lv = LV_FG - k - (fading ? 1 : 0);
      if (lv <= LV_OFF) {
        continue;
      }
      matMax((int)lroundf(y), (int)lroundf(x), (uint8_t)lv);
    }
  }
  float rad = tau / 0.75f * 14.0f;
  for (int i = 0; i < GAUGE_DOTS; i++) {
    float d = rad - fabsf(i - 11.5f);
    int lv = LV_OFF;
    if (d >= 0.0f && d < 1.5f) {
      lv = LV_FG;
    } else if (d >= 1.5f && d < 3.0f) {
      lv = LV_MID;
    } else if (d >= 3.0f && d < 4.5f) {
      lv = LV_LO;
    }
    if (fading && lv > LV_OFF) {
      lv--;
    }
    if (lv > gaugeNext[i]) {
      gaugeNext[i] = (uint8_t)lv;
    }
  }
}

// New max: a comet runs the gauge twice; over lit dots it is a dark notch.
static void fxComet(unsigned long t) {
  int h = (int)((t % 300) * 27 / 300);
  for (int i = 0; i < GAUGE_DOTS; i++) {
    int d = h - i;
    if (d < 0 || d > 2) {
      continue;
    }
    gaugeNext[i] = gaugeNext[i] == LV_FG ? (uint8_t)(LV_OFF + d) : (uint8_t)(LV_FG - d);
  }
}

// Milestone past 999: the gauge fills left to right, then a dim wash follows it.
static void fxSweep(unsigned long t) {
  if (t < 350) {
    int k = (int)(t * (GAUGE_DOTS + 1) / 350);
    for (int i = 0; i < k && i < GAUGE_DOTS; i++) {
      gaugeNext[i] = LV_FG;
    }
    return;
  }
  int k = (int)((t - 350) * (GAUGE_DOTS + 1) / 350);
  for (int i = 0; i < GAUGE_DOTS; i++) {
    gaugeNext[i] = i < k ? LV_LO : LV_FG;
  }
}

static void gaugeFx(unsigned long now, uint8_t ovl, bool sending) {
  if (ovl == OVL_HOLD || ovl == OVL_BLE_AUTH || sending) {
    return;
  }
  unsigned long t;
  if (clipAt(clipMsLive, now, ANIM_MS_LIVE_MS, &t)) {
    fxSweep(t);
    return;
  }
  if (clipAt(clipNewMax, now, ANIM_NEWMAX_MS, &t)) {
    fxComet(t);
  }
}

// Press to allow: chevrons flow down each digit block toward the button.
static void fxPressArrow(unsigned long now) {
  static const uint8_t CHEVRON[3] = {0x11, 0x0A, 0x04};
  static const int BLOCK_COL[3] = {0, 6, 14};
  matGrid();
  int off = (int)(((now - clipAuth.startMs) / 90) % 4);
  for (int y = off - 4; y < MAT_ROWS; y += 4) {
    for (int dy = 0; dy < 3; dy++) {
      int r = y + dy;
      if (r < 0 || r >= MAT_ROWS) {
        continue;
      }
      uint8_t lv = r <= 1 ? LV_LO : (r <= 3 ? LV_MID : LV_FG);
      for (int b = 0; b < 3; b++) {
        for (int c = 0; c < 5; c++) {
          if ((CHEVRON[dy] >> (4 - c)) & 1) {
            matSet(r, BLOCK_COL[b] + c, lv);
          }
        }
      }
    }
  }
}

// Phone allowed: the Bluetooth mark, drawn along its pen path, then fades.
static const uint8_t RUNE_PATH[][2] = {
  {5, 0}, {4, 1}, {3, 2}, {2, 3}, {1, 4}, {0, 3}, {0, 2}, {1, 2}, {2, 2},
  {4, 2}, {5, 2}, {6, 2}, {6, 3}, {5, 4}, {4, 3}, {2, 1}, {1, 0}
};

static void fxRune(unsigned long t) {
  matGrid();
  if (t >= 1200) {
    return;
  }
  const int n = sizeof(RUNE_PATH) / sizeof(RUNE_PATH[0]);
  int shown = t >= 600 ? n : (int)(t * n / 600) + 1;
  uint8_t lv = t < 1000 ? LV_FG : (t < 1100 ? LV_MID : LV_LO);
  for (int i = 0; i < shown; i++) {
    matSet(RUNE_PATH[i][0], 7 + RUNE_PATH[i][1], lv);
  }
}

static uint8_t sparkle(int r, int c, unsigned long t, uint32_t seed, uint32_t density) {
  uint32_t h = animHash(seed ^ ((uint32_t)(r * MAT_COLS + c) * 0x85EBCA6BUL)
                        ^ ((uint32_t)(t / 100) * 0xC2B2AE35UL));
  if (h % density) {
    return LV_KEEP;
  }
  return (uint8_t)(LV_LO + (h >> 16) % 2);
}

// Sparkles keep one cell clear of the number so it never looks smudged.
static bool nearText(const char *s, int col0, int r, int c) {
  for (int dr = -1; dr <= 1; dr++) {
    for (int dc = -1; dc <= 1; dc++) {
      if (matTextLit(s, col0, r + dr, c + dc)) {
        return true;
      }
    }
  }
  return false;
}

// Milestone: sparkles, the distance pops in, then dissolves back to the speed.
static void fxMilestone(unsigned long t, const char *digits, uint16_t value, uint32_t seed) {
  char num[6];
  snprintf(num, sizeof(num), "%u", (unsigned)value);
  int col0 = (MAT_COLS - dotTextCols(num)) / 2;
  if (t < 400) {
    for (int r = 0; r < MAT_ROWS; r++) {
      for (int c = 0; c < MAT_COLS; c++) {
        if (matNext[r][c] == LV_FG) {
          if (t >= animHash(seed + r * MAT_COLS + c) % 400) {
            matNext[r][c] = LV_OFF;
          }
          continue;
        }
        uint8_t lv = sparkle(r, c, t, seed, t < 200 ? 18 : 10);
        if (lv != LV_KEEP) {
          matNext[r][c] = lv;
        }
      }
    }
    return;
  }
  matGrid();
  if (t < 2200) {
    for (int r = 0; r < MAT_ROWS; r++) {
      for (int c = 0; c < MAT_COLS; c++) {
        uint8_t lv = sparkle(r, c, t, seed, 8);
        if (lv != LV_KEEP && !nearText(num, col0, r, c)) {
          matNext[r][c] = lv;
        }
      }
    }
    matText(num, col0, 0, t < 480 ? LV_MID : LV_FG, LV_OFF);
    if (t < 1100) {
      fxSweep(t - 400);
    }
    return;
  }
  for (int r = 0; r < MAT_ROWS; r++) {
    for (int c = 0; c < MAT_COLS; c++) {
      uint32_t h = animHash(seed + r * MAT_COLS + c + 977);
      bool numOn = matTextLit(num, col0, r, c) && t < 2200 + h % 400;
      bool digOn = matTextLit(digits, 0, r, c) && t >= 2400 + (h >> 10) % 400;
      if (numOn || digOn) {
        matNext[r][c] = LV_FG;
      } else if (t < 2600 && sparkle(r, c, t, seed, 24) != LV_KEEP
                 && !nearText(num, col0, r, c)) {
        matNext[r][c] = LV_LO;
      }
    }
  }
}

// The 0.0 face. Eye shapes are 5x7 rows like DOT_FONT; pupils are 1x2.
enum { EYE_OPEN, EYE_SQUINT, EYE_SHUT, EYE_LID, EYE_LID_LOW, EYE_SLEEP };
static const uint8_t EYE_ROWS[][7] = {
  {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
  {0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x00},
  {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x1F, 0x11, 0x11, 0x11, 0x0E},
  {0x00, 0x00, 0x00, 0x1F, 0x11, 0x11, 0x0E},
  {0x00, 0x00, 0x00, 0x11, 0x0E, 0x00, 0x00},
};
static const int8_t EYE_PUPIL_TOP[] = {1, 2, 7, 3, 4, 7};
static const int8_t EYE_PUPIL_BOT[] = {5, 4, -1, 5, 5, -1};
static const int8_t GAZE[][2] = {{2, 3}, {1, 3}, {3, 3}, {2, 3}, {2, 1}, {2, 4}};

static void faceEye(int col0, uint8_t shape, int px, int py, uint8_t pupLv) {
  for (int r = 0; r < MAT_ROWS; r++) {
    for (int c = 0; c < 5; c++) {
      if ((EYE_ROWS[shape][r] >> (4 - c)) & 1) {
        matSet(r, col0 + c, LV_FG);
      }
    }
  }
  for (int r = py; r <= py + 1; r++) {
    if (r >= EYE_PUPIL_TOP[shape] && r <= EYE_PUPIL_BOT[shape]) {
      matSet(r, col0 + px, pupLv);
    }
  }
}

static uint8_t faceBlink(unsigned long d) {
  return d < 50 ? EYE_SQUINT : (d < 130 ? EYE_SHUT : EYE_SQUINT);
}

static uint8_t faceAwakeShape(unsigned long ft, uint32_t seed) {
  if (ft >= 300 && ft < 500) {
    return faceBlink(ft - 300);
  }
  if (ft < 2000) {
    return EYE_OPEN;
  }
  unsigned long j = (ft - 2000) / 4500;
  uint32_t h = animHash(seed ^ ((uint32_t)j * 0x165667B1UL));
  unsigned long tb = 2000 + j * 4500 + h % 1500;
  if (ft >= tb) {
    unsigned long d = ft - tb;
    if (d < 200) {
      return faceBlink(d);
    }
    if ((h & 0x10000UL) && d >= 320 && d < 520) {
      return faceBlink(d - 320);
    }
  }
  return EYE_OPEN;
}

static int stepToward(int from, int to, int steps) {
  int d = to - from;
  if (d > steps) {
    d = steps;
  } else if (d < -steps) {
    d = -steps;
  }
  return from + d;
}

static void faceGaze(unsigned long ft, uint32_t seed, int *px, int *py) {
  *px = GAZE[0][0];
  *py = GAZE[0][1];
  if (ft < 800) {
    return;
  }
  unsigned long k = (ft - 800) / 3500;
  uint32_t h = animHash(seed ^ ((uint32_t)k * 0x27D4EB2FUL));
  int prev = k == 0 ? 0 : (int)((animHash(seed ^ ((uint32_t)(k - 1) * 0x27D4EB2FUL)) >> 8) % 6);
  int cur = (int)((h >> 8) % 6);
  unsigned long tc = 800 + k * 3500 + h % 1500;
  *px = GAZE[prev][0];
  *py = GAZE[prev][1];
  if (ft < tc) {
    return;
  }
  int steps = (int)((ft - tc) / 60);
  *px = stepToward(*px, GAZE[cur][0], steps);
  *py = stepToward(*py, GAZE[cur][1], steps);
}

// One z at a time rises through the empty tens block and fades.
static void faceZ(unsigned long zt) {
  static const uint8_t Z[4] = {0xF, 0x2, 0x4, 0xF};
  unsigned long p = zt % 2000;
  int row0 = 3 - (int)(p / 500);
  int col0 = p < 1000 ? 1 : 0;
  uint8_t lv = p < 1000 ? LV_FG : (p < 1500 ? LV_MID : LV_LO);
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      if ((Z[r] >> (3 - c)) & 1) {
        matSet(row0 + r, col0 + c, lv);
      }
    }
  }
}

static void faceBreath(unsigned long zt) {
  unsigned long p = zt % 4000;
  uint8_t mid = (p >= 1000 && p < 3000) ? LV_MID : LV_LO;
  gaugeNext[11] = mid;
  gaugeNext[12] = mid;
  if (p >= 1000 && p < 2000) {
    gaugeNext[10] = LV_LO;
    gaugeNext[13] = LV_LO;
  }
}

static void fxFace(unsigned long ft, unsigned long stillMs, uint32_t seed) {
  matGrid();
  matSet(6, 12, LV_FG);
  uint8_t shape;
  int px = GAZE[0][0];
  int py = GAZE[0][1];
  uint8_t pupLv = LV_FG;
  if (stillMs < FACE_SLEEPY_MS) {
    shape = faceAwakeShape(ft, seed);
    faceGaze(ft, seed, &px, &py);
    if (ft < 100) {
      pupLv = LV_LO;
    } else if (ft < 200) {
      pupLv = LV_MID;
    }
  } else {
    unsigned long st = stillMs - FACE_SLEEPY_MS;
    if (st < FACE_ASLEEP_MS) {
      shape = EYE_LID;
      unsigned long j = st / 6000;
      unsigned long tb = j * 6000 + 1500 + animHash(seed ^ ((uint32_t)j * 0x9E3779B1UL)) % 2000;
      if (st >= tb && st - tb < 700) {
        unsigned long d = st - tb;
        shape = d < 150 ? EYE_LID_LOW : (d < 550 ? EYE_SHUT : EYE_LID_LOW);
      }
    } else {
      unsigned long zt = st - FACE_ASLEEP_MS;
      if (zt < 400) {
        shape = EYE_LID_LOW;
      } else if (zt < 800) {
        shape = EYE_SHUT;
      } else {
        shape = EYE_SLEEP;
        faceZ(zt - 800);
        faceBreath(zt - 800);
      }
    }
  }
  faceEye(6, shape, px, py, pupLv);
  faceEye(14, shape, px, py, pupLv);
}

// Picks the one matrix scene for this frame, over the digits and gauge
// already composed, then adds the overlays that scene allows.
static void animCompose(unsigned long now, float speedKmh, uint8_t ovl, bool sending,
                        unsigned long lastPulseMs, unsigned long revCount,
                        const char *digits) {
  const bool moving = speedKmh > 0.0f;
  if (revCount != animRevSeen) {
    if (revCount > animRevSeen) {
      clipStart(clipBeat, now);
    }
    animRevSeen = revCount;
  }
  if (moving) {
    zeroSinceMs = 0;
  } else if (zeroSinceMs == 0) {
    zeroSinceMs = now ? now : 1;
  }
  if (!animOn()) {
    return;
  }

  unsigned long t;
  if (clipAt(clipBoot, now, ANIM_BOOT_MS, &t)) {
    fxBoot(t);
    return;
  }
  if (ovl == OVL_HOLD) {
    fxDrain(now);
    return;
  }
  if (ovl == OVL_FLASH && clipAt(clipFirework, now, ANIM_FIREWORK_MS, &t)) {
    fxFirework(t, digits, clipFirework.startMs);
    return;
  }
  if (ovl == OVL_NONE && !sending
      && clipAt(clipMilestone, now, ANIM_MILESTONE_MS, &t)) {
    fxMilestone(t, digits, msShowValue, clipMilestone.startMs);
    msEndMs = now;
    return;
  }
  if (moving) {
    fxHeartbeat(now);
    gaugeFx(now, ovl, sending);
    return;
  }
  if (ovl == OVL_BLE_AUTH) {
    fxPressArrow(now);
    return;
  }
  if (clipAt(clipRune, now, ANIM_RUNE_MS, &t)) {
    fxRune(t);
    return;
  }
  gaugeFx(now, ovl, sending);
  if (tripStarted && lastPulseMs != 0 && now - lastPulseMs >= FACE_AFTER_MS) {
    unsigned long anchor = lastPulseMs + FACE_AFTER_MS;
    if ((long)(msEndMs - anchor) > 0) {
      anchor = msEndMs;
    }
    if ((long)(zeroSinceMs - anchor) > 0) {
      anchor = zeroSinceMs;
    }
    if ((long)(now - anchor) >= 0) {
      fxFace(now - anchor, now - lastPulseMs, lastPulseMs);
    }
  }
}

static void paintStatus(unsigned long now) {
  char buf[20];
  if (gpsIsLive(now)) {
    if (gps.satsKnown) {
      snprintf(buf, sizeof(buf), "%u", (unsigned)gps.sats);
      drawSlot(SLOT_STATUS, &txtStatus, &FreeSans9pt7b, "", buf, MARK_SAT, COL_FG);
    } else {
      drawSlot(SLOT_STATUS, &txtStatus, &FreeSans9pt7b, "", "GPS", MARK_DOT, COL_FG);
    }
  } else {
    drawSlot(SLOT_STATUS, &txtStatus, &FreeSans9pt7b, "", "Searching", MARK_RING, COL_DIM);
  }
}

static int battPercent(float v) {
  static const float V[] = {4.20f, 4.00f, 3.85f, 3.75f, 3.65f, 3.50f, 3.30f};
  static const float P[] = {100.0f, 80.0f, 60.0f, 40.0f, 20.0f, 5.0f, 0.0f};
  const int n = sizeof(V) / sizeof(V[0]);
  if (v >= V[0]) {
    return 100;
  }
  if (v <= V[n - 1]) {
    return 0;
  }
  for (int i = 1; i < n; i++) {
    if (v >= V[i]) {
      float p = P[i] + (v - V[i]) * (P[i - 1] - P[i]) / (V[i - 1] - V[i]);
      return (int)(p + 0.5f);
    }
  }
  return 0;
}

static void paintBatteryIcon(int bars) {
  const int x = PAD;
  const int y = BATT_ICON_Y;
  display.drawRect(x, y, 22, 12, COL_FG);
  display.fillRect(x + 22, y + 3, 2, 6, COL_FG);
  for (int i = 0; i < 5; i++) {
    display.fillRect(x + 2 + i * 4, y + 2, 3, 8, i < bars ? COL_FG : COL_OFF);
  }
}

static void paintBattery() {
  if (battVolts < 0.0f) {
    return;
  }
  int pct = battPercent(battVolts);
  int bars = (pct + 10) / 20;
  if (bars > 5) {
    bars = 5;
  }
  if (bars != battBarsShown) {
    paintBatteryIcon(bars);
    battBarsShown = bars;
  }
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", pct);
  drawSlot(SLOT_BATT, &txtBatt, &FreeSans9pt7b, buf, "", MARK_NONE, COL_FG);
}

static void paintFooter(unsigned long now) {
  char buf[24];
  const bool live = gpsIsLive(now);
  if (live && gps.altKnown) {
    snprintf(buf, sizeof(buf), "Alt %.0f %s",
             (double)cfgAltShown(gps.altM), cfgAltUnit());
  } else {
    strcpy(buf, "Alt --");
  }
  drawSlot(SLOT_FOOT_L, &txtFootL, &FreeSans9pt7b, buf, "", MARK_NONE, COL_DIM);

  if (!sdReady) {
    drawSlot(SLOT_FOOT_R, &txtFootR, &FreeSansBold9pt7b, "", "No card", MARK_ALERT, COL_FG);
  } else if (blePhoneConnected()) {
    drawSlot(SLOT_FOOT_R, &txtFootR, &FreeSans9pt7b, "", "Phone", MARK_DOT, COL_FG);
  } else if (tripStarted && live && gps.posKnown && gps.timeKnown) {
    drawSlot(SLOT_FOOT_R, &txtFootR, &FreeSans9pt7b, "", "Recording", MARK_PILL, COL_FG);
  } else {
    drawSlot(SLOT_FOOT_R, &txtFootR, &FreeSans9pt7b, "", "", MARK_NONE, COL_DIM);
  }
}

static void drawRideScreen(float speedKmh, float distanceKm, float avgSpeedKmh,
                           float maxKmh, unsigned long elapsedMs, unsigned long moveMs,
                           unsigned long now, unsigned long lastPulseMs,
                           unsigned long revCount) {
  ensureChrome(now);

  char buf[24];
  char right[24];

  // Speed: fixed "dd.d" template so the matrix only repaints changed cells.
  // Gauge stays on km/h; hero/Avg/rows use configured units.
  float shown = cfgSpeedShown(speedKmh);
  if (shown < 0.0f) {
    shown = 0.0f;
  }
  if (shown > 99.9f) {
    shown = 99.9f;
  }
  char digits[8];
  snprintf(digits, sizeof(digits), "%4.1f", (double)shown);
  matDigits(digits, 0);

  unsigned long remainSec = 0;
  int holdLit = 0;
  uint8_t ovl = overlayState(now, &remainSec, &holdLit);
  uint8_t sendPct = 0;
  bool sending = bleSendProgress(&sendPct);

  if (ovl == OVL_FLASH) {
    drawSlot(SLOT_CAPTION, &txtCaption, &FreeSans9pt7b, overlayFlashMsg, "", MARK_NONE, COL_FG);
  } else if (ovl == OVL_HOLD) {
    snprintf(right, sizeof(right), "%lu s", remainSec);
    drawSlot(SLOT_CAPTION, &txtCaption, &FreeSans9pt7b, "Hold for new ride", right, MARK_NONE, COL_FG);
  } else if (ovl == OVL_BLE_AUTH) {
    snprintf(right, sizeof(right), "%lu s", remainSec);
    drawSlot(SLOT_CAPTION, &txtCaption, &FreeSans9pt7b, "Press to allow", right, MARK_NONE, COL_FG);
  } else {
    if (sending) {
      snprintf(right, sizeof(right), "Sending %u%%", (unsigned)sendPct);
    } else {
      snprintf(right, sizeof(right), "Avg %.1f", (double)cfgSpeedShown(avgSpeedKmh));
    }
    drawSlot(SLOT_CAPTION, &txtCaption, &FreeSans9pt7b, cfgSpeedUnit(), right, MARK_NONE, COL_DIM);
  }

  // Gauge: new-ride hold, then phone confirm, then transfer, else speed.
  int lit;
  if (ovl == OVL_HOLD || ovl == OVL_BLE_AUTH) {
    lit = holdLit;
  } else if (sending) {
    lit = ((int)sendPct * GAUGE_DOTS + 50) / 100;
  } else {
    lit = (int)(speedKmh / GAUGE_KMH_PER_DOT + 0.5f);
  }
  gaugeBase(lit);
  animTrackDistance(now, distanceKm);
  animCompose(now, speedKmh, ovl, sending, lastPulseMs, revCount, digits);
  flushMatrix();
  flushGauge();

  float distShown = cfgDistShown(distanceKm);
  if (distShown < 0.0f) {
    distShown = 0.0f;
  }
  snprintf(buf, sizeof(buf), distShown < 100.0f ? "%.2f" : "%.1f", (double)distShown);
  drawSlot(rowSlot(0), &txtRow[0], &FreeSansBold12pt7b, "", buf, MARK_NONE, COL_FG);
  formatHms(elapsedMs, buf, sizeof(buf));
  drawSlot(rowSlot(1), &txtRow[1], &FreeSansBold12pt7b, "", buf, MARK_NONE, COL_FG);
  formatHms(moveMs, buf, sizeof(buf));
  drawSlot(rowSlot(2), &txtRow[2], &FreeSansBold12pt7b, "", buf, MARK_NONE, COL_FG);
  snprintf(buf, sizeof(buf), "%.1f", (double)cfgSpeedShown(maxKmh));
  drawSlot(rowSlot(3), &txtRow[3], &FreeSansBold12pt7b, "", buf, MARK_NONE, COL_FG);

  paintBattery();
  paintStatus(now);
  paintFooter(now);
}

static void drawSplash(const char *line) {
  uiChromeDrawn = false;
  animBootPending = true;
  fillScreenDrained(COL_BG);
  display.setTextWrap(false);

  const int track = 6;
  int w = trackedWidth(&FreeSansBold18pt7b, track, "OBJECT");
  drawTracked(&FreeSansBold18pt7b, (SCREEN_W - w) / 2, 164, track, "OBJECT", COL_FG);

  display.setFont(&FreeSans9pt7b);
  int16_t bx, by;
  uint16_t bw, bh;
  display.getTextBounds(line, 0, 0, &bx, &by, &bw, &bh);
  display.setTextColor(COL_DIM);
  display.setCursor((SCREEN_W - (int)bw) / 2 - bx, 198);
  printDrained(line);

  display.setFont(NULL);
  display.setTextSize(1);
  display.setTextColor(COL_LO);
  const int vw = (int)strlen(FW_VERSION) * 6;  // built-in font cell
  display.setCursor((SCREEN_W - vw) / 2, 218);
  printDrained(FW_VERSION);
}

// Phone download of root *.GPX files. Callbacks only set flags; loop()
// does every SD read so the shared SPI bus stays on this task.
// UUIDs share one vendor base. docs/index.html speaks the same bytes.
static const char BLE_RIDE_SVC_UUID[]  = "7A1E0001-4C8B-4D2E-9F63-1B5A0C7E8D24";
static const char BLE_RIDE_CMD_UUID[]  = "7A1E0002-4C8B-4D2E-9F63-1B5A0C7E8D24";
static const char BLE_RIDE_META_UUID[] = "7A1E0003-4C8B-4D2E-9F63-1B5A0C7E8D24";
static const char BLE_RIDE_DATA_UUID[] = "7A1E0004-4C8B-4D2E-9F63-1B5A0C7E8D24";

enum {
  BLE_OP_LIST = 0x01,
  BLE_OP_GET = 0x02,
  BLE_OP_ABORT = 0x03,
  BLE_OP_DELETE = 0x04,
  BLE_OP_CFG_GET = 0x05,
  BLE_OP_CFG_SET = 0x06,
  BLE_OP_INFO = 0x07,
  BLE_OP_FW_BEGIN = 0x08,   // size u32 + crc32 u32
  BLE_OP_FW_DATA = 0x09,    // image chunk
  BLE_OP_FW_COMMIT = 0x0A
};

enum {
  BLE_META_ENTRY = 0x01,
  BLE_META_LIST_END = 0x02,
  BLE_META_START = 0x03,
  BLE_META_DONE = 0x04,
  BLE_META_AUTH_WAIT = 0x05,
  BLE_META_AUTH_OK = 0x06,
  BLE_META_DELETED = 0x07,
  BLE_META_CFG = 0x08,
  BLE_META_CFG_SAVED = 0x09,
  BLE_META_INFO = 0x0A,
  BLE_META_FW_READY = 0x0B,
  BLE_META_FW_DONE = 0x0C,
  BLE_META_FW_PROGRESS = 0x0D,
  BLE_META_ERROR = 0x7F
};

enum {
  BLE_ERR_NO_SD = 1,
  BLE_ERR_NAME = 2,
  BLE_ERR_NOT_FOUND = 3,
  BLE_ERR_IO = 4,
  BLE_ERR_ABORT = 5,
  BLE_ERR_DENIED = 6,
  BLE_ERR_CFG = 7,
  BLE_ERR_FW = 8
};

// Settings record, little-endian, after the op / meta type byte:
//   0 version, 1 flags (bit0 card present; GET only),
//   2-3 wheel_circ_mm u16, 4-5 timezone_offset_min i16, 6 backlight,
//   7 units, 8 backlight_dim, 9 max_speed_kmh, 10-11 stopped_ms u16,
//   12 ble_name length, 13-32 ble_name, 33 animations (0 off, 1 on).
static const uint8_t BLE_CFG_VERSION = 2;
static const uint16_t BLE_CFG_LEN = 34;
static const uint16_t BLE_FW_CHUNK = 244;
static const uint16_t BLE_CMD_MAX = 1 + BLE_FW_CHUNK;

// Stage the new image here, then copy it onto the running app. Must sit
// above the current sketch (about 220 KB from 0x27000) and below the
// bootloader. 0x90000 leaves ~320 KB, enough for this firmware.
static const uint32_t OTA_BANK_ADDR = 0x90000UL;
static const uint32_t OTA_BANK_MAX = 0x50000UL;
static const uint32_t OTA_APP_ADDR = 0x27000UL;
static const uint32_t OTA_PAGE = 4096UL;

enum {
  BLE_JOB_IDLE = 0,
  BLE_JOB_LIST,
  BLE_JOB_SEND
};

enum {
  BLE_AUTH_NONE = 0,
  BLE_AUTH_WAIT,
  BLE_AUTH_OK
};

static BLEService bleSvc(BLE_RIDE_SVC_UUID);
static BLECharacteristic bleCmd(BLE_RIDE_CMD_UUID, CHR_PROPS_WRITE, BLE_CMD_MAX);
static BLECharacteristic bleMeta(BLE_RIDE_META_UUID, CHR_PROPS_NOTIFY, 1 + BLE_CFG_LEN);
static BLECharacteristic bleData(BLE_RIDE_DATA_UUID, CHR_PROPS_NOTIFY, 244);

static bool bleReady = false;
static volatile uint8_t bleCmdPending = 0;
static volatile uint8_t bleLinkLost = 0;
static volatile uint8_t bleLinkUpEdge = 0;
static uint8_t bleCmdOp = 0;
static char bleCmdName[13];
static uint8_t bleCmdBody[BLE_FW_CHUNK];
static uint16_t bleCmdBodyLen = 0;
static uint8_t bleJob = BLE_JOB_IDLE;
static uint8_t bleErrorPending = 0;
static uint8_t bleDeletedPending = 0;
static uint8_t bleCfgPending = 0;
static uint8_t bleCfgSavedPending = 0;
static uint8_t bleInfoPending = 0;
static uint8_t bleFwReadyPending = 0;
static uint8_t bleFwDonePending = 0;
static uint8_t bleFwProgressPending = 0;
static uint8_t bleFwApplyPending = 0;  // commit requested; verify then notify
static uint8_t bleFwApplyNow = 0;      // FW_DONE sent; copy image next
static bool bleRenamePending = false;
static uint8_t bleAuthState = BLE_AUTH_NONE;
static unsigned long bleAuthDeadlineMs = 0;
static uint8_t bleAuthNotifyPending = 0;
static File32 bleFile;
static File32 bleDir;
static bool bleListHave = false;
static bool bleListEnd = false;
static char bleListName[13];
static uint32_t bleListSize = 0;
static char bleSendName[13];
static uint32_t bleOffset = 0;
static uint32_t bleSize = 0;
static uint32_t bleFileLimit = 0;
static bool bleFooterFromRam = false;
static bool bleStartSent = false;
static uint32_t bleCrc = 0xFFFFFFFFUL;
static uint8_t bleChunk[244];

// In-app OTA: phone writes the image into OTA_BANK_ADDR, then we copy it.
static bool otaActive = false;
static uint32_t otaSize = 0;
static uint32_t otaCrcExpect = 0;
static uint32_t otaCrc = 0xFFFFFFFFUL;
static uint32_t otaGot = 0;
static uint32_t otaPageAddr = 0;
static uint16_t otaPageUsed = 0;
static uint8_t otaPage[OTA_PAGE] __attribute__((aligned(4)));
static uint8_t otaFwChunk[BLE_FW_CHUNK];
static uint16_t otaFwChunkLen = 0;
static volatile uint8_t otaChunkPending = 0;

static bool blePhoneConnected() {
  return bleReady && bleAuthState == BLE_AUTH_OK && Bluefruit.connected() > 0;
}

static bool bleAwaitingAuth() {
  return bleReady && bleAuthState == BLE_AUTH_WAIT && Bluefruit.connected() > 0;
}

static void bleClearAuth() {
  bleAuthState = BLE_AUTH_NONE;
  bleAuthDeadlineMs = 0;
  bleAuthNotifyPending = 0;
}

static void bleBeginAuthWait(unsigned long now) {
  bleAuthState = BLE_AUTH_WAIT;
  bleAuthDeadlineMs = now + BLE_AUTH_MS;
  bleAuthNotifyPending = BLE_META_AUTH_WAIT;
  clipStart(clipAuth, now);
}

static void bleConfirmAuth(unsigned long now) {
  if (bleAuthState != BLE_AUTH_WAIT) {
    return;
  }
  bleAuthState = BLE_AUTH_OK;
  bleAuthDeadlineMs = 0;
  bleAuthNotifyPending = BLE_META_AUTH_OK;
  clipStart(clipRune, now);
  Serial.println("BLE auth ok");
}

// Remaining confirm window for the caption / draining gauge.
static bool bleAuthProgress(unsigned long now, unsigned long *remainSec, int *lit) {
  if (!bleAwaitingAuth()) {
    return false;
  }
  long remainMs = (long)(bleAuthDeadlineMs - now);
  if (remainMs < 0) {
    remainMs = 0;
  }
  *remainSec = ((unsigned long)remainMs + 999UL) / 1000UL;
  if (*remainSec == 0 && remainMs > 0) {
    *remainSec = 1;
  }
  *lit = (int)(((long)remainMs * GAUGE_DOTS + (long)BLE_AUTH_MS / 2) /
               (long)BLE_AUTH_MS);
  if (*lit < 0) {
    *lit = 0;
  }
  if (*lit > GAUGE_DOTS) {
    *lit = GAUGE_DOTS;
  }
  return true;
}

static bool bleGpxNameOk(const char *name) {
  size_t n = strlen(name);
  if (n < 5 || n > 12) {
    return false;
  }
  const char *dot = strchr(name, '.');
  if (!dot || strchr(dot + 1, '.')) {
    return false;
  }
  size_t base = (size_t)(dot - name);
  if (base < 1 || base > 8 || strcmp(dot, ".GPX") != 0) {
    return false;
  }
  for (size_t i = 0; i < base; i++) {
    char c = name[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
      return false;
    }
  }
  return true;
}

static bool bleNormalizeGpx(const char *in, char *out, size_t outlen) {
  if (!in || outlen < 13) {
    return false;
  }
  size_t n = strlen(in);
  if (n < 5 || n > 12) {
    return false;
  }
  for (size_t i = 0; i < n; i++) {
    char c = in[i];
    if (c >= 'a' && c <= 'z') {
      c = (char)(c - 'a' + 'A');
    }
    out[i] = c;
  }
  out[n] = '\0';
  return bleGpxNameOk(out);
}

static void bleCloseFiles() {
  gpsDrain();
  if (bleFile.isOpen()) {
    bleFile.close();
  }
  if (bleDir.isOpen()) {
    bleDir.close();
  }
  gpsDrain();
}

static void bleResetXfer() {
  bleCloseFiles();
  bleJob = BLE_JOB_IDLE;
  bleListHave = false;
  bleListEnd = false;
  bleListName[0] = '\0';
  bleListSize = 0;
  bleSendName[0] = '\0';
  bleOffset = 0;
  bleSize = 0;
  bleFileLimit = 0;
  bleFooterFromRam = false;
  bleStartSent = false;
  bleCrc = 0xFFFFFFFFUL;
}

static void bleQueueError(uint8_t code) {
  bleResetXfer();
  bleDeletedPending = 0;
  bleErrorPending = code;
}

static void bleStopForNewRide() {
  if (!bleReady || (bleJob == BLE_JOB_IDLE && bleErrorPending == 0)) {
    return;
  }
  bleQueueError(BLE_ERR_ABORT);
  Serial.println("BLE transfer aborted — new ride");
}

// CURRENT.GPX grows at gpxBodyEnd. Bytes below that stay put; the footer
// is sent from RAM so a later append cannot tear the downloaded file.
static uint32_t bleDownloadSize(File32 &f, const char *name, bool *footerFromRam) {
  uint32_t sz = (uint32_t)f.fileSize();
  if (footerFromRam) {
    *footerFromRam = false;
  }
  if (strcmp(name, GPX_NAME) == 0 && gpxBodyEnd > 0 && gpxBodyEnd <= sz) {
    if (footerFromRam) {
      *footerFromRam = true;
    }
    return gpxBodyEnd + (uint32_t)strlen(GPX_FOOTER);
  }
  return sz;
}

static bool bleNotifyMeta(const uint8_t *data, uint16_t len) {
  if (!bleMeta.notifyEnabled()) {
    return false;
  }
  return bleMeta.notify(data, len);
}

static bool bleNotifyNamed(uint8_t type, const char *name, uint32_t size) {
  uint8_t buf[17];
  memset(buf, 0, sizeof(buf));
  buf[0] = type;
  size_t n = strlen(name);
  if (n > 12) {
    n = 12;
  }
  memcpy(buf + 1, name, n);
  buf[13] = (uint8_t)(size & 0xff);
  buf[14] = (uint8_t)((size >> 8) & 0xff);
  buf[15] = (uint8_t)((size >> 16) & 0xff);
  buf[16] = (uint8_t)((size >> 24) & 0xff);
  return bleNotifyMeta(buf, sizeof(buf));
}

static bool bleScanOneEntry() {
  File32 ent;
  int skipped = 0;
  while (skipped < 8) {
    gpsDrain();
    if (!ent.openNext(&bleDir, O_RDONLY)) {
      gpsDrain();
      bleListEnd = true;
      return false;
    }
    char raw[32];
    memset(raw, 0, sizeof(raw));
    bool isDir = ent.isDir();
    if (!isDir) {
      ent.getName(raw, sizeof(raw));
    }
    char name[13];
    bool ok = !isDir && bleNormalizeGpx(raw, name, sizeof(name));
    uint32_t sz = 0;
    if (ok) {
      sz = bleDownloadSize(ent, name, NULL);
    }
    ent.close();
    gpsDrain();
    if (!ok) {
      skipped++;
      continue;
    }
    memcpy(bleListName, name, sizeof(bleListName));
    bleListSize = sz;
    bleListHave = true;
    return true;
  }
  return false;
}

// List end carries the card's FAT volume size in KiB, u32 little-endian.
static bool bleFinishList() {
  uint32_t kib = (uint32_t)(((uint64_t)sd.clusterCount() * sd.bytesPerCluster()) >> 10);
  uint8_t endb[5] = {BLE_META_LIST_END,
                     (uint8_t)(kib & 0xff), (uint8_t)((kib >> 8) & 0xff),
                     (uint8_t)((kib >> 16) & 0xff), (uint8_t)((kib >> 24) & 0xff)};
  if (!bleNotifyMeta(endb, sizeof(endb))) {
    return false;
  }
  bleCloseFiles();
  bleJob = BLE_JOB_IDLE;
  bleListEnd = false;
  Serial.println("BLE list end");
  return true;
}

static bool blePumpList() {
  if (!sdReady) {
    bleQueueError(BLE_ERR_NO_SD);
    return false;
  }
  if (!bleDir.isOpen()) {
    gpsDrain();
    bool opened = bleDir.open("/");
    gpsDrain();
    if (!opened) {
      bleQueueError(BLE_ERR_IO);
      return false;
    }
  }
  if (bleListEnd) {
    return bleFinishList();
  }
  if (!bleListHave) {
    bleScanOneEntry();
    if (bleListEnd) {
      return bleFinishList();
    }
    if (!bleListHave) {
      return false;
    }
  }
  if (!bleNotifyNamed(BLE_META_ENTRY, bleListName, bleListSize)) {
    return false;
  }
  bleListHave = false;
  return true;
}

static uint16_t blePayloadMax() {
  uint16_t mtu = 23;
  if (Bluefruit.connected()) {
    BLEConnection *conn = Bluefruit.Connection(Bluefruit.connHandle());
    if (conn) {
      mtu = conn->getMtu();
    }
  }
  if (mtu < 23) {
    mtu = 23;
  }
  uint16_t att = mtu - 3;
  if (att <= 4) {
    return 1;
  }
  uint16_t payload = att - 4;
  if (payload > 240) {
    payload = 240;
  }
  return payload;
}

static bool blePumpSend() {
  if (!bleStartSent) {
    if (!bleNotifyNamed(BLE_META_START, bleSendName, bleSize)) {
      return false;
    }
    bleStartSent = true;
    return true;
  }
  if (bleOffset >= bleSize) {
    uint32_t crc = ~bleCrc;
    uint8_t buf[5];
    buf[0] = BLE_META_DONE;
    buf[1] = (uint8_t)(crc & 0xff);
    buf[2] = (uint8_t)((crc >> 8) & 0xff);
    buf[3] = (uint8_t)((crc >> 16) & 0xff);
    buf[4] = (uint8_t)((crc >> 24) & 0xff);
    if (!bleNotifyMeta(buf, sizeof(buf))) {
      return false;
    }
    Serial.print("BLE sent ");
    Serial.print(bleSendName);
    Serial.print(" bytes=");
    Serial.print(bleSize);
    Serial.print(" crc=");
    Serial.println(crc, HEX);
    bleResetXfer();
    return true;
  }

  uint16_t payload = blePayloadMax();
  uint32_t remain = bleSize - bleOffset;
  if ((uint32_t)payload > remain) {
    payload = (uint16_t)remain;
  }

  uint8_t *dst = bleChunk + 4;
  uint16_t filled = 0;
  if (bleOffset < bleFileLimit) {
    uint32_t avail = bleFileLimit - bleOffset;
    uint16_t n = payload;
    if ((uint32_t)n > avail) {
      n = (uint16_t)avail;
    }
    gpsDrain();
    bool seekOk = bleFile.seekSet(bleOffset);
    int got = seekOk ? bleFile.read(dst, n) : -1;
    gpsDrain();
    if (got != (int)n) {
      bleQueueError(BLE_ERR_IO);
      return false;
    }
    filled = n;
  }
  if (filled < payload) {
    uint32_t footerAt = (bleOffset + filled) - bleFileLimit;
    uint16_t n = (uint16_t)(payload - filled);
    if (!bleFooterFromRam || footerAt + n > (uint32_t)strlen(GPX_FOOTER)) {
      bleQueueError(BLE_ERR_IO);
      return false;
    }
    memcpy(dst + filled, GPX_FOOTER + footerAt, n);
    filled = (uint16_t)(filled + n);
  }

  bleChunk[0] = (uint8_t)(bleOffset & 0xff);
  bleChunk[1] = (uint8_t)((bleOffset >> 8) & 0xff);
  bleChunk[2] = (uint8_t)((bleOffset >> 16) & 0xff);
  bleChunk[3] = (uint8_t)((bleOffset >> 24) & 0xff);

  if (!bleData.notifyEnabled() || !bleData.notify(bleChunk, (uint16_t)(4 + filled))) {
    return false;
  }
  bleCrc = crc32Update(bleCrc, dst, filled);
  bleOffset += filled;
  return true;
}

static void bleBeginList() {
  bleResetXfer();
  bleErrorPending = 0;
  bleDeletedPending = 0;
  if (!sdReady) {
    bleQueueError(BLE_ERR_NO_SD);
    return;
  }
  bleJob = BLE_JOB_LIST;
  Serial.println("BLE list");
}

static void bleBeginGet(const char *rawName) {
  char name[13];
  if (!bleNormalizeGpx(rawName, name, sizeof(name))) {
    bleQueueError(BLE_ERR_NAME);
    return;
  }
  if (!sdReady) {
    bleQueueError(BLE_ERR_NO_SD);
    return;
  }
  bleResetXfer();
  bleErrorPending = 0;
  bleDeletedPending = 0;
  gpsDrain();
  bool opened = bleFile.open(name, O_RDONLY);
  gpsDrain();
  if (!opened) {
    bleQueueError(BLE_ERR_NOT_FOUND);
    return;
  }
  bool footer = false;
  bleSize = bleDownloadSize(bleFile, name, &footer);
  bleFooterFromRam = footer;
  bleFileLimit = footer ? gpxBodyEnd : bleSize;
  memcpy(bleSendName, name, sizeof(bleSendName));
  bleCrc = 0xFFFFFFFFUL;
  bleOffset = 0;
  bleStartSent = false;
  bleJob = BLE_JOB_SEND;
  Serial.print("BLE get ");
  Serial.print(name);
  Serial.print(" bytes=");
  Serial.println(bleSize);
}

// Archives only — CURRENT.GPX stays on the card while recording.
static void bleBeginDelete(const char *rawName) {
  char name[13];
  if (!bleNormalizeGpx(rawName, name, sizeof(name))) {
    bleQueueError(BLE_ERR_NAME);
    return;
  }
  if (strcmp(name, GPX_NAME) == 0) {
    bleQueueError(BLE_ERR_NAME);
    Serial.println("BLE delete denied — CURRENT.GPX");
    return;
  }
  if (!sdReady) {
    bleQueueError(BLE_ERR_NO_SD);
    return;
  }
  bleResetXfer();
  bleErrorPending = 0;
  bleDeletedPending = 0;
  gpsDrain();
  bool exists = sd.exists(name);
  gpsDrain();
  if (!exists) {
    bleQueueError(BLE_ERR_NOT_FOUND);
    return;
  }
  gpsDrain();
  bool removed = sd.remove(name);
  gpsDrain();
  if (!removed) {
    bleQueueError(BLE_ERR_IO);
    Serial.print("BLE delete failed ");
    Serial.println(name);
    return;
  }
  bleDeletedPending = 1;
  Serial.print("BLE deleted ");
  Serial.println(name);
}

static void bleCfgPack(uint8_t *b) {
  memset(b, 0, BLE_CFG_LEN);
  uint16_t wheel = (uint16_t)(cfg.wheelCircMm + 0.5f);
  uint16_t tz = (uint16_t)cfg.timezoneOffsetMin;
  uint16_t stopped = (uint16_t)cfg.stoppedMs;
  size_t n = strlen(cfg.bleName);
  b[0] = BLE_CFG_VERSION;
  b[1] = sdReady ? 0x01 : 0x00;
  b[2] = (uint8_t)(wheel & 0xff);
  b[3] = (uint8_t)(wheel >> 8);
  b[4] = (uint8_t)(tz & 0xff);
  b[5] = (uint8_t)(tz >> 8);
  b[6] = cfg.backlight;
  b[7] = cfg.units;
  b[8] = cfg.backlightDim;
  b[9] = (uint8_t)(cfg.maxSpeedKmh + 0.5f);
  b[10] = (uint8_t)(stopped & 0xff);
  b[11] = (uint8_t)(stopped >> 8);
  b[12] = (uint8_t)n;
  memcpy(b + 13, cfg.bleName, n);
  b[33] = cfg.animations;
}

// All or nothing: one bad field rejects the whole record.
static bool bleCfgUnpack(const uint8_t *b, Cfg *out) {
  if (b[0] != BLE_CFG_VERSION) {
    return false;
  }
  float wheel = (float)(uint16_t)(b[2] | (b[3] << 8));
  long tz = (int16_t)(uint16_t)(b[4] | (b[5] << 8));
  long stopped = (long)(uint16_t)(b[10] | (b[11] << 8));
  float maxKmh = (float)b[9];
  uint8_t n = b[12];
  if (!cfgWheelOk(wheel) || !cfgTimezoneOk(tz) || b[6] >= BL_MODE_COUNT
      || b[7] > UNITS_IMPERIAL || !cfgDimOk(b[8]) || !cfgMaxSpeedOk(maxKmh)
      || !cfgStoppedOk(stopped) || !cfgBleNameOk((const char *)b + 13, n)
      || b[33] > 1) {
    return false;
  }
  out->wheelCircMm = wheel;
  out->timezoneOffsetMin = (int16_t)tz;
  out->backlight = b[6];
  out->units = b[7];
  out->backlightDim = b[8];
  out->maxSpeedKmh = maxKmh;
  out->stoppedMs = (unsigned long)stopped;
  memcpy(out->bleName, b + 13, n);
  out->bleName[n] = '\0';
  out->animations = b[33];
  return true;
}

static void bleBeginCfgSet(const uint8_t *body, uint16_t len) {
  Cfg next = cfg;
  if (len != BLE_CFG_LEN || !bleCfgUnpack(body, &next)) {
    bleQueueError(BLE_ERR_CFG);
    Serial.println("BLE settings rejected");
    return;
  }
  if (!sdReady) {
    bleQueueError(BLE_ERR_NO_SD);
    return;
  }
  if (!configSave(next)) {
    bleQueueError(BLE_ERR_IO);
    return;
  }
  Cfg old = cfg;
  cfg = next;
  configApplyChange(old);
  if (strcmp(cfg.bleName, old.bleName) != 0) {
    Bluefruit.setName(cfg.bleName);
    bleRenamePending = true;
  }
  btnFlash("Settings saved", millis());
  bleCfgSavedPending = 1;
  bleCfgPending = 1;
  Serial.println("BLE settings saved");
  configLog();
}

#if defined(NRF52840_XXAA)
// Bluefruit's SOC task already calls sd_evt_get() and forwards flash
// completion to flash_nrf5x_event_cb. Waiting on sd_evt_get() here races
// that task: the first 4 KB page flush never returns, loop() stalls, and a
// retry cannot even begin. S140 also asserts on a whole-page sd_flash_write;
// flash_nrf5x writes half-pages and waits on the same callback.
// Do not erase here — flush() erases once if the page differs. A second
// erase doubles the radio-blocked window and trips the 2 s BLE timeout.
static bool otaFlashWritePage(uint32_t addr, const uint8_t *data) {
  if (((uint32_t)data & 3u) != 0) {
    return false;
  }
  if (flash_nrf5x_write(addr, data, OTA_PAGE) != (int)OTA_PAGE) {
    return false;
  }
  flash_nrf5x_flush();
  return true;
}

// Runs from RAM after SoftDevice is off: copy staged image onto the app and reset.
static void otaCopyAndReset(uint32_t src, uint32_t dst, uint32_t len) {
  __disable_irq();
  for (uint32_t off = 0; off < len; off += OTA_PAGE) {
    uint32_t page = dst + off;
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
    }
    NRF_NVMC->ERASEPAGE = page;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
    }

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
    }
    uint32_t n = len - off;
    if (n > OTA_PAGE) {
      n = OTA_PAGE;
    }
    for (uint32_t i = 0; i < n; i += 4) {
      *(volatile uint32_t *)(page + i) = *(const uint32_t *)(src + off + i);
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
      }
    }
  }
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
  }
  NVIC_SystemReset();
}
#else
static bool otaFlashWritePage(uint32_t, const uint8_t *) { return true; }
static void otaCopyAndReset(uint32_t, uint32_t, uint32_t) {}
#endif

static void otaCancel(void) {
  otaActive = false;
  otaChunkPending = 0;
  otaFwChunkLen = 0;
  otaPageUsed = 0;
  bleFwReadyPending = 0;
  bleFwDonePending = 0;
  bleFwProgressPending = 0;
  bleFwApplyPending = 0;
  bleFwApplyNow = 0;
}

static bool otaFlushPage(bool pad) {
  if (otaPageUsed == 0) {
    return true;
  }
  if (!pad && otaPageUsed != OTA_PAGE) {
    return true;
  }
  while (otaPageUsed < OTA_PAGE) {
    otaPage[otaPageUsed++] = 0xFF;
  }
  if (!otaFlashWritePage(otaPageAddr, otaPage)) {
    return false;
  }
  otaPageAddr += OTA_PAGE;
  otaPageUsed = 0;
  return true;
}

static void otaTuneLink(void) {
  BLEConnection *conn = Bluefruit.Connection(Bluefruit.connHandle());
  if (!conn) {
    return;
  }
  // Flash holds the radio for tens of ms. The default 2 s supervision
  // timeout drops the phone around 90% of a ~200 KB image; 8 s covers a
  // stalled page. 40 ms interval leaves bigger gaps for sd_flash_*.
  conn->requestConnectionParameter(MS100TO125(40), 0, 800);
}

static bool otaBegin(uint32_t size, uint32_t crc) {
  if (size < 256 || size > OTA_BANK_MAX || (size & 3u) != 0) {
    return false;
  }
#if defined(NRF52840_XXAA)
  flash_nrf5x_flush();
#endif
  otaCancel();
  otaTuneLink();
  otaActive = true;
  otaSize = size;
  otaCrcExpect = crc;
  otaCrc = 0xFFFFFFFFUL;
  otaGot = 0;
  otaPageAddr = OTA_BANK_ADDR;
  otaPageUsed = 0;
  return true;
}

static bool otaAcceptChunk(const uint8_t *data, uint16_t len) {
  if (!otaActive || len == 0) {
    return false;
  }
  if ((uint32_t)len > otaSize - otaGot) {
    return false;
  }
  otaCrc = crc32Update(otaCrc, data, len);
  uint16_t off = 0;
  while (off < len) {
    uint16_t n = (uint16_t)(OTA_PAGE - otaPageUsed);
    uint16_t left = (uint16_t)(len - off);
    if (n > left) {
      n = left;
    }
    memcpy(otaPage + otaPageUsed, data + off, n);
    otaPageUsed = (uint16_t)(otaPageUsed + n);
    off = (uint16_t)(off + n);
    otaGot += n;
    if (otaPageUsed == OTA_PAGE) {
      if (!otaFlushPage(false)) {
        return false;
      }
    }
  }
  return true;
}

static bool otaCommit(void) {
  if (!otaActive || otaGot != otaSize) {
    return false;
  }
  if (!otaFlushPage(true)) {
    return false;
  }
  if ((~otaCrc) != otaCrcExpect) {
    return false;
  }
  return true;
}

// SoftDevice off, then run the flash copy from RAM so erasing the app is safe.
static void otaApply(uint32_t revCount, uint32_t elapsedMsNow) {
  Serial.println("BLE firmware apply — copying staged image");
  if (sdReady) {
    if (gpxFile.isOpen()) {
      gpxFinalize();
      gpxFile.close();
    }
    tripDatSave(revCount, elapsedMsNow);
  }
  drawSplash("Updating");
  delay(50);
#if defined(NRF52840_XXAA)
  if (Bluefruit.connected()) {
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.disconnect(Bluefruit.connHandle());
    delay(200);
  }
  sd_softdevice_disable();
  static uint8_t ramFn[512];
  uint32_t srcFn = (uint32_t)&otaCopyAndReset;
  memcpy(ramFn, (const void *)(srcFn & ~1u), sizeof(ramFn));
  typedef void (*ota_fn_t)(uint32_t, uint32_t, uint32_t);
  ota_fn_t fn = (ota_fn_t)(((uint32_t)ramFn) | 1u);
  fn(OTA_BANK_ADDR, OTA_APP_ADDR, otaSize);
#else
  (void)revCount;
  (void)elapsedMsNow;
#endif
}

static void bleTakeCommand() {
  uint8_t op;
  char name[13];
  uint8_t body[BLE_FW_CHUNK];
  uint16_t bodyLen;
  noInterrupts();
  op = bleCmdOp;
  memcpy(name, bleCmdName, sizeof(name));
  memcpy(body, bleCmdBody, sizeof(body));
  bodyLen = bleCmdBodyLen;
  bleCmdPending = 0;
  interrupts();

  if (op == BLE_OP_ABORT) {
    if (otaActive) {
      otaCancel();
      Serial.println("BLE firmware update aborted");
    }
    if (bleJob != BLE_JOB_IDLE || bleErrorPending != 0 || bleDeletedPending != 0) {
      bleDeletedPending = 0;
      bleQueueError(BLE_ERR_ABORT);
      Serial.println("BLE abort");
    }
    return;
  }
  if (bleAuthState != BLE_AUTH_OK) {
    bleQueueError(BLE_ERR_DENIED);
    Serial.println("BLE denied — waiting for confirm");
    return;
  }
  if (op == BLE_OP_LIST) {
    bleBeginList();
    return;
  }
  if (op == BLE_OP_GET) {
    bleBeginGet(name);
    return;
  }
  if (op == BLE_OP_DELETE) {
    bleBeginDelete(name);
    return;
  }
  if (op == BLE_OP_CFG_GET) {
    bleCfgPending = 1;
    return;
  }
  if (op == BLE_OP_CFG_SET) {
    bleBeginCfgSet(body, bodyLen);
    return;
  }
  if (op == BLE_OP_INFO) {
    bleInfoPending = 1;
    return;
  }
  if (op == BLE_OP_FW_BEGIN) {
    if (bodyLen < 8) {
      bleQueueError(BLE_ERR_FW);
      return;
    }
    uint32_t size = (uint32_t)body[0] | ((uint32_t)body[1] << 8) |
                    ((uint32_t)body[2] << 16) | ((uint32_t)body[3] << 24);
    uint32_t crc = (uint32_t)body[4] | ((uint32_t)body[5] << 8) |
                   ((uint32_t)body[6] << 16) | ((uint32_t)body[7] << 24);
    if (!otaBegin(size, crc)) {
      bleQueueError(BLE_ERR_FW);
      return;
    }
    bleFwReadyPending = 1;
    Serial.print("BLE firmware begin size=");
    Serial.println(size);
    return;
  }
  if (op == BLE_OP_FW_DATA) {
    // Handled in bleOnWrite so chunks are not queued behind loop().
    return;
  }
  if (op == BLE_OP_FW_COMMIT) {
    if (!otaActive) {
      bleQueueError(BLE_ERR_FW);
      return;
    }
    bleFwApplyPending = 1;
    return;
  }
  bleQueueError(BLE_ERR_NAME);
}

static bool bleActive() {
  return bleReady && (bleJob != BLE_JOB_IDLE || bleErrorPending != 0 || bleDeletedPending != 0 ||
                      otaActive || otaChunkPending);
}

// Percent of the file sent to the phone, while a download runs.
static bool bleSendProgress(uint8_t *pct) {
  if (!bleReady || bleJob != BLE_JOB_SEND || bleSize == 0) {
    return false;
  }
  uint32_t p = (uint32_t)((uint64_t)bleOffset * 100u / bleSize);
  if (p > 100) {
    p = 100;
  }
  *pct = (uint8_t)p;
  return true;
}

static bool bleService() {
  if (!bleReady) {
    return false;
  }

  unsigned long now = millis();

  if (bleLinkUpEdge) {
    bleLinkUpEdge = 0;
    bleBeginAuthWait(now);
    Serial.println("BLE connected — waiting for confirm");
  }
  if (bleLinkLost || ((bleJob != BLE_JOB_IDLE || otaActive) && !Bluefruit.connected())) {
    bleLinkLost = 0;
    noInterrupts();
    bleCmdPending = 0;
    otaChunkPending = 0;
    interrupts();
    bleErrorPending = 0;
    bleDeletedPending = 0;
    bleCfgPending = 0;
    bleCfgSavedPending = 0;
    bleFwReadyPending = 0;
    bleFwDonePending = 0;
    if (otaActive && !bleFwApplyPending && !bleFwApplyNow) {
      otaCancel();
    }
    bleResetXfer();
    bleClearAuth();
    Serial.println("BLE disconnected");
    if (bleRenamePending) {
      bleRenamePending = false;
      Bluefruit.Advertising.stop();
      Bluefruit.ScanResponse.clearData();
      Bluefruit.ScanResponse.addName();
      Bluefruit.Advertising.start(0);
      Serial.print("BLE advertising ");
      Serial.println(cfg.bleName);
    }
    return false;
  }

  if (bleAuthState == BLE_AUTH_WAIT) {
    if ((long)(now - bleAuthDeadlineMs) >= 0) {
      Serial.println("BLE auth timeout — disconnect");
      bleClearAuth();
      Bluefruit.disconnect(Bluefruit.connHandle());
      return false;
    }
  }

  if (bleAuthNotifyPending != 0) {
    uint8_t meta = bleAuthNotifyPending;
    if (bleNotifyMeta(&meta, 1)) {
      bleAuthNotifyPending = 0;
      return true;
    }
  }

  if (bleCmdPending) {
    bleTakeCommand();
    return true;
  }
  if (otaChunkPending) {
    uint8_t chunk[BLE_FW_CHUNK];
    uint16_t n;
    noInterrupts();
    n = otaFwChunkLen;
    memcpy(chunk, otaFwChunk, n);
    otaChunkPending = 0;
    interrupts();
    if (!otaAcceptChunk(chunk, n)) {
      otaCancel();
      bleQueueError(BLE_ERR_FW);
    } else {
      bleFwProgressPending = 1;
    }
    return true;
  }
  if (bleErrorPending) {
    uint8_t buf[2] = {BLE_META_ERROR, bleErrorPending};
    if (!bleNotifyMeta(buf, sizeof(buf))) {
      return false;
    }
    Serial.print("BLE error ");
    Serial.println(bleErrorPending);
    bleErrorPending = 0;
    return true;
  }
  if (bleDeletedPending) {
    uint8_t meta = BLE_META_DELETED;
    if (!bleNotifyMeta(&meta, 1)) {
      return false;
    }
    bleDeletedPending = 0;
    return true;
  }
  if (bleCfgSavedPending) {
    uint8_t meta = BLE_META_CFG_SAVED;
    if (!bleNotifyMeta(&meta, 1)) {
      return false;
    }
    bleCfgSavedPending = 0;
    return true;
  }
  if (bleCfgPending) {
    uint8_t buf[1 + BLE_CFG_LEN];
    buf[0] = BLE_META_CFG;
    bleCfgPack(buf + 1);
    if (!bleNotifyMeta(buf, sizeof(buf))) {
      return false;
    }
    bleCfgPending = 0;
    return true;
  }
  if (bleInfoPending) {
    uint8_t buf[1 + BLE_CFG_LEN];
    size_t n = strlen(FW_VERSION);
    if (n > BLE_CFG_LEN) {
      n = BLE_CFG_LEN;
    }
    buf[0] = BLE_META_INFO;
    memcpy(buf + 1, FW_VERSION, n);
    if (!bleNotifyMeta(buf, (uint16_t)(1 + n))) {
      return false;
    }
    bleInfoPending = 0;
    return true;
  }
  if (bleFwReadyPending) {
    uint8_t meta = BLE_META_FW_READY;
    if (!bleNotifyMeta(&meta, 1)) {
      return false;
    }
    bleFwReadyPending = 0;
    return true;
  }
  if (bleFwProgressPending) {
    uint8_t buf[5];
    buf[0] = BLE_META_FW_PROGRESS;
    buf[1] = (uint8_t)(otaGot);
    buf[2] = (uint8_t)(otaGot >> 8);
    buf[3] = (uint8_t)(otaGot >> 16);
    buf[4] = (uint8_t)(otaGot >> 24);
    if (!bleNotifyMeta(buf, sizeof(buf))) {
      return false;
    }
    bleFwProgressPending = 0;
    return true;
  }
  if (bleFwApplyPending) {
    bleFwApplyPending = 0;
    if (!otaCommit()) {
      otaCancel();
      bleQueueError(BLE_ERR_FW);
      Serial.println("BLE firmware commit failed");
      return true;
    }
    bleFwDonePending = 1;
    return true;
  }
  if (bleFwDonePending) {
    uint8_t meta = BLE_META_FW_DONE;
    if (!bleNotifyMeta(&meta, 1)) {
      return false;
    }
    bleFwDonePending = 0;
    bleFwApplyNow = 1;
    return true;
  }
  if (bleJob == BLE_JOB_LIST) {
    return blePumpList();
  }
  if (bleJob == BLE_JOB_SEND) {
    return blePumpSend();
  }
  return false;
}

static void bleOnWrite(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len) {
  (void)conn_hdl;
  (void)chr;
  if (len < 1 || len > BLE_CMD_MAX) {
    return;
  }
  uint8_t op = data[0];
  char name[13];
  memset(name, 0, sizeof(name));
  if (op == BLE_OP_GET || op == BLE_OP_DELETE) {
    if (len > 1) {
      uint16_t n = (uint16_t)(len - 1);
      if (n > 12) {
        n = 12;
      }
      memcpy(name, data + 1, n);
    }
  }
  // Queue one chunk for loop(). SoftDevice flash APIs must not run inside
  // the BLE write callback.
  if (op == BLE_OP_FW_DATA) {
    if (!otaActive || len < 2 || otaChunkPending) {
      otaCancel();
      bleErrorPending = BLE_ERR_FW;
      return;
    }
    uint16_t n = (uint16_t)(len - 1);
    memcpy(otaFwChunk, data + 1, n);
    otaFwChunkLen = n;
    otaChunkPending = 1;
    return;
  }
  uint16_t bodyLen = 0;
  noInterrupts();
  if (op == BLE_OP_CFG_SET || op == BLE_OP_FW_BEGIN) {
    bodyLen = (uint16_t)(len - 1);
    if (bodyLen > sizeof(bleCmdBody)) {
      bodyLen = sizeof(bleCmdBody);
    }
    memcpy(bleCmdBody, data + 1, bodyLen);
  }
  bleCmdOp = op;
  memcpy(bleCmdName, name, sizeof(bleCmdName));
  bleCmdBodyLen = bodyLen;
  bleCmdPending = 1;
  interrupts();
}

static void bleOnConnect(uint16_t conn_hdl) {
  BLEConnection *conn = Bluefruit.Connection(conn_hdl);
  if (conn) {
    conn->requestMtuExchange(247);
    conn->requestPHY(BLE_GAP_PHY_2MBPS);
  }
  bleLinkUpEdge = 1;
}

static void bleOnDisconnect(uint16_t conn_hdl, uint8_t reason) {
  (void)conn_hdl;
  (void)reason;
  bleLinkLost = 1;
}

static void bleStart() {
  // Keep the speed LED. Bluefruit's connect blink uses that pin.
  Bluefruit.autoConnLed(false);
  // MTU 247 with a short event. BANDWIDTH_MAX's event length does not fit
  // the SoftDevice RAM this board reserves.
  Bluefruit.configPrphConn(247, 6, 2, 1);
  if (!Bluefruit.begin(1, 0)) {
    Serial.println("BLE begin failed");
    return;
  }
  bleReady = true;
  Bluefruit.setTxPower(4);
  Bluefruit.setName(cfg.bleName);
  Bluefruit.Periph.setConnectCallback(bleOnConnect);
  Bluefruit.Periph.setDisconnectCallback(bleOnDisconnect);
  Bluefruit.Periph.setConnIntervalMS(15, 30);
  Bluefruit.Periph.setConnSupervisionTimeoutMS(8000);

  bleCmd.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  bleCmd.setWriteCallback(bleOnWrite, true);
  bleMeta.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bleData.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bleSvc.begin();
  bleCmd.begin();
  bleMeta.begin();
  bleData.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleSvc);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  if (!Bluefruit.Advertising.start(0)) {
    Serial.println("BLE advertise failed");
    return;
  }
  Serial.print("BLE advertising ");
  Serial.println(cfg.bleName);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_REED, INPUT_PULLUP);
  analogReadResolution(12);
  pinMode(PIN_LCD_BL, OUTPUT);
  backlightApply();
  pinMode(PIN_LCD_CS, OUTPUT);
  digitalWrite(PIN_LCD_CS, HIGH);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  // Core auto-starts Serial1 at 115200 before setup(); stop it so the 64-byte
  // RX ring is not already overflowing unread GPS traffic.
  if (Serial1) {
    Serial1.end();
  }

  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("XIAO cycling computer — ILI9341 + reed D0 + btn/batt A4 + GPS Serial1 + SD D5");
  Serial.print("Firmware ");
  Serial.println(FW_VERSION);

  // Hardware SPI: SCK=D8, MOSI=D10, MISO=D9. LCD CS=D1, SD CS=D5.
  // RST is tied to 3.3V; begin() sends a software reset. Backlight is D3.
  display.begin(32000000);
  display.setRotation(0);
  display.invertDisplay(true);
  display.setTextWrap(false);
  drawSplash("Checking card");

  sdBeginShared();
  const char *sdLine = "No card";
  if (sdReady) {
    configEnsureFile();
    configLoad();
    sdLine = tripResumeSd();
  } else {
    configLoad();
    sdLine = "No card";
  }
  drawSplash(sdLine);
  delay(1000);

  gps.src = "-";
  Serial1.begin(GPS_BAUD);
  gpsConfigure();
  gpsInjectAid();
  gpsPollNavPvt();
  Serial.println("GPS Serial1 115200 on D6/D7 — 1 Hz NAV-PVT/GGA");

  lastLoopMs = millis();
  if (tripStarted && elapsedAnchorMs == 0) {
    elapsedAnchorMs = lastLoopMs;
  }

  // Attach after display is up so early glitches don't race an empty UI.
  attachInterrupt(digitalPinToInterrupt(PIN_REED), reedIsr, FALLING);

  bleStart();

  Serial.println(tripStarted ? "Ready — resuming trip" : "Ready — waiting for wheel pulses");
}

void loop() {
  gpsPoll();

  // Snapshot ISR-owned state with interrupts briefly off (avoid torn reads).
  noInterrupts();
  unsigned long revCount     = g_revCount;
  unsigned long lastPulseMs  = g_lastPulseMs;
  unsigned long prevPulseMs  = g_prevPulseMs;
  unsigned long olderPulseMs = g_olderPulseMs;
  interrupts();

  unsigned long now = millis();
  btnHandle(now, lastPulseMs);

  // New-ride may have cleared the ISR counters; snapshot again.
  noInterrupts();
  revCount     = g_revCount;
  lastPulseMs  = g_lastPulseMs;
  prevPulseMs  = g_prevPulseMs;
  olderPulseMs = g_olderPulseMs;
  interrupts();

  float speedKmh = 0.0f;

  // Need two pulses this session (prevPulseMs != 0) so a restored revCount
  // cannot pair with a zero timestamp and invent a huge speed.
  unsigned long minRevMs = g_minRevMs;
  if (revCount >= 2 && prevPulseMs != 0 && (now - lastPulseMs) < cfg.stoppedMs) {
    unsigned long dtMs = lastPulseMs - prevPulseMs;
    // Same ceiling as the ISR: ignore intervals that imply > max_speed_kmh.
    if (dtMs >= minRevMs) {
      // mm/ms -> km/h: (mm/ms) * (3600 s/h) / 1e6 (mm/km) = * 3.6
      float raw = (cfg.wheelCircMm / (float)dtMs) * 3.6f;
      if (raw <= cfg.maxSpeedKmh) {
        speedKmh = raw;
      }
    }
  }

  float distanceKm = (revCount * cfg.wheelCircMm) / 1000000.0f;

  // Trip starts on the first reed pulse.
  if (revCount >= 1 && !tripStarted) {
    tripStarted = true;
    elapsedBaseMs = 0;
    elapsedAnchorMs = lastPulseMs;
  }

  unsigned long loopDt = now - lastLoopMs;
  if (speedKmh > 0.0f) {
    movingMs += loopDt;
  }
  lastLoopMs = now;

  // Max needs two consecutive intervals in agreement so one spurious pulse
  // (pothole, magnet wobble, loose wire) cannot lock a high reading.
  if (speedKmh > maxSpeedKmh && speedKmh <= cfg.maxSpeedKmh
      && revCount >= 3 && olderPulseMs != 0) {
    unsigned long dtNew = lastPulseMs - prevPulseMs;
    unsigned long dtOld = prevPulseMs - olderPulseMs;
    if (dtNew >= minRevMs && dtOld >= minRevMs) {
      unsigned long dtLo = (dtNew < dtOld) ? dtNew : dtOld;
      unsigned long dtHi = (dtNew > dtOld) ? dtNew : dtOld;
      if (dtHi > 0 && (float)dtLo >= MAX_CONFIRM_RATIO * (float)dtHi) {
        animNoteMax(now, speedKmh);
        maxSpeedKmh = speedKmh;
      }
    }
  }

  // Avg = distance / moving time (stops do not dilute Avg).
  float avgSpeedKmh = 0.0f;
  if (movingMs > 0) {
    avgSpeedKmh = distanceKm / (movingMs / 3600000.0f);
  }

  unsigned long elapsedMs = 0;
  if (tripStarted) {
    elapsedMs = elapsedBaseMs + (now - elapsedAnchorMs);
  }

  if (!otaActive) {
    tripLogGps(now, (uint32_t)revCount, (uint32_t)elapsedMs);
    tripMaybeSave(now, (uint32_t)revCount, (uint32_t)elapsedMs);
    drawRideScreen(speedKmh, distanceKm, avgSpeedKmh, maxSpeedKmh, elapsedMs, movingMs, now,
                   lastPulseMs, revCount);
    gpsDrain();
  }

  digitalWrite(LED_BUILTIN, (speedKmh > 0.05f) ? LOW : HIGH);

  bool bleSent = bleService();
  if (bleFwApplyNow) {
    bleFwApplyNow = 0;
    otaApply((uint32_t)revCount, (uint32_t)elapsedMs);
    return;  // not reached: the board resets into the new image
  }
  if (otaActive) {
    yield();
  } else if (bleSent) {
    gpsDrain();
  } else if (bleActive()) {
    gpsWait(5);
  } else {
    gpsWait(50);
  }
}
