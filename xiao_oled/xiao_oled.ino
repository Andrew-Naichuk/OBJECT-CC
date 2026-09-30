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
 * Tact button:
 *   One side -> D4, other side -> GND
 *   INPUT_PULLUP; pressed = LOW.
 *   Short press (release before 2 s): backlight bright -> dim -> off.
 *   Hold 4 s while stopped: save the ride and start a new one.
 *
 * BLE file download (no extra wiring):
 *   Advertises as "OBJECT-001". Double short-press while stopped opens a
 *   transfer window; tools/index.html then lists root *.GPX files and saves
 *   them on the phone. Checkpoints (TRIP_A/TRIP_B) are not offered. Open
 *   that page over HTTPS (Android Chrome, or a Web Bluetooth browser on iPhone).
 *
 * Libraries (Arduino Library Manager):
 *   Adafruit ILI9341, Adafruit GFX Library, Adafruit BusIO
 * SdFat and Bluefruit are bundled with the Seeeduino nRF52 core
 * (do not install SdFat 2.3.x).
 *
 * Wheel: 700x32C (ISO 32-622) -> circumference 2155 mm.
 *
 * TFT layout (portrait 240x320), monochrome to match the OBJECT ride page:
 *   OBJECT | GPS state, km/h | Avg, dot-matrix speed, 24-dot gauge,
 *   Distance / Time / Moving / Max rows, Altitude | card, phone, recording.
 * The gauge shows speed (2 km/h a dot), the new-ride hold, or a phone
 * download in progress.
 * Rotation 0. If the image is upside down relative to the pin header, use 2.
 */

#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <SdFat.h>
#include <bluefruit.h>
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

static const int PIN_LCD_CS = D1;
static const int PIN_LCD_DC = D2;
static const int PIN_LCD_BL = D3;
static const int PIN_REED   = D0;
static const int PIN_BTN    = D4;
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

// Portrait stack, top to bottom.
static const int STATUS_BASE = 19;   // wordmark baseline
static const int CAPTION_Y   = 34;
static const int CAPTION_H   = 20;
static const int HERO_Y      = 60;   // speed matrix top
static const int HERO_PITCH  = 10;   // matrix cell
static const int HERO_DOT    = 8;    // lit square inside the cell
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

// Circumference of 700x32C (32-622), millimetres.
static const float WHEEL_CIRC_MM = 2155.0f;

// Instantaneous speeds above this are treated as reed noise / bounce.
// (Real road descents rarely top this; 200–400 km/h spikes are not.)
static const float MAX_SPEED_KMH = 100.0f;

// Contact-bounce floor. Real minimum gap is also derived from MAX_SPEED_KMH
// so a double-fire after the floor cannot invent absurd km/h and lock Max.
static const unsigned long DEBOUNCE_MS = 15;
static const unsigned long MIN_REV_MS =
    (unsigned long)((WHEEL_CIRC_MM * 3.6f) / MAX_SPEED_KMH + 0.5f);

// If no pulse for this long, treat the bike as stopped.
static const unsigned long STOPPED_MS = 3000;

static const unsigned long BTN_DEBOUNCE_MS = 30;
static const unsigned long BTN_ARM_MS = 2000;
static const unsigned long BTN_EXEC_MS = 4000;
static const unsigned long BTN_FLASH_MS = 1500;

enum {
  BL_BRIGHT = 0,
  BL_DIM,
  BL_OFF,
  BL_MODE_COUNT
};
// Dim is ~16% so night use keeps the digits readable and cuts most of the
// backlight current. Off leaves the panel updating with the LEDs dark.
static const uint8_t BL_DUTY[BL_MODE_COUNT] = {255, 40, 0};

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
static const unsigned AVG_MIN_REVS = 5;
static const unsigned long AVG_MIN_MOVING_MS = 10000;
static const unsigned long BLE_XFER_WINDOW_MS = 90000;
static const unsigned long BTN_DOUBLE_MS = 600;

static const char GPX_NAME[] = "CURRENT.GPX";
static const char DAT_NAME_A[] = "TRIP_A.DAT";
static const char DAT_NAME_B[] = "TRIP_B.DAT";
static const char DAT_NAME_LEGACY[] = "TRIP.DAT";
static const char FINISH_NAME[] = "FINISH.DAT";
static const uint32_t TRIP_MAGIC = 0x50495254UL;  // "TRIP"
static const uint32_t FINISH_MAGIC = 0x48534E46UL;  // "FNSH"
static const uint16_t TRIP_VERSION = 3;
static const uint16_t TRIP_VERSION_V2 = 2;
static const uint16_t TRIP_VERSION_LEGACY = 1;
static const uint16_t FINISH_VERSION = 1;

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
volatile unsigned long g_lastIsrMs = 0;      // for debounce only
volatile unsigned long g_movingAccMs = 0;    // valid pulse intervals waiting for loop()

// --- Trip state (loop only) ---
static bool tripStarted = false;
static unsigned long elapsedBaseMs = 0;      // elapsed at last power-on / first pulse
static unsigned long elapsedAnchorMs = 0;    // millis() when elapsedBaseMs was set
static unsigned long movingMs = 0;
static unsigned long lastLoopMs = 0;
static float maxSpeedKmh = 0.0f;

static bool sdReady = false;
static bool sdTrackOk = true;
static bool sdCheckOk = true;
static bool sdArchiveOk = true;
static uint32_t gpxBodyEnd = 0;
static unsigned long lastGpxMs = 0;
static unsigned long lastDatMs = 0;
static unsigned long gpxPointCount = 0;
static int32_t lastGpxLatE7 = 0;
static int32_t lastGpxLonE7 = 0;
static bool lastGpxPosValid = false;
static uint16_t lastGpxYear = 0;
static uint8_t lastGpxMonth = 0;
static uint8_t lastGpxDay = 0;
static uint8_t lastGpxHour = 0;
static uint8_t lastGpxMinute = 0;
static uint8_t lastGpxSec = 0;
static bool lastGpxStampValid = false;
static bool gpxLogStopped = true;
static uint32_t tripDatSeq = 0;
static uint32_t tripRideGen = 1;
static unsigned long bleXferWindowUntil = 0;
static unsigned long btnLastShortMs = 0;
static bool btnPendingBacklight = false;
static unsigned long btnPendingBlMs = 0;

static bool btnSample = false;
static bool btnHeld = false;
static bool btnArmed = false;
static bool btnDidExec = false;
static bool btnReleased = false;
static bool btnReleaseDidExec = false;
static unsigned long btnDebounceMs = 0;
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
static TextCache txtCaption;
static TextCache txtRow[ROW_COUNT];
static TextCache txtFootL;
static TextCache txtFootR;
static char heroShown[8];
static bool heroValid = false;
static int gaugeLit = -1;
static bool uiChromeDrawn = false;

#pragma pack(push, 1)
struct TripDatV2 {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t seq;
  uint32_t revCount;
  uint32_t movingMs;
  uint32_t elapsedMs;
  float maxSpeedKmh;
  uint32_t gpxBodyEnd;
  uint32_t crc;
};

struct TripDat {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t seq;
  uint32_t revCount;
  uint32_t movingMs;
  uint32_t elapsedMs;
  float maxSpeedKmh;
  uint32_t gpxBodyEnd;
  uint32_t rideGen;
  uint32_t crc;
};

enum {
  FINISH_PHASE_ARCHIVING = 1,
  FINISH_PHASE_SUM_DONE = 2,
  FINISH_PHASE_TRACK_READY = 3,
  FINISH_PHASE_CHECKPOINT_DONE = 4
};

struct FinishDat {
  uint32_t magic;
  uint16_t version;
  uint16_t phase;
  char dest[13];
  uint8_t hasPoints;
  uint8_t needSum;
  uint32_t revCount;
  uint32_t movingMs;
  uint32_t elapsedMs;
  float maxSpeedKmh;
  uint32_t points;
  uint32_t fromRideGen;
  uint32_t crc;
};
#pragma pack(pop)

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

void reedIsr() {
  unsigned long now = millis();
  // Drop edges closer than bounce floor or the interval at MAX_SPEED_KMH.
  unsigned long minGap = MIN_REV_MS;
  if (minGap < DEBOUNCE_MS) {
    minGap = DEBOUNCE_MS;
  }
  if (now - g_lastIsrMs < minGap) {
    return;
  }
  g_lastIsrMs = now;

  if (g_lastPulseMs != 0) {
    unsigned long dtMs = now - g_lastPulseMs;
    if (dtMs >= MIN_REV_MS && dtMs < STOPPED_MS) {
      g_movingAccMs += dtMs;
    }
  }

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
  uint8_t flags = p[21];
  uint8_t numSV = p[23];
  int32_t lonE7 = leI32(p + 24);
  int32_t latE7 = leI32(p + 28);
  int32_t hMslMm = leI32(p + 36);
  // fixType 2..4 plus gnssFixOK (flags bit 0): position meets receiver masks.
  bool valid = (fixType >= 2) && (fixType <= 4) && ((flags & 0x01) != 0);
  gps.ubxOk++;
  gps.fixType = fixType;
  gpsApplyFix(valid, true, numSV, valid, (float)hMslMm / 1000.0f, "UBX");
  gpsApplyPos(valid, latE7, lonE7);
  bool timeOk = ((validFlags & 0x03) == 0x03) && year >= 2020 && year < 2100
                && month >= 1 && month <= 12 && day >= 1 && day <= 31;
  gpsApplyTime(timeOk, year, month, day, hour, minute, sec);
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

static void gpsWait(unsigned long ms) {
  unsigned long start = millis();
  do {
    gpsDrain();
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

static void sdNoteTrack(bool ok) {
  sdTrackOk = ok;
}

static void sdNoteCheck(bool ok) {
  sdCheckOk = ok;
}

static void sdNoteArchive(bool ok) {
  sdArchiveOk = ok;
}

static bool sdPersistenceOk() {
  return sdTrackOk && sdCheckOk && sdArchiveOk;
}

static bool sdFileSync(File32 &f) {
  // flush() pushes the cache; sync() also updates the directory entry.
  f.flush();
  return f.sync();
}

static bool tripDatValidateV2(const TripDatV2 *d) {
  if (d->magic != TRIP_MAGIC || d->version != TRIP_VERSION_V2) {
    return false;
  }
  return crc32((const uint8_t *)d, offsetof(TripDatV2, crc)) == d->crc;
}

static bool tripDatValidate(const TripDat *d) {
  if (d->magic != TRIP_MAGIC || d->version != TRIP_VERSION) {
    return false;
  }
  return crc32((const uint8_t *)d, offsetof(TripDat, crc)) == d->crc;
}

static void tripDatFromV2(const TripDatV2 *v2, TripDat *out) {
  memset(out, 0, sizeof(*out));
  out->magic = TRIP_MAGIC;
  out->version = TRIP_VERSION;
  out->seq = v2->seq;
  out->revCount = v2->revCount;
  out->movingMs = v2->movingMs;
  out->elapsedMs = v2->elapsedMs;
  out->maxSpeedKmh = v2->maxSpeedKmh;
  out->gpxBodyEnd = v2->gpxBodyEnd;
  out->rideGen = 1;
  out->crc = crc32((const uint8_t *)out, offsetof(TripDat, crc));
}

static bool tripDatLoadSlot(const char *name, TripDat *out) {
  File32 f;
  if (!f.open(name, O_RDONLY)) {
    return false;
  }
  uint32_t sz = (uint32_t)f.fileSize();
  if (sz == sizeof(TripDat)) {
    TripDat d;
    int n = f.read(&d, sizeof(d));
    f.close();
    if (n != (int)sizeof(d) || !tripDatValidate(&d)) {
      return false;
    }
    *out = d;
    return true;
  }
  if (sz == sizeof(TripDatV2)) {
    TripDatV2 v2;
    int n = f.read(&v2, sizeof(v2));
    f.close();
    if (n != (int)sizeof(v2) || !tripDatValidateV2(&v2)) {
      return false;
    }
    tripDatFromV2(&v2, out);
    return true;
  }
  f.close();
  return false;
}

// One-shot migration from the old single-file O_TRUNC checkpoint.
static bool tripDatLoadLegacy(TripDat *out) {
  File32 f;
  if (!f.open(DAT_NAME_LEGACY, O_RDONLY)) {
    return false;
  }
#pragma pack(push, 1)
  struct TripDatV1 {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t revCount;
    uint32_t movingMs;
    uint32_t elapsedMs;
    float maxSpeedKmh;
    uint32_t gpxBodyEnd;
    uint32_t crc;
  } v1;
#pragma pack(pop)
  int n = f.read(&v1, sizeof(v1));
  f.close();
  if (n != (int)sizeof(v1)) {
    return false;
  }
  if (v1.magic != TRIP_MAGIC || v1.version != TRIP_VERSION_LEGACY) {
    return false;
  }
  if (crc32((const uint8_t *)&v1, offsetof(TripDatV1, crc)) != v1.crc) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->magic = TRIP_MAGIC;
  out->version = TRIP_VERSION;
  out->seq = 1;
  out->revCount = v1.revCount;
  out->movingMs = v1.movingMs;
  out->elapsedMs = v1.elapsedMs;
  out->maxSpeedKmh = v1.maxSpeedKmh;
  out->gpxBodyEnd = v1.gpxBodyEnd;
  out->rideGen = 1;
  out->crc = crc32((const uint8_t *)out, offsetof(TripDat, crc));
  return true;
}

static bool tripDatLoad(TripDat *out) {
  TripDat a;
  TripDat b;
  bool ha = tripDatLoadSlot(DAT_NAME_A, &a);
  bool hb = tripDatLoadSlot(DAT_NAME_B, &b);
  if (ha && hb) {
    *out = (a.seq >= b.seq) ? a : b;
    return true;
  }
  if (ha) {
    *out = a;
    return true;
  }
  if (hb) {
    *out = b;
    return true;
  }
  if (tripDatLoadLegacy(out)) {
    Serial.println("SD migrated legacy TRIP.DAT");
    return true;
  }
  return false;
}

static bool tripDatWriteSlot(const char *name, const TripDat *d) {
  gpsDrain();
  File32 f;
  if (!f.open(name, O_RDWR | O_CREAT | O_TRUNC)) {
    gpsDrain();
    return false;
  }
  bool ok = f.write(d, sizeof(*d)) == sizeof(*d);
  if (ok) {
    ok = sdFileSync(f);
  }
  f.close();
  gpsDrain();
  return ok;
}

static bool tripDatSave(uint32_t revCount, uint32_t elapsedMsNow) {
  if (!sdReady) {
    return false;
  }

  uint32_t nextSeq = tripDatSeq + 1u;
  if (nextSeq == 0) {
    nextSeq = 1;
  }

  TripDat d;
  memset(&d, 0, sizeof(d));
  d.magic = TRIP_MAGIC;
  d.version = TRIP_VERSION;
  d.seq = nextSeq;
  d.revCount = revCount;
  d.movingMs = (uint32_t)movingMs;
  d.elapsedMs = elapsedMsNow;
  d.maxSpeedKmh = maxSpeedKmh;
  d.gpxBodyEnd = gpxBodyEnd;
  d.rideGen = tripRideGen;
  d.crc = crc32((const uint8_t *)&d, offsetof(TripDat, crc));

  // Alternate slots so a torn write cannot destroy the last good checkpoint.
  const char *name = (nextSeq & 1u) ? DAT_NAME_A : DAT_NAME_B;

  bool ok = tripDatWriteSlot(name, &d);
  if (ok) {
    tripDatSeq = nextSeq;
    lastDatMs = millis();
    // Best-effort cleanup of the pre-A/B single file.
    if (sd.exists(DAT_NAME_LEGACY)) {
      sd.remove(DAT_NAME_LEGACY);
    }
  }
  sdNoteCheck(ok);
  return ok;
}

// Write zeroed checkpoints to both slots so finish cannot revive the old ride.
static bool tripDatZeroBoth(uint32_t newRideGen) {
  if (!sdReady) {
    return false;
  }

  uint32_t s1 = tripDatSeq + 1u;
  if (s1 == 0) {
    s1 = 1;
  }
  uint32_t s2 = s1 + 1u;
  if (s2 == 0) {
    s2 = 1;
  }

  TripDat d;
  memset(&d, 0, sizeof(d));
  d.magic = TRIP_MAGIC;
  d.version = TRIP_VERSION;
  d.rideGen = newRideGen;
  d.gpxBodyEnd = gpxBodyEnd;

  d.seq = s1;
  d.crc = crc32((const uint8_t *)&d, offsetof(TripDat, crc));
  const char *n1 = (s1 & 1u) ? DAT_NAME_A : DAT_NAME_B;
  if (!tripDatWriteSlot(n1, &d)) {
    sdNoteCheck(false);
    return false;
  }

  d.seq = s2;
  d.crc = crc32((const uint8_t *)&d, offsetof(TripDat, crc));
  const char *n2 = (s2 & 1u) ? DAT_NAME_A : DAT_NAME_B;
  if (!tripDatWriteSlot(n2, &d)) {
    sdNoteCheck(false);
    return false;
  }

  tripDatSeq = s2;
  tripRideGen = newRideGen;
  lastDatMs = millis();
  if (sd.exists(DAT_NAME_LEGACY)) {
    sd.remove(DAT_NAME_LEGACY);
  }
  sdNoteCheck(true);
  return true;
}

static int gpsTimeCmp(uint16_t y1, uint8_t mo1, uint8_t d1,
                      uint8_t h1, uint8_t mi1, uint8_t s1,
                      uint16_t y2, uint8_t mo2, uint8_t d2,
                      uint8_t h2, uint8_t mi2, uint8_t s2) {
  if (y1 != y2) {
    return (y1 < y2) ? -1 : 1;
  }
  if (mo1 != mo2) {
    return (mo1 < mo2) ? -1 : 1;
  }
  if (d1 != d2) {
    return (d1 < d2) ? -1 : 1;
  }
  if (h1 != h2) {
    return (h1 < h2) ? -1 : 1;
  }
  if (mi1 != mi2) {
    return (mi1 < mi2) ? -1 : 1;
  }
  if (s1 != s2) {
    return (s1 < s2) ? -1 : 1;
  }
  return 0;
}

static bool gpsTimeAdvanced() {
  if (!lastGpxStampValid) {
    return true;
  }
  return gpsTimeCmp(gps.year, gps.month, gps.day, gps.hour, gps.minute, gps.sec,
                    lastGpxYear, lastGpxMonth, lastGpxDay,
                    lastGpxHour, lastGpxMinute, lastGpxSec) > 0;
}

static void gpxNoteStamp() {
  lastGpxYear = gps.year;
  lastGpxMonth = gps.month;
  lastGpxDay = gps.day;
  lastGpxHour = gps.hour;
  lastGpxMinute = gps.minute;
  lastGpxSec = gps.sec;
  lastGpxStampValid = true;
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
  return sdFileSync(gpxFile);
}

// Prefer the last complete </trkpt> over size-minus-footer (which can leave a tear).
static bool gpxRecoverBodyEnd(uint32_t *outEnd) {
  uint32_t sz = (uint32_t)gpxFile.fileSize();
  size_t headerLen = strlen(GPX_HEADER);
  if (sz <= headerLen) {
    *outEnd = (uint32_t)headerLen;
    return sz >= headerLen;
  }

  const char marker[] = "</trkpt>";
  const size_t markerLen = sizeof(marker) - 1;
  const size_t chunkMax = 2048;
  uint8_t buf[2048];
  uint32_t best = 0;
  uint32_t pos = (sz > chunkMax) ? (sz - (uint32_t)chunkMax) : 0;
  // Walk overlapping windows from the start of the last 8 KiB toward EOF.
  uint32_t scanFrom = (sz > 8192u) ? (sz - 8192u) : 0;
  pos = scanFrom;
  while (pos < sz) {
    uint32_t want = sz - pos;
    if (want > chunkMax) {
      want = (uint32_t)chunkMax;
    }
    if (!gpxFile.seekSet(pos)) {
      return false;
    }
    int n = gpxFile.read(buf, want);
    if (n <= 0) {
      break;
    }
    for (int i = 0; i + (int)markerLen <= n; i++) {
      if (memcmp(buf + i, marker, markerLen) == 0) {
        uint32_t end = pos + (uint32_t)i + (uint32_t)markerLen;
        // Optional trailing newline after the tag.
        if (i + (int)markerLen < n && buf[i + (int)markerLen] == '\n') {
          end++;
        } else if (end < sz) {
          // Peek one byte if the newline fell outside this read.
          uint8_t nl = 0;
          if (gpxFile.seekSet(end) && gpxFile.read(&nl, 1) == 1 && nl == '\n') {
            end++;
          }
        }
        if (end > best) {
          best = end;
        }
      }
    }
    if (want < chunkMax) {
      break;
    }
    // Overlap so a marker straddling the boundary is not missed.
    pos += (uint32_t)chunkMax - (uint32_t)markerLen;
  }

  if (best >= headerLen) {
    *outEnd = best;
    return true;
  }
  *outEnd = (uint32_t)headerLen;
  return true;
}

static bool gpxBodyEndLooksComplete(uint32_t bodyEnd) {
  size_t headerLen = strlen(GPX_HEADER);
  if (bodyEnd == headerLen) {
    return true;
  }
  if (bodyEnd < headerLen + 8) {
    return false;
  }
  const char marker[] = "</trkpt>";
  const size_t markerLen = sizeof(marker) - 1;
  uint8_t tail[16];
  uint32_t from = bodyEnd - (uint32_t)markerLen;
  // Allow an optional '\n' after the marker.
  if (bodyEnd >= headerLen + markerLen + 1) {
    from = bodyEnd - (uint32_t)markerLen - 1;
  }
  if (!gpxFile.seekSet(from)) {
    return false;
  }
  int n = gpxFile.read(tail, sizeof(tail));
  if (n < (int)markerLen) {
    return false;
  }
  for (int i = 0; i + (int)markerLen <= n; i++) {
    if (memcmp(tail + i, marker, markerLen) == 0) {
      uint32_t end = from + (uint32_t)i + (uint32_t)markerLen;
      if (i + (int)markerLen < n && tail[i + (int)markerLen] == '\n') {
        end++;
      }
      return end == bodyEnd;
    }
  }
  return false;
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
      sdNoteTrack(false);
      return false;
    }
    sdNoteTrack(true);
    Serial.println("SD new CURRENT.GPX");
    return true;
  }

  size_t headerLen = strlen(GPX_HEADER);
  bool bodyOk = (gpxBodyEnd >= headerLen && gpxBodyEnd <= sz &&
                 gpxBodyEndLooksComplete(gpxBodyEnd));
  if (!bodyOk) {
    uint32_t recovered = 0;
    if (!gpxRecoverBodyEnd(&recovered)) {
      Serial.println("SD GPX recover failed");
      return false;
    }
    gpxBodyEnd = recovered;
    Serial.print("SD CURRENT.GPX recovered bodyEnd=");
    Serial.println(gpxBodyEnd);
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
    sdNoteTrack(false);
    return false;
  }
  size_t fsz = strlen(GPX_FOOTER);
  if (gpxFile.write(GPX_FOOTER, fsz) != fsz) {
    gpsDrain();
    sdNoteTrack(false);
    return false;
  }
  uint32_t finalSize = gpxBodyEnd + (uint32_t)fsz;
  if (!gpxFile.truncate(finalSize)) {
    gpsDrain();
    sdNoteTrack(false);
    return false;
  }
  bool ok = sdFileSync(gpxFile);
  gpsDrain();
  sdNoteTrack(ok);
  if (ok) {
    Serial.print("SD GPX finalized size=");
    Serial.println(finalSize);
  }
  return ok;
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
    sdNoteTrack(false);
    return false;
  }
  if (gpxFile.write(line, (size_t)n) != (size_t)n) {
    gpsDrain();
    sdNoteTrack(false);
    return false;
  }
  uint32_t newBody = (uint32_t)gpxFile.curPosition();
  size_t fsz = strlen(GPX_FOOTER);
  bool ok = gpxFile.write(GPX_FOOTER, fsz) == fsz;
  if (ok) {
    ok = sdFileSync(gpxFile);
  }
  gpsDrain();
  if (ok) {
    gpxBodyEnd = newBody;
    gpxPointCount++;
    lastGpxLatE7 = gps.latE7;
    lastGpxLonE7 = gps.lonE7;
    lastGpxPosValid = true;
    gpxNoteStamp();
  }
  sdNoteTrack(ok);
  return ok;
}

static bool bleXferWindowOpen(unsigned long now) {
  return bleXferWindowUntil != 0 && (long)(now - bleXferWindowUntil) < 0;
}

static void bleArmXferWindow(unsigned long now) {
  bleXferWindowUntil = now + BLE_XFER_WINDOW_MS;
}

static void bleCloseXferWindow() {
  bleXferWindowUntil = 0;
}

static void tripLogGps(unsigned long now, uint32_t revCount, uint32_t elapsedMsNow,
                       unsigned long lastPulseMs) {
  if (!sdReady || !tripStarted || !gpsIsLive(now) || !gps.posKnown || !gps.timeKnown) {
    return;
  }

  bool stopped = (lastPulseMs == 0) || ((now - lastPulseMs) >= STOPPED_MS);
  bool forceEndpoint = false;

  if (stopped) {
    if (gpxLogStopped) {
      return;
    }
    forceEndpoint = true;
  } else {
    if (gpxLogStopped) {
      forceEndpoint = true;
    }
  }

  if (!forceEndpoint) {
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
  } else if (now - lastGpxMs < 200) {
    // Avoid double-writing stop/resume within the same blink of an eye.
    return;
  }

  if (!gpsTimeAdvanced()) {
    return;
  }

  if (gpxAppendPoint()) {
    lastGpxMs = now;
    gpxLogStopped = stopped;
    tripDatSave(revCount, elapsedMsNow);
  }
  // On failure leave lastGpxMs / gpxLogStopped alone so the next loop can retry.
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

static void sumStemFromGpx(const char *gpxName, char *stem, size_t stemLen) {
  stem[0] = '\0';
  if (!gpxName || stemLen < 2) {
    return;
  }
  const char *dot = strrchr(gpxName, '.');
  size_t n = dot ? (size_t)(dot - gpxName) : strlen(gpxName);
  if (n >= stemLen) {
    n = stemLen - 1;
  }
  memcpy(stem, gpxName, n);
  stem[n] = '\0';
}

static bool tripStemFree(const char *gpxName) {
  if (sd.exists(gpxName)) {
    return false;
  }
  char stem[9];
  sumStemFromGpx(gpxName, stem, sizeof(stem));
  if (stem[0] == '\0') {
    return false;
  }
  char sumName[13];
  snprintf(sumName, sizeof(sumName), "%s.SUM", stem);
  return !sd.exists(sumName);
}

static bool finishValidate(const FinishDat *d) {
  if (d->magic != FINISH_MAGIC || d->version != FINISH_VERSION) {
    return false;
  }
  return crc32((const uint8_t *)d, offsetof(FinishDat, crc)) == d->crc;
}

static bool finishLoad(FinishDat *out) {
  File32 f;
  if (!f.open(FINISH_NAME, O_RDONLY)) {
    return false;
  }
  FinishDat d;
  int n = f.read(&d, sizeof(d));
  f.close();
  if (n != (int)sizeof(d) || !finishValidate(&d)) {
    return false;
  }
  *out = d;
  return true;
}

static bool finishSave(FinishDat *d) {
  d->magic = FINISH_MAGIC;
  d->version = FINISH_VERSION;
  d->crc = crc32((const uint8_t *)d, offsetof(FinishDat, crc));
  gpsDrain();
  File32 f;
  if (!f.open(FINISH_NAME, O_RDWR | O_CREAT | O_TRUNC)) {
    gpsDrain();
    sdNoteArchive(false);
    return false;
  }
  bool ok = f.write(d, sizeof(*d)) == sizeof(*d);
  if (ok) {
    ok = sdFileSync(f);
  }
  f.close();
  gpsDrain();
  sdNoteArchive(ok);
  return ok;
}

static void finishClear() {
  gpsDrain();
  if (sd.exists(FINISH_NAME)) {
    sd.remove(FINISH_NAME);
  }
  gpsDrain();
}

static void tripResetRam() {
  noInterrupts();
  g_revCount = 0;
  g_lastPulseMs = 0;
  g_prevPulseMs = 0;
  g_lastIsrMs = 0;
  g_movingAccMs = 0;
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
  lastGpxStampValid = false;
  gpxLogStopped = true;
}

static bool tripWriteSummary(const char *gpxName, uint32_t revCount,
                             uint32_t elapsedMsNow, unsigned long points,
                             uint32_t movingMsSnap, float maxSnap,
                             bool allowReplace) {
  char stem[9];
  sumStemFromGpx(gpxName, stem, sizeof(stem));
  if (stem[0] == '\0') {
    return false;
  }
  char sumName[13];
  snprintf(sumName, sizeof(sumName), "%s.SUM", stem);

  float distanceM = (revCount * WHEEL_CIRC_MM) / 1000.0f;
  char body[256];
  int n = snprintf(body, sizeof(body),
                   "rev=%lu\n"
                   "distance_m=%.1f\n"
                   "moving_ms=%lu\n"
                   "elapsed_ms=%lu\n"
                   "max_kmh=%.2f\n"
                   "points=%lu\n"
                   "end=%04u-%02u-%02uT%02u:%02u:%02uZ\n",
                   (unsigned long)revCount,
                   (double)distanceM,
                   (unsigned long)movingMsSnap,
                   (unsigned long)elapsedMsNow,
                   (double)maxSnap,
                   (unsigned long)points,
                   gps.timeKnown ? (unsigned)gps.year : 0u,
                   gps.timeKnown ? (unsigned)gps.month : 0u,
                   gps.timeKnown ? (unsigned)gps.day : 0u,
                   gps.timeKnown ? (unsigned)gps.hour : 0u,
                   gps.timeKnown ? (unsigned)gps.minute : 0u,
                   gps.timeKnown ? (unsigned)gps.sec : 0u);
  if (n <= 0 || n >= (int)sizeof(body)) {
    return false;
  }

  gpsDrain();
  File32 f;
  const int flags = allowReplace ? (O_RDWR | O_CREAT | O_TRUNC)
                               : (O_RDWR | O_CREAT | O_EXCL);
  if (!f.open(sumName, flags)) {
    gpsDrain();
    // Retry path: exclusive create failed because a prior attempt left a file.
    if (!allowReplace && sd.exists(sumName)) {
      if (!f.open(sumName, O_RDWR | O_CREAT | O_TRUNC)) {
        sdNoteArchive(false);
        return false;
      }
    } else {
      sdNoteArchive(false);
      return false;
    }
  }
  bool ok = f.write(body, (size_t)n) == (size_t)n;
  if (ok) {
    ok = sdFileSync(f);
  }
  f.close();
  gpsDrain();
  sdNoteArchive(ok);
  if (ok) {
    Serial.print("SD wrote ");
    Serial.println(sumName);
  }
  return ok;
}

static bool tripPickArchiveName(char *buf, size_t buflen) {
  if (buflen < 13) {
    return false;
  }

  if (gps.timeKnown) {
    unsigned yy = (unsigned)(gps.year % 100);
    unsigned mo = (unsigned)gps.month;
    unsigned dd = (unsigned)gps.day;
    unsigned hh = (unsigned)gps.hour;
    snprintf(buf, buflen, "%02u%02u%02u%02u.GPX", yy, mo, dd, hh);
    if (tripStemFree(buf)) {
      return true;
    }
    // Hours are 00-23; 24-99 are collision sequence slots (not clock hours).
    for (unsigned seq = 24; seq < 100; seq++) {
      snprintf(buf, buflen, "%02u%02u%02u%02u.GPX", yy, mo, dd, seq);
      if (tripStemFree(buf)) {
        return true;
      }
    }
  }

  for (unsigned n = 1; n <= 9999; n++) {
    snprintf(buf, buflen, "RIDE%04u.GPX", n);
    if (tripStemFree(buf)) {
      return true;
    }
    if ((n & 0x1F) == 0) {
      gpsDrain();
    }
  }
  return false;
}

// NEW_RIDE_SAVED / NEW_RIDE_NO_GPS / NEW_RIDE_RESET / NEW_RIDE_FAIL.
enum {
  NEW_RIDE_FAIL = 0,
  NEW_RIDE_RESET,
  NEW_RIDE_SAVED,
  NEW_RIDE_NO_GPS
};

static void bleStopForNewRide();

static bool tripEnsureNewCurrent(bool hasPoints) {
  gpxBodyEnd = 0;
  if (!hasPoints) {
    gpsDrain();
    if (gpxFile.isOpen()) {
      gpxFile.close();
    }
    if (!gpxFile.open(GPX_NAME, O_RDWR | O_CREAT | O_TRUNC)) {
      gpsDrain();
      sdReady = false;
      Serial.println("SD CURRENT.GPX reset failed");
      sdNoteTrack(false);
      return false;
    }
    if (!gpxWriteHeader()) {
      gpsDrain();
      Serial.println("SD CURRENT.GPX header reset failed");
      sdNoteTrack(false);
      return false;
    }
    sdNoteTrack(true);
    gpsDrain();
    return true;
  }
  if (!gpxOpenOrCreate()) {
    sdReady = false;
    Serial.println("SD new CURRENT.GPX failed after archive");
    sdNoteTrack(false);
    return false;
  }
  return true;
}

// Advance an in-progress finish journal. RAM counters stay until checkpoints
// for the new ride are committed to both slots.
static int tripFinishAdvance(FinishDat *fin) {
  if (fin->phase == FINISH_PHASE_ARCHIVING) {
    bool allowReplaceSum = false;
    if (fin->needSum && fin->dest[0] != '\0') {
      char stem[9];
      sumStemFromGpx(fin->dest, stem, sizeof(stem));
      char sumName[13];
      snprintf(sumName, sizeof(sumName), "%s.SUM", stem);
      allowReplaceSum = sd.exists(sumName);
    }

    if (fin->hasPoints) {
      if (sd.exists(GPX_NAME) && fin->dest[0] != '\0') {
        gpsDrain();
        if (!sd.rename(GPX_NAME, fin->dest)) {
          gpsDrain();
          // Already renamed on a prior attempt.
          if (!sd.exists(fin->dest)) {
            Serial.print("SD rename failed -> ");
            Serial.println(fin->dest);
            sdNoteArchive(false);
            return NEW_RIDE_FAIL;
          }
        }
        gpsDrain();
        Serial.print("SD archived ");
        Serial.println(fin->dest);
      }
    } else {
      Serial.println("SD empty GPX — skip archive");
    }

    if (fin->needSum) {
      if (fin->dest[0] == '\0') {
        sdNoteArchive(false);
        return NEW_RIDE_FAIL;
      }
      if (!tripWriteSummary(fin->dest, fin->revCount, fin->elapsedMs, fin->points,
                            fin->movingMs, fin->maxSpeedKmh, allowReplaceSum)) {
        return NEW_RIDE_FAIL;
      }
    }

    fin->phase = FINISH_PHASE_SUM_DONE;
    if (!finishSave(fin)) {
      return NEW_RIDE_FAIL;
    }
  }

  if (fin->phase == FINISH_PHASE_SUM_DONE) {
    if (!tripEnsureNewCurrent(fin->hasPoints != 0)) {
      return NEW_RIDE_FAIL;
    }
    fin->phase = FINISH_PHASE_TRACK_READY;
    if (!finishSave(fin)) {
      return NEW_RIDE_FAIL;
    }
  }

  if (fin->phase == FINISH_PHASE_TRACK_READY) {
    uint32_t newGen = fin->fromRideGen + 1u;
    if (newGen == 0) {
      newGen = 1;
    }
    if (!tripDatZeroBoth(newGen)) {
      Serial.println("SD checkpoint reset failed — keeping finish journal");
      return NEW_RIDE_FAIL;
    }
    fin->phase = FINISH_PHASE_CHECKPOINT_DONE;
    if (!finishSave(fin)) {
      // Checkpoints are already zeroed; still clear RAM and journal best-effort.
    }
  }

  tripResetRam();
  finishClear();
  sdNoteArchive(true);
  return fin->hasPoints ? NEW_RIDE_SAVED : NEW_RIDE_NO_GPS;
}

static int tripStartNewRide() {
  bleStopForNewRide();
  bleCloseXferWindow();

  FinishDat existing;
  if (sdReady && finishLoad(&existing)) {
    Serial.println("SD resuming interrupted finish");
    return tripFinishAdvance(&existing);
  }

  noInterrupts();
  uint32_t revSnap = (uint32_t)g_revCount;
  interrupts();
  unsigned long elapsedSnap = 0;
  if (tripStarted) {
    elapsedSnap = elapsedBaseMs + (millis() - elapsedAnchorMs);
  }
  unsigned long pointsSnap = gpxPointCount;
  uint32_t movingSnap = (uint32_t)movingMs;
  float maxSnap = maxSpeedKmh;

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

  FinishDat fin;
  memset(&fin, 0, sizeof(fin));
  fin.phase = FINISH_PHASE_ARCHIVING;
  fin.hasPoints = hasPoints ? 1 : 0;
  fin.needSum = (hasPoints || revSnap > 0) ? 1 : 0;
  fin.revCount = revSnap;
  fin.movingMs = movingSnap;
  fin.elapsedMs = (uint32_t)elapsedSnap;
  fin.maxSpeedKmh = maxSnap;
  fin.points = hasPoints ? (uint32_t)pointsSnap : 0;
  fin.fromRideGen = tripRideGen;

  if (fin.needSum) {
    if (!tripPickArchiveName(fin.dest, sizeof(fin.dest))) {
      Serial.println("SD archive name failed");
      if (!gpxOpenOrCreate()) {
        sdReady = false;
      }
      sdNoteArchive(false);
      return NEW_RIDE_FAIL;
    }
  }

  if (!finishSave(&fin)) {
    if (!gpxOpenOrCreate()) {
      sdReady = false;
    }
    return NEW_RIDE_FAIL;
  }

  return tripFinishAdvance(&fin);
}

static const char *tripResumeSd() {
  FinishDat fin;
  if (finishLoad(&fin)) {
    Serial.println("SD finish journal present — reconciling");
    int result = tripFinishAdvance(&fin);
    if (result == NEW_RIDE_FAIL) {
      sdArchiveOk = false;
      // Keep snapped stats from the journal visible until a later retry.
      noInterrupts();
      g_revCount = fin.revCount;
      g_lastPulseMs = 0;
      g_prevPulseMs = 0;
      g_movingAccMs = 0;
      interrupts();
      movingMs = fin.movingMs;
      maxSpeedKmh = (fin.maxSpeedKmh > 0.0f && fin.maxSpeedKmh <= MAX_SPEED_KMH)
                        ? fin.maxSpeedKmh
                        : 0.0f;
      if (fin.revCount >= 1 || fin.elapsedMs > 0 || fin.movingMs > 0) {
        tripStarted = true;
        elapsedBaseMs = fin.elapsedMs;
        elapsedAnchorMs = millis();
      }
      if (!gpxFile.isOpen() && !gpxOpenOrCreate()) {
        sdReady = false;
        sdTrackOk = false;
        return "No card";
      }
      return "Finish pending";
    }
    if (!gpxFile.isOpen() && !gpxOpenOrCreate()) {
      sdReady = false;
      sdTrackOk = false;
      return "No card";
    }
    return "Card ready";
  }

  TripDat d;
  bool haveDat = tripDatLoad(&d);
  if (haveDat) {
    gpxBodyEnd = d.gpxBodyEnd;
    tripDatSeq = d.seq;
    tripRideGen = (d.rideGen == 0) ? 1 : d.rideGen;
  }

  if (!gpxOpenOrCreate()) {
    sdReady = false;
    sdTrackOk = false;
    sdCheckOk = false;
    gpxFile.close();
    return "No card";
  }

  if (!haveDat) {
    tripDatSave(0, 0);
    Serial.println("SD new trip checkpoint");
    return "Card ready";
  }

  noInterrupts();
  g_revCount = d.revCount;
  g_lastPulseMs = 0;
  g_prevPulseMs = 0;
  g_movingAccMs = 0;
  interrupts();
  movingMs = d.movingMs;
  // Drop a previously saved noise spike so resume does not keep 200+ km/h Max.
  maxSpeedKmh = (d.maxSpeedKmh > 0.0f && d.maxSpeedKmh <= MAX_SPEED_KMH)
                    ? d.maxSpeedKmh
                    : 0.0f;
  if (d.revCount >= 1 || d.elapsedMs > 0 || d.movingMs > 0) {
    tripStarted = true;
    elapsedBaseMs = d.elapsedMs;
    elapsedAnchorMs = millis();
  }
  gpxLogStopped = true;

  Serial.print("SD resume revs=");
  Serial.print(d.revCount);
  Serial.print(" gen=");
  Serial.print(d.rideGen);
  Serial.print(" moveMs=");
  Serial.print(d.movingMs);
  Serial.print(" elapsedMs=");
  Serial.print(d.elapsedMs);
  Serial.print(" max=");
  Serial.println(d.maxSpeedKmh, 1);
  return tripStarted ? "Resuming ride" : "Card ready";
}

static bool bikeStopped(unsigned long now, unsigned long lastPulseMs) {
  return lastPulseMs == 0 || (now - lastPulseMs) >= STOPPED_MS;
}

static void backlightApply() {
  analogWrite(PIN_LCD_BL, BL_DUTY[blMode]);
}

static void backlightNext() {
  blMode = (uint8_t)((blMode + 1) % BL_MODE_COUNT);
  backlightApply();
}

static void btnPoll(unsigned long now) {
  bool raw = digitalRead(PIN_BTN) == LOW;
  if (raw != btnSample) {
    btnSample = raw;
    btnDebounceMs = now;
  }
  if ((now - btnDebounceMs) < BTN_DEBOUNCE_MS) {
    return;
  }

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
    btnPendingBacklight = false;
    btnLastShortMs = 0;
    int result = tripStartNewRide();
    if (result == NEW_RIDE_SAVED) {
      btnFlash("Ride saved", now);
    } else if (result == NEW_RIDE_NO_GPS) {
      btnFlash("No GPS track", now);
    } else if (result == NEW_RIDE_RESET) {
      btnFlash("Stats reset", now);
    } else {
      btnFlash("Save failed", now);
    }
  }

  // Release before the new-ride countdown starts. A cancelled hold, or a
  // hold that already saved the ride, leaves the backlight alone.
  // Double short-press while stopped opens the BLE transfer window.
  // Single-press backlight is deferred until the double-click window expires.
  if (btnReleased) {
    bool didExec = btnReleaseDidExec;
    unsigned long held = btnReleasedHeldMs;
    btnReleased = false;
    if (!didExec && held < BTN_ARM_MS) {
      if (stopped && btnLastShortMs != 0 &&
          (now - btnLastShortMs) <= BTN_DOUBLE_MS) {
        btnLastShortMs = 0;
        btnPendingBacklight = false;
        bleArmXferWindow(now);
        btnFlash("Transfer on", now);
        Serial.println("BLE transfer window armed");
      } else {
        btnLastShortMs = now;
        btnPendingBacklight = true;
        btnPendingBlMs = now;
      }
    }
  }

  if (btnPendingBacklight &&
      (long)(now - btnPendingBlMs) >= (long)BTN_DOUBLE_MS) {
    btnPendingBacklight = false;
    btnLastShortMs = 0;
    backlightNext();
  }
}

static void sdBeginShared() {
  pinMode(PIN_LCD_CS, OUTPUT);
  digitalWrite(PIN_LCD_CS, HIGH);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  gpsDrain();
  SdSpiConfig cfg(PIN_SD_CS, SHARED_SPI, SD_SCK_MHZ(4), &SPI);
  sdReady = sd.begin(cfg);
  gpsDrain();
  if (!sdReady) {
    sdTrackOk = false;
    sdCheckOk = false;
    sdArchiveOk = false;
    Serial.println("SD begin failed — riding without log");
  } else {
    sdTrackOk = true;
    sdCheckOk = true;
    sdArchiveOk = true;
  }
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
static bool bleSendProgress(uint8_t *pct);

static GFXcanvas1 textCanvas(SCREEN_W, 32);
static uint16_t blitLine[SCREEN_W * 4];

enum {
  MARK_NONE = 0,
  MARK_RING,
  MARK_DOT,
  MARK_ALERT,
  MARK_PILL
};

static const Slot SLOT_STATUS  = {108, 4, SCREEN_W - PAD - 108, 22, 15};
static const Slot SLOT_CAPTION = {PAD, CAPTION_Y, SCREEN_W - 2 * PAD, CAPTION_H, 15};
static const Slot SLOT_FOOT_L  = {PAD, FOOTER_Y, 122, FOOTER_H, 15};
static const Slot SLOT_FOOT_R  = {PAD + 122, FOOTER_Y, SCREEN_W - 2 * PAD - 122, FOOTER_H, 15};

static const char *const ROW_LABELS[ROW_COUNT] = {"Distance", "Time", "Moving", "Max"};
static const char *const ROW_UNITS[ROW_COUNT]  = {"km", "", "", "km/h"};

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

// Repaint only the matrix cells that changed. old == NULL paints every cell.
static void drawDotGlyph(int x, int y, const DotGlyph *old, const DotGlyph *g) {
  for (int r = 0; r < 7; r++) {
    for (int c = 0; c < g->cols; c++) {
      bool on = dotOn(g, r, c);
      if (old && dotOn(old, r, c) == on) {
        continue;
      }
      display.fillRect(x + c * HERO_PITCH, y + r * HERO_PITCH,
                       HERO_DOT, HERO_DOT, on ? COL_FG : COL_OFF);
    }
    gpsDrain();
  }
}

static int dotTextWidth(const char *s) {
  int cols = 0;
  int n = 0;
  for (const char *p = s; *p; p++, n++) {
    cols += dotGlyph(*p)->cols;
  }
  if (n == 0) {
    return 0;
  }
  return (cols + n - 1) * HERO_PITCH - (HERO_PITCH - HERO_DOT);
}

static void paintHero(const char *s) {
  size_t n = strlen(s);
  bool full = !heroValid || strlen(heroShown) != n;
  if (!full) {
    for (size_t i = 0; i < n; i++) {
      if (dotGlyph(heroShown[i])->cols != dotGlyph(s[i])->cols) {
        full = true;
        break;
      }
    }
  }
  if (!full && strcmp(heroShown, s) == 0) {
    return;
  }
  if (full) {
    fillRectDrained(0, HERO_Y, SCREEN_W, 7 * HERO_PITCH, COL_BG);
  }

  int x = (SCREEN_W - dotTextWidth(s)) / 2;
  for (size_t i = 0; i < n; i++) {
    const DotGlyph *g = dotGlyph(s[i]);
    const DotGlyph *old = full ? NULL : dotGlyph(heroShown[i]);
    if (old != g) {
      drawDotGlyph(x, HERO_Y, old, g);
    }
    x += (g->cols + 1) * HERO_PITCH;
  }

  strncpy(heroShown, s, sizeof(heroShown) - 1);
  heroShown[sizeof(heroShown) - 1] = '\0';
  heroValid = true;
}

static void paintGauge(int lit) {
  if (lit < 0) {
    lit = 0;
  }
  if (lit > GAUGE_DOTS) {
    lit = GAUGE_DOTS;
  }
  if (lit == gaugeLit) {
    return;
  }
  const int x0 = (SCREEN_W - (GAUGE_DOTS - 1) * GAUGE_PITCH) / 2;
  for (int i = 0; i < GAUGE_DOTS; i++) {
    bool now = i < lit;
    if (gaugeLit >= 0 && (i < gaugeLit) == now) {
      continue;
    }
    display.fillCircle(x0 + i * GAUGE_PITCH, GAUGE_Y, GAUGE_R, now ? COL_FG : COL_OFF);
  }
  gpsDrain();
  gaugeLit = lit;
}

// Units sit in one column flush with the right margin; values end before it.
static int unitX() {
  return SCREEN_W - PAD - trackedWidth(&FreeSans9pt7b, 0, "km/h");
}

static int rowTop(int i) {
  return ROWS_Y + i * ROW_H;
}

static Slot rowSlot(int i) {
  Slot s;
  s.x = VALUE_X;
  s.y = rowTop(i) + 4;
  int right = ROW_UNITS[i][0] ? unitX() - 6 : SCREEN_W - PAD;
  s.w = right - VALUE_X;
  s.h = 28;
  s.base = 21;
  return s;
}

static void drawStaticChrome() {
  drawTracked(&FreeSansBold9pt7b, PAD, STATUS_BASE, 3, "OBJECT", COL_FG);

  for (int i = 0; i <= ROW_COUNT; i++) {
    display.drawFastHLine(PAD, rowTop(i), SCREEN_W - 2 * PAD, COL_RULE);
  }
  gpsDrain();

  for (int i = 0; i < ROW_COUNT; i++) {
    const int base = rowTop(i) + 4 + 21;
    drawLabel(&FreeSans9pt7b, PAD, base, COL_DIM, ROW_LABELS[i]);
    if (ROW_UNITS[i][0]) {
      drawLabel(&FreeSans9pt7b, unitX(), base, COL_DIM, ROW_UNITS[i]);
    }
  }
}

static void invalidateAllFields() {
  invalidateText(&txtStatus);
  invalidateText(&txtCaption);
  for (int i = 0; i < ROW_COUNT; i++) {
    invalidateText(&txtRow[i]);
  }
  invalidateText(&txtFootL);
  invalidateText(&txtFootR);
  heroValid = false;
  heroShown[0] = '\0';
  gaugeLit = -1;
}

static void ensureChrome() {
  if (uiChromeDrawn) {
    return;
  }
  fillScreenDrained(COL_BG);
  display.setTextWrap(false);
  drawStaticChrome();
  invalidateAllFields();
  uiChromeDrawn = true;
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
  OVL_HOLD
};

// Hold-for-new-ride countdown, or the short message after it fires.
static uint8_t overlayState(unsigned long now, unsigned long *remainSec, int *holdLit) {
  if (overlayFlashMsg && (long)(overlayFlashUntilMs - now) > 0) {
    return OVL_FLASH;
  }
  overlayFlashMsg = NULL;

  if (!btnArmed || !btnHeld || btnDidExec) {
    return OVL_NONE;
  }
  unsigned long held = now - btnHoldStartMs;
  if (held < BTN_ARM_MS || held >= BTN_EXEC_MS) {
    return OVL_NONE;
  }
  unsigned long remainMs = BTN_EXEC_MS - held;
  *remainSec = (remainMs + 999UL) / 1000UL;
  *holdLit = (int)((long)(held - BTN_ARM_MS) * GAUGE_DOTS /
                   (long)(BTN_EXEC_MS - BTN_ARM_MS)) + 1;
  return OVL_HOLD;
}

static void paintStatus(unsigned long now) {
  char buf[20];
  if (gpsIsLive(now)) {
    if (gps.satsKnown) {
      snprintf(buf, sizeof(buf), "%u satellites", (unsigned)gps.sats);
    } else {
      strcpy(buf, "GPS");
    }
    drawSlot(SLOT_STATUS, &txtStatus, &FreeSans9pt7b, "", buf, MARK_DOT, COL_FG);
  } else {
    drawSlot(SLOT_STATUS, &txtStatus, &FreeSans9pt7b, "", "Searching", MARK_RING, COL_DIM);
  }
}

static void paintFooter(unsigned long now) {
  char buf[24];
  const bool live = gpsIsLive(now);
  if (live && gps.altKnown) {
    snprintf(buf, sizeof(buf), "Altitude %.0f m", (double)gps.altM);
  } else {
    strcpy(buf, "Altitude --");
  }
  drawSlot(SLOT_FOOT_L, &txtFootL, &FreeSans9pt7b, buf, "", MARK_NONE, COL_DIM);

  if (!sdReady) {
    drawSlot(SLOT_FOOT_R, &txtFootR, &FreeSansBold9pt7b, "", "No card", MARK_ALERT, COL_FG);
  } else if (!sdPersistenceOk()) {
    drawSlot(SLOT_FOOT_R, &txtFootR, &FreeSansBold9pt7b, "", "Write failed", MARK_ALERT, COL_FG);
  } else if (bleXferWindowOpen(now)) {
    drawSlot(SLOT_FOOT_R, &txtFootR, &FreeSans9pt7b, "", "Transfer", MARK_PILL, COL_FG);
  } else if (blePhoneConnected()) {
    drawSlot(SLOT_FOOT_R, &txtFootR, &FreeSans9pt7b, "", "Phone", MARK_DOT, COL_FG);
  } else if (tripStarted && live && gps.posKnown && gps.timeKnown && sdTrackOk) {
    drawSlot(SLOT_FOOT_R, &txtFootR, &FreeSans9pt7b, "", "Recording", MARK_PILL, COL_FG);
  } else {
    drawSlot(SLOT_FOOT_R, &txtFootR, &FreeSans9pt7b, "", "", MARK_NONE, COL_DIM);
  }
}

static void drawRideScreen(float speedKmh, float distanceKm, float avgSpeedKmh,
                           bool avgValid, float maxKmh, unsigned long elapsedMs,
                           unsigned long moveMs, unsigned long now) {
  ensureChrome();

  char buf[24];
  char right[24];

  // Speed: fixed "dd.d" template so the matrix only repaints changed cells.
  float shown = speedKmh;
  if (shown < 0.0f) {
    shown = 0.0f;
  }
  if (shown > 99.9f) {
    shown = 99.9f;
  }
  snprintf(buf, sizeof(buf), "%4.1f", (double)shown);
  paintHero(buf);

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
  } else {
    if (sending) {
      snprintf(right, sizeof(right), "Sending %u%%", (unsigned)sendPct);
    } else if (avgValid) {
      snprintf(right, sizeof(right), "Avg %.1f", (double)avgSpeedKmh);
    } else {
      snprintf(right, sizeof(right), "Avg --");
    }
    drawSlot(SLOT_CAPTION, &txtCaption, &FreeSans9pt7b, "km/h", right, MARK_NONE, COL_DIM);
  }

  // Gauge: new-ride hold, then phone transfer, else speed at 2 km/h a dot.
  int lit;
  if (ovl == OVL_HOLD) {
    lit = holdLit;
  } else if (sending) {
    lit = ((int)sendPct * GAUGE_DOTS + 50) / 100;
  } else {
    lit = (int)(speedKmh / GAUGE_KMH_PER_DOT + 0.5f);
  }
  paintGauge(lit);

  if (distanceKm < 0.0f) {
    distanceKm = 0.0f;
  }
  snprintf(buf, sizeof(buf), distanceKm < 100.0f ? "%.2f" : "%.1f", (double)distanceKm);
  drawSlot(rowSlot(0), &txtRow[0], &FreeSansBold12pt7b, "", buf, MARK_NONE, COL_FG);
  formatHms(elapsedMs, buf, sizeof(buf));
  drawSlot(rowSlot(1), &txtRow[1], &FreeSansBold12pt7b, "", buf, MARK_NONE, COL_FG);
  formatHms(moveMs, buf, sizeof(buf));
  drawSlot(rowSlot(2), &txtRow[2], &FreeSansBold12pt7b, "", buf, MARK_NONE, COL_FG);
  snprintf(buf, sizeof(buf), "%.1f", (double)maxKmh);
  drawSlot(rowSlot(3), &txtRow[3], &FreeSansBold12pt7b, "", buf, MARK_NONE, COL_FG);

  paintStatus(now);
  paintFooter(now);
}

static void drawSplash(const char *line) {
  uiChromeDrawn = false;
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
}

// Phone download of root *.GPX files. Callbacks only set flags; loop()
// does every SD read so the shared SPI bus stays on this task.
// UUIDs share one vendor base. tools/index.html speaks the same bytes.
static const char BLE_RIDE_SVC_UUID[]  = "7A1E0001-4C8B-4D2E-9F63-1B5A0C7E8D24";
static const char BLE_RIDE_CMD_UUID[]  = "7A1E0002-4C8B-4D2E-9F63-1B5A0C7E8D24";
static const char BLE_RIDE_META_UUID[] = "7A1E0003-4C8B-4D2E-9F63-1B5A0C7E8D24";
static const char BLE_RIDE_DATA_UUID[] = "7A1E0004-4C8B-4D2E-9F63-1B5A0C7E8D24";

enum {
  BLE_OP_LIST = 0x01,
  BLE_OP_GET = 0x02,
  BLE_OP_ABORT = 0x03
};

enum {
  BLE_META_ENTRY = 0x01,
  BLE_META_LIST_END = 0x02,
  BLE_META_START = 0x03,
  BLE_META_DONE = 0x04,
  BLE_META_END_TIME = 0x05,
  BLE_META_ERROR = 0x7F
};

enum {
  BLE_ERR_NO_SD = 1,
  BLE_ERR_NAME = 2,
  BLE_ERR_NOT_FOUND = 3,
  BLE_ERR_IO = 4,
  BLE_ERR_ABORT = 5,
  BLE_ERR_LOCKED = 6
};

enum {
  BLE_JOB_IDLE = 0,
  BLE_JOB_LIST,
  BLE_JOB_SEND
};

static BLEService bleSvc(BLE_RIDE_SVC_UUID);
static BLECharacteristic bleCmd(BLE_RIDE_CMD_UUID, CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP, 13);
static BLECharacteristic bleMeta(BLE_RIDE_META_UUID, CHR_PROPS_NOTIFY, 17);
static BLECharacteristic bleData(BLE_RIDE_DATA_UUID, CHR_PROPS_NOTIFY, 244);

static bool bleReady = false;
static volatile uint8_t bleCmdPending = 0;
static volatile uint8_t bleLinkLost = 0;
static volatile uint8_t bleLinkUpEdge = 0;
static uint8_t bleCmdOp = 0;
static char bleCmdName[13];
static uint8_t bleJob = BLE_JOB_IDLE;
static uint8_t bleErrorPending = 0;
static File32 bleFile;
static File32 bleDir;
static bool bleListHave = false;
static bool bleListEnd = false;
static bool bleListNeedTime = false;
static char bleListName[13];
static uint32_t bleListSize = 0;
static uint32_t bleListEndUtc = 0;
static char bleSendName[13];
static uint32_t bleOffset = 0;
static uint32_t bleSize = 0;
static uint32_t bleFileLimit = 0;
static bool bleFooterFromRam = false;
static bool bleStartSent = false;
static uint32_t bleCrc = 0xFFFFFFFFUL;
static uint8_t bleChunk[244];

static bool blePhoneConnected() {
  return bleReady && Bluefruit.connected() > 0;
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
  bleListNeedTime = false;
  bleListName[0] = '\0';
  bleListSize = 0;
  bleListEndUtc = 0;
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

// Civil date to Unix seconds (UTC). Returns 0 on invalid/unknown.
static uint32_t civilToUnix(unsigned y, unsigned mo, unsigned d,
                            unsigned h, unsigned mi, unsigned s) {
  if (y < 1970 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
      h > 23 || mi > 59 || s > 60) {
    return 0;
  }
  // Howard Hinnant's days_from_civil
  y -= (mo <= 2);
  const unsigned era = y / 400;
  const unsigned yoe = y - era * 400;
  const unsigned doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  int64_t days = (int64_t)era * 146097 + (int64_t)doe - 719468;
  int64_t secs = days * 86400 + (int64_t)h * 3600 + (int64_t)mi * 60 + (int64_t)s;
  if (secs <= 0 || secs > (int64_t)0xFFFFFFFFULL) {
    return 0;
  }
  return (uint32_t)secs;
}

static uint32_t sumReadEndUtc(const char *gpxName) {
  char stem[9];
  sumStemFromGpx(gpxName, stem, sizeof(stem));
  if (stem[0] == '\0') {
    return 0;
  }
  char sumName[13];
  snprintf(sumName, sizeof(sumName), "%s.SUM", stem);
  File32 f;
  gpsDrain();
  if (!f.open(sumName, O_RDONLY)) {
    gpsDrain();
    return 0;
  }
  char buf[256];
  int n = f.read(buf, sizeof(buf) - 1);
  f.close();
  gpsDrain();
  if (n <= 0) {
    return 0;
  }
  buf[n] = '\0';
  const char *p = strstr(buf, "end=");
  if (!p) {
    return 0;
  }
  p += 4;
  unsigned y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;
  if (sscanf(p, "%u-%u-%uT%u:%u:%u", &y, &mo, &d, &h, &mi, &s) != 6) {
    return 0;
  }
  if (y == 0) {
    return 0;
  }
  return civilToUnix(y, mo, d, h, mi, s);
}

static bool bleNotifyEndTime(uint32_t endUtc) {
  uint8_t buf[5];
  buf[0] = BLE_META_END_TIME;
  buf[1] = (uint8_t)(endUtc & 0xff);
  buf[2] = (uint8_t)((endUtc >> 8) & 0xff);
  buf[3] = (uint8_t)((endUtc >> 16) & 0xff);
  buf[4] = (uint8_t)((endUtc >> 24) & 0xff);
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
    bleListEndUtc = sumReadEndUtc(name);
    bleListHave = true;
    bleListNeedTime = false;
    return true;
  }
  return false;
}

static bool bleFinishList() {
  uint8_t endb = BLE_META_LIST_END;
  if (!bleNotifyMeta(&endb, 1)) {
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
  if (!bleListNeedTime) {
    if (!bleNotifyNamed(BLE_META_ENTRY, bleListName, bleListSize)) {
      return false;
    }
    bleListNeedTime = true;
    return true;
  }
  if (!bleNotifyEndTime(bleListEndUtc)) {
    return false;
  }
  bleListHave = false;
  bleListNeedTime = false;
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
  if (!bleXferWindowOpen(millis())) {
    bleQueueError(BLE_ERR_LOCKED);
    Serial.println("BLE list denied — transfer window closed");
    return;
  }
  if (!sdReady) {
    bleQueueError(BLE_ERR_NO_SD);
    return;
  }
  // Activity extends the owner-approved window.
  bleArmXferWindow(millis());
  bleJob = BLE_JOB_LIST;
  Serial.println("BLE list");
}

static void bleBeginGet(const char *rawName) {
  char name[13];
  if (!bleNormalizeGpx(rawName, name, sizeof(name))) {
    bleQueueError(BLE_ERR_NAME);
    return;
  }
  if (!bleXferWindowOpen(millis())) {
    bleQueueError(BLE_ERR_LOCKED);
    Serial.println("BLE get denied — transfer window closed");
    return;
  }
  if (!sdReady) {
    bleQueueError(BLE_ERR_NO_SD);
    return;
  }
  bleResetXfer();
  bleErrorPending = 0;
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
  bleArmXferWindow(millis());
  bleJob = BLE_JOB_SEND;
  Serial.print("BLE get ");
  Serial.print(name);
  Serial.print(" bytes=");
  Serial.println(bleSize);
}

static void bleTakeCommand() {
  uint8_t op;
  char name[13];
  noInterrupts();
  op = bleCmdOp;
  memcpy(name, bleCmdName, sizeof(name));
  bleCmdPending = 0;
  interrupts();

  if (op == BLE_OP_ABORT) {
    if (bleJob != BLE_JOB_IDLE || bleErrorPending != 0) {
      bleQueueError(BLE_ERR_ABORT);
      Serial.println("BLE abort");
    }
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
  bleQueueError(BLE_ERR_NAME);
}

static bool bleActive() {
  return bleReady && (bleJob != BLE_JOB_IDLE || bleErrorPending != 0);
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

  if (bleLinkUpEdge) {
    bleLinkUpEdge = 0;
    Serial.println("BLE connected");
  }
  if (bleLinkLost || (bleJob != BLE_JOB_IDLE && !Bluefruit.connected())) {
    bleLinkLost = 0;
    noInterrupts();
    bleCmdPending = 0;
    interrupts();
    bleErrorPending = 0;
    bleResetXfer();
    Serial.println("BLE disconnected");
    return false;
  }

  if (bleCmdPending) {
    bleTakeCommand();
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
  if (len < 1 || len > 13) {
    return;
  }
  uint8_t op = data[0];
  char name[13];
  memset(name, 0, sizeof(name));
  if (op == BLE_OP_GET && len > 1) {
    uint16_t n = (uint16_t)(len - 1);
    if (n > 12) {
      n = 12;
    }
    memcpy(name, data + 1, n);
  }
  noInterrupts();
  bleCmdOp = op;
  memcpy(bleCmdName, name, sizeof(bleCmdName));
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
  // Keep the physical transfer window so the owner can reconnect within it.
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
  Bluefruit.setName("OBJECT-001");
  Bluefruit.Periph.setConnectCallback(bleOnConnect);
  Bluefruit.Periph.setDisconnectCallback(bleOnDisconnect);
  Bluefruit.Periph.setConnIntervalMS(15, 30);

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
  Serial.println("BLE advertising OBJECT-001");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_REED, INPUT_PULLUP);
  pinMode(PIN_BTN, INPUT_PULLUP);
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
  Serial.println("XIAO cycling computer — ILI9341 + reed D0 + btn D4 + GPS Serial1 + SD D5");

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
    sdLine = tripResumeSd();
  } else {
    sdLine = "No card";
  }
  drawSplash(sdLine);
  delay(1000);

  gps.src = "-";
  Serial1.begin(GPS_BAUD);
  gpsConfigure();
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
  unsigned long revCount    = g_revCount;
  unsigned long lastPulseMs = g_lastPulseMs;
  unsigned long prevPulseMs = g_prevPulseMs;
  unsigned long movingAcc   = g_movingAccMs;
  g_movingAccMs = 0;
  interrupts();
  movingMs += movingAcc;

  unsigned long now = millis();
  btnHandle(now, lastPulseMs);

  // New-ride may have cleared the ISR counters; snapshot again.
  noInterrupts();
  revCount    = g_revCount;
  lastPulseMs = g_lastPulseMs;
  prevPulseMs = g_prevPulseMs;
  movingAcc   = g_movingAccMs;
  g_movingAccMs = 0;
  interrupts();
  movingMs += movingAcc;

  float speedKmh = 0.0f;

  // Need two pulses this session (prevPulseMs != 0) so a restored revCount
  // cannot pair with a zero timestamp and invent a huge speed.
  if (revCount >= 2 && prevPulseMs != 0 && (now - lastPulseMs) < STOPPED_MS) {
    unsigned long dtMs = lastPulseMs - prevPulseMs;
    // Same ceiling as the ISR: ignore intervals that imply > MAX_SPEED_KMH.
    if (dtMs >= MIN_REV_MS) {
      // mm/ms -> km/h: (mm/ms) * (3600 s/h) / 1e6 (mm/km) = * 3.6
      float raw = (WHEEL_CIRC_MM / (float)dtMs) * 3.6f;
      if (raw <= MAX_SPEED_KMH) {
        speedKmh = raw;
      }
    }
  }

  float distanceKm = (revCount * WHEEL_CIRC_MM) / 1000000.0f;

  // Trip starts on the first reed pulse.
  if (revCount >= 1 && !tripStarted) {
    tripStarted = true;
    elapsedBaseMs = 0;
    elapsedAnchorMs = lastPulseMs;
  }

  lastLoopMs = now;

  if (speedKmh > maxSpeedKmh && speedKmh <= MAX_SPEED_KMH) {
    maxSpeedKmh = speedKmh;
  }

  // Avg = distance / moving time (stops do not dilute Avg).
  float avgSpeedKmh = 0.0f;
  bool avgValid = (revCount >= AVG_MIN_REVS) && (movingMs >= AVG_MIN_MOVING_MS);
  if (avgValid) {
    avgSpeedKmh = distanceKm / (movingMs / 3600000.0f);
  }

  unsigned long elapsedMs = 0;
  if (tripStarted) {
    elapsedMs = elapsedBaseMs + (now - elapsedAnchorMs);
  }

  tripLogGps(now, (uint32_t)revCount, (uint32_t)elapsedMs, lastPulseMs);
  tripMaybeSave(now, (uint32_t)revCount, (uint32_t)elapsedMs);

  drawRideScreen(speedKmh, distanceKm, avgSpeedKmh, avgValid, maxSpeedKmh,
                 elapsedMs, movingMs, now);
  gpsDrain();

  digitalWrite(LED_BUILTIN, (speedKmh > 0.05f) ? LOW : HIGH);

  bool bleSent = bleService();
  if (bleSent) {
    gpsDrain();
  } else if (bleActive()) {
    gpsWait(5);
  } else {
    gpsWait(50);
  }
}
