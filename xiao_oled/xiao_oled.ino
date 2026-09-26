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
 * Libraries (Arduino Library Manager):
 *   Adafruit ILI9341, Adafruit GFX Library, Adafruit BusIO
 * SdFat is bundled with the Seeeduino nRF52 core (do not install 2.3.x).
 *
 * Wheel: 700x32C (ISO 32-622) -> circumference 2155 mm.
 *
 * TFT layout (portrait 240x320): DIST/TIME | speed + avg | MOVE/MAX | GPS.
 * Rotation 0. If the image is upside down relative to the pin header, use 2.
 */

#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
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

static const uint16_t COL_BG   = ILI9341_BLACK;
static const uint16_t COL_FG   = ILI9341_WHITE;
static const uint16_t COL_RULE = ILI9341_DARKGREY;
static const uint16_t COL_LIVE = ILI9341_GREEN;
static const uint16_t COL_CONN = ILI9341_YELLOW;
static const uint16_t COL_NOSD = ILI9341_RED;

// Portrait stack. Slots reserve the preferred glyph height so a smaller
// fallback size does not leave the previous digits behind.
static const int ROW_LABEL_Y  = 10;
static const int ROW_VALUE_Y  = 32;
static const int ROW_VALUE_H  = 24;  // text size 3
static const int RULE1_Y      = 68;
static const int SPEED_Y      = 84;
static const int SPEED_H      = 48;  // text size 6
static const int AVG_Y        = 148;
static const int AVG_H        = 24;  // text size 3
static const int AVG_ICON     = 16;
static const int RULE2_Y      = 188;
static const int MOVE_LABEL_Y = 200;
static const int MOVE_VALUE_Y = 222;
static const int GPS_Y        = 290;
static const int GPS_H        = 16;  // text size 2
static const int UNIT_SIZE    = 2;

// Circumference of 700x32C (32-622), millimetres.
static const float WHEEL_CIRC_MM = 2155.0f;

// Ignore reed edges closer than this (contact bounce).
static const unsigned long DEBOUNCE_MS = 15;

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

static const char GPX_NAME[] = "CURRENT.GPX";
static const char DAT_NAME[] = "TRIP.DAT";
static const uint32_t TRIP_MAGIC = 0x50495254UL;  // "TRIP"
static const uint16_t TRIP_VERSION = 1;

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

struct FieldCache {
  char text[20];
  uint8_t size;
  bool valid;
};

static FieldCache fldDist;
static FieldCache fldTime;
static FieldCache fldSpeed;
static FieldCache fldAvg;
static FieldCache fldMove;
static FieldCache fldMax;
static FieldCache fldGpsStatus;
static FieldCache fldGpsSats;
static FieldCache fldGpsAlt;
static uint16_t fldGpsColor = 0;
static bool uiChromeDrawn = false;
static bool overlayWasShown = false;
static char overlayDrawnSub[8];
static const char *overlayDrawnTitle = NULL;
static int overlayDrawnFilled = -2;

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
  if (now - g_lastIsrMs < DEBOUNCE_MS) {
    return;
  }
  g_lastIsrMs = now;

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

static uint32_t crc32(const uint8_t *data, size_t len) {
  uint32_t c = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    c ^= data[i];
    for (int b = 0; b < 8; b++) {
      uint32_t mask = (uint32_t)-(int32_t)(c & 1UL);
      c = (c >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~c;
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

static uint32_t gpxTimeStamp() {
  uint32_t days = (uint32_t)(gps.year - 2020) * 366u
                  + (uint32_t)gps.month * 31u
                  + (uint32_t)gps.day;
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
  if (haveDat) {
    gpxBodyEnd = d.gpxBodyEnd;
  }

  if (!gpxOpenOrCreate()) {
    sdReady = false;
    gpxFile.close();
    return "NO SD";
  }

  if (!haveDat) {
    tripDatSave(0, 0);
    Serial.println("SD new TRIP.DAT");
    return "SD OK";
  }

  noInterrupts();
  g_revCount = d.revCount;
  g_lastPulseMs = 0;
  g_prevPulseMs = 0;
  interrupts();
  movingMs = d.movingMs;
  maxSpeedKmh = d.maxSpeedKmh;
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
  return tripStarted ? "SD RESUME" : "SD OK";
}

static void tripResetRam() {
  noInterrupts();
  g_revCount = 0;
  g_lastPulseMs = 0;
  g_prevPulseMs = 0;
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
}

static bool tripPickArchiveName(char *buf, size_t buflen) {
  if (buflen < 13) {
    return false;
  }

  if (gps.timeKnown) {
    unsigned yy = (unsigned)(gps.year % 100);
    unsigned mo = (unsigned)gps.month;
    unsigned dd = (unsigned)gps.day;
    snprintf(buf, buflen, "%02u%02u%02u%02u.GPX", yy, mo, dd, (unsigned)gps.hour);
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

static int tripStartNewRide() {
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
    int result = tripStartNewRide();
    if (result == NEW_RIDE_SAVED) {
      btnFlash("RIDE SAVED", now);
    } else if (result == NEW_RIDE_RESET) {
      btnFlash("RESET", now);
    } else {
      btnFlash("SAVE FAIL", now);
    }
  }

  // Release before the new-ride countdown starts. A cancelled hold, or a
  // hold that already saved the ride, leaves the backlight alone.
  if (btnReleased) {
    bool didExec = btnReleaseDidExec;
    unsigned long held = btnReleasedHeldMs;
    btnReleased = false;
    if (!didExec && held < BTN_ARM_MS) {
      backlightNext();
    }
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
    Serial.println("SD begin failed — riding without log");
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

// Default GFX glyph cell is 6x8 at text size 1.
static int textPixelWidth(const char *s, uint8_t size) {
  return (int)strlen(s) * 6 * (int)size;
}

static void invalidateField(FieldCache *f) {
  f->valid = false;
  f->text[0] = '\0';
  f->size = 0;
}

static void invalidateAllFields() {
  invalidateField(&fldDist);
  invalidateField(&fldTime);
  invalidateField(&fldSpeed);
  invalidateField(&fldAvg);
  invalidateField(&fldMove);
  invalidateField(&fldMax);
  invalidateField(&fldGpsStatus);
  invalidateField(&fldGpsSats);
  invalidateField(&fldGpsAlt);
  fldGpsColor = 0;
}

static bool fieldSame(const FieldCache *f, const char *s, uint8_t size) {
  return f->valid && f->size == size && strcmp(f->text, s) == 0;
}

static void fieldStore(FieldCache *f, const char *s, uint8_t size) {
  strncpy(f->text, s, sizeof(f->text) - 1);
  f->text[sizeof(f->text) - 1] = '\0';
  f->size = size;
  f->valid = true;
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

static void drawHRule(int y) {
  display.drawFastHLine(PAD, y, SCREEN_W - 2 * PAD, COL_RULE);
  gpsDrain();
}

static void printLeft(int x, int y, uint8_t size, uint16_t color, const char *s) {
  display.setTextSize(size);
  display.setTextColor(color);
  display.setCursor(x, y);
  printDrained(s);
}

static void printRight(int rightEdge, int y, uint8_t size, uint16_t color, const char *s) {
  int w = textPixelWidth(s, size);
  printLeft(rightEdge - w, y, size, color, s);
}

static int unitWidth() {
  return textPixelWidth("KM/H", UNIT_SIZE);
}

// Average-speed mark (diameter symbol), 16x16.
static void drawAvgIcon(int x, int y) {
  display.drawCircle(x + 7, y + 7, 6, COL_FG);
  display.fillCircle(x + 7, y + 7, 2, COL_FG);
  display.drawLine(x + 3, y + 11, x + 11, y + 3, COL_FG);
  gpsDrain();
}

// H:MM:SS always (matches Figma samples like 7:34:12).
static void formatHms(unsigned long ms, char *buf, size_t buflen) {
  unsigned long totalSec = ms / 1000UL;
  unsigned long h = totalSec / 3600UL;
  unsigned long m = (totalSec / 60UL) % 60UL;
  unsigned long s = totalSec % 60UL;
  snprintf(buf, buflen, "%lu:%02lu:%02lu", h, m, s);
}

// Pick decimals so the string fits in maxChars (includes sign/dot).
static void formatFloatFit(float value, int maxChars, char *buf, size_t buflen) {
  if (value < 0.0f) {
    value = 0.0f;
  }

  for (int decimals = 2; decimals >= 0; decimals--) {
    snprintf(buf, buflen, "%.*f", decimals, (double)value);
    if ((int)strlen(buf) <= maxChars) {
      return;
    }
  }

  // Last resort: integer, truncated if still too wide.
  snprintf(buf, buflen, "%.0f", (double)value);
  if ((int)strlen(buf) > maxChars) {
    buf[maxChars] = '\0';
  }
}

// Prefer size 3; drop until both values fit on one row.
static uint8_t fitPairSize(const char *left, const char *right, int gap) {
  const int avail = SCREEN_W - 2 * PAD;
  for (uint8_t size = 3; size > 1; size--) {
    if (textPixelWidth(left, size) + textPixelWidth(right, size) + gap <= avail) {
      return size;
    }
  }
  return 1;
}

static void paintValueRow(int y, int slotH, FieldCache *leftF, FieldCache *rightF,
                          const char *left, const char *right) {
  uint8_t size = fitPairSize(left, right, 8);
  if (fieldSame(leftF, left, size) && fieldSame(rightF, right, size)) {
    return;
  }
  fillRectDrained(PAD, y, SCREEN_W - 2 * PAD, slotH, COL_BG);
  int textY = y + (slotH - 8 * (int)size) / 2;
  printLeft(PAD, textY, size, COL_FG, left);
  printRight(SCREEN_W - PAD, textY, size, COL_FG, right);
  fieldStore(leftF, left, size);
  fieldStore(rightF, right, size);
}

static void paintSpeed(const char *speedBuf) {
  const int slotW = SCREEN_W - 2 * PAD - unitWidth() - 6;
  uint8_t speedSize = 6;
  while (speedSize > 1 && textPixelWidth(speedBuf, speedSize) > slotW) {
    speedSize--;
  }
  if (fieldSame(&fldSpeed, speedBuf, speedSize)) {
    return;
  }
  fillRectDrained(PAD, SPEED_Y, slotW, SPEED_H, COL_BG);
  int textY = SPEED_Y + (SPEED_H - 8 * (int)speedSize) / 2;
  printLeft(PAD, textY, speedSize, COL_FG, speedBuf);
  fieldStore(&fldSpeed, speedBuf, speedSize);
}

static void paintAvg(const char *avgBuf) {
  const int textX = PAD + AVG_ICON + 6;
  const int slotW = SCREEN_W - PAD - unitWidth() - 6 - textX;
  uint8_t avgSize = 3;
  while (avgSize > 1 && textPixelWidth(avgBuf, avgSize) > slotW) {
    avgSize--;
  }
  if (fieldSame(&fldAvg, avgBuf, avgSize)) {
    return;
  }
  fillRectDrained(textX, AVG_Y, slotW, AVG_H, COL_BG);
  int textY = AVG_Y + (AVG_H - 8 * (int)avgSize) / 2;
  printLeft(textX, textY, avgSize, COL_FG, avgBuf);
  fieldStore(&fldAvg, avgBuf, avgSize);
}

static void paintGpsFooter(unsigned long now) {
  char satBuf[8];
  char altBuf[12];
  const bool live = gpsIsLive(now);
  const char *status = !sdReady ? "NO SD" : (live ? "LIVE" : "CONNECTING");
  uint16_t statusColor = !sdReady ? COL_NOSD : (live ? COL_LIVE : COL_CONN);

  if (gpsFrameFresh(now) && gps.satsKnown) {
    snprintf(satBuf, sizeof(satBuf), "%u", (unsigned)gps.sats);
  } else {
    memcpy(satBuf, "--", 3);
  }

  if (live && gps.altKnown) {
    snprintf(altBuf, sizeof(altBuf), "%.0fm", (double)gps.altM);
  } else {
    memcpy(altBuf, "--", 3);
  }

  const uint8_t size = 2;
  if (fieldSame(&fldGpsStatus, status, size) &&
      fieldSame(&fldGpsSats, satBuf, size) &&
      fieldSame(&fldGpsAlt, altBuf, size) &&
      fldGpsColor == statusColor) {
    return;
  }

  fillRectDrained(PAD, GPS_Y, SCREEN_W - 2 * PAD, GPS_H, COL_BG);
  int textY = GPS_Y + (GPS_H - 8 * (int)size) / 2;
  printLeft(PAD, textY, size, statusColor, status);
  printLeft(PAD + textPixelWidth(status, size) + 8, textY, size, COL_FG, satBuf);
  printRight(SCREEN_W - PAD, textY, size, COL_FG, altBuf);
  fieldStore(&fldGpsStatus, status, size);
  fieldStore(&fldGpsSats, satBuf, size);
  fieldStore(&fldGpsAlt, altBuf, size);
  fldGpsColor = statusColor;
}

static void drawSpeedChrome() {
  const int unitH = 8 * UNIT_SIZE;
  printRight(SCREEN_W - PAD, SPEED_Y + (SPEED_H - unitH) / 4, UNIT_SIZE, COL_FG, "KM/H");
  drawAvgIcon(PAD, AVG_Y + (AVG_H - AVG_ICON) / 2);
  printRight(SCREEN_W - PAD, AVG_Y + (AVG_H - unitH) / 2, UNIT_SIZE, COL_FG, "KM/H");
}

static void drawStaticChrome() {
  printLeft(PAD, ROW_LABEL_Y, 2, COL_FG, "DIST");
  printRight(SCREEN_W - PAD, ROW_LABEL_Y, 2, COL_FG, "TIME");
  drawHRule(RULE1_Y);
  drawSpeedChrome();
  drawHRule(RULE2_Y);
  printLeft(PAD, MOVE_LABEL_Y, 2, COL_FG, "MOVE");
  printRight(SCREEN_W - PAD, MOVE_LABEL_Y, 2, COL_FG, "MAX");
}

static void ensureChrome() {
  if (uiChromeDrawn) {
    return;
  }
  fillScreenDrained(COL_BG);
  display.setTextWrap(false);
  drawStaticChrome();
  uiChromeDrawn = true;
  invalidateAllFields();
}

static void clearOverlayDrawn() {
  overlayDrawnTitle = NULL;
  overlayDrawnFilled = -2;
  overlayDrawnSub[0] = '\0';
}

static void overlayBoxRect(int *x, int *y, int *w, int *h) {
  *w = 200;
  *h = 80;
  *x = (SCREEN_W - *w) / 2;
  const int blockTop = SPEED_Y;
  const int blockBot = AVG_Y + AVG_H;
  *y = blockTop + (blockBot - blockTop - *h) / 2;
}

static bool overlayActive(unsigned long now, char *sub, size_t sublen,
                          int *barFilled, int *barTotal, const char **title) {
  *barFilled = 0;
  *barTotal = 0;
  sub[0] = '\0';

  if (overlayFlashMsg && (long)(overlayFlashUntilMs - now) > 0) {
    *title = overlayFlashMsg;
    return true;
  }
  overlayFlashMsg = NULL;

  if (!btnArmed || !btnHeld || btnDidExec) {
    return false;
  }

  unsigned long held = now - btnHoldStartMs;
  if (held < BTN_ARM_MS || held >= BTN_EXEC_MS) {
    return false;
  }

  unsigned long remainMs = BTN_EXEC_MS - held;
  unsigned long remainSec = (remainMs + 999UL) / 1000UL;
  snprintf(sub, sublen, "%lu", remainSec);
  *barFilled = (int)(held - BTN_ARM_MS);
  *barTotal = (int)(BTN_EXEC_MS - BTN_ARM_MS);
  *title = "NEW RIDE";
  return true;
}

static void drawOverlayBox(const char *title, const char *sub, int barFilled, int barTotal) {
  int x, y, boxW, boxH;
  overlayBoxRect(&x, &y, &boxW, &boxH);

  const int bw = boxW - 24;
  long fw = 0;
  if (barTotal > 0) {
    fw = (long)(bw - 2) * (long)barFilled / (long)barTotal;
    if (fw < 0) {
      fw = 0;
    }
    if (fw > (bw - 2)) {
      fw = bw - 2;
    }
  }

  if (overlayDrawnTitle == title && overlayDrawnFilled == (int)fw &&
      strcmp(overlayDrawnSub, sub) == 0) {
    return;
  }

  fillRectDrained(x, y, boxW, boxH, COL_BG);
  display.drawRect(x, y, boxW, boxH, COL_FG);
  gpsDrain();

  int tw = textPixelWidth(title, 2);
  printLeft(x + (boxW - tw) / 2, y + 10, 2, COL_FG, title);

  if (sub && sub[0]) {
    int sw = textPixelWidth(sub, 3);
    printLeft(x + (boxW - sw) / 2, y + 32, 3, COL_FG, sub);
  }

  if (barTotal > 0) {
    const int bx = x + 12;
    const int by = y + boxH - 16;
    const int bh = 8;
    display.drawRect(bx, by, bw, bh, COL_FG);
    if (fw > 0) {
      display.fillRect(bx + 1, by + 1, (int)fw, bh - 2, COL_FG);
    }
    gpsDrain();
  }

  overlayDrawnTitle = title;
  overlayDrawnFilled = (int)fw;
  strncpy(overlayDrawnSub, sub ? sub : "", sizeof(overlayDrawnSub) - 1);
  overlayDrawnSub[sizeof(overlayDrawnSub) - 1] = '\0';
}

static void drawRideScreen(float speedKmh, float distanceKm, float avgSpeedKmh,
                           float maxKmh, unsigned long elapsedMs, unsigned long moveMs,
                           unsigned long now) {
  char distBuf[12];
  char timeBuf[16];
  char moveBuf[16];
  char maxBuf[12];
  char speedBuf[12];
  char avgBuf[12];

  formatFloatFit(distanceKm, 6, distBuf, sizeof(distBuf));
  formatHms(elapsedMs, timeBuf, sizeof(timeBuf));
  formatHms(moveMs, moveBuf, sizeof(moveBuf));
  formatFloatFit(maxKmh, 5, maxBuf, sizeof(maxBuf));
  formatFloatFit(speedKmh, 5, speedBuf, sizeof(speedBuf));
  formatFloatFit(avgSpeedKmh, 5, avgBuf, sizeof(avgBuf));

  const int speedSlotW = SCREEN_W - 2 * PAD - unitWidth() - 6;
  int guard = 6;
  while (guard-- > 0 && textPixelWidth(speedBuf, 1) > speedSlotW && strlen(speedBuf) > 1) {
    formatFloatFit(speedKmh, (int)strlen(speedBuf) - 1, speedBuf, sizeof(speedBuf));
  }

  ensureChrome();

  char sub[8];
  int barFilled = 0;
  int barTotal = 0;
  const char *title = NULL;
  bool showOverlay = overlayActive(now, sub, sizeof(sub), &barFilled, &barTotal, &title);

  if (showOverlay) {
    drawOverlayBox(title, sub, barFilled, barTotal);
    overlayWasShown = true;
    return;
  }

  if (overlayWasShown) {
    int x, y, boxW, boxH;
    overlayBoxRect(&x, &y, &boxW, &boxH);
    fillRectDrained(x, y, boxW, boxH, COL_BG);
    drawSpeedChrome();
    invalidateField(&fldSpeed);
    invalidateField(&fldAvg);
    overlayWasShown = false;
    clearOverlayDrawn();
  }

  paintValueRow(ROW_VALUE_Y, ROW_VALUE_H, &fldDist, &fldTime, distBuf, timeBuf);
  paintSpeed(speedBuf);
  paintAvg(avgBuf);
  paintValueRow(MOVE_VALUE_Y, ROW_VALUE_H, &fldMove, &fldMax, moveBuf, maxBuf);
  paintGpsFooter(now);
}

static void drawSplash(const char *sdLine) {
  uiChromeDrawn = false;
  overlayWasShown = false;
  clearOverlayDrawn();
  invalidateAllFields();
  fillScreenDrained(COL_BG);
  display.setTextWrap(false);

  const char *line1 = "Cycling computer";
  const char *line2 = "700x32C / D0";
  int w1 = textPixelWidth(line1, 2);
  int w2 = textPixelWidth(line2, 2);
  int w3 = textPixelWidth(sdLine, 2);
  printLeft((SCREEN_W - w1) / 2, 118, 2, COL_FG, line1);
  printLeft((SCREEN_W - w2) / 2, 148, 2, COL_FG, line2);
  printLeft((SCREEN_W - w3) / 2, 178, 2, COL_FG, sdLine);
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
  drawSplash("SD...");

  sdBeginShared();
  const char *sdLine = "NO SD";
  if (sdReady) {
    sdLine = tripResumeSd();
  } else {
    sdLine = "NO SD";
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

  Serial.println(tripStarted ? "Ready — resuming trip" : "Ready — waiting for wheel pulses");
}

void loop() {
  gpsPoll();

  // Snapshot ISR-owned state with interrupts briefly off (avoid torn reads).
  noInterrupts();
  unsigned long revCount    = g_revCount;
  unsigned long lastPulseMs = g_lastPulseMs;
  unsigned long prevPulseMs = g_prevPulseMs;
  interrupts();

  unsigned long now = millis();
  btnHandle(now, lastPulseMs);

  // New-ride may have cleared the ISR counters; snapshot again.
  noInterrupts();
  revCount    = g_revCount;
  lastPulseMs = g_lastPulseMs;
  prevPulseMs = g_prevPulseMs;
  interrupts();

  float speedKmh = 0.0f;

  // Need two pulses this session (prevPulseMs != 0) so a restored revCount
  // cannot pair with a zero timestamp and invent a huge speed.
  if (revCount >= 2 && prevPulseMs != 0 && (now - lastPulseMs) < STOPPED_MS) {
    unsigned long dtMs = lastPulseMs - prevPulseMs;
    if (dtMs > 0) {
      // mm/ms -> km/h: (mm/ms) * (3600 s/h) / 1e6 (mm/km) = * 3.6
      speedKmh = (WHEEL_CIRC_MM / (float)dtMs) * 3.6f;
    }
  }

  float distanceKm = (revCount * WHEEL_CIRC_MM) / 1000000.0f;

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

  if (speedKmh > maxSpeedKmh) {
    maxSpeedKmh = speedKmh;
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

  tripLogGps(now, (uint32_t)revCount, (uint32_t)elapsedMs);
  tripMaybeSave(now, (uint32_t)revCount, (uint32_t)elapsedMs);

  drawRideScreen(speedKmh, distanceKm, avgSpeedKmh, maxSpeedKmh, elapsedMs, movingMs, now);
  gpsDrain();

  digitalWrite(LED_BUILTIN, (speedKmh > 0.05f) ? LOW : HIGH);

  gpsWait(50);
}
