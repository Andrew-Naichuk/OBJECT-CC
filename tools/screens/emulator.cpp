// OBJECT screen emulator.
//
// Compiles the unmodified firmware (xiao_oled/xiao_oled.ino) against the host
// stubs in ./stubs and the real Adafruit GFX library, then drives it with a
// virtual millisecond clock and virtual hardware: a wheel magnet on the reed
// switch, a u-blox GPS byte stream, the button / battery divider on A4, a
// microSD card (in memory, with fault injection) and a BLE phone.
//
// One process runs one scenario from a cold boot, exactly like the board after
// power-on. regenerate.py builds this file, runs every scenario and turns the
// framebuffer dumps into the README images.
//
//   emulator --list               print scenario names
//   emulator OUT_DIR SCENARIO     run one scenario, write OUT_DIR/<shot>.ppm
//                                 and append OUT_DIR/shots.tsv
//
// Set OBJECT_EMU_LOG=1 to see the firmware's Serial output on stderr.
//
// To document a new screen: reach the state inside a scenario below and call
// shot("NN_name", "what it shows"). Keep names stable; regenerate.py and the
// README refer to them.
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "xiao_oled.ino"

// ---------------------------------------------------------------- hardware
EmuCard g_card;
EmuBle g_ble;
EmuBluefruit Bluefruit;
SPIClass SPI;
HardwareSerial Serial(false);
HardwareSerial Serial1(true);

static unsigned long g_now = 0;
static bool g_btnDown = false;
static double g_battV = 4.02;  // LiPo voltage behind the 100k/100k divider
static int g_blDuty = 255;
static void (*g_reedIsr)() = nullptr;
static double g_wheelKmh = 0.0;
static double g_nextPulse = 0.0;
static std::deque<uint8_t> g_rx;
static bool g_log = false;

enum GpsMode { GPS_OFF, GPS_UBX, GPS_NMEA_NOSATS };
static GpsMode g_gpsMode = GPS_OFF;
static unsigned long g_gpsFixAtMs = 0;
static unsigned long g_nextGpsMs = 0;
static double g_lat = 48.2105, g_lon = 16.3062, g_altM = 247.0;  // Vienna
static uint8_t g_sats = 0;
static const long long GPS_EPOCH0 = 1790867700LL;  // 2026-10-01 15:15:00 UTC

static std::function<void()> g_onSdBegin;
static std::function<void(unsigned long)> g_onDelay;

// Unix seconds -> UTC civil date (portable replacement for gmtime_r).
struct Utc { int year, month, day, hour, minute, second; };
static Utc utcFromUnix(long long t) {
  long long days = t / 86400, rem = t % 86400;
  Utc u;
  u.hour = (int)(rem / 3600);
  u.minute = (int)(rem % 3600 / 60);
  u.second = (int)(rem % 60);
  days += 719468;
  long long era = days / 146097;
  long long doe = days - era * 146097;
  long long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  long long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  long long mp = (5 * doy + 2) / 153;
  u.day = (int)(doy - (153 * mp + 2) / 5 + 1);
  u.month = (int)(mp < 10 ? mp + 3 : mp - 9);
  u.year = (int)(yoe + era * 400 + (u.month <= 2));
  return u;
}

static void pushUbx(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t n) {
  uint8_t a = 0, b = 0;
  auto add = [&](uint8_t v) { a += v; b += a; g_rx.push_back(v); };
  g_rx.push_back(0xB5);
  g_rx.push_back(0x62);
  add(cls); add(id); add(n & 0xFF); add(n >> 8);
  for (uint16_t i = 0; i < n; i++) add(p[i]);
  g_rx.push_back(a);
  g_rx.push_back(b);
}

static void gpsEmitFrame() {
  Utc u = utcFromUnix(GPS_EPOCH0 + (long long)(g_now / 1000));
  bool fix = g_now >= g_gpsFixAtMs;

  if (g_gpsMode == GPS_NMEA_NOSATS) {
    // GGA with an empty satellite-count field.
    char body[128];
    double la = fabs(g_lat), lo = fabs(g_lon);
    int lad = (int)la, lod = (int)lo;
    snprintf(body, sizeof body,
             "GPGGA,%02d%02d%02d.00,%02d%08.5f,N,%03d%08.5f,E,%d,,,%.1f,M,,M,,",
             u.hour, u.minute, u.second, lad, (la - lad) * 60.0, lod,
             (lo - lod) * 60.0, fix ? 1 : 0, g_altM);
    uint8_t ck = 0;
    for (char *p = body; *p; p++) ck ^= (uint8_t)*p;
    char line[160];
    snprintf(line, sizeof line, "$%s*%02X\r\n", body, ck);
    for (char *p = line; *p; p++) g_rx.push_back((uint8_t)*p);
    return;
  }

  // UBX NAV-PVT, only the fields the firmware reads.
  uint8_t p[92];
  memset(p, 0, sizeof p);
  p[4] = (uint8_t)(u.year & 0xFF);
  p[5] = (uint8_t)(u.year >> 8);
  p[6] = (uint8_t)u.month;
  p[7] = (uint8_t)u.day;
  p[8] = (uint8_t)u.hour;
  p[9] = (uint8_t)u.minute;
  p[10] = (uint8_t)u.second;
  p[11] = fix ? 0x07 : 0x00;
  p[20] = fix ? 3 : 0;
  p[21] = fix ? 0x01 : 0x00;
  p[23] = fix ? g_sats : (uint8_t)(g_sats / 3);
  int32_t lon = (int32_t)llround(g_lon * 1e7), lat = (int32_t)llround(g_lat * 1e7);
  int32_t hmsl = (int32_t)llround(g_altM * 1000.0);
  memcpy(p + 24, &lon, 4);
  memcpy(p + 28, &lat, 4);
  memcpy(p + 32, &hmsl, 4);
  memcpy(p + 36, &hmsl, 4);
  pushUbx(0x01, 0x07, p, sizeof p);
}

static void worldTick() {
  g_now++;
  if (g_wheelKmh > 0.0 && g_reedIsr && (double)g_now >= g_nextPulse) {
    g_reedIsr();
    g_nextPulse += cfg.wheelCircMm * 3.6 / g_wheelKmh;
  }
  if (g_now % 1000 == 0 && g_wheelKmh > 0.0) {
    double m = g_wheelKmh / 3.6;  // metres this second; the track heads NE
    g_lat += m * 0.6 / 111320.0;
    g_lon += m * 0.8 / (111320.0 * cos(g_lat * M_PI / 180.0));
    g_altM += 0.4 * sin((double)g_now / 60000.0);
  }
  if (g_gpsMode != GPS_OFF && g_now >= g_nextGpsMs) {
    gpsEmitFrame();
    g_nextGpsMs += 1000;
  }
}

unsigned long millis() { return g_now; }
unsigned long micros() { return g_now * 1000UL; }
void yield() { worldTick(); }
void delay(unsigned long ms) {
  if (g_onDelay) g_onDelay(ms);
  for (unsigned long i = 0; i < ms; i++) worldTick();
}
void pinMode(int, int) {}
void digitalWrite(int, int) {}
int digitalRead(int pin) {
  // Older firmware read the button digitally on D4; keep that working.
  if (pin == D4) return g_btnDown ? LOW : HIGH;
  return HIGH;
}
int analogRead(int pin) {
  if (pin != A4) return 0;
  if (g_btnDown) return 20;  // button shorts the divider to ground
  // 12-bit ADC, 3.6 V full scale, pin sees Vbat / 2.
  int raw = (int)(g_battV / 2.0 / 3.6 * 4096.0 + 0.5);
  return raw > 4095 ? 4095 : raw;
}
void analogWrite(int pin, int v) {
  if (pin == D3) g_blDuty = v;
}
void attachInterrupt(int, void (*fn)(), int) { g_reedIsr = fn; }

int HardwareSerial::available() { return gps_ ? (int)g_rx.size() : 0; }
int HardwareSerial::read() {
  if (!gps_ || g_rx.empty()) return -1;
  int c = g_rx.front();
  g_rx.pop_front();
  return c;
}
size_t HardwareSerial::write(uint8_t c) {
  if (!gps_ && g_log) fputc(c, stderr);
  return 1;
}

void emuOnSdBegin() {
  if (g_onSdBegin) g_onSdBegin();
}

// ---------------------------------------------------------------- harness
static std::string g_outDir = ".";

static void writePpm(const std::string &path) {
  FILE *f = fopen(path.c_str(), "wb");
  if (!f) {
    fprintf(stderr, "cannot write %s\n", path.c_str());
    exit(2);
  }
  fprintf(f, "P6\n%d %d\n255\n", Adafruit_ILI9341::W, Adafruit_ILI9341::H);
  for (int i = 0; i < Adafruit_ILI9341::W * Adafruit_ILI9341::H; i++) {
    uint16_t c = display.fb[i];
    uint8_t r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
    uint8_t px[3] = {(uint8_t)((r << 3) | (r >> 2)), (uint8_t)((g << 2) | (g >> 4)),
                     (uint8_t)((b << 3) | (b >> 2))};
    fwrite(px, 1, 3, f);
  }
  fclose(f);
}

static void runFor(unsigned long ms);

// Dump the panel as a binary PPM plus one line of metadata.
static void shot(const char *name, const char *caption) {
  writePpm(g_outDir + "/" + name + ".ppm");
  std::string meta = g_outDir + "/shots.tsv";
  FILE *m = fopen(meta.c_str(), "ab");
  fprintf(m, "%s\t%d\t%lu\t%s\n", name, g_blDuty, g_now, caption);
  fclose(m);
  fprintf(stderr, "  %-28s t=%7lu ms  backlight=%3d  %s\n", name, g_now, g_blDuty, caption);
}

// A frame of a sequence that is also kept as an ordinary shot.
struct KeyFrame {
  int at;
  const char *shot;
  const char *caption;
};

// Capture `count` frames `stepMs` apart as NAME~NN.ppm, for an animated strip.
static void frames(const char *name, int count, unsigned long stepMs, const char *caption,
                   std::initializer_list<KeyFrame> keys = {}) {
  unsigned long t0 = g_now;
  for (int i = 0; i < count; i++) {
    char suffix[8];
    snprintf(suffix, sizeof suffix, "~%02d", i);
    writePpm(g_outDir + "/" + name + suffix + ".ppm");
    for (const KeyFrame &k : keys) {
      if (k.at == i) shot(k.shot, k.caption);
    }
    if (i + 1 < count) runFor(stepMs);
  }
  std::string meta = g_outDir + "/frames.tsv";
  FILE *m = fopen(meta.c_str(), "ab");
  fprintf(m, "%s\t%d\t%lu\t%lu\t%s\n", name, count, stepMs, t0, caption);
  fclose(m);
  fprintf(stderr, "  %-28s t=%7lu ms  %d frames x %lu ms  %s\n", name, t0, count, stepMs, caption);
}

static void runFor(unsigned long ms) {
  unsigned long end = g_now + ms;
  while (g_now < end) loop();
}

// Run the firmware until `cond` holds after a loop() pass; a scenario that
// never gets there is a bug, so stop rather than capture the wrong screen.
static void runUntil(const std::function<bool()> &cond, unsigned long maxMs, const char *what) {
  unsigned long end = g_now + maxMs;
  while (g_now < end) {
    loop();
    if (cond()) return;
  }
  fprintf(stderr, "timed out waiting for %s\n", what);
  exit(3);
}

static void setWheel(double kmh) {
  if (kmh > 0.0 && g_wheelKmh <= 0.0) g_nextPulse = (double)g_now + 1.0;
  g_wheelKmh = kmh;
}

static void btn(bool down) { g_btnDown = down; }

static void tap() {
  btn(true);
  runFor(200);
  btn(false);
  runFor(300);
}

static void gpsOn(GpsMode mode, unsigned long fixAfterMs, uint8_t sats) {
  g_gpsMode = mode;
  g_gpsFixAtMs = g_now + fixAfterMs;
  g_nextGpsMs = g_now + 100;
  g_sats = sats;
}

static void phoneConnect() {
  g_ble.linked = true;
  if (g_ble.onConnect) g_ble.onConnect(0);
}

static void phoneWrite(uint8_t op, const char *name) {
  uint8_t b[13];
  memset(b, 0, sizeof b);
  b[0] = op;
  uint16_t n = 1;
  if (name) {
    size_t k = strlen(name);
    memcpy(b + 1, name, k);
    n += (uint16_t)k;
  }
  bleOnWrite(0, &bleCmd, b, n);
}

static void phoneDisconnect() { Bluefruit.disconnect(0); }

// Send the device's current settings back with one field changed, the way
// the hub's Settings view does.
static void phoneSaveSettings(void (*edit)(uint8_t *rec)) {
  uint8_t b[BLE_CMD_MAX];
  b[0] = BLE_OP_CFG_SET;
  bleCfgPack(b + 1);
  edit(b + 1);
  bleOnWrite(0, &bleCmd, b, sizeof b);
}

static void writeConfig(const char *text) {
  g_card.files[CFG_NAME] = std::make_shared<std::vector<uint8_t>>(text, text + strlen(text));
}

// Ride a smooth speed profile around `base` km/h.
static void rideProfile(double base, double amp, unsigned long ms, double periodMs = 47000) {
  unsigned long end = g_now + ms;
  while (g_now < end) {
    setWheel(base + amp * sin((double)g_now / periodMs * 2 * M_PI));
    runFor(250);
  }
}

static void rampTo(double kmh, unsigned long ms) {
  double v0 = g_wheelKmh;
  int steps = (int)(ms / 250);
  for (int i = 1; i <= steps; i++) {
    double v = v0 + (kmh - v0) * i / steps;
    setWheel(v < 0.5 ? 0.0 : v);
    runFor(250);
  }
  setWheel(kmh);
}

// ---------------------------------------------------------------- scenarios
static void scBootNew() {
  g_onSdBegin = [] { shot("01_boot_checking_card", "Boot splash while the microSD card is read"); };
  g_onDelay = [](unsigned long ms) {
    if (ms == 1000) shot("03_boot_card_ready", "Boot splash: card found, no saved trip");
  };
  setup();
  g_onDelay = nullptr;
  runFor(4000);
  shot("05_idle_gps_searching", "Ready, waiting for the first wheel pulse, GPS searching");
}

static void scNoCard() {
  g_card.present = false;
  g_battV = 3.86;
  g_onDelay = [](unsigned long ms) {
    if (ms == 1000) shot("02_boot_no_card", "Boot splash: no microSD card");
  };
  setup();
  g_onDelay = nullptr;
  runFor(3000);
  shot("09_idle_no_card", "Ride screen with no card: alert badge in the footer");
  rampTo(21, 6000);
  rideProfile(22, 3, 150000);
  rampTo(0, 6000);
  runFor(4000);
  shot("17_ride_no_card", "Rode without a card: stats live in RAM only");
  btn(true);
  runFor(2400);
  shot("25_hold_no_card", "Holding for a new ride with no card");
  runFor(2100);
  shot("26_flash_stats_reset", "Hold completed without a card: counters cleared in RAM");
  btn(false);
  runFor(3000);
  shot("27_after_stats_reset", "After the reset: zeroed ride screen, still no card");
}

// A ride interrupted by a power-off: CURRENT.GPX with a few points and a
// CRC-valid TRIP.DAT built with the firmware's own struct and CRC.
static void writeResume() {
  std::string gpx = GPX_HEADER;
  for (int i = 0; i < 40; i++) {
    char l[192];
    snprintf(l, sizeof l,
             "<trkpt lat=\"48.%07d\" lon=\"16.%07d\"><ele>251.0</ele>"
             "<time>2026-10-01T09:%02d:%02dZ</time></trkpt>\n",
             2000000 + i * 700, 3000000 + i * 900, 20 + i / 60, i % 60);
    gpx += l;
  }
  gpx += GPX_FOOTER;
  g_card.files[GPX_NAME] = std::make_shared<std::vector<uint8_t>>(gpx.begin(), gpx.end());

  TripDat d;
  memset(&d, 0, sizeof d);
  d.magic = TRIP_MAGIC;
  d.version = TRIP_VERSION;
  d.revCount = 52400;                                  // 112.9 km at 2155 mm
  d.elapsedMs = ((5UL * 60 + 47) * 60 + 12) * 1000UL;  // 5:47:12
  d.movingMs = ((4UL * 60 + 58) * 60 + 3) * 1000UL;    // 4:58:03
  d.maxSpeedKmh = 58.3f;
  d.gpxBodyEnd = 0;
  d.crc = crc32((const uint8_t *)&d, offsetof(TripDat, crc));
  g_card.files[DAT_NAME] =
      std::make_shared<std::vector<uint8_t>>((uint8_t *)&d, (uint8_t *)&d + sizeof d);
}

static void scResume() {
  writeResume();
  g_battV = 3.68;  // most of a long day's charge used
  g_onDelay = [](unsigned long ms) {
    if (ms == 1000) shot("04_boot_resuming_ride", "Boot splash: valid TRIP.DAT found, ride restored");
  };
  setup();
  g_onDelay = nullptr;
  runFor(2500);
  shot("15_ride_resumed", "Resumed 112.9 km ride, waiting for the wheel, GPS still searching");
  gpsOn(GPS_UBX, 9000, 14);
  rampTo(27, 8000);
  rideProfile(28, 2.5, 60000);
  shot("16_ride_over_100km", "Long ride: distance of 100 or more shows one decimal");
}

static void scRecording() {
  setup();
  gpsOn(GPS_UBX, 12000, 6);
  runFor(6000);
  shot("06_idle_gps_acquiring", "GPS sending frames but no fix yet: still Searching");
  runFor(9000);
  g_sats = 11;
  runFor(3000);
  shot("07_idle_gps_fix", "GPS fix: satellites and altitude, no wheel pulse yet so not recording");

  rampTo(24, 15000);
  rideProfile(26, 4, 300000);
  g_sats = 13;
  rampTo(27.4, 4000);
  runFor(3000);
  shot("10_ride_recording", "Riding with a GPS fix: CURRENT.GPX recording");

  rampTo(49, 20000);
  rideProfile(51, 1.0, 12000, 9000);
  shot("11_ride_fast_gauge_full", "Fast descent: the 24-dot gauge is full from about 47 km/h");

  rampTo(18, 15000);
  rideProfile(20, 2, 240000);
  rampTo(0, 8000);
  runFor(6000);
  shot("12_ride_stopped", "Stopped mid-ride: speed 0.0, ride still recording");

  // Short presses cycle the backlight bright -> dim -> off -> bright.
  tap();
  runFor(1000);
  shot("18_backlight_dim", "Short press: backlight dim (backlight_dim, default 40)");
  tap();
  runFor(1000);
  shot("19_backlight_off", "Short press again: backlight off, panel still updating");
  tap();
  runFor(1000);

  // Phone: connect, confirm on the device, download CURRENT.GPX.
  phoneConnect();
  runFor(300);
  shot("28_phone_allow_10s", "Phone connected over BLE: 10 s to allow on the device");
  runFor(5800);
  shot("29_phone_allow_4s", "Confirm window draining, the gauge counts down");
  tap();
  runFor(500);
  shot("30_phone_connected", "Link allowed: Bluetooth mark on the matrix, footer shows Phone");
  runFor(500);
  phoneWrite(BLE_OP_LIST, nullptr);
  runFor(1500);
  phoneWrite(BLE_OP_GET, "CURRENT.GPX");
  uint8_t pct = 0;
  while (!(bleSendProgress(&pct) && pct >= 42)) loop();
  g_ble.stall = true;  // hold the transfer still for the picture
  runFor(400);
  shot("31_phone_sending", "Downloading CURRENT.GPX: caption and gauge show progress");  g_ble.stall = false;
  while (bleSendProgress(&pct)) loop();
  runFor(1500);
  phoneDisconnect();
  runFor(1500);

  // Hold for a new ride.
  btn(true);
  runFor(2150);
  shot("20_hold_2s", "Holding while stopped: the countdown appears after 2 s");
  runFor(1100);
  shot("21_hold_1s", "Countdown at 1 s, gauge almost full");
  runFor(1300);
  shot("22_flash_ride_saved", "Hold reached 4 s: track archived, counters reset");
  btn(false);
  runFor(3000);
  shot("23_new_ride_ready", "Fresh ride after the save, GPS still locked");
}

static void scNoGps() {
  setup();
  rampTo(17, 10000);
  rideProfile(18.5, 2.5, 200000);
  rampTo(16.3, 3000);
  runFor(2000);
  shot("13_ride_no_gps", "Riding without a GPS fix: wheel stats only, nothing recorded");
}

static void scNmeaOnly() {
  setup();
  gpsOn(GPS_NMEA_NOSATS, 3000, 0);
  runFor(8000);
  shot("08_idle_gps_no_sat_count", "Fix from NMEA GGA without a satellite count: status reads GPS");
}

static void scImperial() {
  writeConfig("wheel_circ_mm=2155\nunits=imperial\nbacklight=bright\n");
  setup();
  gpsOn(GPS_UBX, 8000, 12);
  runFor(10000);
  rampTo(30, 12000);
  rideProfile(31, 3, 400000);
  rampTo(32.5, 3000);
  runFor(2000);
  shot("14_ride_imperial", "units=imperial: mph, mi and ft (gauge stays at 2 km/h per dot)");
}

static void scSaveFailed() {
  g_card.failRename = true;
  setup();
  gpsOn(GPS_UBX, 5000, 10);
  runFor(7000);
  rampTo(22, 8000);
  rideProfile(23, 2, 90000);
  rampTo(0, 6000);
  runFor(4000);
  btn(true);
  runFor(4200);
  shot("24_flash_save_failed", "Card refused the archive rename: ride kept, Save failed");
  btn(false);
  runFor(2500);
}

// Phone switches units from the hub mid-ride: the screen redraws in mph at once.
static void scSettings() {
  setup();
  gpsOn(GPS_UBX, 5000, 12);
  runFor(7000);
  rampTo(24, 10000);
  rideProfile(25, 2, 120000);
  rampTo(0, 6000);
  runFor(4000);
  phoneConnect();
  runFor(300);
  tap();
  runFor(1000);
  phoneSaveSettings([](uint8_t *rec) { rec[7] = UNITS_IMPERIAL; });
  runFor(600);
  shot("32_flash_settings_saved", "Settings saved from the hub: units switch to imperial right away");
  runFor(2000);
  phoneDisconnect();
  runFor(1000);
}

static unsigned long stillMs() { return g_now - g_lastPulseMs; }

// The face reads straight from the panel cells: left eye starts at column 6.
static bool faceLookingLeft() {
  return matShown[0][7] == LV_FG && matShown[3][7] == LV_FG && matShown[4][7] == LV_FG &&
         matShown[3][8] == LV_OFF;
}
static bool faceShut() {
  return matShown[3][6] == LV_FG && matShown[3][8] == LV_FG && matShown[2][6] == LV_OFF &&
         matShown[4][6] == LV_OFF;
}

// Every matrix animation in one ride, as key shots plus frame sequences for
// the animated strips in the README.
static void scAnimations() {
  setup();
  gpsOn(GPS_UBX, 3000, 12);
  runFor(1);  // the first ride-screen frame starts the self-test
  frames("anim_boot", 16, 50, "Boot self-test",
         {{7, "33_anim_boot", "Boot self-test: a diagonal band lights every cell and gauge dot"}});
  runFor(6000);

  rampTo(24, 8000);
  rideProfile(25, 1, 130000);
  runUntil([] { return matShown[0][12] == LV_LO; }, 2000, "heartbeat");
  shot("34_anim_heartbeat", "Wheel heartbeat: the cells above the decimal point glow on each turn");
  frames("anim_heartbeat", 14, 50, "Wheel heartbeat");

  setWheel(33);
  runUntil([] { return clipNewMax.on; }, 20000, "new max");
  frames("anim_new_max", 14, 50, "New max",
         {{4, "35_anim_new_max", "New max after two minutes moving: a comet runs the gauge"}});
  rideProfile(31, 1, 6000);

  // Jump to just short of 10 km so the milestone comes up without a long ride.
  noInterrupts();
  g_revCount = (unsigned long)(9950000.0 / cfg.wheelCircMm);
  interrupts();
  runUntil([] { return clipMilestone.on; }, 30000, "10 km milestone");
  frames("anim_milestone", 58, 50, "10 km milestone while riding",
         {{18, "36_anim_milestone", "Passing 10 km: the distance pops up between sparkles"}});
  rideProfile(30, 1, 4000);

  rampTo(0, 5000);

  runUntil([] { return stillMs() >= FACE_AFTER_MS; }, 20000, "face");
  frames("anim_face_wake", 20, 50, "Face appears",
         {{19, "38_face_awake", "Stopped 8 s mid-ride: the 0.0 becomes a face"}});
  runUntil(faceLookingLeft, 60000, "face looking left");
  shot("39_face_look_left", "The face looks around every few seconds");
  runUntil(faceShut, 60000, "face blink");
  shot("40_face_blink", "and blinks");
  frames("anim_face", 100, 50, "Face looking around and blinking");

  runUntil([] { return stillMs() >= FACE_SLEEPY_MS + 1000; }, 120000, "sleepy face");
  shot("41_face_sleepy", "Stopped for 2 min: heavy eyelids");
  runUntil([] { return stillMs() >= FACE_SLEEPY_MS + FACE_ASLEEP_MS + 1400; }, 60000, "asleep");
  shot("42_face_asleep", "Asleep: a z drifts up and the gauge breathes");
  frames("anim_face_sleep", 40, 100, "Face asleep");

  rampTo(15, 3000);
  rampTo(0, 3000);
  runFor(4000);
  btn(true);
  runFor(2000);
  frames("anim_drain", 40, 50, "Hold for a new ride: the digits drain");
  runUntil([] { return clipFirework.on; }, 1000, "ride saved");
  frames("anim_firework", 30, 50, "Ride saved",
         {{8, "43_anim_firework", "Ride saved: a firework bursts from the centre"},
          {21, "44_anim_drop_in", "then the new 0.0 drops in"}});
  btn(false);
  runFor(3000);

  phoneConnect();
  runFor(100);
  frames("anim_press", 8, 45, "Press to allow");
  btn(true);
  runFor(200);
  btn(false);
  runUntil([] { return clipRune.on; }, 500, "phone allowed");
  frames("anim_rune", 28, 50, "Phone allowed",
         {{16, "45_anim_rune", "Phone allowed: the Bluetooth mark draws itself, then fades"}});
  runFor(1000);
  phoneDisconnect();
  runFor(1000);
}

static void scAnimationsOff() {
  writeConfig("wheel_circ_mm=2155\nanimations=off\n");
  setup();
  rampTo(20, 5000);
  rideProfile(21, 1, 30000);
  rampTo(0, 4000);
  runFor(15000);
  shot("46_animations_off", "animations=off: stopped 15 s, plain digits and no effects");
}

struct Scenario {
  const char *name;
  void (*fn)();
};
static const Scenario SCENARIOS[] = {
    {"boot_new", scBootNew},   {"no_card", scNoCard},     {"resume", scResume},
    {"recording", scRecording}, {"no_gps", scNoGps},       {"nmea_only", scNmeaOnly},
    {"imperial", scImperial},  {"save_failed", scSaveFailed}, {"settings", scSettings},
    {"animations", scAnimations}, {"animations_off", scAnimationsOff},
};

int main(int argc, char **argv) {
  if (argc == 2 && strcmp(argv[1], "--list") == 0) {
    for (const Scenario &s : SCENARIOS) printf("%s\n", s.name);
    return 0;
  }
  if (argc != 3) {
    fprintf(stderr, "usage: %s --list | OUT_DIR SCENARIO\n", argv[0]);
    return 2;
  }
  g_outDir = argv[1];
  g_log = getenv("OBJECT_EMU_LOG") != nullptr;
  for (const Scenario &s : SCENARIOS) {
    if (strcmp(argv[2], s.name) == 0) {
      s.fn();
      return 0;
    }
  }
  fprintf(stderr, "unknown scenario %s\n", argv[2]);
  return 2;
}
