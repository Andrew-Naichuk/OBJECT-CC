// Host-side regressions for audit F1–F4 helpers (no Arduino runtime).
// Build: c++ -std=c++17 -O0 -Wall -Wextra -o audit_regressions audit_regressions.cpp
// Run:   ./audit_regressions

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

static int g_fails = 0;

static void expect(bool cond, const char *msg) {
  if (!cond) {
    std::fprintf(stderr, "FAIL: %s\n", msg);
    g_fails++;
  } else {
    std::printf("ok: %s\n", msg);
  }
}

// --- Simulated FAT names (8.3 upper) ---
static std::set<std::string> g_files;
static std::map<std::string, std::string> g_contents;

static bool sd_exists(const char *name) {
  return g_files.count(name) != 0;
}

static void sd_create(const char *name, const std::string &body = "") {
  g_files.insert(name);
  g_contents[name] = body;
}

static void sd_reset() {
  g_files.clear();
  g_contents.clear();
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
  if (sd_exists(gpxName)) {
    return false;
  }
  char stem[9];
  sumStemFromGpx(gpxName, stem, sizeof(stem));
  if (stem[0] == '\0') {
    return false;
  }
  char sumName[13];
  std::snprintf(sumName, sizeof(sumName), "%s.SUM", stem);
  return !sd_exists(sumName);
}

static bool tripPickArchiveName(char *buf, size_t buflen, bool timeKnown,
                                unsigned yy, unsigned mo, unsigned dd, unsigned hh) {
  if (buflen < 13) {
    return false;
  }
  if (timeKnown) {
    std::snprintf(buf, buflen, "%02u%02u%02u%02u.GPX", yy, mo, dd, hh);
    if (tripStemFree(buf)) {
      return true;
    }
    for (unsigned seq = 24; seq < 100; seq++) {
      std::snprintf(buf, buflen, "%02u%02u%02u%02u.GPX", yy, mo, dd, seq);
      if (tripStemFree(buf)) {
        return true;
      }
    }
  }
  for (unsigned n = 1; n <= 9999; n++) {
    std::snprintf(buf, buflen, "RIDE%04u.GPX", n);
    if (tripStemFree(buf)) {
      return true;
    }
  }
  return false;
}

// F3-style independent flags.
struct PersistFlags {
  bool track = true;
  bool check = true;
  bool archive = true;
  void noteTrack(bool ok) { track = ok; }
  void noteCheck(bool ok) { check = ok; }
  void noteArchive(bool ok) { archive = ok; }
  bool anyFailed() const { return !track || !check || !archive; }
  bool recordingOk() const { return track; }
};

// F4-style ISR accumulator.
struct MovingAcc {
  unsigned long lastPulseMs = 0;
  unsigned long acc = 0;
  unsigned long movingMs = 0;
  static const unsigned long MIN_REV_MS = 78;
  static const unsigned long STOPPED_MS = 3000;

  void pulse(unsigned long now) {
    if (lastPulseMs != 0) {
      unsigned long dt = now - lastPulseMs;
      if (dt >= MIN_REV_MS && dt < STOPPED_MS) {
        acc += dt;
      }
    }
    lastPulseMs = now;
  }

  void drain() {
    movingMs += acc;
    acc = 0;
  }
};

// F2-style dual-slot zero + rideGen.
struct TripSlot {
  uint32_t seq = 0;
  uint32_t rev = 0;
  uint32_t rideGen = 1;
  bool valid = false;
};

static TripSlot loadNewest(const TripSlot &a, const TripSlot &b) {
  if (a.valid && b.valid) {
    return (a.seq >= b.seq) ? a : b;
  }
  if (a.valid) {
    return a;
  }
  return b;
}

static void zeroBoth(TripSlot &a, TripSlot &b, uint32_t &seq, uint32_t newGen) {
  uint32_t s1 = seq + 1;
  uint32_t s2 = s1 + 1;
  TripSlot z{};
  z.valid = true;
  z.rev = 0;
  z.rideGen = newGen;
  z.seq = s1;
  if (s1 & 1) {
    a = z;
  } else {
    b = z;
  }
  z.seq = s2;
  if (s2 & 1) {
    a = z;
  } else {
    b = z;
  }
  seq = s2;
}

int main() {
  // F1A: consecutive no-GPS summaries must not collide on the same stem.
  {
    sd_reset();
    char dest[13];
    expect(tripPickArchiveName(dest, sizeof(dest), false, 0, 0, 0, 0), "first RIDE stem");
    expect(std::strcmp(dest, "RIDE0001.GPX") == 0, "first stem RIDE0001");
    char stem[9];
    sumStemFromGpx(dest, stem, sizeof(stem));
    char sum1[13];
    std::snprintf(sum1, sizeof(sum1), "%s.SUM", stem);
    sd_create(sum1, "rev=10\n");
    expect(tripPickArchiveName(dest, sizeof(dest), false, 0, 0, 0, 0), "second RIDE stem");
    expect(std::strcmp(dest, "RIDE0002.GPX") == 0, "second stem skips SUM-only RIDE0001");
  }

  // F1A: date stem with only .SUM is not reused.
  {
    sd_reset();
    sd_create("26093010.SUM", "rev=1\n");
    char dest[13];
    expect(tripPickArchiveName(dest, sizeof(dest), true, 26, 9, 30, 10),
           "date stem with SUM-only");
    expect(std::strcmp(dest, "26093010.GPX") != 0, "does not reuse SUM-only hour stem");
  }

  // F3: checkpoint success must not clear a track failure.
  {
    PersistFlags f;
    f.noteTrack(false);
    f.noteCheck(true);
    expect(f.anyFailed(), "track fail still unhealthy");
    expect(!f.recordingOk(), "Recording gated on track flag");
    f.noteTrack(true);
    expect(!f.anyFailed(), "track recovery clears track error");
    f.noteArchive(false);
    expect(f.anyFailed(), "archive fail stays visible");
    expect(f.recordingOk(), "track can be ok while archive failed");
  }

  // F4: three one-second intervals between drains accumulate 3000 ms.
  {
    MovingAcc m;
    m.pulse(1000);
    m.pulse(2000);
    m.pulse(3000);
    m.pulse(4000);
    m.drain();
    expect(m.movingMs == 3000, "three intervals add 3000 ms");
    m.drain();
    expect(m.movingMs == 3000, "re-drain without pulses adds nothing");
  }

  // F2: dual-slot zero prevents older slot from reviving archived counters.
  {
    TripSlot a{};
    TripSlot b{};
    a.valid = true;
    a.seq = 9;
    a.rev = 100;
    a.rideGen = 1;
    b.valid = true;
    b.seq = 8;
    b.rev = 100;
    b.rideGen = 1;
    uint32_t seq = 9;
    zeroBoth(a, b, seq, 2);
    TripSlot n = loadNewest(a, b);
    expect(n.rev == 0 && n.rideGen == 2, "newest after finish is zeroed new gen");
    // Invalidate newest: still get the other zeroed slot, not old ride.
    if (a.seq > b.seq) {
      a.valid = false;
    } else {
      b.valid = false;
    }
    n = loadNewest(a, b);
    expect(n.valid && n.rev == 0 && n.rideGen == 2,
           "fallback slot cannot revive completed ride");
  }

  if (g_fails) {
    std::fprintf(stderr, "%d failure(s)\n", g_fails);
    return 1;
  }
  std::printf("all host firmware regressions passed\n");
  return 0;
}
