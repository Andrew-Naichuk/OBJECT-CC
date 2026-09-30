// Host regressions for the ride-identity finish transaction and protocol v2 parsing.
// Build: c++ -std=c++17 -O0 -Wall -Wextra -I ../../xiao_oled -o finish_txn_test finish_txn_test.cpp
// Run from this directory, or pass no args; the header is found via -I.

#include "finish_txn.h"
#include "ble_proto.h"

#include <cstdio>
#include <cstring>
#include <map>
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

enum {
  FAULT_NONE = 0,
  FAULT_WRITE_COMMIT,  // full bytes land, write() returns failure
  FAULT_WRITE_TRUNC,   // short bytes land, write() returns failure
  FAULT_WRITE_DROP,    // nothing lands
  FAULT_READ,
  FAULT_REMOVE,
  FAULT_REMOVE_JOURNAL,
  FAULT_RENAME,
  FAULT_CONT_CREATE
};

struct Card {
  std::map<std::string, std::vector<uint8_t>> files;
  int fault = FAULT_NONE;
  int faultLeft = 0;
  int removes = 0;
  int gpxRemoves = 0;

  int stat(const char *name) const {
    return files.count(name) ? 1 : 0;
  }

  int read(const char *name, void *buf, uint32_t cap, uint32_t *len) {
    if (fault == FAULT_READ && faultLeft > 0) {
      faultLeft--;
      return -1;
    }
    auto it = files.find(name);
    if (it == files.end()) {
      return -1;
    }
    *len = (uint32_t)it->second.size();
    uint32_t n = *len < cap ? *len : cap;
    if (n && buf) {
      memcpy(buf, it->second.data(), n);
    }
    return 0;
  }

  int write(const char *name, const void *data, uint32_t len, int excl) {
    bool cont = name[0] == 'C' && std::strstr(name, ".GPX");
    if (fault == FAULT_CONT_CREATE && cont) {
      return -1;
    }
    if (faultLeft > 0 && (fault == FAULT_WRITE_COMMIT || fault == FAULT_WRITE_TRUNC ||
                          fault == FAULT_WRITE_DROP)) {
      faultLeft--;
      if (fault == FAULT_WRITE_DROP) {
        return -1;
      }
      if (excl && files.count(name)) {
        return -1;
      }
      uint32_t n = (fault == FAULT_WRITE_TRUNC) ? (len > 4 ? 4 : len) : len;
      files[name].assign((const uint8_t *)data, (const uint8_t *)data + n);
      return -1;
    }
    if (excl && files.count(name)) {
      return -1;
    }
    files[name].assign((const uint8_t *)data, (const uint8_t *)data + len);
    return 0;
  }

  int rename(const char *from, const char *to) {
    if (fault == FAULT_RENAME) {
      return -1;
    }
    auto it = files.find(from);
    if (it == files.end() || files.count(to)) {
      return -1;
    }
    files[to] = it->second;
    files.erase(it);
    return 0;
  }

  int remove(const char *name) {
    bool journal = std::strncmp(name, "FINISH", 6) == 0;
    if (fault == FAULT_REMOVE || (fault == FAULT_REMOVE_JOURNAL && journal)) {
      return -1;
    }
    auto it = files.find(name);
    if (it == files.end()) {
      return 0;
    }
    removes++;
    if (std::strstr(name, ".GPX") || std::strstr(name, ".ID")) {
      gpxRemoves++;
    }
    files.erase(it);
    return 0;
  }

  bool has(const char *name) const { return files.count(name) != 0; }

  std::string text(const char *name) const {
    auto it = files.find(name);
    if (it == files.end()) {
      return "";
    }
    return std::string(it->second.begin(), it->second.end());
  }
};

static int fsStat(void *ctx, const char *name) { return ((Card *)ctx)->stat(name); }
static int fsRead(void *ctx, const char *name, void *buf, uint32_t cap, uint32_t *len) {
  return ((Card *)ctx)->read(name, buf, cap, len);
}
static int fsWrite(void *ctx, const char *name, const void *data, uint32_t len, int excl) {
  return ((Card *)ctx)->write(name, data, len, excl);
}
static int fsRename(void *ctx, const char *from, const char *to) {
  return ((Card *)ctx)->rename(from, to);
}
static int fsRemove(void *ctx, const char *name) { return ((Card *)ctx)->remove(name); }

static FinishFs fsOf(Card *c) {
  FinishFs fs{};
  fs.ctx = c;
  fs.stat = fsStat;
  fs.read = fsRead;
  fs.write = fsWrite;
  fs.rename = fsRename;
  fs.remove = fsRemove;
  return fs;
}

static const char *kHdr = "<hdr>";
static const char *kFtr = "<ftr>";

static FinishCfg testCfg() {
  FinishCfg cfg{};
  cfg.gpxHeader = kHdr;
  cfg.gpxFooter = kFtr;
  cfg.wheelMm = 1000.0f;
  return cfg;
}

static FinishSnap makeSnap(uint32_t gen, const char *dest, bool points, uint32_t rev) {
  FinishSnap s{};
  s.revCount = rev;
  s.movingMs = rev * 1000;
  s.elapsedMs = rev * 1500;
  s.maxSpeedKmh = 20.0f;
  s.points = points ? rev : 0;
  s.fromRideGen = gen;
  s.gpxBodyEnd = points ? 40 : (uint32_t)std::strlen(kHdr);
  s.hasPoints = points ? 1 : 0;
  s.needSum = (points || rev > 0) ? 1 : 0;
  s.endTimeKnown = 1;
  s.endYear = 2026;
  s.endMonth = 9;
  s.endDay = 30;
  s.endHour = 10;
  s.endMin = 11;
  s.endSec = 12;
  if (dest) {
    std::snprintf(s.dest, sizeof(s.dest), "%s", dest);
  }
  return s;
}

static void putBytes(Card *c, const char *name, const char *text) {
  c->files[name] = std::vector<uint8_t>(text, text + std::strlen(text));
}

static void putRaw(Card *c, const char *name, const void *data, uint32_t len) {
  c->files[name].assign((const uint8_t *)data, (const uint8_t *)data + len);
}

static void putTrip(Card *c, const char *name, uint32_t seq, uint32_t rev, uint32_t gen) {
  TripDat d{};
  d.seq = seq;
  d.revCount = rev;
  d.movingMs = rev;
  d.elapsedMs = rev;
  d.maxSpeedKmh = 10.0f;
  d.gpxBodyEnd = 20;
  d.rideGen = gen;
  tripDatSeal(&d);
  putRaw(c, name, &d, sizeof(d));
}

static bool readTrip(Card *c, const char *name, TripDat *out) {
  auto it = c->files.find(name);
  if (it == c->files.end() || it->second.size() != sizeof(TripDat)) {
    return false;
  }
  memcpy(out, it->second.data(), sizeof(TripDat));
  return tripDatValidate(out);
}

static void putSidecar(Card *c, uint32_t gen, uint32_t bodyEnd) {
  char gpx[13];
  char idn[13];
  finishGenName(gen, 'C', "GPX", gpx, sizeof(gpx));
  finishGenName(gen, 'G', "ID", idn, sizeof(idn));
  RideSidecar s{};
  s.rideGen = gen;
  std::snprintf(s.gpxName, sizeof(s.gpxName), "%s", gpx);
  s.gpxBodyEnd = bodyEnd;
  rideSidecarSeal(&s);
  putRaw(c, idn, &s, sizeof(s));
}

static void putV1(Card *c, const FinishDatV1 *in) {
  FinishDatV1 d = *in;
  finishSealV1(&d);
  putRaw(c, FINISH_FILE_V1, &d, sizeof(d));
}

static void putV2(Card *c, const char *slot, FinishDatV2 *d) {
  finishSealV2(d);
  putRaw(c, slot, d, sizeof(*d));
}

static int beginRun(Card *c, FinishTxn *t, const FinishSnap *snap) {
  FinishFs fs = fsOf(c);
  FinishCfg cfg = testCfg();
  finishTxnInit(t);
  int code = finishTxnBegin(t, &fs, &cfg, snap);
  if (code != FINISH_YIELD) {
    return code;
  }
  return finishTxnRun(t, &fs, &cfg);
}

static int boot(Card *c, FinishTxn *t) {
  FinishFs fs = fsOf(c);
  FinishCfg cfg = testCfg();
  finishTxnInit(t);
  return finishTxnBoot(t, &fs, &cfg);
}

static bool journalPresent(Card *c) {
  return c->has(FINISH_FILE_A) || c->has(FINISH_FILE_B) || c->has(FINISH_FILE_V1);
}

static void testNames() {
  char buf[13];
  expect(finishGenName(0x2A, 'C', "GPX", buf, sizeof(buf)), "gen name fits");
  expect(std::strcmp(buf, "C000002A.GPX") == 0, "generation 0x2A is C000002A.GPX");
  expect(!finishGenName(0x10000000u, 'C', "GPX", buf, sizeof(buf)), "gen above 28 bits is refused");
  expect(finishNextGen(0x0FFFFFFFu) == 0, "next gen does not wrap the name");
  expect(finishNextGen(0) == 1, "generation skips 0");
}

static void testAdoptExistingContinuation() {
  Card card;
  FinishDatV2 rec{};
  rec.phase = FINISH_PHASE_ARCHIVING;
  std::snprintf(rec.dest, sizeof(rec.dest), "26093010.GPX");
  rec.hasPoints = 1;
  rec.needSum = 0;
  rec.revCount = 12;
  rec.fromRideGen = 1;
  rec.seq = 1;
  putV2(&card, FINISH_FILE_A, &rec);
  putBytes(&card, "26093010.GPX", "OLD-RIDE");
  char cont[13];
  finishGenName(2, 'C', "GPX", cont, sizeof(cont));
  putBytes(&card, cont, "CONTDATA-KEEP");
  putSidecar(&card, 2, 12);
  int removesBefore = card.gpxRemoves;
  FinishTxn txn;
  int code = boot(&card, &txn);
  expect(code == FINISH_SAVED || code == FINISH_NO_GPS, "adopt path finishes");
  expect(card.text(cont) == "CONTDATA-KEEP", "continuation is not truncated");
  expect(card.text("26093010.GPX") == "OLD-RIDE", "continuation is not archived as the old ride");
  expect(card.has(cont), "continuation file stays");
  expect(card.gpxRemoves == removesBefore, "adopt does not delete gpx or sidecar");
}

static void testInvalidJournalKeepsContinuation() {
  Card card;
  putBytes(&card, FINISH_FILE_A, "torn");
  putBytes(&card, FINISH_FILE_B, "xx");
  char cont[13];
  finishGenName(2, 'C', "GPX", cont, sizeof(cont));
  putBytes(&card, cont, "KEEP-CONT");
  putTrip(&card, FINISH_TRIP_A, 3, 99, 2);
  int filesBefore = (int)card.files.size();
  FinishTxn txn;
  int code = boot(&card, &txn);
  expect(code == FINISH_ERROR, "invalid journal with continuation is Finish error");
  expect(!txn.out.recordGps, "invalid journal does not resume ordinary recording");
  expect(card.text(cont) == "KEEP-CONT", "continuation is not deleted");
  expect(card.has(FINISH_FILE_A) && card.has(FINISH_FILE_B), "invalid journals are kept");
  TripDat d;
  expect(readTrip(&card, FINISH_TRIP_A, &d) && d.revCount == 99 && d.rideGen == 2,
         "counters are not reset");
  expect((int)card.files.size() == filesBefore, "no replacement track is created");
}

static void testV1Cases() {
  // Torn v2 beside a valid automatic v1 still recovers from v1.
  {
    Card card;
    putBytes(&card, FINISH_FILE_A, "torn-v2");
    putBytes(&card, "CURRENT.GPX", "LEGACY-POINTS");
    FinishDatV1 v1{};
    v1.phase = FINISH_PHASE_ARCHIVING;
    std::snprintf(v1.dest, sizeof(v1.dest), "26093010.GPX");
    v1.hasPoints = 1;
    v1.needSum = 1;
    v1.revCount = 8;
    v1.points = 2;
    v1.fromRideGen = 1;
    putV1(&card, &v1);
    FinishTxn txn;
    int code = boot(&card, &txn);
    expect(code == FINISH_SAVED, "torn v2 uses automatic v1");
    expect(!card.has("CURRENT.GPX"), "v1 archive renames CURRENT.GPX");
    expect(card.text("26093010.GPX") == "LEGACY-POINTS", "v1 keeps the archived bytes");
    char cont[13];
    finishGenName(2, 'C', "GPX", cont, sizeof(cont));
    expect(card.has(cont), "v1 recovery creates the continuation");
    expect(!journalPresent(&card), "v1 journal retires after recovery");
  }
  // v1 with both dest and CURRENT.GPX is not finished and nothing is deleted.
  {
    Card card;
    putBytes(&card, "CURRENT.GPX", "STILL");
    putBytes(&card, "26093010.GPX", "DEST");
    FinishDatV1 v1{};
    v1.phase = FINISH_PHASE_ARCHIVING;
    std::snprintf(v1.dest, sizeof(v1.dest), "26093010.GPX");
    v1.hasPoints = 1;
    v1.needSum = 1;
    v1.revCount = 3;
    v1.fromRideGen = 1;
    putV1(&card, &v1);
    int n = (int)card.files.size();
    FinishTxn txn;
    int code = boot(&card, &txn);
    expect(code == FINISH_ERROR, "v1 with dest and CURRENT is Finish error");
    expect(card.text("CURRENT.GPX") == "STILL" && card.text("26093010.GPX") == "DEST",
           "v1 ambiguous files are not deleted");
    expect((int)card.files.size() == n, "v1 ambiguous case creates nothing");
  }
  // Non-automatic v1 (continuation already present, dest missing) is preserved.
  {
    Card card;
    char cont[13];
    finishGenName(2, 'C', "GPX", cont, sizeof(cont));
    putBytes(&card, cont, "NEXT");
    FinishDatV1 v1{};
    v1.phase = FINISH_PHASE_ARCHIVING;
    std::snprintf(v1.dest, sizeof(v1.dest), "26093010.GPX");
    v1.hasPoints = 1;
    v1.needSum = 1;
    v1.fromRideGen = 1;
    putV1(&card, &v1);
    FinishTxn txn;
    int code = boot(&card, &txn);
    expect(code == FINISH_ERROR, "v1 dest missing beside continuation is preserved");
    expect(card.has(cont) && card.has(FINISH_FILE_V1), "v1 preserve deletes nothing");
  }
}

static void testNoGpsPulses() {
  Card card;
  FinishSnap snap = makeSnap(1, "RIDE0001.GPX", false, 10);
  FinishTxn txn;
  FinishFs fs = fsOf(&card);
  FinishCfg cfg = testCfg();
  finishTxnInit(&txn);
  int code = finishTxnBegin(&txn, &fs, &cfg, &snap);
  expect(code == FINISH_YIELD, "no-gps journal commits");
  FinishContRam cont{};
  cont.rev = 4;
  cont.movingMs = 4000;
  cont.elapsedMs = 5000;
  cont.maxSpeed = 8.0f;
  finishTxnNoteCont(&txn, &cont);
  expect(finishTxnSyncCont(&txn, &fs) == 0, "continuation pulses sync");
  FinishLoad ld;
  expect(finishLoadJournal(&fs, &ld) == LD_V2 && ld.rec.contRev == 4,
         "last synced journal slot keeps the extra pulses");
  expect(ld.rec.revCount == 10, "journal snapshot stays the original ride");
  code = finishTxnRun(&txn, &fs, &cfg);
  expect(code == FINISH_NO_GPS, "no-gps finish completes");
  expect(!card.has("RIDE0001.GPX"), "no-gps finish does not require an archive GPX");
  expect(card.text("RIDE0001.SUM").find("rev=10\n") != std::string::npos,
         "summary keeps the original revolution count");
  expect(card.text("RIDE0001.SUM").find("end=2026-09-30T10:11:12Z") != std::string::npos,
         "summary uses the original end time");
  TripDat a, b;
  bool ha = readTrip(&card, FINISH_TRIP_A, &a);
  bool hb = readTrip(&card, FINISH_TRIP_B, &b);
  const TripDat *n = (ha && hb) ? (a.seq >= b.seq ? &a : &b) : (ha ? &a : &b);
  expect((ha || hb) && n->revCount == 4 && n->rideGen == 2,
         "checkpoint keeps continuation pulses");
}

static void testRenameAndCreateFail() {
  {
    Card card;
    putBytes(&card, "CURRENT.GPX", "POINTS");
    card.fault = FAULT_RENAME;
    FinishSnap snap = makeSnap(1, "26093010.GPX", true, 5);
    FinishTxn txn;
    int code = beginRun(&card, &txn, &snap);
    expect(code == FINISH_PENDING, "rename failure stays pending");
    expect(!txn.out.recordGps && txn.out.showPending, "rename failure is not recording");
    expect(card.has("CURRENT.GPX"), "failed rename leaves the source");
  }
  {
    Card card;
    putBytes(&card, "CURRENT.GPX", "POINTS");
    card.fault = FAULT_CONT_CREATE;
    FinishSnap snap = makeSnap(1, "26093010.GPX", true, 5);
    FinishTxn txn;
    int code = beginRun(&card, &txn, &snap);
    expect(code == FINISH_PENDING, "continuation create failure stays pending");
    expect(!txn.out.recordGps, "create failure is not recording");
    char cont[13];
    finishGenName(2, 'C', "GPX", cont, sizeof(cont));
    expect(!card.has(cont), "failed create does not leave a continuation");
  }
}

static void testThreeRides() {
  Card card;
  putBytes(&card, "CURRENT.GPX", "RIDE-1");
  const char *dests[] = {"26093010.GPX", "26093011.GPX", "26093012.GPX"};
  uint32_t gen = 1;
  for (int i = 0; i < 3; i++) {
    FinishSnap snap = makeSnap(gen, dests[i], true, (uint32_t)(10 + i));
    FinishTxn txn;
    int code = beginRun(&card, &txn, &snap);
    expect(code == FINISH_SAVED, "ride finish completes");
    FinishTxn again;
    int bootCode = boot(&card, &again);
    expect(bootCode == FINISH_ABSENT, "reboot finds no journal");
    char cur[13];
    FinishFs fs = fsOf(&card);
    expect(finishActiveTrackName(&fs, gen + 1, cur, sizeof(cur)), "active name");
    char expectName[13];
    finishGenName(gen + 1, 'C', "GPX", expectName, sizeof(expectName));
    expect(std::strcmp(cur, expectName) == 0, "current file is the checkpoint generation");
    if (i == 0) {
      expect(!card.has("CURRENT.GPX"), "first finish archives CURRENT.GPX");
      expect(card.text(dests[0]) == "RIDE-1", "first archive is the legacy track");
    } else {
      expect(card.text(dests[i]).find("GENFILE") != std::string::npos,
             "later finish archives that generation file");
    }
    gen++;
    char next[13];
    finishGenName(gen, 'C', "GPX", next, sizeof(next));
    card.files[next].insert(card.files[next].end(), {'G', 'E', 'N', 'F', 'I', 'L', 'E'});
  }
  expect(!card.has("CURRENT.GPX"), "no leftover CURRENT.GPX after three finishes");
  TripDat a, b;
  bool ha = readTrip(&card, FINISH_TRIP_A, &a);
  bool hb = readTrip(&card, FINISH_TRIP_B, &b);
  const TripDat *n = (ha && hb) ? (a.seq >= b.seq ? &a : &b) : (ha ? &a : &b);
  expect((ha || hb) && n->rideGen == 4, "after the last reboot the checkpoint generation is current");
  char only[13];
  finishGenName(4, 'C', "GPX", only, sizeof(only));
  int currents = 0;
  if (card.has(only)) {
    currents++;
  }
  if (card.has("CURRENT.GPX")) {
    currents++;
  }
  expect(currents == 1 && card.has(only), "exactly one current ride file");
  expect(card.has("26093010.GPX") && card.has("26093011.GPX") && card.has("26093012.GPX"),
         "three archives stay separate");
  expect(card.has("26093010.SUM") && card.has("26093011.SUM") && card.has("26093012.SUM"),
         "three summaries stay separate");
}

static void testFirstJournalReconcile() {
  FinishSnap snap = makeSnap(1, "26093010.GPX", true, 15);
  // Sync reports failure but a re-read is a valid record: do not roll back.
  {
    Card card;
    putBytes(&card, "CURRENT.GPX", "P");
    card.fault = FAULT_WRITE_COMMIT;
    card.faultLeft = 1;
    FinishTxn txn;
    FinishFs fs = fsOf(&card);
    FinishCfg cfg = testCfg();
    finishTxnInit(&txn);
    int code = finishTxnBegin(&txn, &fs, &cfg, &snap);
    expect(code == FINISH_YIELD && !txn.out.folded, "valid re-read does not roll back");
    expect(journalPresent(&card), "committed journal stays");
  }
  // Truncated record can be removed: fold the post-cut pulses back.
  {
    Card card;
    putBytes(&card, "CURRENT.GPX", "P");
    putTrip(&card, FINISH_TRIP_A, 2, 15, 1);
    card.fault = FAULT_WRITE_TRUNC;
    card.faultLeft = 1;
    FinishTxn txn;
    FinishFs fs = fsOf(&card);
    FinishCfg cfg = testCfg();
    finishTxnInit(&txn);
    int code = finishTxnBegin(&txn, &fs, &cfg, &snap);
    expect(code == FINISH_ROLLED_BACK && txn.out.folded, "invalid first journal rolls back");
    expect(txn.out.foldedFrom.revCount == 15, "fold restores the original snapshot");
    expect(!journalPresent(&card), "invalid journal records are gone");
    // Further riding, then a reboot, resumes that checkpoint and does not replay.
    TripDat d;
    expect(readTrip(&card, FINISH_TRIP_A, &d), "original checkpoint remains");
    d.revCount = 22;
    d.seq = 3;
    tripDatSeal(&d);
    putRaw(&card, FINISH_TRIP_A, &d, sizeof(d));
    FinishTxn later;
    int bootCode = boot(&card, &later);
    expect(bootCode == FINISH_ABSENT, "reboot after rollback does not replay a finish");
    expect(!later.out.showError, "rollback reboot is not Finish error");
    TripDat now;
    expect(readTrip(&card, FINISH_TRIP_A, &now) && now.revCount == 22 && now.rideGen == 1,
           "reboot resumes the original ride");
  }
  // Invalid bytes that cannot be removed stay pending, with no ordinary recording.
  {
    Card card;
    putBytes(&card, "CURRENT.GPX", "P");
    card.fault = FAULT_WRITE_TRUNC;
    card.faultLeft = 1;
    FinishTxn txn;
    FinishFs fs = fsOf(&card);
    FinishCfg cfg = testCfg();
    finishTxnInit(&txn);
    // Remove will fail once the truncated file exists. Arm it before begin.
    card.fault = FAULT_WRITE_TRUNC;
    card.faultLeft = 1;
    int code = finishTxnBegin(&txn, &fs, &cfg, &snap);
    // The trunc fault is consumed by the write. Remove is not failed in this setup
    // unless we switch fault. Re-run with a card whose remove fails and whose
    // journal is already invalid.
    (void)code;
  }
  {
    Card card;
    putBytes(&card, FINISH_FILE_A, "bad");
    putBytes(&card, "CURRENT.GPX", "P");
    card.fault = FAULT_REMOVE;
    FinishTxn txn;
    FinishFs fs = fsOf(&card);
    FinishCfg cfg = testCfg();
    finishTxnInit(&txn);
    txn.liveAttempt = true;
    txn.haveAttempt = true;
    txn.committed = false;
    txn.attempt = snap;
    int code = finishTxnStep(&txn, &fs, &cfg);
    expect(code == FINISH_PENDING, "unremovable invalid journal stays pending");
    expect(!txn.out.recordGps, "unremovable invalid journal is not recording");
    expect(card.has(FINISH_FILE_A), "invalid journal remains");
  }
  // The confirming re-read fails.
  {
    Card card;
    putBytes(&card, "CURRENT.GPX", "P");
    card.fault = FAULT_READ;
    card.faultLeft = 1;
    FinishTxn txn;
    FinishFs fs = fsOf(&card);
    FinishCfg cfg = testCfg();
    finishTxnInit(&txn);
    int code = finishTxnBegin(&txn, &fs, &cfg, &snap);
    expect(code == FINISH_PENDING || code == FINISH_ERROR, "failed re-read does not resume");
    expect(!txn.out.recordGps && !txn.out.folded, "failed re-read does not fold pulses back");
  }
}

static void testBootInvalidNoAttempt() {
  Card card;
  putBytes(&card, FINISH_FILE_A, "garbage");
  putBytes(&card, "CURRENT.GPX", "ORIG");
  putTrip(&card, FINISH_TRIP_A, 4, 30, 1);
  FinishTxn txn;
  int code = boot(&card, &txn);
  expect(code == FINISH_ERROR, "boot invalid journal is Finish error");
  expect(card.text("CURRENT.GPX") == "ORIG" && card.has(FINISH_FILE_A), "boot keeps every file");
  TripDat d;
  expect(readTrip(&card, FINISH_TRIP_A, &d) && d.revCount == 30, "boot does not zero the original ride");
  expect(!txn.out.folded, "boot does not treat the journal as cancelled");
}

static void testCheckpointRepair() {
  FinishDatV2 rec{};
  rec.phase = FINISH_PHASE_ARCHIVING;
  std::snprintf(rec.dest, sizeof(rec.dest), "26093010.GPX");
  rec.hasPoints = 1;
  rec.needSum = 1;
  rec.revCount = 9;
  rec.fromRideGen = 1;
  rec.seq = 2;
  rec.contRev = 10;
  rec.contMovingMs = 10;
  rec.contElapsedMs = 10;
  rec.contMaxSpeed = 10;
  rec.contGpxEnd = 12;
  rec.endTimeKnown = 1;
  rec.endYear = 2026;
  rec.endMonth = 9;
  rec.endDay = 30;
  rec.endHour = 10;
  rec.endMin = 11;
  rec.endSec = 12;

  {
    Card card;
    FinishDatV2 copy = rec;
    putV2(&card, FINISH_FILE_A, &copy);
    putBytes(&card, "26093010.GPX", "OLD");
    char cont[13];
    finishGenName(2, 'C', "GPX", cont, sizeof(cont));
    putBytes(&card, cont, "NEWER");
    putSidecar(&card, 2, 12);
    putTrip(&card, FINISH_TRIP_A, 5, 10, 2);
    putTrip(&card, FINISH_TRIP_B, 4, 100, 1);
    FinishTxn txn;
    FinishFs fs = fsOf(&card);
    FinishCfg cfg = testCfg();
    finishTxnInit(&txn);
    int code = finishTxnStep(&txn, &fs, &cfg);
    expect(code == FINISH_YIELD, "repair writes the other slot first");
    expect(journalPresent(&card), "journal stays until both slots match");
    expect(txn.out.revCount == 10, "newer counters are resumed");
    TripDat b;
    expect(readTrip(&card, FINISH_TRIP_B, &b) && b.rideGen == 2 && b.revCount == 10,
           "other slot is copied from the newer counters");
    code = finishTxnRun(&txn, &fs, &cfg);
    expect(code == FINISH_SAVED, "delete is allowed after the repair");
    expect(!journalPresent(&card), "journal retires after both slots match");
  }
  {
    Card card;
    FinishDatV2 copy = rec;
    putV2(&card, FINISH_FILE_A, &copy);
    putBytes(&card, "26093010.GPX", "OLD");
    char cont[13];
    finishGenName(2, 'C', "GPX", cont, sizeof(cont));
    putBytes(&card, cont, "NEWER");
    putSidecar(&card, 2, 12);
    putBytes(&card, FINISH_TRIP_A, "bad-slot");
    putTrip(&card, FINISH_TRIP_B, 4, 100, 1);
    FinishTxn txn;
    int code = boot(&card, &txn);
    expect(code == FINISH_SAVED, "invalidated newer slot rebuilds from the journal");
    TripDat a, b;
    expect(readTrip(&card, FINISH_TRIP_A, &a) && readTrip(&card, FINISH_TRIP_B, &b),
           "both slots rewritten");
    expect(a.rideGen == 2 && b.rideGen == 2 && a.revCount == 10 && b.revCount == 10,
           "rebuild uses the journal continuation, not the completed ride");
  }
}

static void testStaleJournal() {
  Card card;
  FinishDatV2 rec{};
  rec.phase = FINISH_PHASE_CHECKPOINT_DONE;
  std::snprintf(rec.dest, sizeof(rec.dest), "26093010.GPX");
  rec.hasPoints = 1;
  rec.needSum = 1;
  rec.revCount = 4;
  rec.fromRideGen = 1;
  rec.seq = 3;
  rec.contRev = 1;
  rec.contGpxEnd = 8;
  rec.endTimeKnown = 1;
  rec.endYear = 2026;
  rec.endMonth = 1;
  rec.endDay = 2;
  rec.endHour = 3;
  rec.endMin = 4;
  rec.endSec = 5;
  putV2(&card, FINISH_FILE_A, &rec);
  putTrip(&card, FINISH_TRIP_A, 9, 80, 6);
  putTrip(&card, FINISH_TRIP_B, 8, 80, 6);
  card.fault = FAULT_REMOVE_JOURNAL;
  FinishTxn txn;
  int code = boot(&card, &txn);
  expect(code == FINISH_PENDING, "undeletable stale journal stays");
  expect(txn.out.rideGen == 6 && txn.out.revCount == 80, "stale journal does not downgrade counters");
  TripDat a, b;
  expect(readTrip(&card, FINISH_TRIP_A, &a) && a.rideGen == 6 && a.revCount == 80, "slot A unchanged");
  expect(readTrip(&card, FINISH_TRIP_B, &b) && b.rideGen == 6 && b.revCount == 80, "slot B unchanged");
  expect(journalPresent(&card), "stale journal remains on the card");
}

static void testPowerLossAndSummaryTime() {
  Card origin;
  putBytes(&origin, "CURRENT.GPX", "TRACK-BYTES");
  FinishSnap snap = makeSnap(1, "26093010.GPX", true, 6);
  FinishTxn live;
  FinishFs fs = fsOf(&origin);
  FinishCfg cfg = testCfg();
  finishTxnInit(&live);
  expect(finishTxnBegin(&live, &fs, &cfg, &snap) == FINISH_YIELD, "power-loss setup commits");
  std::vector<Card> snaps;
  snaps.push_back(origin);
  for (int i = 0; i < 20; i++) {
    int code = finishTxnStep(&live, &fs, &cfg);
    snaps.push_back(origin);
    if (code != FINISH_YIELD) {
      break;
    }
  }
  expect(snaps.size() > 2, "power loss has intermediate cards");
  for (size_t i = 0; i < snaps.size(); i++) {
    Card card = snaps[i];
    std::string contBefore;
    char cont[13];
    finishGenName(2, 'C', "GPX", cont, sizeof(cont));
    bool hadCont = card.has(cont);
    if (hadCont) {
      contBefore = card.text(cont);
    }
    bool hadJournal = journalPresent(&card);
    FinishTxn txn;
    int code = boot(&card, &txn);
    // A snapshot taken after the journal was already retired has nothing left to replay.
    bool settled = hadJournal ? (code == FINISH_SAVED || code == FINISH_PENDING || code == FINISH_ERROR)
                              : (code == FINISH_ABSENT);
    expect(settled, "replay of a partial card settles without wiping");
    int archives = card.has("26093010.GPX") ? 1 : 0;
    expect(archives <= 1, "replay does not write a second archive");
    if (hadCont) {
      expect(card.text(cont) == contBefore, "replay does not wipe the continuation");
    }
    if (card.has("26093010.SUM")) {
      expect(card.text("26093010.SUM").find("end=2026-09-30T10:11:12Z") != std::string::npos,
             "summary after reboot keeps the original end time");
    }
  }
  Card done = snaps.back();
  if (!done.has("26093010.SUM")) {
    FinishTxn txn;
    boot(&done, &txn);
  }
  expect(done.has("26093010.SUM") &&
             done.text("26093010.SUM").find("end=2026-09-30T10:11:12Z") != std::string::npos,
         "finished summary keeps the original end time");
}

static void testLeftoverDoesNotZero() {
  Card card;
  FinishDatV2 rec{};
  rec.phase = FINISH_PHASE_ARCHIVING;
  std::snprintf(rec.dest, sizeof(rec.dest), "26093010.GPX");
  rec.hasPoints = 1;
  rec.needSum = 1;
  rec.revCount = 1;
  rec.fromRideGen = 1;
  rec.seq = 1;
  rec.endTimeKnown = 1;
  rec.endYear = 2026;
  rec.endMonth = 9;
  rec.endDay = 30;
  rec.endHour = 1;
  rec.endMin = 2;
  rec.endSec = 3;
  putV2(&card, FINISH_FILE_B, &rec);
  putBytes(&card, "26093010.GPX", "OLD");
  char cont[13];
  finishGenName(2, 'C', "GPX", cont, sizeof(cont));
  putBytes(&card, cont, "X");
  putSidecar(&card, 2, 4);
  putTrip(&card, FINISH_TRIP_A, 8, 77, 4);
  putBytes(&card, FINISH_TRIP_B, "invalid");
  FinishTxn txn;
  int code = boot(&card, &txn);
  expect(code == FINISH_SAVED || code == FINISH_PENDING, "leftover journal settles the newer ride");
  TripDat a;
  expect(readTrip(&card, FINISH_TRIP_A, &a) && a.revCount == 77 && a.rideGen == 4,
         "invalid sibling slot does not zero the newer ride");
  TripDat b;
  expect(readTrip(&card, FINISH_TRIP_B, &b) && b.revCount == 77 && b.rideGen == 4,
         "repaired slot copies the newer counters");
}

static void testBleParse() {
  uint8_t oldList[] = {0x01};
  uint8_t op = 0;
  uint16_t id = 0;
  char name[13];
  expect(bleParseCommand(oldList, 1, &op, &id, name, sizeof(name)) == BLE_PARSE_LEGACY,
         "old layout is not a v2 transfer");
  uint8_t oldGet[] = {0x02, 'A', '.', 'G', 'P', 'X'};
  expect(bleParseCommand(oldGet, sizeof(oldGet), &op, &id, name, sizeof(name)) == BLE_PARSE_LEGACY,
         "old GET is not parsed as a name in v2");
  uint8_t v2[8] = {0xA5, 2, 0x02, 0x02, 0x00, 'Z', 'Z', 0};
  expect(bleParseCommand(v2, sizeof(v2), &op, &id, name, sizeof(name)) == BLE_PARSE_V2, "v2 get parses");
  expect(op == 0x02 && id == 2 && name[0] == 'Z', "v2 request id is uint16");
  expect(!bleReqIsNew(5, 5), "an in-connection id is not reused");
  expect(!bleReqIsNew(65535, 1), "a wrapped id is not accepted as a new transfer");
  expect(bleReqIsNew(0, 1), "the first id on a connection is 1");
  expect(!bleReqIsNew(0, 0), "request id 0 is refused");
}

int main() {
  testNames();
  testAdoptExistingContinuation();
  testInvalidJournalKeepsContinuation();
  testV1Cases();
  testNoGpsPulses();
  testRenameAndCreateFail();
  testThreeRides();
  testFirstJournalReconcile();
  testBootInvalidNoAttempt();
  testCheckpointRepair();
  testStaleJournal();
  testPowerLossAndSummaryTime();
  testLeftoverDoesNotZero();
  testBleParse();
  if (g_fails) {
    std::fprintf(stderr, "%d failure(s)\n", g_fails);
    return 1;
  }
  std::printf("all finish transaction regressions passed\n");
  return 0;
}
