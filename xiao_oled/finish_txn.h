// Finish transaction keyed by ride generation.
// The sketch supplies the filesystem; host tests supply a simulated card.
// GPS, the display, and SdFat stay in the sketch.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const uint32_t TRIP_MAGIC = 0x50495254UL;    // "TRIP"
static const uint32_t FINISH_MAGIC = 0x48534E46UL;  // "FNSH"
static const uint32_t RIDE_ID_MAGIC = 0x31444952UL; // "RID1"
static const uint16_t TRIP_VERSION = 3;
static const uint16_t TRIP_VERSION_V2 = 2;
static const uint16_t TRIP_VERSION_LEGACY = 1;
static const uint16_t FINISH_VERSION_V1 = 1;
static const uint16_t FINISH_VERSION = 2;
static const uint16_t RIDE_ID_VERSION = 1;
static const uint32_t FINISH_GEN_MAX = 0x0FFFFFFFu;

static const char FINISH_FILE_A[] = "FINISH_A.DAT";
static const char FINISH_FILE_B[] = "FINISH_B.DAT";
static const char FINISH_FILE_V1[] = "FINISH.DAT";
static const char FINISH_CURRENT_GPX[] = "CURRENT.GPX";
static const char FINISH_TRIP_A[] = "TRIP_A.DAT";
static const char FINISH_TRIP_B[] = "TRIP_B.DAT";
static const char FINISH_TRIP_LEGACY[] = "TRIP.DAT";

enum {
  FINISH_PHASE_ARCHIVING = 1,
  FINISH_PHASE_SUM_DONE = 2,
  FINISH_PHASE_TRACK_READY = 3,
  FINISH_PHASE_CHECKPOINT_DONE = 4
};

enum {
  FINISH_ABSENT = 0,
  FINISH_YIELD = 1,
  FINISH_PENDING = 2,
  FINISH_ERROR = 3,
  FINISH_SAVED = 4,
  FINISH_NO_GPS = 5,
  FINISH_ROLLED_BACK = 6,
  FINISH_FILES_DONE = 64
};

enum {
  LD_ABSENT = 0,
  LD_UNREAD = 1,
  LD_INVALID = 2,
  LD_V1 = 3,
  LD_V2 = 4
};

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
};

struct FinishDatV1 {
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

struct FinishDatV2 {
  uint32_t magic;
  uint16_t version;
  uint16_t phase;
  char dest[13];
  uint8_t hasPoints;
  uint8_t needSum;
  uint8_t endTimeKnown;
  uint8_t endMonth;
  uint8_t endDay;
  uint8_t endHour;
  uint8_t endMin;
  uint8_t endSec;
  uint16_t endYear;
  uint32_t revCount;
  uint32_t movingMs;
  uint32_t elapsedMs;
  float maxSpeedKmh;
  uint32_t points;
  uint32_t fromRideGen;
  uint32_t seq;
  uint32_t contRev;
  uint32_t contMovingMs;
  uint32_t contElapsedMs;
  float contMaxSpeed;
  uint32_t contPoints;
  uint32_t contGpxEnd;
  uint32_t crc;
};

struct RideSidecar {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t rideGen;
  char gpxName[13];
  uint32_t gpxBodyEnd;
  uint32_t crc;
};
#pragma pack(pop)

struct FinishFs {
  void *ctx;
  // 1 exists, 0 absent, -1 unreadable.
  int (*stat)(void *ctx, const char *name);
  // 0 ok, -1 unreadable. *len is the file size; buf receives min(len, cap).
  int (*read)(void *ctx, const char *name, void *buf, uint32_t cap, uint32_t *len);
  // 0 reported ok, -1 reported failure. Bytes may still have landed. excl=1 is O_EXCL.
  int (*write)(void *ctx, const char *name, const void *data, uint32_t len, int excl);
  int (*rename)(void *ctx, const char *from, const char *to);
  int (*remove)(void *ctx, const char *name);
};

struct FinishCfg {
  const char *gpxHeader;
  const char *gpxFooter;
  float wheelMm;
};

struct FinishSnap {
  uint32_t revCount;
  uint32_t movingMs;
  uint32_t elapsedMs;
  float maxSpeedKmh;
  uint32_t points;
  uint32_t fromRideGen;
  uint32_t gpxBodyEnd;
  uint8_t hasPoints;
  uint8_t needSum;
  uint8_t endTimeKnown;
  uint16_t endYear;
  uint8_t endMonth;
  uint8_t endDay;
  uint8_t endHour;
  uint8_t endMin;
  uint8_t endSec;
  char dest[13];
};

struct FinishContRam {
  uint32_t rev;
  uint32_t movingMs;
  uint32_t elapsedMs;
  float maxSpeed;
  uint32_t points;
  uint32_t gpxEnd;
};

struct FinishOutcome {
  int code;
  bool recordGps;
  bool showPending;
  bool showError;
  bool journalLive;
  bool checkpointsAuthoritative;
  bool archiveHealthy;
  bool preserveCard;
  bool folded;
  bool trackKnown;
  char track[13];
  uint32_t rideGen;
  uint32_t revCount;
  uint32_t movingMs;
  uint32_t elapsedMs;
  float maxSpeedKmh;
  uint32_t points;
  uint32_t gpxBodyEnd;
  FinishSnap foldedFrom;
};

struct FinishTxn {
  bool liveAttempt;
  bool committed;
  bool haveAttempt;
  bool v1Keep;
  FinishSnap attempt;
  FinishContRam cont;
  bool contDirty;
  FinishOutcome out;
};

struct FinishLoad {
  int kind;
  FinishDatV2 rec;
  FinishDatV1 v1;
  int slot;
};

static inline uint32_t crc32Update(uint32_t c, const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    c ^= data[i];
    for (int b = 0; b < 8; b++) {
      uint32_t mask = (uint32_t)-(int32_t)(c & 1UL);
      c = (c >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return c;
}

static inline uint32_t crc32(const uint8_t *data, size_t len) {
  return ~crc32Update(0xFFFFFFFFUL, data, len);
}

static inline uint32_t finishNextSeq(uint32_t seq) {
  uint32_t n = seq + 1u;
  if (n == 0) {
    n = 1;
  }
  return n;
}

static inline uint32_t finishNextGen(uint32_t fromGen) {
  uint32_t g = fromGen + 1u;
  if (g == 0) {
    g = 1;
  }
  if (g > FINISH_GEN_MAX) {
    return 0;
  }
  return g;
}

static inline bool finishGenName(uint32_t gen, char tag, const char *ext, char *out, size_t n) {
  if (!out || n < 13 || gen == 0 || gen > FINISH_GEN_MAX || !ext) {
    return false;
  }
  int wrote = snprintf(out, n, "%c%07lX.%s", tag, (unsigned long)gen, ext);
  return wrote > 0 && (size_t)wrote < n;
}

static inline void tripDatSeal(TripDat *d) {
  d->magic = TRIP_MAGIC;
  d->version = TRIP_VERSION;
  d->crc = crc32((const uint8_t *)d, offsetof(TripDat, crc));
}

static inline bool tripDatValidate(const TripDat *d) {
  if (d->magic != TRIP_MAGIC || d->version != TRIP_VERSION) {
    return false;
  }
  return crc32((const uint8_t *)d, offsetof(TripDat, crc)) == d->crc;
}

static inline bool tripDatValidateV2(const TripDatV2 *d) {
  if (d->magic != TRIP_MAGIC || d->version != TRIP_VERSION_V2) {
    return false;
  }
  return crc32((const uint8_t *)d, offsetof(TripDatV2, crc)) == d->crc;
}

static inline void tripDatFromV2(const TripDatV2 *v2, TripDat *out) {
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

static inline bool tripDatFromLegacyBytes(const uint8_t *bytes, uint32_t len, TripDat *out) {
  if (!bytes || len < sizeof(TripDatV1)) {
    return false;
  }
  TripDatV1 v1;
  memcpy(&v1, bytes, sizeof(v1));
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

static inline void finishSealV1(FinishDatV1 *d) {
  d->magic = FINISH_MAGIC;
  d->version = FINISH_VERSION_V1;
  d->crc = crc32((const uint8_t *)d, offsetof(FinishDatV1, crc));
}

static inline bool finishValidateV1(const FinishDatV1 *d) {
  if (d->magic != FINISH_MAGIC || d->version != FINISH_VERSION_V1) {
    return false;
  }
  return crc32((const uint8_t *)d, offsetof(FinishDatV1, crc)) == d->crc;
}

static inline void finishSealV2(FinishDatV2 *d) {
  d->magic = FINISH_MAGIC;
  d->version = FINISH_VERSION;
  d->crc = crc32((const uint8_t *)d, offsetof(FinishDatV2, crc));
}

static inline bool finishValidateV2(const FinishDatV2 *d) {
  if (d->magic != FINISH_MAGIC || d->version != FINISH_VERSION) {
    return false;
  }
  return crc32((const uint8_t *)d, offsetof(FinishDatV2, crc)) == d->crc;
}

static inline void rideSidecarSeal(RideSidecar *d) {
  d->magic = RIDE_ID_MAGIC;
  d->version = RIDE_ID_VERSION;
  d->crc = crc32((const uint8_t *)d, offsetof(RideSidecar, crc));
}

static inline bool rideSidecarValidate(const RideSidecar *d) {
  if (d->magic != RIDE_ID_MAGIC || d->version != RIDE_ID_VERSION) {
    return false;
  }
  return crc32((const uint8_t *)d, offsetof(RideSidecar, crc)) == d->crc;
}

static inline void finishTxnInit(FinishTxn *t) {
  memset(t, 0, sizeof(*t));
  t->out.code = FINISH_ABSENT;
  t->out.recordGps = true;
}

static inline int finishStat(FinishFs *fs, const char *name) {
  if (!fs || !fs->stat || !name) {
    return -1;
  }
  return fs->stat(fs->ctx, name);
}

static inline int finishRead(FinishFs *fs, const char *name, void *buf, uint32_t cap, uint32_t *len) {
  if (!fs || !fs->read) {
    return -1;
  }
  return fs->read(fs->ctx, name, buf, cap, len);
}

static inline bool finishCopyName(char *dst, size_t n, const char *src) {
  if (!dst || n == 0 || !src) {
    return false;
  }
  size_t i = 0;
  for (; src[i] && i + 1 < n; i++) {
    dst[i] = src[i];
  }
  dst[i] = '\0';
  return src[i] == '\0';
}

static inline bool finishSameIdentity(const FinishDatV2 *rec, const FinishSnap *snap) {
  if (!rec || !snap) {
    return false;
  }
  if (rec->revCount != snap->revCount || rec->movingMs != snap->movingMs ||
      rec->elapsedMs != snap->elapsedMs || rec->points != snap->points ||
      rec->fromRideGen != snap->fromRideGen || rec->hasPoints != snap->hasPoints ||
      rec->needSum != snap->needSum || rec->endTimeKnown != snap->endTimeKnown) {
    return false;
  }
  if (rec->maxSpeedKmh != snap->maxSpeedKmh) {
    return false;
  }
  if (rec->endTimeKnown) {
    if (rec->endYear != snap->endYear || rec->endMonth != snap->endMonth ||
        rec->endDay != snap->endDay || rec->endHour != snap->endHour ||
        rec->endMin != snap->endMin || rec->endSec != snap->endSec) {
      return false;
    }
  }
  return strcmp(rec->dest, snap->dest) == 0;
}

static inline int finishLoadSlotV2(FinishFs *fs, const char *name, FinishDatV2 *out, bool *exists) {
  int st = finishStat(fs, name);
  if (st < 0) {
    return LD_UNREAD;
  }
  if (st == 0) {
    if (exists) {
      *exists = false;
    }
    return LD_ABSENT;
  }
  if (exists) {
    *exists = true;
  }
  uint8_t buf[sizeof(FinishDatV2)];
  uint32_t len = 0;
  if (finishRead(fs, name, buf, sizeof(buf), &len) != 0) {
    return LD_UNREAD;
  }
  if (len == sizeof(FinishDatV2)) {
    FinishDatV2 d;
    memcpy(&d, buf, sizeof(d));
    if (finishValidateV2(&d)) {
      *out = d;
      return LD_V2;
    }
  }
  return LD_INVALID;
}

static inline int finishLoadV1File(FinishFs *fs, FinishDatV1 *out, bool *exists) {
  int st = finishStat(fs, FINISH_FILE_V1);
  if (st < 0) {
    return LD_UNREAD;
  }
  if (st == 0) {
    if (exists) {
      *exists = false;
    }
    return LD_ABSENT;
  }
  if (exists) {
    *exists = true;
  }
  uint8_t buf[sizeof(FinishDatV1)];
  uint32_t len = 0;
  if (finishRead(fs, FINISH_FILE_V1, buf, sizeof(buf), &len) != 0) {
    return LD_UNREAD;
  }
  if (len == sizeof(FinishDatV1)) {
    FinishDatV1 d;
    memcpy(&d, buf, sizeof(d));
    if (finishValidateV1(&d)) {
      *out = d;
      return LD_V1;
    }
  }
  return LD_INVALID;
}

// Newest valid v2 wins. A torn v2 slot does not hide a valid v1 journal.
// Unreadable names stay uncertain. Absent only when every name was re-read as missing.
static inline int finishLoadJournal(FinishFs *fs, FinishLoad *out) {
  memset(out, 0, sizeof(*out));
  out->slot = -1;
  FinishDatV2 a;
  FinishDatV2 b;
  bool aEx = false;
  bool bEx = false;
  bool v1Ex = false;
  int la = finishLoadSlotV2(fs, FINISH_FILE_A, &a, &aEx);
  int lb = finishLoadSlotV2(fs, FINISH_FILE_B, &b, &bEx);
  FinishDatV1 v1;
  int l1 = finishLoadV1File(fs, &v1, &v1Ex);
  if (la == LD_UNREAD || lb == LD_UNREAD || l1 == LD_UNREAD) {
    out->kind = LD_UNREAD;
    return LD_UNREAD;
  }
  bool anyFile = aEx || bEx || v1Ex;
  if (la == LD_V2 && lb == LD_V2) {
    out->kind = LD_V2;
    if (a.seq >= b.seq) {
      out->rec = a;
      out->slot = 0;
    } else {
      out->rec = b;
      out->slot = 1;
    }
    return LD_V2;
  }
  if (la == LD_V2) {
    out->kind = LD_V2;
    out->rec = a;
    out->slot = 0;
    return LD_V2;
  }
  if (lb == LD_V2) {
    out->kind = LD_V2;
    out->rec = b;
    out->slot = 1;
    return LD_V2;
  }
  if (l1 == LD_V1) {
    out->kind = LD_V1;
    out->v1 = v1;
    return LD_V1;
  }
  if (!anyFile) {
    out->kind = LD_ABSENT;
    return LD_ABSENT;
  }
  out->kind = LD_INVALID;
  return LD_INVALID;
}

static inline int finishSet(FinishTxn *t, int code) {
  FinishOutcome *o = &t->out;
  o->code = code;
  o->showError = (code == FINISH_ERROR);
  o->showPending = (code == FINISH_PENDING);
  o->preserveCard = (code == FINISH_ERROR || code == FINISH_PENDING);
  if (code == FINISH_ERROR) {
    o->recordGps = false;
    o->journalLive = true;
    o->archiveHealthy = false;
    o->checkpointsAuthoritative = false;
  } else if (code == FINISH_PENDING) {
    o->journalLive = true;
    o->archiveHealthy = false;
    o->checkpointsAuthoritative = false;
  } else if (code == FINISH_SAVED || code == FINISH_NO_GPS) {
    o->recordGps = true;
    o->journalLive = false;
    o->archiveHealthy = true;
    o->checkpointsAuthoritative = true;
    o->showPending = false;
    o->showError = false;
    o->preserveCard = false;
  } else if (code == FINISH_ROLLED_BACK) {
    o->recordGps = true;
    o->journalLive = false;
    o->showPending = false;
    o->showError = false;
    o->archiveHealthy = true;
    o->preserveCard = false;
    o->folded = true;
  } else if (code == FINISH_ABSENT) {
    o->recordGps = true;
    o->journalLive = false;
    o->showPending = false;
    o->showError = false;
    o->journalLive = false;
    o->preserveCard = false;
  }
  return code;
}

static inline int finishHold(FinishTxn *t, int code) {
  t->out.recordGps = false;
  return finishSet(t, code);
}

static inline const char *finishSlotName(int slot) {
  return (slot == 0) ? FINISH_FILE_A : FINISH_FILE_B;
}

// A false write does not prove the bytes are missing. Confirm length, and the
// full body when it fits. Larger creates (a GPX header) match on length and prefix.
static inline int finishWriteConfirmed(FinishFs *fs, const char *name, const void *data,
                                      uint32_t len, int excl) {
  if (!fs || !fs->write) {
    return -1;
  }
  (void)fs->write(fs->ctx, name, data, len, excl);
  uint8_t buf[256];
  uint32_t got = 0;
  if (finishRead(fs, name, buf, sizeof(buf), &got) != 0) {
    return -1;
  }
  if (got != len) {
    return -1;
  }
  uint32_t n = len <= sizeof(buf) ? len : 64u;
  if (n > 0 && memcmp(buf, data, n) != 0) {
    return -1;
  }
  return 0;
}

static inline bool finishTripCountersSame(const TripDat *a, const TripDat *b) {
  return a->rideGen == b->rideGen && a->revCount == b->revCount &&
         a->movingMs == b->movingMs && a->elapsedMs == b->elapsedMs &&
         a->maxSpeedKmh == b->maxSpeedKmh && a->gpxBodyEnd == b->gpxBodyEnd;
}

static inline int finishReadTrip(FinishFs *fs, const char *name, TripDat *out) {
  int st = finishStat(fs, name);
  if (st < 0) {
    return -1;
  }
  if (st == 0) {
    return 0;
  }
  uint8_t buf[64];
  uint32_t len = 0;
  if (finishRead(fs, name, buf, sizeof(buf), &len) != 0) {
    return -1;
  }
  if (len == sizeof(TripDat)) {
    TripDat d;
    memcpy(&d, buf, sizeof(d));
    if (tripDatValidate(&d)) {
      *out = d;
      return 1;
    }
    return 0;
  }
  if (len == sizeof(TripDatV2)) {
    TripDatV2 v2;
    memcpy(&v2, buf, sizeof(v2));
    if (!tripDatValidateV2(&v2)) {
      return 0;
    }
    tripDatFromV2(&v2, out);
    return 1;
  }
  if (strcmp(name, FINISH_TRIP_LEGACY) == 0 && tripDatFromLegacyBytes(buf, len, out)) {
    return 1;
  }
  return 0;
}

static inline void finishApplyTrack(FinishTxn *t, FinishFs *fs, uint32_t gen, uint32_t bodyEnd,
                                    uint32_t rev, uint32_t moving, uint32_t elapsed, float maxSp,
                                    uint32_t points) {
  char genFile[13];
  t->out.rideGen = gen;
  t->out.revCount = rev;
  t->out.movingMs = moving;
  t->out.elapsedMs = elapsed;
  t->out.maxSpeedKmh = maxSp;
  t->out.points = points;
  t->out.gpxBodyEnd = bodyEnd;
  t->out.trackKnown = false;
  t->out.track[0] = '\0';
  if (finishGenName(gen, 'C', "GPX", genFile, sizeof(genFile)) && finishStat(fs, genFile) == 1) {
    finishCopyName(t->out.track, sizeof(t->out.track), genFile);
    t->out.trackKnown = true;
    return;
  }
  if (finishStat(fs, FINISH_CURRENT_GPX) == 1 || gen != 0) {
    finishCopyName(t->out.track, sizeof(t->out.track), FINISH_CURRENT_GPX);
    t->out.trackKnown = true;
  }
}

static inline int finishSummaryBody(const FinishDatV2 *rec, float wheelMm, char *buf, size_t n) {
  float distanceM = (rec->revCount * wheelMm) / 1000.0f;
  unsigned y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;
  if (rec->endTimeKnown) {
    y = rec->endYear;
    mo = rec->endMonth;
    d = rec->endDay;
    h = rec->endHour;
    mi = rec->endMin;
    s = rec->endSec;
  }
  return snprintf(buf, n,
                  "rev=%lu\n"
                  "distance_m=%.1f\n"
                  "moving_ms=%lu\n"
                  "elapsed_ms=%lu\n"
                  "max_kmh=%.2f\n"
                  "points=%lu\n"
                  "end=%04u-%02u-%02uT%02u:%02u:%02uZ\n",
                  (unsigned long)rec->revCount, (double)distanceM,
                  (unsigned long)rec->movingMs, (unsigned long)rec->elapsedMs,
                  (double)rec->maxSpeedKmh, (unsigned long)rec->points,
                  y, mo, d, h, mi, s);
}

static inline bool finishSumName(const char *gpxName, char *out, size_t n) {
  if (!gpxName || !out || n < 5) {
    return false;
  }
  const char *dot = strrchr(gpxName, '.');
  size_t stem = dot ? (size_t)(dot - gpxName) : strlen(gpxName);
  if (stem == 0 || stem > 8 || stem + 5 > n) {
    return false;
  }
  memcpy(out, gpxName, stem);
  memcpy(out + stem, ".SUM", 5);
  return true;
}

static inline bool finishSummaryMatches(FinishFs *fs, const FinishCfg *cfg, const FinishDatV2 *rec) {
  char name[13];
  char expect[320];
  if (!rec->needSum || rec->dest[0] == '\0') {
    return true;
  }
  if (!finishSumName(rec->dest, name, sizeof(name))) {
    return false;
  }
  int n = finishSummaryBody(rec, cfg->wheelMm, expect, sizeof(expect));
  if (n <= 0 || (size_t)n >= sizeof(expect)) {
    return false;
  }
  int st = finishStat(fs, name);
  if (st != 1) {
    return false;
  }
  uint8_t buf[320];
  uint32_t len = 0;
  if (finishRead(fs, name, buf, sizeof(buf), &len) != 0) {
    return false;
  }
  return len == (uint32_t)n && memcmp(buf, expect, (size_t)n) == 0;
}

static inline int finishPickSource(FinishFs *fs, uint32_t fromGen, char *name, size_t n) {
  char genFile[13];
  if (fromGen != 0 && fromGen <= FINISH_GEN_MAX &&
      finishGenName(fromGen, 'C', "GPX", genFile, sizeof(genFile))) {
    int st = finishStat(fs, genFile);
    if (st < 0) {
      return -1;
    }
    if (st == 1) {
      finishCopyName(name, n, genFile);
      return 1;
    }
  } else if (fromGen > FINISH_GEN_MAX) {
    return -2;
  }
  int st = finishStat(fs, FINISH_CURRENT_GPX);
  if (st < 0) {
    return -1;
  }
  if (st == 1) {
    finishCopyName(name, n, FINISH_CURRENT_GPX);
    return 1;
  }
  return 0;
}

static inline int finishReadSidecar(FinishFs *fs, uint32_t gen, RideSidecar *out) {
  char name[13];
  if (!finishGenName(gen, 'G', "ID", name, sizeof(name))) {
    return -2;
  }
  int st = finishStat(fs, name);
  if (st < 0) {
    return -1;
  }
  if (st == 0) {
    return 0;
  }
  uint8_t buf[sizeof(RideSidecar)];
  uint32_t len = 0;
  if (finishRead(fs, name, buf, sizeof(buf), &len) != 0) {
    return -1;
  }
  if (len != sizeof(RideSidecar)) {
    return 2;
  }
  RideSidecar s;
  memcpy(&s, buf, sizeof(s));
  if (!rideSidecarValidate(&s)) {
    return 2;
  }
  *out = s;
  char gpx[13];
  if (!finishGenName(gen, 'C', "GPX", gpx, sizeof(gpx)) || s.rideGen != gen ||
      strcmp(s.gpxName, gpx) != 0) {
    return 2;
  }
  return 1;
}

static inline bool finishArtifacts(FinishFs *fs, const FinishSnap *snap) {
  if (snap->dest[0] && finishStat(fs, snap->dest) == 1) {
    return true;
  }
  uint32_t ng = finishNextGen(snap->fromRideGen);
  char cont[13];
  char side[13];
  if (ng && finishGenName(ng, 'C', "GPX", cont, sizeof(cont)) && finishStat(fs, cont) == 1) {
    return true;
  }
  if (ng && finishGenName(ng, 'G', "ID", side, sizeof(side)) && finishStat(fs, side) == 1) {
    return true;
  }
  return false;
}

static inline bool finishJournalsGone(FinishFs *fs) {
  return finishStat(fs, FINISH_FILE_A) == 0 && finishStat(fs, FINISH_FILE_B) == 0 &&
         finishStat(fs, FINISH_FILE_V1) == 0;
}

static inline int finishRemoveConfirmed(FinishFs *fs, const char *name) {
  int st = finishStat(fs, name);
  if (st < 0) {
    return -1;
  }
  if (st == 0) {
    return 0;
  }
  if (!fs->remove || fs->remove(fs->ctx, name) != 0) {
    st = finishStat(fs, name);
    return (st == 0) ? 0 : -1;
  }
  st = finishStat(fs, name);
  if (st < 0) {
    return -1;
  }
  return (st == 0) ? 0 : -1;
}

static inline int finishRollback(FinishTxn *t, FinishFs *fs) {
  const char *names[] = {FINISH_FILE_A, FINISH_FILE_B, FINISH_FILE_V1};
  for (unsigned i = 0; i < 3; i++) {
    if (finishRemoveConfirmed(fs, names[i]) != 0) {
      return finishHold(t, FINISH_PENDING);
    }
  }
  if (!finishJournalsGone(fs)) {
    return finishHold(t, FINISH_PENDING);
  }
  t->out.folded = true;
  t->out.foldedFrom = t->attempt;
  t->out.recordGps = true;
  t->committed = false;
  return finishSet(t, FINISH_ROLLED_BACK);
}

static inline FinishDatV2 finishFromSnap(const FinishSnap *snap) {
  FinishDatV2 d;
  memset(&d, 0, sizeof(d));
  d.phase = FINISH_PHASE_ARCHIVING;
  finishCopyName(d.dest, sizeof(d.dest), snap->dest);
  d.hasPoints = snap->hasPoints;
  d.needSum = snap->needSum;
  d.endTimeKnown = snap->endTimeKnown;
  d.endYear = snap->endYear;
  d.endMonth = snap->endMonth;
  d.endDay = snap->endDay;
  d.endHour = snap->endHour;
  d.endMin = snap->endMin;
  d.endSec = snap->endSec;
  d.revCount = snap->revCount;
  d.movingMs = snap->movingMs;
  d.elapsedMs = snap->elapsedMs;
  d.maxSpeedKmh = snap->maxSpeedKmh;
  d.points = snap->points;
  d.fromRideGen = snap->fromRideGen;
  d.seq = 1;
  return d;
}

static inline FinishDatV2 finishFromV1(const FinishDatV1 *v1) {
  FinishDatV2 d;
  memset(&d, 0, sizeof(d));
  d.phase = v1->phase;
  finishCopyName(d.dest, sizeof(d.dest), v1->dest);
  d.hasPoints = v1->hasPoints;
  d.needSum = v1->needSum;
  d.revCount = v1->revCount;
  d.movingMs = v1->movingMs;
  d.elapsedMs = v1->elapsedMs;
  d.maxSpeedKmh = v1->maxSpeedKmh;
  d.points = v1->points;
  d.fromRideGen = v1->fromRideGen;
  d.seq = 1;
  return d;
}

static inline int finishWriteJournalSlot(FinishFs *fs, int slot, FinishDatV2 *rec) {
  finishSealV2(rec);
  return finishWriteConfirmed(fs, finishSlotName(slot), rec, sizeof(*rec), 0);
}

static inline void finishFillCont(FinishDatV2 *rec, const FinishContRam *c) {
  rec->contRev = c->rev;
  rec->contMovingMs = c->movingMs;
  rec->contElapsedMs = c->elapsedMs;
  rec->contMaxSpeed = c->maxSpeed;
  rec->contPoints = c->points;
  rec->contGpxEnd = c->gpxEnd;
}

static inline bool finishContAhead(const FinishContRam *c, const FinishDatV2 *rec) {
  return c->rev != rec->contRev || c->movingMs != rec->contMovingMs ||
         c->elapsedMs != rec->contElapsedMs || c->maxSpeed != rec->contMaxSpeed ||
         c->points != rec->contPoints || c->gpxEnd != rec->contGpxEnd;
}

static inline int finishSyncCont(FinishTxn *t, FinishFs *fs) {
  if (!t->contDirty) {
    return 0;
  }
  FinishLoad ld;
  int kind = finishLoadJournal(fs, &ld);
  if (kind != LD_V2) {
    return -1;
  }
  if (!finishContAhead(&t->cont, &ld.rec)) {
    t->contDirty = false;
    return 0;
  }
  FinishDatV2 next = ld.rec;
  finishFillCont(&next, &t->cont);
  next.seq = finishNextSeq(ld.rec.seq);
  int slot = (ld.slot == 0) ? 1 : 0;
  if (finishWriteJournalSlot(fs, slot, &next) != 0) {
    return -1;
  }
  t->contDirty = false;
  return 0;
}

static inline int finishNoteUnread(FinishTxn *t) {
  t->out.recordGps = false;
  return finishSet(t, t->liveAttempt ? FINISH_PENDING : FINISH_ERROR);
}

static inline int finishClassifyV1(FinishFs *fs, const FinishDatV1 *v1) {
  uint32_t newGen = finishNextGen(v1->fromRideGen);
  if (newGen == 0) {
    return -2;
  }
  char cont[13];
  char side[13];
  if (!finishGenName(newGen, 'C', "GPX", cont, sizeof(cont)) ||
      !finishGenName(newGen, 'G', "ID", side, sizeof(side))) {
    return -2;
  }
  int stDest = 0;
  if (v1->dest[0]) {
    stDest = finishStat(fs, v1->dest);
    if (stDest < 0) {
      return -1;
    }
  }
  int stCur = finishStat(fs, FINISH_CURRENT_GPX);
  int stCont = finishStat(fs, cont);
  int stSide = finishStat(fs, side);
  if (stCur < 0 || stCont < 0 || stSide < 0) {
    return -1;
  }
  bool destExists = stDest == 1;
  bool curExists = stCur == 1;
  bool contExists = stCont == 1 || stSide == 1;
  if (v1->hasPoints && v1->phase == FINISH_PHASE_ARCHIVING && !destExists && curExists &&
      !contExists) {
    return 1;
  }
  if (v1->hasPoints && destExists && !curExists && !contExists) {
    return 2;
  }
  return -2;
}

static inline int finishStepV1(FinishTxn *t, FinishFs *fs, const FinishDatV1 *v1) {
  int cls = finishClassifyV1(fs, v1);
  if (cls == -1) {
    return finishNoteUnread(t);
  }
  if (cls != 1 && cls != 2) {
    return finishHold(t, FINISH_ERROR);
  }
  FinishDatV2 rec = finishFromV1(v1);
  if (cls == 2 && rec.phase == FINISH_PHASE_ARCHIVING) {
    rec.phase = FINISH_PHASE_SUM_DONE;
  }
  finishSealV2(&rec);
  if (finishWriteJournalSlot(fs, 0, &rec) != 0) {
    FinishLoad again;
    if (finishLoadJournal(fs, &again) == LD_V2 && finishValidateV2(&again.rec)) {
      return FINISH_YIELD;
    }
    return finishHold(t, FINISH_PENDING);
  }
  return FINISH_YIELD;
}

static inline void finishWantFromRec(const FinishDatV2 *rec, uint32_t gen, TripDat *want) {
  memset(want, 0, sizeof(*want));
  want->rideGen = gen;
  want->revCount = rec->contRev;
  want->movingMs = rec->contMovingMs;
  want->elapsedMs = rec->contElapsedMs;
  want->maxSpeedKmh = rec->contMaxSpeed;
  want->gpxBodyEnd = rec->contGpxEnd;
}

static inline bool finishCheckpointAhead(const TripDat *slot, uint32_t contGen, const TripDat *cont) {
  if (slot->rideGen > contGen) {
    return true;
  }
  if (slot->rideGen < contGen) {
    return false;
  }
  if (slot->revCount != cont->revCount) {
    return slot->revCount > cont->revCount;
  }
  return slot->gpxBodyEnd > cont->gpxBodyEnd;
}

static inline int finishWriteTripSlot(FinishFs *fs, const char *name, TripDat *d) {
  tripDatSeal(d);
  return finishWriteConfirmed(fs, name, d, sizeof(*d), 0);
}

static inline int finishCreateGpx(FinishFs *fs, const FinishCfg *cfg, const char *name) {
  if (!cfg->gpxHeader || !cfg->gpxFooter) {
    return -1;
  }
  size_t hl = strlen(cfg->gpxHeader);
  size_t fl = strlen(cfg->gpxFooter);
  if (hl + fl > 480) {
    return -1;
  }
  uint8_t body[480];
  memcpy(body, cfg->gpxHeader, hl);
  memcpy(body + hl, cfg->gpxFooter, fl);
  return finishWriteConfirmed(fs, name, body, (uint32_t)(hl + fl), 1);
}

static inline int finishCreateSidecar(FinishFs *fs, uint32_t gen, const char *gpx, uint32_t bodyEnd) {
  char name[13];
  if (!finishGenName(gen, 'G', "ID", name, sizeof(name))) {
    return -1;
  }
  RideSidecar s;
  memset(&s, 0, sizeof(s));
  s.rideGen = gen;
  finishCopyName(s.gpxName, sizeof(s.gpxName), gpx);
  s.gpxBodyEnd = bodyEnd;
  rideSidecarSeal(&s);
  return finishWriteConfirmed(fs, name, &s, sizeof(s), 1);
}

static inline int finishStepFiles(FinishTxn *t, FinishFs *fs, const FinishCfg *cfg,
                                  const FinishDatV2 *rec, uint32_t newGen) {
  char cont[13];
  char source[13];
  if (!finishGenName(newGen, 'C', "GPX", cont, sizeof(cont))) {
    return finishHold(t, FINISH_ERROR);
  }
  RideSidecar side;
  memset(&side, 0, sizeof(side));
  int sideKind = finishReadSidecar(fs, newGen, &side);
  if (sideKind < 0) {
    return finishNoteUnread(t);
  }
  int src = finishPickSource(fs, rec->fromRideGen, source, sizeof(source));
  if (src == -1) {
    return finishNoteUnread(t);
  }
  if (src == -2) {
    return finishHold(t, FINISH_ERROR);
  }
  int stCont = finishStat(fs, cont);
  if (stCont < 0) {
    return finishNoteUnread(t);
  }
  int stDest = 0;
  if (rec->dest[0]) {
    stDest = finishStat(fs, rec->dest);
    if (stDest < 0) {
      return finishNoteUnread(t);
    }
  }
  bool destExists = stDest == 1;
  bool contExists = stCont == 1;
  bool sideOk = sideKind == 1;
  bool sideBad = sideKind == 2;

  uint32_t fromGen = rec->fromRideGen;
  RideSidecar fromSide;
  int fromSideKind = 0;
  if (fromGen != 0 && fromGen <= FINISH_GEN_MAX) {
    fromSideKind = finishReadSidecar(fs, fromGen, &fromSide);
    if (fromSideKind < 0) {
      return finishNoteUnread(t);
    }
  }
  if (sideOk && fromSideKind == 1 && strcmp(side.gpxName, fromSide.gpxName) == 0) {
    return finishHold(t, FINISH_ERROR);
  }
  if (sideBad) {
    return finishHold(t, FINISH_ERROR);
  }
  if (rec->hasPoints && !destExists && (contExists || sideOk)) {
    return finishHold(t, FINISH_ERROR);
  }
  if (rec->hasPoints && rec->dest[0] == '\0') {
    return finishHold(t, FINISH_ERROR);
  }

  bool renameDone = !rec->hasPoints || destExists;
  if (!renameDone) {
    if (src != 1) {
      return finishHold(t, FINISH_ERROR);
    }
    if (strcmp(source, cont) == 0) {
      return finishHold(t, FINISH_ERROR);
    }
    if (!fs->rename || fs->rename(fs->ctx, source, rec->dest) != 0) {
      int nowDest = finishStat(fs, rec->dest);
      int nowSrc = finishStat(fs, source);
      if (nowDest == 1 && nowSrc == 0) {
        return FINISH_YIELD;
      }
      t->out.recordGps = false;
      return finishHold(t, FINISH_PENDING);
    }
    return FINISH_YIELD;
  }

  if (sideOk && !contExists) {
    return finishHold(t, FINISH_ERROR);
  }
  if (!contExists && !sideOk) {
    if (finishCreateGpx(fs, cfg, cont) != 0) {
      if (finishStat(fs, cont) == 1) {
        return FINISH_YIELD;
      }
      t->out.recordGps = false;
      return finishHold(t, FINISH_PENDING);
    }
    return FINISH_YIELD;
  }
  if (contExists && !sideOk) {
    uint32_t len = 0;
    uint8_t scratch[8];
    if (finishRead(fs, cont, scratch, sizeof(scratch), &len) != 0) {
      return finishNoteUnread(t);
    }
    size_t hl = cfg->gpxHeader ? strlen(cfg->gpxHeader) : 0;
    size_t fl = cfg->gpxFooter ? strlen(cfg->gpxFooter) : 0;
    uint32_t bodyEnd = 0;
    if (hl > 0 && len == hl + fl) {
      bodyEnd = (uint32_t)hl;
    } else if (rec->contGpxEnd >= hl && len >= rec->contGpxEnd) {
      bodyEnd = rec->contGpxEnd;
    } else {
      return finishHold(t, FINISH_ERROR);
    }
    if (finishCreateSidecar(fs, newGen, cont, bodyEnd) != 0) {
      RideSidecar check;
      if (finishReadSidecar(fs, newGen, &check) == 1) {
        return FINISH_YIELD;
      }
      t->out.recordGps = false;
      return finishHold(t, FINISH_PENDING);
    }
    return FINISH_YIELD;
  }

  // Adopted. Do not truncate the continuation.
  if (rec->needSum && rec->dest[0]) {
    if (!finishSummaryMatches(fs, cfg, rec)) {
      char name[13];
      char body[320];
      if (!finishSumName(rec->dest, name, sizeof(name))) {
        return finishHold(t, FINISH_ERROR);
      }
      int n = finishSummaryBody(rec, cfg->wheelMm, body, sizeof(body));
      if (n <= 0 || (size_t)n >= sizeof(body)) {
        return finishHold(t, FINISH_ERROR);
      }
      if (finishWriteConfirmed(fs, name, body, (uint32_t)n, 0) != 0) {
        if (!finishSummaryMatches(fs, cfg, rec)) {
          t->out.recordGps = false;
          return finishHold(t, FINISH_PENDING);
        }
      }
      return FINISH_YIELD;
    }
  }
  return FINISH_FILES_DONE;
}

static inline void finishPublishCont(FinishTxn *t, FinishFs *fs, uint32_t gen, const TripDat *slot) {
  finishApplyTrack(t, fs, gen, slot->gpxBodyEnd, slot->revCount, slot->movingMs, slot->elapsedMs,
                   slot->maxSpeedKmh, 0);
  t->out.recordGps = t->out.trackKnown;
  t->out.journalLive = true;
}

static inline int finishRetireJournal(FinishTxn *t, FinishFs *fs, const FinishDatV2 *rec) {
  const char *names[] = {FINISH_FILE_A, FINISH_FILE_B, FINISH_FILE_V1};
  for (unsigned i = 0; i < 3; i++) {
    int st = finishStat(fs, names[i]);
    if (st < 0) {
      t->out.archiveHealthy = true;
      t->out.checkpointsAuthoritative = true;
      t->out.journalLive = true;
      t->out.recordGps = true;
      return finishSet(t, FINISH_PENDING);
    }
    if (st == 1) {
      if (finishRemoveConfirmed(fs, names[i]) != 0) {
        // Delete failed. The ride stays; archive health already came from the safe pair.
        t->out.archiveHealthy = true;
        t->out.checkpointsAuthoritative = true;
        t->out.journalLive = true;
        t->out.recordGps = true;
        t->out.showPending = false;
        t->out.showError = false;
        t->out.preserveCard = true;
        t->out.code = FINISH_PENDING;
        return FINISH_PENDING;
      }
      // The step that removes the last journal record is the finish itself.
      // Yielding here would make the next step see an empty card and report absent.
      if (finishJournalsGone(fs)) {
        t->out.journalLive = false;
        t->out.archiveHealthy = true;
        t->out.checkpointsAuthoritative = true;
        t->out.recordGps = true;
        return finishSet(t, rec->hasPoints ? FINISH_SAVED : FINISH_NO_GPS);
      }
      return FINISH_YIELD;
    }
  }
  t->out.journalLive = false;
  t->out.archiveHealthy = true;
  t->out.checkpointsAuthoritative = true;
  t->out.recordGps = true;
  return finishSet(t, rec->hasPoints ? FINISH_SAVED : FINISH_NO_GPS);
}

static inline int finishRepairSlots(FinishTxn *t, FinishFs *fs, const FinishDatV2 *rec, const TripDat *want,
                                    const TripDat *a, int ha, const TripDat *b, int hb, int hl) {
  bool aMatch = (ha == 1) && finishTripCountersSame(a, want);
  bool bMatch = (hb == 1) && finishTripCountersSame(b, want);
  bool legacySafe = (hl == 0);
  if (ha < 0 || hb < 0 || hl < 0) {
    t->out.recordGps = true;
    t->out.journalLive = true;
    return finishSet(t, t->liveAttempt ? FINISH_PENDING : FINISH_ERROR);
  }
  if (aMatch && bMatch && legacySafe) {
    t->out.checkpointsAuthoritative = true;
    t->out.archiveHealthy = true;
    finishPublishCont(t, fs, want->rideGen, want);
    return finishRetireJournal(t, fs, rec);
  }
  if (hl == 1 && aMatch && bMatch) {
    if (finishRemoveConfirmed(fs, FINISH_TRIP_LEGACY) != 0) {
      t->out.journalLive = true;
      t->out.recordGps = true;
      t->out.checkpointsAuthoritative = false;
      return finishSet(t, FINISH_PENDING);
    }
    return FINISH_YIELD;
  }
  uint32_t seq = 0;
  if (ha == 1 && a->seq > seq) {
    seq = a->seq;
  }
  if (hb == 1 && b->seq > seq) {
    seq = b->seq;
  }
  TripDat slot = *want;
  slot.seq = finishNextSeq(seq);
  const char *which = NULL;
  if (!aMatch && !bMatch) {
    bool aNewer = (ha == 1) && (hb != 1 || a->seq >= b->seq);
    which = aNewer ? FINISH_TRIP_B : FINISH_TRIP_A;
  } else if (!aMatch) {
    which = FINISH_TRIP_A;
  } else {
    which = FINISH_TRIP_B;
  }
  if (finishWriteTripSlot(fs, which, &slot) != 0) {
    TripDat check;
    int got = finishReadTrip(fs, which, &check);
    if (got == 1 && finishTripCountersSame(&check, want)) {
      return FINISH_YIELD;
    }
    t->out.journalLive = true;
    t->out.recordGps = true;
    return finishSet(t, FINISH_PENDING);
  }
  return FINISH_YIELD;
}

static inline int finishStepV2(FinishTxn *t, FinishFs *fs, const FinishCfg *cfg, const FinishDatV2 *recIn) {
  FinishDatV2 rec = *recIn;
  if (t->contDirty && finishContAhead(&t->cont, &rec)) {
    if (finishSyncCont(t, fs) != 0) {
      t->out.recordGps = false;
      return finishHold(t, FINISH_PENDING);
    }
    return FINISH_YIELD;
  }
  uint32_t newGen = finishNextGen(rec.fromRideGen);
  if (newGen == 0) {
    return finishHold(t, FINISH_ERROR);
  }
  if (!t->v1Keep && finishStat(fs, FINISH_FILE_V1) == 1) {
    if (finishRemoveConfirmed(fs, FINISH_FILE_V1) == 0) {
      return FINISH_YIELD;
    }
    t->v1Keep = true;
  }

  TripDat a, b, leg;
  memset(&a, 0, sizeof(a));
  memset(&b, 0, sizeof(b));
  memset(&leg, 0, sizeof(leg));
  int ha = finishReadTrip(fs, FINISH_TRIP_A, &a);
  int hb = finishReadTrip(fs, FINISH_TRIP_B, &b);
  int hl = finishReadTrip(fs, FINISH_TRIP_LEGACY, &leg);
  if (ha < 0 || hb < 0) {
    return finishNoteUnread(t);
  }
  const TripDat *active = NULL;
  if (ha == 1 && hb == 1) {
    active = (a.seq >= b.seq) ? &a : &b;
  } else if (ha == 1) {
    active = &a;
  } else if (hb == 1) {
    active = &b;
  }

  TripDat contWant;
  finishWantFromRec(&rec, newGen, &contWant);
  bool newer = false;
  TripDat resume = contWant;
  if (active && active->rideGen > rec.fromRideGen) {
    if (active->rideGen > newGen || finishCheckpointAhead(active, newGen, &contWant)) {
      resume = *active;
      newer = true;
    } else if (active->rideGen == newGen && !finishCheckpointAhead(&contWant, newGen, active) &&
               !finishTripCountersSame(active, &contWant)) {
      // Same generation, checkpoint is not behind the journal.
      if (active->revCount > contWant.revCount ||
          (active->revCount == contWant.revCount && active->gpxBodyEnd >= contWant.gpxBodyEnd)) {
        resume = *active;
        newer = true;
      }
    } else if (active->rideGen == newGen && finishTripCountersSame(active, &contWant)) {
      resume = *active;
      newer = true;
    }
  }
  if (active && active->rideGen > newGen) {
    resume = *active;
    newer = true;
  }

  if (newer) {
    finishPublishCont(t, fs, resume.rideGen, &resume);
    t->out.recordGps = true;
    t->out.showPending = false;
    if (active && resume.rideGen != active->rideGen && active->rideGen > resume.rideGen) {
      return finishHold(t, FINISH_ERROR);
    }
    return finishRepairSlots(t, fs, &rec, &resume, &a, ha, &b, hb, hl);
  }

  // No newer checkpoint. A completed-generation slot must not be resumed.
  if (active && active->rideGen != 0 && active->rideGen < rec.fromRideGen) {
    return finishHold(t, FINISH_ERROR);
  }

  int files = finishStepFiles(t, fs, cfg, &rec, newGen);
  if (files != FINISH_FILES_DONE) {
    return files;
  }

  RideSidecar side;
  if (finishReadSidecar(fs, newGen, &side) != 1) {
    return finishHold(t, FINISH_ERROR);
  }
  if (rec.contGpxEnd == 0) {
    rec.contGpxEnd = side.gpxBodyEnd;
  }
  finishWantFromRec(&rec, newGen, &contWant);
  finishPublishCont(t, fs, newGen, &contWant);
  t->out.points = rec.contPoints;
  t->out.recordGps = true;
  // Journal continuation gpx end may still be 0 until the next slot write.
  if (recIn->contGpxEnd != rec.contGpxEnd) {
    FinishDatV2 next = *recIn;
    next.contGpxEnd = rec.contGpxEnd;
    next.seq = finishNextSeq(recIn->seq);
    int slot = 0;
    FinishLoad ld;
    if (finishLoadJournal(fs, &ld) == LD_V2) {
      next = ld.rec;
      next.contGpxEnd = rec.contGpxEnd;
      if (t->contDirty) {
        finishFillCont(&next, &t->cont);
        next.contGpxEnd = rec.contGpxEnd;
      }
      next.seq = finishNextSeq(ld.rec.seq);
      slot = (ld.slot == 0) ? 1 : 0;
    }
    if (finishWriteJournalSlot(fs, slot, &next) != 0) {
      return finishHold(t, FINISH_PENDING);
    }
    t->contDirty = false;
    return FINISH_YIELD;
  }
  return finishRepairSlots(t, fs, &rec, &contWant, &a, ha, &b, hb, hl);
}

static inline int finishTxnStep(FinishTxn *t, FinishFs *fs, const FinishCfg *cfg) {
  FinishLoad ld;
  int kind = finishLoadJournal(fs, &ld);
  if (kind == LD_UNREAD) {
    return finishNoteUnread(t);
  }
  if (kind == LD_ABSENT) {
    if (t->liveAttempt && !t->committed) {
      t->out.folded = true;
      t->out.foldedFrom = t->attempt;
      t->out.recordGps = true;
      return finishSet(t, FINISH_ROLLED_BACK);
    }
    t->out.journalLive = false;
    t->out.recordGps = true;
    return finishSet(t, FINISH_ABSENT);
  }
  if (kind == LD_INVALID) {
    if (t->liveAttempt && !t->committed && !finishArtifacts(fs, &t->attempt)) {
      return finishRollback(t, fs);
    }
    return finishHold(t, FINISH_ERROR);
  }
  if (kind == LD_V1) {
    return finishStepV1(t, fs, &ld.v1);
  }
  if (t->liveAttempt && t->haveAttempt && finishSameIdentity(&ld.rec, &t->attempt)) {
    t->committed = true;
  } else if (t->liveAttempt && t->haveAttempt && !t->committed) {
    // A valid journal that is not this attempt.
    return finishHold(t, FINISH_PENDING);
  } else if (kind == LD_V2) {
    t->committed = true;
  }
  return finishStepV2(t, fs, cfg, &ld.rec);
}

static inline int finishTxnRun(FinishTxn *t, FinishFs *fs, const FinishCfg *cfg) {
  for (int i = 0; i < 48; i++) {
    int code = finishTxnStep(t, fs, cfg);
    if (code != FINISH_YIELD) {
      return code;
    }
  }
  return finishHold(t, FINISH_ERROR);
}

static inline int finishTxnBoot(FinishTxn *t, FinishFs *fs, const FinishCfg *cfg) {
  t->liveAttempt = false;
  t->committed = false;
  t->haveAttempt = false;
  return finishTxnRun(t, fs, cfg);
}

static inline int finishTxnBegin(FinishTxn *t, FinishFs *fs, const FinishCfg *cfg, const FinishSnap *snap) {
  (void)cfg;
  t->liveAttempt = true;
  t->committed = false;
  t->haveAttempt = true;
  t->attempt = *snap;
  t->contDirty = false;
  memset(&t->cont, 0, sizeof(t->cont));
  FinishLoad existing;
  int kind = finishLoadJournal(fs, &existing);
  if (kind == LD_UNREAD) {
    return finishHold(t, FINISH_PENDING);
  }
  if (kind == LD_V2 || kind == LD_V1) {
    t->liveAttempt = false;
    t->committed = true;
    return FINISH_YIELD;
  }
  if (kind == LD_INVALID) {
    return finishHold(t, FINISH_ERROR);
  }
  FinishDatV2 rec = finishFromSnap(snap);
  int slot = 0;
  if (finishWriteJournalSlot(fs, slot, &rec) != 0) {
    FinishLoad again;
    int ak = finishLoadJournal(fs, &again);
    if (ak == LD_UNREAD) {
      return finishHold(t, FINISH_PENDING);
    }
    if (ak == LD_V2 && finishSameIdentity(&again.rec, snap)) {
      // The write's confirming read failed. A later read shows this attempt
      // committed, so do not roll back and do not resume ordinary recording.
      t->committed = true;
      return finishHold(t, FINISH_PENDING);
    }
    if (ak == LD_V2 || ak == LD_V1) {
      return finishHold(t, FINISH_PENDING);
    }
    if (finishArtifacts(fs, snap)) {
      return finishHold(t, FINISH_PENDING);
    }
    return finishRollback(t, fs);
  }
  t->committed = true;
  return FINISH_YIELD;
}

static inline void finishTxnNoteCont(FinishTxn *t, const FinishContRam *c) {
  t->cont = *c;
  t->contDirty = true;
}

static inline int finishTxnSyncCont(FinishTxn *t, FinishFs *fs) {
  return finishSyncCont(t, fs);
}

// Active file for a checkpoint generation: the generation name when that file
// exists, otherwise the legacy CURRENT.GPX name. A sidecar alone is not a signal.
static inline bool finishActiveTrackName(FinishFs *fs, uint32_t gen, char *out, size_t n) {
  char genFile[13];
  if (finishGenName(gen, 'C', "GPX", genFile, sizeof(genFile)) && finishStat(fs, genFile) == 1) {
    return finishCopyName(out, n, genFile);
  }
  return finishCopyName(out, n, FINISH_CURRENT_GPX);
}
