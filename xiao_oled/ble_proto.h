// Protocol v2 command bytes shared by the sketch and the host test.
// New commands: 0xA5, version 2, op, uint16 request id, payload.
// Old commands start with op 0x01, 0x02, or 0x03 and must not be parsed as v2.
#pragma once

#include <stdint.h>
#include <string.h>

static const uint8_t BLE_PROTO_MAGIC = 0xA5;
static const uint8_t BLE_PROTO_VERSION = 2;

enum {
  BLE_OP_LIST = 0x01,
  BLE_OP_GET = 0x02,
  BLE_OP_ABORT = 0x03
};

enum {
  BLE_PARSE_REJECT = 0,
  BLE_PARSE_LEGACY = 1,
  BLE_PARSE_V2 = 2
};

// Meta notifications (v2), little-endian request id after the type byte:
//   ENTRY    type, id, name[12], size u32, flags u8   (flags bit0 = current ride)
//   LIST_END type, id
//   START    type, id, name[12], size u32
//   DONE     type, id, crc u32
//   END_TIME type, id, unix u32
//   ERROR    type, id, code u8
// Data notifications: id u16, offset u32, payload.
// A legacy META_ERROR is type, code (2 bytes) and is only sent to an old page.

static int bleParseCommand(const uint8_t *data, uint16_t len,
                           uint8_t *op, uint16_t *reqId,
                           char *name, uint16_t nameCap) {
  if (op) {
    *op = 0;
  }
  if (reqId) {
    *reqId = 0;
  }
  if (name && nameCap > 0) {
    name[0] = '\0';
  }
  if (!data || len < 1) {
    return BLE_PARSE_REJECT;
  }
  if (data[0] == BLE_OP_LIST || data[0] == BLE_OP_GET || data[0] == BLE_OP_ABORT) {
    return BLE_PARSE_LEGACY;
  }
  if (len < 5 || data[0] != BLE_PROTO_MAGIC || data[1] != BLE_PROTO_VERSION) {
    return BLE_PARSE_REJECT;
  }
  uint8_t opc = data[2];
  if (opc != BLE_OP_LIST && opc != BLE_OP_GET && opc != BLE_OP_ABORT) {
    return BLE_PARSE_REJECT;
  }
  if (op) {
    *op = opc;
  }
  if (reqId) {
    *reqId = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
  }
  if (name && nameCap > 0 && opc == BLE_OP_GET && len > 5) {
    uint16_t n = (uint16_t)(len - 5);
    if (n >= nameCap) {
      n = (uint16_t)(nameCap - 1);
    }
    memcpy(name, data + 5, n);
    name[n] = '\0';
  }
  return BLE_PARSE_V2;
}

// Ids on one GATT connection start at 1 and only increase. 0 and any reuse
// (including a wrap back through 1) are refused. ABORT carries the job's id
// and is checked against the active job by the caller.
static inline bool bleReqIsNew(uint16_t lastAccepted, uint16_t reqId) {
  if (reqId == 0) {
    return false;
  }
  if (lastAccepted != 0 && reqId <= lastAccepted) {
    return false;
  }
  return true;
}
