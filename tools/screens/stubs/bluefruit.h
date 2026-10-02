// Minimal Bluefruit model: one peripheral link the harness can open/close.
#pragma once
#include <stdint.h>

#define BLE_GAP_PHY_2MBPS 2
#define BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE 6
#define CHR_PROPS_WRITE 0x08
#define CHR_PROPS_WRITE_WO_RESP 0x04
#define CHR_PROPS_NOTIFY 0x10
#define SECMODE_NO_ACCESS 0
#define SECMODE_OPEN 1

class BLEConnection {
 public:
  bool requestMtuExchange(uint16_t) { return true; }
  bool requestPHY(uint8_t) { return true; }
  uint16_t getMtu() { return 247; }
};

struct EmuBle {
  bool linked = false;
  bool stall = false;   // phone stops acknowledging notifications
  void (*onConnect)(uint16_t) = nullptr;
  void (*onDisconnect)(uint16_t, uint8_t) = nullptr;
  BLEConnection conn;
};
extern EmuBle g_ble;

class BLEService {
 public:
  explicit BLEService(const char *) {}
  bool begin() { return true; }
};

class BLECharacteristic {
 public:
  typedef void (*write_cb_t)(uint16_t, BLECharacteristic *, uint8_t *, uint16_t);
  BLECharacteristic(const char *, uint8_t, uint16_t) {}
  void setPermission(int, int) {}
  void setWriteCallback(write_cb_t, bool = true) {}
  bool begin() { return true; }
  bool notifyEnabled() { return g_ble.linked; }
  bool notify(const uint8_t *, uint16_t) { return g_ble.linked && !g_ble.stall; }
};

struct EmuAdvertising {
  void addFlags(int) {}
  void addTxPower() {}
  void addService(BLEService &) {}
  void addName() {}
  void clearData() {}
  void restartOnDisconnect(bool) {}
  void setInterval(int, int) {}
  void setFastTimeout(int) {}
  bool start(int) { return true; }
  bool stop() { return true; }
};

struct EmuPeriph {
  void setConnectCallback(void (*f)(uint16_t)) { g_ble.onConnect = f; }
  void setDisconnectCallback(void (*f)(uint16_t, uint8_t)) { g_ble.onDisconnect = f; }
  void setConnIntervalMS(int, int) {}
};

class EmuBluefruit {
 public:
  EmuAdvertising Advertising, ScanResponse;
  EmuPeriph Periph;
  void autoConnLed(bool) {}
  void configPrphConn(int, int, int, int) {}
  bool begin(int, int) { return true; }
  void setTxPower(int) {}
  void setName(const char *) {}
  uint8_t connected() { return g_ble.linked ? 1 : 0; }
  uint16_t connHandle() { return 0; }
  BLEConnection *Connection(uint16_t) { return g_ble.linked ? &g_ble.conn : nullptr; }
  bool disconnect(uint16_t h) {
    if (!g_ble.linked) return false;
    g_ble.linked = false;
    if (g_ble.onDisconnect) g_ble.onDisconnect(h, 0x13);
    return true;
  }
};
extern EmuBluefruit Bluefruit;
