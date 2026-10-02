// In-memory FAT model with just the SdFat surface the firmware touches.
#pragma once
#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <stdio.h>
#include <string.h>
#include "SPI.h"

typedef int oflag_t;
#undef O_RDONLY
#define O_RDONLY 0x00
#undef O_WRONLY
#define O_WRONLY 0x01
#undef O_RDWR
#define O_RDWR   0x02
#undef O_ACCMODE
#define O_ACCMODE 0x03
#undef O_CREAT
#define O_CREAT  0x40
#undef O_EXCL
#define O_EXCL   0x80
#undef O_TRUNC
#define O_TRUNC  0x200
#define SHARED_SPI 1
#define SD_SCK_MHZ(m) ((m) * 1000000UL)

struct SdSpiConfig {
  SdSpiConfig(int, int, unsigned long, SPIClass *) {}
};

// Card contents and fault switches, owned by the harness.
struct EmuCard {
  bool present = true;
  bool failRename = false;
  std::map<std::string, std::shared_ptr<std::vector<uint8_t>>> files;
};
extern EmuCard g_card;
void emuOnSdBegin();

class File32 {
 public:
  bool open(const char *path, oflag_t flags = O_RDONLY) {
    close();
    if (!g_card.present) return false;
    std::string p(path);
    if (p == "/") { isDir_ = true; open_ = true; dirIdx_ = 0; return true; }
    auto it = g_card.files.find(p);
    if (it == g_card.files.end()) {
      if (!(flags & O_CREAT)) return false;
      g_card.files[p] = std::make_shared<std::vector<uint8_t>>();
      it = g_card.files.find(p);
    } else if (flags & O_EXCL && flags & O_CREAT) {
      return false;
    }
    data_ = it->second;
    name_ = p;
    if (flags & O_TRUNC) data_->clear();
    pos_ = 0;
    open_ = true;
    return true;
  }
  bool openNext(File32 *dir, oflag_t = O_RDONLY) {
    close();
    if (!dir || !dir->isDir_) return false;
    size_t i = 0;
    for (auto &kv : g_card.files) {
      if (i++ == dir->dirIdx_) {
        dir->dirIdx_++;
        data_ = kv.second; name_ = kv.first; pos_ = 0; open_ = true;
        return true;
      }
    }
    return false;
  }
  bool isDir() const { return isDir_; }
  size_t getName(char *b, size_t n) {
    snprintf(b, n, "%s", name_.c_str());
    return strlen(b);
  }
  void close() { open_ = false; isDir_ = false; data_.reset(); }
  bool isOpen() const { return open_; }
  int read() {
    if (!data_ || pos_ >= data_->size()) return -1;
    return (*data_)[pos_++];
  }
  int read(void *b, size_t n) {
    if (!data_) return -1;
    size_t k = 0;
    while (k < n && pos_ < data_->size()) ((uint8_t *)b)[k++] = (*data_)[pos_++];
    return (int)k;
  }
  size_t write(const void *b, size_t n) {
    if (!data_) return 0;
    if (data_->size() < pos_ + n) data_->resize(pos_ + n);
    memcpy(data_->data() + pos_, b, n);
    pos_ += n;
    return n;
  }
  size_t write(uint8_t c) { return write(&c, 1); }
  bool seekSet(uint32_t p) { if (!data_ || p > data_->size()) return false; pos_ = p; return true; }
  uint32_t curPosition() const { return (uint32_t)pos_; }
  uint32_t fileSize() const { return data_ ? (uint32_t)data_->size() : 0; }
  bool truncate(uint32_t n) { if (!data_) return false; data_->resize(n); if (pos_ > n) pos_ = n; return true; }
  bool truncate() { return truncate((uint32_t)pos_); }
  bool flush() { return true; }
  int available() { return data_ ? (int)(data_->size() - pos_) : 0; }
  operator bool() const { return open_; }

 private:
  std::shared_ptr<std::vector<uint8_t>> data_;
  std::string name_;
  size_t pos_ = 0;
  bool open_ = false;
  bool isDir_ = false;
  size_t dirIdx_ = 0;
};

class SdFat {
 public:
  bool begin(const SdSpiConfig &) { emuOnSdBegin(); return g_card.present; }
  bool exists(const char *p) { return g_card.files.count(p) > 0; }
  bool remove(const char *p) { return g_card.files.erase(p) > 0; }
  bool rename(const char *a, const char *b) {
    if (g_card.failRename) return false;
    auto it = g_card.files.find(a);
    if (it == g_card.files.end() || g_card.files.count(b)) return false;
    g_card.files[b] = it->second;
    g_card.files.erase(a);
    return true;
  }
};
