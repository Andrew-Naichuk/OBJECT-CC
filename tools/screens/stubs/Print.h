#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <string>
class __FlashStringHelper;
class String : public std::string {
 public:
  using std::string::string;
  String() {}
  String(const std::string &s) : std::string(s) {}
};
#define DEC 10
#define HEX 16

class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t *b, size_t n) {
    size_t k = 0;
    while (n--) k += write(*b++);
    return k;
  }
  size_t write(const char *s) { return s ? write((const uint8_t *)s, strlen(s)) : 0; }
  size_t write(const char *b, size_t n) { return write((const uint8_t *)b, n); }

  size_t print(const char *s) { return write(s); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(unsigned long v, int base = DEC) { return num(v, base, false); }
  size_t print(long v, int base = DEC) {
    return v < 0 ? (print('-') + num((unsigned long)(-v), base, false)) : num(v, base, false);
  }
  size_t print(unsigned int v, int base = DEC) { return print((unsigned long)v, base); }
  size_t print(int v, int base = DEC) { return print((long)v, base); }
  size_t print(unsigned char v, int base = DEC) { return print((unsigned long)v, base); }
  size_t print(double v, int digits = 2) {
    char b[48];
    snprintf(b, sizeof b, "%.*f", digits, v);
    return write(b);
  }
  template <typename T>
  size_t println(T v) { size_t n = print(v); return n + println(); }
  template <typename T>
  size_t println(T v, int f) { size_t n = print(v, f); return n + println(); }
  size_t println() { return write("\r\n"); }

 private:
  size_t num(unsigned long v, int base, bool) {
    char b[40];
    snprintf(b, sizeof b, base == HEX ? "%lX" : "%lu", v);
    return write(b);
  }
};
