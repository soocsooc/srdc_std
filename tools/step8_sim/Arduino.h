// PC シミュレーション用の Arduino.h の代わり。STEP8 の迷路ロジックが使う分だけを用意する
#ifndef SIM_ARDUINO_H_
#define SIM_ARDUINO_H_

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>

// Arduino の min/max は型の違う引数を受け付けるマクロなので、それに合わせる
template <class A, class B>
inline typename std::common_type<A, B>::type min(A a, B b)
{
  return (b < a) ? b : a;
}
template <class A, class B>
inline typename std::common_type<A, B>::type max(A a, B b)
{
  return (a < b) ? b : a;
}
#define PI 3.1415926535897932384626433832795
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

extern unsigned long g_sim_ms;  // delay() で進む仮想時刻[ms]
extern bool g_sim_verbose;      // true のとき Serial の出力を表示する
inline void delay(unsigned long ms) { g_sim_ms += ms; }
inline unsigned long millis(void) { return g_sim_ms; }

class String
{
public:
  std::string s;
  String(const char * c = "") : s(c) {}
  String(const std::string & c) : s(c) {}
  String(int v) : s(std::to_string(v)) {}
  const char * c_str(void) const { return s.c_str(); }
  String operator+(const String & o) const { return String(s + o.s); }
  String & operator+=(const String & o)
  {
    s += o.s;
    return *this;
  }
};

class SimSerial
{
public:
  // printf() の出力を 1 回ごとに受け取る関数。ハーネスが Serial の値を読むのに使う(nullptr なら使わない)
  void (*hook)(const char * s) = nullptr;

  int printf(const char * fmt, ...)
  {
    if (!g_sim_verbose && !hook) return 0;
    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (hook) hook(buf);
    if (g_sim_verbose) fputs(buf, stdout);
    return n;
  }
  void print(const char * s)
  {
    if (g_sim_verbose) fputs(s, stdout);
  }
  void print(const String & s) { print(s.c_str()); }
  void print(long v)
  {
    if (g_sim_verbose) ::printf("%ld", v);
  }
  void println(void) { print("\n"); }
  template <class T>
  void println(const T & v)
  {
    print(v);
    print("\n");
  }
};
extern SimSerial Serial;

#endif  // SIM_ARDUINO_H_
