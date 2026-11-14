// tools/vario_test 用の最小 Arduino.h スタブ
// imu.cpp の先頭で #include <Arduino.h> されるため、ここで周辺モジュール（link / airdata / mysd / gnss）の
// インクルードガードを立てて最小スタブを提供することで、実機の imu.cpp を無改造でコンパイルする。
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include "../../lora_link/link_proto.h"

extern uint32_t g_test_us;
static inline uint32_t time_us_32() { return g_test_us; }
static inline uint32_t millis() { return g_test_us / 1000UL; }
static inline void test_set_us(uint32_t t) { g_test_us = t; }
static inline void test_advance_us(uint32_t d) { g_test_us += d; }

struct DummySerial {
    void println(const char*) {}
    void print(const char*) {}
    void printf(const char*, ...) {}
};
extern DummySerial Serial;

// ---- link.h スタブ ----
#define LINK_H
extern bool     g_stub_mirror_active;
extern float    g_stub_link_vs;
extern float    g_stub_link_alt;
extern uint16_t g_stub_link_have;
extern uint16_t g_stub_link_status;
static inline bool     link_mirror_active()          { return g_stub_mirror_active; }
static inline float    link_get_kf_vspeed()          { return g_stub_link_vs; }
static inline float    link_get_kf_altitude()        { return g_stub_link_alt; }
static inline bool     link_has_value(uint16_t bit)  { return (g_stub_link_have & bit) != 0; }
static inline uint16_t link_sender_status()          { return g_stub_link_status; }

// ---- airdata.h スタブ ----
#define AIRDATA_H
extern bool  g_stub_airdata_ok;
extern float g_stub_airdata_vs;
extern float g_stub_ground_alt_adj;
static inline bool  get_airdata_ok()                   { return g_stub_airdata_ok; }
static inline float get_airdata_vspeed()               { return g_stub_airdata_vs; }
static inline void  airdata_adjust_ground_alt(float d) { g_stub_ground_alt_adj += d; }

// ---- mysd.h スタブ ----
#define MYSD_H
extern bool g_stub_replay_mode;
extern int  g_stub_log_count;
extern char g_stub_last_log[256];
static inline bool getReplayMode() { return g_stub_replay_mode; }
static inline int createLogSdfTask(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_stub_last_log, sizeof(g_stub_last_log), fmt, ap);
    va_end(ap);
    g_stub_log_count++;
    return 1;
}
static inline bool enqueueTask(int) { return true; }

// ---- gnss.h スタブ ----
#define GNSS_H
extern uint32_t g_stub_itow_ms;
extern uint16_t g_stub_replay_have;
extern float    g_stub_replay_vs;
extern float    g_stub_replay_alt;
static inline uint32_t get_gnss_itow_ms()             { return g_stub_itow_ms; }
static inline bool     replay_has_value(uint16_t bit) { return (g_stub_replay_have & bit) != 0; }
static inline float    replay_get_kf_vspeed()         { return g_stub_replay_vs; }
static inline float    replay_get_kf_altitude()       { return g_stub_replay_alt; }
