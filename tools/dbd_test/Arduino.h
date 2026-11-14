// tools/dbd_test 用の最小 Arduino.h スタブ
// gnss_dbd.h の先頭で #include <Arduino.h> されるため、ここで周辺モジュール（gnss / mysd / imu）の
// インクルードガードを立てて最小スタブを提供し、実機の gnss_dbd.cpp を無改造でコンパイルする。
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include "../../lora_link/link_proto.h"

extern uint32_t g_test_ms;
static inline uint32_t millis() { return g_test_ms; }
static inline uint32_t time_us_32() { return g_test_ms * 1000UL; }

struct DummySerial {
    char last_out[256] = "";
    void println(const char* s) { snprintf(last_out, sizeof(last_out), "%s\n", s); }
    void print(const char* s)   { snprintf(last_out, sizeof(last_out), "%s", s); }
    void printf(const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(last_out, sizeof(last_out), fmt, ap);
        va_end(ap);
    }
};
extern DummySerial Serial;

struct DummyUart {
    int      avail_write = 256;
    uint8_t  tx_buf[65536];
    uint32_t tx_len = 0;
    int  availableForWrite() const { return avail_write; }
    void write(uint8_t b) {
        if (tx_len < sizeof(tx_buf)) tx_buf[tx_len++] = b;
    }
    void clear() { tx_len = 0; }
};
extern DummyUart Serial1;

// ---- mutex スタブ ----
typedef int mutex_t;
extern mutex_t taskQueueMutex;
static inline void mutex_enter_blocking(mutex_t*) {}
static inline void mutex_exit(mutex_t*) {}

// ---- gnss.h スタブ ----
#define GNSS_H
struct GnssDate {
    uint16_t _year = 0; uint8_t _month = 0; uint8_t _day = 0; bool _valid = false;
    bool     isValid() const { return _valid; }
    uint16_t year()    const { return _year;  }
    uint8_t  month()   const { return _month; }
    uint8_t  day()     const { return _day;   }
};
struct GnssTime {
    uint8_t _hour = 0; uint8_t _min = 0; uint8_t _sec = 0; uint8_t _cs = 0; bool _valid = false;
    bool    isValid()     const { return _valid; }
    uint8_t hour()        const { return _hour;  }
    uint8_t minute()      const { return _min;   }
    uint8_t second()      const { return _sec;   }
    uint8_t centisecond() const { return _cs;    }
};

extern GnssDate g_gnss_date;
extern GnssTime g_gnss_time;
extern bool     g_rtc_checked;
extern bool     g_rtc_alive;
extern bool     g_sig_settled;
extern bool     g_gnss_conn;
extern bool     g_own_fix;
extern double   g_own_gs;
extern uint32_t g_fix_millis;
extern int      g_last_ubx_cls;
extern int      g_last_ubx_id;
extern uint8_t  g_last_ubx_payload[64];
extern uint16_t g_last_ubx_len;
extern int      g_ubx_send_count;

static inline GnssDate get_gnss_date()                 { return g_gnss_date; }
static inline GnssTime get_gnss_time()                 { return g_gnss_time; }
static inline uint32_t gnss_ubx_ck_errors()            { return 0; }
static inline uint32_t gnss_rx_overflows()             { return 0; }
static inline int      gnss_rx_backlog_peak_take()     { return 0; }
static inline bool     gnss_rtc_boot_checked()         { return g_rtc_checked; }
static inline bool     gnss_rtc_alive_on_boot()        { return g_rtc_alive; }
static inline bool     gnss_signal_cfg_settled()       { return g_sig_settled; }
static inline bool     get_gnss_connection()           { return g_gnss_conn; }
static inline uint32_t get_gnss_fix_millis()           { return g_fix_millis; }
static inline int      get_gsa_numsat()                { return 12; }
static inline bool     get_gnss_ana_busy()             { return false; }
static inline int      get_gnss_ana_data_svs()         { return 20; }
static inline void     gnss_ana_off_for_cold_test()    {}
static inline bool     gnss_get_own_fix(double& lat, double& lon, double& gs, double& trk) {
    lat = 35.29; lon = 136.25; gs = g_own_gs; trk = 0.0;
    return g_own_fix;
}
static inline void gnss_ubx_send(uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len) {
    g_last_ubx_cls = cls;
    g_last_ubx_id  = id;
    g_last_ubx_len = len;
    if (p && len <= sizeof(g_last_ubx_payload)) memcpy(g_last_ubx_payload, p, len);
    g_ubx_send_count++;
}

// ---- mysd.h スタブ ----
#define MYSD_H
enum StubTaskType { STUB_TASK_LOG = 1, STUB_TASK_SAVE_DBD = 2, STUB_TASK_LOAD_DBD = 3 };
extern bool g_replay_mode;
extern bool g_queue_accept;
extern int  g_save_dbd_tasks;
extern int  g_load_dbd_tasks;
extern char g_last_log[256];
extern char g_log_history[4096];

static inline bool getReplayMode() { return g_replay_mode; }
static inline int createLogSdfTask(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_last_log, sizeof(g_last_log), fmt, ap);
    va_end(ap);
    size_t cur = strlen(g_log_history);
    if (cur + strlen(g_last_log) + 2 < sizeof(g_log_history)) {
        strcat(g_log_history, g_last_log);
        strcat(g_log_history, "\n");
    }
    return STUB_TASK_LOG;
}
static inline int createSaveDbdTask() { return STUB_TASK_SAVE_DBD; }
static inline int createLoadDbdTask() { return STUB_TASK_LOAD_DBD; }
static inline bool enqueueTask(int t) {
    if (!g_queue_accept) return false;
    if (t == STUB_TASK_SAVE_DBD) g_save_dbd_tasks++;
    if (t == STUB_TASK_LOAD_DBD) g_load_dbd_tasks++;
    return true;
}

// ---- imu.h スタブ ----
#define IMU_H
static inline void imu_service_if_due() {}
