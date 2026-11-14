// ============================================================
// File    : tools/dbd_test/main.cpp
// Role    : gnss_dbd.cpp のホスト単体テスト。
//           1) gnss_dbd_frames_ok: 256B 境界を跨ぐ複数 MGA-DBD フレームの検証と破損・件数不一致の拒絶
//           2) コールド起動時の復元（ST_BOOT → ST_SEND → notime 発生 → ST_WAIT → 時刻注入 → resend → ST_IDLE）
//           3) 吸い出し（ST_DUMP）と 0 件応答・件数不一致時の既存ファイル保護（BUG-66）
//           4) コールド試験（PONS:DBD,COLDONLY）中の時刻注入拒否と fix/time 落下待ち（BUG-71）
// ============================================================
#include "Arduino.h"
#include "gnss_dbd.h"
#include "settings.h"

uint32_t    g_test_ms = 1000;
DummySerial Serial;
DummyUart   Serial1;
mutex_t     taskQueueMutex = 0;

GnssDate g_gnss_date;
GnssTime g_gnss_time;
bool     g_rtc_checked  = false;
bool     g_rtc_alive    = false;
bool     g_sig_settled  = true;
bool     g_gnss_conn    = true;
bool     g_own_fix      = false;
double   g_own_gs       = 0.0;
uint32_t g_fix_millis   = 0;
int      g_last_ubx_cls = -1;
int      g_last_ubx_id  = -1;
uint8_t  g_last_ubx_payload[64];
uint16_t g_last_ubx_len = 0;
int      g_ubx_send_count = 0;

bool g_replay_mode    = false;
bool g_queue_accept   = true;
int  g_save_dbd_tasks = 0;
int  g_load_dbd_tasks = 0;
char g_last_log[256]  = "";
char g_log_history[4096] = "";

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg, ...) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] line %d: " msg "\n", __LINE__, ##__VA_ARGS__); } \
} while (0)

static void ubx_fletcher(const uint8_t* d, uint32_t n, uint8_t& a, uint8_t& b) {
    a = 0; b = 0;
    for (uint32_t i = 0; i < n; i++) { a += d[i]; b += a; }
}

// 1 通の UBX-MGA-DBD フレーム（0xB5 0x62 0x13 0x80 len payload ckA ckB）を dst に組み立てて長さを返す
static uint32_t build_dbd_frame(uint8_t* dst, uint32_t tag, uint16_t payload_len) {
    dst[0] = 0xB5; dst[1] = 0x62; dst[2] = 0x13; dst[3] = 0x80;
    dst[4] = (uint8_t)(payload_len & 0xFF);
    dst[5] = (uint8_t)(payload_len >> 8);
    // 先頭 4 バイトに識別用の tag を入れ、残りをパターンで埋める
    dst[6] = (uint8_t)(tag & 0xFF);
    dst[7] = (uint8_t)((tag >> 8) & 0xFF);
    dst[8] = (uint8_t)((tag >> 16) & 0xFF);
    dst[9] = (uint8_t)((tag >> 24) & 0xFF);
    for (uint16_t i = 4; i < payload_len; i++) dst[6 + i] = (uint8_t)((tag + i) & 0xFF);
    uint8_t ca, cb;
    ubx_fletcher(dst + 2, 4u + payload_len, ca, cb);
    dst[6 + payload_len] = ca;
    dst[7 + payload_len] = cb;
    return 8u + payload_len;
}

// MGA-ACK (0x13 0x60, 8B) を gnss_dbd_on_ubx へ渡す
static void feed_mga_ack(uint8_t type_ok, uint8_t info_code, uint8_t msg_id, const uint8_t head4[4]) {
    uint8_t p[8] = { type_ok, 0x00, info_code, msg_id, head4[0], head4[1], head4[2], head4[3] };
    gnss_dbd_on_ubx(0x13, 0x60, p, 8);
}

static void test_frames_ok() {
    // 256B Flash ページ境界を跨ぐ 5 通のフレーム列（合計 > 600B）を作る
    uint8_t buf[1024];
    uint32_t off = 0;
    const uint16_t lens[5] = { 72, 136, 164, 88, 120 };
    for (int i = 0; i < 5; i++) {
        off += build_dbd_frame(buf + off, 0x1000 + i, lens[i]);
    }
    CHECK(off > 512, "multi-page frame buffer size=%u", off);
    CHECK(gnss_dbd_frames_ok(buf, off, 5), "valid 5 frames across 256B boundaries");
    CHECK(!gnss_dbd_frames_ok(buf, off, 4), "reject count mismatch (4 vs 5)");
    CHECK(!gnss_dbd_frames_ok(buf, off - 1, 5), "reject truncated last byte");

    // 途中の 1 ビット破損を検出できること
    buf[260] ^= 0x04;
    CHECK(!gnss_dbd_frames_ok(buf, off, 5), "reject corrupted byte at 260");
    buf[260] ^= 0x04;
    CHECK(gnss_dbd_frames_ok(buf, off, 5), "restored valid");
}

static void test_boot_restore_and_resend() {
    // 1) Core1 が起動時に 3 通のフレームを読み終え、GNSS から SEC-UNIQID が届き、BT1 が切れていた（rtc_alive=false）
    uint8_t* ram = gnss_dbd_buffer_core1();
    uint32_t off = 0;
    for (int i = 0; i < 3; i++) {
        off += build_dbd_frame(ram + off, 0xA000 + i, 64);
    }
    GnssDbdHead h;
    memset(&h, 0, sizeof(h));
    memcpy(h.magic, GNSS_DBD_MAGIC, 8);
    h.utc   = 1794620000UL; // 2026-11-14 01:33:20 UTC
    h.count = 3;
    h.bytes = off;
    const uint8_t gid[6] = { 0xB3, 0x86, 0x59, 0x8F, 0xAF, 0x54 };
    memcpy(h.gnss_id, gid, 6);
    gnss_dbd_loaded_core1(true, &h, "");

    uint8_t uniq_pl[10] = { 1, 0, 0, 0, 0xB3, 0x86, 0x59, 0x8F, 0xAF, 0x54 };
    gnss_dbd_on_ubx(0x27, 0x03, uniq_pl, 10);

    g_rtc_checked = true;
    g_rtc_alive   = false;
    g_sig_settled = true;

    // ST_BOOT: 1 回目の tick で EP_BOOT 開始、2 回目で start_restore(1) → ST_SEND、3 回目で 1 通目を送信
    g_test_ms = 2000;
    gnss_dbd_tick();
    g_test_ms = 2005;
    gnss_dbd_tick();
    Serial1.clear();
    g_test_ms = 2010;
    gnss_dbd_tick();
    CHECK(Serial1.tx_len == 72, "1st frame sent (len=%u)", Serial1.tx_len);

    // 1 通目: ok (type=1), 2 通目: ok (type=1), 3 通目: notime (type=0, infoCode=1)
    uint8_t tag0[4] = { 0x00, 0xA0, 0x00, 0x00 };
    feed_mga_ack(1, 0, 0x80, tag0);
    g_test_ms = 2030;
    gnss_dbd_tick(); // 2 通目送信
    uint8_t tag1[4] = { 0x01, 0xA0, 0x00, 0x00 };
    feed_mga_ack(1, 0, 0x80, tag1);
    g_test_ms = 2050;
    gnss_dbd_tick(); // 3 通目送信
    uint8_t tag2[4] = { 0x02, 0xA0, 0x00, 0x00 };
    feed_mga_ack(0, 1, 0x80, tag2); // infoCode=1 (notime)
    g_test_ms = 2070;
    gnss_dbd_tick(); // 全 3 通完了 → ST_WAIT へ

    CHECK(strstr(g_log_history, "GNSS DBD RESTORE boot: sent 3 ok 2 notime 1") != nullptr,
          "restore pass 1 log: %s", g_last_log);
    CHECK(gnss_dbd_time_want() == GNSS_TIME_WANT_YES, "wants external time in ST_WAIT");

    // 2) 外からありえない時刻（2020年・DBD保存時刻より前）を渡すと BAD_TIME で弾かれること
    CHECK(strcmp(gnss_dbd_give_time(1580000000000ULL, g_test_ms, GNSS_TIME_SRC_PC), "BAD_TIME") == 0,
          "reject old 2020 time");
    CHECK(strcmp(gnss_dbd_give_time((uint64_t)(h.utc - 120) * 1000ULL, g_test_ms, GNSS_TIME_SRC_PC), "BAD_TIME") == 0,
          "reject time older than saved DBD");

    // 3) 正しい時刻（DBD保存の 1 時間後）を PC から注入 → MGA-INI-TIME_UTC 送信 → ACK 受理 → resend 開始
    const uint64_t valid_utc_ms = (uint64_t)(h.utc + 3600) * 1000ULL;
    g_test_ms = 3000;
    const char* r = gnss_dbd_give_time(valid_utc_ms, g_test_ms, GNSS_TIME_SRC_PC);
    CHECK(strcmp(r, "SENT") == 0, "give_time SENT (got %s)", r);
    CHECK(g_last_ubx_cls == 0x13 && g_last_ubx_id == 0x40 && g_last_ubx_len == 24, "MGA-INI-TIME_UTC sent");

    // MGA-INI に対する MGA-ACK (msgId=0x40, msgPayloadStart[0]=0x10)
    const uint8_t ini_head[4] = { 0x10, 0x00, 0x00, 0x80 };
    g_test_ms = 3040;
    feed_mga_ack(1, 0, 0x40, ini_head);
    CHECK(gnss_dbd_time_want() == GNSS_TIME_WANT_NO, "time want closed after INI accepted");

    // tick を進めると時刻が分かったことを検知して resend (pass 2) へ入り、3 通とも ok で完了すること
    g_test_ms = 3050;
    gnss_dbd_tick(); // episode_tick で時刻判明 & wait_tick で start_restore(2) → ST_SEND
    g_test_ms = 3060;
    gnss_dbd_tick(); // resend 1 通目送信
    feed_mga_ack(1, 0, 0x80, tag0);
    g_test_ms = 3070;
    gnss_dbd_tick(); // resend 2 通目送信
    feed_mga_ack(1, 0, 0x80, tag1);
    g_test_ms = 3080;
    gnss_dbd_tick(); // resend 3 通目送信
    feed_mga_ack(1, 0, 0x80, tag2);
    g_test_ms = 3090;
    gnss_dbd_tick(); // resend 完了 → ST_IDLE

    CHECK(strstr(g_log_history, "GNSS DBD RESTORE resend: sent 3 ok 3 notime 0") != nullptr,
          "restore pass 2 log: %s", g_last_log);

    char tag_buf[64];
    gnss_dbd_fix_tag(tag_buf, sizeof(tag_buf));
    CHECK(strstr(tag_buf, "rtc=lost dbd=2/3 resend=3/3") != nullptr && strstr(tag_buf, "(pc)") != nullptr,
          "fix_tag: %s", tag_buf);
}

static void test_dump_and_zero_msg_guard() {
    // fix して時刻が分かった状態にする
    g_own_fix = true;
    g_own_gs  = 0.0;
    g_gnss_date = { 2026, 11, 14, true };
    g_gnss_time = { 2, 35, 0, 0, true };
    g_fix_millis = 4000;
    g_test_ms = 4000;
    gnss_dbd_tick(); // episode_tick が fix で終了

    // BUG-66: PONS:DBD,SAVE で 0 件の MGA-ACK が返ってきたとき、createSaveDbdTask を積まず既存ファイルを壊さないこと
    const int saves_before = g_save_dbd_tasks;
    g_test_ms = 5000;
    gnss_dbd_command("SAVE");
    CHECK(strstr(Serial.last_out, "PONS:DBD,OK,SAVE") != nullptr, "SAVE started: %s", Serial.last_out);
    const uint8_t zero_cnt[4] = { 0, 0, 0, 0 };
    feed_mga_ack(1, 0, 0x80, zero_cnt);
    CHECK(g_save_dbd_tasks == saves_before, "0-msg dump must not enqueue save task");
    CHECK(strstr(g_last_log, "got 0 msgs but GNSS says 0 (not saved)") != nullptr, "0-msg log: %s", g_last_log);

    // 正常な 2 件の吸い出しで保存タスクが積まれること
    g_test_ms = 6000;
    gnss_dbd_command("SAVE");
    uint8_t dummy_pl[32] = { 1, 2, 3, 4 };
    gnss_dbd_on_ubx(0x13, 0x80, dummy_pl, 32);
    gnss_dbd_on_ubx(0x13, 0x80, dummy_pl, 40);
    const uint8_t two_cnt[4] = { 2, 0, 0, 0 };
    feed_mga_ack(1, 0, 0x80, two_cnt);
    CHECK(g_save_dbd_tasks == saves_before + 1, "2-msg dump enqueued save task");

    GnssDbdHead save_h;
    CHECK(gnss_dbd_head_for_save_core1(&save_h), "head_for_save_core1 ready");
    CHECK(save_h.count == 2 && save_h.bytes == (8 + 32 + 8 + 40), "save head count/bytes");
    gnss_dbd_saved_core1(true, "", 15);
    g_test_ms = 6100;
    gnss_dbd_tick(); // ST_SAVING → ST_IDLE
}

static void test_coldonly_bug71() {
    // BUG-71: PONS:DBD,COLDONLY を打った直後、fix が落ちていても NAV-PVT の日時がまだ残っている間は
    // 「時刻が分かった」と誤判定せず、また外部からの時刻注入も "TEST" で拒むこと。
    g_own_fix = false; // fix はまだ無いが gnss_time_known() は true（リセット前の NAV-PVT の残り）
    g_gnss_date = { 2026, 11, 14, true };
    g_gnss_time = { 2, 40, 0, 0, true };
    g_test_ms = 10000;
    gnss_dbd_command("COLDONLY");
    CHECK(strstr(Serial.last_out, "PONS:DBD,OK,COLDONLY") != nullptr, "COLDONLY accepted");

    // 試験中に PC から時刻が来ても "TEST" で拒むこと
    const uint64_t valid_utc_ms = 1794624000000ULL;
    CHECK(strcmp(gnss_dbd_give_time(valid_utc_ms, g_test_ms, GNSS_TIME_SRC_PC), "TEST") == 0,
          "give_time rejected with TEST during COLDONLY");

    // リセット直後（時刻がまだ落ちていない）の tick では「GNSS time known」にならないこと
    g_last_log[0] = '\0';
    g_test_ms = 10500;
    gnss_dbd_tick();
    CHECK(strstr(g_last_log, "GNSS time") == nullptr, "did not latch stale pre-reset time");

    // リセットが効いて時刻も一度落ちる
    g_gnss_date._valid = false;
    g_gnss_time._valid = false;
    g_test_ms = 11000;
    gnss_dbd_tick();

    // その後、衛星から時刻が読めたら初めて "GNSS time known" が出ること
    g_gnss_date = { 2026, 11, 14, true };
    g_gnss_time = { 2, 40, 15, 0, true };
    g_test_ms = 25000;
    gnss_dbd_tick();
    CHECK(strstr(g_last_log, "GNSS DBD: GNSS time known 15.0s after reset") != nullptr,
          "latched real time after drop: %s", g_last_log);
}

int main() {
    test_frames_ok();
    test_boot_restore_and_resend();
    test_dump_and_zero_msg_guard();
    test_coldonly_bug71();

    printf("dbd_test: %d passed, %d failed\n", g_pass, g_fail);
    printf("結果: %s\n", g_fail == 0 ? "PASS — GNSS DBD 全テスト合格" : "FAIL");
    return g_fail == 0 ? 0 : 1;
}
