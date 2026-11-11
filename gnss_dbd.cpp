// ============================================================
// File    : gnss_dbd.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS の航法データベース（UBX-MGA-DBD）を SD に写しておき、時計の電池（BT1）が切れていた起動で
//           GNSS へ戻す（2026-11-09〜）。コールドスタートを縮めるため。無線は使わない。
//           外から来た時刻（PC の PONS:TIME。のちに LoRa）を GNSS へ入れる口もここ（2026-11-10〜）。
//           v7 と v7.1 で同じ中身。計画と根拠は docs/pons_ttff_assist_plan.md §3。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/10
// ============================================================
//
// ■ 何をするか
//   - 吸い出し（保存）: 起動後最初の fix から GNSS_DBD_FIRST_SAVE_MS 後、その後は GNSS_DBD_SAVE_INTERVAL_MS ごとに、
//     UBX-MGA-DBD のポールを送り、返ってくる MGA-DBD のフレームを RAM（s_buf）にためる。最後に来る MGA-ACK が
//     「吐き出した数」を言うので、受けた数と合ったときだけ Core1 が SD の GNSS_DBD_FILE に書く。
//   - 戻す（復元）: 起動直後の NAV-PVT で時計が死んでいた（BT1 が切れていた）ときだけ、SD のフレームを
//     1 通ずつ GNSS へ書く。1 通ごとに MGA-ACK を待つ（gnss.cpp の設定 ⑩ CFG-NAVSPG-ACKAIDING）。
//     BT1 が生きていれば GNSS の BBR に新しいものがあるので触らない。
//   - 入れ直し: 時刻を知らない GNSS は「時刻が無いので使えない」（MGA-ACK の infoCode=1）と断ることがある
//     （Interface Description §3.13.1。Integration Manual §3.11.4 も、戻すときは時刻を入れるよう勧めている）。
//     断られた分があれば、GNSS が衛星から時刻を読んだところで 1 回だけ全部を入れ直す。時刻は 1 衛星の
//     1 語から読めるので、衛星ごとのエフェメリスよりずっと先にわかる（計画書 §3.4）。
//   - 効くのは、前回の吸い出しから予測軌道の寿命（約 3 日）以内の起動。それより古いとアルマナックだけ。
//   - 外から来た時刻（2026-11-10〜。計画書 §3.6）: GNSS が時刻を知らない間に PC（USB シリアルの PONS:TIME。Flight Viewer が
//     接続のたびに送る）から時刻が来たら、UBX-MGA-INI-TIME_UTC で入れる。断られていた DBD は、受け取られたらすぐ入れ直す。
//     LoRa で仲間からもらう時刻（計画書 §4）も gnss_dbd_give_time() に渡すだけで同じように入る（出どころは GNSS_TIME_SRC_*）。
//     入れる・入れないの判断は time_block() の 1 か所。GNSS が時刻を知っていれば入れない（GNSS の時刻のほうが正確）。
//     机の上の試験（COLD / COLDONLY / HOT）の最中も入れない（比べる素の値を汚さない）。COLD,TIME / COLDONLY,TIME の試験は、
//     リセット前に本体が覚えた時刻（GNSS_TIME_SRC_SELF）を、外から来た時刻の代わりに入れる。
//     精度は GNSS_TIME_ACC_*_MS を控えめに伝える（書いた精度より外れると性能がかなり落ちる）。最初の fix で、入れた時刻が
//     本当はどれだけずれていたかを log.txt に残す（PC の時計があてになるかを確かめるため）。
//
// ■ 守ること
//   - 吸い出しは GNSS の UART（38400bps）を数秒埋め、その間 NAV-PVT が遅れる。**止まっているときだけ**やる
//     （自機の対地速度が READY_STILL_MPS 以下で GNSS_DBD_STILL_MS 続いた。飛行中・走行中はやらない）。
//   - 送るのは少しずつ（UART の送信 FIFO が空いた分だけ。Core0 を待たせない）。送りかけのフレームがあるときに
//     ほかのコマンドを GNSS へ書くと両方壊れるので、書く側は先に gnss_dbd_finish_tx() を呼ぶ。
//   - DBD の中身はファームウェアごとに違い、作った受信機へだけ戻す前提（Interface Description §3.13.4）。
//     ファイルには GNSS チップの固有 ID（SEC-UNIQID）と本体の固有 ID を入れ、違えば戻さない。
//   - SD は Core1 だけ（mysd.cpp の gnss_dbd_save_sd / gnss_dbd_load_sd）。RAM の s_buf は持ち主を状態で決め、
//     Core1 が使っている間（起動時の読み込み・書き込み・試験の読み直し）は Core0 は触らない。
//   - リプレイ中は GNSS の UART を読み捨てるので、吸い出しと戻しは打ち切る。
//
// ■ log.txt（あとで効き目を確かめるため）
//   GNSS DBD file: 52 msgs 7340 bytes, saved 2026-11-09 03:12Z      起動時に SD から読んだ結果（Core1）
//   GNSS ID: ... / GNSS VER: ...                                     GNSS チップの固有 ID と版（ROM SPG 5.10 か）
//   GNSS DBD RESTORE boot: sent 52 ok 40 notime 12 other 0 noack 0 (3.1s after boot)
//   GNSS DBD: GNSS time known 14.2s after boot (file age 30.5h)
//   GNSS DBD RESTORE resend: sent 52 ok 52 ...
//   GNSS ORB (after restore): eph 0/12 alm 0/30 ana 0/25 ...         NAV-ORB のまとめ（使える/期限不明）
//   GNSS FIXED ... 35.2s after boot rtc=lost dbd=40/52 resend=52/52 time=14.2s   （gnss.cpp の TTFF の行）
//   GNSS DBD SAVE (auto): 52 msgs 7340 bytes, dump 2310ms, ANA data 25 SVs
//   GNSS DBD TEST cold+dbd: fix 35.2s after reset ...               机の上の試験（USB シリアル PONS:DBD,...）
//   ── 通信がうまくいっているか（あとで定数を直す材料。2026-11-09〜）──
//   GNSS DBD IO (restore boot|save auto|...): 4.1s, NAV-PVT gap max 620ms, rx peak 812B, ckerr 0, ovf 0, txwait max 0ms, ack avg 38ms max 95ms late 0 odd 0
//       吸い出し・戻しの 1 回ごと。航法の更新が待たされた最大・受信 FIFO のたまり・バイトの欠け・返事の遅れ（io_log の説明）
//   GNSS DBD RESTORE ...: other 2 (code 5x2) ...                     断られた理由の内訳（infoCode 別）
//   GNSS DBD SAVE: due 30 min ago but waiting (moving or no fix)     時間なのに吸い出せない理由（30 分ごと）
//   GNSS ID: no answer to SEC-UNIQID ... / GNSS VER: no answer ...    ポールに返事が無い（このとき DBD は動かない）
//   GNSS DBD file: ... (read 12ms) / GNSS DBD SAVE ...: sd 85ms      Core1 が SD を読み書きした時間
//   ── 外から来た時刻（2026-11-10〜）──
//   GNSS TIME (pc): gave 2026-11-10 03:12:45.12Z acc 2.0s -> accepted (ack 35ms, try 1, 12.3s after boot)
//       入れた時刻と結果（accepted / refused code N / no answer）
//   GNSS TIME (pc): kept for later (before 1st NAV-PVT)              すぐには入れられないので覚えておいた
//   GNSS TIME (pc): NOT_NEEDED                                       PONS:TIME が来たが GNSS は時刻を知っていた（など返事のまま）
//   GNSS DBD: GNSS time given (pc) 12.3s after boot (file age 30.5h) 戻しの入れ直しはここから（衛星から読んだときは "known"）
//   GNSS TIME (pc): given time was +0.08s off at 1st fix (told acc 2.0s)   入れた時刻の本当のずれ（NAV-PVT の遅れのぶん + に出る）
#include "gnss_dbd.h"
#include "settings.h"
#include "gnss.h"
#include "mysd.h"   // enqueueTask・createLogSdfTask・create*DbdTask・taskQueueMutex
#include "imu.h"    // imu_service_if_due()（gnss_dbd_finish_tx の待ちの間）

#if GNSS_DBD_ENABLE
static_assert(GNSS_DBD_BUF_BYTES <= 65535, "link_crc16 は 16 ビットの長さまで");
static uint8_t s_buf[GNSS_DBD_BUF_BYTES];
#else
static uint8_t s_buf[1];
#endif

// ============================================================
//  Core1 とやりとりする状態（taskQueueMutex で守る）
// ============================================================
enum : uint8_t { LD_BUSY = 0, LD_OK, LD_NG };
static volatile uint8_t s_ld = LD_BUSY;   // 起動時は Core1 の setup1() が読むので「読み中」から始める
static GnssDbdHead      s_file;           // 読んだファイルの頭（LD_OK のとき）
static const char*      s_ld_why = "";

enum : uint8_t { SV_IDLE = 0, SV_PENDING, SV_OK, SV_NG };
static volatile uint8_t s_sv = SV_IDLE;
static GnssDbdHead      s_save_head;      // Core0 が用意する（本体 ID と CRC は Core1 が足す）
static const char*      s_sv_why = "";
static uint32_t         s_sv_ms = 0;      // Core1 が SD に書くのにかかった時間 [ms]

static uint8_t load_state() {
  mutex_enter_blocking(&taskQueueMutex);
  const uint8_t v = s_ld;
  mutex_exit(&taskQueueMutex);
  return v;
}

uint8_t* gnss_dbd_buffer_core1() { return s_buf; }

bool gnss_dbd_head_for_save_core1(GnssDbdHead* h) {
  mutex_enter_blocking(&taskQueueMutex);
  const bool ok = (s_sv == SV_PENDING);
  if (ok) *h = s_save_head;
  mutex_exit(&taskQueueMutex);
  return ok;
}

void gnss_dbd_saved_core1(bool ok, const char* why, uint32_t ms) {
  mutex_enter_blocking(&taskQueueMutex);
  s_sv     = ok ? SV_OK : SV_NG;
  s_sv_why = why ? why : "";
  s_sv_ms  = ms;
  mutex_exit(&taskQueueMutex);
}

void gnss_dbd_loaded_core1(bool ok, const GnssDbdHead* h, const char* why) {
  mutex_enter_blocking(&taskQueueMutex);
  if (ok && h) s_file = *h;
  s_ld     = (ok && h) ? LD_OK : LD_NG;
  s_ld_why = why ? why : "";
  mutex_exit(&taskQueueMutex);
}

// UBX の Fletcher-8（class から payload の終わりまで）
static void ubx_ck(const uint8_t* d, uint32_t n, uint8_t& a, uint8_t& b) {
  a = 0; b = 0;
  for (uint32_t i = 0; i < n; i++) { a += d[i]; b += a; }
}

bool gnss_dbd_frames_ok(const uint8_t* b, uint32_t bytes, uint16_t count) {
  uint32_t off = 0;
  uint16_t n = 0;
  while (off < bytes) {
    if (off + 8 > bytes) return false;
    const uint8_t* f = b + off;
    if (f[0] != 0xB5 || f[1] != 0x62 || f[2] != 0x13 || f[3] != 0x80) return false;
    const uint32_t len = (uint32_t)f[4] | ((uint32_t)f[5] << 8);
    if (off + 8 + len > bytes) return false;
    uint8_t ca, cb;
    ubx_ck(f + 2, 4 + len, ca, cb);
    if (f[6 + len] != ca || f[7 + len] != cb) return false;
    off += 8 + len;
    n++;
  }
  return n == count;
}

#if GNSS_DBD_ENABLE

// ============================================================
//  Core0 の状態
// ============================================================
enum DbdState : uint8_t {
  ST_BOOT = 0,   // 起動直後: 最初の NAV-PVT（時計の生死）・SD の読み込み・GNSS の固有 ID を待つ
  ST_SEND,       // 戻している（フレームを 1 通ずつ書き、返事を待つ）
  ST_WAIT,       // 送り終えた。GNSS が時刻を知ったら、断られた分を入れ直す（fix するか窓が終わるまで）
  ST_IDLE,       // 何もしていない（RAM は空き。吸い出しの条件を見る）
  ST_DUMP,       // 吸い出し中（MGA-DBD を受けて RAM にためる）
  ST_SAVING,     // Core1 が SD に書いている
  ST_TEST_LOAD,  // 試験: Core1 が SD から読み直すのを待つ
};
static DbdState s_st = ST_BOOT;
static const char* st_name(DbdState s) {
  switch (s) {
    case ST_BOOT: return "boot"; case ST_SEND: return "send"; case ST_WAIT: return "wait";
    case ST_IDLE: return "idle"; case ST_DUMP: return "dump"; case ST_SAVING: return "saving";
    case ST_TEST_LOAD: return "test-load";
  }
  return "?";
}

// ---- 1 回の「コールドからの立ち上がり」（起動時に時計が死んでいた・試験でリセットした）----
// GNSS が時刻を知った時刻と、試験なら fix までの時間を測る。戻しの入れ直しもこれを見る。
enum EpKind : uint8_t { EP_NONE = 0, EP_BOOT, EP_TEST_DBD, EP_TEST_COLD, EP_TEST_HOT };
static EpKind   s_ep       = EP_NONE;
static uint32_t s_ep_t0    = 0;      // 始めた millis()（起動なら 0）
static bool     s_ep_nofix = false;  // 始めてから「fix していない」を見たか（リセット直前の fix を数えないため）
static uint32_t s_ep_tk_ms = 0;      // GNSS が時刻を知った millis()（0 = まだ）
static float    s_tk_after_s = -1.0f;   // そのときの経過秒（fix のログ用。-1 = まだ）
static uint8_t  s_ep_tk_src = 0xFF;  // その時刻の出どころ（GNSS_TIME_SRC_*。0xFF = GNSS が自分で衛星から読んだ）
static bool     s_ep_time = false;   // 試験 COLD,TIME / COLDONLY,TIME（リセットのあと本体が覚えた時刻を入れる）
static const char* s_ep_end = "";    // 終わった理由（"fix" / "window end" / "no reset"。戻しを途中でやめた行に出す）
static const char* ep_name(EpKind k) {
  switch (k) {
    case EP_BOOT: return "boot";
    case EP_TEST_DBD:  return s_ep_time ? "cold+dbd+time" : "cold+dbd";
    case EP_TEST_COLD: return s_ep_time ? "cold+time" : "cold";
    case EP_TEST_HOT:  return "hot";
    default: return "-";
  }
}

// ---- 戻し ----
struct RsCount { uint16_t sent, ok, notime, other, noack; uint16_t code_n[8]; };   // code_n = other の infoCode 別の数
static RsCount  s_rc[2];               // [0] = 1 回目、[1] = 入れ直し
static uint8_t  s_rs_pass = 0;         // 0 = 戻していない、1 = 1 回目、2 = 入れ直し（済み・不要を含む）
static uint32_t s_rs_off = 0;          // 次に書くフレームの位置
static uint32_t s_rs_bytes = 0;
static bool     s_rs_waiting = false;  // 書いたフレームの返事を待っている
static uint32_t s_rs_ack_ms = 0;
static uint8_t  s_rs_noack_run = 0;    // 続けて返事が来なかった数（3 回で待つのをやめて GNSS_DBD_PACE_MS 間隔にする）
static uint32_t s_rs_frame_off = 0;    // 返事を待っているフレームの位置（返事の照らし合わせ用）
static const char* s_rs_skip = nullptr;   // 戻さなかった理由（fix のログ用）

// ---- 通信がうまくいっているかの記録（吸い出し・戻しの 1 回ごとに "GNSS DBD IO" の 1 行）----
// あとで定数（返事の待ち時間・RAM・吸い出しの条件）を直すための材料。どれも吸い出し・戻しの間だけ数える。
struct IoStat {
  bool     on;
  uint32_t t0;            // 始めた millis()
  uint32_t pvt_last;      // 最後に NAV-PVT を受けた millis()
  uint32_t pvt_gap_max;   // NAV-PVT の間隔の最大 [ms]（ふだん 500。航法がどれだけ待たされたか）
  uint32_t ck0, ovf0;     // 始めたときの UBX のチェックサム誤り・受信 FIFO あふれの数（gnss.cpp）
  uint32_t txwait_max;    // ほかのコマンドが送りかけのフレームを待った最大 [ms]（gnss_dbd_finish_tx）
  uint32_t tx_done_ms;    // いまのフレームを書き終えた millis()（返事の遅れを測る起点。0 = まだ）
  uint32_t ack_sum, ack_max;
  uint16_t ack_n;         // 書き終えてから MGA-ACK が来るまでの遅れ [ms]
  uint16_t late_acks;     // 待つのをやめたあとに来た MGA-ACK（GNSS_DBD_ACK_TIMEOUT_MS が短すぎる印）
  uint16_t odd_acks;      // DBD 以外への MGA-ACK（来ないはず）
};
static IoStat s_io;
static uint32_t s_io_close_at = 0;   // 吸い出しのあと、この時刻まで NAV-PVT の途切れを測ってから 1 行にする（0 = 予定なし）
static char     s_io_what[24];

// ---- 送信（RAM の s_tx_pos..s_tx_end を UART の FIFO が空いた分だけ書く）----
static uint32_t s_tx_pos = 0, s_tx_end = 0;
static inline bool tx_busy() { return s_tx_pos < s_tx_end; }
static void pump_tx() {
  if (s_tx_pos >= s_tx_end) return;
  while (s_tx_pos < s_tx_end && GNSS_SERIAL.availableForWrite() > 0) GNSS_SERIAL.write(s_buf[s_tx_pos++]);
  if (s_tx_pos >= s_tx_end) s_io.tx_done_ms = millis();   // 書き終えた（返事の遅れはここから測る）
}

// ---- GNSS チップの固有 ID ----
static uint8_t  s_gid[6];
static bool     s_gid_ok = false;
static uint32_t s_gid_poll_ms = 0;
static uint8_t  s_gid_polls = 0;

// ---- 吸い出し ----
static uint32_t s_dp_len = 0, s_dp_total = 0, s_dp_t0 = 0, s_dp_utc = 0, s_dp_ms = 0;
static uint16_t s_dp_count = 0;
static bool     s_dp_over = false;
static const char* s_dp_why = "";
static uint32_t s_dp_last_ms = 0;      // 最後に吸い出しを始めた millis()（0 = まだ）

// ---- fix と「止まっている」 ----
static uint32_t s_first_fix_ms = 0;
static uint32_t s_still_since  = 0;
// リプレイを抜けた直後は、位置と fix が再生した CSV の最後の行のまま残る（gnss.cpp は日時だけ無効にする）。
// 本物の NAV-PVT が 1 つ届くまで fix・止まっているか・時刻を見ない。見ると偽の fix で、試験の TTFF や
// 戻しの窓を終わらせてしまう。NAV-PVT はリプレイ中は gnss_dbd_on_ubx() まで来ないので、来たら本物。
static bool     s_after_replay = false;

// ---- 机の上の試験（PONS:DBD,COLD / COLDONLY / HOT）をしたら、再起動するまで自動の吸い出しを止める ----
// COLDONLY で GNSS のデータベースを消したあと、fix して 30 秒止まると、薄い（予測軌道の無い）DBD で
// SD のファイルを上書きしてしまい、次の COLD の比べ物にならなくなるため。手動の PONS:DBD,SAVE は効く。
static bool     s_auto_off = false;

// ---- NAV-ORB のポール（戻した直後に、GNSS が何を持っているかを 1 行残す）----
static const char* s_orb_tag = nullptr;
static uint32_t s_orb_at = 0;
static bool     s_orb_sent = false;

// ---- 版のポール（返事が来なかったことを 1 回だけ残す）----
static bool     s_ver_ok = false;
static uint32_t s_ver_poll_ms = 0;
static bool     s_ver_fail_logged = false;
static bool     s_gid_fail_logged = false;

// ---- 吸い出しを待っている理由（時間が来ているのに始められないとき、GNSS_DBD_SAVE_INTERVAL_MS ごとに残す）----
static uint32_t s_due_since = 0;
static uint32_t s_wait_log_ms = 0;

// ---- 外から来た時刻（gnss_dbd_give_time。PC の PONS:TIME、のちに LoRa、試験の COLD,TIME）----
// 受けた時刻はいちばん新しいものを 1 つ覚えておき（s_gt）、入れてよくなったら time_tick() が入れる。
// 入れたら返事（MGA-ACK、msgId 0x40）を待つ。GNSS がリセットしたら（gnss_dbd_on_setup・試験）数え直す。
struct GivenTime { bool on; uint64_t utc_ms; uint32_t at_ms; uint8_t src; };
static GivenTime s_gt = { false, 0, 0, 0 };
enum : uint8_t { TI_NONE = 0, TI_WAIT, TI_OK, TI_NG, TI_NOACK };
static uint8_t  s_ti = TI_NONE;      // 最後に入れた時刻の返事
static uint8_t  s_ti_tries = 0;      // この GNSS に入れた回数（GNSS_TIME_MAX_TRIES まで）
static uint8_t  s_ti_src = 0;        // 入れた時刻の出どころ
static uint32_t s_ti_ms = 0;         // 入れた millis()
static uint64_t s_ti_utc = 0;        // 入れた時刻 [UTC ms]（0 = 入れていない）。s_ti_ms の瞬間の値
static uint32_t s_ti_acc = 0;        // GNSS に伝えた精度 [ms]
static bool     s_ti_checked = false;   // 最初の fix で、入れた時刻のずれを残したか
static char     s_ti_txt[28];        // 入れた時刻の文字列（ログ用）
static const char* const TB_KNOWN = "gnss has time";   // time_block(): GNSS は時刻を知っている
static const char* const TB_TEST  = "test";            // time_block(): 机の上の試験（素の値）の最中
static const char* const TB_GIVEN = "given";           // time_block(): もう入れて受け取られた
static const char* const TB_GAVE_UP = "gave up";       // time_block(): 入れる回数の上限


// ============================================================
//  小物
// ============================================================
// 1970-01-01 からの日数（rtc_backup.cpp と同じ。Howard Hinnant の days_from_civil）
static int32_t days_from_civil(int y, unsigned m, unsigned d) {
  y -= (m <= 2);
  const int32_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int32_t)doe - 719468;
}

// GNSS が日付まで含めた時刻を知っているか（NAV-PVT の validDate / validTime と年）。
// ★ リプレイ中は ubx_* に CSV の日時が入るので呼ばない（gnss_dbd_tick() はリプレイ中すぐ戻る）。
static bool gnss_time_known() {
  const GnssDate d = get_gnss_date();
  const GnssTime t = get_gnss_time();
  return d.isValid() && t.isValid() && d.year() >= 2026;
}
static uint32_t utc_now() {
  const GnssDate d = get_gnss_date();
  const GnssTime t = get_gnss_time();
  return (uint32_t)days_from_civil(d.year(), d.month(), d.day()) * 86400UL +
         (uint32_t)t.hour() * 3600UL + (uint32_t)t.minute() * 60UL + t.second();
}
// いまの UTC [s]。GNSS が知っていればそれ。まだ NAV-PVT に出ていなくても、外から入れた時刻があればそれを進めたもの。
// どちらも無ければ 0。★ 時刻を入れた直後は NAV-PVT にまだ日時が出ていないことがあり、utc_now() は意味の無い値になる。
static uint32_t utc_best(uint32_t now) {
  if (gnss_time_known()) return utc_now();
  if (s_ti_utc != 0 && s_ti != TI_NG) return (uint32_t)((s_ti_utc + (uint32_t)(now - s_ti_ms)) / 1000ULL);
  return 0;
}
static float file_age_h() {
  const uint32_t u = utc_best(millis());
  return (u > s_file.utc) ? (float)(u - s_file.utc) / 3600.0f : 0.0f;
}
// 1970-01-01 からの日数 → 年月日（days_from_civil の逆。Howard Hinnant の civil_from_days）
static void civil_from_days(int32_t z, int& y, unsigned& m, unsigned& d) {
  z += 719468;
  const int32_t era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = (unsigned)(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp  = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp < 10 ? mp + 3 : mp - 9;
  y = (int)yoe + era * 400 + (m <= 2);
}
// UTC [ms] を "2026-11-10 03:12:45.12Z" にする（ログ用）
static void fmt_utc(char* buf, size_t n, uint64_t t) {
  int y; unsigned mo, d;
  civil_from_days((int32_t)(t / 86400000ULL), y, mo, d);
  const uint32_t ms = (uint32_t)(t % 86400000ULL);
  snprintf(buf, n, "%04d-%02u-%02u %02lu:%02lu:%02lu.%02luZ", y, mo, d, (unsigned long)(ms / 3600000UL),
           (unsigned long)(ms / 60000UL % 60), (unsigned long)(ms / 1000UL % 60), (unsigned long)(ms % 1000UL / 10));
}
static const char* src_name(uint8_t s) {
  switch (s) { case GNSS_TIME_SRC_PC: return "pc"; case GNSS_TIME_SRC_LORA: return "lora"; case GNSS_TIME_SRC_SELF: return "self"; }
  return "?";
}
// GNSS に伝える精度 [ms]（受けた時点の値。入れるまでの本体の時計のずれは give_time_now() で足す）
static uint32_t src_acc_ms(uint8_t s) {
  return (s == GNSS_TIME_SRC_LORA) ? GNSS_TIME_ACC_LORA_MS : GNSS_TIME_ACC_PC_MS;   // SELF は PC の代わりの試験なので PC と同じ
}
// ありえる時刻か（ビルドした日の前日より後で、2100 年より前）。PC の時計が 1970 年や 2000 年に戻っているのを弾く
static bool time_plausible(uint64_t utc_ms) {
  const int32_t d = (int32_t)(utc_ms / 86400000ULL);
  const int32_t d_build = days_from_civil(BUILDDATE / 10000, (BUILDDATE / 100) % 100, BUILDDATE % 100);
  return d >= d_build - 1 && d < days_from_civil(2100, 1, 1);
}
static float since_s(uint32_t now, uint32_t t0) { return (now - t0) / 1000.0f; }

static void io_log(const char* what, uint32_t now);
static void io_begin(uint32_t now) {
  if (s_io.on && s_io_close_at != 0) { io_log(s_io_what, now); }   // 前の吸い出しの行を先に出す
  s_io_close_at = 0;
  memset(&s_io, 0, sizeof(s_io));
  s_io.on       = true;
  s_io.t0       = now;
  s_io.pvt_last = now;
  s_io.ck0      = gnss_ubx_ck_errors();
  s_io.ovf0     = gnss_rx_overflows();
  (void)gnss_rx_backlog_peak_take();   // ここから数え直す
}

// 例: "GNSS DBD IO (restore boot): 4.1s, NAV-PVT gap max 620ms, rx peak 812B, ckerr 0, ovf 0, txwait max 0ms, ack avg 38ms max 95ms late 0 odd 0"
//   NAV-PVT gap max : 航法の更新がいちばん待たされた時間（ふだん 500ms。吸い出しで延びる量）
//   rx peak         : 受信 FIFO（GNSS_SERIAL_FIFO_SIZE）のたまりの最大。4096 に近ければあふれかけ
//   ckerr / ovf     : UBX のチェックサム誤り・FIFO あふれ（0 でなければバイトが欠けている）
//   ack             : 戻すとき、1 通書き終えてから返事までの遅れ。late は待つのをやめたあとに来た返事の数
static void io_log(const char* what, uint32_t now) {
  if (!s_io.on) return;
  s_io.on = false;
  s_io_close_at = 0;
  uint32_t gap = s_io.pvt_gap_max;
  if (now - s_io.pvt_last > gap) gap = now - s_io.pvt_last;   // 終わる時点で途切れていた分
  char ack[80] = "";
  if (s_io.ack_n || s_io.late_acks || s_io.odd_acks)
    snprintf(ack, sizeof(ack), ", ack avg %lums max %lums late %u odd %u",
             (unsigned long)(s_io.ack_n ? s_io.ack_sum / s_io.ack_n : 0), (unsigned long)s_io.ack_max,
             s_io.late_acks, s_io.odd_acks);
  enqueueTask(createLogSdfTask("GNSS DBD IO (%s): %.1fs, NAV-PVT gap max %lums, rx peak %dB, ckerr %lu, ovf %lu, txwait max %lums%s",
                               what, since_s(now, s_io.t0), (unsigned long)gap, gnss_rx_backlog_peak_take(),
                               (unsigned long)(gnss_ubx_ck_errors() - s_io.ck0),
                               (unsigned long)(gnss_rx_overflows() - s_io.ovf0),
                               (unsigned long)s_io.txwait_max, ack));
}

static void orb_request(const char* tag, uint32_t delay_ms) {
  s_orb_tag  = tag;
  s_orb_at   = millis() + delay_ms;
  s_orb_sent = false;
}

static void start_episode(EpKind k, uint32_t now) {
  s_ep = k; s_ep_t0 = now; s_ep_nofix = (k == EP_BOOT); s_ep_tk_ms = 0; s_tk_after_s = -1.0f; s_ep_end = "";
  s_ep_tk_src = 0xFF; s_ep_time = false;   // 試験の ,TIME は呼んだ側が立てる
}
static void end_episode(const char* why) { s_ep = EP_NONE; s_ep_end = why; }

static void start_restore(uint8_t pass) {
  s_rs_pass = pass;
  s_rs_off = 0;
  s_rs_bytes = s_file.bytes;
  s_rs_waiting = false;
  s_rs_noack_run = 0;
  memset(&s_rc[pass - 1], 0, sizeof(RsCount));
  s_st = ST_SEND;
  io_begin(millis());
}

static const char* restore_name() {
  return (s_rs_pass == 2) ? "resend" : ((s_ep == EP_BOOT) ? "boot" : "test");
}

static void restore_done(uint32_t now) {
  const RsCount& c = s_rc[s_rs_pass - 1];
  const char* what = restore_name();
  // other の内訳（infoCode 別。2 版違い・3 大きさ違い・4 しまえない・5 まだ受け取れない・6 知らない種類）
  char codes[48] = "";
  for (int k = 0, n = 0; k < 8; k++) {
    if (c.code_n[k] == 0) continue;
    n += snprintf(codes + n, sizeof(codes) - n, "%s%dx%u", n ? " " : " (code ", k, c.code_n[k]);
    if (n >= (int)sizeof(codes) - 2) break;
  }
  if (codes[0]) strlcat(codes, ")", sizeof(codes));
  enqueueTask(createLogSdfTask("GNSS DBD RESTORE %s: sent %u ok %u notime %u other %u%s noack %u (%.1fs after %s)",
                               what, c.sent, c.ok, c.notime, c.other, codes, c.noack,
                               since_s(now, s_ep_t0), s_ep == EP_BOOT ? "boot" : "reset"));
  char io[24];
  snprintf(io, sizeof(io), "restore %s", what);
  io_log(io, now);
  orb_request(s_rs_pass == 2 ? "after resend" : "after restore", 2000);
  // 入れ直しまで済んだら RAM はもう要らない。ST_WAIT に残すと、屋内のように fix しない場所では
  // 窓（GNSS_DBD_RESTORE_WINDOW_MS = 30 分）が終わるまで PONS:DBD の試験がぜんぶ BUSY になる。
  // TTFF の計測は s_ep（episode_tick）が続けるので、ここで抜けても変わらない。
  s_st = (s_rs_pass == 2) ? ST_IDLE : ST_WAIT;
}

// 吸い出し・戻しの途中で打ち切る（GNSS のポートを開き直す・リプレイに入った）。
// ★ ST_WAIT（入れ直し待ち）と ST_TEST_LOAD（試験の読み込み待ち）は打ち切らない。どちらも GNSS へ
//   書いていない状態で、リプレイを抜けた・掴み直したあとも入れ直し・試験として意味がある。
static void abort_op(const char* why) {
  if (s_st == ST_SEND) {
    // ★ **書きかけのフレームは捨てずに書き切る**（gnss_dbd.h の約束。最大約 45ms・200ms で打ち切り）。
    //   途中で切ると、GNSS は次に届いたコマンドの頭を MGA-DBD の残りとして食い、チェックサムが合わずに
    //   **そのコマンドごと黙って捨てる**（掴み直しなら最初の設定コマンド）。2026-11-10 までは切っていた。
    //   ここはポートを閉じる前（gnss_setup_impl() の頭）とリプレイ中（送信は生きている）なので書ける。
    gnss_dbd_finish_tx();
    enqueueTask(createLogSdfTask("GNSS DBD RESTORE %s: aborted (%s) after %u sent",
                                 restore_name(), why, s_rc[s_rs_pass - 1].sent));
    io_log("restore aborted", millis());
    s_st = ST_IDLE;
  } else if (s_st == ST_DUMP) {
    enqueueTask(createLogSdfTask("GNSS DBD SAVE (%s): aborted (%s) after %u msgs", s_dp_why, why, s_dp_count));
    io_log("save aborted", millis());
    s_st = ST_IDLE;
  }
}

static void skip_restore(const char* why) {
  s_rs_skip = why;
  enqueueTask(createLogSdfTask("GNSS DBD: not restored (rtc lost, %s)", why));
  s_st = ST_IDLE;
}

static void start_dump(const char* why, uint32_t now) {
  s_dp_len = 0; s_dp_total = 0; s_dp_count = 0; s_dp_over = false;
  s_dp_t0 = now; s_dp_utc = utc_now(); s_dp_why = why;
  s_dp_last_ms = now ? now : 1;
  s_due_since = 0;
  s_st = ST_DUMP;
  io_begin(now);
  gnss_ubx_send(0x13, 0x80, nullptr, 0);   // UBX-MGA-DBD のポール（ペイロード無し）
}

static void dump_end(uint32_t n) {
  const uint32_t now = millis();
  s_dp_ms = now - s_dp_t0;
  s_st = ST_IDLE;
  // "GNSS DBD IO" の行は 2 秒後に出す。吸い出しの後ろに並んでいた NAV-PVT が届く（あるいは落ちていた）分まで測るため。
  snprintf(s_io_what, sizeof(s_io_what), "save %s", s_dp_why);
  s_io_close_at = now + 2000;
  if (s_io_close_at == 0) s_io_close_at = 1;
  if (s_dp_over) {
    enqueueTask(createLogSdfTask("GNSS DBD SAVE (%s): too big, %u msgs %lu bytes > GNSS_DBD_BUF_BYTES %u (not saved)",
                                 s_dp_why, s_dp_count, (unsigned long)s_dp_total, (unsigned)GNSS_DBD_BUF_BYTES));
    return;
  }
  if (n != s_dp_count || n == 0) {
    enqueueTask(createLogSdfTask("GNSS DBD SAVE (%s): got %u msgs but GNSS says %lu (not saved)",
                                 s_dp_why, s_dp_count, (unsigned long)n));
    return;
  }
  GnssDbdHead h;
  memset(&h, 0, sizeof(h));
  memcpy(h.magic, GNSS_DBD_MAGIC, 8);
  h.utc   = s_dp_utc;
  h.count = s_dp_count;
  h.bytes = s_dp_len;
  memcpy(h.gnss_id, s_gid, 6);
  mutex_enter_blocking(&taskQueueMutex);
  s_save_head = h;
  s_sv = SV_PENDING;
  mutex_exit(&taskQueueMutex);
  if (enqueueTask(createSaveDbdTask())) {
    s_st = ST_SAVING;
  } else {
    mutex_enter_blocking(&taskQueueMutex);
    s_sv = SV_IDLE;
    mutex_exit(&taskQueueMutex);
    enqueueTask(createLogSdfTask("GNSS DBD SAVE (%s): task queue full (not saved)", s_dp_why));
  }
}

// ============================================================
//  外から来た時刻を GNSS へ入れる（gnss_dbd_give_time が覚え、time_tick が入れる）
// ============================================================
// いま入れてよいか。nullptr = 入れる。TB_* はそれぞれの返事に、それ以外の文字列は「あとで」の理由（ログに出す）。
// ★ 入れる・入れないの判断はここだけ。出どころ（PC・LoRa・試験）ごとに条件を書き足さないこと。
static const char* time_block(uint32_t now) {
  if (s_ep != EP_NONE && s_ep != EP_BOOT) {                  // 机の上の試験
    if (!s_ep_time) return TB_TEST;                          // COLD / COLDONLY / HOT は入れない（比べる素の値）
    // GNSS が立ち上がり直してから。fix が落ちた＝リセットのあとの NAV-PVT が来た（日時もそれで新しくなっている）
    if (!s_ep_nofix || now - s_ep_t0 < 1000) return "test reset";
  }
  if (!gnss_rtc_boot_checked())           return "before 1st NAV-PVT";   // 時計が生きているかまだ分からない
  if (gnss_time_known())                  return TB_KNOWN;
  if (s_ti == TI_OK)                      return TB_GIVEN;
  if (s_ti == TI_WAIT)                    return "waiting answer";
  if (s_ti_tries >= GNSS_TIME_MAX_TRIES)  return TB_GAVE_UP;
  if (s_ti_tries > 0 && now - s_ti_ms < GNSS_TIME_RETRY_MS) return "retry wait";
  if (!get_gnss_connection())             return "no GNSS";
  return nullptr;
}

// GNSS が時刻を知った時刻の文字列（fix のログ用）。"14.2s" / 外から入れたなら "1.2s(pc)" / まだなら "at-fix"
// （NAV-PVT は 0.5 秒ごとで、その間に gnss_dbd_tick() が回るので、fix より前に知っていれば見ている）
static void tk_text(char* b, size_t n) {
  if (s_tk_after_s < 0.0f) { snprintf(b, n, "at-fix"); return; }
  if (s_ep_tk_src == 0xFF) snprintf(b, n, "%.1fs", s_tk_after_s);
  else                     snprintf(b, n, "%.1fs(%s)", s_tk_after_s, src_name(s_ep_tk_src));
}

// 入れた時刻の結果を 1 行にする（ack = 返事が来たか。来なければ待った時間は出さない）
static void ti_log(const char* res, uint32_t now, bool ack) {
  const bool test = (s_ep != EP_NONE && s_ep != EP_BOOT);
  char a[20] = "";
  if (ack) snprintf(a, sizeof(a), "ack %lums, ", (unsigned long)(now - s_ti_ms));
  enqueueTask(createLogSdfTask("GNSS TIME (%s): gave %s acc %.1fs -> %s (%stry %u, %.1fs after %s)",
                               src_name(s_ti_src), s_ti_txt, s_ti_acc / 1000.0f, res, a, s_ti_tries,
                               test ? since_s(s_ti_ms, s_ep_t0) : s_ti_ms / 1000.0f, test ? "reset" : "boot"));
}

// 覚えている時刻（s_gt）を、いまの時刻まで進めて UBX-MGA-INI-TIME_UTC で入れる（Interface Description §3.13.9.3）。
static void give_time_now() {
  gnss_dbd_finish_tx();   // DBD のフレームを書きかけていたら先に書き切る（待ったぶん時刻が進むので、時刻はこのあと読む）
  const uint32_t now = millis();
  const uint32_t el  = now - s_gt.at_ms;
  const uint64_t t_now = s_gt.utc_ms + el;
  // GNSS は受け取り終えた瞬間の時刻として使う（ref.source=0）。32 バイトを 38400bps で書き終えるまで約 8ms。
  const uint64_t t = t_now + 8;
  const uint32_t acc = src_acc_ms(s_gt.src) + (uint32_t)((uint64_t)el * GNSS_TIME_DRIFT_PPM / 1000000ULL);
  int y; unsigned mo, d;
  civil_from_days((int32_t)(t / 86400000ULL), y, mo, d);
  const uint32_t ms  = (uint32_t)(t % 86400000ULL);
  const uint32_t ns  = (ms % 1000UL) * 1000000UL;
  const uint16_t acs = (uint16_t)(acc / 1000UL);
  const uint32_t acn = (acc % 1000UL) * 1000000UL;
  uint8_t p[24];
  memset(p, 0, sizeof(p));
  p[0]  = 0x10;                 // type = TIME_UTC
  p[1]  = 0x00;                 // version
  p[2]  = 0x00;                 // ref.source = 0（受け取った瞬間。EXTINT のパルスは使わない）
  p[3]  = 0x80;                 // leapSecs = -128（知らない。GNSS が持っている値を使う）
  p[4]  = (uint8_t)(y & 0xFF);  p[5] = (uint8_t)(y >> 8);
  p[6]  = (uint8_t)mo;          p[7] = (uint8_t)d;
  p[8]  = (uint8_t)(ms / 3600000UL);
  p[9]  = (uint8_t)(ms / 60000UL % 60);
  p[10] = (uint8_t)(ms / 1000UL % 60);
  p[11] = 0x00;                 // bitfield0.trustedSource = 0（分からない）
  p[12] = (uint8_t)ns;  p[13] = (uint8_t)(ns >> 8);  p[14] = (uint8_t)(ns >> 16);  p[15] = (uint8_t)(ns >> 24);
  p[16] = (uint8_t)acs; p[17] = (uint8_t)(acs >> 8);                                   // tAccS（p[18..19] は予約）
  p[20] = (uint8_t)acn; p[21] = (uint8_t)(acn >> 8); p[22] = (uint8_t)(acn >> 16); p[23] = (uint8_t)(acn >> 24);
  gnss_ubx_send(0x13, 0x40, p, sizeof(p));
  s_ti         = TI_WAIT;
  s_ti_ms      = now ? now : 1;
  s_ti_utc     = t_now;
  s_ti_acc     = acc;
  s_ti_src     = s_gt.src;
  s_ti_checked = false;
  if (s_ti_tries < 255) s_ti_tries++;
  fmt_utc(s_ti_txt, sizeof(s_ti_txt), t_now);
}

// UBX-MGA-INI-TIME_UTC への返事（MGA-ACK の msgPayloadStart の先頭 = type 0x10）。遅れて来たものは捨てる。
static void on_ini_ack(const uint8_t* p) {
  if (p[4] != 0x10 || s_ti != TI_WAIT) return;
  const uint32_t now = millis();
  if (p[0] == 1) { s_ti = TI_OK; ti_log("accepted", now, true); return; }
  s_ti = TI_NG;
  char r[24];
  snprintf(r, sizeof(r), "refused code %u", p[2]);
  ti_log(r, now, true);
}

// 毎回（gnss_dbd_tick から）。返事の待ちを切り、入れた時刻のずれを残し、入れてよくなったら入れる。
static void time_tick(uint32_t now, bool fix) {
  if (s_ti == TI_WAIT && now - s_ti_ms > GNSS_DBD_ACK_TIMEOUT_MS) { s_ti = TI_NOACK; ti_log("no answer", now, false); }
  // 最初の fix で、入れた時刻が本当はどれだけずれていたか（fix した NAV-PVT の時刻は衛星から解いたもの）。
  // NAV-PVT は測位の瞬間から数十 ms 遅れて届くので、ずれはそのぶん + に出る。精度より大きければ印を付ける。
  if (fix && s_ti_utc != 0 && !s_ti_checked && s_ti != TI_NG && gnss_time_known()) {
    s_ti_checked = true;
    const int64_t g    = (int64_t)utc_now() * 1000 + get_gnss_time().centisecond() * 10;
    const int64_t mine = (int64_t)s_ti_utc + (int32_t)(get_gnss_fix_millis() - s_ti_ms);
    const float   off  = (float)(mine - g) / 1000.0f;
    enqueueTask(createLogSdfTask("GNSS TIME (%s): given time was %+.2fs off at 1st fix (told acc %.1fs)%s",
                                 src_name(s_ti_src), off, s_ti_acc / 1000.0f,
                                 fabsf(off) * 1000.0f > (float)s_ti_acc ? " ** more than acc **" : ""));
  }
  if (s_gt.on && time_block(now) == nullptr) give_time_now();
}

// ============================================================
//  受けた UBX
// ============================================================
static void on_dbd_frame(const uint8_t* p, uint16_t len) {
  if (s_st != ST_DUMP) return;
  const uint32_t flen = 8u + len;
  s_dp_total += flen;
  if (s_dp_count < 0xFFFF) s_dp_count++;
  if (s_dp_over || s_dp_len + flen > GNSS_DBD_BUF_BYTES) { s_dp_over = true; return; }
  uint8_t* d = s_buf + s_dp_len;
  d[0] = 0xB5; d[1] = 0x62; d[2] = 0x13; d[3] = 0x80;
  d[4] = (uint8_t)(len & 0xFF); d[5] = (uint8_t)(len >> 8);
  memcpy(d + 6, p, len);
  uint8_t ca, cb;
  ubx_ck(d + 2, 4u + len, ca, cb);
  d[6 + len] = ca; d[7 + len] = cb;
  s_dp_len += flen;
}

static void on_mga_ack(const uint8_t* p) {
  if (p[3] == 0x40) { on_ini_ack(p); return; }   // MGA-INI（外から入れた時刻）への返事
  if (p[3] != 0x80) { if (s_io.on && s_io.odd_acks < 0xFFFF) s_io.odd_acks++; return; }   // MGA-DBD についての返事だけ
  if (s_st == ST_DUMP) {      // 吸い出しの終わり。msgPayloadStart に吐き出した数（U4）
    dump_end((uint32_t)p[4] | ((uint32_t)p[5] << 8) | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24));
    return;
  }
  // 返事の msgPayloadStart（p[4..7]）は、返事の元のフレームのペイロードの先頭 4 バイト。いま待っているフレームと
  // 違えば、待つのをやめた前のフレームへの遅れた返事なので、いまのフレームの返事として数えない
  // （中身が同じフレームどうしは見分けられないが、そのときは数え方が 1 つずれるだけ）。
  const bool mine = s_st == ST_SEND && s_rs_waiting && s_rs_frame_off + 10 <= s_rs_bytes &&
                    memcmp(p + 4, s_buf + s_rs_frame_off + 6, 4) == 0;
  if (mine) {
    RsCount& c = s_rc[s_rs_pass - 1];
    if (p[0] == 1)       c.ok++;         // 使った
    else if (p[2] == 1)  c.notime++;     // 時刻を知らないので使えない
    else { c.other++; c.code_n[p[2] < 8 ? p[2] : 7]++; }
    s_rs_waiting = false;
    s_rs_noack_run = 0;
    if (s_io.tx_done_ms != 0) {          // 書き終えてから返事までの遅れ
      const uint32_t lat = millis() - s_io.tx_done_ms;
      s_io.ack_sum += lat;
      if (lat > s_io.ack_max) s_io.ack_max = lat;
      if (s_io.ack_n < 0xFFFF) s_io.ack_n++;
    }
  } else if (s_io.on && s_io.late_acks < 0xFFFF) {
    s_io.late_acks++;                    // 待つのをやめたあとに来た（noack に数えたフレームの返事）
  }
}

static void on_uniqid(const uint8_t* p) {
  memcpy(s_gid, p + 4, 6);
  if (!s_gid_ok) {
    s_gid_ok = true;
    enqueueTask(createLogSdfTask("GNSS ID: %02X%02X%02X%02X%02X%02X (%.1fs after boot, poll %u)",
                                 s_gid[0], s_gid[1], s_gid[2], s_gid[3], s_gid[4], s_gid[5],
                                 millis() / 1000.0f, s_gid_polls));
  }
}

// MON-VER: 版を 1 回だけ残す（ROM SPG 5.10 かの確認。拡張の文字列のうち FWVER / PROTVER / MOD）
static void on_monver(const uint8_t* p, uint16_t len) {
  s_ver_ok = true;
  static bool logged = false;
  if (logged) return;
  logged = true;
  char sw[31], hw[11], ext[3][31];
  memcpy(sw, p, 30); sw[30] = 0;
  memcpy(hw, p + 30, 10); hw[10] = 0;
  for (int k = 0; k < 3; k++) ext[k][0] = 0;
  static const char* const keys[3] = { "FWVER=", "PROTVER=", "MOD=" };
  for (uint16_t off = 40; off + 30 <= len; off += 30) {
    char s[31];
    memcpy(s, p + off, 30); s[30] = 0;
    for (int k = 0; k < 3; k++) if (strncmp(s, keys[k], strlen(keys[k])) == 0) strlcpy(ext[k], s, sizeof(ext[k]));
  }
  enqueueTask(createLogSdfTask("GNSS VER: %s (hw %s) %s %s %s", sw, hw, ext[0], ext[1], ext[2]));
}

// NAV-ORB: 衛星ごとのエフェメリス・アルマナック・予測軌道（ANA）が「使えるか」を数えて 1 行にする。
//   使える = 残り期間が 1 以上（期限が分かっている）。期限不明 = 31（時刻を知らないときなど）。
static void on_orb(const uint8_t* p, uint16_t len) {
  if (!s_orb_tag || !s_orb_sent) return;
  uint16_t n = p[5];
  if (8u + n * 6u > len) n = (len - 8) / 6;
  uint16_t eu = 0, ex = 0, au = 0, ax = 0, nu = 0, nx = 0, nmax = 0;
  for (uint16_t i = 0; i < n; i++) {
    const uint8_t* s = p + 8 + i * 6;
    const uint8_t eph = s[3], alm = s[4], oth = s[5];
    if (eph >> 5) { const uint8_t u = eph & 0x1F; if (u == 31) ex++; else if (u > 0) eu++; }
    if (alm >> 5) { const uint8_t u = alm & 0x1F; if (u == 31) ax++; else if (u > 0) au++; }
    if ((oth >> 5) == 2) {
      const uint8_t u = oth & 0x1F;
      if (u == 31) nx++;
      else if (u > 0) { nu++; if (u < 30 && u > nmax) nmax = u; }
    }
  }
  enqueueTask(createLogSdfTask("GNSS ORB (%s): eph %u/%u alm %u/%u ana %u/%u (usable/age unknown, ana up to %ud) of %u SVs",
                               s_orb_tag, eu, ex, au, ax, nu, nx, nmax, n));
  s_orb_tag = nullptr;
}

#endif // GNSS_DBD_ENABLE

void gnss_dbd_on_ubx(uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len) {
#if GNSS_DBD_ENABLE
  if (cls == 0x01 && id == 0x07) {                  // NAV-PVT の間隔（吸い出し・戻しが航法をどれだけ待たせたか）
    s_after_replay = false;                         // 本物の NAV-PVT が来た（リプレイ中はここまで来ない）
    if (s_io.on) {
      const uint32_t now = millis();
      if (now - s_io.pvt_last > s_io.pvt_gap_max) s_io.pvt_gap_max = now - s_io.pvt_last;
      s_io.pvt_last = now;
    }
    return;
  }
  if (cls == 0x13 && id == 0x80)                    on_dbd_frame(p, len);
  else if (cls == 0x13 && id == 0x60 && len >= 8)   on_mga_ack(p);
  else if (cls == 0x27 && id == 0x03 && len >= 10)  on_uniqid(p);
  else if (cls == 0x0A && id == 0x04 && len >= 40)  on_monver(p, len);
  else if (cls == 0x01 && id == 0x34 && len >= 8)   on_orb(p, len);
#else
  (void)cls; (void)id; (void)p; (void)len;
#endif
}

void gnss_dbd_finish_tx() {
#if GNSS_DBD_ENABLE
  if (s_tx_pos >= s_tx_end) return;
  const uint32_t t0 = millis();
  while (s_tx_pos < s_tx_end) {
    if (GNSS_SERIAL.availableForWrite() > 0) {
      GNSS_SERIAL.write(s_buf[s_tx_pos++]);
    } else if (millis() - t0 > 200) {
      // ふだんは 1 通（最大 172 バイト）で約 45ms。UART が止まっていたら残りを捨てて Core0 を返す
      // （GNSS には壊れたフレームとして捨てられ、そのフレームは noack に数えられる）。
      enqueueTask(createLogSdfTask("GNSS DBD: UART stuck, dropped %lu bytes", (unsigned long)(s_tx_end - s_tx_pos)));
      s_tx_end = s_tx_pos;
    } else {
      imu_service_if_due();
    }
  }
  const uint32_t now = millis();
  s_io.tx_done_ms = now;
  if (s_io.on && now - t0 > s_io.txwait_max) s_io.txwait_max = now - t0;
#endif
}

void gnss_dbd_on_setup() {
#if GNSS_DBD_ENABLE
  abort_op("GNSS setup");
  // GNSS を掴み直す（GNSS だけがリセットした・ボーレートを替えた）。入れた時刻を失っているかもしれないので、
  // 覚えている時刻（s_gt）をまた入れられるように数え直す（GNSS が時刻を知っていれば time_block() が止める）。
  s_ti = TI_NONE;
  s_ti_tries = 0;
#endif
}

void gnss_dbd_after_setup() {
#if GNSS_DBD_ENABLE
  gnss_ubx_send(0x27, 0x03, nullptr, 0);   // UBX-SEC-UNIQID のポール
  gnss_ubx_send(0x0A, 0x04, nullptr, 0);   // UBX-MON-VER のポール
  s_ver_poll_ms = millis();
  if (s_ver_poll_ms == 0) s_ver_poll_ms = 1;
  s_gid_poll_ms = millis();
  if (s_gid_poll_ms == 0) s_gid_poll_ms = 1;
  s_gid_polls = 1;
#endif
}

void gnss_dbd_fix_tag(char* buf, size_t n) {
  const char* rtc = !gnss_rtc_boot_checked() ? "?" : (gnss_rtc_alive_on_boot() ? "alive" : "lost");
#if GNSS_DBD_ENABLE
  if (!gnss_rtc_boot_checked() || gnss_rtc_alive_on_boot()) { snprintf(buf, n, "rtc=%s", rtc); return; }
  if (s_rs_pass == 0) { snprintf(buf, n, "rtc=lost dbd=none(%s)", s_rs_skip ? s_rs_skip : "not yet"); return; }
  char rs[24] = "";
  if (s_rs_pass == 2 && s_rc[1].sent > 0) snprintf(rs, sizeof(rs), " resend=%u/%u", s_rc[1].ok, s_rc[1].sent);
  char tk[24];
  tk_text(tk, sizeof(tk));   // GNSS が時刻を知った時刻（外から入れたなら出どころ付き）
  snprintf(buf, n, "rtc=lost dbd=%u/%u%s time=%s", s_rc[0].ok, s_rc[0].sent, rs, tk);
#else
  snprintf(buf, n, "rtc=%s", rtc);
#endif
}

// ============================================================
//  見回り（gnss_loop() から毎回）
// ============================================================
#if GNSS_DBD_ENABLE
static void episode_tick(uint32_t now, bool fix) {
  if (s_ep == EP_NONE) return;
  if (!s_ep_nofix) {   // リセットが効いて fix が落ちたのを見てから数える（直前の fix を数えない）
    if (!fix) {
      s_ep_nofix = true;
    } else {
      if (now - s_ep_t0 > 5000) {
        enqueueTask(createLogSdfTask("GNSS DBD TEST %s: fix did not drop in 5s (reset not applied?)", ep_name(s_ep)));
        end_episode("no reset");
      }
      return;
    }
  }
  // GNSS が時刻を知った: 外から入れた時刻が受け取られた（NAV-PVT に出る前でも）か、NAV-PVT に日時が出た。
  // 入れ直し（wait_tick）はここから。入れた時刻の返事が来なくても、入れたあとに NAV-PVT に出たなら入れた時刻のおかげ。
  if (s_ep_tk_ms == 0 && (s_ti == TI_OK || gnss_time_known())) {
    s_ep_tk_ms   = now ? now : 1;
    s_tk_after_s = since_s(now, s_ep_t0);
    s_ep_tk_src  = (s_ti_utc != 0 && s_ti != TI_NG) ? s_ti_src : 0xFF;
    char age[32] = "";
    if (load_state() == LD_OK) snprintf(age, sizeof(age), " (file age %.1fh)", file_age_h());
    char how[16] = "known";
    if (s_ep_tk_src != 0xFF) snprintf(how, sizeof(how), "given (%s)", src_name(s_ep_tk_src));
    enqueueTask(createLogSdfTask("GNSS DBD: GNSS time %s %.1fs after %s%s",
                                 how, s_tk_after_s, s_ep == EP_BOOT ? "boot" : "reset", age));
  }
  if (fix) {
    if (s_ep != EP_BOOT) {
      char tk[24];
      tk_text(tk, sizeof(tk));
      char rs[40] = "";
      if (s_ep == EP_TEST_DBD && s_rs_pass > 0)
        snprintf(rs, sizeof(rs), " dbd=%u/%u resend=%u/%u", s_rc[0].ok, s_rc[0].sent, s_rc[1].ok, s_rc[1].sent);
      enqueueTask(createLogSdfTask("GNSS DBD TEST %s: fix %.1fs after reset (sats %d, time %s%s)",
                                   ep_name(s_ep), since_s(now, s_ep_t0), get_gsa_numsat(), tk, rs));
    }
    end_episode("fix");
    return;
  }
  if (now - s_ep_t0 >= GNSS_DBD_RESTORE_WINDOW_MS) {
    enqueueTask(createLogSdfTask("GNSS DBD %s: no fix %lu min after %s", ep_name(s_ep),
                                 (unsigned long)((now - s_ep_t0) / 60000UL), s_ep == EP_BOOT ? "boot" : "reset"));
    end_episode("window end");
  }
}

static void send_tick(uint32_t now) {
  if (tx_busy()) return;                       // まだ書いている
  if (s_ep == EP_NONE) {                       // fix した（窓が終わった・試験のリセットが効かなかった）。残りは送らない
    enqueueTask(createLogSdfTask("GNSS DBD RESTORE %s: stopped at %s (%u of %u sent)", restore_name(),
                                 s_ep_end, s_rc[s_rs_pass - 1].sent, s_file.count));
    io_log("restore stopped", now);
    s_st = ST_IDLE;
    return;
  }
  if (s_rs_waiting) {
    const uint32_t lim = (s_rs_noack_run >= 3) ? GNSS_DBD_PACE_MS : GNSS_DBD_ACK_TIMEOUT_MS;
    if (now - s_rs_ack_ms < lim) return;
    s_rc[s_rs_pass - 1].noack++;
    if (s_rs_noack_run < 255) s_rs_noack_run++;
    s_rs_waiting = false;
  }
  if (s_rs_off >= s_rs_bytes) { restore_done(now); return; }
  const uint8_t* f = s_buf + s_rs_off;
  const uint32_t flen = 8u + ((uint32_t)f[4] | ((uint32_t)f[5] << 8));
  if (s_rs_off + 8 > s_rs_bytes || f[0] != 0xB5 || f[1] != 0x62 || s_rs_off + flen > s_rs_bytes) {
    enqueueTask(createLogSdfTask("GNSS DBD RESTORE: broken frame at %lu", (unsigned long)s_rs_off));   // 読み込みで確かめてあるので来ないはず
    restore_done(now);
    return;
  }
  s_tx_pos = s_rs_off;
  s_tx_end = s_rs_off + flen;
  s_rs_frame_off = s_rs_off;
  s_rs_off += flen;
  s_rc[s_rs_pass - 1].sent++;
  s_rs_waiting = true;
  s_rs_ack_ms  = now;
  pump_tx();
}

static void wait_tick() {
  if (s_ep == EP_NONE) { s_st = ST_IDLE; return; }   // fix した・窓が終わった
  if (s_rs_pass != 1 || s_ep_tk_ms == 0) return;
  const RsCount& c = s_rc[0];
  // 入れ直しが要らないと決まったら抜ける（理由は restore_done()）
  if (c.ok >= c.sent) { s_rs_pass = 2; s_st = ST_IDLE; return; }   // 全部使われていた。入れ直しは要らない
  const float age = file_age_h();
  if (age > GNSS_DBD_MAX_AGE_H) {
    enqueueTask(createLogSdfTask("GNSS DBD: not resent (file %.0fh old > %d)", age, (int)GNSS_DBD_MAX_AGE_H));
    s_rs_pass = 2;
    s_st = ST_IDLE;
    return;
  }
  start_restore(2);
}

// 吸い出してよいか。nullptr = よい、DUMP_NOT_DUE = まだその時間ではない、それ以外 = 時間は来ているが足りないもの。
static const char* const DUMP_NOT_DUE = "not due";
static const char* dump_block(uint32_t now) {
  if (s_auto_off) return DUMP_NOT_DUE;                            // 机の上の試験をした（再起動まで。上の s_auto_off）
  if (s_ep != EP_NONE) return DUMP_NOT_DUE;                       // まだ最初の fix の前（戻している最中を含む）
  if (s_first_fix_ms == 0 || now - s_first_fix_ms < GNSS_DBD_FIRST_SAVE_MS) return DUMP_NOT_DUE;
  if (s_dp_last_ms != 0 && now - s_dp_last_ms < GNSS_DBD_SAVE_INTERVAL_MS) return DUMP_NOT_DUE;
  if (tx_busy())                    return "sending";
  if (s_still_since == 0)           return "moving or no fix";
  if (now - s_still_since < GNSS_DBD_STILL_MS) return "just stopped";
  if (get_gnss_ana_busy())          return "ANA busy";
  if (!s_gid_ok)                    return "no GNSS id";
  if (!gnss_time_known())           return "no time";
  if (load_state() == LD_BUSY)      return "SD read";             // 起動時の読み込みが RAM を使い終えていない
  return nullptr;
}

static void saving_tick() {
  mutex_enter_blocking(&taskQueueMutex);
  const uint8_t sv = s_sv;
  const char* why = s_sv_why;
  if (sv == SV_OK || sv == SV_NG) s_sv = SV_IDLE;
  mutex_exit(&taskQueueMutex);
  if (sv == SV_OK) {
    // SD のファイルが替わったので、頭を覚え直す（STATUS と試験の "file age" が新しいほうを指すように）。
    // s_buf の中身も書いたフレームのままなので、LD_OK の「RAM にファイルの中身がある」とも食い違わない。
    mutex_enter_blocking(&taskQueueMutex);
    s_file   = s_save_head;
    s_ld     = LD_OK;
    s_ld_why = "";
    mutex_exit(&taskQueueMutex);
    enqueueTask(createLogSdfTask("GNSS DBD SAVE (%s): %u msgs %lu bytes, dump %lums, sd %lums, ANA data %d SVs",
                                 s_dp_why, s_dp_count, (unsigned long)s_dp_len, (unsigned long)s_dp_ms,
                                 (unsigned long)s_sv_ms, get_gnss_ana_data_svs()));
    s_st = ST_IDLE;
  } else if (sv == SV_NG) {
    enqueueTask(createLogSdfTask("GNSS DBD SAVE (%s): not written (%s)", s_dp_why, why));
    s_st = ST_IDLE;
  }
}

static bool file_matches_gnss(const char*& why) {
  if (!s_gid_ok)                                { why = "no GNSS id"; return false; }
  if (memcmp(s_file.gnss_id, s_gid, 6) != 0)   { why = "file from another GNSS"; return false; }
  return true;
}
#endif // GNSS_DBD_ENABLE

void gnss_dbd_tick() {
#if GNSS_DBD_ENABLE
  const uint32_t now = millis();
  if (getReplayMode()) {   // GNSS の UART を読み捨てているので返事が来ない
    abort_op("replay");
    if (s_io.on && s_io_close_at != 0) io_log(s_io_what, now);   // 吸い出しの後ろの行は待たずに出す（再生中は NAV-PVT を見ない）
    s_still_since  = 0;
    s_after_replay = true;
    return;
  }
  pump_tx();
  if (s_io.on && s_io_close_at != 0 && (int32_t)(now - s_io_close_at) >= 0) io_log(s_io_what, now);

  // GNSS の固有 ID が来なければポールし直す（5 秒ごと、合わせて 4 回まで）
  if (!s_gid_ok && s_gid_poll_ms != 0 && s_gid_polls < 4 && now - s_gid_poll_ms >= 5000) {
    gnss_ubx_send(0x27, 0x03, nullptr, 0);
    s_gid_poll_ms = now ? now : 1;
    s_gid_polls++;
  }
  if (!s_gid_ok && !s_gid_fail_logged && s_gid_polls >= 4 && now - s_gid_poll_ms >= 5000) {
    s_gid_fail_logged = true;
    enqueueTask(createLogSdfTask("GNSS ID: no answer to SEC-UNIQID after %u polls (DBD neither restored nor saved)", s_gid_polls));
  }
  if (!s_ver_ok && !s_ver_fail_logged && s_ver_poll_ms != 0 && now - s_ver_poll_ms >= 10000) {
    s_ver_fail_logged = true;
    enqueueTask(createLogSdfTask("GNSS VER: no answer to MON-VER"));
  }
  // NAV-ORB のポール
  if (s_orb_tag) {
    if (!s_orb_sent && (int32_t)(now - s_orb_at) >= 0 && !tx_busy()) {
      gnss_ubx_send(0x01, 0x34, nullptr, 0);
      s_orb_sent = true;
      s_orb_at   = now;
    } else if (s_orb_sent && now - s_orb_at > 3000) {
      enqueueTask(createLogSdfTask("GNSS ORB (%s): no answer", s_orb_tag));
      s_orb_tag = nullptr;
    }
  }

  if (s_after_replay) {   // リプレイを抜けた直後。本物の NAV-PVT を待つ（上の s_after_replay）
    if (s_st == ST_SAVING) saving_tick();
    return;
  }

  double lat, lon, gs, trk;
  const bool fix = gnss_get_own_fix(lat, lon, gs, trk);
  if (fix && s_first_fix_ms == 0) s_first_fix_ms = now ? now : 1;
  if (fix && gs <= READY_STILL_MPS) { if (s_still_since == 0) s_still_since = now ? now : 1; }
  else s_still_since = 0;

  time_tick(now, fix);      // 外から来た時刻を入れる（戻し・入れ直しより先。GNSS は届いた順に読む）
  episode_tick(now, fix);

  switch (s_st) {
    case ST_BOOT: {
      if (!gnss_rtc_boot_checked()) return;              // 最初の NAV-PVT を待つ
      if (gnss_rtc_alive_on_boot()) {                    // BT1 が生きていた。BBR のほうが新しい
        enqueueTask(createLogSdfTask("GNSS DBD: not restored (rtc alive at boot)"));
        s_st = ST_IDLE;
        return;
      }
      static bool ep_started = false;
      if (!ep_started) { ep_started = true; start_episode(EP_BOOT, 0); return; }
      if (s_ep == EP_NONE) { skip_restore("fixed before restore"); return; }   // 待っている間に fix した
      const uint8_t ld = load_state();
      const bool waiting = now < GNSS_DBD_LOAD_WAIT_MS;
      if (ld == LD_BUSY && waiting) return;              // SD の読み込みを待つ
      if (ld != LD_OK) { skip_restore(ld == LD_BUSY ? "SD read timeout" : s_ld_why); return; }
      if (!s_gid_ok && waiting) return;                  // GNSS の固有 ID を待つ
      const char* why = nullptr;
      if (!file_matches_gnss(why)) { skip_restore(why); return; }
      start_restore(1);
      return;
    }
    case ST_SEND:   send_tick(now); return;
    case ST_WAIT:   wait_tick(); return;
    case ST_IDLE: {
      const char* why = dump_block(now);
      if (why == nullptr) { start_dump("auto", now); return; }
      if (why == DUMP_NOT_DUE) { s_due_since = 0; return; }
      // 時間は来ているのに始められない。GNSS_DBD_SAVE_INTERVAL_MS ごとに理由を残す（動き続けるボートなど）
      if (s_due_since == 0) { s_due_since = now ? now : 1; s_wait_log_ms = s_due_since; }
      if (now - s_wait_log_ms >= GNSS_DBD_SAVE_INTERVAL_MS) {
        s_wait_log_ms = now;
        enqueueTask(createLogSdfTask("GNSS DBD SAVE: due %lu min ago but waiting (%s)",
                                     (unsigned long)((now - s_due_since) / 60000UL), why));
      }
      return;
    }
    case ST_DUMP:
      if (now - s_dp_t0 > GNSS_DBD_DUMP_TIMEOUT_MS) {
        enqueueTask(createLogSdfTask("GNSS DBD SAVE (%s): no end mark in %lus (%u msgs, not saved)",
                                     s_dp_why, (unsigned long)(GNSS_DBD_DUMP_TIMEOUT_MS / 1000), s_dp_count));
        io_log("save timeout", now);
        s_st = ST_IDLE;
      }
      return;
    case ST_SAVING: saving_tick(); return;
    case ST_TEST_LOAD: {
      if (s_ep == EP_NONE) {                             // リセットが効かなかった（fix が 5 秒落ちなかった）
        enqueueTask(createLogSdfTask("GNSS DBD TEST: not restored (%s)", s_ep_end));
        s_st = ST_IDLE;                                  // Core1 がまだ読んでいても、吸い出しと試験は LD_BUSY を見て待つ
        return;
      }
      // GNSS が立ち上がり直す（fix が落ちる）のを待つ。立ち上がる前に書いた分は捨てられるため。
      if (now - s_ep_t0 < 1000 || !s_ep_nofix) return;
      // COLD,TIME は時刻を先に入れる（Integration Manual §3.11.4 の勧め。GNSS は届いた順に読むので返事は待たない）。
      // 入れられないまま 5 秒経ったら、時刻なしで戻す（何が起きたかは "GNSS TIME" の行と、試験の行の time= に出る）。
      if (s_ep_time && s_gt.on && s_ti == TI_NONE && now - s_ep_t0 < 5000) return;
      const uint8_t ld = load_state();
      if (ld == LD_BUSY) {
        if (now - s_ep_t0 > GNSS_DBD_LOAD_WAIT_MS) { enqueueTask(createLogSdfTask("GNSS DBD TEST: SD read timeout")); s_st = ST_IDLE; }
        return;
      }
      const char* why = (ld != LD_OK) ? s_ld_why : nullptr;
      if (!why) file_matches_gnss(why);
      if (why) {
        s_rs_skip = why;
        enqueueTask(createLogSdfTask("GNSS DBD TEST: not restored (%s)", why));
        s_st = ST_IDLE;
        return;
      }
      start_restore(1);
      return;
    }
  }
#endif
}

// ============================================================
//  机の上の試験（USB シリアルの PONS:DBD,<arg>。GPS_TFT_map.ino の handle_serial_command）
// ============================================================
//   SAVE      いま吸い出して SD に書く（fix していれば、時間・止まっているかの条件は見ない）
//   COLD      GNSS をコールドスタートさせ（CFG-RST navBbrMask=0xFFFF。BT1 が切れたのと同じ）、SD の DBD を戻す
//   COLDONLY  コールドスタートだけ（比べるための素の値）
//   HOT       ホットスタート（navBbrMask=0x0000）
//   COLD,TIME / COLDONLY,TIME
//             COLD / COLDONLY と同じだが、リセットのあと（COLD は DBD より先に）、リセット前に本体が覚えた時刻を
//             PC・LoRa から来た時刻の代わりに入れる（精度は GNSS_TIME_ACC_PC_MS と伝える）。計画書 §7 の
//             「DBD ＋ INI」「INI だけ」。GNSS が時刻を知っていないと ERROR,NO_TIME
//   STATUS    状態を USB シリアルに出す
//   結果は log.txt の "GNSS DBD TEST <種類>: fix 35.2s after reset ..." と "GNSS DBD: GNSS time known|given ..."。
//   リセットは GNSS だけの再起動（resetMode=0x02）なので、設定（38400bps・UBX）は消えない。
//   COLD / COLDONLY / HOT を 1 回でも打つと、再起動するまで自動の吸い出しはしない（s_auto_off。SAVE は効く）。
//   素の試験（,TIME の無いもの）の間は、PC・LoRa から時刻が来ても入れない（PONS:TIME は ERROR,TEST）。
//   リプレイ中と抜けた直後（本物の NAV-PVT が来るまで）は STATUS 以外を ERROR,REPLAY で断る。
#if GNSS_DBD_ENABLE
static const char* ti_state() {
  switch (s_ti) {
    case TI_NONE: return s_gt.on ? "kept" : "none";
    case TI_WAIT: return "wait"; case TI_OK: return "ok"; case TI_NG: return "refused"; case TI_NOACK: return "noack";
  }
  return "?";
}
#endif

void gnss_dbd_command(const char* arg) {
#if !GNSS_DBD_ENABLE
  (void)arg;
  Serial.println("PONS:DBD,ERROR,DISABLED");
#else
  const uint32_t now = millis();
  if (strcmp(arg, "STATUS") == 0) {
    const uint8_t ld = load_state();
    Serial.printf("PONS:DBD,STATUS,state=%s,ep=%s,file=%s", st_name(s_st), ep_name(s_ep),
                  ld == LD_OK ? "ok" : (ld == LD_BUSY ? "reading" : s_ld_why));
    if (ld == LD_OK) Serial.printf(",msgs=%u,bytes=%lu", s_file.count, (unsigned long)s_file.bytes);
    if (ld == LD_OK && gnss_time_known()) Serial.printf(",age=%.1fh", file_age_h());
    Serial.printf(",gnss_id=%s,restore=%u/%u,resend=%u/%u,last_dump=%s,auto=%s", s_gid_ok ? "ok" : "none",
                  s_rc[0].ok, s_rc[0].sent, s_rc[1].ok, s_rc[1].sent, s_dp_last_ms ? s_dp_why : "none",
                  s_auto_off ? "off" : "on");
    // 外から来た時刻: none / <出どころ>/<kept|wait|ok|refused|noack>（入れた回数）
    if (s_ti == TI_NONE && !s_gt.on) Serial.printf(",time=none\r\n");
    else Serial.printf(",time=%s/%s(%u)\r\n", src_name(s_ti != TI_NONE ? s_ti_src : s_gt.src), ti_state(), s_ti_tries);
    return;
  }
  // リプレイ中と抜けた直後は、fix と日時が再生した CSV の値なので受けない（SAVE の時刻・試験の TTFF が狂う）
  if (getReplayMode() || s_after_replay) { Serial.println("PONS:DBD,ERROR,REPLAY"); return; }
  const bool busy = (s_st != ST_IDLE) || tx_busy() || load_state() == LD_BUSY;
  if (strcmp(arg, "SAVE") == 0) {
    double a, b, gs, t;
    const char* ng = nullptr;
    if (busy)                                  ng = "BUSY";
    else if (!gnss_get_own_fix(a, b, gs, t))   ng = "NO_FIX";
    else if (!s_gid_ok)                        ng = "NO_GNSS_ID";
    else if (!gnss_time_known())               ng = "NO_TIME";
    if (ng) { Serial.printf("PONS:DBD,ERROR,%s\r\n", ng); return; }
    start_dump("manual", now);
    Serial.println("PONS:DBD,OK,SAVE");
    return;
  }
  // 試験の名前と、COLD / COLDONLY のあとの ",TIME"
  char name[16];
  strlcpy(name, arg, sizeof(name));
  bool with_time = false;
  if (char* c = strchr(name, ',')) {
    *c = '\0';
    if (strcmp(c + 1, "TIME") == 0) with_time = true; else name[0] = '\0';
  }
  EpKind k = EP_NONE;
  if (strcmp(name, "COLD") == 0)          k = EP_TEST_DBD;
  else if (strcmp(name, "COLDONLY") == 0) k = EP_TEST_COLD;
  else if (strcmp(name, "HOT") == 0)      k = with_time ? EP_NONE : EP_TEST_HOT;   // ホットは時刻を失わない
  if (k == EP_NONE) {
    Serial.println("PONS:DBD,ERROR,UNKNOWN (SAVE / COLD / COLDONLY / HOT / STATUS / COLD,TIME / COLDONLY,TIME)");
    return;
  }
  // ★ 入れ直し待ち（ST_WAIT）は試験で上書きしてよい。GNSS をリセットして数え直すので待っていた入れ直しは
  //   意味を失い、COLD は SD から読み直す。屋内で COLD を打って時刻が読めないと、ここで断ると
  //   窓が終わるまで 30 分、次の試験ができなかった。送りかけのフレームと SD の読み込みは待つ（busy の残り）。
  const bool test_busy = (s_st != ST_IDLE && s_st != ST_WAIT) || tx_busy() || load_state() == LD_BUSY;
  if (test_busy) { Serial.println("PONS:DBD,ERROR,BUSY"); return; }
  // ,TIME: リセットの前に、いまの時刻を本体が覚える（最後の NAV-PVT の時刻と、それを受けた millis()）
  uint64_t self_utc = 0;
  uint32_t self_at  = 0;
  if (with_time) {
    if (!gnss_time_known()) { Serial.println("PONS:DBD,ERROR,NO_TIME"); return; }
    self_utc = (uint64_t)utc_now() * 1000ULL + (uint64_t)get_gnss_time().centisecond() * 10ULL;
    self_at  = get_gnss_fix_millis();
  }
  if (s_st == ST_WAIT) s_st = ST_IDLE;

  const uint16_t mask = (k == EP_TEST_HOT) ? 0x0000 : 0xFFFF;
  const uint8_t rst[4] = { (uint8_t)(mask & 0xFF), (uint8_t)(mask >> 8), 0x02, 0x00 };  // resetMode 0x02 = GNSS だけ
  gnss_ubx_send(0x06, 0x04, rst, sizeof(rst));   // UBX-CFG-RST（返事は来ない）
  start_episode(k, now ? now : 1);
  s_ep_time = with_time;
  // GNSS をリセットするので、外から入れた時刻も数え直す。素の試験では覚えていた時刻も捨てる（あとで入れない）。
  s_ti = TI_NONE; s_ti_tries = 0; s_ti_utc = 0; s_ti_checked = false;
  if (with_time) s_gt = { true, self_utc, self_at, (uint8_t)GNSS_TIME_SRC_SELF };
  else           s_gt.on = false;
  s_rs_pass = 0;
  s_rs_skip = nullptr;
  memset(s_rc, 0, sizeof(s_rc));
  enqueueTask(createLogSdfTask("GNSS DBD TEST %s: GNSS reset sent%s", ep_name(k),
                               s_auto_off ? "" : " (auto save off until reboot)"));
  s_auto_off = true;   // 再起動まで自動の吸い出しをしない（上の s_auto_off）
  if (k == EP_TEST_DBD) {   // SD から読み直す（起動時に読んだものは、時計が生きていれば手放している）
    mutex_enter_blocking(&taskQueueMutex);
    s_ld = LD_BUSY;
    mutex_exit(&taskQueueMutex);
    // キューが満杯なら読み込みは失敗扱い。それでも ST_TEST_LOAD を通して "not restored (task queue full)" を残す
    // （2026-11-10 まで ST_IDLE のままで、戻していないのに "TEST cold+dbd: fix ..." だけが出ていた）。
    if (!enqueueTask(createLoadDbdTask())) gnss_dbd_loaded_core1(false, nullptr, "task queue full");
    s_st = ST_TEST_LOAD;
  }
  Serial.printf("PONS:DBD,OK,%s\r\n", arg);
#endif
}

// ============================================================
//  外から GNSS へ時刻を入れる口（gnss_dbd.h の説明）
// ============================================================
// ★ 出どころ（PC・LoRa・…）を足すときもここへ渡すだけ。いつ入れるか・入れ直すかは time_block() が決める。
const char* gnss_dbd_give_time(uint64_t utc_ms, uint32_t at_ms, GnssTimeSrc src) {
#if !GNSS_DBD_ENABLE
  (void)utc_ms; (void)at_ms; (void)src;
  return "DISABLED";
#else
  if (getReplayMode() || s_after_replay) return "REPLAY";   // 日時と fix が再生した CSV の値なので比べられない
  if (!time_plausible(utc_ms)) return "BAD_TIME";
  const uint32_t now = millis();
  const char* b = time_block(now);
  if (b == TB_TEST)    return "TEST";
  if (b == TB_KNOWN)   return "NOT_NEEDED";
  if (b == TB_GIVEN)   return "ALREADY";
  if (b == TB_GAVE_UP) return "GAVE_UP";
  // いちばん新しい時刻を覚える（入れ直すときも、これをその時点まで進めて使う）
  const bool was_on = s_gt.on;
  s_gt = { true, utc_ms, at_ms, (uint8_t)src };
  if (b == nullptr) { give_time_now(); return "SENT"; }
  // 入れられるようになったら time_tick() が入れる。ログは覚え始めたときだけ（LoRa は 2 秒ごとに来るため）
  if (!was_on) enqueueTask(createLogSdfTask("GNSS TIME (%s): kept for later (%s)", src_name(src), b));
  return "LATER";
#endif
}

// USB シリアルの "PONS:TIME,<UNIX 時刻 ms>"（Flight Viewer が PONS:READY を受けるたびに送る。
// Arduino IDE のシリアルモニタからも打てる）。返事は "PONS:TIME,OK,<SENT|LATER|NOT_NEEDED|ALREADY>" か
// "PONS:TIME,ERROR,<理由>"。Flight Viewer は返事を見ない（PONS:ERROR, で始まらないので警告も出ない）。
void gnss_dbd_time_command(const char* arg, uint32_t rx_ms) {
  uint64_t v = 0;
  int n = 0;
  for (const char* c = arg; *c; c++) {
    if (*c < '0' || *c > '9' || n >= 16) { n = 0; break; }   // 数字だけ（いまは 13 桁）
    v = v * 10 + (uint64_t)(*c - '0');
    n++;
  }
  if (n == 0) { Serial.println("PONS:TIME,ERROR,BAD_FORMAT"); return; }
  const char* r = gnss_dbd_give_time(v, rx_ms, GNSS_TIME_SRC_PC);
  const bool ok = !strcmp(r, "SENT") || !strcmp(r, "LATER") || !strcmp(r, "NOT_NEEDED") || !strcmp(r, "ALREADY");
  Serial.printf("PONS:TIME,%s,%s\r\n", ok ? "OK" : "ERROR", r);
  // 入れた・覚えたときは give_time の側が残す。それ以外も、PC が時刻を送ってきたことは 1 行残す（接続のたびに 1 回）
  if (strcmp(r, "SENT") != 0 && strcmp(r, "LATER") != 0) {
#if GNSS_DBD_ENABLE
    char t[28];
    fmt_utc(t, sizeof(t), v);
    enqueueTask(createLogSdfTask("GNSS TIME (pc): %s (pc says %s)", r, t));
#else
    enqueueTask(createLogSdfTask("GNSS TIME (pc): %s", r));
#endif
  }
}
