// ============================================================
// File    : link_assist.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : GNSS の時刻の助け合い（LoRa）。仕組みは link_assist.h、計画と決めごとは docs/pons_ttff_assist_plan.md §4。
//           v7 と v7.1 で同じ中身。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/13
// ============================================================
//
// ■ 流れ
//   1. 窓（s_win）: 起動後、GNSS が時刻を知らないと分かったら開き（gnss_dbd_time_want() が YES）、
//      時刻を受け取った・GNSS が自分で読んだ・30 分たった、で閉じる。**その起動の間は二度と開けない。**
//      無線 OFF・link_assist OFF・リプレイ中・デモ中は閉じずに止めておく（window_tick の説明）。
//      ・機体（TX）: 窓の間はテレメトリに LINK_ST_TIME_WANTED を立て、送信の合間も mode 0 で提供ビーコンを聞く
//      ・ボート（RX）: 機体のテレメトリの時刻と提供ビーコンを使う。何も聞こえなければ要求ビーコンを送る
//   2. 受けた時刻は、同じ送り主の 2 発で確かめてから gnss_dbd_give_time(..., GNSS_TIME_SRC_LORA, 精度) に渡す（consider_time）。
//      入れるかどうか・入れ直しはそちらが決める（ここに条件を書き足さない）。
//      ★ **fix の前の時刻（GNSS の時計＝RTC から）も共有する**（2026-11-13）。送り主の GNSS の精度（NAV-PVT tAcc）を
//        テレメトリは status のビット（LINK_ST_TIME_COARSE / BAD）、ビーコンは BC_TACC で添え、受けた側はその精度を GNSS に伝える。
//        4 秒（LINK_ASSIST_TACC_MAX_MS。tAcc は 4.29 秒で頭打ち）より悪い・分からない時刻は使わない（書いた精度より外れると GNSS の性能がかえって落ちる）。
//   3. 提供（ボート）: 自分の GNSS が精度 4 秒以内の時刻を持っていて（fix は要らない）、機体の LINK_ST_TIME_WANTED か要求ビーコンを直前 15 秒に聞いていたら、
//      機体のテレメトリの合間の枠（lassist_slots）でくじを引いて送る。先に別のボートが送ったら黙り、しばらく休む。
//
// ■ 送る時刻の枠（機体のテレメトリを受けた時刻 t0 から。lassist_slots）
//   ・最初の枠は t0 ＋ LINK_ASSIST_SLOT_FIRST_MS（受信機は受けた直後に地図を描くので、その後ろ）
//   ・アッパーバンドは機体の休止（送信時間の 10 倍）が明けてから（休止の間に受けられるかが分からないため。計画書 §8）
//   ・最後の枠は、終わってから機体の次の送信まで LINK_ASSIST_SLOT_GUARD_MS 残るところまで。枠の幅は自分が塞がる時間
//     （送信時間の約 1.5 倍）＋ LINK_ASSIST_SLOT_SEP_MS
//   ・計算の結果（SF8 ローワーで 300 / 465 / 630ms。ローワーの SF11 は入らない＝送らない）は計画書 §4.5 の表
//
// ■ log.txt
//   LINK ASSIST: window open (GNSS has no time, TX)            窓を開けた
//   LINK ASSIST: time from aircraft|boat 1A2B rssi -71 (pair 2.0s, off +12ms) -> SENT|LATER|...   GNSS へ渡した
//   LINK ASSIST: time mismatch from ... (+2003ms over 2001ms), start over        2 発が食い違った
//   LINK ASSIST: window closed (gnss has time) after 12.3s                         窓を閉じた
//   LINK ASSIST: offering time (asked by aircraft|boat) / stopped offering (...)  提供の始まりと終わり
//   LINK ASSIST: asking for time (nothing heard for 10s)                           要求の始まり
//   LINK ASSIST: 60s win=open offer tx 3 rx 0 req tx 0 rx 0 used 1 noRf 0 late 0 busy 0   1 分のまとめ（動きがあったときだけ）
//
#include "link_assist.h"
#include "link.h"
#include "e220.h"
#include "gnss.h"
#include "gnss_dbd.h"
#include "mysd.h"
#include "settings.h"
#include "lora_link/link_beacon.h"
#include <pico/unique_id.h>

uint8_t link_assist_setting = 1;   // settings.txt の link_assist（mysd.cpp）

static bool assist_on() {
#if LINK_ASSIST_ENABLE
  return link_assist_setting != 0;
#else
  return false;
#endif
}

// ---- 窓（このユニットが時刻をほしがっている間）----
enum : uint8_t { W_WAIT = 0, W_OPEN, W_DONE };
static uint8_t  s_win = W_WAIT;
static uint32_t s_winOpenMs = 0;

// ---- 受けた時刻の確かめ（同じ送り主の 2 発）----
// ★ 送り主ごとに 1 発目を持つ。精度の違う送り主（RTC の時刻の機体と、fix しているボート）の時刻を組にすると、
//   どちらも正しいのに食い違って見え、いつまでも入れられないため。id 0 = 機体のテレメトリ。
struct TimeRef { bool on; uint16_t id; uint64_t utc_ms; uint32_t at_ms; };
#define LASSIST_REFS 4
static TimeRef     s_refs[LASSIST_REFS];
static uint8_t     s_refNext = 0;          // 次に上書きする枠（表が埋まったとき）
static const char* s_lastGive = nullptr;   // 最後に gnss_dbd_give_time() が返した文字列（ログを変わったときだけ出す）
static uint8_t     s_mismatchLogs = 0;     // 食い違いのログの数（1 起動 5 行まで）

// ---- 聞こえたもの ----
static uint32_t s_acT0 = 0;          // 機体のテレメトリを最後に受けた millis()（枠の基準）
static bool     s_acNew = false;     // 新しい機体のフレームを受けた（枠を引く合図）
static bool     s_acWants = false;   // 最後に受けた機体のフレームに LINK_ST_TIME_WANTED が立っていたか
static uint32_t s_askAcMs = 0;       // 機体の LINK_ST_TIME_WANTED を最後に聞いた
static uint32_t s_askReqMs = 0;      // 同じグループのボートの要求を最後に聞いた
static uint32_t s_srcHeardMs = 0;    // 時刻の出どころ（使える時刻の載った機体のテレメトリ・提供）を最後に聞いた
static uint32_t s_otherOfferMs = 0;  // 別のボートの提供を最後に聞いた（休む決まり・同じ回の譲り合い）

// ---- 送る側 ----
struct Plan { bool on; uint32_t at; uint32_t made; uint8_t kind; };
static Plan     s_plan = { false, 0, 0, 0 };
static uint8_t  s_seq = 0;
static uint32_t s_lastOfferTxMs = 0;   // 0 = まだ送っていない
static uint32_t s_lastReqTxMs = 0;
static bool     s_offering = false;    // 提供している（ログの始まり・終わり用）
static uint8_t  s_offerLogs = 0;       // 提供の始まり・終わりのログの数
static uint16_t s_offerRun = 0;        // 今回の提供で送った数
static bool     s_asking = false;      // 要求を送り始めた（ログ用）
static bool     s_auxWatch = false;    // 送ったあと AUX が落ちるか見ている（noRf）
static uint32_t s_auxWatchMs = 0;
static bool     s_noRoomLogged = false;

// ---- 1 分のまとめ ----
static uint16_t s_cOfferTx = 0, s_cOfferRx = 0, s_cReqTx = 0, s_cReqRx = 0, s_cUsed = 0;
static uint16_t s_cNoRf = 0, s_cLate = 0, s_cBusy = 0, s_cOtherGrp = 0;
static uint32_t s_reportMs = 0;

// ============================================================
//  小物
// ============================================================
// 送り主の短い ID（本体の固有 ID の CRC16。0 は使わない）
static uint16_t my_id() {
  static uint16_t id = 0;
  if (id == 0) {
    pico_unique_board_id_t u;
    pico_get_unique_board_id(&u);
    id = link_crc16(u.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
    if (id == 0) id = 1;
  }
  return id;
}
static uint32_t rnd(uint32_t n) { return n ? rp2040.hwrand32() % n : 0; }
static bool after(uint32_t a, uint32_t b) { return (int32_t)(a - b) >= 0; }   // a が b 以後か（millis の巻き戻りに強い）
static bool heard_within(uint32_t t, uint32_t now, uint32_t ms) { return t != 0 && now - t < ms; }

static uint32_t airtime_ms() { return e220_airtime_ms(link_radio_profile); }
static uint32_t busy_ms()    { return airtime_ms() * 3 / 2; }   // 自分が塞がる時間（キャリアセンスを含む実測は約 1.5 倍）
static bool     upper()      { return link_radio_ch_upper(link_radio_ch); }
// アッパーバンドで 1 回送ったあとの休止を含めて、次に送れるまで [ms]
static uint32_t rest_ms()    { return upper() ? busy_ms() + airtime_ms() * LINK_UPPER_PAUSE_X + LINK_ASSIST_SLOT_GUARD_MS : 0; }
// 受けたフレームの時刻が「いつ」の値か: 受けた millis() から、電波に乗っていた時間と UART の 33 バイト（115200bps で約 3ms）を引く
static uint32_t at_of_rx(uint32_t now) { return now - airtime_ms() - 3; }

// t0（機体のフレームを受けた時刻）から見た枠の位置 [ms]。戻り値は枠の数（0 = 機体の送信の合間に入らない）。
// ★ t0 の正確さは、受信機が機体のフレームの届くころに描かないこと（link_draw_window_ok）に頼っている。
//   届いたときにループが描画で止まっていると t0 が遅れ、枠が後ろへずれて機体の次の送信に近づく。
//   機体の送信は遅れることはあっても早まることはない（1 秒たった次のループで送る）ので、t0 が正しければ重ならない。
static uint8_t lassist_slots(uint32_t* out) {
  const uint32_t per  = link_tx_period_ms();
  const uint32_t busy = busy_ms();
  uint32_t first = LINK_ASSIST_SLOT_FIRST_MS;
  // アッパーは機体の休止が明けてから（t0 は機体の送信が終わった直後なので、休止の長さだけ待てばよい）
  if (upper()) { const uint32_t r = airtime_ms() * LINK_UPPER_PAUSE_X + LINK_ASSIST_SLOT_GUARD_MS; if (r > first) first = r; }
  // 機体の次の送信は t0 ＋（周期 − 機体が塞がる時間 − 受けて気づくまでの 5ms）で始まる。自分の送信はその手前で終える
  const int32_t last = (int32_t)per - (int32_t)(2 * busy) - 5 - (int32_t)LINK_ASSIST_SLOT_GUARD_MS;
  uint8_t n = 0;
  for (uint32_t s = first; (int32_t)s <= last && n < LINK_ASSIST_SLOTS_MAX; s += busy + LINK_ASSIST_SLOT_SEP_MS) out[n++] = s;
  return n;
}

// 自分の GNSS の時刻の精度 [ms]（NAV-PVT tAcc を切り上げ）。分からなければ 0xFFFF
// ★ tAcc は 32 ビットの ns なので 4.29 秒までしか表せない（それ以上は頭打ち）。足し算で切り上げると溢れて
//   「とても確か」に化けるので、割ってから余りで切り上げる（PC の試験台で見つけた）。
static uint16_t own_tacc_ms() {
  const uint32_t ns = gnss_own_tacc_ns();
  if (ns == 0xFFFFFFFFUL) return 0xFFFF;
  return (uint16_t)(ns / 1000000UL + ((ns % 1000000UL) ? 1 : 0));   // 最大 4295
}

// 自分の GNSS のいまの UTC [ms] と精度 [ms]。時刻が有効で、精度が LINK_ASSIST_TACC_MAX_MS 以内で、NAV-PVT が新しいときだけ。
// ★ **fix は要らない**（2026-11-13〜）。BT1 が生きていれば、fix の前でも GNSS の時計（RTC）の時刻が精度つきで出る。
// ミラー中もこちらは自機の値（get_gnss_date/time は差し替わらない）。
static bool own_utc_ms_now(uint64_t* out, uint16_t* tacc) {
  if (link_rf_suppressed()) return false;           // リプレイ中は ubx_* が CSV の値
  if (!get_gnss_connection()) return false;
  const GnssDate d = get_gnss_date();
  const GnssTime t = get_gnss_time();
  if (!d.isValid() || !t.isValid() || d.year() < 2026) return false;
  const uint16_t ta = own_tacc_ms();
  if (ta > LINK_ASSIST_TACC_MAX_MS) return false;
  const uint32_t age = millis() - get_gnss_fix_millis();   // 最後の NAV-PVT を受けてから
  if (age > 3000) return false;
  const int64_t days = w2_days_from_civil(d.year(), d.month(), d.day());
  *out = (uint64_t)((days * 86400LL + t.hour() * 3600LL + t.minute() * 60LL + t.second()) * 1000LL)
       + (uint64_t)t.centisecond() * 10ULL + age;
  if (tacc) *tacc = ta;
  return true;
}

// 受けた時刻の精度 [ms] から、GNSS へ伝える精度 [ms]。0 = 既定（GNSS_TIME_ACC_LORA_MS）。使えなければ 0xFFFFFFFF
static uint32_t told_acc_ms(uint32_t sender_tacc_ms) {
  if (sender_tacc_ms > LINK_ASSIST_TACC_MAX_MS) return 0xFFFFFFFFUL;
  if (sender_tacc_ms <= LINK_ASSIST_TACC_FINE_MS) return 0;
  return sender_tacc_ms + LINK_ASSIST_TACC_MARGIN_MS;
}

// ============================================================
//  受けた時刻を確かめて GNSS へ渡す
// ============================================================
// src_id: 0 = 機体のテレメトリ、それ以外 = 提供したボートの ID。acc_ms: GNSS へ伝える精度（0 = 既定。told_acc_ms）
static void consider_time(uint64_t utc_ms, uint32_t at_ms, uint16_t src_id, int16_t rssi, uint32_t acc_ms) {
  // この送り主の 1 発目
  TimeRef* r = nullptr;
  for (int i = 0; i < LASSIST_REFS; i++) if (s_refs[i].on && s_refs[i].id == src_id) { r = &s_refs[i]; break; }
  if (!r) {
    for (int i = 0; i < LASSIST_REFS && !r; i++) if (!s_refs[i].on) r = &s_refs[i];
    if (!r) { r = &s_refs[s_refNext]; s_refNext = (uint8_t)((s_refNext + 1) % LASSIST_REFS); }
    *r = { true, src_id, utc_ms, at_ms };
    return;
  }
  if (at_ms - r->at_ms > LINK_ASSIST_PAIR_MAX_MS) { *r = { true, src_id, utc_ms, at_ms }; return; }
  const uint32_t drx = at_ms - r->at_ms;
  if (drx < LINK_ASSIST_PAIR_MIN_MS) return;          // 1 発目と近すぎる。1 発目を残して次を待つ
  const int64_t off = (int64_t)utc_ms - (int64_t)r->utc_ms - (int64_t)drx;
  *r = { true, src_id, utc_ms, at_ms };                // 次はこれを 1 発目にする
  char who[16];
  if (src_id == 0) snprintf(who, sizeof(who), "aircraft");
  else             snprintf(who, sizeof(who), "boat %04X", src_id);
  if (off > (int64_t)LINK_ASSIST_PAIR_TOL_MS || off < -(int64_t)LINK_ASSIST_PAIR_TOL_MS) {
    // 時の繰り上げの取り違え・止まった時刻・壊れた時計。何もしないより悪いので入れない
    if (s_mismatchLogs < 5) {
      s_mismatchLogs++;
      enqueueTask(createLogSdfTask("LINK ASSIST: time mismatch from %s (%+ldms over %lums), start over",
                                   who, (long)off, (unsigned long)drx));
    }
    return;
  }
  const char* res = gnss_dbd_give_time(utc_ms, at_ms, GNSS_TIME_SRC_LORA, acc_ms);
  s_cUsed++;
  if (res != s_lastGive) {   // 文字列リテラルなので、同じ返事なら同じポインタ（2 秒ごとに同じ行を出さない）
    s_lastGive = res;
    char acc[16];
    if (acc_ms) snprintf(acc, sizeof(acc), "acc %.1fs", acc_ms / 1000.0f);
    else        snprintf(acc, sizeof(acc), "acc default");
    enqueueTask(createLogSdfTask("LINK ASSIST: time from %s rssi %d (pair %.1fs, off %+ldms, %s) -> %s",
                                 who, (int)rssi, drx / 1000.0f, (long)off, acc, res));
  }
}

// ============================================================
//  窓
// ============================================================
static void window_close(const char* why, uint32_t now) {
  s_win = W_DONE;
  s_plan.on = false;
  enqueueTask(createLogSdfTask("LINK ASSIST: window closed (%s) after %.1fs",
                               why, (now - s_winOpenMs) / 1000.0f));
}

// ★ 無線が OFF・link_assist が OFF・リプレイ中・デモ中は、窓を**閉じずに止めておく**（何も送らず、何も使わない。
//   機体の LINK_ST_TIME_WANTED と起こしたままも、それぞれ役と lassist_keep_awake() の条件で落ちる）。
//   無線モードは回転式で OFF を通ることがあり、閉じてしまうとその起動の間は二度と開かないため。
//   30 分の上限だけは止めている間も数える。
static void window_tick(uint32_t now) {
  if (s_win == W_DONE) return;
  if (s_win == W_OPEN && now - s_winOpenMs >= LINK_ASSIST_WINDOW_MAX_MS) { window_close("time limit", now); return; }
  if (!assist_on() || link_mode_setting == LINK_MODE_OFF || link_rf_suppressed()) return;
  const GnssTimeWant w = gnss_dbd_time_want();   // リプレイを抜けた直後など、分からない間は UNKNOWN
  if (s_win == W_WAIT) {
    if (w == GNSS_TIME_WANT_NO) { s_win = W_DONE; return; }      // 時計が生きていた（ふつうの起動）。何もしない
    if (w != GNSS_TIME_WANT_YES) return;                          // まだ分からない（最初の NAV-PVT の前）
    s_win = W_OPEN;
    s_winOpenMs = now ? now : 1;
    memset(s_refs, 0, sizeof(s_refs));
    enqueueTask(createLogSdfTask("LINK ASSIST: window open (GNSS has no time, %s)",
                                 link_mode_setting == LINK_MODE_TX ? "TX" : "RX"));
    return;
  }
  if (w == GNSS_TIME_WANT_NO) window_close("gnss has time", now);
}

// 窓が開いていて、いま動いてよいか（止めている間は false）
static bool window_active() {
  return s_win == W_OPEN && assist_on() && link_mode_setting != LINK_MODE_OFF && !link_rf_suppressed();
}

// ============================================================
//  送る
// ============================================================
static bool send_beacon(uint8_t kind, uint32_t now) {
  LinkBeacon b;
  memset(&b, 0, sizeof(b));
  b.group  = link_group;
  b.seq    = s_seq++;
  b.kind   = kind;
  b.sender = my_id();
  if (kind == LINK_BC_OFFER) {
    if (!own_utc_ms_now(&b.utc_ms, &b.tacc_ms)) return false;
    b.has_time = true;
    const int ns = get_gsa_numsat();
    b.numsat = (uint8_t)(ns < 0 ? 0 : (ns > 255 ? 255 : ns));
    double lat, lon, gs, trk;
    if (gnss_get_own_fix(lat, lon, gs, trk)) {
      b.has_pos  = true;
      b.lat_1e7  = (int32_t)lround(lat * 1e7);
      b.lon_1e7  = (int32_t)lround(lon * 1e7);
      const double a = gnss_own_altitude();
      b.alt_m    = (int16_t)(a > 32000.0 ? 32000 : (a < -32000.0 ? -32000 : lround(a)));
    }
  }
  uint8_t f[LINK_BC_LEN];
  link_bc_encode(&b, f);
  // ★ 受信役（mode 0）のまま送る。受信の組み立てバッファは捨てない（e220_send() は触らない。pons_link.md §6.8）。
  //   AUX が Low（受信の吐き出し中・前の送信・アッパーの休止）なら e220_send() が断るので、その回は送らない。
  if (!e220_send(f, LINK_BC_LEN)) { s_cBusy++; return false; }
  s_auxWatch = true;
  s_auxWatchMs = now;
  return true;
}

// 提供してよい状態か（頼まれていて、自分の GNSS が精度 4 秒以内の時刻を持っている）。理由を返す（nullptr = よい）
static const char* offer_block(uint32_t now) {
  if (link_mode_setting != LINK_MODE_RX) return "not receiver";
  if (!assist_on())                      return "link_assist OFF";
  if (link_rf_suppressed())              return "replay/demo";
  const bool ac  = s_acWants && heard_within(s_askAcMs, now, LINK_ASSIST_ASK_HOLD_MS);
  const bool req = heard_within(s_askReqMs, now, LINK_ASSIST_ASK_HOLD_MS);
  if (!ac && !req)                       return "not asked";
  uint64_t u;
  if (!own_utc_ms_now(&u, nullptr))      return "no time";
  return nullptr;
}

// 提供の間隔（休む決まりの基準）。ローワーは 2 秒（機体の 2 周期に 1 回）、アッパーの SF9・SF10 は機体の 1 周期
static uint32_t offer_interval() {
  const uint32_t per = link_tx_period_ms();
  return per > LINK_ASSIST_OFFER_MIN_MS ? per : LINK_ASSIST_OFFER_MIN_MS;
}

// この回に提供するか（間隔と休む決まり）
static bool offer_due(uint32_t now) {
  if (heard_within(s_otherOfferMs, now, LINK_ASSIST_YIELD_OFFERS * offer_interval())) return false;   // 別のボートが送っている
  // 前回から「間隔 − 半周期」たっていれば、この回に引く（枠の位置が回ごとに違うので、ちょうど 2 秒では切らない）
  const int32_t gap = (int32_t)LINK_ASSIST_OFFER_MIN_MS - (int32_t)(link_tx_period_ms() / 2);
  if (s_lastOfferTxMs != 0 && gap > 0 && now - s_lastOfferTxMs < (uint32_t)gap) return false;
  return true;
}

// 困っているボートが要求を送るか
static bool req_due(uint32_t now) {
  if (link_mode_setting != LINK_MODE_RX || !window_active()) return false;
  if (now - s_winOpenMs < LINK_ASSIST_REQ_WAIT_MS) return false;                      // まず聞く
  if (heard_within(s_srcHeardMs, now, LINK_ASSIST_SRC_HEARD_MS)) return false;        // 機体のテレメトリか提供が聞こえている
  if (s_acWants && heard_within(s_askAcMs, now, LINK_ASSIST_SRC_HEARD_MS)) return false;   // 機体が頼んでいる（提供は同報で届く）
  if (heard_within(s_askReqMs, now, LINK_ASSIST_REQ_INTERVAL_MS)) return false;       // 別のボートが頼んでいる
  if (heard_within(s_lastReqTxMs, now, LINK_ASSIST_REQ_INTERVAL_MS)) return false;
  return true;
}

// 機体のフレームを受けた回の枠を引く（t0 はそのフレームを受けた millis()）
static void plan_in_cycle(uint32_t t0, uint8_t kind) {
  uint32_t sl[LINK_ASSIST_SLOTS_MAX];
  const uint8_t n = lassist_slots(sl);
  if (n == 0) {
    if (!s_noRoomLogged) {
      s_noRoomLogged = true;
      enqueueTask(createLogSdfTask("LINK ASSIST: no slot between sender frames (ch %u sf %u), cannot send",
                                   link_radio_ch, e220_profile_to_sf(link_radio_profile)));
    }
    return;
  }
  uint8_t idx = 0;
  if (n == 1) { if (rnd(2)) return; }   // 枠が 1 つなら、送るかどうかをくじで（2 台が毎回ぶつからないように）
  else idx = (uint8_t)rnd(n);
  s_plan = { true, t0 + sl[idx], millis(), kind };
}
// 機体が聞こえないとき（ボート同士）。いまから少し後のどこか
static void plan_free(uint32_t now, uint8_t kind) {
  s_plan = { true, (uint32_t)(now + LINK_ASSIST_SLOT_FIRST_MS + rnd(700)), now, kind };
}

static void send_tick(uint32_t now) {
  // 送ったあと AUX が落ちたか（落ちないまま LINK_TX_TXSTART_MS ＝ モジュールに捨てられた）
  if (s_auxWatch) {
    if (digitalRead(E220_AUX_PIN) == LOW) s_auxWatch = false;
    else if (now - s_auxWatchMs >= LINK_TX_TXSTART_MS) { s_auxWatch = false; s_cNoRf++; }
  }

  // ---- 提供の始まり・終わり（ログ）----
  // ★ 自分の fix がちらつくと始まり・終わりが続くので、ログは 1 起動 20 行まで
  const char* ob = offer_block(now);
  if (ob == nullptr && !s_offering) {
    s_offering = true;
    s_offerRun = 0;
    const bool by_ac = s_acWants && heard_within(s_askAcMs, now, LINK_ASSIST_ASK_HOLD_MS);
    if (s_offerLogs < 20) {
      s_offerLogs++;
      enqueueTask(createLogSdfTask("LINK ASSIST: offering time (asked by %s)", by_ac ? "aircraft" : "boat"));
    }
  } else if (ob != nullptr && s_offering) {
    s_offering = false;
    if (s_plan.on && s_plan.kind == LINK_BC_OFFER) s_plan.on = false;
    if (s_offerLogs < 20) {
      s_offerLogs++;
      enqueueTask(createLogSdfTask("LINK ASSIST: stopped offering (%s), sent %u", ob, (unsigned)s_offerRun));
    }
  }

  // ---- 枠を引く ----
  const bool ac_recent = heard_within(s_acT0, now, link_tx_period_ms() * 3 / 2);
  if (!s_plan.on) {
    uint8_t kind = 0;
    if (s_offering && offer_due(now)) kind = LINK_BC_OFFER;
    else if (req_due(now))            kind = LINK_BC_REQ;
    if (kind) {
      if (ac_recent) { if (s_acNew) plan_in_cycle(s_acT0, kind); }   // 機体の送信の合間に入れる
      else plan_free(now, kind);
    }
  }
  s_acNew = false;

  // ---- 枠の時刻が来たら送る ----
  if (!s_plan.on || !after(now, s_plan.at)) return;
  const Plan p = s_plan;
  s_plan.on = false;
  if (now - p.at > LINK_ASSIST_SLOT_LATE_MS) { s_cLate++; return; }        // ループが遅れた。ずれた時刻に送らない
  if (p.kind == LINK_BC_OFFER) {
    if (offer_block(now) != nullptr) return;
    if (s_otherOfferMs != 0 && after(s_otherOfferMs, p.made)) return;      // この回、先に別のボートが送った
    if (s_lastOfferTxMs != 0 && now - s_lastOfferTxMs < rest_ms()) return;  // アッパーの休止が明けていない
    if (send_beacon(LINK_BC_OFFER, now)) { s_lastOfferTxMs = now ? now : 1; s_cOfferTx++; s_offerRun++; }
  } else {
    if (!req_due(now)) return;
    if (send_beacon(LINK_BC_REQ, now)) {
      s_lastReqTxMs = now ? now : 1;
      s_cReqTx++;
      if (!s_asking) {
        s_asking = true;
        enqueueTask(createLogSdfTask("LINK ASSIST: asking for time (nothing heard for %lus)",
                                     (unsigned long)(LINK_ASSIST_SRC_HEARD_MS / 1000)));
      }
    }
  }
}

static void report_tick(uint32_t now) {
  if (now - s_reportMs < 60000UL) return;
  s_reportMs = now;
  const bool any = s_cOfferTx || s_cOfferRx || s_cReqTx || s_cReqRx || s_cUsed || s_cNoRf || s_cLate || s_cBusy || s_cOtherGrp;
  if (!any && s_win != W_OPEN) return;
  enqueueTask(createLogSdfTask(
      "LINK ASSIST: 60s win=%s offer tx %u rx %u req tx %u rx %u used %u noRf %u late %u busy %u othergrp %u",
      s_win == W_OPEN ? "open" : (s_win == W_DONE ? "done" : "wait"),
      (unsigned)s_cOfferTx, (unsigned)s_cOfferRx, (unsigned)s_cReqTx, (unsigned)s_cReqRx, (unsigned)s_cUsed,
      (unsigned)s_cNoRf, (unsigned)s_cLate, (unsigned)s_cBusy, (unsigned)s_cOtherGrp));
  s_cOfferTx = s_cOfferRx = s_cReqTx = s_cReqRx = s_cUsed = s_cNoRf = s_cLate = s_cBusy = s_cOtherGrp = 0;
}

// ============================================================
//  外から
// ============================================================
// 自分の GNSS の時刻の精度を log.txt に残す（時刻が出た最初と、最初の fix）。RTC の時刻がどれだけ確かかを実機で見るため
static void own_tacc_log_tick() {
  static bool logged_time = false, logged_fix = false;
  if (logged_fix || link_rf_suppressed()) return;
  const GnssDate d = get_gnss_date();
  const GnssTime t = get_gnss_time();
  if (!d.isValid() || !t.isValid() || d.year() < 2026) return;
  const bool fix = gnss_own_fixok();
  if (!logged_time || fix) {
    const uint32_t ns = gnss_own_tacc_ns();
    char rtc[8] = "?";
    if (gnss_rtc_boot_checked()) strlcpy(rtc, gnss_rtc_alive_on_boot() ? "alive" : "lost", sizeof(rtc));
    enqueueTask(createLogSdfTask("LINK ASSIST: own time tAcc %.3fs (%s, rtc=%s)",
                                 ns == 0xFFFFFFFFUL ? -1.0f : ns / 1e9f, fix ? "at first fix" : "no fix yet", rtc));
    logged_time = true;
    if (fix) logged_fix = true;
  }
}

void lassist_loop() {
  const uint32_t now = millis();
  own_tacc_log_tick();
  window_tick(now);
  send_tick(now);
  report_tick(now);
}

void lassist_on_beacon(const uint8_t* p, int16_t rssi) {
  if (!assist_on()) return;
  LinkBeacon b;
  if (!link_bc_decode(p, &b)) return;                    // 知らない版・種類
  if (b.group != link_group) { s_cOtherGrp++; return; }  // 別のグループのものは使わない（2026-11-13 決定）
  if (b.sender == my_id()) return;
  const uint32_t now = millis();
  if (b.kind == LINK_BC_REQ) {
    s_cReqRx++;
    s_askReqMs = now ? now : 1;
    return;
  }
  s_cOfferRx++;
  s_otherOfferMs = now ? now : 1;
  if (!b.has_time) return;
  const uint32_t acc = told_acc_ms(b.tacc_ms);       // 送り主の精度が 4 秒より悪い・分からなければ使わない
  if (acc == 0xFFFFFFFFUL) return;
  s_srcHeardMs = s_otherOfferMs;
  if (window_active()) consider_time(b.utc_ms, at_of_rx(now), b.sender, rssi, acc);
}

void lassist_on_telem(const uint8_t* p, const LinkW2Rx* st, int16_t rssi) {
  if (!assist_on()) return;
  const uint32_t now = millis();
  const uint16_t sb = w2_get16(p + W2_STATUS);
  s_acT0  = now ? now : 1;
  s_acNew = true;
  s_acWants = (sb & LINK_ST_TIME_WANTED) != 0;
  if (s_acWants) s_askAcMs = s_acT0;
  // 機体の時刻（fix の前の RTC の時刻も。精度は status のビット）。使えるものが載っていれば「時刻の出どころが聞こえる」
  uint64_t utc;
  const uint8_t q = link_w2_utc_ms(p, st, &utc);
  if (q == LINK_W2_TIME_NONE) return;
  s_srcHeardMs = s_acT0;
  if (!window_active() || link_mode_setting != LINK_MODE_RX) return;
  const uint32_t acc = (q == LINK_W2_TIME_COARSE) ? LINK_ASSIST_TACC_MAX_MS + LINK_ASSIST_TACC_MARGIN_MS : 0;
  consider_time(utc + LINK_ASSIST_TELEM_AGE_MS, at_of_rx(now), 0, rssi, acc);
}

// 送信機のテレメトリの status に添える、自分の時刻の確からしさ（LINK_ST_TIME_COARSE / LINK_ST_TIME_BAD）。
// 時刻が無いときは 0（時刻の欄そのものが「無し」になる）。よいとき（1 秒以内）も 0（古い受信機の画面を赤くしない）。
uint16_t lassist_time_status_bits() {
  const GnssDate d = get_gnss_date();
  const GnssTime t = get_gnss_time();
  if (!d.isValid() || !t.isValid()) return 0;
  const uint16_t ta = own_tacc_ms();
  if (ta > LINK_ASSIST_TACC_MAX_MS)  return LINK_ST_TIME_BAD;
  if (ta > LINK_ASSIST_TACC_FINE_MS) return LINK_ST_TIME_COARSE;
  return 0;
}

bool lassist_time_wanted() { return link_mode_setting == LINK_MODE_TX && window_active(); }
bool lassist_keep_awake()  { return lassist_time_wanted() && !link_chscan_running(); }
