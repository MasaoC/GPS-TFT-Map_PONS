// ============================================================
// File    : backlight.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : v7.1 の TFT バックライト。輝度スイッチと照度センサー（VEML7700）で
//           PWM の点灯率を決める。電池低下のときは 2 段階で点滅させて電池をもたせる。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/08
// ============================================================
//
// ■ 決め方（値は settings.h の TFT_BL_* / BL_AUTO_* / BL_DIM_*。括弧は設定画面の表示名 = backlight_mode_label）
//   輝度スイッチ High … 照度によらず最大（SW BRIGHT）
//   輝度スイッチ Low  … VEML7700 の照度で自動（AUTO）。曲線は Eco / Bright の 2 種類（設定画面の
//                        AUTO Brightness。auto_duty）。センサーが一度も読めていないときは
//                        TFT_BL_DUTY_DIM（NO SENSOR）、途中で読めなくなったら最後の照度のまま（LAST LUX）
//   待機中の自動減光 … 静止が BL_DIM_TIMEOUT_MS 続いたら TFT_BL_DUTY_DIM（IDLE DIM。dim_tick）。
//                        照度で手が来たと分かったら（1 秒で半分に落ちた＝かざした、30% 落ちて
//                        1 秒以内に戻った＝あおいだ。lux_hand_feed）戻す。
//                        ただし暗い所では自動調光の値を超えない（減光で明るくしない）
//
// ■ 電池低下のとき（backlight_set_saver。判定は GPS_TFT_map.ino の batt_alarm_tick）
//   第 1 段階（<=3.5V）は BL_SAVER_ON_MS 点けて BL_SAVER_OFF_MS 消す、
//   第 2 段階（<=3.0V）は BL_SAVER_CRIT_ON_MS 点けて BL_SAVER_CRIT_OFF_MS 消すをくり返す。
//   点いている間は上の明るさのまま。
//   ★ 消している間も「本来の明るさ」（s_duty）は追い続ける。出力だけ 0 にする。
//     s_duty まで 0 にすると、点いた瞬間から照度へゆっくり寄せ直す（BL_AUTO_TAU_MS）ので、
//     2 秒の点灯がほとんど暗いまま終わる。
//
// ■ ハード
//   Q1（P-ch）のゲートを GPIO34 の PWM で駆動。**Low で点灯**。輝度スイッチは GPIO35（外付けプルダウン R48）。
//   ★ PWM を設定するまでは、GPIO のリセット時のプルダウンが R47（100kΩ）に勝って、ほぼ最大で点く。
//   VEML7700（0x10）はケースカバー基板にあり、J8 経由で i2c0 に載る（MS5611 と共用）。
//   カバーを付けていない機体もあるので、居なくても動くようにしてある。
#include "backlight.h"
#include "settings.h"

#include "airdata.h"   // myWire（i2c0）
#include "mysd.h"
#include "gnss.h"      // gnss_get_own_fix(), get_gnss_connection(), getReplayMode(), is_demo_active()
#include "imu.h"       // get_imu_alive()
#include "attitude.h"   // attitude_is_static()
#include "power.h"     // usb_power_present(), power_sw_off_on_usb(), power_fault_active(), power_charge_hot_active(), power_overdischarge_latched()
#include "link.h"      // link_mirror_active()
#include "display_tft.h"  // map_notice_shown()（地図に通知の箱が出ている間は減光しない）
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"                 // 診断（log_pwm_diag）でピンを読む
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/timer.h"

#define BL_TICK_MS         250UL    // 点灯率を決める間隔
// 照度を読む間隔（BL_TICK_MS より細かく）。明るい所（VEML_CONF_LO）は積分が 25ms しかないので、
// 250ms ごとでは 1 割の時間しか見ておらず、あおいだ手の影（1 回 50〜100ms）を取りこぼす
// （シミュレーションで 1 回のあおぎを 14〜35% しか拾えなかった）。50ms なら半分を見ていて、ほぼ必ず拾う。
// i2c0（400kHz）の 1 回の読みは約 0.15ms（48 ビット分の計算値）なので、1 秒に約 3ms。
// 同じバスの MS5611 は 24ms ごとの変換と読み出しで 1 秒に約 16ms 使っている。
// IMU（4ms ごと）の読み出しが待たされるのは、重なった回でもこの 0.15ms だけ。
#define ALS_READ_MS         50UL
#define ALS_FIRST_READ_MS  150UL    // 設定してから 1 回目の積分（100ms）が終わるまで読まない
#define ALS_RETRY_MS     10000UL    // 居ないときに探しに行く間隔
#define ALS_CONF_CHECK_MS 5000UL    // 設定が残っているかを確かめる間隔（下の read_als）

#define VEML7700_ADDR   0x10
#define VEML_REG_CONF   0x00
#define VEML_REG_ALS    0x04
#define VEML_REG_ID     0x07
#define VEML_ID_LOW     0x81        // ID レジスタの下位バイト（デバイスコード）
// 測る範囲は 2 段で、照度に合わせて切り替える（read_als。閾値は settings.h の VEML_RANGE_*）。
//   VEML_CONF_HI … ゲイン ×1・積分 100ms・割り込みなし・動作中（ALS_SD=0）。全ビット 0 がちょうどこの設定。
//                  約 3775 lx で飽和（65535）。暗い所の分解能（0.0576 lx）がほしいので普段はこちら。
//   VEML_CONF_LO … ゲイン ×1/8（ALS_GAIN bit12:11 = 10b）・積分 25ms（ALS_IT bit9:6 = 1100b）。
//                  感度は 1/32 で約 120000 lx まで。Eco の曲線は 20000 lx 前後で点灯率が変わるので、
//                  ×1 のまま飽和させると曇りと晴れを見分けられない。
#define VEML_CONF_HI    0x0000
#define VEML_CONF_LO    ((0x2u << 11) | (0xCu << 6))
#define VEML_ALS_SD     0x0001      // 停止。範囲を変えるときに一度止めてから動かす（積分をやり直させる）

static bool     s_sw_bright  = false;
static float    s_duty       = 0.0f;    // いま出している点灯率
static bool     s_als_ok     = false;
static uint32_t s_als_t0     = 0;       // 設定した時刻（ALS_FIRST_READ_MS の起点）
static uint32_t s_als_retry  = 0;
static uint32_t s_als_check  = 0;       // 設定を最後に確かめた時刻
static float    s_lux        = -1.0f;   // 点灯率に使う照度。-1 = 一度も読めていない
static bool     s_lux_fresh  = false;   // s_lux が今のセンサーから読めた値か（ログ用）
static bool     s_als_lo     = false;   // いま VEML_CONF_LO（明るい所用）で測っているか
// 設定画面の AUTO Brightness（SD から読むのは Core1、使うのは Core0 なので volatile）。
static volatile uint8_t s_auto_mode    = BL_AUTO_ECO;
static volatile bool    s_auto_changed = false;   // 切り替えた直後の 1 回はすぐに合わせる
static uint32_t       s_last_ms  = 0;
static BacklightSaver s_saver    = BL_SAVER_OFF; // 電池低下で点滅中か（2 段階）
static uint32_t       s_saver_t0 = 0;            // 今の段階の点滅を始めた時刻（点灯から始める）
static float          s_out      = -1.0f;        // 実際に出している点灯率（点滅で 0 になる。-1 = まだ出していない）

// ---- 待機中自動減光（Auto-Dimming）ステート ----
static bool     s_dimmed       = false;    // 待機中自動減光中か
static uint32_t s_dim_wait_t0  = 0;        // 待機継続の起点時刻 [ms]

// ---- 照度で見る「手が来た」（かざす・あおぐ・人の影）。read_als() が見つけ、dim_tick() が使う ----
// 新しく読めた照度を、直近約 1 秒（BL_DIM_LUX_WINDOW_MS）の読みの最大と比べる（2 つの形は settings.h）。
// ★ 照度は**上限なし**で比べる。前の「ゆっくり追う基準から 30%」は晴れ⇔曇りでも反応するので
//   3775 lx で頭打ちにしており、本番の明るさ（20000 lx 以上）では手の影がほとんど効かなかった。
//   「1 秒で半分」なら手の影は捕まえ、数秒かけて変わる雲の影はまず捕まえない。
// ★ あおぐ（下がって 1 秒以内に戻る）は 30% と浅くてよい。雲の縁は下がったまま戻らないので、
//   「戻る」を見れば雲と手を分けられる。
// ★ 読む間隔は ALS_READ_MS（50ms）。履歴は時刻つきで持ち、読みが遅れた回があっても 1 秒で数える。
#define LUX_HIST_N  (BL_DIM_LUX_WINDOW_MS / ALS_READ_MS + 1)   // 50ms ごとの読みで 21 個
struct LuxSample { uint32_t ms; float lux; };
static LuxSample s_lux_hist[LUX_HIST_N];
static uint8_t s_lux_hist_n = 0;       // 入っている数
static uint8_t s_lux_hist_i = 0;       // 次に書く位置
static bool    s_lux_hand   = false;   // 手が来たのを見つけた（dim_tick が毎回読んで消す）
static char    s_lux_hand_why[40];     // ログ用（"drop 52000->14000" / "wave 52000->30000->45000"）
// あおぐの途中（30% 下がったところ。そこから BL_DIM_LUX_WINDOW_MS 以内に戻るのを待つ）
static bool     s_wave_armed = false;
static uint32_t s_wave_ms    = 0;      // 30% 下がった時刻
static float    s_wave_from  = 0.0f;   // そのときの直近の最大（ログ用）
static float    s_wave_dip   = 0.0f;   // そこからのいちばん暗い読み

static void update_duty(float k);

static void lux_hand_reset() {
  s_lux_hist_n = 0;
  s_lux_hist_i = 0;
  s_lux_hand   = false;
  s_wave_armed = false;
}

// 新しく読めた照度を履歴へ入れ、手が来た形（settings.h の「かざす」「あおぐ」）なら s_lux_hand を立てる。
// 手をかざしたままだと、窓に明るい読みが残る約 1 秒の間は毎回立つ（戻すのは 1 回で、残りは
// 減光に入るのを延ばすだけなので困らない）。
static void lux_hand_feed(float lux, uint32_t now) {
  float ref = -1.0f;   // 直近 BL_DIM_LUX_WINDOW_MS の最大（いまの読みは入れない）
  for (uint8_t i = 0; i < s_lux_hist_n; i++)
    if (now - s_lux_hist[i].ms <= BL_DIM_LUX_WINDOW_MS && s_lux_hist[i].lux > ref) ref = s_lux_hist[i].lux;
  const bool fell = (ref >= 0.0f && ref - lux >= BL_DIM_LUX_DELTA_MIN);

  // かざす
  if (fell && lux <= ref * (1.0f - BL_DIM_LUX_DROP_RATIO)) {
    s_lux_hand = true;
    snprintf(s_lux_hand_why, sizeof(s_lux_hand_why), "drop %.0f->%.0f", ref, lux);
  }
  // あおぐ（30% 下がってから BL_DIM_LUX_WINDOW_MS 以内に、いちばん暗い読みから 30% 戻った）
  if (s_wave_armed && now - s_wave_ms > BL_DIM_LUX_WINDOW_MS) s_wave_armed = false;   // 戻らなかった（雲など）
  if (s_wave_armed) {
    if (lux - s_wave_dip >= BL_DIM_LUX_DELTA_MIN && lux >= s_wave_dip * (1.0f + BL_DIM_LUX_WAVE_RISE_RATIO)) {
      s_lux_hand = true;
      snprintf(s_lux_hand_why, sizeof(s_lux_hand_why), "wave %.0f->%.0f->%.0f", s_wave_from, s_wave_dip, lux);
      s_wave_armed = false;
    } else if (lux < s_wave_dip) {
      s_wave_dip = lux;
    }
  }
  if (!s_wave_armed && fell && lux <= ref * (1.0f - BL_DIM_LUX_WAVE_DROP_RATIO)) {
    s_wave_armed = true;
    s_wave_ms    = now;
    s_wave_from  = ref;
    s_wave_dip   = lux;
  }

  s_lux_hist[s_lux_hist_i] = {now, lux};
  s_lux_hist_i = (uint8_t)((s_lux_hist_i + 1) % LUX_HIST_N);
  if (s_lux_hist_n < LUX_HIST_N) s_lux_hist_n++;
}

static bool showing_own_state() {
  return !getReplayMode() && !link_mirror_active() && !is_demo_active();
}

bool backlight_is_dimmed() {
  return s_dimmed;
}

void backlight_notify_activity() {
  const uint32_t now = millis();
  s_dim_wait_t0 = now;
  if (s_dimmed) {
    s_dimmed = false;
    enqueueTask(createLogSdTask("BACKLIGHT: wake (manual activity)"));
    update_duty(1.0f);   // パッと瞬時に本来の明るさへ復帰
  }
}

// ---- PWM ----
static void pwm_write(float level) {
  if (fabsf(level - s_out) <= 0.001f) return;
  s_out = level;
  const uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
  const uint32_t top = pwm_hw->slice[slice].top;
  pwm_set_gpio_level(TFT_BL_PIN, (uint16_t)(level * (top + 1)));
}

// ---- 診断（一時的。2026-11-05 に追加。原因が分かったら外す）----
// ★ 約 100 回に 1 回、起動したあと輝度スイッチ Low でバックライトがまったく点かない
//   （High にすると最大で点く。Low に戻すとまた真っ暗。FPC 未接続なので Low は 5% のはず）。
//   log.txt の bl= は 5% のままなので、ファームの計算ではなく PWM の出力側の問題。
//   起動後と輝度スイッチを動かしたときに、PWM slice の設定と GPIO34 の状態を残して切り分ける。
//   pin_low は GPIO34 を約 2ms 読み続けて Low だった割合。Low で点灯なので、これが実際の点灯率。
//   正常なら Low で pin_low≒5%、csr=0x05（EN + A 反転）、div=0x010（1.0）、top=5999（150MHz / 25kHz - 1）、
//   ctr は 2 回で値が変わる（数えている）、ctrl の下位 5bit = 4（PWM）。
static bool s_diag_armed = false;   // 起動後の 1 回目を書いたか（それまでスイッチの診断も書かない）

static void log_pwm_diag(const char* why) {
  const uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
  const uint32_t csr  = pwm_hw->slice[slice].csr;
  const uint32_t div  = pwm_hw->slice[slice].div;
  const uint32_t top  = pwm_hw->slice[slice].top;
  const uint32_t cc   = pwm_hw->slice[slice].cc;
  const uint32_t ctr1 = pwm_hw->slice[slice].ctr;
  busy_wait_us_32(3);
  const uint32_t ctr2 = pwm_hw->slice[slice].ctr;
  const uint32_t en   = pwm_hw->en;
  const uint32_t ctrl = io_bank0_hw->io[TFT_BL_PIN].ctrl;
  const uint32_t pad  = pads_bank0_hw->io[TFT_BL_PIN];
  uint32_t n = 0, lows = 0;
  const uint32_t t0 = time_us_32();
  while (time_us_32() - t0 < 2000) {   // 25kHz の 50 周期ぶん
    if (!gpio_get(TFT_BL_PIN)) lows++;
    n++;
  }
  const unsigned long low_x10 = n ? (unsigned long)((uint64_t)lows * 1000u / n) : 0;
  // ★ 2 行に分けてある（log_sd() が 128 バイトで切っていたころの名残。いまは本文 256 バイトまで入る）。
  enqueueTask(createLogSdfTask(
      "BL DIAG %s: sw=%d duty=%d%% out=%d%% pin_low=%lu.%lu%% n=%lu top=%lu cc=0x%08lx",
      why, s_sw_bright ? 1 : 0, (int)lroundf(s_duty * 100.0f), (int)lroundf(s_out * 100.0f),
      low_x10 / 10, low_x10 % 10, (unsigned long)n, (unsigned long)top, (unsigned long)cc));
  enqueueTask(createLogSdfTask(
      "BL DIAG %s: slice=%u csr=0x%02lx div=0x%03lx ctr=%lu->%lu en=0x%03lx ctrl=0x%08lx pad=0x%03lx clk=%lu",
      why, slice, (unsigned long)csr, (unsigned long)div, (unsigned long)ctr1, (unsigned long)ctr2,
      (unsigned long)en, (unsigned long)ctrl, (unsigned long)pad, (unsigned long)clock_get_hz(clk_sys)));
}

// 点滅の消灯側にいるか。
static bool saver_dark(uint32_t now) {
  if (s_saver == BL_SAVER_OFF) return false;
  const bool crit = (s_saver == BL_SAVER_CRIT);
  const uint32_t on_ms  = crit ? BL_SAVER_CRIT_ON_MS  : BL_SAVER_ON_MS;
  const uint32_t off_ms = crit ? BL_SAVER_CRIT_OFF_MS : BL_SAVER_OFF_MS;
  return (now - s_saver_t0) % (on_ms + off_ms) >= on_ms;
}

// 本来の点灯率を決め、点滅の消灯側なら出力だけ 0 にする（上の ■ 電池低下のとき）。
static void pwm_apply(float duty) {
  s_duty = duty;
  pwm_write(saver_dark(millis()) ? 0.0f : duty);
}

// ---- VEML7700（レジスタは 16bit、下位バイトが先）----
static bool veml_write(uint8_t reg, uint16_t v) {
  myWire.beginTransmission(VEML7700_ADDR);
  myWire.write(reg);
  myWire.write(v & 0xFF);
  myWire.write(v >> 8);
  return myWire.endTransmission() == 0;
}

static bool veml_read(uint8_t reg, uint16_t& v) {
  myWire.beginTransmission(VEML7700_ADDR);
  myWire.write(reg);
  if (myWire.endTransmission(false) != 0) return false;   // 繰り返しスタートで読む
  if (myWire.requestFrom((uint8_t)VEML7700_ADDR, (uint8_t)2) != 2) return false;
  v  = myWire.read();
  v |= (uint16_t)myWire.read() << 8;
  return true;
}

static bool veml_init() {
  // まず 0 バイト書き込み（アドレスの確認だけ）で居るかを見る。MS5611 の復旧と同じ方法。
  // ★ いきなり veml_read() で探さないこと。繰り返しスタート用の書き込みが NACK で終わると、
  //   SDK が「次は繰り返しスタート」の状態を残し、直後の MS5611 の通信に持ち越す。
  //   カバーの無い機体では 10 秒ごとにこれが起きる。
  myWire.beginTransmission(VEML7700_ADDR);
  if (myWire.endTransmission() != 0) return false;
  uint16_t id;
  if (!veml_read(VEML_REG_ID, id) || (id & 0xFF) != VEML_ID_LOW) return false;
  // 見つけ直したときは暗い側（×1）から始める。明るければ 1 回目の読みで切り替わる。
  if (!veml_write(VEML_REG_CONF, VEML_CONF_HI)) return false;
  s_als_lo = false;
  s_als_t0 = s_als_check = millis();
  s_lux_fresh = false;   // 1 回読むまでは、前に読んだ値をログに出さない
  lux_hand_reset();      // 見失う前の読みと比べない
  return true;
}

static uint16_t veml_conf_now() { return s_als_lo ? VEML_CONF_LO : VEML_CONF_HI; }

// 測る範囲を切り替える。一度止めて（ALS_SD）から新しい設定で動かし、積分をやり直させる。
// 新しい積分が終わるまで（ALS_FIRST_READ_MS）は読まないので、その間 s_lux は前の値のまま。
// ★ 書き込めたときだけ s_als_lo を変える。先に変えると、書き込みに失敗したとき
//   チップは前の範囲のまま照度だけ 32 倍ずれて計算され、設定の確かめ（5 秒ごと）まで続く。
//   失敗したら次の読みでもう一度切り替えを試す（途中で止まっていたら設定の確かめが直す）。
static void veml_set_range(bool lo, uint32_t now) {
  const uint16_t conf = lo ? VEML_CONF_LO : VEML_CONF_HI;
  if (veml_write(VEML_REG_CONF, conf | VEML_ALS_SD) && veml_write(VEML_REG_CONF, conf))
    s_als_lo = lo;
  s_als_t0 = now;
}

// 対数で 2 点の間を結ぶ（lux は l0〜l1 の間にあること）。
static float log_lerp(float lux, float l0, float d0, float l1, float d1) {
  const float t = log10f(lux / l0) / log10f(l1 / l0);
  return d0 + (d1 - d0) * t;
}

// 照度 → 点灯率。明るさの感じ方は対数なので、照度の対数で直線（折れ線）にする。
// 曲線は設定画面の AUTO Brightness で選ぶ（値は settings.h の BL_AUTO_* / BL_ECO_*）。
static float auto_duty(float lux) {
  if (lux <= BL_AUTO_LUX_LOW) return BL_AUTO_DUTY_MIN;
  if (s_auto_mode == BL_AUTO_BRIGHT) {
    if (lux >= BL_AUTO_LUX_HIGH) return TFT_BL_DUTY_BRIGHT;
    return log_lerp(lux, BL_AUTO_LUX_LOW, BL_AUTO_DUTY_MIN, BL_AUTO_LUX_HIGH, TFT_BL_DUTY_BRIGHT);
  }
  // Eco
  if (lux >= BL_ECO_LUX_FULL) return TFT_BL_DUTY_BRIGHT;
  if (lux >= BL_ECO_LUX_MID)
    return log_lerp(lux, BL_ECO_LUX_MID, BL_ECO_DUTY_MID, BL_ECO_LUX_FULL, TFT_BL_DUTY_BRIGHT);
  return log_lerp(lux, BL_AUTO_LUX_LOW, BL_AUTO_DUTY_MIN, BL_ECO_LUX_MID, BL_ECO_DUTY_MID);
}

BacklightAutoMode backlight_get_auto_mode() { return (BacklightAutoMode)s_auto_mode; }

void backlight_set_auto_mode(BacklightAutoMode m) {
  if (m != BL_AUTO_ECO && m != BL_AUTO_BRIGHT) m = BL_AUTO_ECO;
  if (m == s_auto_mode) return;
  s_auto_mode    = m;
  s_auto_changed = true;
}

void backlight_setup() {
  pinMode(TFT_SW_PIN, INPUT);   // 外付けプルダウン（R48）。内部プルは使わない
  gpio_set_function(TFT_BL_PIN, GPIO_FUNC_PWM);
  const uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
  pwm_set_clkdiv(slice, 1.0f);
  pwm_set_wrap(slice, clock_get_hz(clk_sys) / TFT_BL_PWM_HZ - 1);
  // Low で点灯するので出力を反転する（レベル＝点灯している割合になる）。
  // 音声（PIN_PWMTONE）は別のスライスなので干渉しない。
  pwm_set_output_polarity(slice, (TFT_BL_PIN & 1) == 0, (TFT_BL_PIN & 1) == 1);
  s_sw_bright = digitalRead(TFT_SW_PIN);
  // 照度センサーはまだ使えない（i2c0 が未初期化）ので、スイッチだけで決める。
  pwm_apply(s_sw_bright ? TFT_BL_DUTY_BRIGHT : TFT_BL_DUTY_DIM);
  pwm_set_enabled(slice, true);
}

// 照度を読む。読めなくなっても s_lux は**最後の値のまま**にする
// （昼間に 5% へ落とすと画面が見えなくなる。センサーが元から無いときだけ TFT_BL_DUTY_DIM）。
static void read_als(uint32_t now) {
  if (s_als_ok) {
    uint16_t raw;
    if (now - s_als_t0 < ALS_FIRST_READ_MS) return;   // 1 回目の積分が終わるまでは 0 が返る
    // ★ 設定が残っているかを時々確かめる。カバーの接触不良などで VEML7700 だけが
    //   電源リセットされると、設定が初期値（シャットダウン）に戻る。通信は普通に
    //   成功するので lost にならず、照度 0 を読み続けて**昼間に最暗になる**。
    //   比べる相手は**いまの範囲の設定**（VEML_CONF_HI / LO）。
    if (now - s_als_check >= ALS_CONF_CHECK_MS) {
      s_als_check = now;
      uint16_t conf;
      if (veml_read(VEML_REG_CONF, conf) && conf != veml_conf_now()) {
        enqueueTask(createLogSdfTask("VEML7700 conf reset (0x%04X), reconfigured", conf));
        if (veml_write(VEML_REG_CONF, veml_conf_now())) s_als_t0 = now;
        s_lux_fresh = false;
        lux_hand_reset();   // 設定が消えていた間の読み（0 など）と比べない
        return;   // 新しい積分が終わるまで読まない
      }
    }
    if (veml_read(VEML_REG_ALS, raw)) {
      s_lux = raw * (s_als_lo ? VEML7700_LUX_PER_COUNT_LO : VEML7700_LUX_PER_COUNT);
      s_lux_fresh = true;
      lux_hand_feed(s_lux, now);   // 手が来たか（かざす・あおぐ）。上限なしの照度で見る
      // 範囲の切り替え（VEML_CONF_HI / LO の説明）。切り替えた回の値はそのまま使う
      // （×1 で飽和の手前なら約 2900 lx 以上なのは確か。次の読みで本当の値になる）。
      if (!s_als_lo && raw >= VEML_RANGE_DOWN_RAW)     veml_set_range(true, now);
      else if (s_als_lo && s_lux < VEML_RANGE_UP_LUX)  veml_set_range(false, now);
    } else {
      s_als_ok = false;
      s_lux_fresh = false;
      s_als_retry = now;
      enqueueTask(createLogSdTask("VEML7700 lost"));
    }
  } else if (now - s_als_retry >= ALS_RETRY_MS) {
    s_als_retry = now;
    if (veml_init()) {
      s_als_ok = true;
      enqueueTask(createLogSdTask("VEML7700 found"));
    }
  }
}

// 本来の点灯率と、それをどの決まりで決めたか（画面の表示用。backlight_mode_label）。
// ★ update_duty() と画面の表示で同じ分岐を使う。書き写すと「表示は AUTO なのに 5%」が起きる。
static float choose_target(bool sw, const char** why) {
  const char* w;
  float t;
  if (sw)                { w = "SW BRIGHT"; t = TFT_BL_DUTY_BRIGHT; }  // スイッチ High は何があっても常時最大（緊急バイパス）
  // ★ センサー無しは減光より先に見る。どちらも TFT_BL_DUTY_DIM で点灯率は同じだが、減光を先に見ると
  //   30 秒止まったところで表示が "IDLE DIM" に変わり、センサーが読めていないことが文字から消えた（2026-11-10 まで）。
  else if (s_lux < 0.0f) { w = "NO SENSOR"; t = TFT_BL_DUTY_DIM; }     // センサー無し
  else if (s_dimmed) {
    // 待機中自動減光（5%）。★ 暗い所（約 1.2〜1.3 lx 未満）では自動調光の方が暗いので、そちらを超えない。
    //   超えると「減光」に入った瞬間に明るくなる。
    w = "IDLE DIM";
    t = min(TFT_BL_DUTY_DIM, auto_duty(s_lux));
  }
  else                   { w = s_als_ok ? "AUTO" : "LAST LUX"; t = auto_duty(s_lux); }  // 通常自動調光（読めなくなったら最後の照度）
  if (why) *why = w;
  return t;
}

// 点灯率を決めて出す。k = 照度へ寄せる割合（1 = すぐに合わせる）。
static void update_duty(float k) {
  const bool sw = digitalRead(TFT_SW_PIN);
  const bool sw_changed = (sw != s_sw_bright);
  if (sw_changed) k = 1.0f;                    // スイッチを動かしたときはすぐに効かせる
  if (s_auto_changed) { s_auto_changed = false; k = 1.0f; }   // AUTO Brightness を切り替えたときも同じ
  s_sw_bright = sw;
  const float target = choose_target(sw, nullptr);

  const float duty = s_duty + (target - s_duty) * k;
  // 本来の点灯率が変わらなくても、点滅の切り替わりで出力は変わるので毎回通す
  // （同じ値なら pwm_write() が何もしない）。
  pwm_apply(duty);
  // 診断（上の log_pwm_diag）。新しい点灯率は次の周期の頭（40µs 以内）から効くので、
  // 2ms 読む pin_low にはほぼ新しい方が出る。
  if (sw_changed && s_diag_armed) log_pwm_diag(sw ? "sw high" : "sw low");
}

// 待機中の自動減光・復帰監視。
// ★ スイッチ High は緊急バイパスとして何があっても常時最大（減光しない）。
// ★ 減光中に何か一つでもトリガーを検知したらパッと通常輝度へ即座に復帰し、
//    タイマー起点を現在時刻にリセットして最低 BL_DIM_TIMEOUT_MS（30秒）は通常輝度を維持する。
static void dim_tick(uint32_t now) {
  // 照度で見た「手が来た」（read_als → lux_hand_feed）。ここで読んで消す。
  // ★ 下の途中の return を通っても、古い検知を次の回へ持ち越さないよう先頭で消す。
  const bool lux_hand = s_lux_hand;
  s_lux_hand = false;

  // 1) 輝度スイッチ High の監視（緊急バイパス）
  const bool sw_high = digitalRead(TFT_SW_PIN);
  if (sw_high) {
    if (s_dimmed) {
      s_dimmed = false;
      enqueueTask(createLogSdTask("BACKLIGHT: wake (switch HIGH)"));
      update_duty(1.0f);
    }
    s_dim_wait_t0 = now ? now : 1;
    return;
  }

  // 2) 操作ボタン（SW_PUSH）の押下検知
  if (digitalRead(SW_PUSH) == LOW) {
    s_dim_wait_t0 = now ? now : 1;
    if (s_dimmed) {
      s_dimmed = false;
      enqueueTask(createLogSdTask("BACKLIGHT: wake (button)"));
      update_duty(1.0f);
      return;
    }
  }

  // 2') 地図に通知の箱（GNSS 断線・無信号・送信前チェック・Scanning / Weak GNSS Signal など）が出ている間は
  //     減光しない。減光中に出たらすぐ戻す（2026-11-08）。それまでは暗いまま箱が出て、音でしか気づけなかった。
  //     判定は描画側の「いま箱を出したか」（display_tft.cpp の map_notice_shown）を読むだけで、条件を書き写さない。
  if (map_notice_shown()) {
    if (s_dimmed) {
      s_dimmed = false;
      enqueueTask(createLogSdTask("BACKLIGHT: wake (notice on map)"));
      update_duty(1.0f);
    }
    s_dim_wait_t0 = now ? now : 1;
    return;
  }

  // 3) システム前提条件（実機フライト待機中、USBなし、重大安全警告なし）
  const bool sys_ok = showing_own_state() &&
                      !usb_power_present() &&
                      !power_sw_off_on_usb() &&
                      !power_overdischarge_latched() &&
                      !power_fault_active() &&
                      !power_charge_hot_active();

  if (!sys_ok) {
    if (s_dimmed) {
      s_dimmed = false;
      enqueueTask(createLogSdTask("BACKLIGHT: wake (system state / USB)"));
      update_duty(1.0f);
    }
    s_dim_wait_t0 = now ? now : 1;
    return;
  }

  // 4) 減光中の復帰トリガー判定
  if (s_dimmed) {
    const char* wake_reason = nullptr;
    char reason_buf[48];

    // ① 対地速度の上昇（発進）
    double lat, lon, gs = 0.0, trk;
    const bool fix = get_gnss_connection() && gnss_get_own_fix(lat, lon, gs, trk);
    if (fix && gs >= BL_DIM_SPEED_EXIT) {
      snprintf(reason_buf, sizeof(reason_buf), "speed %.2fm/s", gs);
      wake_reason = reason_buf;
    }

    // ② IMU 振動（漕ぎ始め・ペダリング・機体運動）
    // ★ SCH16T 姿勢推定コア（attitude.cpp）の 250Hz ジャイロ＋比力静止判定を使用
    if (!wake_reason && get_imu_alive() && !attitude_is_static()) {
      wake_reason = "imu motion";
    }

    // ③ 照度で見た「手が来た」（かざす・あおぐ・人の影。lux_hand_feed）
    if (!wake_reason && lux_hand) {
      snprintf(reason_buf, sizeof(reason_buf), "lux %s", s_lux_hand_why);
      wake_reason = reason_buf;
    }

    if (wake_reason) {
      s_dimmed = false;
      s_dim_wait_t0 = now ? now : 1;   // ★ 復帰後は起点を現在時刻にリセットし最低30秒維持！
      enqueueTask(createLogSdfTask("BACKLIGHT: wake (%s)", wake_reason));
      update_duty(1.0f);    // パッと瞬時に復帰
    }
    return;
  }

  // 5) 減光突入の判定（通常表示中）
  static bool s_ever_flown = false;

  // 対地速度
  double lat, lon, gs = 0.0, trk;
  const bool fix = get_gnss_connection() && gnss_get_own_fix(lat, lon, gs, trk);
  if (fix && gs >= BL_DIM_SPEED_ENTER) {
    s_ever_flown = true;
  }

  // 飛行実績があった後は、GNSS が抜けても未測位を安易に静止とみなさない（フェイルセーフ）
  const bool speed_still = fix ? (gs < BL_DIM_SPEED_ENTER) : (!s_ever_flown);

  // IMU 振動（静止しているか）。IMU 異常時・非稼働時は静止とみなさない
  const bool imu_still = get_imu_alive() && attitude_is_static();

  // 照度。手が来た（かざした・あおいだ・人が影を落とした）ら、まだ誰かが触っているとみて待ち直す。
  // 戻す条件（上の ③）と同じ検知を使うので、「かざしたのに減光に入る」「戻ったのにすぐ減光」が起きない。
  // ゆっくりした変化（雲・夕暮れ）では待ち直さない。
  const bool lux_still = !lux_hand;

  const bool is_still = speed_still && imu_still && lux_still;

  if (!is_still) {
    s_dim_wait_t0 = now ? now : 1;
  } else {
    if (s_dim_wait_t0 == 0) s_dim_wait_t0 = now ? now : 1;
    if (now - s_dim_wait_t0 >= BL_DIM_TIMEOUT_MS) {
      s_dimmed = true;
      enqueueTask(createLogSdfTask("BACKLIGHT: enter auto-dim (gs=%.2f lux=%.0f)", gs, s_lux));
    }
  }
}

void backlight_set_saver(BacklightSaver mode) {
  if (mode != s_saver) {
    // 段階が変わったとき（OFF→LOW / LOW↔CRIT）は点灯の側から周期を張り直す。
    // 張り直さないと (now - s_saver_t0) % (on + off) の周期長が変わった瞬間に位相が飛ぶ。
    if (mode != BL_SAVER_OFF) s_saver_t0 = millis();
    s_saver = mode;
  }
  pwm_write(saver_dark(millis()) ? 0.0f : s_duty);
}

void backlight_als_setup() {
  s_als_ok = veml_init();
  s_als_retry = millis();
  enqueueTask(createLogSdTask(s_als_ok ? "VEML7700 OK" : "VEML7700 not found"));
  if (s_als_ok) {
    // 起動画面（約 10 秒）から照度で点けたいので、ここで 1 回読んでおく。
    // i2c0 の直後で、まだ IMU を立ち上げる前なので 150ms 待っても困らない。
    delay(ALS_FIRST_READ_MS);
    read_als(millis());
    update_duty(1.0f);
  }
}

void backlight_tick() {
  const uint32_t now = millis();
  // 照度は点灯率を決める間隔より細かく読む（ALS_READ_MS の説明。手の影を取りこぼさないため）。
  static uint32_t s_als_read_ms = 0;
  if (now - s_als_read_ms >= ALS_READ_MS) {
    s_als_read_ms = now;
    read_als(now);
  }
  const uint32_t dt  = now - s_last_ms;
  if (dt < BL_TICK_MS) return;
  s_last_ms = now;
  // 診断（上の log_pwm_diag）。最初の backlight_tick()（loop()、または setup() 後半の過放電の画面）で 1 回。
  // backlight_setup() では書かない（log.txt の 1 行目を "BOOT PONS" に保つため。setup() の該当コメント）。
  if (!s_diag_armed) {
    s_diag_armed = true;
    log_pwm_diag("boot");
  }
  dim_tick(now);
  // 照度はゆっくり追う（手の影や機体の向きでちらつかせない）。
  // 係数は経過時間から作る（呼ばれる頻度で時定数を変えない）。
  float k = (float)dt / (float)BL_AUTO_TAU_MS;
  if (k > 1.0f) k = 1.0f;
  update_duty(k);
}

// ログには**いま読めている値だけ**を出す。読めなくなった後も点灯率は最後の照度で
// 決めているが、それをログに書くと検証のときに「その照度だった」と誤解させる。
float backlight_get_lux()      { return s_lux_fresh ? s_lux : -1.0f; }
int   backlight_get_duty_pct() { return (int)lroundf(s_duty * 100.0f); }
bool  backlight_als_ok()       { return s_als_ok; }

BacklightSaver backlight_get_saver() { return s_saver; }

const char* backlight_mode_label() {
  const char* why;
  (void)choose_target(s_sw_bright, &why);   // 最後に読んだスイッチで（ここで GPIO を読み直さない）
  return why;
}

void backlight_format_status(char* buf, size_t n) {
  const float lux = backlight_get_lux();
  const char* sv = (s_saver == BL_SAVER_CRIT) ? " saver2"
                 : (s_saver == BL_SAVER_LOW)  ? " saver" : "";
  const char* dm = s_dimmed ? " dim" : "";
  // 曲線（AUTO Brightness）も残す。bl= と lux= の関係を後から読むときに、どちらの曲線か要るため。
  const char* am = (s_auto_mode == BL_AUTO_BRIGHT) ? " bright" : " eco";
  if (lux < 0.0f) snprintf(buf, n, " bl=%d%% lux=--%s%s%s", backlight_get_duty_pct(), am, sv, dm);
  else            snprintf(buf, n, " bl=%d%% lux=%.0f%s%s%s", backlight_get_duty_pct(), lux, am, sv, dm);
}

