// ============================================================
// File    : backlight.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : v7.1 の TFT バックライト。輝度スイッチと照度センサー（VEML7700）で
//           PWM の点灯率を決める。電池低下のときは点滅させて電池をもたせる。
//           v7 では何もしない（スイッチで直接切り替える）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/22
// ============================================================
//
// ■ 決め方（値は settings.h の TFT_BL_* / BL_AUTO_*）
//   輝度スイッチ High … 照度によらず最大
//   輝度スイッチ Low  … VEML7700 の照度で自動。センサーが無い・壊れたときは TFT_BL_DUTY_DIM
//
// ■ 電池低下のとき（backlight_set_saver。判定は GPS_TFT_map.ino の batt_alarm_tick）
//   BL_SAVER_ON_MS 点けて BL_SAVER_OFF_MS 消すをくり返す。点いている間は上の明るさのまま。
//   ★ 消している間も「本来の明るさ」（s_duty）は追い続ける。出力だけ 0 にする。
//     s_duty まで 0 にすると、点いた瞬間から照度へゆっくり寄せ直す（BL_AUTO_TAU_MS）ので、
//     2 秒の点灯がほとんど暗いまま終わる。
//
// ■ ハード
//   Q1（P-ch）のゲートを GPIO34 の PWM で駆動。**Low で点灯**（R47 でプルアップして
//   あるので、PWM を設定するまでは消灯）。輝度スイッチは GPIO35（外付けプルダウン R48）。
//   VEML7700（0x10）はケースカバー基板にあり、J8 経由で i2c0 に載る（MS5611 と共用）。
//   カバーを付けていない機体もあるので、居なくても動くようにしてある。
#include "backlight.h"
#include "settings.h"

#if PONS_BOARD == PONS_BOARD_V71
#include "airdata.h"   // myWire（i2c0）
#include "mysd.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define BL_TICK_MS         250UL
#define ALS_FIRST_READ_MS  150UL    // 設定してから 1 回目の積分（100ms）が終わるまで読まない
#define ALS_RETRY_MS     10000UL    // 居ないときに探しに行く間隔
#define ALS_CONF_CHECK_MS 5000UL    // 設定が残っているかを確かめる間隔（下の read_als）

#define VEML7700_ADDR   0x10
#define VEML_REG_CONF   0x00
#define VEML_REG_ALS    0x04
#define VEML_REG_ID     0x07
#define VEML_ID_LOW     0x81        // ID レジスタの下位バイト（デバイスコード）
// ゲイン ×1・積分 100ms・割り込みなし・動作中（ALS_SD=0）。全ビット 0 がちょうどこの設定。
// この設定で約 3800 lx が上限（それ以上は 65535 で飽和）だが、自動調光は
// BL_AUTO_LUX_HIGH で最大になるので困らない。
#define VEML_CONF       0x0000

static bool     s_sw_bright  = false;
static float    s_duty       = 0.0f;    // いま出している点灯率
static bool     s_als_ok     = false;
static uint32_t s_als_t0     = 0;       // 設定した時刻（ALS_FIRST_READ_MS の起点）
static uint32_t s_als_retry  = 0;
static uint32_t s_als_check  = 0;       // 設定を最後に確かめた時刻
static float    s_lux        = -1.0f;   // 点灯率に使う照度。-1 = 一度も読めていない
static bool     s_lux_fresh  = false;   // s_lux が今のセンサーから読めた値か（ログ用）
static uint32_t s_last_ms    = 0;
static bool     s_saver      = false;   // 電池低下で点滅中か
static uint32_t s_saver_t0   = 0;       // 点滅を始めた時刻（点灯から始める）
static float    s_out        = -1.0f;   // 実際に出している点灯率（点滅で 0 になる。-1 = まだ出していない）

// ---- PWM ----
static void pwm_write(float level) {
  if (fabsf(level - s_out) <= 0.001f) return;
  s_out = level;
  const uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
  const uint32_t top = pwm_hw->slice[slice].top;
  pwm_set_gpio_level(TFT_BL_PIN, (uint16_t)(level * (top + 1)));
}

// 点滅の消灯側にいるか。
static bool saver_dark(uint32_t now) {
  if (!s_saver) return false;
  return (now - s_saver_t0) % (BL_SAVER_ON_MS + BL_SAVER_OFF_MS) >= BL_SAVER_ON_MS;
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
  if (!veml_write(VEML_REG_CONF, VEML_CONF)) return false;
  s_als_t0 = s_als_check = millis();
  s_lux_fresh = false;   // 1 回読むまでは、前に読んだ値をログに出さない
  return true;
}

// 照度 → 点灯率。明るさの感じ方は対数なので、照度の対数で直線にする。
static float auto_duty(float lux) {
  if (lux <= BL_AUTO_LUX_LOW)  return BL_AUTO_DUTY_MIN;
  if (lux >= BL_AUTO_LUX_HIGH) return TFT_BL_DUTY_BRIGHT;
  const float t = log10f(lux / BL_AUTO_LUX_LOW) / log10f(BL_AUTO_LUX_HIGH / BL_AUTO_LUX_LOW);
  return BL_AUTO_DUTY_MIN + (TFT_BL_DUTY_BRIGHT - BL_AUTO_DUTY_MIN) * t;
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
    if (now - s_als_check >= ALS_CONF_CHECK_MS) {
      s_als_check = now;
      uint16_t conf;
      if (veml_read(VEML_REG_CONF, conf) && conf != VEML_CONF) {
        enqueueTask(createLogSdfTask("VEML7700 conf reset (0x%04X), reconfigured", conf));
        if (veml_write(VEML_REG_CONF, VEML_CONF)) s_als_t0 = now;
        s_lux_fresh = false;
        return;   // 新しい積分が終わるまで読まない
      }
    }
    if (veml_read(VEML_REG_ALS, raw)) {
      s_lux = raw * VEML7700_LUX_PER_COUNT;
      s_lux_fresh = true;
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

// 点灯率を決めて出す。k = 照度へ寄せる割合（1 = すぐに合わせる）。
static void update_duty(float k) {
  const bool sw = digitalRead(TFT_SW_PIN);
  if (sw != s_sw_bright) k = 1.0f;             // スイッチを動かしたときはすぐに効かせる
  s_sw_bright = sw;
  float duty;
  if (sw)                duty = TFT_BL_DUTY_BRIGHT;
  else if (s_lux < 0.0f) duty = TFT_BL_DUTY_DIM;   // センサー無し
  else                   duty = s_duty + (auto_duty(s_lux) - s_duty) * k;
  // 本来の点灯率が変わらなくても、点滅の切り替わりで出力は変わるので毎回通す
  // （同じ値なら pwm_write() が何もしない）。
  pwm_apply(duty);
}

void backlight_set_saver(bool on) {
  if (on && !s_saver) s_saver_t0 = millis();   // 点灯の側から始める
  s_saver = on;
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
  const uint32_t dt  = now - s_last_ms;
  if (dt < BL_TICK_MS) return;
  s_last_ms = now;
  read_als(now);
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

void backlight_format_status(char* buf, size_t n) {
  const float lux = backlight_get_lux();
  const char* sv = s_saver ? " saver" : "";
  if (lux < 0.0f) snprintf(buf, n, " bl=%d%% lux=--%s", backlight_get_duty_pct(), sv);
  else            snprintf(buf, n, " bl=%d%% lux=%.0f%s", backlight_get_duty_pct(), lux, sv);
}

#else  // v7: 輝度スイッチがバックライトの電源を直接切り替えるので、何もしない

void backlight_setup() {}
void backlight_als_setup() {}
void backlight_tick() {}
void backlight_set_saver(bool) {}
void backlight_format_status(char* buf, size_t n) { if (n) buf[0] = '\0'; }
float backlight_get_lux() { return -1.0f; }
int   backlight_get_duty_pct() { return -1; }

#endif
