// ============================================================
// File    : backlight.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : v7.1 の TFT バックライト。輝度スイッチと照度センサー（VEML7700）で
//           PWM の点灯率を決める。電池低下のときは 2 段階で点滅させて電池をもたせる。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/29
// ============================================================
//
// ■ 決め方（値は settings.h の TFT_BL_* / BL_AUTO_*）
//   輝度スイッチ High … 照度によらず最大
//   輝度スイッチ Low  … VEML7700 の照度で自動。センサーが無い・壊れたときは TFT_BL_DUTY_DIM
//
// ■ 電池低下のとき（backlight_set_saver。判定は GPS_TFT_map.ino の batt_alarm_tick）
//   第 1 段階（<=3.5V）は BL_SAVER_ON_MS 点けて BL_SAVER_OFF_MS 消す、
//   第 2 段階（<=3.0V）は BL_SAVER_ON_MS 点けて BL_SAVER_CRIT_OFF_MS 消すをくり返す。
//   点いている間は上の明るさのまま。
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

#include "airdata.h"   // myWire（i2c0）
#include "mysd.h"
#include "gnss.h"      // gnss_get_own_fix(), get_gnss_connection(), getReplayMode(), is_demo_active()
#include "imu.h"       // get_imu_alive()
#include "attitude.h"   // attitude_is_static()
#include "power.h"     // usb_power_present(), power_sw_off_on_usb(), power_fault_active(), power_charge_hot_active(), power_overdischarge_latched()
#include "link.h"      // link_mirror_active()
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
static uint32_t       s_last_ms  = 0;
static BacklightSaver s_saver    = BL_SAVER_OFF; // 電池低下で点滅中か（2 段階）
static uint32_t       s_saver_t0 = 0;            // 今の段階の点滅を始めた時刻（点灯から始める）
static float          s_out      = -1.0f;        // 実際に出している点灯率（点滅で 0 になる。-1 = まだ出していない）

// ---- 待機中自動減光（Auto-Dimming）ステート ----
static bool     s_dimmed       = false;    // 待機中自動減光中か
static uint32_t s_dim_wait_t0  = 0;        // 待機継続の起点時刻 [ms]
static float    s_lux_baseline = -1.0f;    // 静止待機中の基準照度 [lx]

static void update_duty(float k);

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
    s_lux_baseline = s_lux;
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

// 点滅の消灯側にいるか。
static bool saver_dark(uint32_t now) {
  if (s_saver == BL_SAVER_OFF) return false;
  const uint32_t off_ms = (s_saver == BL_SAVER_CRIT) ? BL_SAVER_CRIT_OFF_MS : BL_SAVER_OFF_MS;
  return (now - s_saver_t0) % (BL_SAVER_ON_MS + off_ms) >= BL_SAVER_ON_MS;
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
  float target;
  if (sw)                target = TFT_BL_DUTY_BRIGHT;  // スイッチ High は何があっても常時最大（緊急バイパス）
  else if (s_dimmed)     target = TFT_BL_DUTY_DIM;     // 待機中自動減光（5%）
  else if (s_lux < 0.0f) target = TFT_BL_DUTY_DIM;     // センサー無し
  else                   target = auto_duty(s_lux);    // 通常自動調光

  const float duty = s_duty + (target - s_duty) * k;
  // 本来の点灯率が変わらなくても、点滅の切り替わりで出力は変わるので毎回通す
  // （同じ値なら pwm_write() が何もしない）。
  pwm_apply(duty);
}

// 待機中の自動減光・復帰監視。
// ★ スイッチ High は緊急バイパスとして何があっても常時最大（減光しない）。
// ★ 減光中に何か一つでもトリガーを検知したらパッと通常輝度へ即座に復帰し、
//    タイマー起点を現在時刻にリセットして最低 BL_DIM_TIMEOUT_MS（30秒）は通常輝度を維持する。
static void dim_tick(uint32_t now) {
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
      s_lux_baseline = s_lux;
      enqueueTask(createLogSdTask("BACKLIGHT: wake (button)"));
      update_duty(1.0f);
      return;
    }
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
      s_lux_baseline = s_lux;
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

    // ③ 照度急変（手を仰ぐ）
    if (!wake_reason && s_als_ok && s_lux >= 0.0f && s_lux_baseline >= 0.0f) {
      const float d_lux = fabsf(s_lux - s_lux_baseline);
      const float thresh = max(BL_DIM_LUX_CHANGE_EXIT_MIN, s_lux_baseline * BL_DIM_LUX_CHANGE_EXIT_RATIO);
      if (d_lux >= thresh) {
        snprintf(reason_buf, sizeof(reason_buf), "lux change %.0f->%.0f", s_lux_baseline, s_lux);
        wake_reason = reason_buf;
      }
    }

    if (wake_reason) {
      s_dimmed = false;
      s_dim_wait_t0 = now ? now : 1;   // ★ 復帰後は起点を現在時刻にリセットし最低30秒維持！
      s_lux_baseline = s_lux;
      enqueueTask(createLogSdfTask("BACKLIGHT: wake (%s)", wake_reason));
      update_duty(1.0f);    // パッと瞬時に復帰
    } else {
      // ★ 減光中も環境光の自然なドリフト（雲の移動や夕暮れ）にベースラインを緩やかに追従させ、
      //    長時間の待機中に自然光変化で誤復帰するのを防ぐ
      if (s_als_ok && s_lux >= 0.0f) {
        if (s_lux_baseline < 0.0f) s_lux_baseline = s_lux;
        else s_lux_baseline += (s_lux - s_lux_baseline) * 0.05f;
      }
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

  // 照度安定
  bool lux_still = true;
  if (s_als_ok && s_lux >= 0.0f) {
    if (s_lux_baseline < 0.0f) {
      s_lux_baseline = s_lux;
    } else {
      const float d_lux = fabsf(s_lux - s_lux_baseline);
      const float thresh = max(BL_DIM_LUX_CHANGE_EXIT_MIN, s_lux_baseline * BL_DIM_LUX_CHANGE_EXIT_RATIO);
      if (d_lux >= thresh) {
        lux_still = false;
        s_lux_baseline = s_lux;  // 急変したので新ベースラインに更新
      } else {
        s_lux_baseline += (s_lux - s_lux_baseline) * 0.05f;  // 緩やかに追従
      }
    }
  }

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
  const uint32_t dt  = now - s_last_ms;
  if (dt < BL_TICK_MS) return;
  s_last_ms = now;
  read_als(now);
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

void backlight_format_status(char* buf, size_t n) {
  const float lux = backlight_get_lux();
  const char* sv = (s_saver == BL_SAVER_CRIT) ? " saver2"
                 : (s_saver == BL_SAVER_LOW)  ? " saver" : "";
  const char* dm = s_dimmed ? " dim" : "";
  if (lux < 0.0f) snprintf(buf, n, " bl=%d%% lux=--%s%s", backlight_get_duty_pct(), sv, dm);
  else            snprintf(buf, n, " bl=%d%% lux=%.0f%s%s", backlight_get_duty_pct(), lux, sv, dm);
}

