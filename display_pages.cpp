// ============================================================
// File    : display_pages.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 地上で見る詳細画面の描画（VARIO 1-2 / GNSS DETAIL 1-3 / CH SCAN）。
//           display_tft.cpp から分けただけで、役割は同じ「描画だけ」。
//           判断や発報は置かない（CLAUDE.md の約束）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/27
// ============================================================
//
// ★ **スプライトとフォントは display_tft.cpp のものを借りている**
//   （`backscreen` / `header_footer` / `tft`。宣言は display_tft.h）。
//   新しいスプライトをここで作らないこと — 画面は 1 枚しかないので、
//   同じ領域を 2 つのスプライトから push すると描き合いになる。
// ★ ここに置くのは「地上で見る画面」だけ。飛行中の地図・ヘッダ・フッタと
//   無線のオーバーレイは display_tft.cpp に残してある（飛行中に効くものを
//   1 ファイルにまとめておきたいため）。
//
#include <cstring>

#include "settings.h"
#include "display_tft.h"
#include "attitude.h"
#include "link.h"
#include "e220.h"
#include "gnss.h"
#include "mysd.h"
#include "navdata.h"
#include "src/flashdata/font_data.h"
#include "src/sound.h"
#include "src/button.h"
#include "airdata.h"
#include "imu.h"
#include "imu_sensor.h"
#include "src/vectormap.h"
#include "TFT_eSPI/CopySetupFile_TFT_eSPI.h"


// 衛星の方位角・仰角をスカイプロット（円形の衛星配置図）上の X 座標に変換する。
// 仰角 0°（水平）→ 外周、90°（天頂）→ 中心。方位角が X/Y の方向を決める。
static int skyplot_x(float azimuth, float elevation) {
  const int shift_left = 10;
  const int radius = SCREEN_WIDTH / 2 - shift_left;
  return (SCREEN_WIDTH / 2) + (int)(cos(radians(azimuth)) * (radius) * (1 - elevation / 90.0)) - shift_left;
}

// 衛星の方位角・仰角をスカイプロット上の Y 座標に変換する（skyplot_x の Y 版）。
static int skyplot_y(float azimuth, float elevation) {
  const int height = SCREEN_WIDTH;
  const int radius = SCREEN_WIDTH / 2 - 20;
  const int shift_down = 0;
  return (height / 2) - (int)(sin(radians(azimuth)) * (radius) * (1 - elevation / 90.0)) + shift_down;
}

// VARIO 1/2: センサー接続状況 + VSI 全比較 + MS5611 + GNSS 垂直。
// ★ 画面の用意（ヘッダ・backscreen・フォント）は draw_variodetail() が済ませている。
static void draw_vario_page1() {
  int y = 2;
  const int line_height = 12;
  const int nextcolumn_height = 14;
  // ---- Page 1: センサー接続状況 + VSI 全比較 + MS5611 + GNSS 垂直 ----

  // センサー接続状況
  // ★ ここだけは**この装置**のハードウェアの話（ミラー中も自機）。
  //   下の GNSS / KF / 気圧はミラーされるので、混同しないよう見出しで区別する。
  backscreen.setCursor(2, y);
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.print(link_mirror_active() ? "-- Sensor Status (RX unit) --"
                                        : "-- Sensor Status --");
  y += line_height;

  // MS5611 と BNO085 を 1 行にまとめる
  backscreen.setCursor(2, y);
  backscreen.setTextColor(get_airdata_ok() ? COLOR_GREEN : COLOR_RED, COLOR_WHITE);
  backscreen.printf("MS5611:%s", get_airdata_ok() ? "OK" : "NG");
  backscreen.setCursor(84, y);
  backscreen.setTextColor(get_imu_ok() ? COLOR_GREEN : COLOR_RED, COLOR_WHITE);
  backscreen.printf("%s:%s", imu_sensor_name(), get_imu_ok() ? "OK" : "NG");
  y += line_height;

  // GNSS fix 状態
  // ★ **fixtype はリンクに載っていない**（運んでいるのは fixOK / numsat / hAcc）。
  //   ミラー中に自機の fixtype で 3D 判定すると、**機体が NOFIX でもボートが
  //   3D なら「3D-OK」と出る**。分からないものは `--` にする。
  const bool _mirror = link_mirror_active();
  bool _gnssAny = get_gnss_fixok();
  bool _gnss3d  = _gnssAny && !_mirror && get_gnss_fixtype() >= 3;
  backscreen.setCursor(2, y);
  backscreen.setTextColor(_gnssAny ? (_gnss3d || _mirror ? COLOR_GREEN : COLOR_ORANGE)
                                   : COLOR_RED, COLOR_WHITE);
  if (_mirror)
    backscreen.printf("GNSS:%s fix=-- %dsats",
      _gnssAny ? "FIX" : "NOFIX", get_gnss_numsat());
  else
    backscreen.printf("GNSS:%s fix=%d %dsats",
      _gnss3d ? "3D-OK" : _gnssAny ? "2D" : "NOFIX",
      get_gnss_fixtype(), get_gnss_numsat());
  y += line_height; 

  // 動作モード（GNSS VSI 融合状況を含む）
  backscreen.setCursor(2, y);
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  if (_mirror) {
    // ★ 送信機がどのセンサーを融合しているかは電波から完全には分からない。
    //   分かるのは BNO085 の生死だけ（LINK_ST_IMU_ERROR）。MS5611 の状態を
    //   運ぶビットは無い（あっても電波の 32 バイトの枠を増やす価値は無い）。
    backscreen.printf("KF: sender (IMU %s)",
      (link_sender_status() & LINK_ST_IMU_ERROR) ? "NG" : "OK");
  }
  else if (get_imu_ok() && get_airdata_ok() && _gnss3d) backscreen.print("KF: Baro+IMU+GNSS");
  else if (get_imu_ok() && get_airdata_ok())              backscreen.print("KF: Baro+IMU");
  else if (get_airdata_ok() && _gnss3d)                   backscreen.print("KF: Baro+GNSS");
  else if (get_airdata_ok())                              backscreen.print("KF: Baro only");
  else if (get_imu_ok())                                  backscreen.printf("Mode: %s only", imu_sensor_name());
  else                                                     backscreen.print("Mode: No sensors");
  y += nextcolumn_height;

  // VSI 全比較（Baro / GNSS / KF）
  backscreen.setCursor(2, y);
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.print("-- VSI Comparison --");
  y += line_height;

  if (get_airdata_ok()) {
    float baro_vsi = get_airdata_vspeed();

    backscreen.setCursor(2, y);
    backscreen.setTextColor((baro_vsi > 0.1f) ? COLOR_GREEN : (baro_vsi < -0.1f) ? COLOR_RED : COLOR_BLACK, COLOR_WHITE);
    // ★ 気圧 V/S と気圧高度は**リンクに載っていない**（運んでいるのは気圧そのもの）。
    //   下の KF 行はミラーされるので、並べると機体の値に見える。(RX) を付ける。
    backscreen.printf(_mirror ? "Baro(RX):%+.2fm/s Alt:%.1fm"
                              : "Baro:%+.2fm/s  Alt:%.1fm",
                      baro_vsi, get_airdata_altitude());
    y += line_height;

    // GNSS VSI — sAcc ゲート状態を色で示す
    // ★ この行は**丸ごと自機の値**（veld も sAcc もリンクに載っていない）。
    //   上下の Baro / KF はミラーされるので、ミラー中に数字を出すと
    //   機体の GNSS 昇降率に見える。gnss_acc_unknown() のコメント参照。
    float _gnss_vsi = get_gnss_veld_mps();
    float _sacc_now = get_gnss_sacc_mmps() / 1000.0f;
    bool  _gate_ok  = _gnss3d && (_sacc_now < GNSS_VSI_SACC_MAX_MPS);
    backscreen.setCursor(2, y);
    backscreen.setTextColor(gnss_acc_unknown() ? COLOR_GRAY
                          : _gate_ok ? COLOR_BLACK : COLOR_GRAY, COLOR_WHITE);
    if (gnss_acc_unknown()) backscreen.print("GNSS:  --     (not on link)");
    else backscreen.printf("GNSS:%+.2fm/s %s", _gnss_vsi, _gate_ok ? "[KF]" : "[--]");
    y += line_height;

    // KF VSI（BNO085 なし時も Baro+GNSS KF から出力）
    float kf_vsi = get_imu_vspeed();
    backscreen.setCursor(2, y);
    backscreen.setTextColor((kf_vsi > 0.1f) ? COLOR_GREEN : (kf_vsi < -0.1f) ? COLOR_RED : COLOR_BLACK, COLOR_WHITE);
    backscreen.printf("KF  :%+.2fm/s", kf_vsi);
    y += nextcolumn_height;

    // 高度 KF まとめ（各センサーと KF MSL 出力を一覧表示）
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("-- Altitude KF (MSL) --");
    y += line_height;

    // 現在の気圧・気温 ＋ 起動時の基準気圧
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
    // ★ get_airdata_pressure() はミラーされるが気温はされない。**同じ行で
    //   出どころが混ざっていた**ので、ここは _raw（必ず自機）で揃える。
    //   機体の気圧は下の GNSS/KF 高度から読める。
    backscreen.printf(_mirror ? "Pres(RX):%.1fhPa Temp:%.1fC"
                              : "Pres:%.1fhPa Temp:%.1fC",
                      get_airdata_pressure_raw(), get_airdata_temperature());
    y += line_height;

    // 起動時の気圧（これを 0m 基準に使用）と起動地のISA絶対高度
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.printf("GND:%.1fhPa ISA:%.1fm", get_airdata_ground_pressure(), get_airdata_ground_altitude());
    y += line_height;

    // GNSS MSL 高度と KF MSL 高度（gnss_kf_offset 確定後は GNSS に収束）
    bool _msl_ready = get_imu_gnss_offset_ready();
    float _kf_msl   = get_imu_altitude_msl();
    float _gnss_msl = get_gnss_altitude();
    backscreen.setCursor(2, y);
    backscreen.setTextColor(_msl_ready ? COLOR_BLACK : COLOR_GRAY, COLOR_WHITE);
    backscreen.printf("GNSS:%.1fm  KF:%.1fm%s", _gnss_msl, _kf_msl, _msl_ready ? "" : "(-)");
    y += line_height;

    // B.S.L. = MSL - ELEVATION_GEOID_OFFSET_M（琵琶湖面 0m 基準）
    float _kf_bsl   = _kf_msl   - ELEVATION_GEOID_OFFSET_M;
    float _gnss_bsl = _gnss_msl - ELEVATION_GEOID_OFFSET_M;
    backscreen.setCursor(2, y);
    backscreen.setTextColor(_msl_ready ? COLOR_BLACK : COLOR_GRAY, COLOR_WHITE);
    backscreen.printf("BSL GNSS:%.1fm KF:%.1fm", _gnss_bsl, _kf_bsl);
    y += nextcolumn_height;

  } else {
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
    backscreen.print("MS5611: no data");
    y += nextcolumn_height;
  }

  // GNSS 垂直詳細
  backscreen.setCursor(2, y);
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.print("-- GNSS Vertical --");
  y += line_height;

  float vacc_m   = get_gnss_vacc_mm()   / 1000.0f;
  float sacc_mps = get_gnss_sacc_mmps() / 1000.0f;
  float gnss_alt = get_gnss_altitude();
  float r_vel    = sacc_mps * sacc_mps * GNSS_VSI_R_SCALE;
  if (r_vel < 0.001f) r_vel = 0.001f;  // 下限クランプ（imu.cpp と同じ）
  bool  _sacc_ok = sacc_mps < GNSS_VSI_SACC_MAX_MPS;

  backscreen.setCursor(2, y);
  if (gnss_acc_unknown()) {
    // ★ Alt はミラーされているのに sAcc/vAcc は送られてこない。並べると
    //   受信機自身の値が機体の値として読まれる（gnss_acc_unknown() 参照）。
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("sAcc/vAcc: -- (not on link)");
    y += line_height;
    backscreen.setCursor(2, y);
    backscreen.printf("R_vel: --      Alt:%.1fm(MSL)", gnss_alt);
  } else {
    backscreen.setTextColor(_sacc_ok ? (sacc_mps > GNSS_VSI_SACC_MAX_MPS * 0.7f ? COLOR_ORANGE : COLOR_BLACK)
                                     : COLOR_RED, COLOR_WHITE);
    backscreen.printf("sAcc:%.3fm/s(<%.1f)%s  vAcc:%.2fm",
      sacc_mps, GNSS_VSI_SACC_MAX_MPS, _sacc_ok ? "OK" : "NG", vacc_m);
    y += line_height;
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.printf("R_vel:%.4fm2  Alt:%.1fm(MSL)", r_vel, gnss_alt);
  }
  y += line_height;

  // ★ ミラー中は出さない。gnss_alt は機体、get_airdata_altitude() は自機なので
  //   **引き算そのものに意味が無い**（ボートと機体の高度差でもない）。
  if (get_airdata_ok() && !_mirror) {
    float dalt = gnss_alt - get_airdata_altitude();
    backscreen.setCursor(2, y);
    backscreen.setTextColor(fabsf(dalt) < 20.0f ? COLOR_BLACK : COLOR_ORANGE, COLOR_WHITE);
    backscreen.printf("AltDiff:%+.1fm(GNSS-Baro)", dalt);
    y += line_height;
  }

  backscreen.setCursor(2, y);
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  if (_mirror)   // fixtype はリンクに載っていない（上の GNSS fix 行と同じ理由）
    backscreen.printf("fix=-- gnssOK=%s sats=%d",
      get_gnss_fixok() ? "Y" : "N", get_gnss_numsat());
  else
    backscreen.printf("fix=%d gnssOK=%s sats=%d",
      get_gnss_fixtype(), get_gnss_fixok() ? "Y" : "N", get_gnss_numsat());

}

// VARIO 2/2: IMU と Kalman filter の状態。
static void draw_vario_page2() {
  int y = 2;
  const int line_height = 12;
  const int nextcolumn_height = 14;
  // ---- Page 2: BNO085 IMU データ + Kalman 設定 + GNSS VSI 融合 ----

  // ★ このページは**丸ごとこの装置の IMU と KF の内部**。ミラー中も自機の値を
  //   出す（KF MSL/VSI も _raw を使う）。地図画面が機体を出している流れで
  //   読むので、節見出しに (RX) を付けて区別する。
  // ★ **行を足して断るのはやめた。** このページは下端 224/240px まで使っており、
  //   1 行（12px）足すと最終行の "KF: Active" が切れる。見出しなら 0 行で済む。
  const bool _mirror2 = link_mirror_active();
  backscreen.setCursor(2, y);
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.printf(_mirror2 ? "-- %s IMU (RX) --" : "-- %s IMU --", imu_sensor_name());
  y += line_height;

  if (get_imu_ok()) {
    // イベント受信レート
    backscreen.setCursor(2, y);
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    // ★ 融合出力のレートは imu_caps() で確かめてから出す。無いのに出すと
    //   0.0Hz が並んで「飢餓している」と誤読されるので、行ごと差し替える。
    if (imu_caps() & IMU_CAP_QUAT)
      backscreen.printf("GRV:%.1fHz LACC:%.1fHz RV:%.1fHz",
                        get_imu_grv_hz(), get_imu_lacc_hz(), get_imu_rv_hz());
    else
      backscreen.printf("%s: no fusion output", imu_sensor_name());
    y += line_height;

    float az = get_imu_az();
    backscreen.setCursor(2, y);
    backscreen.setTextColor((az > 0.2f) ? COLOR_GREEN : (az < -0.2f) ? COLOR_RED : COLOR_BLACK, COLOR_WHITE);
    backscreen.printf("VertAccel:%+.3fm/s2", az);
    y += line_height;

    backscreen.setCursor(2, y);
    backscreen.setTextColor(get_imu_gnss_offset_ready() ? COLOR_BLACK : COLOR_GRAY, COLOR_WHITE);
    // ★ _raw（必ず自機）を使う。この節の他の行（GRV/LACC/RV・VertAccel・
    //   LinAccel）は自機しか出しようがないので、ここだけミラー値にすると
    //   **同じ枠の中で出どころが混ざる**。
    backscreen.printf("KF MSL:%.1fm  VSI:%+.2fm/s",
                      get_imu_altitude_msl_raw(), get_imu_vspeed_raw());
    y += nextcolumn_height;

    // 線形加速度（ボディフレーム）— チップ内で重力を引いた値（imu_caps で確かめる）。
    if (imu_caps() & IMU_CAP_LINACC) {
      float lax, lay, laz;
      get_imu_linaccel(lax, lay, laz);
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.print("-- LinAccel[body] --");
      y += line_height;
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("X:%+.3f Y:%+.3f Z:%+.3fm/s2", lax, lay, laz);
      y += line_height;
    }

    // Euler 角（Roll/Pitch: GAME RV、Yaw: RV+Mag）
    // ★ **これはチップ内の融合の出力**で、機上の推定には使っていない
    //   （0.983 で ESKF から切り離した）。ここは ESKF と突き合わせるための
    //   比較表示。融合が無いチップでは節ごと出さない。
    if (imu_caps() & IMU_CAP_QUAT) {
      float roll, pitch, yaw;
      get_imu_euler(roll, pitch, yaw);
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.print("-- Euler Angles --");
      y += line_height;
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
      backscreen.printf("R:%+.1f P:%+.1f (GAME RV)", roll, pitch);
      y += line_height;
      if (imu_caps() & IMU_CAP_MAGYAW) {
        const float mag_acc = get_imu_mag_accuracy_deg();
        backscreen.setCursor(2, y);
        if (mag_acc >= 0.0f) {
          backscreen.setTextColor(mag_acc < 5.0f ? COLOR_GREEN : mag_acc < 15.0f ? COLOR_ORANGE : COLOR_RED, COLOR_WHITE);
          backscreen.printf("Yaw:%+.1f(mag) acc:+/-%.1fdeg", yaw, mag_acc);
        } else {
          backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
          backscreen.printf("Yaw:%+.1f(game,no mag)", yaw);
        }
        y += line_height;
      }
      y += nextcolumn_height - line_height;
    }

  } else {
    backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
    backscreen.setCursor(2, y);
    backscreen.printf("%s not connected.", imu_sensor_name());
    y += line_height;
    if (get_airdata_ok()) {
      backscreen.setCursor(2, y);
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.printf("Baro VSI:%+.2fm/s (only)", get_airdata_vspeed());
      y += line_height;
    }
    y += 3;
  }

  // Kalman フィルター設定
  backscreen.setCursor(2, y);
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.print("-- Kalman Config --");
  y += line_height;

  backscreen.setCursor(2, y);
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
  backscreen.printf("Q_vel:%.4f  Q_bias:%.4f", get_imu_kf_q_vel(), get_imu_kf_q_bias());
  y += line_height;

  backscreen.setCursor(2, y);
  backscreen.printf("R_baro:%.4fm2", get_imu_kf_R());
  y += line_height;

  {
    float _ha   = get_imu_horiz_accel();
    float _qeff = get_imu_kf_q_vel() + KF_HORIZ_ACCEL_GAIN * _ha * _ha;
    backscreen.setCursor(2, y);
    // Q_vel_eff が基準値より大きいとき（水平加速中）はオレンジで強調
    backscreen.setTextColor(_ha > 1.0f ? COLOR_ORANGE : COLOR_BLACK, COLOR_WHITE);
    backscreen.printf("HorizA:%.2f gain:%.3f Q_eff:%.4f", _ha, KF_HORIZ_ACCEL_GAIN, _qeff);
    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
  }
  y += nextcolumn_height;

  // GNSS VSI KF セクション
  backscreen.setCursor(2, y);
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.print(_mirror2 ? "-- GNSS VSI Fusion (RX) --" : "-- GNSS VSI Fusion --");
  y += line_height;

  backscreen.setCursor(2, y);
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
  backscreen.printf("sAcc gate:<%.1fm/s  R_scale:%.1f", GNSS_VSI_SACC_MAX_MPS, GNSS_VSI_R_SCALE);
  y += line_height;

  float _sacc_mps = get_gnss_sacc_mmps() / 1000.0f;
  float _r_vel    = _sacc_mps * _sacc_mps * GNSS_VSI_R_SCALE;
  if (_r_vel < 0.001f) _r_vel = 0.001f;  // 下限クランプ（imu.cpp と同じ）

  backscreen.setCursor(2, y);
  // sAcc が閾値に近づくとオレンジ、超えると赤で警告
  backscreen.setTextColor(gnss_acc_unknown() ? COLOR_GRAY
                        : _sacc_mps < GNSS_VSI_SACC_MAX_MPS * 0.7f ? COLOR_BLACK
                        : _sacc_mps < GNSS_VSI_SACC_MAX_MPS        ? COLOR_ORANGE
                        : COLOR_RED, COLOR_WHITE);
  if (gnss_acc_unknown()) backscreen.print("sAcc: --      R_vel: --");
  else backscreen.printf("sAcc:%.3fm/s  R_vel:%.4fm2", _sacc_mps, _r_vel);
  y += line_height;

  bool _gnss3d2 = get_gnss_fixok() && get_gnss_fixtype() >= 3;
  bool _sacc_ok2 = (_sacc_mps < GNSS_VSI_SACC_MAX_MPS);
  bool _gate2    = _gnss3d2 && _sacc_ok2;
  backscreen.setCursor(2, y);
  if (gnss_acc_unknown()) {
    // ★ ミラー中は `loop()` が自機の KF へ GNSS を入れない（受信した機体の
    //   高度を入れると気圧基準と _gnss_kf_offset が壊れる。pons_link.md §5）。
    //   だから PASS/FAIL ではなく「入れていない」が正しい。
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print("Gate: -- (no GNSS into KF)");
  } else if (_gate2) {
    backscreen.setTextColor(COLOR_GREEN, COLOR_WHITE);
    backscreen.printf("Gate:PASS sAcc=%.3f", _sacc_mps);
  } else {
    backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
    if (!_gnss3d2) backscreen.print("Gate:FAIL no 3D fix");
    else           backscreen.printf("Gate:FAIL sAcc%.3f>=%.1f", _sacc_mps, GNSS_VSI_SACC_MAX_MPS);
  }
  y += line_height;

  backscreen.setCursor(2, y);
  // ★ ミラー中は自機 KF に GNSS が入っていないので false。上の Gate と揃える。
  bool _gnss3d_kf = !link_mirror_active() &&
                    get_gnss_fixok() && get_gnss_fixtype() >= 3;
  if (get_imu_ok() && get_airdata_ok() && _gnss3d_kf) {
    backscreen.setTextColor(COLOR_GREEN, COLOR_WHITE);
    backscreen.print("KF: Active (Baro+IMU+GNSS)");
  } else if (get_imu_ok() && get_airdata_ok()) {
    backscreen.setTextColor(COLOR_GREEN, COLOR_WHITE);
    backscreen.print("KF: Active (Baro+IMU)");
  } else if (get_airdata_ok() && _gnss3d_kf) {
    backscreen.setTextColor(COLOR_ORANGE, COLOR_WHITE);
    backscreen.print("KF: Active (Baro+GNSS)");
  } else if (get_airdata_ok()) {
    backscreen.setTextColor(COLOR_ORANGE, COLOR_WHITE);
    backscreen.print("KF: Active (Baro only)");
  } else {
    backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
    backscreen.print("KF: Inactive (no baro)");
  }
}

// ============================================================
// Vario 詳細画面を描画する（screen_mode == MODE_VARIODETAIL 時）。
// page % 2 == 0: センサー接続状況・VSI 全比較・MS5611 気圧データ・GNSS 垂直速度。
// page % 2 == 1: BNO085 IMU データ（線形加速度・Euler 角）・Kalman 設定・GNSS VSI 融合。
// ヘッダーは GNSS DETAIL 同様に 10px 上にずらして pushSprite(0,-10) で転送。
// VSI バーは GPS_TFT_map.ino のループ内で airdata_updated タイミングに更新される。
void draw_variodetail(int page) {
  // ---- ヘッダー: GNSS DETAIL と同様に 10px 上ずらし ----
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);

  const char* page_titles[] = {
    "VARIO 1:STATUS+VSI",
    "VARIO 2:IMU+KALMAN"
  };
  header_footer.setCursor(1, 11);  // +10px（GNSS DETAIL と同じオフセット）
  header_footer.print(page_titles[page % 2]);
  header_footer.pushSprite(0, -10);  // 10px 上にずらして底辺を y=39 に合わせる

  // ---- バックスクリーン（AA_FONT_SMALL を使用）----
  backscreen.fillScreen(COLOR_WHITE);
  backscreen.loadFont(AA_FONT_SMALL);
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);

  if (page % 2 == 0) draw_vario_page1();
  else               draw_vario_page2();

  backscreen.pushSprite(0, 40);

  // ---- フッター: ページ番号 ----
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.loadFont(AA_FONT_SMALL);
  header_footer.drawFastHLine(0, 0, SCREEN_WIDTH, COLOR_BLACK);
  header_footer.setTextColor(COLOR_GRAY, COLOR_WHITE);
  header_footer.setCursor(2, 5);
  header_footer.printf("Page %d/2  (short: next / long: back)", page % 2 + 1);
  header_footer.pushSprite(0, SCREEN_HEIGHT - 40);
}


// GNSS DETAIL 1: スカイプロット（全衛星の方位・仰角を円形に描く）。
//   - 衛星種別（GPS=シアン, GLONASS=緑, GALILEO=青, QZSS=赤, BeiDou=オレンジ）で色分け。
//   - SNR=0 の衛星は小さい丸（追跡中だが信号が弱い）。
//   - フッターに UTC 時刻と衛星数・フィックス状態。
// ★ 3 枚とも自分でヘッダ・本体・フッタを描く。共通なのは fillScreen() だけ。
static void draw_gnss_page1() {
  // ヘッダー（凡例）: y を +10 してから pushSprite(0,-10) で push
  // → ヘッダー底辺が screen y=39 になり backscreen(y=40) との重複10px が解消する
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);
  header_footer.setCursor(1, 11);  // +10
  header_footer.println("GNSS DETAIL 1:CONSTELLATION");
  header_footer.setTextColor(COLOR_CYAN,   COLOR_WHITE); header_footer.setCursor(0, 28); header_footer.print("GPS ");  // +10
  header_footer.setTextColor(COLOR_GREEN,  COLOR_WHITE); header_footer.print("GLO ");
  header_footer.setTextColor(COLOR_BLUE,   COLOR_WHITE); header_footer.print("GAL ");
  header_footer.setTextColor(COLOR_RED,    COLOR_WHITE); header_footer.print("QZS ");
  header_footer.setTextColor(COLOR_ORANGE, COLOR_WHITE); header_footer.print("BEI ");
  header_footer.pushSprite(0, -10);  // 10px 上にずらして底辺を y=39 に

  // フッター（UTC・衛星数）
  header_footer.fillSprite(COLOR_WHITE);
  header_footer.setCursor(23, 5);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  GnssDate date = get_gnss_date();
  GnssTime time = get_gnss_time();
  if (date.isValid() && time.isValid()) {
    header_footer.printf("%d.%d.%d %02d:%02d:%02d UTC", date.year(), date.month(), date.day(), time.hour(), time.minute(), time.second());
  }
  header_footer.setCursor(23, 17);
  header_footer.printf("%d sats, Fix=%s",get_gnss_numsat(),get_gnss_fix()?"yes":"no");
  header_footer.pushSprite(0,SCREEN_HEIGHT-40);

  backscreen.fillSprite(COLOR_BLACK);                         // 暗い背景
  backscreen.drawCircle(120, 120, 118, COLOR_GRAY);           // 外周円（地平線 0°）
  backscreen.drawCircle(120, 120, 79, COLOR_BRIGHTGRAY);      // 仰角 30° リング
  backscreen.drawCircle(120, 120, 39, COLOR_BRIGHTGRAY);      // 仰角 60° リング

  // N/E/S/W ラベルを外周に描画（N は赤）
  backscreen.setTextSize(1);
  backscreen.setTextColor(COLOR_RED,   COLOR_BLACK); backscreen.setCursor(116, 4);   backscreen.print("N");
  backscreen.setTextColor(COLOR_WHITE, COLOR_BLACK); backscreen.setCursor(116, 228); backscreen.print("S");
  backscreen.setTextColor(COLOR_WHITE, COLOR_BLACK); backscreen.setCursor(228, 116); backscreen.print("E");
  backscreen.setTextColor(COLOR_WHITE, COLOR_BLACK); backscreen.setCursor(4,   116); backscreen.print("W");


  for (int i = 0; i < MAX_SATELLITES; i++) {
    if (satellites[i].PRN == 0) continue;  // Skip empty entries
    //if(satellites[i].SNR == 0) continue; //Skip unknown signal strength.

    // Calculate position on display based on azimuth and elevation
    float azimuth = satellites[i].azimuth;
    float elevation = satellites[i].elevation;

    int x = skyplot_x(azimuth, elevation);
    int y = skyplot_y(azimuth, elevation);

    uint16_t color;
    if (satellites[i].satelliteType == SATELLITE_TYPE_QZSS)        color = COLOR_RED;
    else if (satellites[i].satelliteType == SATELLITE_TYPE_GPS)     color = COLOR_CYAN;
    else if (satellites[i].satelliteType == SATELLITE_TYPE_GLONASS) color = COLOR_GREEN;
    else if (satellites[i].satelliteType == SATELLITE_TYPE_GALILEO) color = COLOR_BLUE;
    else if (satellites[i].satelliteType == SATELLITE_TYPE_BEIDOU)  color = COLOR_ORANGE;
    else                                                             color = COLOR_GRAY;

    // SNR あり: 塗りつぶし円、SNR なし: 枠線のみ（信号未確認の衛星）
    if (satellites[i].SNR > 0) {
      backscreen.fillCircle(x, y, 5, color);
    } else {
      backscreen.drawCircle(x, y, 4, color);
    }

    int textx = constrain(x + 6, 0, SCREEN_WIDTH - 20);
    if (satellites[i].PRN >= 10)
      textx -= 10;
    if (satellites[i].PRN >= 100)
      textx -= 10;
    backscreen.setTextColor(COLOR_WHITE, COLOR_BLACK);
    backscreen.setCursor(textx, y - 3);
    backscreen.print(satellites[i].PRN);
  }
  backscreen.pushSprite(0,40);
}

// GNSS DETAIL 2: 最新 MAX_LAST_NMEA 件の UBX フレームラベル。
//   1 秒以内に受信したものは黒、古いものは灰色（鮮度の視覚化）。
static void draw_gnss_page2() {
  tft.fillRect(0, 20, 240, 320-20, COLOR_WHITE);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);
  header_footer.setCursor(1, 1);
  header_footer.println("GNSS DETAIL 2: UBX frames");
  header_footer.pushSprite(0,0);

  tft.unloadFont();
  tft.setTextSize(1);
  int posy = 0;
  tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
  for (int i = 0; i < MAX_LAST_NMEA; i++) {
    posy += 18;
    tft.setCursor(1, posy);
    if(get_gnss_nmea_time(i) < millis()-1000){
      tft.setTextColor(COLOR_GRAY);
    }else{
      tft.setTextColor(COLOR_BLACK);
    }
    tft.println(get_gnss_nmea(i));
  }
  tft.loadFont(AA_FONT_SMALL);  // Must load the font first
}

// GNSS DETAIL 3: GSA 情報（フィックス種別・DOP・緯度経度・使用衛星 PRN リスト）。
  // ---- page % 3 == 2: GSA 情報画面 ----------------------------------------
  // フィックス種別・DOP値・緯度経度・UTC時刻・測位使用衛星 PRN リストを表示する。
static void draw_gnss_page3() {
  tft.fillScreen(COLOR_WHITE);

  // ヘッダー
  header_footer.fillSprite(COLOR_WHITE);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);
  header_footer.setCursor(1, 1);
  header_footer.println("GNSS DETAIL 3: GSA INFO");
  header_footer.pushSprite(0, 0);

  tft.unloadFont();
  tft.setTextSize(1);
  int y = 22;

  // ★ ミラー中は出どころが混ざるページなので、1 行で断っておく。
  //   機体: 緯度経度・hAcc・fixOK・衛星数 ／ 自機: DOP・PRN・UTC・vAcc・sAcc。
  const bool _mirror = link_mirror_active();
  if (_mirror) {
    tft.setTextColor(COLOR_ORANGE, COLOR_WHITE);
    tft.setCursor(1, y);
    tft.print("MIRROR: pos/hAcc=sender, rest=RX unit");
    y += 12;
  }

  // ---- Fix Status ----
  // ★ **fixtype はリンクに載っていない。** ミラー中に自機の fixtype を
  //   この大きさで出すと、**ボートの測位状態を機体のものとして読む**。
  //   運んでいる fixOK だけで決める（2D/3D の区別は付かない）。
  int fixtype = get_gnss_fixtype();
  const char* fixstr;
  uint16_t fixcolor;
  if (_mirror)           { const bool _ok = get_gnss_fixok();
                           fixstr = _ok ? "Fix" : "No Fix";
                           fixcolor = _ok ? COLOR_GREEN : COLOR_RED; }
  else if (fixtype == 3) { fixstr = "3D Fix"; fixcolor = COLOR_GREEN; }
  else if (fixtype == 2) { fixstr = "2D Fix"; fixcolor = COLOR_ORANGE; }
  else                   { fixstr = "No Fix"; fixcolor = COLOR_RED; }

  tft.setTextSize(2);
  tft.setTextColor(fixcolor, COLOR_WHITE);
  tft.setCursor(1, y);
  tft.printf("Fix: %s", fixstr);
  y += 20;

  // ---- 衛星使用数 ----
  tft.setTextSize(1);
  tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
  tft.setCursor(1, y);
  tft.printf("Sats used: %d / total: %d", get_gsa_numsat(), get_gnss_numsat());
  y += 14;

  // ---- DOP ----
  // DOP 値: 1未満=非常に良好(緑), 2未満=良好(黒), 5未満=やや悪い(オレンジ), 5以上=悪い(赤)
  auto dopColor = [](float d) -> uint16_t {
    if (d < 1.0f) return COLOR_GREEN;
    if (d < 2.0f) return COLOR_BLACK;
    if (d < 5.0f) return COLOR_ORANGE;
    return COLOR_RED;
  };
  float pdop = get_gnss_pdop();
  float hdop = get_gnss_hdop();
  float vdop = get_gnss_vdop();

  tft.setCursor(1, y);
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  tft.print("PDOP:");
  tft.setTextColor(dopColor(pdop), COLOR_WHITE);
  tft.printf("%.2f  ", pdop);
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  tft.print("HDOP:");
  tft.setTextColor(dopColor(hdop), COLOR_WHITE);
  tft.printf("%.2f", hdop);
  y += 14;

  tft.setCursor(1, y);
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  tft.print("VDOP:");
  tft.setTextColor(dopColor(vdop), COLOR_WHITE);
  tft.printf("%.2f", vdop);
  y += 18;

  // ---- 緯度・経度 ----
  tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
  tft.setCursor(1, y);
  tft.printf("Lat: %.6f", get_gnss_lat());
  y += 12;
  tft.setCursor(1, y);
  tft.printf("Lon: %.6f", get_gnss_lon());
  y += 18;

  // ---- UTC 時刻・日付 ----
  GnssDate date = get_gnss_date();
  GnssTime time = get_gnss_time();
  tft.setCursor(1, y);
  if (date.isValid() && time.isValid()) {
    tft.printf("UTC %04d-%02d-%02d  %02d:%02d:%02d",
      date.year(), date.month(), date.day(),
      time.hour(), time.minute(), time.second());
  } else {
    tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
    tft.print("UTC: --");
  }
  y += 18;

  // ---- GNSS ボーレート ----
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  tft.setCursor(1, y);
  tft.printf("Baud: %lu bps", get_gnss_baudrate());
  y += 14;

  // ---- 測位使用衛星 PRN リスト ----
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  tft.setCursor(1, y);
  tft.print("PRNs in fix:");
  y += 12;
  tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
  tft.setCursor(1, y);
  int col = 0;
  for (int i = 0; i < 12; i++) {
    int prn = get_gsa_prn(i);
    if (prn == 0) continue;
    if (col > 0) tft.print(" ");
    tft.printf("%3d", prn);
    col++;
    if (col == 8) { y += 12; tft.setCursor(1, y); col = 0; }
  }

  // ---- UBX NAV-PVT 精度情報 ----------------------------------------
  // gnssFixOK・hAcc・vAcc・sAcc を色分け表示する。
  // 閾値: hAcc <5m=緑 / <25m=橙 / ≥25m=赤、vAcc <10m=緑 / <50m=橙 / ≥50m=赤
  //        sAcc <1m/s=緑 / <3m/s=橙 / ≥3m/s=赤
  y += 16;
  tft.drawFastHLine(0, y - 4, SCREEN_WIDTH, COLOR_GRAY);  // 区切り線

  // gnssFixOK
  tft.setCursor(1, y);
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  tft.print("gnssFixOK:");
  bool fixOK = get_gnss_fixok();
  tft.setTextColor(fixOK ? COLOR_GREEN : COLOR_RED, COLOR_WHITE);
  tft.print(fixOK ? "YES" : "NO");
  y += 12;

  // hAcc / vAcc を横並び表示
  float hacc_m = get_gnss_hacc_mm() / 1000.0f;
  float vacc_m = get_gnss_vacc_mm() / 1000.0f;
  uint16_t haccColor = (hacc_m < 5.0f)  ? COLOR_GREEN : (hacc_m < 25.0f) ? COLOR_ORANGE : COLOR_RED;
  uint16_t vaccColor = (vacc_m < 10.0f) ? COLOR_GREEN : (vacc_m < 50.0f) ? COLOR_ORANGE : COLOR_RED;

  tft.setCursor(1, y);
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  tft.print("hAcc:");
  tft.setTextColor(haccColor, COLOR_WHITE);
  tft.printf("%.1fm  ", hacc_m);
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  tft.print("vAcc:");
  // ★ hAcc はリンクで運んでいるが vAcc は運んでいない。ミラー中にここへ
  //   受信機自身の vAcc を出すと、隣の hAcc と同じ機体の値に見える
  //   （gnss_acc_unknown() のコメント参照）。
  if (gnss_acc_unknown()) { tft.setTextColor(COLOR_GRAY, COLOR_WHITE); tft.print("--"); }
  else { tft.setTextColor(vaccColor, COLOR_WHITE); tft.printf("%.1fm", vacc_m); }
  y += 12;

  // sAcc（速度精度）
  float sacc_mps = get_gnss_sacc_mmps() / 1000.0f;
  uint16_t saccColor = (sacc_mps < 1.0f) ? COLOR_GREEN : (sacc_mps < 3.0f) ? COLOR_ORANGE : COLOR_RED;
  tft.setCursor(1, y);
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  tft.print("sAcc:");
  if (gnss_acc_unknown()) { tft.print("-- (not on link)"); }
  else {
    tft.setTextColor(saccColor, COLOR_WHITE);
    tft.printf("%.2fm/s (%.1fkm/h)", sacc_mps, sacc_mps * 3.6f);
  }
  y += 14;

  // ---- AssistNow Autonomous（GNSS が自分で予測した衛星軌道） ----
  // ON/NG は GNSS 自身の申告（NAV-AOPSTATUS）で決める。設定を送っただけでは ON と出さない。
  // data = 予測データを持つ衛星数、use = いま予測軌道を使っている衛星数、
  // calc = 予測を計算中（この間は GNSS の消費電流が増える）。
  // ミラー中もこの行は自機の GNSS の話（ページ先頭の "rest=RX unit" に含まれる）。
  tft.setCursor(1, y);
  tft.setTextColor(COLOR_GRAY, COLOR_WHITE);
  tft.print("ANA:");
  const int ana = get_gnss_ana_state();
  if (!GNSS_USE_ANA)  { tft.print("OFF"); }
  else if (ana < 0)   { tft.print("--"); }
  else if (ana == 0)  { tft.setTextColor(COLOR_RED, COLOR_WHITE);   tft.print("NG"); }
  else                { tft.setTextColor(COLOR_GREEN, COLOR_WHITE); tft.print("ON"); }
  tft.setTextColor(COLOR_BLACK, COLOR_WHITE);
  tft.printf("  data:%d  use:%d", get_gnss_ana_data_svs(), get_gnss_ana_orbit_svs());
  if (get_gnss_ana_busy()) {
    tft.setTextColor(COLOR_ORANGE, COLOR_WHITE);
    tft.print("  calc");
  }
}

// GNSS 詳細画面（screen_mode == MODE_GNSSDETAIL）。中身は各ページ関数のコメント。
void draw_gnssdetail(int page) {
  header_footer.fillScreen(COLOR_WHITE);
  // ページ番号は画面の見出しに合わせる（page%3: 0→1 枚目 / 1→2 枚目 / 2→3 枚目）。
  switch (page % 3) {
    case 0:  draw_gnss_page1(); break;
    case 1:  draw_gnss_page2(); break;
    default: draw_gnss_page3(); break;
  }
}

// ============================================================
//  チャンネル調査の一覧（screen_mode == MODE_CHSCAN、link.cpp の link_chscan_*）
//    選べるチャンネル（CH0 / 3 / 6 / 9 / 12）の雑音と、解読できた PONS を並べる。
//    その下に、アッパーバンドの CH15 / 21 / 27 を区切って並べる（SF11 のときはアッパーでは使えないので灰色）。
//    帯域ごとに一番静かなものに best を付ける（大会当日にローワーが混んでいたら、アッパーと比べて選ぶ）。
//    判断は link.cpp（link_chscan_best*）にあり、ここは並べて色を付けるだけ。
//    ★ チャンネルはここでは変えない。3 台そろえる必要があるので、WIRELESS 画面の Ch で人が選ぶ。
// ============================================================
void draw_chscan() {
  header_footer.fillScreen(COLOR_WHITE);
  header_footer.setTextColor(COLOR_BLACK, COLOR_WHITE);
  header_footer.setTextSize(2);
  header_footer.setCursor(1, 11);
  header_footer.print("CH SCAN");
  header_footer.pushSprite(0, -10);

  backscreen.fillScreen(COLOR_WHITE);
  backscreen.loadFont(AA_FONT_SMALL);
  backscreen.setTextWrap(false);
  const int lh = 14;
  int y = 4;

  // ---- 状態 ----
  const LinkScanState st = link_chscan_state();
  backscreen.setCursor(2, y);
  if (st == LINK_SCAN_RUNNING) {
    backscreen.setTextColor(COLOR_BLUE, COLOR_WHITE);
    backscreen.printf(" Scanning  sweep %u/%u", (unsigned)link_chscan_sweep(), (unsigned)LINK_SCAN_SWEEPS);
    y += lh;
    // 調べている間は送信も受信も止まっている。送信機ならボートが無信号になる
    backscreen.setTextColor(COLOR_ORANGE, COLOR_WHITE);
    backscreen.setCursor(2, y);
    const uint8_t m = link_get_mode();
    backscreen.print(m == LINK_MODE_TX ? " TX paused while scanning"
                   : m == LINK_MODE_RX ? " RX paused while scanning"
                                       : " Radio busy while scanning");
  } else if (st == LINK_SCAN_DONE) {
    backscreen.setTextColor(COLOR_GREEN, COLOR_WHITE);
    backscreen.printf(" DONE  (%u sweeps, SF%u)", (unsigned)link_chscan_sweep(),
                      e220_profile_to_sf(link_get_radio_profile()));
  } else {
    backscreen.setTextColor(COLOR_ORANGE, COLOR_WHITE);
    backscreen.printf(" STOPPED  (sweep %u)", (unsigned)link_chscan_sweep());
  }
  y += lh + 4;

  // ---- 一覧 ----
  // 列: 今の CH の印・CH・周波数・雑音の中央値・最大値・メモ
  static const int cx[6] = { 2, 12, 34, 86, 126, 166 };
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.setCursor(cx[1], y); backscreen.print("CH");
  backscreen.setCursor(cx[2], y); backscreen.print("MHz");
  backscreen.setCursor(cx[3], y); backscreen.print("med");
  backscreen.setCursor(cx[4], y); backscreen.print("max");
  backscreen.setCursor(cx[5], y); backscreen.print("note");
  y += lh;

  const int8_t  best    = link_chscan_best();
  const int8_t  best_lo = link_chscan_best_band(false);
  const int8_t  best_up = link_chscan_best_band(true);
  const uint8_t cur     = link_get_radio_ch();
  for (uint8_t i = 0; i < LINK_SCAN_CH_COUNT; i++) {
    LinkScanRow r;
    link_chscan_row(i, &r);
    // ローワーとアッパーの境目に区切りの行（SF11 ならアッパーは使えないと書く）
    if (i == LINK_SCAN_LOWER_COUNT) {
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.setCursor(cx[1], y);
      backscreen.print(r.usable ? "-- upper band --" : "-- upper band (no SF11) --");
      y += lh;
    }
    // 今使っている CH には印。測っている CH は青。いまの SF で使えない CH は灰色。
    backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
    if (r.ch == cur) { backscreen.setCursor(cx[0], y); backscreen.print(">"); }
    backscreen.setTextColor(r.measuring ? COLOR_BLUE : (r.usable ? COLOR_BLACK : COLOR_GRAY), COLOR_WHITE);
    backscreen.setCursor(cx[1], y); backscreen.printf("%u", r.ch);
    backscreen.setCursor(cx[2], y); backscreen.printf("%.1f", 920.8f + 0.2f * r.ch);
    if (r.n == 0) {
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.setCursor(cx[3], y);
      backscreen.print(r.measuring ? "..." : "--");
      y += lh;
      continue;
    }
    // 中央値は送信前チェックと同じしきい値（LINK_PF_NOISE_WARN_DBM）を超えたら橙
    backscreen.setTextColor(r.med > LINK_PF_NOISE_WARN_DBM ? COLOR_ORANGE : COLOR_BLACK, COLOR_WHITE);
    backscreen.setCursor(cx[3], y); backscreen.printf("%d", r.med);
    backscreen.setTextColor(r.burst ? COLOR_ORANGE : COLOR_BLACK, COLOR_WHITE);
    backscreen.setCursor(cx[4], y); backscreen.printf("%d", r.max);
    // メモ: 他チームの PONS（赤）> 自チームの送信機（灰）> 断続的な電波（橙）> 帯域で一番静か（緑）
    backscreen.setCursor(cx[5], y);
    if (r.pons_other > 0) {
      backscreen.setTextColor(COLOR_RED, COLOR_WHITE);
      backscreen.print("PONS");
    } else if (r.pons_same > 0) {
      backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
      backscreen.print("own TX");
    } else if (r.burst) {
      backscreen.setTextColor(COLOR_ORANGE, COLOR_WHITE);
      backscreen.print("busy");
    } else if ((int8_t)i == best_lo || (int8_t)i == best_up) {
      backscreen.setTextColor(COLOR_GREEN, COLOR_WHITE);
      backscreen.print("best");
    }
    y += lh;
  }
  y += 4;

  // ---- まとめ ----
  backscreen.setCursor(2, y);
  if (best >= 0) {
    LinkScanRow b;
    link_chscan_row((uint8_t)best, &b);
    backscreen.setTextColor(COLOR_GREEN, COLOR_WHITE);
    backscreen.printf(" Quietest: CH%u (%d dBm)%s", b.ch, b.med, b.upper ? " UP" : "");
  } else {
    backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
    backscreen.print(" Quietest: --");
  }
  y += lh;
  backscreen.setTextColor(COLOR_BLACK, COLOR_WHITE);
  backscreen.setCursor(2, y);
  backscreen.print(" Set Ch on WIRELESS (all 3)");
  y += lh + 4;
  backscreen.setTextColor(COLOR_GRAY, COLOR_WHITE);
  backscreen.setCursor(2, y);
  backscreen.print(st == LINK_SCAN_RUNNING ? " short: stop   long: back"
                                           : " short: rescan  long: back");

  backscreen.unloadFont();
  // draw_wireless() と同じく自前で置く（push_backscreen は地図専用）。フッタも消す。
  backscreen.pushSprite(0, 40);
  tft.fillRect(0, SCREEN_HEIGHT - 40, SCREEN_WIDTH, 40, COLOR_WHITE);
}
