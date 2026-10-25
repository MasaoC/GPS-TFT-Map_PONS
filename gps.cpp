// ============================================================
// File    : gps.cpp
// Project : PONS v5 (Pilot Oriented Navigation System for HPA)
// Role    : GPS受信・解析の実装（Quectel LC86G・NMEA 受信）。
//           NMEA（RMC/GGA は TinyGPS++、GSA/GSV/GST/PAIR001 は自前）の解析、
//           衛星情報収集、位置・速度・時刻の取得、リプレイ管理、
//           地点選択式のデモ飛行（琵琶湖/白浜/笠岡/富士川/東京湾）、
//           フライトログCSVへの定期保存トリガー。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
// Handle GNSS modules. Currently optimized for LC86GPAMD.
//
// ■ v6 との違い（2026-10 に v6 の 0.950v6 から移植したとき）
//   v6 は u-blox SAM-M10Q の UBX（NAV-PVT）を読む。v5 は Quectel LC86G なので NMEA のまま。
//   画面・判断の側が使う値（get_gps_*）は v6 と同じ名前で出し、受信の層だけを入れ替えた。
//   NMEA に無いもの（速度精度 sAcc・上下方向の速度・測位エポックの iTOW）は持たない。
//   水平精度 hAcc は GST の緯度・経度の標準偏差から作る（GST が来ていなければ「分からない」）。
//   ★ 測位は 1Hz（このチップの仕様。v6 は 2Hz）。
#include <Arduino.h>

#include "gps.h"
#include "mysd.h"
#include "navdata.h"
#include "settings.h"
#include "display_tft.h"

// TinyGPS++: RMC / GGA の緯度・経度・速度・方位・高度・衛星数・HDOP を読む。
// ★ 「有効か」の判定には使わない。TinyGPS++ は欄が空でも commit すると isValid() が
//   true になり（起動直後の時刻 00:00:00 など）、一度 true になると測位を失っても戻らない。
//   測位の有無・時刻と日付の有無は、下の nmea_handle_line() が欄を見て自分で決める。
TinyGPSPlus gps;

// GPS時刻を最後に受信したときのmillis()（時刻推定用）
static uint32_t gps_fix_millis = 0;
uint32_t get_gps_fix_millis() { return gps_fix_millis; }

// GPS の UTC 日時から、JST の日付を YYYYMMDD の整数で返す。0 = 時刻がまだ無い（v7 の 0.970）。
// ★ UTC の日付では使えない。夜明けの飛行（05:00 JST）は 前日 20:00 UTC なので、日付を取り違える。
uint32_t get_gps_jst_yyyymmdd() {
  GpsDate d = get_gpsdate();
  GpsTime t = get_gpstime();
  if (!d.isValid() || !t.isValid()) return 0;
  int y = d.year(), mo = d.month(), da = d.day(), h = t.hour();
  utcToJst(&y, &mo, &da, &h);
  return (uint32_t)y * 10000UL + (uint32_t)mo * 100UL + (uint32_t)da;
}

// --- LC86G の設定コマンド（PAIR）---
// チェックサムは送るときに計算する（send_pair()）。手で書いた "*XX" の打ち間違いで
// 黙って無視されるのを防ぐため。応答は $PAIR001,<コマンド番号>,<結果>（0 = 成功）。
//   PAIR062,<種別>,<間隔>   : NMEA の出力（種別 2=GSA 3=GSV 8=GST、間隔 0=止める 1=毎測位）
//   PAIR490,<0|1>          : EASY（軌道の自己予測。u-blox の AssistNow Autonomous に当たる）
//   PAIR864,0,0,<baud>     : ボーレート（モジュールの再起動が要る）
#define PAIR_RESULT_OK          0
#define PAIR_RESULT_NONE      (-1)   // 応答が来なかった
static int16_t pair_last_cmd    = -1;   // 直近に受けた $PAIR001 のコマンド番号（PAIR062 なら 62）
static int8_t  pair_last_result = PAIR_RESULT_NONE;
static int    easy_result      = PAIR_RESULT_NONE;  // 起動時の PAIR490 の結果（GPS DETAIL 用）
static int    gst_result       = PAIR_RESULT_NONE;  // 起動時の PAIR062,8 の結果（GST が出せるか）
int get_gps_easy_result() { return easy_result; }
uint32_t gps_current_baudrate = 0;  // GPS シリアルの現在ボーレート（gps_setup() で更新）
int get_gps_gst_result()  { return gst_result; }

// --- GPS 最新値の保持変数 ---
// NMEA（リプレイ中は CSV）から取り出した値をここに保存し、getter 経由で公開する。
double stored_longitude, stored_latitude, stored_truetrack, stored_gnss_altitude, stored_fixtype, stored_gs;
int stored_numsats;

// --- 最大 G/S 保持変数 ---
// maxgs        : 起動後の全時間最大 G/S [m/s]
// maxgs_5min   : 直近 5 分間の最大 G/S [m/s]（5分間更新がなければ現在値にリセット）
// *_hour / *_min : 最大値を記録した時刻（JST 時・分）
static float  maxgs            = 0.0f;
static int    maxgs_hour       = 0;
static int    maxgs_min        = 0;
static float  maxgs_5min       = 0.0f;
static int    maxgs_5min_hour  = 0;
static int    maxgs_5min_min   = 0;
static unsigned long maxgs_5min_last_update = 0;  // 5分保持の最終更新時刻 [ms]
bool gps_connection = false;  // GPS モジュールから1文字でも受信したことがあるか
// ★ 最後にバイトが届いた時刻。**飛行中の断線を検出するために要る**（v7 の 0.979）。
//   gps_connection だけだと一度 true になったら戻らないので、コネクタが抜けても
//   画面は正常なまま航法を続けてしまっていた（get_gps_connection 参照）。
static uint32_t gps_last_byte_ms = 0;
// デモ飛行の現在地点（DEMO_OFF でデモ停止）。GPS を使わず仮想位置を生成する。
demo_site_t demo_site = DEMO_OFF;
double demo_lat = PLA_LAT;        // 仮想機体の現在位置
double demo_lon = PLA_LON;
double demo_mps = 0;              // 仮想対地速度 [m/s]
double demo_truetrack = 280;      // 仮想真方位 [deg]

// --- 衛星情報・受信 NMEA ログ ---
SatelliteData satellites[MAX_SATELLITES];  // 最大 MAX_SATELLITES 衛星分のデータを保持（PRN 0 = 空き）
char last_nmea[MAX_LAST_NMEA][NMEA_MAX_CHAR];   // 直近の受信 NMEA のリングバッファ（GPS詳細画面用）
unsigned long last_nmea_time[MAX_LAST_NMEA];     // 各行の受信時刻（受信途絶の監視にも使う）
unsigned long last_gps_setup_time = 0;   // 最後に gps_setup() を呼んだ時刻（再接続ガード用）
int stored_nmea_index = 0;               // リングバッファの書き込み位置
bool new_location_arrived = false;       // 新しい位置情報が届いたことを Core0 に知らせるフラグ
bool newcourse_arrived = false;          // 新しいコース情報が届いたフラグ（毎秒更新）
static int readfail_counter = 0;         // ASCII でない文字の数（ボーレート違いの検出。gps_loop）

// --- 測位品質（DOP・フィックスタイプ・使用衛星 PRN）---
// GSA の受信時に更新する。LC86G は衛星系ごとに GSA を出す（$GNGSA が複数行）ので、
// 使用中の PRN は 1 エポックぶんを集めてから入れ替える（nmea_epoch で区切る）。
#define GSA_MAX_PRN 12
static int    gsa_fixtype = 1;                // 1=No Fix, 2=2D, 3=3D
static float  gsa_pdop    = 0.0f;             // Position DOP
static float  gsa_hdop    = 0.0f;             // Horizontal DOP
static float  gsa_vdop    = 0.0f;             // Vertical DOP
static int    gsa_prns[GSA_MAX_PRN] = {};     // 測位に使用中の衛星 PRN（0=未使用）
static int    gsa_numsat  = 0;                // 測位使用衛星数（GGA の衛星数）
static uint32_t gsa_last_ms = 0;              // 最後に GSA を受けた時刻（0 = まだ）

// --- GST（水平精度）---
// GST の緯度・経度の 1σ [m] から水平精度を作る（sqrt(σlat² + σlon²)）。
// u-blox の hAcc と同じく「だいたいこのくらい外れうる」の目安。
static uint32_t gst_hacc_mm  = 0;
static uint32_t gst_last_ms  = 0;             // 最後に GST を受けた時刻（0 = まだ）
// これより古い GST / GSA は使わない [ms]（1Hz なので 3 エポックぶん）
#define NMEA_FRESH_MS  3000UL

// --- 1 エポックぶんの測位（RMC で締める）---
// LC86G は 1 秒ごとに GGA → GSA … → RMC の順で出す（v5 の 0.893 で確認済みの順序。RMC が最後）。
// RMC を受けたところで 1 エポック分が揃ったとみなし、gps_loop() が反映する。
static uint32_t nmea_epoch        = 0;      // RMC を受けるたびに 1 進む
static int      gga_quality       = 0;      // GGA の測位品質（0 = 測位なし）
static bool     rmc_status_ok     = false;  // RMC のステータス 'A'
static bool     rmc_time_present  = false;  // RMC の時刻欄が空でない
static bool     rmc_date_present  = false;  // RMC の日付欄が空でない
static bool     rmc_course_present = false; // RMC の対地方位欄が空でない（止まっていると空になる）
static bool     nmea_epoch_ready  = false;  // gps_loop() に知らせる（RMC を受けた）

// 測位の結果（NMEA かリプレイの CSV が書く）。get_gpsdate()/get_gpstime() とログ保存が読む。
static bool     g_time_valid  = false;  // UTC 時刻が有効か
static bool     g_date_valid  = false;  // UTC 日付が有効か
static uint16_t g_year        = 0;
static uint8_t  g_month       = 0;
static uint8_t  g_day         = 0;
static uint8_t  g_hour        = 0;
static uint8_t  g_min         = 0;
static uint8_t  g_sec         = 0;
static uint8_t  g_cs          = 0;      // センチ秒
static bool     g_pos_valid   = false;  // 有効な 2D/3D 測位があるか
static bool     g_gnssFixOK   = false;  // 測位があり、品質も OK（RMC 'A' かつ GGA 品質 > 0）

void removeStaleSatellites();

// ============================================================
// NMEA の行の解析（自前で読む文）
// ============================================================
// 1 行のチェックサムを確かめる。$ と * の間の XOR が * の後ろの 16 進 2 桁と一致すれば true。
// ★ 自前で読む文はこれを通してから使う。TinyGPS++ は自分で確かめているが、
//   こちらは化けた行を読むと衛星表示や精度が一瞬おかしな値になる。
static int hex_digit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}
static bool nmea_checksum_ok(const char* s) {
  if (s[0] != '$') return false;
  uint8_t x = 0;
  const char* p = s + 1;
  while (*p && *p != '*') x ^= (uint8_t)*p++;
  if (*p != '*') return false;
  const int hi = hex_digit(p[1]), lo = hex_digit(p[2]);
  if (hi < 0 || lo < 0) return false;
  return x == (uint8_t)(hi * 16 + lo);
}

// カンマ区切りの n 番目の欄（0 = 文の名前）の先頭を返す。無ければ nullptr。
// 欄の終わりは ',' か '*' か '\0'。
static const char* nmea_field(const char* s, int n) {
  const char* p = s;
  for (int i = 0; i < n; i++) {
    p = strchr(p, ',');
    if (p == nullptr) return nullptr;
    p++;
  }
  return p;
}
static bool nmea_field_empty(const char* f) {
  return f == nullptr || *f == ',' || *f == '*' || *f == '\0';
}

// 文の種類（"RMC" など）か。talker（GN / GP / GL / GA / GB / GQ ...）は問わない。
static bool nmea_is(const char* s, const char* type) {
  return s[0] == '$' && strlen(s) > 6 && strncmp(s + 3, type, 3) == 0 && s[6] == ',';
}

// GSV（Satellites in View）を手で解析して satellites[] 配列に衛星情報を格納する。
// 例: $GPGSV,4,4,14,194,,,,195,,,*7D
static void parseGSV(const char *nmea) {
  // 衛星系は talker で分かる。QZSS は LC86G では $GPGSV に混ざって出る（PRN 193〜199）。
  int satelliteType = SATELLITE_TYPE_UNKNOWN;
  if      (strncmp(nmea, "$GP", 3) == 0) satelliteType = SATELLITE_TYPE_GPS;
  else if (strncmp(nmea, "$GL", 3) == 0) satelliteType = SATELLITE_TYPE_GLONASS;
  else if (strncmp(nmea, "$GA", 3) == 0) satelliteType = SATELLITE_TYPE_GALILEO;
  else if (strncmp(nmea, "$GB", 3) == 0) satelliteType = SATELLITE_TYPE_BEIDOU;
  else if (strncmp(nmea, "$GQ", 3) == 0) satelliteType = SATELLITE_TYPE_QZSS;

  // 先頭の 4 欄（名前・総文数・文番号・衛星数）を飛ばす
  const char *p = nmea_field(nmea, 4);
  if (p == nullptr) return;

  for (int i = 0; i < 4; i++) {
    if (*p == '*' || *p == '\0') break; // End of sentence or no more data
    int prn = atoi(p);
    p = strchr(p, ','); if (!p) break; p++;
    int elevation = (*p != ',' && *p != '*') ? atoi(p) : -1;
    p = strchr(p, ','); if (!p) break; p++;
    int azimuth = (*p != ',' && *p != '*') ? atoi(p) : -1;
    p = strchr(p, ','); if (!p) break; p++;
    int snr = (*p != ',' && *p != '*') ? atoi(p) : -1;
    // 最後の衛星の SNR の後ろは ',' ではなく '*' や信号 ID の欄が来る。
    const char* q = strchr(p, ',');
    p = (q != nullptr) ? q + 1 : p + strlen(p);

    if (prn > 0 && prn < 200) {
      int type = satelliteType;
      if (193 <= prn && prn <= 199) type = SATELLITE_TYPE_QZSS;
      bool stored = false;
      for (int j = 0; j < MAX_SATELLITES; j++) {
        if ((satellites[j].PRN == prn && satellites[j].satelliteType == type) || satellites[j].PRN == 0) {
          satellites[j].PRN = prn;
          satellites[j].elevation = (elevation >= 0 && elevation <= 90) ? elevation : satellites[j].elevation;
          satellites[j].azimuth = (azimuth >= 0 && azimuth < 360) ? azimuth : satellites[j].azimuth;
          satellites[j].SNR = (snr >= 0) ? snr : satellites[j].SNR;
          satellites[j].satelliteType = type;
          satellites[j].lastReceived = millis();
          stored = true;
          break;
        }
      }
      if (!stored) {
        // 衛星ごとにログを積むとタスクキューを溢れさせて他のタスクを落とす。10 秒に 1 回に制限する。
        static unsigned long last_satfull_log = 0;
        if (millis() - last_satfull_log > 10000) {
          last_satfull_log = millis();
          enqueueTask(createLogSdfTask("WARN:sat full prn=%d type=%d", prn, type));
        }
      }
    }
  }
}

// GSA: 2D/3D、DOP、測位に使っている衛星の PRN。
//   $GNGSA,A,3,03,04,...,(12 個),PDOP,HDOP,VDOP,系ID*cs
static void parseGSA(const char* s) {
  const char* f = nmea_field(s, 2);
  if (nmea_field_empty(f)) return;
  const int mode = atoi(f);
  if (mode >= 1 && mode <= 3) gsa_fixtype = mode;

  // 使用中の PRN は 1 エポックぶんを集めてから入れ替える（衛星系ごとに 1 行ずつ来るため）。
  static uint32_t epoch_seen = 0xFFFFFFFFUL;
  static int      idx = 0;
  if (epoch_seen != nmea_epoch) {
    epoch_seen = nmea_epoch;
    idx = 0;
    for (int i = 0; i < GSA_MAX_PRN; i++) gsa_prns[i] = 0;
  }
  for (int k = 0; k < 12; k++) {
    const char* pf = nmea_field(s, 3 + k);
    if (nmea_field_empty(pf)) continue;
    const int prn = atoi(pf);
    if (prn > 0 && idx < GSA_MAX_PRN) gsa_prns[idx++] = prn;
  }
  const char* pd = nmea_field(s, 15);
  const char* hd = nmea_field(s, 16);
  const char* vd = nmea_field(s, 17);
  if (!nmea_field_empty(pd)) gsa_pdop = atof(pd);
  if (!nmea_field_empty(hd)) gsa_hdop = atof(hd);
  if (!nmea_field_empty(vd)) gsa_vdop = atof(vd);
  gsa_last_ms = millis();
}

// GST: 緯度・経度の 1σ [m] から水平精度を作る。
//   $GNGST,時刻,RMS,長軸,短軸,向き,σ緯度,σ経度,σ高度*cs
static void parseGST(const char* s) {
  const char* fl = nmea_field(s, 6);
  const char* fo = nmea_field(s, 7);
  if (nmea_field_empty(fl) || nmea_field_empty(fo)) return;
  const float sl = atof(fl), so = atof(fo);
  if (!(sl >= 0.0f && so >= 0.0f && sl < 10000.0f && so < 10000.0f)) return;
  gst_hacc_mm = (uint32_t)(sqrtf(sl * sl + so * so) * 1000.0f);
  gst_last_ms = millis();
}

// ---- 設定の送り直し ----
// ★ 起動時の設定（gps_setup() の 1 回目）は、次の 2 つの場合に入らないまま残る。
//   ・モジュールの立ち上がりが本体より遅く、送ったコマンドを取りこぼした
//   ・途中で通信が切れ、再接続でボーレートを切り替えた（PAIR864 でモジュールが再起動し、
//     PAIR062 の出力設定は再起動で消える）
//   そのままだと GSA・GST が出ず、2D/3D と水平精度が「推す」「分からない」のまま飛ぶ。
//   そこで、そうなったときは**正しい NMEA が届き始めた時点で**設定を送り直す（応答は待たない）。
static bool cfg_resend_pending = false;
static bool gsv_enabled        = false;  // 衛星表示の画面で GSV を出しているか
static void send_pair(const char* body);
static void send_output_config() {
  send_pair("PAIR062,2,1");                          // GSA を出す
  send_pair(gsv_enabled ? "PAIR062,3,1" : "PAIR062,3,0");
  send_pair("PAIR062,8,1");                          // GST を出す
  send_pair(GNSS_USE_EASY ? "PAIR490,1" : "PAIR490,0");
}

// 1 行を受け終わったときに呼ぶ（TinyGPS++ はこの行を既に処理し終えている）。
static void nmea_handle_line(const char* s) {
  if (!nmea_checksum_ok(s)) return;
  readfail_counter = 0;   // 正しい行が来ている＝ボーレートは合っている
  if (cfg_resend_pending && !getReplayMode()) {
    cfg_resend_pending = false;
    send_output_config();
    enqueueTask(createLogSdfTask("GPS CFG resent (baud=%lu)", (unsigned long)gps_current_baudrate));
  }

  if (nmea_is(s, "GGA")) {
    const char* q = nmea_field(s, 6);
    gga_quality = nmea_field_empty(q) ? 0 : atoi(q);
  } else if (nmea_is(s, "RMC")) {
    const char* t  = nmea_field(s, 1);
    const char* st = nmea_field(s, 2);
    const char* d  = nmea_field(s, 9);
    rmc_time_present = !nmea_field_empty(t);
    rmc_status_ok    = (st != nullptr && *st == 'A');
    rmc_date_present = !nmea_field_empty(d);
    rmc_course_present = !nmea_field_empty(nmea_field(s, 8));
    nmea_epoch++;
    nmea_epoch_ready = true;
  } else if (nmea_is(s, "GSA")) {
    parseGSA(s);
  } else if (nmea_is(s, "GSV")) {
    parseGSV(s);
  } else if (nmea_is(s, "GST")) {
    parseGST(s);
  } else if (strncmp(s, "$PAIR001,", 9) == 0) {
    const char* c = nmea_field(s, 1);
    const char* r = nmea_field(s, 2);
    if (!nmea_field_empty(c) && !nmea_field_empty(r)) {
      pair_last_cmd    = (int16_t)atoi(c);
      pair_last_result = (int8_t)atoi(r);
      // EASY の結果は送り直したときの応答でも更新する（GPS DETAIL 3 の表示）。
      if (pair_last_cmd == 490) easy_result = pair_last_result;
    }
  }
}

// NMEA の 1 文字を処理する。TinyGPS++ に渡しつつ、行に組み立てて自前の解析と
// GPS 詳細画面の受信ログ（last_nmea）に回す。
static char nmea_line[NMEA_MAX_CHAR];
static int  nmea_line_len = 0;
static void process_char(char c) {
  gps.encode(c);
  if (c == '$') nmea_line_len = 0;
  if (c == '\r') return;
  if (c == '\n' || nmea_line_len >= NMEA_MAX_CHAR - 1) {
    if (nmea_line_len >= 2) {
      nmea_line[nmea_line_len] = '\0';
      strncpy(last_nmea[stored_nmea_index], nmea_line, NMEA_MAX_CHAR - 1);
      last_nmea[stored_nmea_index][NMEA_MAX_CHAR - 1] = '\0';
      last_nmea_time[stored_nmea_index] = millis();
      stored_nmea_index = (stored_nmea_index + 1) % MAX_LAST_NMEA;
      nmea_handle_line(nmea_line);
      #ifdef DEBUG_NMEA
      Serial.println(nmea_line);
      #endif
    }
    nmea_line_len = 0;
    return;
  }
  nmea_line[nmea_line_len++] = c;
}

// RMC でエポックが締まったときに、そのエポックの値を stored_* / g_* に反映する。
// v6 の handle_navpvt() に当たる。
static void apply_nmea_epoch() {
  // --- 日時 ---
  g_date_valid = false;
  if (rmc_date_present && gps.date.isValid()) {
    const uint16_t y = gps.date.year();
    const uint8_t mo = gps.date.month(), d = gps.date.day();
    // 月・日は上限も確認する。月は days_in_month[] の添字に使われるため、
    // 13 以上が通ると配列外読み出しになる。2020 年以前は未取得とみなす。
    if (y >= 2020 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31) {
      g_year = y; g_month = mo; g_day = d;
      g_date_valid = true;
    }
  }
  g_time_valid = rmc_time_present && gps.time.isValid();
  if (g_time_valid) {
    g_hour = gps.time.hour();
    g_min  = gps.time.minute();
    g_sec  = gps.time.second();
    g_cs   = gps.time.centisecond();
  }
  gps_fix_millis = millis();

  // --- 測位の有無 ---
  // GSA が来ていればその 2D/3D を使う。来ていない（GSA を止められている）ときは
  // 測位の有無から 1 / 3 を推す（GGA の品質が付いていれば 3D とみなす）。
  const bool fix = rmc_status_ok && gga_quality > 0;
  if (gsa_last_ms == 0 || millis() - gsa_last_ms > NMEA_FRESH_MS) gsa_fixtype = fix ? 3 : 1;
  if (!fix) gsa_fixtype = 1;

  static int prev_fixtype = 1;  // 前回の fixtype（遷移検出用）
  if (prev_fixtype <= 1 && gsa_fixtype >= 2) {
    // 起動後最初の fix だけ、起動からの秒数を添える。GPS は本体と同じ電源で入るので、
    // **電源を入れて起動したときはこれが TTFF**（EASY が効けば数秒、効かなければ数十秒）。
    char hacc[16];
    if (get_gps_hacc_valid()) snprintf(hacc, sizeof(hacc), "%.1fm", gst_hacc_mm / 1000.0f);
    else                      snprintf(hacc, sizeof(hacc), "-");
    static bool first_fix_logged = false;
    if (!first_fix_logged) {
      first_fix_logged = true;
      enqueueTask(createLogSdfTask("GPS FIXED fix=%d sats=%d hAcc=%s HDOP=%.1f %.1fs after boot",
        gsa_fixtype, gps.satellites.value(), hacc, gps.hdop.hdop(), millis() / 1000.0f));
    } else {
      enqueueTask(createLogSdfTask("GPS FIXED fix=%d sats=%d hAcc=%s HDOP=%.1f",
        gsa_fixtype, gps.satellites.value(), hacc, gps.hdop.hdop()));
    }
  }
  prev_fixtype = gsa_fixtype;

  g_pos_valid = fix;
  g_gnssFixOK = fix && gsa_fixtype >= 2;

  if (g_pos_valid) {
    stored_latitude      = gps.location.lat();
    stored_longitude     = gps.location.lng();
    if (gps.altitude.isValid()) stored_gnss_altitude = gps.altitude.meters();
    stored_gs            = gps.speed.mps();
    // ★ 方位の欄が空なら前の方位を保つ。NMEA の RMC は止まっていると方位を空で出すことがあり、
    //   TinyGPS++ はそれを 0 度として読む。そのまま使うと TRACK UP の地図が北向きへ飛ぶ。
    if (rmc_course_present) {
      double trk         = gps.course.deg();
      if (trk <   0) trk += 360.0;
      if (trk >= 360) trk -= 360.0;
      stored_truetrack   = trk;
    }
  }
  stored_numsats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
  gsa_numsat     = stored_numsats;
  if (gps.hdop.isValid()) gsa_hdop = gps.hdop.hdop();
  stored_fixtype = g_pos_valid ? 2 : 0;

  // 10 秒に 1 回、精度を SD ログに残す（衛星 0 = 明らかに屋内の場合はスキップ）
  static unsigned long last_acc_log = 0;
  if (millis() - last_acc_log >= 10000) {
    last_acc_log = millis();
    if (stored_numsats > 0) {
      char hacc[16];
      if (get_gps_hacc_valid()) snprintf(hacc, sizeof(hacc), "%.1fm", gst_hacc_mm / 1000.0f);
      else                      snprintf(hacc, sizeof(hacc), "-");
      enqueueTask(createLogSdfTask("ACC hAcc=%s HDOP=%.1f PDOP=%.1f fix=%d sats=%d",
        hacc, gsa_hdop, gsa_pdop, gsa_fixtype, stored_numsats));
    }
  }
}

// --- リプレイモード ---
// replaymode_gpsoff = true の時、実 GPS の代わりに SD の飛行 CSV を再生する。
bool replaymode_gpsoff = false;

// 最後に再生した CSV 行（電圧の差し替え replay_get_voltage() が参照する）。
static ReplayRow replay_last_row;
static bool replay_last_valid = false;

// --- 再生時計（Core0 専用）---
// 実時間ではなく「仮想の再生経過時間」を積算する方式にしている。
// 毎回の経過分に倍率を掛けて足すため、再生速度の変更も一時停止も、
// 時刻の飛びを起こさずに途中で行える。
static uint32_t replay_virtual_ms = 0;   // 再生開始からの仮想経過時間 [ms]
static uint32_t replay_last_tick  = 0;   // 前回積算したときの millis()
static uint32_t replay_seen_seq   = 0;   // 確認済みの replay_init_seq
static bool     replay_paused     = false;

// 再生の一時停止／再開。設定画面など地図以外を表示している間は停止させる。
// 一時停止中は仮想時刻を進めないだけなので、再開すると続きから再生される。
void replay_set_paused(bool paused) {
  replay_paused = paused;
}

// リプレイ中で、かつ CSV にその項目の列が存在する場合のみ true。
// （古い v5 のデータは電圧などの列を持たないので false になり、実際の値へフォールバックする）
bool replay_has_value(uint16_t havebit) {
  return replaymode_gpsoff && replay_last_valid && (replay_last_row.have & havebit);
}
float replay_get_voltage()     { return replay_last_row.voltage; }


// GPS の時刻は UTC（協定世界時）で送られてくる。
// この関数で日本標準時（JST = UTC+9）に変換する。
// 日付またぎ・月またぎ・うるう年も正しく処理する。
void utcToJst(int *year, int *month, int *day, int *hour) {
  if(*month <= 0 || *month > 12){
    DEBUGW_P(20250424,"month invalid:");
    DEBUGW_PLN(20250424,*month);
    enqueueTask(createLogSdfTask("month invalid:%d",*month));
    return;
  }
  // Add 9 hours to convert UTC to JST
  *hour += 9;
  // Handle overflow of hours (24-hour format)
  if (*hour >= 24) {
      *hour -= 24;
      (*day)++;
  }
  // Handle overflow of days in each month
  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  // Check for leap year
  bool isLeapYear = ((*year % 4 == 0 && *year % 100 != 0) || (*year % 400 == 0));
  if (isLeapYear) {
      daysInMonth[1] = 29;
  }
  if (*day > daysInMonth[*month - 1]) {
      *day = 1;
      (*month)++;
  }
  // Handle overflow of months
  if (*month > 12) {
      *month = 1;
      (*year)++;
  }
}

// utcToJst() の逆変換（JST = UTC+9 → UTC）。
// PONS の飛行 CSV は JST で記録されているため、リプレイ時に g_* へ入れる前に UTC へ戻す。
// 日付またぎ・月またぎ・うるう年も正しく処理する。
void jstToUtc(int *year, int *month, int *day, int *hour) {
  if(*month <= 0 || *month > 12){
    DEBUGW_P(20260731,"month invalid:");
    DEBUGW_PLN(20260731,*month);
    return;
  }
  // Subtract 9 hours to convert JST to UTC
  *hour -= 9;
  if (*hour < 0) {
      *hour += 24;
      (*day)--;
  }
  // Handle underflow of days
  if (*day < 1) {
      (*month)--;
      if (*month < 1) {
          *month = 12;
          (*year)--;
      }
      int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      bool isLeapYear = ((*year % 4 == 0 && *year % 100 != 0) || (*year % 400 == 0));
      if (isLeapYear) {
          daysInMonth[1] = 29;
      }
      *day = daysInMonth[*month - 1];
  }
}

// i 番目に新しい NMEA 文字列を返す（i=0 が最新）。
// last_nmea はリングバッファなので、stored_nmea_index を基準に逆算してインデックスを求める。
char* get_gps_nmea(int i){
  int index = (stored_nmea_index-1-i)%MAX_LAST_NMEA;
  if(index < 0){
    index += MAX_LAST_NMEA;
  }
  return last_nmea[index];
}

// i 番目に新しい NMEA の受信時刻を返す（i=0 が最新）。
unsigned long get_gps_nmea_time(int i){
  int index = (stored_nmea_index-1-i)%MAX_LAST_NMEA;
  if(index < 0){
    index += MAX_LAST_NMEA;
  }
  return last_nmea_time[index];
}


// 30 秒以上 GSV を受信していない衛星を配列から削除する。
// 衛星が地平線以下に沈んだり、受信が途絶えた場合に古い情報が残らないようにするため。
void removeStaleSatellites() {
  unsigned long currentMillis = millis();
  for (int i = 0; i < MAX_SATELLITES; i++) {
    if (satellites[i].PRN != 0 && (currentMillis - satellites[i].lastReceived > 30000)) {
      satellites[i].PRN = 0;  // PRN を 0 にリセットして「空き」とみなす
    }
  }
}


// ゲッター
int      get_gps_fixtype()    { return gsa_fixtype; }
float    get_gps_pdop()       { return gsa_pdop; }
float    get_gps_hdop()       { return gsa_hdop; }
float    get_gps_vdop()       { return gsa_vdop; }
int      get_gsa_numsat()     { return gsa_numsat; }
int      get_gsa_prn(int i)   { return (i >= 0 && i < GSA_MAX_PRN) ? gsa_prns[i] : 0; }
// 水平精度（GST の推定, mm）。get_gps_hacc_valid() が false のときの値は意味を持たない。
uint32_t get_gps_hacc_mm()    { return gst_hacc_mm; }
// GST が新しいか（NMEA_FRESH_MS 以内）。LC86G が GST を出せない・止まっているときは false。
// ★ false のときは「分からない」。0 m（＝完璧）と取り違えないこと。
bool     get_gps_hacc_valid() {
  if (getReplayMode() || is_demo_active()) return false;
  return gst_last_ms != 0 && (millis() - gst_last_ms) <= NMEA_FRESH_MS;
}
bool     get_gps_gnssFixOK()  { return g_gnssFixOK; }

// NMEA の文を 1 行送る。チェックサムはここで計算する（body は '$' と '*' を含まない）。
// （前方宣言は nmea_handle_line() の上。設定の送り直しがこれを使う）
static void send_pair(const char* body) {
  uint8_t x = 0;
  for (const char* p = body; *p; p++) x ^= (uint8_t)*p;
  char tail[6];
  snprintf(tail, sizeof(tail), "*%02X", x);
  GPS_SERIAL.print('$');
  GPS_SERIAL.print(body);
  GPS_SERIAL.print(tail);
  GPS_SERIAL.print("\r\n");
}

// $PAIR001,<cmd>,<result> を待つ。結果（0 = 成功）を返す。来なければ PAIR_RESULT_NONE。
// 待っている間に届いた NMEA は捨てる（起動時の設定のときだけ使う）。
static int pair_wait_ack(int cmd, uint32_t timeout_ms) {
  pair_last_cmd = -1;
  pair_last_result = PAIR_RESULT_NONE;
  const unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    while (GPS_SERIAL.available() > 0) {
      const char c = (char)GPS_SERIAL.read();
      gps_connection = true;
      gps_last_byte_ms = millis();
      if ((uint8_t)c >= 128) continue;
      process_char(c);
      if (pair_last_cmd == cmd && pair_last_result != PAIR_RESULT_NONE) return pair_last_result;
    }
  }
  return PAIR_RESULT_NONE;
}

// GPS モジュールを「位置取得優先モード」に切り替える（GSV を止める）。
// 設定画面の GPS 詳細からメインマップへ戻る際に呼ばれる。
// ★ GSA は止めない（2D/3D と DOP を見るため。0.893 までは GSA も止めていた）。
void gps_getposition_mode() {
  gsv_enabled = false;
  send_pair("PAIR062,3,0");
}

// GPS モジュールを「星座表示モード」に切り替える（GSV を出す）。
// GPS 詳細画面に入る際に呼ばれる。
void gps_constellation_mode() {
  gsv_enabled = true;
  send_pair("PAIR062,3,1");
  send_pair("PAIR062,2,1");
}

int setupcounter = 1;  // gps_setup() の呼び出し回数（1=初回、2以降=リトライ）
uint32_t get_gps_baudrate() { return gps_current_baudrate; }

// gps_setup() 初回の設定コマンドが全部通ったか（起動画面の進捗表示用）。
// 通らなくても航法は動く（GSA が出なければ 2D/3D を推すだけ）が、起動画面で NG にして気づけるようにする。
static bool gps_cfg_ok = false;

// GPS モジュール（Quectel LC86G）とのシリアル接続を確立する。
// NMEA が 10 秒届かない場合は gps_loop() から自動的に再呼出しされる。
//
//   1回目 : 38400bps で開き（v5 の機体は PAIR864 で 38400 に設定済み）、
//           出力する NMEA（GSA を出す・GSV を止める・GST を出す）と EASY を設定する。
//           応答（$PAIR001）を待ち、結果を log.txt に 1 行残す。
//   2回目以降（リトライ）: ボーレートだけを順に試す（0.893 と同じ順番）。
//           115200 で開いて PAIR864 で 38400 へ切り替え → 115200 のまま → 9600 → …
void gps_setup() {
  last_gps_setup_time = millis();
  DEBUG_P(20251025,"GPS SETUP:setupcounter=");
  DEBUG_PLN(20251025,setupcounter);

  readfail_counter = 0;
  if(setupcounter != 1){
    GPS_SERIAL.end();
  }
  GPS_SERIAL.setTX(GPS_TX);
  GPS_SERIAL.setRX(GPS_RX);// Initialize GNSS

  //初回SETUP
  if(setupcounter == 1){
    DEBUG_PLN(20251025,"QUECTEL 38400");
    GPS_SERIAL.setFIFOSize(GPS_SERIAL_FIFO_SIZE);
    GPS_SERIAL.begin(gps_current_baudrate = 38400);

    // ---- 出力する NMEA と EASY を設定する ----
    // 結果は log.txt に 1 行だけ残す。通らなかった設定は画面に何も出ないので、
    // 後から原因に辿り着く唯一の手掛かりになる。
    // ★ GPS が繋がっていないと 1 つにつき 600ms 空振りする（起動画面のタイトルは先に出ている）。
    send_pair("PAIR062,2,1");                        // GSA を出す（2D/3D・DOP）
    const int r_gsa = pair_wait_ack(62, 600);
    send_pair("PAIR062,3,0");                        // GSV を止める（衛星表示の画面だけで出す）
    const int r_gsv = pair_wait_ack(62, 600);
    send_pair("PAIR062,8,1");                        // GST を出す（水平精度）
    gst_result = pair_wait_ack(62, 600);
    send_pair(GNSS_USE_EASY ? "PAIR490,1" : "PAIR490,0");  // EASY（軌道の自己予測）
    easy_result = pair_wait_ack(490, 600);
    gps_cfg_ok = (r_gsa == PAIR_RESULT_OK && r_gsv == PAIR_RESULT_OK);
    // 通らなかったら、NMEA が届き始めたところで送り直す（上の「設定の送り直し」）。
    cfg_resend_pending = !gps_cfg_ok;
    // 結果の数字は $PAIR001 のまま（0=成功 1=処理中 2=失敗 3=非対応 4=引数の誤り 5=混雑、-1=応答なし）。
    enqueueTask(createLogSdfTask("GPS CFG %s GSA=%d GSV=%d GST=%d EASY%s=%d",
      gps_cfg_ok ? "OK" : "NG", r_gsa, r_gsv, gst_result,
      GNSS_USE_EASY ? "on" : "off", easy_result));
  }else{
    // リトライ: ここでは設定コマンドは送らず、ボーレートを順に試す（0.893 と同じ順番）。
    // 出力の設定は、正しい NMEA が届き始めたところで送り直す（上の「設定の送り直し」）。
    cfg_resend_pending = true;
    switch (setupcounter % 3) {
      case 1:
        DEBUG_PLN(20251025,"from 115200 to 38400 (For QUECTEL)");
        GPS_SERIAL.setFIFOSize(GPS_SERIAL_FIFO_SIZE);
        GPS_SERIAL.begin(115200);
        send_pair("PAIR864,0,0,38400");  // モジュールの再起動が要る
        delay (100);//なぜか必要（Do not delete without care.)
        GPS_SERIAL.end();
        delay(50);//なぜか必要（Do not delete without care.)
        GPS_SERIAL.setFIFOSize(GPS_SERIAL_FIFO_SIZE);
        GPS_SERIAL.begin(gps_current_baudrate = 38400);
        break;
      case 2:
        DEBUG_PLN(20251025,"Simple setup try 115200");
        GPS_SERIAL.setFIFOSize(GPS_SERIAL_FIFO_SIZE);
        GPS_SERIAL.begin(gps_current_baudrate = 115200);
        break;
      default:
        DEBUG_PLN(20251025,"Simple setup try 9600");
        GPS_SERIAL.setFIFOSize(GPS_SERIAL_FIFO_SIZE);
        GPS_SERIAL.begin(gps_current_baudrate = 9600);
        break;
    }
  }

  for(int i = 0; i < MAX_LAST_NMEA;i++){
    last_nmea[i][0] = 0;
    last_nmea_time[i] = millis();
  }

  setupcounter++;
}

// 最大 G/S の全情報をリセットする（デモ/リプレイモード切替時に呼ぶ）
static void reset_maxgs() {
  maxgs                  = 0.0f;
  maxgs_hour             = 0;
  maxgs_min              = 0;
  maxgs_5min             = 0.0f;
  maxgs_5min_hour        = 0;
  maxgs_5min_min         = 0;
  maxgs_5min_last_update = 0;
}

// 最大 G/S を更新する（NMEAパス・DEMOパス共通）。
// 時刻が有効な場合は JST 時刻を記録し、無効な場合は 0:00 として速度値だけ更新する。
static void update_maxgs(float gs) {
  int jst_h = 0, jst_m = 0;
  // NMEA・リプレイの両方で g_hour/g_min を参照（両方とも更新済み）
  if (g_time_valid) {
    jst_h = (g_hour + 9) % 24;
    jst_m = g_min;
  }
  // 全時間最大 G/S
  if (gs > maxgs) {
    maxgs      = gs;
    maxgs_hour = jst_h;
    maxgs_min  = jst_m;
  }
  // 5 分保持最大 G/S
  if (gs > maxgs_5min) {
    maxgs_5min             = gs;
    maxgs_5min_hour        = jst_h;
    maxgs_5min_min         = jst_m;
    maxgs_5min_last_update = millis();
  } else if (millis() - maxgs_5min_last_update > 5UL * 60 * 1000) {
    // 5 分間更新がなければ現在の G/S にリセット
    maxgs_5min             = gs;
    maxgs_5min_hour        = jst_h;
    maxgs_5min_min         = jst_m;
    maxgs_5min_last_update = millis();
  }
}

extern unsigned long last_gps_time;  // 実体は下方で定義（apply_replay_row から先に参照するため）

// リプレイ: CSV 1行分の内容を GPS の内部状態に反映する。
// 通常モードで apply_nmea_epoch() が NMEA から行っていることと同じ内容を CSV から行う。
// CSV に列が無い項目（古い v5 データの高度など）は更新せず、直前の値のままにする。
static void apply_replay_row(const ReplayRow* row) {
  last_gps_time = millis();

  // --- 位置 ---
  if (stored_latitude != row->lat || stored_longitude != row->lon)
    if (!is_demo_active()) new_location_arrived = true;
  stored_latitude  = row->lat;
  stored_longitude = row->lon;
  g_pos_valid      = true;
  stored_fixtype   = 2;

  // --- 対地速度・真方位・高度 ---
  if (row->have & RHAVE_GNSSALT)
    stored_gnss_altitude = row->gnss_altitude;
  if (row->have & RHAVE_GS) {
    stored_gs = row->gs;
    update_maxgs(stored_gs);
  }
  if (row->have & RHAVE_TTRACK) {
    if (!is_demo_active()) newcourse_arrived = true;
    stored_truetrack = row->ttrack;
    if (stored_truetrack < 0 || stored_truetrack > 360)
      stored_truetrack = 0;
  }
  if (row->have & RHAVE_NUMSAT)
    stored_numsats = row->numsat;

  // --- 日時 ---
  // CSV は JST で記録されているが g_* は UTC 前提で扱われるので UTC に戻す。
  int y = row->year, mo = row->month, d = row->day, h = row->hour;
  if (row->have & RHAVE_DATE) {
    jstToUtc(&y, &mo, &d, &h);
    g_year  = y;
    g_month = mo;
    g_day   = d;
    g_date_valid = true;
  } else {
    h = (h + 24 - 9) % 24;  // 日付列が無い場合は時刻だけ 9 時間戻す
  }
  g_hour = h;
  g_min  = row->minute;
  g_sec  = row->second;
  g_cs   = row->centisecond;
  g_time_valid = true;
  gps_fix_millis = millis();

  // 電圧の差し替え（replay_get_voltage）用に最後の行を保持する
  replay_last_row   = *row;
  replay_last_valid = true;

  // リプレイ中は CSV 保存されないが、航跡の記録（add_latlon_track）はここで行われる
  try_enque_savecsv();

  // 初回のみ再生開始時刻を SD ログに残す
  static bool first_time_logged_replay = false;
  if (!first_time_logged_replay && (row->have & RHAVE_DATE)) {
    first_time_logged_replay = true;
    enqueueTask(createLogSdfTask("GPS TIME(REPLAY): %04d-%02d-%02d %02d:%02d:%02d JST",
      row->year, row->month, row->day, row->hour, row->minute, row->second));
  }
}

// ---- デモ飛行の地点定義 ----
// lat/lon はその地点のおおよその中心。basetrack は基準となる進行方位で、
// 海岸線や川筋に沿って飛んでいるように見える向きを選んである。
struct demo_site_def {
  const char* name;
  double lat;
  double lon;
  int basetrack;
};
static const demo_site_def DEMO_SITES[DEMO_SITE_COUNT] = {
  { "OFF",       0,             0,             0   },
  { "BIWAKO",    PLA_LAT,       PLA_LON,       280 },
  { "SHIRAHAMA", SHIRAHAMA_LAT, SHIRAHAMA_LON, 200 },
  { "KASAOKA",   KASAOKA_LAT,   KASAOKA_LON,   250 },
  { "FUJIGAWA",  FUJIGAWA_LAT,  FUJIGAWA_LON,  190 },
  { "TOKYO",     SHINURA_LAT,   SHINURA_LON,   220 },
  { "OSAKA",     OSAKA_LAT,     OSAKA_LON,     240 },
};

demo_site_t get_demo_site() {
  return demo_site;
}

const char* get_demo_site_name(demo_site_t site) {
  if (site >= DEMO_SITE_COUNT) return "?";
  return DEMO_SITES[site].name;
}

bool is_demo_active() {
  return demo_site != DEMO_OFF;
}

// 地点を切り替える。仮想機体をその地点の中心に置き直し、軌跡も消す
// （前の地点の軌跡が地球の反対側に残っていると描画が破綻するため）。
void set_demo_site(demo_site_t site) {
  if (site >= DEMO_SITE_COUNT) site = DEMO_OFF;
  const bool was_demo = (demo_site != DEMO_OFF);
  demo_site = site;
  if (site != DEMO_OFF) {
    demo_lat = DEMO_SITES[site].lat;
    demo_lon = DEMO_SITES[site].lon;
    demo_truetrack = DEMO_SITES[site].basetrack;
  }
  latlon_manager.reset();
  reset_maxgs();  // モード切替時に最大 G/S をリセット
  // AUTO10K の折返しフェーズ。デモ中は仮想機体の位置で動くので、抜けるときに
  // 実飛行の値へ戻す（リプレイと同じ理由。apply_auto10k_status() のコメント参照）。
  // 地点から地点へ移るときは戻さない ― デモはそのまま続いているため。
  if (was_demo && site == DEMO_OFF) auto10k_restore_flight_phase();
}

void next_demo_site() {
  set_demo_site((demo_site_t)((demo_site + 1) % DEMO_SITE_COUNT));
}

void set_demo_off() {
  set_demo_site(DEMO_OFF);
}

unsigned long last_latlon_manager = 0;
unsigned long last_gps_save_time = 0;
unsigned long last_gps_time = 0;// last position update time.


// g_* 変数（NMEA の解析結果 or リプレイモードでミラーされた値）から GpsDate を返す
GpsDate get_gpsdate(){
  GpsDate d;
  d._year  = g_year;
  d._month = g_month;
  d._day   = g_day;
  d._valid = g_date_valid;
  return d;
}
// g_* 変数から GpsTime を返す
GpsTime get_gpstime(){
  GpsTime t;
  t._hour  = g_hour;
  t._min   = g_min;
  t._sec   = g_sec;
  t._cs    = g_cs;
  t._valid = g_time_valid;
  return t;
}

unsigned long last_demo_gpsupdate = 0;

// 飛行軌跡（トラックログ）に現在位置を追加する。
// 追加間隔を速度に応じて調整することで、約 50m おきに記録する。
//   低速（停止近く）→ 最大 15 秒間隔（静止中に大量の点が積まれるのを防ぐ）
//   高速（約 20m/s）→ 最小 1 秒間隔
void add_latlon_track(float lat,float lon){
  int tracklog_interval = constrain(50000/(1.0+stored_gs/2.0), 1000, 15000);//約50mおきに一回記録するような計算となる。
  if(is_demo_active()){
    tracklog_interval = 900;  // デモモードは短めの間隔でアニメーション的に記録
  }
  if(millis() - last_latlon_manager > tracklog_interval){
    latlon_manager.addCoord({ lat, lon});
    last_latlon_manager = millis();
  }
}

bool gps_new_location_arrived(){
  if(is_demo_active()){
    // 500ms 間隔で仮想位置を更新する（v6 の 2Hz に合わせた値のまま。実 GPS は 1Hz だが、
    // デモは画面の動きを見るためのものなので滑らかな方がよい）。
    if(millis() - last_demo_gpsupdate > 500){
      const demo_site_def& site = DEMO_SITES[demo_site];
      int demo_spd = 10;
      demo_lat += 0.000025*demo_spd*cos(radians(get_gps_truetrack()));
      // 選択中の地点から離れすぎたら中心に戻す（地図の収録範囲から出ないように）
      if(calculateDistanceKm(demo_lat,demo_lon,site.lat,site.lon) > 15){
        demo_lat = site.lat;
        demo_lon = site.lon;
        latlon_manager.reset();
      }
      demo_lon += 0.000025*demo_spd*sin(radians(get_gps_truetrack()));
      last_demo_gpsupdate = millis();
      new_location_arrived = true;
      demo_mps = 7 + sin(millis() / 1500.0);
      update_maxgs(demo_mps);  // DEMOモードでも最大 G/S を更新
      
      
      int basetrack = site.basetrack;
      if(demo_site == DEMO_BIWAKO && destination_mode == DMODE_AUTO10K && auto10k_status == AUTO10K_INTO)
        basetrack = 100;  // 琵琶湖の折り返し後は東向き

      
      int target_angle = basetrack+(20 + 10*sin(millis() / 2100.0)) * sin(millis() / 3000.0)+50*sin(millis() / 10000.0);

      int demo_steer_angle = target_angle - demo_truetrack;
      if(demo_steer_angle < -180){
        demo_steer_angle += 360;
      }else if(demo_steer_angle > 180){
        demo_steer_angle -= 360;
      }
      // 2Hz 化により呼び出し頻度が 2 倍になったため、1 呼び出しあたりの旋回ゲインを半分 (0.2 → 0.1) にして旋回速度を同等に保つ。
      demo_truetrack += demo_steer_angle*0.1*(max(0,sin(millis() / 5000.0)));//basetrack + (10 + 5*sin(millis() / 2100.0)) * sin(millis() / 3000.0)+50*sin(millis() / 10000.0);
      if(demo_truetrack > 360)
        demo_truetrack -= 360;
      else if(demo_truetrack < 0)
        demo_truetrack += 360;

      newcourse_arrived = true;
      add_latlon_track(demo_lat, demo_lon);
    }
  }
  return new_location_arrived;
}

void set_new_location_off(){
  new_location_arrived = false;
}



bool getReplayMode(){
  return replaymode_gpsoff;
}

void set_replaymode(bool replaymode){
  const bool was_replay = replaymode_gpsoff;
  replaymode_gpsoff = replaymode;
  // ★ 再生を始めるときに必ず落とす。Core1 の init_replay() も先頭で落とすが、
  //   あちらは非同期なので、Core0 が先に前回の true を読んで
  //   **正常なファイルの再生をいきなり解除してしまう**競合が起きる。
  if (replaymode) replay_file_bad = REPLAY_BAD_NONE;
  reset_maxgs();  // モード切替時に最大 G/S をリセット
  // AUTO10K の折返しフェーズ。再生中は再生データで動くので、抜けるときに
  // 実飛行の値へ戻す（再生した過去フライトのフェーズを持ち込まないため）。
  if (was_replay && !replaymode) auto10k_restore_flight_phase();
  // 別ファイルに切り替えた直後に、前のファイルの値を一瞬だけ表示してしまわないようにクリアする
  replay_last_valid = false;

  // ★重要: 再生で書き込まれた「過去の日時」を捨てる（v6 の 0.94x で踏んだ不具合）。
  // apply_replay_row() は CSV の日時を g_* にそのまま入れるため、リプレイを
  // 抜けた直後は日付が再生した飛行の日のままになる。屋内など GPS フィックスが
  // 無い場所ではこれが更新されないので、get_jst_now() が過去の日付を返し続け、
  // 飛行 CSV などが「その日のファイル」へ机の上の値を追記してしまう。
  // 本物の NMEA（RMC）が届くまで日時は無効とする。
  g_date_valid = false;
  g_time_valid = false;
  g_gnssFixOK  = false;   // 書き込み側の保険（get_gps_gnssFixOK()）も一緒に落とす
}
// GPS モジュールからの Serial データを受信・解析するメインループ処理。
// 描画の途中でも複数回呼ばれることで、GPS データの取りこぼしを防ぐ（id は呼び出し箇所識別子）。
//
// 異常検知:
//   - 10 秒間 NMEA が届かない → gps_setup() で再接続を試みる（0.893 と同じ間隔）
//   - ASCII でない文字が 10 回 → ボーレートが合っていない → gps_setup()
//   - FIFO バッファが 256 を超えている → 警告ログを出す
void gps_loop(int id) {

  // 10 秒以上 NMEA が途絶えた場合は GPS を再初期化する。
  // リプレイ中は実 GPS のデータを読み捨てており last_nmea_time が更新されないため、
  // このウォッチドッグが誤発火してしまう。リプレイ中は無効にする。
  if(!replaymode_gpsoff &&
     GPS_SERIAL.available() == 0 && millis() - get_gps_nmea_time(0) > 10000 && millis() - last_gps_setup_time > 10000){
    DEBUGW_PLN(20250923,"Lost NMEA MSG for 10 seconds. resetup.");
    gps_setup();
  }
  if(GPS_SERIAL.available() > 256){
    DEBUGW_P(20250923,"ID=");
    DEBUGW_P(20250923,id);
    DEBUGW_P(20250923," Caution, remaining FIFO buffer. avail=");
    DEBUGW_PLN(20250923,GPS_SERIAL.available());
  }

  while (GPS_SERIAL.available() > 0) {
    char c = (char)GPS_SERIAL.read();
    if(GPS_SERIAL.overflow()){
      DEBUGW_P(20250923,"!!WARNING!! GPS FIFO Overflow avail=");
      DEBUGW_PLN(20250923,GPS_SERIAL.available());
      static unsigned long last_ovf_log = 0;
      if (millis() - last_ovf_log > 10000) {
        last_ovf_log = millis();
        enqueueTask(createLogSdTask("!!ERROR!! GPS FIFO overflow"));
      }
    }
    gps_connection = true;
    gps_last_byte_ms = millis();
    if (replaymode_gpsoff) continue;   // リプレイ中は実 GPS のデータを読み捨てる
    if ((uint8_t)c >= 128) {
      // NMEA は ASCII だけ。128 以上が続くのはボーレートが合っていない証拠。
      readfail_counter++;
      if (readfail_counter > 10) {
        DEBUGW_P(20250923,"Read Failed 10 times, non ascii char:");
        DEBUGW_PLN(20250923,(int)c);
        gps_setup();
      }
      continue;
    }
    process_char(c);
  }

  // ============================================================
  // リプレイモード: SD の飛行 CSV を再生する
  //
  // Core1 (load_replay) が CSV を1行ずつパースして replay_rows[] に積み、
  // ここでは「再生経過時間に達した行」を取り出して stored_* / g_* に反映する。
  // ペーシングを Core0 側で行うため、SD の応答待ちに再生速度が引きずられない。
  // ============================================================
  if (replaymode_gpsoff) {
    static unsigned long last_replay_request = 0;

    // ★ リプレイを続けられない。ここで解除しないと、下の「末尾ならループ再生」が
    //   延々と init を繰り返し、地図が固まったまま SD を叩き続ける。
    //   通常の GPS に戻して音で知らせ、理由をログに残す（対処が違うため）。
    if (replay_file_bad != REPLAY_BAD_NONE) {
      const uint8_t reason = replay_file_bad;
      replay_file_bad = REPLAY_BAD_NONE;
      set_replaymode(false);
      set_replay_filename("");
      // 最低保証音量は付けない。設定画面で操作した直後に鳴るので持ち主はその場にいる
      // （60 は「離れていても電池切れに気づける」ための仕組みで、電池警告専用）。
      enqueueTask(createPlayMultiToneTask(330, 200, 2, 3));
      enqueueTask(createLogSdTask(reason == REPLAY_BAD_NOSD
                                  ? "REPLAY canceled: SD not available"
                                  : "REPLAY canceled: file is not a flight log"));
      return;
    }

    // --- 再生時計の更新 ---
    uint32_t now = millis();
    if (replay_seen_seq != replay_init_seq) {
      // Core1 が init_replay() を実行した → 先頭から再生し直す
      replay_seen_seq   = replay_init_seq;
      replay_virtual_ms = 0;
      replay_last_tick  = now;
    }
    if (replay_paused) {
      replay_last_tick = now;   // 停止中は経過を積まない（再開時に飛ばないように基準だけ進める）
    } else {
      replay_virtual_ms += (now - replay_last_tick) * (uint32_t)get_replay_speed();
      replay_last_tick   = now;
    }
    uint32_t elapsed = replay_virtual_ms;
    ReplayRow row;
    // 再生時刻に達した行をまとめて反映する（描画等で遅れた場合の取りこぼしを防ぐ）
    while (replay_available() && replay_peek_t_ms() <= elapsed) {
      if (!replay_pop(&row)) break;
      apply_replay_row(&row);
    }

    if (replay_eof && !replay_available()) {
      // ファイル末尾に到達 → 先頭に戻ってループ再生する
      // （キュー内・実行中の両方を確認しないと、Core1 が処理中の間に二重投入してしまう）
      if (!isTaskInQueue(TASK_INIT_REPLAY) && !isTaskRunning(TASK_INIT_REPLAY))
        enqueueTask(createInitReplayTask());
    } else if (replay_buffer_has_space() &&
               // 高速再生ほど消費が速いので補充間隔も倍率で詰める
               millis() - last_replay_request > (REPLAY_REQ_INTERVAL_MS / (uint32_t)get_replay_speed()) &&
               !isTaskInQueue(TASK_LOAD_REPLAY) && !isTaskRunning(TASK_LOAD_REPLAY)) {
      last_replay_request = millis();
      enqueueTask(createLoadReplayTask());  // Core1 にバッファの補充を依頼
    }
    return;
  }

  // ============================================================
  // 通常モード (NMEA): RMC でエポックが締まったら、そのエポックの値を反映する
  // ============================================================
  if (nmea_epoch_ready) {
    nmea_epoch_ready = false;
    apply_nmea_epoch();
    last_gps_time = millis();
    if (!is_demo_active()) {
      if (g_pos_valid) new_location_arrived = true;
      newcourse_arrived = true;
    }
    update_maxgs(stored_gs);

    if (stored_truetrack < 0 || stored_truetrack > 360) {
      DEBUGW_P(20250923,"ERROR:MT");
      DEBUGW_PLN(20250923,stored_truetrack);
      enqueueTask(createLogSdfTask("ERR truetrack=%.1f (forced 0)", stored_truetrack));
      stored_truetrack = 0;
    }

    try_enque_savecsv();

    // 初回 GPS 時刻取得時の SD ログ（1回だけ）
    if (g_time_valid && g_date_valid) {
      static bool first_time_logged = false;
      if (!first_time_logged) {
        first_time_logged = true;
        enqueueTask(createLogSdfTask("GPS TIME: %04d-%02d-%02d %02d:%02d:%02d UTC",
          g_year, g_month, g_day, g_hour, g_min, g_sec));
      }
    }

    // 30 秒以上届いていない衛星を表示から外す（GSV は衛星表示の画面でしか出さないため）
    removeStaleSatellites();
  }
}


extern int max_adreading;  // display_tft.cpp で計測したバッテリー ADC 最大値

// GPS データが揃っている場合、フライトログ CSV への保存タスクをキューに積む。
// 900ms に1回だけ保存するようにクールダウンを設けている（1Hz の GPS に合わせて）。
// NMEA・リプレイの両方で g_* 変数を参照する（リプレイモードでは apply_replay_row() でミラー済み）。
// リプレイモードでは保存しない。
void try_enque_savecsv(){
  bool all_valid = true;
  if (g_pos_valid) {
    stored_fixtype = (stored_numsats > 0) ? 2 : 1;
  } else {
    DEBUG_PLN(20250923,"Location: Not Available");
    all_valid = false;
    stored_fixtype = 0;
  }
  if (!g_date_valid) {
    DEBUG_PLN(20250923,"Date: Not Available");
    all_valid = false;
  }
  if (!g_time_valid) {
    DEBUG_PLN(20250923,"Time: Not Available");
    all_valid = false;
  }

  // 位置・日時が全て揃っていて、前回保存から 900ms 以上経過した時のみ保存（1Hz GPS に合わせて調整）
  if(all_valid && millis() - last_gps_save_time > 900){
    if(!is_demo_active()){ //デモは別の場所で登録済み。
      add_latlon_track(get_gps_lat(),get_gps_lon());
    }

    //リプレイは保存しない。
    if(!getReplayMode()){
      #ifndef RELEASE
      // 保存間隔の実測（デバッグ専用）。RELEASE では DEBUG_* が消えるので丸ごと除く。
      DEBUG_P(20250923,"SAVED!");
      static int lastsavedtime = 0;
      DEBUG_PLN(20250923,millis()-lastsavedtime);
      lastsavedtime = millis();
      #endif
      int year  = g_year;
      int month = g_month;
      int day   = g_day;
      int hour  = g_hour;
      utcToJst(&year, &month, &day, &hour);
      // 実測の電圧を書く（リプレイ中の表示値ではない）。リプレイ再生中は
      // そもそもこのパスを通らないが、意図を明示するため get_input_voltage() を使う。
      float csv_voltage   = get_input_voltage();
      // 水平精度（GST）。分からないときは負を渡し、CSV では空欄にする。
      float csv_hacc      = get_gps_hacc_valid() ? gst_hacc_mm / 1000.0f : -1.0f;
      enqueueTask(createSaveCsvTask(stored_latitude, stored_longitude, stored_gs, stored_truetrack,
        stored_gnss_altitude, csv_hacc, csv_voltage, stored_numsats,
        year, month, day, hour, g_min, g_sec, g_cs));

      last_gps_save_time = millis();
    }
  }
}






// 現在の緯度を返す。
// デモモードや各デバッグシミュレーション設定が有効な場合は、実際の GPS 座標の代わりに
// 設定した固定座標やオフセット座標を返す（settings.h の #define で切り替える）。
double get_gps_lat() {
  if (is_demo_active()) {
    return demo_lat;
  }
#ifdef DEBUG_GPS_SIM_BIWAKO
  return PLA_LAT;
#endif
#ifdef DEBUG_GPS_SIM_SHINURA
  int timeelapsed = ((int)(millis()/1000)) % 20000;
  return SHINURA_LAT + timeelapsed / 1600.0;
#endif
#ifdef DEBUG_GPS_SIM_SAPPORO
  return SAPPORO_LAT;
#endif
#ifdef DEBUG_GPS_SIM_SHISHI
  return SHISHI_LAT;
#endif
#ifdef DEBUG_GPS_SIM_SHINURA2BIWA
  return PLA_LAT + stored_latitude - SHINURA_LAT+0.02;
#endif
#ifdef DEBUG_GPS_SIM_OSAKA2BIWA
  return PLA_LAT + stored_latitude - OSAKA_LAT;
#endif

#ifdef DEBUG_GPS_SIM_SHINURA2OSAKA
  return OSAKA_LAT + stored_latitude - SHINURA_LAT;
#endif

  return stored_latitude;
}

double get_gps_lon() {
  if (is_demo_active()) {
    return demo_lon;
  }

#ifdef DEBUG_GPS_SIM_BIWAKO
  return PLA_LON;
#endif
#ifdef DEBUG_GPS_SIM_SHINURA
  int timeelapsed = ((int)(millis()/1000)) % 20000;
  return SHINURA_LON + timeelapsed / 16000.0;
#endif
#ifdef DEBUG_GPS_SIM_SAPPORO
    return SAPPORO_LON;
#endif
#ifdef DEBUG_GPS_SIM_SHISHI
    return SHISHI_LON;
#endif
#ifdef DEBUG_GPS_SIM_SHINURA2BIWA
  return PLA_LON + stored_longitude - SHINURA_LON-0.03;
#endif
#ifdef DEBUG_GPS_SIM_OSAKA2BIWA
  return PLA_LON + stored_longitude - OSAKA_LON;
#endif

#ifdef DEBUG_GPS_SIM_SHINURA2OSAKA
  return OSAKA_LON + stored_longitude - SHINURA_LON;
#endif

  return stored_longitude;
}


double get_gps_mps() {
  if (is_demo_active()) {
    return demo_mps;
  }
  return stored_gs;
}

// 最大 G/S getter 関数群
float get_maxgs()           { return maxgs; }
int   get_maxgs_hour()      { return maxgs_hour; }
int   get_maxgs_min()       { return maxgs_min; }
float get_maxgs_5min()      { return maxgs_5min; }
int   get_maxgs_5min_hour() { return maxgs_5min_hour; }
int   get_maxgs_5min_min()  { return maxgs_5min_min; }


// GPS モジュールと「いま」通信できているか。
// ★★ **一度でも受信したか、ではない。** GPS_CONN_TIMEOUT_MS のあいだバイトが
//   届かなければ false を返す。飛行中にコネクタが抜けたり配線が切れたりしても
//   画面が正常なまま航法を続けるのを防ぐため（表示は display_tft.cpp の
//   draw_gnss_disconnect_warning()、ログと音は GPS_TFT_map.ino の gnss_conn_tick()）。
// ★ 起動直後（まだ 1 バイトも来ていない）も false。従来と同じ。
bool get_gps_cfg_ok() {
  return gps_cfg_ok;
}

bool get_gps_connection() {
  if (!gps_connection) return false;
  return (uint32_t)(millis() - gps_last_byte_ms) < GPS_CONN_TIMEOUT_MS;
}
bool get_gps_fix() {
  if(is_demo_active()){
    return get_gps_numsat() != 0;
  }
  if(stored_fixtype >= 1){
    return true;
  }
  return false;
}

double get_gps_altitude() {
  return stored_gnss_altitude;
}


double get_gps_truetrack() {
  #ifdef DEBUG_GPS_SIM_SHINURA
    return 40 + (38.5 + sin(millis() / 2100.0)) * sin(millis() / 3000.0);
  #endif
  if (is_demo_active()) {
    return demo_truetrack;
  }
  return stored_truetrack;     // Heading in degrees;
}


int get_gps_numsat() {
  if(is_demo_active()){
    return (int)(20.0*sin(millis()/5000))+20;
  }else if(getReplayMode()){
    // CSV に numsat 列があればその値を使う。無い場合は「衛星情報なし」を示す 99 を返す。
    if(replay_has_value(RHAVE_NUMSAT))
      return stored_numsats;
    return 99;
  }
  return stored_numsats;
}

