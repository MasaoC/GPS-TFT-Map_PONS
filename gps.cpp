// ============================================================
// File    : gps.cpp
// Project : PONS v3 (Pilot Oriented Navigation System for HPA)
// Role    : GPS（u-blox SAM-M10Q）の受信と解析。UBX バイナリ（NAV-PVT / NAV-SAT）。
//           設定（2Hz・Airborne <1g・AssistNow Autonomous）、通信断と測位喪失の検出、
//           最大 G/S、飛行 CSV と航跡への記録、デモとシミュレーションの位置。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/10/25
// ============================================================
//
// ■ NMEA から UBX へ（0.503。v7 の 0.914 と同じ移行）
//   以前は NMEA を TinyGPS++ で読んでいた。UBX の NAV-PVT は 1 つのフレームに位置・速度・方位・時刻・
//   精度（hAcc / sAcc）・gnssFixOK が入り、チェックサムで化けを確実に弾ける。量も 1/10 ほどになり、
//   地図を描いている間に UART の FIFO があふれる心配がなくなった。
//
// ■ v7 と違うところ
//   ・通信断のやり直しで、毎回 9600bps からすべての設定を送り直す。v3 の GPS はケーブルの先にあり、
//     GPS だけ電源が瞬断すると工場出荷（9600bps・NMEA）に戻る。v7 のやり直しはボーレートを
//     切り替えるだけで設定を送らないので、その場合に戻ってこられない。
//   ・UBX のヘッダの長さが受信バッファより大きいフレームは、その場で捨てて同期を取り直す
//     （化けたヘッダの長さを信じると、最大 64KB ぶん＝約 17 秒、正しいフレームまで読み捨てる）。
#include <Arduino.h>

#include "gps.h"
#include "mysd.h"
#include "navdata.h"
#include "settings.h"
#include "power.h"

double stored_longitude, stored_latitude, stored_truetrack, stored_altitude, stored_gs;
int stored_numsats;
bool gps_connection = false;          // 一度でもチェックサムの合う UBX のフレームを受けたか
unsigned long last_valid_frame_ms = 0; // チェックサムの合う UBX のフレームを最後に受けた時刻
unsigned long time_last_gps_frame = 0; // 同じ（受信中の表示用。display_tft.cpp）
unsigned long last_gps_setup_ms = 0;  // gps_setup() を最後に呼んだ時刻
bool gps_resetup_logged = false;      // 今回の通信断で SD に記録したか
bool demo_biwako = false;

SatelliteData satellites[MAX_SATELLITES];
static char framelog[GPS_FRAMELOG_LINES][GPS_FRAMELOG_CHARS];
static int framelog_index = 0;

// ----- NAV-PVT から取り出した値 -----
static uint8_t  pvt_fixtype   = 0;      // u-blox fixType（0=なし 1=DR 2=2D 3=3D 4=GNSS+DR 5=時刻のみ）
static bool     pvt_fixok     = false;  // flags bit0 gnssFixOK
static bool     pvt_pos_valid = false;  // 位置を使ってよいか（2D/3D/GNSS+DR かつ gnssFixOK）
static uint32_t pvt_hacc_mm   = 0;
static uint32_t pvt_sacc_mmps = 0;
static float    pvt_pdop      = 0.0f;
static GpsDate  pvt_date;
static GpsTime  pvt_time;
static unsigned long last_fix_ms = 0;   // 位置が有効な NAV-PVT を最後に受けた時刻（0 = まだ）
static bool     pvt_new       = false;  // gps_loop() に新しい NAV-PVT を知らせる
static uint8_t  ana_data_svs  = 0;      // AssistNow Autonomous の予測データを持つ衛星数（NAV-SAT aopAvail）
static uint8_t  ana_orbit_svs = 0;      // いま予測軌道で測っている衛星数（NAV-SAT orbitSource=4）
static uint8_t  eph_orbit_svs = 0;      // いま放送暦で測っている衛星数（NAV-SAT orbitSource=1）
static bool     ana_fix_report_pending = false;  // 最初の測位のあと、次の NAV-SAT で軌道の出どころを記録する

static void maxgs_update(float gs);
static void maxgs_reset();
static bool rp_active = false;   // リプレイ中（下の「リプレイ」）。実際の GPS の位置は使わない

void utcToJst(int *year, int *month, int *day, int *hour) {
  if(*month <= 0 || *month > 12){
    Serial.print("month invalid:");
    Serial.println(*month);
    enqueueLogSdf("month invalid:%d",*month);
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

const char* get_gps_framelog(int i){
  int index = (framelog_index-1-i)%GPS_FRAMELOG_LINES;
  if(index < 0){
    index += GPS_FRAMELOG_LINES;
  }
  return framelog[index];
}

static void framelog_add(const char* format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(framelog[framelog_index], GPS_FRAMELOG_CHARS, format, args);
  va_end(args);
  framelog_index = (framelog_index + 1) % GPS_FRAMELOG_LINES;
}

void removeStaleSatellites() {
  unsigned long currentMillis = millis();
  for (int i = 0; i < MAX_SATELLITES; i++) {
    if (satellites[i].PRN != 0 && (currentMillis - satellites[i].lastReceived > 30000)) {
      satellites[i].PRN = 0;  // Mark as stale and remove
    }
  }
}


// ============================================================
// UBX の受信と解析
// ============================================================
static int32_t le_i32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

// UBX-NAV-PVT（class 0x01, id 0x07, 92 バイト）。位置・速度・方位・時刻・精度。
static void handle_navpvt(const uint8_t* p, uint16_t len) {
  if (len < 84) return;

  // ★ リプレイ中は実際の GPS の位置・時刻を使わない（再生している飛行の値で上書きしてしまうため）。
  //   通信が生きていることだけは数える（process_ubx の last_valid_frame_ms）。
  if (rp_active) {
    framelog_add("NAV-PVT (replay: ignored) fix=%d", p[20]);
    return;
  }
  // UTC の日時（valid: bit0=validDate, bit1=validTime）
  const uint8_t valid = p[11];
  pvt_date.valid = (valid & 0x01) != 0;
  pvt_time.valid = (valid & 0x02) != 0;
  pvt_date.y  = p[4] | (p[5] << 8);
  pvt_date.mo = p[6];
  pvt_date.d  = p[7];
  pvt_time.h  = p[8];
  pvt_time.mi = p[9];
  pvt_time.s  = p[10];
  // 起動直後に validDate=1 でも 2000 年 0 月 0 日を返すことがある（v7 で確認）。月は utcToJst() の添字になる。
  if (pvt_date.y < 2020 || pvt_date.mo < 1 || pvt_date.mo > 12 || pvt_date.d < 1 || pvt_date.d > 31) pvt_date.valid = false;
  if (pvt_time.h > 23 || pvt_time.mi > 59 || pvt_time.s > 60) pvt_time.valid = false;

  pvt_fixtype = p[20];
  pvt_fixok   = (p[21] & 0x01) != 0;
  const bool was_valid = pvt_pos_valid;
  pvt_pos_valid = pvt_fixok && pvt_fixtype >= 2 && pvt_fixtype <= 4;
  stored_numsats = p[23];
  pvt_hacc_mm   = (uint32_t)le_i32(p + 40);
  pvt_sacc_mmps = (uint32_t)le_i32(p + 68);
  pvt_pdop      = (uint16_t)(p[76] | (p[77] << 8)) * 0.01f;

  // ★ 位置・速度・方位は測位しているときだけ更新する（測位していない回の値は 0 や前の値で意味が無い）。
  //   測位を失ったことは get_gps_fix() が last_fix_ms で判定し、画面がグレーと「GPS FIX LOST」で知らせる。
  if (pvt_pos_valid) {
    stored_longitude = le_i32(p + 24) * 1e-7;
    stored_latitude  = le_i32(p + 28) * 1e-7;
    stored_altitude  = le_i32(p + 36) * 1e-3;  // hMSL [mm] → [m]
    stored_gs        = le_i32(p + 60) * 1e-3;  // gSpeed [mm/s] → [m/s]
    double trk = le_i32(p + 64) * 1e-5;        // headMot [1e-5 deg]
    if (trk < 0) trk += 360.0;
    if (trk >= 360.0) trk -= 360.0;
    stored_truetrack = trk;
    last_fix_ms = millis();
  }

  // 測位できた瞬間を残す。起動後最初の 1 回は起動からの秒数も（電源を入れて起動したときは、これが
  // 測位までの時間。AssistNow Autonomous が効いていれば数秒、効いていなければ 20〜30 秒）。
  if (!was_valid && pvt_pos_valid) {
    static bool first_fix_logged = false;
    if (!first_fix_logged) {
      first_fix_logged = true;
      ana_fix_report_pending = true;
      enqueueLogSdf("GPS FIXED fix=%d sats=%d hAcc=%.1fm %.1fs after boot",
                                   pvt_fixtype, stored_numsats, pvt_hacc_mm / 1000.0f, millis() / 1000.0f);
    } else {
      enqueueLogSdf("GPS FIXED fix=%d sats=%d hAcc=%.1fm", pvt_fixtype, stored_numsats, pvt_hacc_mm / 1000.0f);
    }
  }
  pvt_new = true;
  framelog_add("NAV-PVT fix=%d ok=%d sv=%d hAcc=%.1f", pvt_fixtype, pvt_fixok ? 1 : 0, stored_numsats, pvt_hacc_mm / 1000.0f);
}

// UBX-NAV-SAT（class 0x01, id 0x35）。衛星ごとの仰角・方位・C/N0（GPS 詳細画面の配置図）。
static void handle_navsat(const uint8_t* p, uint16_t len) {
  if (len < 8) return;
  int numSvs = p[5];
  if (8 + numSvs * 12 > len) numSvs = (len - 8) / 12;
  removeStaleSatellites();
  uint8_t n_ana_data = 0, n_ana_orbit = 0, n_eph_orbit = 0;
  for (int s = 0; s < numSvs; s++) {
    const uint8_t* sv = p + 8 + s * 12;
    const uint8_t gnssId = sv[0];
    const uint8_t svId   = sv[1];
    const uint8_t cno    = sv[2];
    const int8_t  elev   = (int8_t)sv[3];
    const int16_t azim   = (int16_t)(sv[4] | (sv[5] << 8));
    const uint32_t flags = (uint32_t)le_i32(sv + 8);
    if (svId == 0) continue;
    // flags bit8-10 orbitSource（1=放送暦 / 4=AssistNow Autonomous）、bit14 aopAvail
    const uint8_t orbitSource = (flags >> 8) & 0x07;
    if (flags & (1UL << 14)) n_ana_data++;
    if (orbitSource == 4) n_ana_orbit++;
    if (orbitSource == 1) n_eph_orbit++;

    int type;
    switch (gnssId) {
      case 0: type = SATELLITE_TYPE_GPS;     break;
      case 2: type = SATELLITE_TYPE_GALILEO; break;
      case 3: type = SATELLITE_TYPE_BEIDOU;  break;
      case 5: type = SATELLITE_TYPE_QZSS;    break;
      case 6: type = SATELLITE_TYPE_GLONASS; break;
      default: type = SATELLITE_TYPE_UNKNOWN; break;
    }
    // 同じ番号が衛星系ごとにあるので、番号と衛星系の両方で探す
    for (int j = 0; j < MAX_SATELLITES; j++) {
      if ((satellites[j].PRN == svId && satellites[j].satelliteType == type) || satellites[j].PRN == 0) {
        satellites[j].PRN = svId;
        satellites[j].elevation = (elev >= 0 && elev <= 90) ? elev : satellites[j].elevation;
        satellites[j].azimuth = (azim >= 0 && azim < 360) ? azim : satellites[j].azimuth;
        satellites[j].SNR = cno;
        satellites[j].satelliteType = type;
        satellites[j].lastReceived = millis();
        break;
      }
    }
  }
  ana_data_svs  = n_ana_data;
  ana_orbit_svs = n_ana_orbit;
  eph_orbit_svs = n_eph_orbit;
  if (ana_fix_report_pending) {
    // 最初の測位で何の軌道を使っていたか。ANA が多ければ AssistNow Autonomous が効いた（v7 と同じ記録）。
    ana_fix_report_pending = false;
    enqueueLogSdf("GPS ORBIT after 1st fix: ANA=%d EPH=%d (ANA data %d SVs)",
                                 n_ana_orbit, n_eph_orbit, n_ana_data);
  }
  framelog_add("NAV-SAT svs=%d ANA=%d/%d", numSvs, n_ana_orbit, n_ana_data);
}

// UBX のフレームを 1 バイトずつ組み立てる。チェックサムが合ったフレームだけ解析する。
#define UBX_BUF_SIZE 1024  // NAV-SAT は 8+12×衛星数。80 衛星まで入る
enum { UBX_SYNC1, UBX_SYNC2, UBX_CLASS, UBX_ID, UBX_LEN1, UBX_LEN2, UBX_PAYLOAD, UBX_CKA, UBX_CKB };
static uint8_t  ubx_state = UBX_SYNC1;
static uint8_t  ubx_cls, ubx_id, ubx_ck_a, ubx_ck_b, ubx_rx_ck_a;
static uint16_t ubx_len, ubx_idx;
static uint8_t  ubx_buf[UBX_BUF_SIZE];

static void ubx_ck(uint8_t b) { ubx_ck_a += b; ubx_ck_b += ubx_ck_a; }

static void process_ubx(uint8_t b) {
  switch (ubx_state) {
    case UBX_SYNC1: if (b == 0xB5) ubx_state = UBX_SYNC2; break;
    case UBX_SYNC2: ubx_state = (b == 0x62) ? UBX_CLASS : (b == 0xB5 ? UBX_SYNC2 : UBX_SYNC1); break;
    case UBX_CLASS: ubx_cls = b; ubx_ck_a = 0; ubx_ck_b = 0; ubx_ck(b); ubx_state = UBX_ID; break;
    case UBX_ID:    ubx_id = b; ubx_ck(b); ubx_state = UBX_LEN1; break;
    case UBX_LEN1:  ubx_len = b; ubx_ck(b); ubx_state = UBX_LEN2; break;
    case UBX_LEN2:
      ubx_len |= (uint16_t)b << 8;
      ubx_ck(b);
      ubx_idx = 0;
      // ★ 受信バッファより長いフレームは来ないはず（化けたヘッダ）。長さを信じず同期を取り直す。
      if (ubx_len > UBX_BUF_SIZE) ubx_state = UBX_SYNC1;
      else ubx_state = (ubx_len == 0) ? UBX_CKA : UBX_PAYLOAD;
      break;
    case UBX_PAYLOAD:
      ubx_buf[ubx_idx++] = b;
      ubx_ck(b);
      if (ubx_idx >= ubx_len) ubx_state = UBX_CKA;
      break;
    case UBX_CKA: ubx_rx_ck_a = b; ubx_state = UBX_CKB; break;
    case UBX_CKB:
      ubx_state = UBX_SYNC1;
      if (ubx_rx_ck_a != ubx_ck_a || b != ubx_ck_b) break;  // 化けたフレームは捨てる
      gps_connection = true;
      last_valid_frame_ms = millis();
      time_last_gps_frame = last_valid_frame_ms;
      if (ubx_cls == 0x01 && ubx_id == 0x07) handle_navpvt(ubx_buf, ubx_len);
      else if (ubx_cls == 0x01 && ubx_id == 0x35) handle_navsat(ubx_buf, ubx_len);
      else if (ubx_cls == 0x05) framelog_add("%s %02X-%02X", ubx_id == 0x01 ? "ACK" : "NAK",
                                             ubx_len >= 1 ? ubx_buf[0] : 0, ubx_len >= 2 ? ubx_buf[1] : 0);
      else framelog_add("UBX %02X-%02X len=%d", ubx_cls, ubx_id, ubx_len);
      break;
    default: ubx_state = UBX_SYNC1; break;
  }
}


// ============================================================
// u-blox（SAM-M10Q）の設定
// ============================================================
// 設定は RAM にだけ書く（AssistNow Autonomous の有効・無効だけ BBR にも）。GPS の電源が切れれば
// 工場出荷の 9600bps・1Hz・NMEA に戻るので、gps_setup() は毎回 9600bps で開いて 38400bps・UBX のみ出力に
// 切り替えてから、ここで送り直す。使うのは v7 と同じ旧形式の UBX-CFG（M10 でも ACK が返る。v7 で実機確認済み）。

bool sat_fast = false;       // GPS 詳細画面を開いている間だけ NAV-SAT を 1Hz に（gps_setup() でもこれに合わせる）
int gps_cfg_acked = 0;       // 直近の gps_setup() で ACK が返った設定の数（GPS 詳細画面とログ）
int gps_cfg_total = 0;
bool gps_cfg_log_pending = false;  // gps_setup() の結果を、次の gps_loop() で SD に残す
static bool gps_prt_ack = false;   // 9600bps で送った CFG-PRT に ACK が返ったか（38400bps 済みなら返らない）

// UBX のフレームを組んで送る（チェックサムは class〜payload の Fletcher-8）。
static void ubx_send(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  uint8_t ck_a = 0, ck_b = 0;
  const uint8_t head[4] = { cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  GPS_SERIAL.write(0xB5);
  GPS_SERIAL.write(0x62);
  for (int i = 0; i < 4; i++) { GPS_SERIAL.write(head[i]); ck_a += head[i]; ck_b += ck_a; }
  for (int i = 0; i < len; i++) { GPS_SERIAL.write(payload[i]); ck_a += payload[i]; ck_b += ck_a; }
  GPS_SERIAL.write(ck_a);
  GPS_SERIAL.write(ck_b);
}

// UBX のフレームを 1 つずつ読み、ACK-ACK なら true、ACK-NAK かタイムアウトなら false。
// 間に来る NAV-PVT などは読み捨てる（設定中だけなので構わない）。v7 の ubxWaitAck() と同じ。
static bool ubx_wait_ack(uint32_t timeout_ms) {
  const unsigned long t0 = millis();
  uint8_t state = 0, cls = 0, id = 0;
  uint16_t paylen = 0, payidx = 0;
  while (millis() - t0 < timeout_ms) {
    if (!GPS_SERIAL.available()) continue;
    const uint8_t b = GPS_SERIAL.read();
    switch (state) {
      case 0: state = (b == 0xB5) ? 1 : 0; break;
      case 1: state = (b == 0x62) ? 2 : (b == 0xB5 ? 1 : 0); break;
      case 2: cls = b; state = 3; break;
      case 3: id  = b; state = 4; break;
      case 4: paylen = b; state = 5; break;
      case 5: paylen |= ((uint16_t)b << 8); payidx = 0; state = (paylen > 0) ? 6 : 7; break;
      case 6: if (++payidx >= paylen) state = 7; break;   // ペイロードは読み飛ばす
      case 7: state = 8; break;                            // ck_a
      case 8: state = 0;                                   // ck_b → フレーム完了
              if (cls == 0x05 && id == 0x01) return true;  // ACK-ACK
              if (cls == 0x05 && id == 0x00) return false; // ACK-NAK
              break;
    }
  }
  return false;
}

// UBX の NAV メッセージの出力レート（測位 rate 回に 1 回。0 = 出さない）。UBX-CFG-MSG。
static void ubx_set_nav_rate(uint8_t nav_id, uint8_t rate) {
  const uint8_t p[3] = { 0x01, nav_id, rate };
  ubx_send(0x06, 0x01, p, sizeof(p));
}
#define UBX_NAV_PVT 0x07
#define UBX_NAV_SAT 0x35
#define NAVSAT_RATE_SLOW (10 * GPS_RATE_HZ)  // 10 秒に 1 回（地図の画面）
#define NAVSAT_RATE_FAST (GPS_RATE_HZ)       // 1Hz（GPS 詳細画面）

static bool cfg_step(bool acked) {
  gps_cfg_total++;
  if (acked) gps_cfg_acked++;
  return acked;
}

// 測位レート・出力するメッセージ・ダイナミックモデル・AssistNow Autonomous を設定する。38400bps に切り替えた後に呼ぶ。
// ★ 最初の設定に ACK が返らなければ残りは送らない。GPS が居ないとき（通信断のやり直し）に
//   1 つ 0.3 秒ずつ待って画面を止めないため。
static void gps_configure() {
  gps_cfg_acked = 0;
  gps_cfg_total = 0;
  delay(50);  // ボーレートを切り替えた直後なので少し待つ（v7 と同じ）
  // UBX-CFG-RATE: 測位を 1000/GPS_RATE_HZ ms ごとに（navRate=1, timeRef=GPS）
  const uint16_t meas_ms = 1000 / GPS_RATE_HZ;
  const uint8_t rate[6] = { (uint8_t)(meas_ms & 0xFF), (uint8_t)(meas_ms >> 8), 0x01, 0x00, 0x01, 0x00 };
  ubx_send(0x06, 0x08, rate, sizeof(rate));
  if (!cfg_step(ubx_wait_ack(GPS_ACK_TIMEOUT_MS))) return;

  ubx_set_nav_rate(UBX_NAV_PVT, 1);  // 毎回
  cfg_step(ubx_wait_ack(GPS_ACK_TIMEOUT_MS));
  ubx_set_nav_rate(UBX_NAV_SAT, sat_fast ? NAVSAT_RATE_FAST : NAVSAT_RATE_SLOW);
  cfg_step(ubx_wait_ack(GPS_ACK_TIMEOUT_MS));

  // UBX-CFG-NAV5: ダイナミックモデルを Airborne <1g（6）に。mask=0x0001（dynModel だけ変える）。v7 と同じ。
  uint8_t nav5[36] = { 0x01, 0x00, 0x06 };
  ubx_send(0x06, 0x24, nav5, sizeof(nav5));
  cfg_step(ubx_wait_ack(GPS_ACK_TIMEOUT_MS));

  // UBX-CFG-VALSET: CFG-ANA-USE_ANA（0x10230001）。layers=RAM+BBR（電源を入れた瞬間から効かせるため。v7 と同じ）。
  // ★ GNSS_USE_ANA が 0 のときも「無効」を書く（BBR に残った「有効」を消すため）。
  const uint8_t ana[9] = { 0x00, 0x03, 0x00, 0x00, 0x01, 0x00, 0x23, 0x10, (uint8_t)(GNSS_USE_ANA ? 1 : 0) };
  ubx_send(0x06, 0x8A, ana, sizeof(ana));
  cfg_step(ubx_wait_ack(GPS_ACK_TIMEOUT_MS));
}

//Due compability
void gps_getposition_mode() {
  Serial.println("POS MODE");
  // GPS 詳細画面を閉じたら NAV-SAT を 10 秒に 1 回へ戻す（v7 と同じ）。ACK は待たない。
  sat_fast = false;
  ubx_set_nav_rate(UBX_NAV_SAT, NAVSAT_RATE_SLOW);
}
//Due compability
void gps_constellation_mode() {
  Serial.println("CONST MODE");
  // GPS 詳細画面（衛星の配置）を開いたら NAV-SAT を 1Hz に（v7 と同じ）。ACK は待たない。
  sat_fast = true;
  ubx_set_nav_rate(UBX_NAV_SAT, NAVSAT_RATE_FAST);
}


void gps_setup() {
  last_gps_setup_ms = millis();
  GPS_SERIAL.end();
  GPS_SERIAL.setTX(GPS_TX);
  GPS_SERIAL.setRX(GPS_RX);// Initialize GNSS
  GPS_SERIAL.setFIFOSize(512);
  GPS_SERIAL.begin(9600);

  // UBX-CFG-PRT: UART1 を 38400bps・入出力とも UBX のみにする（NMEA を止める）。v7 と同じバイト列。
  // GPS がすでに 38400bps なら（本体だけ再起動したとき）、9600bps で送ったこれは届かないが、
  // 前回の設定のまま 38400bps・UBX なので問題ない。
  const unsigned char UBLOX_CFG_PRT_38400_UBXONLY[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xC0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x7B,0x54};
  delay (50);// なぜか必要（Do not delete without care.)
  for (unsigned i = 0; i < sizeof(UBLOX_CFG_PRT_38400_UBXONLY); i++) {
    GPS_SERIAL.write(UBLOX_CFG_PRT_38400_UBXONLY[i]);
  }
  // CFG-PRT の ACK は旧ボーレート（9600）で返ってくる。読み捨てながら ACK かどうかだけ見る（v7 と同じ）。
  delay (100);//なぜか必要（Do not delete without care.)
  gps_prt_ack = ubx_wait_ack(200);
  GPS_SERIAL.end();
  delay(50);//なぜか必要（Do not delete without care.)
  GPS_SERIAL.begin(38400);
  ubx_state = UBX_SYNC1;
  gps_configure();
  // ★ ここでは SD に書かない（起動時は Core1 のキューの用意より先に来うる）。次の gps_loop() で書く。
  gps_cfg_log_pending = true;
  Serial.printf("GPS CFG: %d/%d acked (prt=%d)\n", gps_cfg_acked, gps_cfg_total, gps_prt_ack ? 1 : 0);

  for(int i = 0; i < GPS_FRAMELOG_LINES;i++){
    framelog[i][0] = 0;
  }
}

void toggle_demo_biwako() {
  demo_biwako = !demo_biwako;
}

bool get_demo_biwako() {
  return demo_biwako;
}

unsigned long last_latlon_manager = 0;
unsigned long last_gps_save_time = 0;

GpsDate get_gpsdate(){
  return pvt_date;
}
GpsTime get_gpstime(){
  return pvt_time;
}


// ============================================================
// リプレイ（SD の飛行 CSV を本体で再生する。v7 の 0.92）
// ============================================================
// Core1（mysd.cpp）が CSV を読んでリングに積み、ここで CSV の時刻どおりに 1 行ずつ取り出して、
// NAV-PVT を受けたのと同じように位置・速度・進路・精度を入れる。表示・誘導・Auto 10km はそのまま動く。
// ★ リプレイ中は: 実際の GPS の位置を使わない（handle_navpvt）・飛行 CSV に書かない（on_new_pvt）・
//   Auto 10km のフェーズを SD に残さない（GPS_TFT_map.ino。デモと同じ「仮想の位置」扱い）。
static bool rp_init_pending = false;   // ファイルを開き直して、Core1 が終えるのを待っている
static bool rp_init_queued = false;    // そのための TASK_REPLAY_INIT を積めた（キューが満杯なら次の回に積む）
static uint32_t rp_init_sent = 0;      // 積めた TASK_REPLAY_INIT の数。Core1 の replay_init_seq がこれに
                                       // 追いつけば、積んだものが全部終わっている（途中の古い分で走り出さない）
static char rp_name[REPLAY_NAME_LEN] = "";
static int rp_speed = 1;
static uint32_t rp_clock = 0, rp_last_tick = 0;
static int rp_selected = -1;           // 設定画面で選んでいる番号（-1 = OFF）

bool replay_active() { return rp_active; }
int replay_get_speed() { return rp_speed; }
void replay_cycle_speed() { rp_speed = (rp_speed == 1) ? 4 : (rp_speed == 4) ? 16 : 1; }
void replay_select_next() {
  const int n = replay_file_count();
  rp_selected = (rp_selected + 1 >= n) ? -1 : rp_selected + 1;   // OFF → 1 つ目 → … → 最後 → OFF
}
void replay_select_none() { rp_selected = -1; }
const char* replay_selected_name() { return rp_selected >= 0 ? replay_file_name(rp_selected) : nullptr; }
bool replay_selected_is_playing() {
  const char* n = replay_selected_name();
  return rp_active && n != nullptr && strcmp(n, rp_name) == 0;
}

// 再生する飛行の値と今の値を混ぜない（航跡・最大 G/S・測位の時刻）
static void replay_reset_view() {
  latlon_manager.reset();
  maxgs_reset();
  last_fix_ms = 0;
  // Auto 10km のフェーズも実際の値に戻す（再生中は再生している飛行の位置で動く）
  auto10k_status = auto10k_status_flight;
}

static void replay_start(const char* name) {
  strncpy(rp_name, name, REPLAY_NAME_LEN - 1);
  rp_name[REPLAY_NAME_LEN - 1] = '\0';
  demo_biwako = false;   // デモとリプレイは同時に使わない
  rp_active = true;
  rp_init_pending = true;
  rp_init_queued = false;   // replay_pump() が積む
  replay_reset_view();
}

static void replay_stop() {
  if (!rp_active) return;
  rp_active = false;
  rp_init_pending = false;
  rp_selected = -1;
  if (enqueueTask(createReplayInitTask(""))) rp_init_sent++;   // ファイルを閉じる
  replay_reset_view();
}

void replay_apply_selection() {
  const char* n = replay_selected_name();
  if (n == nullptr) { replay_stop(); return; }
  if (rp_active && strcmp(n, rp_name) == 0) return;   // 同じファイルを再生中ならそのまま
  replay_start(n);
}

// CSV の日時は JST。画面の時刻は UTC に 9 時間足して出すので、UTC に戻して入れる。
static void jst_to_utc(GpsDate& d, GpsTime& t) {
  t.h -= 9;
  if (t.h >= 0) return;
  t.h += 24;
  if (!d.valid) return;
  if (--d.d >= 1) return;
  if (--d.mo < 1) { d.mo = 12; d.y--; }
  static const int dim[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  const bool leap = (d.y % 4 == 0 && d.y % 100 != 0) || d.y % 400 == 0;
  d.d = dim[d.mo - 1] + ((d.mo == 2 && leap) ? 1 : 0);
}

static void apply_replay_row(const ReplayRow& r) {
  stored_latitude = r.latitude;
  stored_longitude = r.longitude;
  if (!isnan(r.gs)) stored_gs = r.gs;
  if (!isnan(r.truetrack)) {
    double trk = fmod(r.truetrack, 360.0);
    if (trk < 0) trk += 360.0;
    stored_truetrack = trk;
  }
  if (!isnan(r.altitude)) stored_altitude = r.altitude;
  if (r.numsat >= 0) stored_numsats = r.numsat;
  // 精度の列が無い古い CSV は「良い」とみなす（三角の自機で出す。0.502 までは精度を記録していなかった）
  pvt_hacc_mm = isnan(r.hacc) ? 1000 : (uint32_t)(r.hacc * 1000.0f);
  pvt_sacc_mmps = isnan(r.sacc) ? 100 : (uint32_t)(r.sacc * 1000.0f);
  pvt_fixtype = 3;
  pvt_fixok = true;
  pvt_pos_valid = true;
  GpsDate d;
  GpsTime t;
  d.valid = r.year >= 2020 && r.month >= 1 && r.month <= 12 && r.day >= 1 && r.day <= 31;
  d.y = r.year; d.mo = r.month; d.d = r.day;
  t.valid = true;
  t.h = r.hour; t.mi = r.minute; t.s = r.second;
  jst_to_utc(d, t);
  pvt_date = d;
  pvt_time = t;
  last_fix_ms = millis();
  pvt_new = true;
}

// gps_loop() から毎回呼ぶ。再生の時計（実時間 × 倍率）に達した行を取り出して入れる。
static void replay_pump() {
  if (!rp_active) return;
  if (rp_init_pending) {
    if (!rp_init_queued) {
      if (!enqueueTask(createReplayInitTask(rp_name))) return;   // キューが満杯。次の回に積む
      rp_init_queued = true;
      rp_init_sent++;
    }
    if ((int32_t)(replay_init_seq - rp_init_sent) < 0) return;   // Core1 がまだ開き終えていない
    rp_init_pending = false;
    rp_clock = 0;
    rp_last_tick = millis();
    if (replay_bad) {
      enqueueLogSdf("REPLAY canceled: %s", rp_name);
      replay_stop();
      return;
    }
  }
  const uint32_t now = millis();
  rp_clock += (now - rp_last_tick) * (uint32_t)rp_speed;
  rp_last_tick = now;
  while (replay_tail != replay_head) {
    __sync_synchronize();   // head を見てから行を読む（Core1 は行を書いてから head を進めている）
    const ReplayRow& r = replay_rows[replay_tail];
    if (r.t_ms > rp_clock) break;
    apply_replay_row(r);
    replay_tail = (replay_tail + 1) & (REPLAY_BUF_SIZE - 1);
  }
  const uint8_t used = (uint8_t)((replay_head - replay_tail) & (REPLAY_BUF_SIZE - 1));
  if (!replay_load_pending && !replay_eof && used < REPLAY_BUF_SIZE / 2) {
    // ★ 積めなかったときに「積んだ」のままにすると、二度と読み足さずに再生が止まる
    replay_load_pending = true;
    if (!enqueueTask(Task{ TASK_REPLAY_LOAD })) replay_load_pending = false;
  }
  // 最後まで再生したら先頭から（v7 と同じくループ再生）
  if (replay_eof && used == 0 && !replay_load_pending) {
    rp_init_pending = true;
    rp_init_queued = false;
    replay_reset_view();
  }
}

// 新しい NAV-PVT を受けた回の後処理（最大 G/S・航跡・飛行 CSV・精度のログ）。
static void on_new_pvt() {
  if (!pvt_pos_valid) return;
  maxgs_update(stored_gs);
  if (!pvt_date.valid || !pvt_time.valid) return;

  int tracklog_interval = constrain(50000/(1.0+stored_gs/2.0), 1000, 15000);//約50mおきに一回記録するような計算となる。
  if (rp_active) tracklog_interval /= replay_get_speed();   // 早送りでも航跡の点の間隔（距離）が同じになるように
  if(millis() - last_latlon_manager > (unsigned long)tracklog_interval){
    latlon_manager.addCoord({ (float)stored_latitude, (float)stored_longitude });
    last_latlon_manager = millis();
  }
  // ★ リプレイ中は飛行 CSV に書かない（再生した過去の飛行を今日のログに混ぜない。v7 と同じ）
  if (rp_active) return;
  // 測位は GPS_RATE_HZ（2Hz）で来るので、「1000ms より後」だと 1.5 秒おきになる。少し手前で区切る。
  if(millis() - last_gps_save_time >= 900){
    CsvRow row;
    row.latitude  = stored_latitude;
    row.longitude = stored_longitude;
    row.gs        = stored_gs;
    row.truetrack = stored_truetrack;
    row.altitude  = stored_altitude;
    // 電池電圧。USB はあるが SW OFF（電池が切り離されている）・電源の異常のあいだは読めないので空欄（v7 と同じ）
    const BattState bs = power_battery_state();
    row.voltage   = (bs == BATT_OFF || bs == BATT_FAULT) ? NAN : power_battery_volts();
    row.numsat    = stored_numsats;
    row.hacc      = pvt_hacc_mm / 1000.0f;
    row.sacc      = pvt_sacc_mmps / 1000.0f;
    row.year = pvt_date.y; row.month = pvt_date.mo; row.day = pvt_date.d; row.hour = pvt_time.h;
    utcToJst(&row.year, &row.month, &row.day, &row.hour);
    row.minute = pvt_time.mi;
    row.second = pvt_time.s;
    enqueueTask(createSaveCsvTask(row));
    last_gps_save_time = millis();
  }
}

// return true if new position arrived. Hardware Serialの受信を行う。
// 新しい NAV-PVT を受けた直後に true を返す（そこで地図を描けば、次の測位まで約 0.5 秒あく）。
bool gps_loop() {
  static unsigned long last_overflow_log = 0;
  while (GPS_SERIAL.available() > 0) {
    // FIFO があふれた（描画などで読み出しが遅れた）。UBX はチェックサムで弾けるので値は化けないが、記録は残す。
    if (GPS_SERIAL.overflow() && millis() - last_overflow_log > 10000) {
      last_overflow_log = millis();
      Serial.println("!!WARNING!! GPS FIFO overflow");
      enqueueTask(createLogSdTask("!!WARNING!! GPS FIFO overflow"));
    }
    process_ubx(GPS_SERIAL.read());
  }

  // gps_setup() の結果を SD に残す（gps_setup() の中では書けないので、ここで）。
  if (gps_cfg_log_pending) {
    gps_cfg_log_pending = false;
    enqueueLogSdf("GPS CFG %d/%d acked (prt=%s, %dHz, Air1g, ANA=%d)", gps_cfg_acked, gps_cfg_total,
                                 gps_prt_ack ? "ack" : "none", GPS_RATE_HZ, GNSS_USE_ANA);
  }

  // 通信が戻った
  if (gps_resetup_logged && get_gps_connection()) {
    gps_resetup_logged = false;
    enqueueTask(createLogSdTask("GPS data resumed"));
  }

  // 通信断が続いたら設定をやり直す（GPS だけ電源が瞬断して 9600bps・NMEA に戻った、など）。
  // 起動から一度もつながらないときも同じ。gps_setup() は約 0.5 秒止まるので間隔を空ける。
  {
    const unsigned long now = millis();
    const bool no_data = !gps_connection || now - last_valid_frame_ms >= GPS_RESETUP_MS;
    if (no_data && now - last_gps_setup_ms >= GPS_RESETUP_MS) {
      Serial.println("GPS: no valid UBX frame. GPS SETUP again");
      if (!gps_resetup_logged) {
        gps_resetup_logged = true;
        enqueueTask(createLogSdTask("GPS no data. setup again"));
      }
      gps_setup();
    }
  }

  // 精度を定期的に残す（v7 は 10 秒ごと。v3 は SD の書き込みを減らして 60 秒ごと）
  static unsigned long last_acc_log = 0;
  if (gps_connection && !rp_active && millis() - last_acc_log >= GPS_ACC_LOG_MS) {
    last_acc_log = millis();
    enqueueLogSdf("ACC hAcc=%.1fm sAcc=%.2fm/s fix=%d ok=%d sats=%d pDOP=%.1f",
                                 pvt_hacc_mm / 1000.0f, pvt_sacc_mmps / 1000.0f, pvt_fixtype, pvt_fixok ? 1 : 0,
                                 stored_numsats, pvt_pdop);
  }

  //simulated.
  #ifndef RELEASE_GPS
    int tracklog_interval = constrain(50000/(1+get_gps_mps()), 1000, 15000);//約50mおきに一回記録するような計算となる。
    if(millis() - last_latlon_manager > (unsigned long)tracklog_interval){
      latlon_manager.addCoord({ (float)get_gps_lat(), (float)get_gps_lon() });
      last_latlon_manager = millis();
    }
  #endif

  replay_pump();

  if (pvt_new) {
    pvt_new = false;
    on_new_pvt();
    return true;
  }
  return false;
}




double get_gps_lat() {
  if (demo_biwako) {
    int timeelapsed = millis() % 200000;
    return PLA_LAT + timeelapsed / 16000.0 / 1000.0 - 0.002;
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
  return PLA_LAT + stored_latitude - SHINURA_LAT;
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
  if (demo_biwako) {
    int timeelapsed = millis() % 200000;
    return PLA_LON - timeelapsed / 1600.0 / 1000.0 + 0.025;
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
  return PLA_LON + stored_longitude - SHINURA_LON;
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
  #ifndef RELEASE_GPS
    return 8 + 4 * sin(millis() / 1500.0);
  #endif
  if (demo_biwako) {
    return 2 + 2 * sin(millis() / 1500.0);
  }
  return stored_gs;
}


// 直近 GPS_DATA_TIMEOUT_MS にチェックサムの合う UBX のフレームを受けているか。
bool get_gps_connection() {
  return gps_connection && millis() - last_valid_frame_ms < GPS_DATA_TIMEOUT_MS;
}
// 直近 GPS_FIX_TIMEOUT_MS に有効な位置を受けているか。
// （NAV-PVT は測位していなくても毎回来るので、通信が止まったときも含めて「最後に有効だった時刻」で見る）
bool get_gps_fix() {
  return last_fix_ms != 0 && millis() - last_fix_ms < GPS_FIX_TIMEOUT_MS;
}
// 画面の位置・速度・方位が今の値か。デモとシミュレーションは GPS と無関係に動くので常に true。
bool get_gps_live() {
#ifndef RELEASE_GPS
  return true;
#else
  return demo_biwako || get_gps_fix();
#endif
}
// 最後に有効な位置を受けてからの秒数。一度も測位していなければ -1。
int get_gps_fix_age_s() {
  if (last_fix_ms == 0) return -1;
  return (int)((millis() - last_fix_ms) / 1000);
}
bool get_gps_fix3d() {
  return get_gps_fix() && pvt_fixok && pvt_fixtype == 3;
}
int get_gps_fixtype() { return pvt_fixtype; }
float get_gps_hacc_m() {
  return get_gps_fix() ? pvt_hacc_mm / 1000.0f : -1.0f;
}
float get_gps_sacc_mps() {
  return get_gps_fix() ? pvt_sacc_mmps / 1000.0f : -1.0f;
}
float get_gps_pdop() { return pvt_pdop; }
int get_gps_ana_data_svs() { return ana_data_svs; }
int get_gps_ana_orbit_svs() { return ana_orbit_svs; }

// ----- 最大 G/S（v7 と同じく、起動からの最大とその時刻、直近 5 分の最大） -----
// 直近 5 分は 10 秒ごとの箱 30 個の最大で持つ。実際の GPS の値だけを見る（デモは入れない）。
#define MAXGS_BUCKET_MS 10000UL
#define MAXGS_BUCKETS   ((int)(MAXGS_WINDOW_MS / MAXGS_BUCKET_MS))
static float    maxgs_all = 0.0f;
static int      maxgs_all_hhmm = -1;            // 起動からの最大を出した時刻（JST の時*100+分）
static float    maxgs_bucket[MAXGS_BUCKETS];
static uint32_t maxgs_bucket_no[MAXGS_BUCKETS]; // その箱が何番目の 10 秒か（古い箱を見分ける）

static void maxgs_reset() {
  maxgs_all = 0.0f;
  maxgs_all_hhmm = -1;
  for (int i = 0; i < MAXGS_BUCKETS; i++) { maxgs_bucket[i] = 0.0f; maxgs_bucket_no[i] = 0; }
}

static void maxgs_update(float gs) {
  // 精度が悪いときの速度は拾わない（起動直後や反射で飛んだ値で最大が埋まらないように）
  if (pvt_hacc_mm / 1000.0f >= GPS_ACC_SHOW_M || pvt_sacc_mmps / 1000.0f > MAXGS_SACC_MAX_MPS) return;
  const uint32_t no = millis() / MAXGS_BUCKET_MS + 1;  // 0 は「空」に使う
  const int i = no % MAXGS_BUCKETS;
  if (maxgs_bucket_no[i] != no) { maxgs_bucket_no[i] = no; maxgs_bucket[i] = 0.0f; }
  if (gs > maxgs_bucket[i]) maxgs_bucket[i] = gs;
  if (gs > maxgs_all) {
    maxgs_all = gs;
    if (pvt_time.valid) maxgs_all_hhmm = ((pvt_time.h + 9) % 24) * 100 + pvt_time.mi;
  }
}
float get_maxgs_all() { return maxgs_all; }
int get_maxgs_all_hhmm() { return maxgs_all_hhmm; }
float get_maxgs_recent() {
  const uint32_t now_no = millis() / MAXGS_BUCKET_MS + 1;
  float m = 0.0f;
  for (int i = 0; i < MAXGS_BUCKETS; i++) {
    if (maxgs_bucket_no[i] != 0 && now_no - maxgs_bucket_no[i] < (uint32_t)MAXGS_BUCKETS && maxgs_bucket[i] > m)
      m = maxgs_bucket[i];
  }
  return m;
}

double get_gps_altitude() {
  return stored_altitude;
}


double get_gps_truetrack() {
  #ifdef DEBUG_GPS_SIM_SHINURA
    return 40 + (38.5 + sin(millis() / 2100.0)) * sin(millis() / 3000.0);
  #endif
  if (demo_biwako) {
    return 280 + (8.5 + sin(millis() / 2100.0)) * sin(millis() / 3000.0);
  }
  return stored_truetrack;     // Heading in degrees;
}



int get_gps_numsat() {
  return stored_numsats;
}
