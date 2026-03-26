// ============================================================
// File    : gps.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : GPS受信・解析の実装（Quectel LC86G向けに最適化）。
//           TinyGPS++を使ったNMEA解析、衛星情報収集(GSV)、
//           位置・速度・時刻の取得、リプレイ/デモモード管理、
//           フライトログCSVへの定期保存トリガー。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/03/23
// ============================================================
// Handle GNSS modules. Currently optimized for LC86GPAMD.
#include <Arduino.h>
#include <TinyGPS++.h>  // リプレイモード（SD の NMEA 再生）用に引き続き使用

#include "gps.h"
#include "mysd.h"
#include "navdata.h"
#include "settings.h"
#include "display_tft.h"
#include "airdata.h"
#include "imu.h"
#include "mysd.h"
#include "display_tft.h"
#include "gps.h"

// GPS_TFT_map.ino で定義されている USERLED 永続点灯フラグ（致命エラー時に true にする）
extern volatile bool userled_forced_on;

// Create a UBLOX instance
TinyGPSPlus gps;

// GPS時刻を最後に受信したときのmillis()（Euler角ログの時刻推定用）
static uint32_t gps_fix_millis = 0;
uint32_t get_gps_fix_millis() { return gps_fix_millis; }

// --- Mediatek GPS 用 NMEA コマンド ---
// PMTK コマンドは Mediatek チップセット GPS モジュールの設定コマンド。
// 文字列末尾の *XX はチェックサム。
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"                                          // SBAS（補強システム）を有効化
#define PMTK_SET_NMEA_OUTPUT_GSVONLY "$PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29"  // GSV（衛星情報）のみ出力
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  // RMC+GGA（位置・速度・時刻）出力
#define PMTK_SET_NMEA_UPDATE_2HZ "$PMTK220,500*2B"    // 更新レート 2Hz
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"   // 更新レート 1Hz

// --- Quectel GPS 用 PAIR コマンド ---
// LC86GPAMD 等 Quectel チップ向けの独自拡張コマンド。
#define PAIR_SET_38400 "$PAIR864,0,0,38400*23"    // ボーレートを 38400 に変更（モジュール再起動が必要）
#define PAIR_DISABLE_GSV "$PAIR062,3,0*3D"        // GSV 文（衛星情報）の出力を停止
#define PAIR_ENABLE_GSV "$PAIR062,3,1*3C"         // GSV 文の出力を再開
#define PAIR_DISABLE_GSA "$PAIR062,2,0*3C"        // GSA 文（測位精度）の出力を停止
#define PAIR_ENABLE_GSA "$PAIR062,2,1*3D"         // GSA 文の出力を再開
#define PAIR_SET_38400 "$PAIR864,0,0,38400*23"
//#define PQTM_OFF "$PQTMCFGMSGRATE,W,PQTMANTENNASTATUS,0,2*39"


// --- u-blox UBX バイナリ NAV メッセージ設定 ---
// チェックサムは Fletcher-8（class〜payloadの全バイトに対して計算）
//
// UBX-CFG-PRT: UART1 を 38400bps・UBX プロトコルのみ（NMEA 無効）に設定
//   inProtoMask=0x0001(UBX only), outProtoMask=0x0001(UBX only)
#define UBLOX_CFG_PRT_38400_UBXONLY {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xC0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x7B,0x54}
// UBX-CFG-MSG: NAV-PVT(01 07) rate=1（毎測位で送信 → 2Hz）
#define UBLOX_ENABLE_NAVPVT  {0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x07,0x01,0x13,0x51}
// UBX-CFG-MSG: NAV-SAT(01 35) rate=2(1Hz) / rate=20(0.1Hz=10秒に1回)
#define UBLOX_NAVSAT_RATE_2  {0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x35,0x02,0x42,0xAE}
#define UBLOX_NAVSAT_RATE_20 {0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x35,0x14,0x54,0xC0}
// UBX-CFG-MSG: NAV-DOP(01 04) rate=2（1Hz）
#define UBLOX_ENABLE_NAVDOP  {0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x04,0x02,0x11,0x4C}
// UBX-CFG-NAV5: ダイナミックモデルを Airborne <1g（dynModel=6）に設定
//   mask=0x0001（dynModel のみ変更）、payload 36 bytes
#define UBLOX_CFG_NAV5_AIRBORNE1G \
  {0xB5,0x62,0x06,0x24,0x24,0x00, \
   0x01,0x00,0x06,0x00, \
   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, \
   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, \
   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, \
   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, \
   0x55,0xB4}

// --- GPS 最新値の保持変数 ---
// TinyGPS++ から取り出した値をここに保存し、getter 関数経由で他モジュールに公開する。
double stored_longitude, stored_latitude, stored_truetrack, stored_altitude, stored_fixtype, stored_gs;
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
bool gps_connection = false;  // GPS モジュールから1文字でも受信したら true
bool demo_biwako = false;     // 琵琶湖デモモード（GPS を使わず仮想位置を生成する）

// NMEAバッファサイズ（parseGSA/parseGSV より前に定義）
const int NMEA_BUFFER_SIZE = 256;

// --- 衛星情報・NMEA ログ ---
SatelliteData satellites[MAX_SATELLITES];  // 最大 MAX_SATELLITES 衛星分のデータを保持（PRN 0 = 空き）
char last_nmea[MAX_LAST_NMEA][NMEA_MAX_CHAR];   // 直近 NMEA 文のリングバッファ
unsigned long last_nmea_time[MAX_LAST_NMEA];     // 各 NMEA 文の受信時刻
unsigned long last_gps_setup_time = 0;   // 最後に gps_setup() を呼んだ時刻（再接続ガード用）
int stored_nmea_index = 0;               // リングバッファの書き込み位置
int readfail_counter = 0;                // 非 ASCII 文字の連続受信カウンタ（ボーレート不一致の検知）
bool new_location_arrived = false;       // 新しい位置情報が届いたことを Core0 に知らせるフラグ
bool newcourse_arrived = false;          // 新しいコース情報が届いたフラグ（毎秒更新）

// --- GSA 解析結果（DOP・フィックスタイプ・使用衛星 PRN） ---
// parseGSA() で更新。複数の GNGSA が届くたびに PDOP/HDOP/VDOP/fixtype を上書きする。
#define GSA_MAX_PRN 12
static int    gsa_fixtype = 1;                // 1=No Fix, 2=2D, 3=3D
static float  gsa_pdop    = 0.0f;             // Position DOP
static float  gsa_hdop    = 0.0f;             // Horizontal DOP
static float  gsa_vdop    = 0.0f;             // Vertical DOP
static int    gsa_prns[GSA_MAX_PRN] = {};     // 測位に使用中の衛星 PRN（0=未使用）
static int    gsa_numsat  = 0;                // 測位使用衛星数

// UBX ハンドラーから参照するため前方宣言
void removeStaleSatellites();

// ============================================================
// UBX バイナリ受信パーサー（実 GPS 受信用）
// リプレイモードは引き続き NMEA + TinyGPS++ を使用する。
// ============================================================
#define UBX_PAYLOAD_BUF_SIZE 768  // NAV-SAT 最大ペイロード (55 SVs×12+8=668) を余裕で収容

// UBX パーサー ステートマシン状態
enum UbxParseState {
  UBX_SYNC1=0, UBX_SYNC2, UBX_CLASS_ST, UBX_ID_ST,
  UBX_LEN1_ST, UBX_LEN2_ST, UBX_PAYLOAD_ST, UBX_CKA_ST, UBX_CKB_ST
};
static UbxParseState ubx_state   = UBX_SYNC1;
static uint8_t  ubx_cls          = 0;   // 受信中フレームのクラス
static uint8_t  ubx_msgid        = 0;   // 受信中フレームの ID
static uint16_t ubx_paylen       = 0;   // ペイロード長（バイト）
static uint16_t ubx_payidx       = 0;   // ペイロード書き込み位置
static uint8_t  ubx_cka_acc      = 0;   // チェックサム計算用 CK_A 累積
static uint8_t  ubx_ckb_acc      = 0;   // チェックサム計算用 CK_B 累積
static uint8_t  ubx_rx_cka       = 0;   // 受信 CK_A 一時保存
static uint8_t  ubx_buf[UBX_PAYLOAD_BUF_SIZE];  // ペイロード受信バッファ

// UBX NAV-PVT から得た GPS データ（get_gpsdate/get_gpstime・ログ保存などで参照）
static bool     ubx_time_valid   = false;  // UTC 時刻フィールドが有効か
static bool     ubx_date_valid   = false;  // UTC 日付フィールドが有効か
static uint16_t ubx_year         = 0;
static uint8_t  ubx_month        = 0;
static uint8_t  ubx_day          = 0;
static uint8_t  ubx_hour         = 0;
static uint8_t  ubx_min          = 0;
static uint8_t  ubx_sec          = 0;
static uint8_t  ubx_cs           = 0;    // センチ秒（nano フィールドから変換）
static bool     ubx_pos_valid    = false; // 有効な 2D/3D フィックスがあるか
static bool     ubx_pvt_valid    = false; // NAV-PVT を1回以上受信したか
static bool     ubx_pvt_updated  = false; // 新しい NAV-PVT が届いたことを gps_loop() に通知
static bool     ubx_sat_updated  = false; // 新しい NAV-SAT が届いた
static uint32_t ubx_hacc_mm      = 0;     // 水平精度推定値（NAV-PVT bytes 40-43、mm 単位）
static uint32_t ubx_vacc_mm      = 0;     // 垂直精度推定値（NAV-PVT bytes 44-47、mm 単位）
static uint32_t ubx_sacc_mmps    = 0;     // 速度精度推定値（NAV-PVT bytes 68-71、mm/s 単位）
static float    ubx_veld_mps     = 0.0f;  // 垂直速度（NAV-PVT velD bytes 56-59、上昇正、m/s）
static bool     ubx_gnssFixOK    = false; // NAV-PVT flags bit0: gnssFixOK（GNSS フィックスが有効かつ品質 OK）

// UBX-NAV-PVT (class=0x01, id=0x07, payload=92 bytes) ハンドラー
// 位置・速度・時刻・衛星数・フィックスタイプをすべて解析して内部変数を更新する。
static void handle_navpvt(const uint8_t *p, uint16_t len) {
  if (len < 84) return;

  // UTC 日時フィールド（valid bits: bit0=validDate, bit1=validTime）
  uint8_t valid  = p[11];
  ubx_date_valid = (valid & 0x01) != 0;
  ubx_time_valid = (valid & 0x02) != 0;
  if (ubx_date_valid) {
    ubx_year  = (uint16_t)(p[4] | (p[5] << 8));
    ubx_month = p[6];
    ubx_day   = p[7];
    // u-blox は起動直後に validDate=1 でも year=2000/month=0/day=0 を返すことがある。
    // 2020 年以前・月日ゼロは未取得と同じく無効とみなす。
    if (ubx_year < 2020 || ubx_month == 0 || ubx_day == 0) {
      ubx_date_valid = false;
    }
  }
  if (ubx_time_valid) {
    ubx_hour  = p[8];
    ubx_min   = p[9];
    ubx_sec   = p[10];
  }
  // nano (int32, ナノ秒) → センチ秒に変換
  int32_t nano = (int32_t)(p[16] | (p[17]<<8) | (p[18]<<16) | (p[19]<<24));
  ubx_cs = (nano >= 0) ? (uint8_t)(nano / 10000000) : 0;
  // ubx_cs が確定した直後のファームウェア時刻を記録する。
  // ubx_pvt_updated ブロックで記録するより遅延が少なく、パケット間の揺れを抑えられる。
  gps_fix_millis = millis();

  // フィックスタイプ・フラグ（u-blox fixType: 0=NoFix,1=DR,2=2D,3=3D,4=GNSS+DR）
  uint8_t fixType   = p[20];
  bool    gnssFixOK = (p[21] & 0x01) != 0;
  uint8_t numSV     = p[23];

  // 位置（1e-7 度単位の int32）
  int32_t lon  = (int32_t)(p[24]|(p[25]<<8)|(p[26]<<16)|(p[27]<<24));
  int32_t lat  = (int32_t)(p[28]|(p[29]<<8)|(p[30]<<16)|(p[31]<<24));
  int32_t hMSL = (int32_t)(p[36]|(p[37]<<8)|(p[38]<<16)|(p[39]<<24));  // mm above MSL

  // 水平・垂直精度推定値（bytes 40-43: hAcc mm、bytes 44-47: vAcc mm）
  ubx_hacc_mm   = (uint32_t)(p[40]|(p[41]<<8)|(p[42]<<16)|((uint32_t)p[43]<<24));
  ubx_vacc_mm   = (uint32_t)(p[44]|(p[45]<<8)|(p[46]<<16)|((uint32_t)p[47]<<24));
  ubx_gnssFixOK = gnssFixOK;

  // 速度（mm/s）・方向（1e-5 度）
  // velD (bytes 56-59): NED Down 方向（正=下降）→ 上昇正に反転して保存
  int32_t velD    = (int32_t)(p[56]|(p[57]<<8)|(p[58]<<16)|(p[59]<<24));
  int32_t gSpeed  = (int32_t)(p[60]|(p[61]<<8)|(p[62]<<16)|(p[63]<<24));
  int32_t headMot = (int32_t)(p[64]|(p[65]<<8)|(p[66]<<16)|(p[67]<<24));
  ubx_veld_mps  = -velD * 1e-3f;  // mm/s 下降正 → m/s 上昇正
  // 速度精度推定値（bytes 68-71: sAcc mm/s）
  ubx_sacc_mmps = (uint32_t)(p[68]|(p[69]<<8)|(p[70]<<16)|((uint32_t)p[71]<<24));

  // pDOP（0.01 スケール）
  uint16_t pDOP = (uint16_t)(p[76] | (p[77]<<8));
  gsa_pdop  = pDOP * 0.01f;

  // GSA 互換フィックスタイプ（1=NoFix, 2=2D, 3=3D）
  {
    static int prev_fixtype = 1;  // 前回のfixtype（遷移検出用）
    int new_fixtype;
    if      (fixType == 2)  new_fixtype = 2;
    else if (fixType >= 3)  new_fixtype = 3;
    else                    new_fixtype = 1;
    // no-fix → fix への遷移タイミングをSDに記録
    if (prev_fixtype <= 1 && new_fixtype >= 2) {
      enqueueTask(createLogSdfTask("GPS FIXED fix=%d sats=%d hAcc=%.1fm",
        new_fixtype, stored_numsats, ubx_hacc_mm / 1000.0f));
    }
    prev_fixtype  = new_fixtype;
    gsa_fixtype   = new_fixtype;
  }

  ubx_pos_valid = (fixType >= 2) && gnssFixOK;
  ubx_pvt_valid = true;

  if (ubx_pos_valid) {
    double new_lat = lat  * 1e-7;
    double new_lon = lon  * 1e-7;
    stored_latitude  = new_lat;
    stored_longitude = new_lon;
    stored_altitude  = hMSL * 1e-3;  // mm → m
  }

  stored_gs         = gSpeed * 1e-3f;  // mm/s → m/s
  double trk        = headMot * 1e-5;  // 1e-5 deg → deg
  if (trk <   0)  trk += 360.0;
  if (trk >= 360) trk -= 360.0;
  stored_truetrack  = trk;
  stored_numsats    = numSV;
  stored_fixtype    = ubx_pos_valid ? 2 : 0;
  gsa_numsat        = numSV;

  ubx_pvt_updated   = true;
}

// UBX-NAV-SAT (class=0x01, id=0x35) ハンドラー
// 各衛星の gnssId, svId, cno, elev, azim を satellites[] 配列に格納する。
// svUsed フラグが立っている衛星の PRN を gsa_prns[] に記録する。
static void handle_navsat(const uint8_t *p, uint16_t len) {
  if (len < 8) return;
  uint8_t numSvs = p[5];
  if ((uint16_t)(8 + numSvs * 12) > len) numSvs = (len - 8) / 12;

  // gsa_prns をリセットして使用中衛星 PRN を再構築
  int prn_idx = 0;
  for (int i = 0; i < GSA_MAX_PRN; i++) gsa_prns[i] = 0;

  for (int s = 0; s < numSvs; s++) {
    const uint8_t *sv = p + 8 + s * 12;
    uint8_t gnssId = sv[0];
    uint8_t svId   = sv[1];
    uint8_t cno    = sv[2];
    int8_t  elev   = (int8_t)sv[3];
    int16_t azim   = (int16_t)(sv[4] | (sv[5] << 8));
    uint32_t flags = (uint32_t)(sv[8]|(sv[9]<<8)|(sv[10]<<16)|(sv[11]<<24));
    bool svUsed    = (flags & 0x08) != 0;  // bit3=svUsed

    if (svId == 0) continue;

    // gnssId → satelliteType 変換
    int satType;
    switch (gnssId) {
      case 0: satType = SATELLITE_TYPE_GPS;     break;
      case 2: satType = SATELLITE_TYPE_GALILEO; break;
      case 3: satType = SATELLITE_TYPE_BEIDOU;  break;
      case 5: satType = SATELLITE_TYPE_QZSS;    break;
      case 6: satType = SATELLITE_TYPE_GLONASS; break;
      default: satType = SATELLITE_TYPE_UNKNOWN; break;
    }

    bool stored = false;
    for (int j = 0; j < MAX_SATELLITES; j++) {
      if ((satellites[j].PRN == svId && satellites[j].satelliteType == satType) || satellites[j].PRN == 0) {
        satellites[j].PRN          = svId;
        satellites[j].elevation    = (elev >= -90 && elev <= 90) ? elev : satellites[j].elevation;
        satellites[j].azimuth      = (azim >= 0 && azim < 360)   ? azim : satellites[j].azimuth;
        satellites[j].SNR          = (cno <= 99) ? cno : satellites[j].SNR;
        satellites[j].satelliteType = satType;
        satellites[j].lastReceived = millis();
        stored = true;
        break;
      }
    }
    if (!stored) {
      enqueueTask(createLogSdfTask("WARN:sat full svId=%d gnss=%d", svId, gnssId));
    }
    // 使用中衛星を gsa_prns[] に記録（最大 GSA_MAX_PRN 個）
    if (svUsed && prn_idx < GSA_MAX_PRN) gsa_prns[prn_idx++] = svId;
  }
  ubx_sat_updated = true;
}

// UBX-NAV-DOP (class=0x01, id=0x04, payload=18 bytes) ハンドラー
// PDOP / HDOP / VDOP を更新し、10 秒に 1 回 SD ログに保存する。
static void handle_navdop(const uint8_t *p, uint16_t len) {
  if (len < 18) return;
  gsa_pdop = (uint16_t)(p[6]  | (p[7]  << 8)) * 0.01f;
  gsa_hdop = (uint16_t)(p[12] | (p[13] << 8)) * 0.01f;
  gsa_vdop = (uint16_t)(p[10] | (p[11] << 8)) * 0.01f;
  // 10 秒に 1 回 Acc 情報を SD ログに保存（衛星0件=明らかに屋内の場合はスキップ）
  static unsigned long last_dop_log = 0;
  if (millis() - last_dop_log >= 10000) {
    last_dop_log = millis();
    if (stored_numsats > 0) {
      enqueueTask(createLogSdfTask("ACC hAcc=%.1fm vAcc=%.1fm sAcc=%.2fm/s fix=%d sats=%d",
        ubx_hacc_mm / 1000.0f, ubx_vacc_mm / 1000.0f, ubx_sacc_mmps / 1000.0f,
        gsa_fixtype, stored_numsats));
    }
  }
}

// UBX バイナリフレームを 1 バイトずつ処理するステートマシン。
// B5 62 ヘッダーを検出し、クラス・ID・ペイロードを組み立ててチェックサムを確認する。
// チェックサム OK でハンドラーを呼び出し、last_nmea[] にラベルを記録する。
static void process_ubx(uint8_t b) {
  switch (ubx_state) {
    case UBX_SYNC1:
      if (b == 0xB5) ubx_state = UBX_SYNC2;
      break;
    case UBX_SYNC2:
      ubx_state = (b == 0x62) ? UBX_CLASS_ST : UBX_SYNC1;
      break;
    case UBX_CLASS_ST:
      ubx_cls = b;
      ubx_cka_acc = b; ubx_ckb_acc = b;  // Fletcher-8 初期化
      ubx_state = UBX_ID_ST;
      break;
    case UBX_ID_ST:
      ubx_msgid = b;
      ubx_cka_acc += b; ubx_ckb_acc += ubx_cka_acc;
      ubx_state = UBX_LEN1_ST;
      break;
    case UBX_LEN1_ST:
      ubx_paylen = b;
      ubx_cka_acc += b; ubx_ckb_acc += ubx_cka_acc;
      ubx_state = UBX_LEN2_ST;
      break;
    case UBX_LEN2_ST:
      ubx_paylen |= ((uint16_t)b << 8);
      ubx_cka_acc += b; ubx_ckb_acc += ubx_cka_acc;
      ubx_payidx = 0;
      ubx_state  = (ubx_paylen == 0) ? UBX_CKA_ST : UBX_PAYLOAD_ST;
      break;
    case UBX_PAYLOAD_ST:
      if (ubx_payidx < UBX_PAYLOAD_BUF_SIZE) ubx_buf[ubx_payidx] = b;
      ubx_payidx++;
      ubx_cka_acc += b; ubx_ckb_acc += ubx_cka_acc;
      if (ubx_payidx >= ubx_paylen) ubx_state = UBX_CKA_ST;
      break;
    case UBX_CKA_ST:
      ubx_rx_cka = b;  // 受信 CK_A を一時保存
      ubx_state  = UBX_CKB_ST;
      break;
    case UBX_CKB_ST: {
      ubx_state = UBX_SYNC1;
      if (ubx_rx_cka != ubx_cka_acc || b != ubx_ckb_acc) {
        #ifdef DEBUG_GBX_NMEA
        // チェックサム NG: クラス・ID と期待値を出力
        static unsigned long last_ckerr_log = 0;
        if (millis() - last_ckerr_log > 2000) {
          last_ckerr_log = millis();
          Serial.print("[UBX] CK NG cls="); Serial.print(ubx_cls, HEX);
          Serial.print(" id="); Serial.print(ubx_msgid, HEX);
          Serial.print(" len="); Serial.print(ubx_paylen);
          Serial.print(" exp="); Serial.print(ubx_cka_acc, HEX); Serial.print("/"); Serial.print(ubx_ckb_acc, HEX);
          Serial.print(" got="); Serial.print(ubx_rx_cka, HEX); Serial.print("/"); Serial.println(b, HEX);
        }
        #endif
        break;
      }
      // チェックサム OK: タイムスタンプ更新 + ハンドラー呼び出し
      gps_connection = true;
      time_lastnmea  = millis();
      uint16_t used_len = (ubx_paylen < UBX_PAYLOAD_BUF_SIZE) ? ubx_paylen : UBX_PAYLOAD_BUF_SIZE;
      if (ubx_cls == 0x01) {
        if (ubx_msgid == 0x07) {
          // NAV-PVT: 位置・速度・時刻を解析
          handle_navpvt(ubx_buf, used_len);
          snprintf(last_nmea[stored_nmea_index], NMEA_MAX_CHAR,
            "UBX-NAV-PVT fix=%d sv=%d lat=%.5f", gsa_fixtype, stored_numsats, stored_latitude);
          #ifdef DEBUG_GBX_NMEA
          // デバッグ: 5秒に1回 NAV-PVT の主要値を出力
          {
            static unsigned long last_pvt_dbg = 0;
            if (millis() - last_pvt_dbg > 5000) {
              last_pvt_dbg = millis();
              Serial.print("[UBX] NAV-PVT fix="); Serial.print(gsa_fixtype);
              Serial.print(" sv="); Serial.print(stored_numsats);
              Serial.print(" lat="); Serial.print(stored_latitude, 6);
              Serial.print(" lon="); Serial.print(stored_longitude, 6);
              Serial.print(" alt="); Serial.print(stored_altitude, 1);
              Serial.print(" gs="); Serial.print(stored_gs, 2);
              Serial.print("m/s hdg="); Serial.print(stored_truetrack, 1);
              Serial.print(" time="); Serial.print(ubx_hour); Serial.print(":"); Serial.print(ubx_min); Serial.print(":"); Serial.print(ubx_sec);
              Serial.print(" valid="); Serial.print(ubx_date_valid?"D":"d"); Serial.println(ubx_time_valid?"T":"t");
              Serial.print(" hAcc="); Serial.print(ubx_hacc_mm); Serial.print("mm gnssOK="); Serial.println(ubx_gnssFixOK?"Y":"N");
            }
          }
          #endif
        } else if (ubx_msgid == 0x35) {
          // NAV-SAT: 衛星情報を解析
          removeStaleSatellites();
          handle_navsat(ubx_buf, used_len);
          #ifdef DEBUG_GBX_NMEA
          {
            static unsigned long last_sat_dbg = 0;
            if (millis() - last_sat_dbg > 10000) {
              last_sat_dbg = millis();
              Serial.print("[UBX] NAV-SAT numSvs="); Serial.println(ubx_buf[5]);
            }
          }
          #endif
          snprintf(last_nmea[stored_nmea_index], NMEA_MAX_CHAR, "UBX-NAV-SAT svs=%d", ubx_buf[5]);
        } else if (ubx_msgid == 0x04) {
          // NAV-DOP: 精度低下率を解析
          handle_navdop(ubx_buf, used_len);
          #ifdef DEBUG_GBX_NMEA
          {
            static unsigned long last_dop_dbg = 0;
            if (millis() - last_dop_dbg > 10000) {
              last_dop_dbg = millis();
              Serial.print("[UBX] NAV-DOP P="); Serial.print(gsa_pdop, 2);
              Serial.print(" H="); Serial.print(gsa_hdop, 2);
              Serial.print(" V="); Serial.println(gsa_vdop, 2);
            }
          }
          #endif
          snprintf(last_nmea[stored_nmea_index], NMEA_MAX_CHAR,
            "UBX-NAV-DOP P=%.1f H=%.1f V=%.1f", gsa_pdop, gsa_hdop, gsa_vdop);
        } else {
          #ifdef DEBUG_GBX_NMEA
          Serial.print("[UBX] unknown NAV cls=01 id="); Serial.print(ubx_msgid, HEX);
          Serial.print(" len="); Serial.println(ubx_paylen);
          #endif
          snprintf(last_nmea[stored_nmea_index], NMEA_MAX_CHAR,
            "UBX-%02X-%02X len=%d", ubx_cls, ubx_msgid, ubx_paylen);
        }
      } else if (ubx_cls == 0x05) {
        // ACK-ACK (01) / ACK-NAK (00)
        #ifdef DEBUG_GBX_NMEA
        Serial.print("[UBX] "); Serial.print((ubx_msgid==0x01)?"ACK-ACK":"ACK-NAK");
        Serial.print(" for cls="); Serial.print((used_len>=1?ubx_buf[0]:0), HEX);
        Serial.print(" id="); Serial.println((used_len>=2?ubx_buf[1]:0), HEX);
        #endif
        snprintf(last_nmea[stored_nmea_index], NMEA_MAX_CHAR,
          "%s %02X%02X", (ubx_msgid==0x01)?"UBX-ACK-ACK":"UBX-ACK-NAK",
          (used_len>=1?ubx_buf[0]:0), (used_len>=2?ubx_buf[1]:0));
      } else {
        #ifdef DEBUG_GBX_NMEA
        Serial.print("[UBX] frame cls="); Serial.print(ubx_cls, HEX);
        Serial.print(" id="); Serial.print(ubx_msgid, HEX);
        Serial.print(" len="); Serial.println(ubx_paylen);
        #endif
        snprintf(last_nmea[stored_nmea_index], NMEA_MAX_CHAR,
          "UBX-%02X-%02X len=%d", ubx_cls, ubx_msgid, ubx_paylen);
      }
      last_nmea_time[stored_nmea_index] = millis();
      stored_nmea_index = (stored_nmea_index + 1) % MAX_LAST_NMEA;
      break;
    }
    default:
      ubx_state = UBX_SYNC1;
      break;
  }
}

// --- リプレイモード ---
// replaymode_gpsoff = true の時、実 GPS の代わりに SD から読んだ NMEA を再生する。
bool replaymode_gpsoff = false;
unsigned long last_check_nmea_time = 0;


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

// i 番目に古い NMEA 文字列を返す（i=0 が最新）。
// last_nmea はリングバッファなので、stored_nmea_index を基準に逆算してインデックスを求める。
char* get_gps_nmea(int i){
  int index = (stored_nmea_index-1-i)%MAX_LAST_NMEA;
  if(index < 0){
    index += MAX_LAST_NMEA;
  }
  return last_nmea[index];
}

// i 番目に古い NMEA の受信時刻を返す（i=0 が最新）。
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

// GSA（GPS DOP and Active Satellites）NMEA 文をパースして DOP・フィックスタイプ・使用衛星を更新する。
// 形式: $GNGSA,A,3,19,06,17,02,28,09,12,,,,,,,1.33,0.74,1.10*05
//   field[2] = fix type (1=No Fix, 2=2D, 3=3D)
//   field[3..14] = 測位使用衛星 PRN (空フィールドは "")
//   field[15] = PDOP, field[16] = HDOP, field[17] = VDOP（チェックサム付き可）
void parseGSA(char *nmea) {
  // カンマ分割（最大 20 フィールド）
  char buf[NMEA_BUFFER_SIZE];
  strncpy(buf, nmea, NMEA_BUFFER_SIZE - 1);
  buf[NMEA_BUFFER_SIZE - 1] = '\0';
  char *fields[20] = {};
  int nfields = 0;
  char *p = buf;
  fields[nfields++] = p;
  while (*p && nfields < 20) {
    if (*p == ',' || *p == '*') { *p = '\0'; fields[nfields++] = p + 1; }
    p++;
  }
  if (nfields < 18) return;  // フィールド不足は無視

  // field[2] = fix type
  int ft = atoi(fields[2]);
  if (ft >= 1 && ft <= 3) gsa_fixtype = ft;

  // field[3..14] = PRN（最初の GNGSA で配列を更新）
  gsa_numsat = 0;
  for (int i = 0; i < GSA_MAX_PRN; i++) {
    int prn = atoi(fields[3 + i]);
    gsa_prns[i] = (prn > 0) ? prn : 0;
    if (prn > 0) gsa_numsat++;
  }

  // field[15..17] = PDOP, HDOP, VDOP
  float pd = atof(fields[15]);
  float hd = atof(fields[16]);
  float vd = atof(fields[17]);
  if (pd > 0.0f) gsa_pdop = pd;
  if (hd > 0.0f) gsa_hdop = hd;
  if (vd > 0.0f) gsa_vdop = vd;
}

// ゲッター
int      get_gps_fixtype()    { return gsa_fixtype; }
float    get_gps_pdop()       { return gsa_pdop; }
float    get_gps_hdop()       { return gsa_hdop; }
float    get_gps_vdop()       { return gsa_vdop; }
int      get_gsa_numsat()     { return gsa_numsat; }
int      get_gsa_prn(int i)   { return (i >= 0 && i < GSA_MAX_PRN) ? gsa_prns[i] : 0; }
uint32_t get_gps_hacc_mm()    { return ubx_hacc_mm; }    // hAcc（水平精度推定値, mm）
uint32_t get_gps_vacc_mm()    { return ubx_vacc_mm; }    // vAcc（垂直精度推定値, mm）
uint32_t get_gps_sacc_mmps()  { return ubx_sacc_mmps; }  // sAcc（速度精度推定値, mm/s）
float    get_gps_veld_mps()   { return ubx_veld_mps; }  // GNSS 垂直速度（上昇正, m/s）
bool     get_gps_gnssFixOK()  { return ubx_gnssFixOK; } // gnssFixOK フラグ

// GSV（Satellites in View）NMEA 文を手動パースして satellites[] 配列に衛星情報を格納する。
// TinyGPS++ は GSV を解析しないため、自前でパースする必要がある。
// GSV 文の形式: $GPGSV,総文数,文番号,衛星数,PRN,仰角,方位角,SNR,...*チェックサム
// 衛星種別は文の先頭識別子から判定する（GP=GPS, GL=GLONASS, GA=Galileo, GB=BeiDou）。
void parseGSV(char *nmea) {
  // Print the received NMEA sentence for debugging
  #ifdef DEBUG_GBX_NMEA
  DEBUG_P(20250508,"Received NMEA: ");
  DEBUG_PLN(20250508,nmea);
  #endif

  // Determine the satellite type based on the NMEA sentence identifier
  int satelliteType = SATELLITE_TYPE_UNKNOWN;
  if (strstr(nmea, "$GPGSV")) {
    satelliteType = SATELLITE_TYPE_GPS;
    //satelliteType = SATELLITE_TYPE_QZSS;//LC86GPAMD のとき、Undistinguishable from the GPGSV.
  } else if (strstr(nmea, "$GLGSV")) {
    satelliteType = SATELLITE_TYPE_GLONASS;
  } else if (strstr(nmea, "$GAGSV")) {
    satelliteType = SATELLITE_TYPE_GALILEO;
  } else if (strstr(nmea, "$GBGSV")) {//$BDGSV
    satelliteType = SATELLITE_TYPE_BEIDOU;
  } else if (strstr(nmea, "$GQGSV")) {
    satelliteType = SATELLITE_TYPE_QZSS;  // QZSS専用talker（NMEA 4.11）。PRNは1〜10のままで保存。
  }


  #ifdef DEBUG_GBX_NMEA
  // Print the satellite type for debugging
  DEBUG_P(20250508,"Satellite Type: ");
  DEBUG_PLN(20250508,satelliteType);
  #endif

  
  // Example NMEA GSV sentence: $GPGSV,4,4,14,194,,,,195,,,*7D
  char *p = nmea;

  // Skip past the initial part of the sentence
  p = strchr(p, ','); if (!p) return; p++;
  p = strchr(p, ','); if (!p) return; p++;
  p = strchr(p, ','); if (!p) return; p++;
  p = strchr(p, ','); if (!p) return; p++;

  for (int i = 0; i < 4; i++) {
    if (*p == '*' || *p == '\0') break; // End of sentence or no more data

    // Read satellite PRN number
    int prn = atoi(p);
    p = strchr(p, ','); if (!p) break; p++;

    // Read elevation
    int elevation = (*p != ',' && *p != '*') ? atoi(p) : -1;
    p = strchr(p, ','); if (!p) break; p++;

    // Read azimuth
    int azimuth = (*p != ',' && *p != '*') ? atoi(p) : -1;
    p = strchr(p, ','); if (!p) break; p++;

    // Read SNR (Signal to Noise Ratio)
    int snr = (*p != ',' && *p != '*') ? atoi(p) : -1;
    p = strchr(p, ','); if (!p) break; p++;

    #ifdef DEBUG_GBX_NMEA
    // Debugging: Print parsed values
    DEBUG_P(20250508,"PRN: "); DEBUG_P(20250508,prn);
    DEBUG_P(20250508,", Elevation: "); DEBUG_P(20250508,elevation);
    DEBUG_P(20250508,", Azimuth: "); DEBUG_P(20250508,azimuth);
    DEBUG_P(20250508,", SNR: "); DEBUG_P(20250508,snr);
    #endif

    // Validate parsed values and update satellite data
    if (prn > 0 && prn < 200) {
      bool satellite_stored = false;
      for (int j = 0; j < MAX_SATELLITES; j++) {
        // 複合キー（PRN + satelliteType）で重複検索 — 同PRNでも星座が違えば別スロット
        if ((satellites[j].PRN == prn && satellites[j].satelliteType == satelliteType) || satellites[j].PRN == 0) {
          satellites[j].PRN = prn;
          satellites[j].elevation = (elevation >= 0 && elevation <= 90) ? elevation : satellites[j].elevation;
          satellites[j].azimuth = (azimuth >= 0 && azimuth < 360) ? azimuth : satellites[j].azimuth;
          satellites[j].SNR = (snr >= 0 && snr <= 99) ? snr : satellites[j].SNR;  // 上限チェック（390など異常値を棄却）
          satellites[j].satelliteType = satelliteType;
          if(193 <= prn && prn  <= 199)
            satellites[j].satelliteType = SATELLITE_TYPE_QZSS;
          satellites[j].lastReceived = millis();
          satellite_stored = true;
          break;
        }
      }
      if (!satellite_stored) {
        // MAX_SATELLITES を超過して格納できなかった → SDログ＋シリアル警告
        DEBUGW_P(20250509, "WARN: satellites[] full, dropped PRN=");
        DEBUGW_PLN(20250509, prn);
        enqueueTask(createLogSdfTask("WARN:sat full PRN=%d type=%d", prn, satelliteType));
      }
    } else {
      DEBUGW_P(20250508,"Invalid PRN parsed");
      DEBUGW_P(20250508,"PRN: ");
      DEBUGW_PLN(20250508,prn);
    }
  }
}



// GPS モジュールを「位置取得優先モード」に切り替える。
// GSV（衛星情報）出力を停止することで NMEA の量を減らし、位置・速度の更新レートを上げる。
// 設定画面の GPS 詳細からメインマップへ戻る際に呼ばれる。
void gps_getposition_mode() {
  #ifdef QUECTEL_GPS
    GPS_SERIAL.println(PAIR_DISABLE_GSV);
    delay(3);
    GPS_SERIAL.println(PAIR_DISABLE_GSA);
  #endif
  #ifdef MEDIATEK_GPS
  GPS_SERIAL.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS_SERIAL.println(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  #endif
  #ifdef UBLOX_GPS
  // ナビ画面では NAV-SAT を 0.1Hz（10秒に1回）に落としてトラフィックを抑制
  { const unsigned char cmd[] = UBLOX_NAVSAT_RATE_20;
    for (unsigned i = 0; i < sizeof(cmd); i++) GPS_SERIAL.write(cmd[i]); }
  #endif
}

// GPS モジュールを「星座表示モード」に切り替える。
// GSV を有効化して衛星の仰角・方位・SNR を表示できるようにする。
// GPS 詳細画面に入る際に呼ばれる。
void gps_constellation_mode() {
  #ifdef QUECTEL_GPS
    GPS_SERIAL.println(PAIR_ENABLE_GSV);
    delay(3);
    GPS_SERIAL.println(PAIR_ENABLE_GSA);
  #endif
  #ifdef MEDIATEK_GPS
    GPS_SERIAL.println(PMTK_SET_NMEA_OUTPUT_GSVONLY);
    GPS_SERIAL.println(PMTK_SET_NMEA_UPDATE_1HZ);
  #endif
  #ifdef UBLOX_GPS
  // 星座画面では NAV-SAT を 1Hz に上げて衛星情報をリアルタイム更新
  { const unsigned char cmd[] = UBLOX_NAVSAT_RATE_2;
    for (unsigned i = 0; i < sizeof(cmd); i++) GPS_SERIAL.write(cmd[i]); }
  #endif
}

int setupcounter = 1;  // gps_setup() の呼び出し回数（1=初回、2以降=リトライ）
uint32_t gps_current_baudrate = 0;  // GPS シリアルの現在ボーレート（gps_setup() で更新）
uint32_t get_gps_baudrate() { return gps_current_baudrate; }

// UBX フレームを1パケットずつ読み、ACK-ACK なら true を返す。
// NAV-PVT など非 ACK パケットはヘッダ・ペイロード・チェックサムを
// 丸ごと読み飛ばすため、固定バッファに収まらない問題が起きない。
#ifdef UBLOX_GPS
static bool ubxWaitAck(uint32_t timeout_ms) {
  unsigned long t0 = millis();
  uint8_t  state = 0, cls = 0, id = 0;
  uint16_t paylen = 0, payidx = 0;
  while (millis() - t0 < timeout_ms) {
    if (!GPS_SERIAL.available()) continue;
    uint8_t b = GPS_SERIAL.read();
    switch (state) {
      case 0: state = (b == 0xB5) ? 1 : 0; break;
      case 1: state = (b == 0x62) ? 2 : (b == 0xB5 ? 1 : 0); break;
      case 2: cls = b;   state = 3; break;
      case 3: id  = b;   state = 4; break;
      case 4: paylen = b; state = 5; break;
      case 5: paylen |= ((uint16_t)b << 8); payidx = 0;
              state = (paylen > 0) ? 6 : 7; break;
      case 6: if (++payidx >= paylen) state = 7; break;  // ペイロード読み飛ばし
      case 7: state = 8; break;                           // ck_a
      case 8: state = 0;                                  // ck_b → フレーム完了
              if (cls == 0x05 && id == 0x01) return true;   // ACK-ACK
              if (cls == 0x05 && id == 0x00) return false;  // ACK-NAK
              break;
    }
  }
  return false;  // タイムアウト
}
#endif

// GPS モジュールとのシリアル接続を確立する。
// 初回は settings.h で選択したモジュール種別に合わせて初期化する。
// NMEA が 10 秒届かない場合は gps_loop() から自動的に再呼出しされる。
//
// リトライ時は複数のボーレートを順番に試す:
//   1回目: 設定済みボーレートで直接接続
//   2回目: 115200bps で PAIR コマンドを送り 38400 に切り替えてから接続（Quectel が工場出荷時設定の場合）
//   3回目: 115200 bps で直接接続
//   4回目: 38400 bps で直接接続
void gps_setup() {
  last_gps_setup_time = millis();
  DEBUG_P(20260307,"GPS SETUP:setupcounter=");
  DEBUG_PLN(20260307,setupcounter);

  readfail_counter = 0;
  if(setupcounter != 1){
    GPS_SERIAL.end();
  }
  GPS_SERIAL.setTX(GPS_TX);
  GPS_SERIAL.setRX(GPS_RX);

  //初回SETUP
  if(setupcounter == 1){

    #ifdef QUECTEL_GPS
      DEBUG_PLN(20251025,"QUECTEL 38400");
      GPS_SERIAL.setFIFOSize(1024);//LC86GPAMD Bufferサイズ、128では不足するケースあり。
      GPS_SERIAL.begin(gps_current_baudrate = 38400);
    #elif defined(MEDIATEK_GPS)
      DEBUG_PLN(20251025,"MEDIATEK 38400");
      GPS_SERIAL.println(PMTK_ENABLE_SBAS);
      gps_getposition_mode();
      delay(100);//（Do not delete without care.)
      GPS_SERIAL.println("$PMTK251,38400*27");
      delay(100);//（Do not delete without care.)
      GPS_SERIAL.end();
      delay(50);//（Do not delete without care.)
      GPS_SERIAL.begin(gps_current_baudrate = 38400);
    #elif defined(UBLOX_GPS)
      #ifdef DEBUG_GBX_NMEA
      Serial.println("[UBX] setup start");
      #endif
      GPS_SERIAL.setFIFOSize(1024);  // バッファオーバーフロー防止（デフォルト 32 バイトでは不足）
      GPS_SERIAL.begin(gps_current_baudrate = 9600);  // u-blox 工場デフォルトは 9600bps
      #ifdef DEBUG_GBX_NMEA
      Serial.println("[UBX] opened 9600");
      #endif

      // ① UBX-CFG-PRT: UART1 を 38400bps・UBX 出力のみに変更（NMEA 無効化）
      {
        const unsigned char cmd[] = UBLOX_CFG_PRT_38400_UBXONLY;
        delay(50);  // なぜか必要（Do not delete without care.)
        #ifdef DEBUG_GBX_NMEA
        Serial.print("[UBX] sending CFG-PRT (bytes=");
        Serial.print(sizeof(cmd)); Serial.println("):");
        #endif
        for (unsigned i = 0; i < sizeof(cmd); i++) {
          GPS_SERIAL.write(cmd[i]);
          #ifdef DEBUG_GBX_NMEA
          Serial.print(cmd[i], HEX); Serial.print(" ");
          #endif
        }
        #ifdef DEBUG_GBX_NMEA
        Serial.println();
        #endif
      }
      // CFG-PRT の ACK は旧ボーレート（9600）で返ってくる
      delay(100);
      {
        // ACK を読み捨ててバッファをクリアする（機能的に必要）
        uint8_t ack_buf[32] = {};
        int ack_len = 0;
        unsigned long t = millis();
        while (millis() - t < 300 && ack_len < 32) {
          if (GPS_SERIAL.available()) ack_buf[ack_len++] = GPS_SERIAL.read();
        }
        #ifdef DEBUG_GBX_NMEA
        Serial.print("[UBX] CFG-PRT resp ("); Serial.print(ack_len); Serial.print(" bytes): ");
        for (int i = 0; i < ack_len; i++) { Serial.print(ack_buf[i], HEX); Serial.print(" "); }
        Serial.println();
        bool got_ack = false, got_nak = false;
        for (int i = 0; i <= ack_len - 4; i++) {
          if (ack_buf[i]==0xB5 && ack_buf[i+1]==0x62 && ack_buf[i+2]==0x05) {
            if (ack_buf[i+3] == 0x01) got_ack = true;
            if (ack_buf[i+3] == 0x00) got_nak = true;
          }
        }
        if      (got_ack) Serial.println("[UBX] CFG-PRT: ACK OK");
        else if (got_nak) Serial.println("[UBX] CFG-PRT: NAK!");
        else              Serial.println("[UBX] CFG-PRT: no response (may be OK if baud change happened)");
        #endif
      }
      GPS_SERIAL.end();
      delay(50);   // なぜか必要（Do not delete without care.)
      GPS_SERIAL.begin(gps_current_baudrate = 38400);
      #ifdef DEBUG_GBX_NMEA
      Serial.println("[UBX] reopened 38400");
      #endif

      // ② UBX-CFG-RATE: 測位レートを 2Hz（500ms）に設定
      // Payload: measRate=0x01F4(500ms), navRate=0x0001, timeRef=0x0001(GPS)
      {
        const unsigned char UBLOX_RATE_2HZ[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00,0x0B,0x77};
        delay(50);
        #ifdef DEBUG_GBX_NMEA
        Serial.println("[UBX] sending CFG-RATE 2Hz");
        #endif
        for (unsigned i = 0; i < sizeof(UBLOX_RATE_2HZ); i++) GPS_SERIAL.write(UBLOX_RATE_2HZ[i]);
      }
      {
        // NAV-PVT など既存パケットを読み飛ばしながら ACK を待つ
        bool got_ack = ubxWaitAck(600);
        #ifdef DEBUG_GBX_NMEA
        if      (got_ack) Serial.println("[UBX] CFG-RATE 2Hz: ACK OK");
        else              Serial.println("[UBX] CFG-RATE 2Hz: no ACK");
        #endif
      }

      // ③ NAV-PVT を毎測位（2Hz）で出力
      #ifdef DEBUG_GBX_NMEA
      Serial.println("[UBX] sending ENABLE NAVPVT");
      #endif
      { const unsigned char cmd[] = UBLOX_ENABLE_NAVPVT;
        delay(50);
        for (unsigned i = 0; i < sizeof(cmd); i++) GPS_SERIAL.write(cmd[i]); }
      {
        bool got_ack = ubxWaitAck(600);
        #ifdef DEBUG_GBX_NMEA
        Serial.print("[UBX] NAVPVT enable: "); Serial.println(got_ack ? "ACK OK" : "no ACK");
        #endif
      }

      // ④ NAV-SAT を 0.1Hz（10秒に1回）で出力（ナビ画面用。星座画面では 1Hz に切替）
      #ifdef DEBUG_GBX_NMEA
      Serial.println("[UBX] sending NAVSAT rate=20");
      #endif
      { const unsigned char cmd[] = UBLOX_NAVSAT_RATE_20;
        delay(50);
        for (unsigned i = 0; i < sizeof(cmd); i++) GPS_SERIAL.write(cmd[i]); }
      {
        bool got_ack = ubxWaitAck(600);
        #ifdef DEBUG_GBX_NMEA
        Serial.print("[UBX] NAVSAT enable: "); Serial.println(got_ack ? "ACK OK" : "no ACK");
        #endif
      }

      // ⑤ NAV-DOP を 1Hz で出力
      #ifdef DEBUG_GBX_NMEA
      Serial.println("[UBX] sending ENABLE NAVDOP");
      #endif
      { const unsigned char cmd[] = UBLOX_ENABLE_NAVDOP;
        delay(50);
        for (unsigned i = 0; i < sizeof(cmd); i++) GPS_SERIAL.write(cmd[i]); }
      {
        bool got_ack = ubxWaitAck(600);
        #ifdef DEBUG_GBX_NMEA
        Serial.print("[UBX] NAVDOP enable: "); Serial.println(got_ack ? "ACK OK" : "no ACK");
        #endif
      }
      // ⑥ CFG-NAV5: ダイナミックモデルを Airborne <1g に設定
      // パラグライダー/HPA は加速度が小さく <1g に収まる。
      // 地上制約（altitude snap）を除去し、高度追従精度を改善する。
      #ifdef DEBUG_GBX_NMEA
      Serial.println("[UBX] sending CFG-NAV5 Airborne<1g");
      #endif
      { const unsigned char cmd[] = UBLOX_CFG_NAV5_AIRBORNE1G;
        delay(50);
        for (unsigned i = 0; i < sizeof(cmd); i++) GPS_SERIAL.write(cmd[i]); }
      {
        bool got_ack = ubxWaitAck(600);
        #ifdef DEBUG_GBX_NMEA
        Serial.print("[UBX] CFG-NAV5 Airborne<1g: "); Serial.println(got_ack ? "ACK OK" : "no ACK");
        #endif
      }

      #ifdef DEBUG_GBX_NMEA
      Serial.println("[UBX] setup done");
      #endif
    #else
      GPS_SERIAL.begin(gps_current_baudrate = 9600);
    #endif
    
  }else{
    #if defined(UBLOX_GPS)
    // u-blox リトライ: NMEA/PAIR コマンドは使わず baud 切替のみ試みる。
    // 偶数回=9600（工場出荷デフォルト）、奇数回=38400（CFG-PRT 設定済み想定）で交互に試す。
    if(setupcounter % 2 == 0){
      DEBUG_PLN(20251025,"UBX retry: 9600bps (factory default)");
      GPS_SERIAL.setFIFOSize(1024);
      GPS_SERIAL.begin(gps_current_baudrate = 9600);
    }else{
      DEBUG_PLN(20251025,"UBX retry: 38400bps (UBX mode)");
      GPS_SERIAL.setFIFOSize(1024);
      GPS_SERIAL.begin(gps_current_baudrate = 38400);
    }
    #else
    //2nd try
    if(setupcounter%3 == 1){
      DEBUG_PLN(20251025,"from 115200 to 38400 (For QUECTEL)");
      GPS_SERIAL.setFIFOSize(1024);
      GPS_SERIAL.begin(gps_current_baudrate = 115200);
      GPS_SERIAL.println(PAIR_SET_38400);//Need restart of LC86G module.
      delay (100);//なぜか必要（Do not delete without care.)
      GPS_SERIAL.end();
      delay(50);//なぜか必要（Do not delete without care.)
      GPS_SERIAL.setFIFOSize(1024);
      GPS_SERIAL.begin(gps_current_baudrate = 38400);
    }else if(setupcounter%3 == 2){
      DEBUG_PLN(20251025,"Simple setup try 115200");
      //4th try
      GPS_SERIAL.begin(gps_current_baudrate = 115200);
    }else if(setupcounter%3 == 3){
      DEBUG_PLN(20251025,"Simple setup try 38400");
      //3rd try
      GPS_SERIAL.begin(gps_current_baudrate = 38400);
    }else{
      DEBUG_PLN(20251025,"Simple setup try 9600");
      //3rd try
      GPS_SERIAL.begin(gps_current_baudrate = 9600);
    }
    #endif
  }

  for(int i = 0; i < MAX_LAST_NMEA;i++){
    last_nmea[i][0] = 0;
    last_nmea_time[i] = millis();
  }

  //効果なし 調査中
  //GPS_SERIAL.println(PQTM_OFF);

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
// gps.time が有効な場合は JST 時刻を記録し、無効な場合は 0:00 として速度値だけ更新する。
static void update_maxgs(float gs) {
  int jst_h = 0, jst_m = 0;
  // UBX モード・リプレイモード共に ubx_hour/ubx_min を参照（両モードとも更新済み）
  if (ubx_time_valid) {
    jst_h = (ubx_hour + 9) % 24;
    jst_m = ubx_min;
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

void toggle_demo_biwako() {
  demo_biwako = !demo_biwako;
  reset_maxgs();  // モード切替時に最大 G/S をリセット
}

bool get_demo_biwako() {
  return demo_biwako;
}

void set_demo_biwako(bool biwakomode){
  demo_biwako = biwakomode;
  reset_maxgs();  // モード切替時に最大 G/S をリセット
}

unsigned long last_latlon_manager = 0;
unsigned long last_gps_save_time = 0;
unsigned long last_gps_time = 0;// last position update time.
unsigned long time_lastnmea = 0;//last nmea time

int index_buffer1 = 0;
char nmea_buffer1[NMEA_BUFFER_SIZE];


// ubx_* 変数（UBX NAV-PVT 解析結果 or リプレイモードでミラーされた値）から GpsDate を返す
GpsDate get_gpsdate(){
  GpsDate d;
  d._year  = ubx_year;
  d._month = ubx_month;
  d._day   = ubx_day;
  d._valid = ubx_date_valid;
  return d;
}
// ubx_* 変数から GpsTime を返す
GpsTime get_gpstime(){
  GpsTime t;
  t._hour  = ubx_hour;
  t._min   = ubx_min;
  t._sec   = ubx_sec;
  t._cs    = ubx_cs;
  t._valid = ubx_time_valid;
  return t;
}

unsigned long last_demo_gpsupdate = 0;
double demo_biwako_lat = PLA_LAT;
double demo_biwako_lon = PLA_LON;
double demo_biwako_mps = 0;
double demo_biwako_truetrack = 280;

// 飛行軌跡（トラックログ）に現在位置を追加する。
// 追加間隔を速度に応じて調整することで、約 50m おきに記録する。
//   低速（停止近く）→ 最大 15 秒間隔（静止中に大量の点が積まれるのを防ぐ）
//   高速（約 20m/s）→ 最小 1 秒間隔
void add_latlon_track(float lat,float lon){
  int tracklog_interval = constrain(50000/(1.0+stored_gs/2.0), 1000, 15000);//約50mおきに一回記録するような計算となる。
  if(get_demo_biwako()){
    tracklog_interval = 900;  // デモモードは短めの間隔でアニメーション的に記録
  }
  if(millis() - last_latlon_manager > tracklog_interval){
    latlon_manager.addCoord({ lat, lon});
    last_latlon_manager = millis();
  }
}

bool gps_new_location_arrived(){
  if(get_demo_biwako()){
    if(millis() > last_demo_gpsupdate + 1000){
      int biwa_spd = 10;
      demo_biwako_lat += 0.00005*biwa_spd*cos(radians(get_gps_truetrack()));
      if(calculateDistanceKm(demo_biwako_lat,demo_biwako_lon,PLA_LAT,PLA_LON) > 15){
        demo_biwako_lat = PLA_LAT;
        demo_biwako_lon = PLA_LON;
        latlon_manager.reset();
      }
      demo_biwako_lon += 0.00005*biwa_spd*sin(radians(get_gps_truetrack()));
      last_demo_gpsupdate = millis();
      new_location_arrived = true;
      demo_biwako_mps = 7 + sin(millis() / 1500.0);
      update_maxgs(demo_biwako_mps);  // DEMOモードでも最大 G/S を更新
      
      
      int basetrack = 280;
      if(destination_mode == DMODE_AUTO10K && auto10k_status == AUTO10K_INTO)
        basetrack = 100;

      
      int target_angle = basetrack+(20 + 10*sin(millis() / 2100.0)) * sin(millis() / 3000.0)+50*sin(millis() / 10000.0);

      int demo_steer_angle = target_angle - demo_biwako_truetrack;
      if(demo_steer_angle < -180){
        demo_steer_angle += 360;
      }else if(demo_steer_angle > 180){
        demo_steer_angle -= 360;
      }
      demo_biwako_truetrack += demo_steer_angle*0.2*(max(0,sin(millis() / 5000.0)));//basetrack + (10 + 5*sin(millis() / 2100.0)) * sin(millis() / 3000.0)+50*sin(millis() / 10000.0);
      if(demo_biwako_truetrack > 360)
        demo_biwako_truetrack -= 360;
      else if(demo_biwako_truetrack < 0)
        demo_biwako_truetrack += 360;

      newcourse_arrived = true;
      add_latlon_track(demo_biwako_lat, demo_biwako_lon);
    }
  }
  return new_location_arrived;
}

void set_new_location_off(){
  new_location_arrived = false;
}


// GPS から受信した 1 文字を処理する。
// TinyGPS++ に渡して位置・速度・時刻を自動解析させるとともに、
// GSV 文については自前でバッファに蓄積して parseGSV() を呼ぶ。
// '$' で文の先頭を検知してバッファをリセットし、'\n' で1文の終端を検知する。
void process_char(char c){
  gps.encode(c);  // TinyGPS++ に文字を渡す（内部で NMEA を解析する）
  if(c == '$')
    index_buffer1 = 0;  // 新しい NMEA 文の開始 → バッファをリセット

  nmea_buffer1[index_buffer1++] = c;
  if(index_buffer1 >= (NMEA_BUFFER_SIZE-1) || c == '\n'){
    if(index_buffer1 >= 2){
      time_lastnmea = millis();
      nmea_buffer1[index_buffer1-1] = '\0';  // 末尾の改行を null で置換して文字列化
      // 直近 NMEA ログのリングバッファに追記
      strncpy(last_nmea[stored_nmea_index], nmea_buffer1, NMEA_MAX_CHAR - 1);
      last_nmea[stored_nmea_index][NMEA_MAX_CHAR - 1] = '\0';
      last_nmea_time[stored_nmea_index] = millis();
      stored_nmea_index = (stored_nmea_index+1)%MAX_LAST_NMEA;
      #ifdef DEBUG_GBX_NMEA
      DEBUG_P(20260309,index_buffer1);
      DEBUG_PLN(20260309,nmea_buffer1);
      //enqueueTask(createLogSdTask(nmea_buffer1));  // NMEA全文をSDに保存
      #endif
      // GSV/GSA 文は TinyGPS++ が解析しないため、自前でパースする
      if(strstr(nmea_buffer1, "GSV")){
        parseGSV(nmea_buffer1);
      }
      if(strstr(nmea_buffer1, "GSA")){
        parseGSA(nmea_buffer1);
        // GSA 受信時に DOP 情報を 10 秒に 1 回 SD ログに保存する
        static unsigned long last_dop_log = 0;
        if (millis() - last_dop_log >= 10000) {
          enqueueTask(createLogSdfTask("DOP PDOP=%.1f HDOP=%.1f VDOP=%.1f fix=%d sats=%d",
            gsa_pdop, gsa_hdop, gsa_vdop, gsa_fixtype, gsa_numsat));
          last_dop_log = millis();
        }
      }
    }
    index_buffer1 = 0;
  }
}

void toggleReplayMode(){
  replaymode_gpsoff = !replaymode_gpsoff;
  reset_maxgs();  // モード切替時に最大 G/S をリセット
}

bool getReplayMode(){
  return replaymode_gpsoff;
}

void set_replaymode(bool replaymode){
  replaymode_gpsoff = replaymode;
  reset_maxgs();  // モード切替時に最大 G/S をリセット
}
// GPS モジュールからの Serial データを受信・解析するメインループ処理。
// 描画の途中でも複数回呼ばれることで、GPS データの取りこぼしを防ぐ（id は呼び出し箇所識別子）。
//
// 異常検知:
//   - 30 秒間 UBX フレームが届かない → gps_setup() で再接続を試みる
//   - リプレイモードで非 ASCII 文字が 10 回連続 → gps_setup()
//   - FIFO バッファが 256 を超えている → 警告ログを出す
void gps_loop(int id) {

  // 30 秒以上 UBX フレームが途絶えた場合は GPS を再初期化する
  if(GPS_SERIAL.available() == 0 && millis() - get_gps_nmea_time(0) > 30000 && millis() - last_gps_setup_time > 30000){
    DEBUGW_PLN(20250923,"Lost UBX frame for 30 seconds. resetup.");
    gps_setup();
  }
  if(GPS_SERIAL.available() > 256){
    DEBUGW_P(20250923,"ID=");
    DEBUGW_P(20250923,id);
    DEBUGW_P(20250923," Caution, remaining FIFO buffer. avail=");
    DEBUGW_PLN(20250923,GPS_SERIAL.available());
  }

  #ifdef DEBUG_GBX_NMEA
  // 起動後最初の 64 バイトを Hex ダンプ（モジュールが何を出力しているか確認用）
  static int ubx_raw_dump_remain = 64;
  if (ubx_raw_dump_remain > 0 && GPS_SERIAL.available() > 0) {
    Serial.print("[UBX] raw(first64): ");
  }
  #endif

  while (GPS_SERIAL.available() > 0) {
    uint8_t c = GPS_SERIAL.read();
    if(GPS_SERIAL.overflow()){
      DEBUGW_P(20250923,"!!WARNING!! GPS FIFO Overflow avail=");
      DEBUGW_PLN(20250923,GPS_SERIAL.available());
      enqueueTask(createLogSdTask("!!ERROR!! GPS FIFO overflow"));
      userled_forced_on = true;
    }
    if (!replaymode_gpsoff) {
      // UBX バイナリモード: 全バイト値が有効なので ASCII チェック不要
      #ifdef DEBUG_GBX_NMEA
      // 起動後最初の 64 バイトを Hex ダンプ（NMEA混在 or UBX? の確認）
      if (ubx_raw_dump_remain > 0) {
        Serial.print(c, HEX); Serial.print(" ");
        ubx_raw_dump_remain--;
        if (ubx_raw_dump_remain == 0) Serial.println("\n[UBX] raw dump done");
      }
      #endif
      gps_connection = true;
      process_ubx(c);  // UBX バイナリパーサーに渡す
    } else {
      // リプレイモードでは実 GPS シリアルデータを破棄（NMEA 再生は下記で行う）
      if(c >= 128){
        readfail_counter++;
        if(readfail_counter > 10){
          DEBUGW_P(20250923,"Read Failed 10 times, non ascii char:");
          DEBUGW_PLN(20250923,(int)c);
          gps_setup();
        }
      }
      gps_connection = true;
    }
  }

  // リプレイモード: SD から読み込んだ NMEA を 300ms ごとに1文ずつ流す
  if(replaymode_gpsoff){
    if(loaded_replay_nmea){
      for(int i = 0; i < 128; i++){
        process_char(replay_nmea[i]);  // リプレイは NMEA パーサーに渡す
        if(replay_nmea[i] == '\n')
          break;
      }
      loaded_replay_nmea = false;
    }

    if(last_check_nmea_time+300 < millis()){
      last_check_nmea_time = millis();
      enqueueTask(createLoadReplayTask());  // Core1 に次の NMEA 行の読み込みを依頼
    }
  }

  // ============================================================
  // リプレイモード: TinyGPS++ 解析結果を ubx_* 変数にミラーする
  // （try_enque_savecsv / update_maxgs は ubx_* を参照するため）
  // ============================================================
  if (replaymode_gpsoff) {
    if (gps.location.isUpdated()) {
      last_gps_time = millis();
      double new_lat = gps.location.lat();
      double new_lon = gps.location.lng();
      if (stored_latitude != new_lat || stored_longitude != new_lon)
        if (!get_demo_biwako()) new_location_arrived = true;
      stored_latitude  = new_lat;
      stored_longitude = new_lon;
      ubx_pos_valid    = gps.location.isValid();
      stored_fixtype   = ubx_pos_valid ? 2 : 0;
      try_enque_savecsv();
    }
    if (gps.altitude.isUpdated())
      stored_altitude = gps.altitude.meters();
    if (gps.speed.isUpdated()) {
      stored_gs = gps.speed.mps();
      update_maxgs(stored_gs);
    }
    if (gps.course.isUpdated()) {
      if (!get_demo_biwako()) newcourse_arrived = true;
      stored_truetrack = gps.course.deg();
      if (stored_truetrack < 0 || stored_truetrack > 360) {
        enqueueTask(createLogSdfTask("ERR truetrack=%.1f (forced 0)", stored_truetrack));
        stored_truetrack = 0;
      }
    }
    if (gps.satellites.isUpdated()) {
      removeStaleSatellites();
      stored_numsats = gps.satellites.value();
    }
    if (gps.time.isUpdated() && gps.time.isValid()) {
      ubx_hour       = gps.time.hour();
      ubx_min        = gps.time.minute();
      ubx_sec        = gps.time.second();
      ubx_cs         = gps.time.centisecond();
      ubx_time_valid = true;
      gps_fix_millis = millis();
    }
    if (gps.date.isUpdated() && gps.date.isValid()) {
      ubx_year       = gps.date.year();
      ubx_month      = gps.date.month();
      ubx_day        = gps.date.day();
      ubx_date_valid = true;
    }
    // 初回 GPS 時刻取得時の SD ログ（1回だけ）
    if (gps.time.isUpdated() && gps.date.isValid() && gps.time.isValid()) {
      static bool first_time_logged_replay = false;
      if (!first_time_logged_replay) {
        first_time_logged_replay = true;
        enqueueTask(createLogSdfTask("GPS TIME(REPLAY): %04d-%02d-%02d %02d:%02d:%02d UTC",
          gps.date.year(), gps.date.month(), gps.date.day(),
          gps.time.hour(), gps.time.minute(), gps.time.second()));
      }
    }
    return;
  }

  // ============================================================
  // 通常モード (UBX): handle_navpvt() がセットした ubx_pvt_updated フラグを処理する
  // ============================================================
  if (ubx_pvt_updated) {
    ubx_pvt_updated = false;
    last_gps_time = millis();
    if (!get_demo_biwako()) {
      if (ubx_pos_valid) new_location_arrived = true;
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
    // gps_fix_millis は handle_navpvt() 内で ubx_cs 確定直後に記録済みのためここでは不要。

    // 初回 GPS 時刻取得時の SD ログ（1回だけ）
    if (ubx_time_valid && ubx_date_valid) {
      static bool first_time_logged = false;
      if (!first_time_logged) {
        first_time_logged = true;
        enqueueTask(createLogSdfTask("GPS TIME: %04d-%02d-%02d %02d:%02d:%02d UTC",
          ubx_year, ubx_month, ubx_day, ubx_hour, ubx_min, ubx_sec));
      }
    }
  }

  if (ubx_sat_updated) {
    ubx_sat_updated = false;
    // DOP ログは handle_navdop() 内で実施済みのため、ここでは satelite staleness のみ管理
  }
}


extern int max_adreading;  // display_tft.cpp で計測したバッテリー ADC 最大値

// GPS データが揃っている場合、フライトログ CSV への保存タスクをキューに積む。
// 1秒に1回だけ保存するようにクールダウンを設けている（NMEA の都合で同一秒に複数回 update が来るため）。
// リプレイモードとデモモードでは保存しない。
// GPS データが揃っている場合、フライトログ CSV への保存タスクをキューに積む。
// 400ms に1回だけ保存するようにクールダウンを設けている。
// UBX モード・リプレイモード共に ubx_* 変数を参照する（リプレイモードでは gps_loop() でミラー済み）。
// リプレイモードとデモモードでは保存しない。
void try_enque_savecsv(){
  bool all_valid = true;
  if (ubx_pos_valid) {
    stored_fixtype = (stored_numsats > 0) ? 2 : 1;
  } else {
    DEBUG_PLN(20250923,"Location: Not Available");
    all_valid = false;
    stored_fixtype = 0;
  }
  if (!ubx_date_valid) {
    DEBUG_PLN(20250923,"Date: Not Available");
    all_valid = false;
  }
  if (!ubx_time_valid) {
    DEBUG_PLN(20250923,"Time: Not Available");
    all_valid = false;
  }

  // 位置・日時が全て揃っていて、前回保存から 400ms 以上経過した時のみ保存（2Hz GPS に合わせて調整）
  if(all_valid && millis() - last_gps_save_time > 400){
    if(!get_demo_biwako()){ //デモは別の場所で登録済み。
      add_latlon_track(get_gps_lat(),get_gps_lon());
    }

    //リプレイは保存しない。
    if(!getReplayMode()){
      DEBUG_P(20250923,"SAVED!");
      static int lastsavedtime = 0;
      DEBUG_PLN(20250923,millis()-lastsavedtime);

      lastsavedtime = millis();
      int year  = ubx_year;
      int month = ubx_month;
      int day   = ubx_day;
      int hour  = ubx_hour;
      utcToJst(&year, &month, &day, &hour);
      float csv_pressure   = get_airdata_ok() ? get_airdata_pressure() : 0.0f;
      float csv_kf_alt    = get_imu_altitude_msl();  // KF推定高度 [m]（MSL基準・GNSS長期収束済み）
      float csv_kf_vspeed = get_imu_vspeed();   // KF推定上昇率 [m/s]
      enqueueTask(createSaveCsvTask(stored_latitude, stored_longitude, stored_gs, stored_truetrack,
        stored_altitude, csv_kf_alt, csv_kf_vspeed, csv_pressure,
        year, month, day, hour, ubx_min, ubx_sec, ubx_cs));

      last_gps_save_time = millis();
    }
  }
}






// 現在の緯度を返す。
// デモモードや各デバッグシミュレーション設定が有効な場合は、実際の GPS 座標の代わりに
// 設定した固定座標やオフセット座標を返す（settings.h の #define で切り替える）。
double get_gps_lat() {
  if (demo_biwako) {
    return demo_biwako_lat;
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
  if (demo_biwako) {
    return demo_biwako_lon;
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
  if (demo_biwako) {
    return demo_biwako_mps;
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


bool get_gps_connection() {
  return gps_connection;
}
bool get_gps_fix() {
  if(demo_biwako){
    return get_gps_numsat() != 0;
  }
  if(stored_fixtype >= 1){
    return true;
  }
  return false;
}

double get_gps_altitude() {
  return stored_altitude;
}


double get_gps_truetrack() {
  #ifdef DEBUG_GPS_SIM_SHINURA
    return 40 + (38.5 + sin(millis() / 2100.0)) * sin(millis() / 3000.0);
  #endif
  if (demo_biwako) {
    return demo_biwako_truetrack;
  }
  return stored_truetrack;     // Heading in degrees;
}


// 磁気方位（Magnetic Track）を返す。
// 日本の磁気偏差はおよそ +7〜+9 度（西偏）なので、真方位に +8 度加算する。
// 正確な偏差は地域・年によって変わるため、settings.h で調整する。
double get_gps_magtrack() {
  double temp = get_gps_truetrack() + 8.0;
  if (temp > 360.0) {
    temp -= 360.0;
  }
  return temp;
}

int get_gps_numsat() {
  if(get_demo_biwako()){
    return (int)(20.0*sin(millis()/5000))+20;
  }else if(getReplayMode()){
    return 99;
  }
  return stored_numsats;
}

