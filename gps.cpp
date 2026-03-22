// ============================================================
// File    : gps.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : GPS受信・解析の実装（Quectel LC86G向けに最適化）。
//           TinyGPS++を使ったNMEA解析、衛星情報収集(GSV)、
//           位置・速度・時刻の取得、リプレイ/デモモード管理、
//           フライトログCSVへの定期保存トリガー。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/02/26
// ============================================================
// Handle GNSS modules. Currently optimized for LC86GPAMD.
#include <Arduino.h>

#include "gps.h"
#include "mysd.h"
#include "navdata.h"
#include "settings.h"
#include "display_tft.h"
#include "airdata.h"
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


// --- u-blox UBX-CFG-MSG: NMEA メッセージ出力レート制御 ---
// UBX-CFG-MSG: B5 62 06 01 03 00 [msgClass] [msgID] [rate] [CK_A] [CK_B]
// rate=N: N回の測位に1回出力。rate=0で無効化。2Hz時: rate=2→1Hz, rate=20→0.1Hz(10秒に1回)
// チェックサムは Fletcher-8（class〜payloadの全バイトに対して計算）
#define UBLOX_GSV_RATE_2  {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x02,0xFF,0x17}  // GSV(F0 03): 1Hz
#define UBLOX_GSV_RATE_20 {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x14,0x11,0x29}  // GSV(F0 03): 0.1Hz (10秒に1回)
#define UBLOX_GSA_RATE_2  {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x02,0xFE,0x15}  // GSA(F0 02): 1Hz
#define UBLOX_GSA_RATE_20 {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x14,0x10,0x27}  // GSA(F0 02): 0.1Hz (10秒に1回)

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
int   get_gps_fixtype()  { return gsa_fixtype; }
float get_gps_pdop()     { return gsa_pdop; }
float get_gps_hdop()     { return gsa_hdop; }
float get_gps_vdop()     { return gsa_vdop; }
int   get_gsa_numsat()   { return gsa_numsat; }
int   get_gsa_prn(int i) { return (i >= 0 && i < GSA_MAX_PRN) ? gsa_prns[i] : 0; }

// GSV（Satellites in View）NMEA 文を手動パースして satellites[] 配列に衛星情報を格納する。
// TinyGPS++ は GSV を解析しないため、自前でパースする必要がある。
// GSV 文の形式: $GPGSV,総文数,文番号,衛星数,PRN,仰角,方位角,SNR,...*チェックサム
// 衛星種別は文の先頭識別子から判定する（GP=GPS, GL=GLONASS, GA=Galileo, GB=BeiDou）。
void parseGSV(char *nmea) {
  // Print the received NMEA sentence for debugging
  #ifdef DEBUG_NMEA
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


  #ifdef DEBUG_NMEA
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

    #ifdef DEBUG_NMEA
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
  // ナビ画面ではGSV/GSAともに10秒に1回で十分
  { const unsigned char cmd[] = UBLOX_GSV_RATE_20;
    for (unsigned i = 0; i < sizeof(cmd); i++) GPS_SERIAL.write(cmd[i]); }
  { const unsigned char cmd[] = UBLOX_GSA_RATE_20;
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
  // 星座画面ではGSV/GSAともに1Hzに上げて衛星情報を更新しやすくする
  { const unsigned char cmd[] = UBLOX_GSV_RATE_2;
    for (unsigned i = 0; i < sizeof(cmd); i++) GPS_SERIAL.write(cmd[i]); }
  { const unsigned char cmd[] = UBLOX_GSA_RATE_2;
    for (unsigned i = 0; i < sizeof(cmd); i++) GPS_SERIAL.write(cmd[i]); }
  #endif
}

int setupcounter = 1;  // gps_setup() の呼び出し回数（1=初回、2以降=リトライ）
uint32_t gps_current_baudrate = 0;  // GPS シリアルの現在ボーレート（gps_setup() で更新）
uint32_t get_gps_baudrate() { return gps_current_baudrate; }

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
      DEBUG_PLN(20251025,"UBLOX 38400");
      GPS_SERIAL.setFIFOSize(1024);//バッファオーバーフロー防止（デフォルト32バイトでは不足）
      GPS_SERIAL.begin(gps_current_baudrate = 9600);  // u-blox工場デフォルトは9600bps。まずこのボーレートで開いてからコマンドを送る
      // Configure GPS baud rate
      const unsigned char UBLOX_INIT_38400[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xC0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x83,0x90,};
      delay (50);// なぜか必要（Do not delete without care.)
      for (int i = 0; i < sizeof(UBLOX_INIT_38400); i++) {
        GPS_SERIAL.write(UBLOX_INIT_38400[i]);
      }
      delay (100);//なぜか必要（Do not delete without care.)
      GPS_SERIAL.end();
      delay(50);//なぜか必要（Do not delete without care.)
      GPS_SERIAL.begin(gps_current_baudrate = 38400);
      // UBX-CFG-RATE: 測位レートを 2Hz（500ms）に設定
      // Payload: measRate=0x01F4(500ms), navRate=0x0001, timeRef=0x0001(GPS)
      // Checksum: 0x0B 0x77
      const unsigned char UBLOX_RATE_2HZ[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00,0x0B,0x77};
      delay(50);
      for (int i = 0; i < sizeof(UBLOX_RATE_2HZ); i++) {
        GPS_SERIAL.write(UBLOX_RATE_2HZ[i]);
      }
      // ACK 確認: B5 62 05 01 → ACK-ACK、B5 62 05 00 → ACK-NAK
      // u-blox は CFG コマンドに対し UBX-ACK を返す（通常 ~100ms 以内）
      delay(100);
      {
        uint8_t ack_buf[10] = {};
        int ack_len = 0;
        unsigned long t = millis();
        while (millis() - t < 200 && ack_len < 10) {
          if (GPS_SERIAL.available()) ack_buf[ack_len++] = GPS_SERIAL.read();
        }
        // ACK-ACK: B5 62 05 01 xx xx 06 08 ...
        // ACK-NAK: B5 62 05 00 xx xx 06 08 ...
        bool got_ack = false, got_nak = false;
        for (int i = 0; i < ack_len - 3; i++) {
          if (ack_buf[i]==0xB5 && ack_buf[i+1]==0x62 && ack_buf[i+2]==0x05) {
            if (ack_buf[i+3] == 0x01) got_ack = true;
            if (ack_buf[i+3] == 0x00) got_nak = true;
          }
        }
        if (got_ack)      { DEBUGW_PLN(20260309, "UBLOX CFG-RATE 2Hz: ACK OK"); }
        else if (got_nak) { DEBUGW_PLN(20260309, "UBLOX CFG-RATE 2Hz: NAK!"); }
        else              { DEBUGW_PLN(20260309, "UBLOX CFG-RATE 2Hz: no response"); }
      }
      // GSV/GSA 出力を 20 回に 1 回（0.1Hz = 10秒に1回）に落とす。2Hz 測位でもトラフィックを抑制。
      { const unsigned char cmd[] = UBLOX_GSV_RATE_20;
        delay(50);
        for (unsigned i = 0; i < sizeof(cmd); i++) GPS_SERIAL.write(cmd[i]); }
      { const unsigned char cmd[] = UBLOX_GSA_RATE_20;
        delay(50);
        for (unsigned i = 0; i < sizeof(cmd); i++) GPS_SERIAL.write(cmd[i]); }
    #else
      GPS_SERIAL.begin(gps_current_baudrate = 9600);
    #endif
    
  }else{
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
  if (gps.time.isValid()) {
    jst_h = (gps.time.hour() + 9) % 24;
    jst_m = gps.time.minute();
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


TinyGPSDate get_gpsdate(){
  return gps.date;
}
TinyGPSTime get_gpstime(){
  return gps.time;
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
      #ifdef DEBUG_NMEA
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
//   - 10 秒間 NMEA が届かない → gps_setup() で再接続を試みる
//   - 非 ASCII 文字が 10 回連続 → ボーレート不一致とみなして gps_setup()
//   - FIFO バッファが 256 を超えている → 警告ログを出す
void gps_loop(int id) {

  // 30 秒以上 NMEA が途絶えた場合は GPS を再初期化する
  if(GPS_SERIAL.available() == 0 && millis() - get_gps_nmea_time(0) > 30000 && millis() - last_gps_setup_time > 30000){
    DEBUGW_PLN(20250923,"Lost NMEA MSG for 30 seconds. resetup.");
    gps_setup();
  }
  if(GPS_SERIAL.available() > 256){
    DEBUGW_P(20250923,"ID=");
    DEBUGW_P(20250923,id);
    DEBUGW_P(20250923," Caution, remaining FIFO buffer. avail=");
    DEBUGW_PLN(20250923,GPS_SERIAL.available());
  }

  while (GPS_SERIAL.available() > 0) {
    char c = GPS_SERIAL.read();
    if(GPS_SERIAL.overflow()){
      DEBUGW_P(20250923,"!!WARNING!! GPS FIFO Overflow avail=");
      DEBUGW_PLN(20250923,GPS_SERIAL.available());
      // SDカードへエラーログを保存し、USERLEDを永続点灯させる
      enqueueTask(createLogSdTask("!!ERROR!! GPS FIFO overflow"));
      userled_forced_on = true;
    }
    // 128 以上の文字（非 ASCII）が続く場合、ボーレート不一致の可能性がある
    if((int)c >= 128){
      readfail_counter++;
      if(readfail_counter > 10){
        DEBUGW_P(20250923,"Read Failed 10 times, non ascii char:");
        DEBUGW_PLN(20250923,(int)c);
        gps_setup();  // 異なるボーレートで再試行
      }
      continue;
    }
    gps_connection = true;
    if(!replaymode_gpsoff)
      process_char(c);  // 通常モード: 実 GPS データを処理
  }

  // リプレイモード: SD から読み込んだ NMEA を 300ms ごとに1文ずつ流す
  if(replaymode_gpsoff){
    if(loaded_replay_nmea){
      for(int i = 0; i < 128; i++){
        process_char(replay_nmea[i]);
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

    
  if (gps.location.isUpdated()) {
    last_gps_time = millis();
    if(stored_latitude != gps.location.lat() || stored_longitude != gps.location.lng()){
      if(!get_demo_biwako())
        new_location_arrived = true;
    }
    stored_latitude = gps.location.lat();
    stored_longitude = gps.location.lng();
    

    #ifdef DEBUG_NMEA
    DEBUG_P(20250923,F("Location: "));
    DEBUG_PN(20250923,stored_latitude, 6);
    DEBUG_P(20250923,F(", "));
    DEBUG_PNLN(20250923,stored_longitude, 6);
    #endif

    #ifndef QUECTEL_GPS
    // QUECTELの場合は、ここで保存しない。courseで保存する。
    try_enque_savecsv();
    #endif
  }


  if (gps.altitude.isUpdated()) {
    #ifdef DEBUG_NMEA
    DEBUG_P(20250923,F("Altitude: "));
    DEBUG_PLN(20250923,stored_altitude);
    #endif
    stored_altitude = gps.altitude.meters();
  }

  if (gps.speed.isUpdated()) {
    stored_gs = gps.speed.mps();
    #ifdef DEBUG_NMEA
    DEBUG_P(20250923,F("Speed: "));
    DEBUG_P(20250923,stored_gs);  // Speed in meters per second
    DEBUG_PLN(20250923,F(" mps"));
    #endif

    update_maxgs(stored_gs);  // 最大 G/S を更新
  }

  if (gps.course.isUpdated()) {
    #ifdef DEBUG_NMEA
    DEBUG_P(20250923,F("Course: "));
    DEBUG_PLN(20250923,stored_truetrack);
    #endif
    if(!get_demo_biwako())
      newcourse_arrived = true;
    stored_truetrack = gps.course.deg();
    if(stored_truetrack < 0 || stored_truetrack > 360){
      DEBUGW_P(20250923,"ERROR:MT");
      DEBUGW_PLN(20250923,stored_truetrack);
      enqueueTask(createLogSdfTask("ERR truetrack=%.1f (forced 0)", stored_truetrack));
      stored_truetrack = 0;
    }

    #ifdef QUECTEL_GPS
    // LC86GPAMD において、 NMEAのcourseが最後であるため、ここでCSV保存処理を行う。
    try_enque_savecsv();
    #endif
  }

  if (gps.satellites.isUpdated()) {
    // Remove satellites not received for 30 seconds
    removeStaleSatellites();
    stored_numsats = gps.satellites.value();
    #ifdef DEBUG_NMEA
    DEBUG_P(20250923,F("Satellites: "));
    DEBUG_PLN(20250923,stored_numsats);
    #endif
  }

  // 初回 GPS 時刻取得時に UTC 時刻を SD ログに記録（1回だけ）
  if (gps.time.isUpdated() && gps.date.isValid() && gps.time.isValid()) {
    static bool first_time_logged = false;
    if (!first_time_logged) {
      first_time_logged = true;
      enqueueTask(createLogSdfTask("GPS TIME: %04d-%02d-%02d %02d:%02d:%02d UTC",
        gps.date.year(), gps.date.month(), gps.date.day(),
        gps.time.hour(), gps.time.minute(), gps.time.second()));
    }
    // GPS時刻更新のたびにmillis()を記録（Euler角ログの時刻推定に使用）
    gps_fix_millis = millis();
  }
}


extern int max_adreading;  // display_tft.cpp で計測したバッテリー ADC 最大値

// GPS データが揃っている場合、フライトログ CSV への保存タスクをキューに積む。
// 1秒に1回だけ保存するようにクールダウンを設けている（NMEA の都合で同一秒に複数回 update が来るため）。
// リプレイモードとデモモードでは保存しない。
void try_enque_savecsv(){
  bool all_valid = true;
  if (gps.location.isValid()) {
    // 衛星数が有効なら fixtype=2（3D フィックス相当）とする
    if (gps.satellites.isValid()) {
      stored_fixtype = 2;
    } else {
      stored_fixtype = 1;
    }
  } else {
    DEBUG_PLN(20250923,"Location: Not Available");
    all_valid = false;
    stored_fixtype = 0;
  }
  if (!gps.date.isValid()) {
    DEBUG_PLN(20250923,"Date: Not Available");
    all_valid = false;
  }
  if (!gps.time.isValid()) {
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
      int year = gps.date.year();
      int month = gps.date.month();
      int day = gps.date.day();
      int hour = gps.time.hour();
      utcToJst(&year,&month,&day,&hour);
      float calc_voltage = min(BATTERY_MULTIPLYER(max_adreading),4.3);
      float csv_pressure = get_airdata_ok() ? get_airdata_pressure() : 0.0f;
      enqueueTask(createSaveCsvTask(stored_latitude, stored_longitude, stored_gs, stored_truetrack, stored_altitude, csv_pressure, stored_numsats, calc_voltage,year, month, day, hour, gps.time.minute(), gps.time.second(), gps.time.centisecond()));
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

