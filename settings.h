// ============================================================
// File    : settings.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : プロジェクト全体の設定・定数・マクロ定義。
//           デバッグフラグ、GPS/TFT種別選択、ハードウェアピン番号、
//           画面モード定数、バッテリー計算式など全設定の司令塔。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/03/11
// ============================================================
//====== 設定画面 =======
#include <stdint.h>  // uint32_t 等の整数型定義（DEBUG_STACK マクロで使用）

// リリース時
//#define RELEASE

#define BUILDDATE 20260313
#define BUILDVERSION "0.906"
#define VERSION_TEXT "Version 6"




//----------GPS---------
//#define DEBUG_NMEA
//#define QUECTEL_GPS
//#define MEADIATEK_GPS
#define UBLOX_GPS

//GPSのデバッグ用途。ひとつだけ選択。【リリース版は、RELEASE_GPSを選択】
  #define RELEASE_GPS
  //#define DEBUG_GPS_SIM_SHINURA         //新浦安固定座標
  //#define DEBUG_GPS_SIM_BIWAKO         //琵琶湖固定座標
  //#define DEBUG_GPS_SIM_SAPPORO         //札幌固定座標
  //#define DEBUG_GPS_SIM_SHISHI         //しし固定座標
  //#define DEBUG_GPS_SIM_SHINURA2BIWA    //新浦安座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_OSAKA2BIWA      //阪大座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_SHINURA2OSAKA   //新浦安座標から阪大座標に置換


//---------TFT----------
// TFTを選択
  #define TFT_USE_ST7789    //Tested well. Change User Setting at TFT_eSPI
  //#define TFT_USE_ILI9341   //Tested well.  Change User Setting at TFT_eSPI
  //#define TFT_USE_ST7735      //Not supported anymore.  Screen size too small.

#define VERTICAL_FLIP

//強制画面リフレッシュ時間
#define SCREEN_FRESH_INTERVAL 1050


#define MAX_TRACK_CORDS 500



// =====Hardware Settings =====

// Hardware Ver6
#define SW_PUSH 35  //30(v6 proto)
#define BATTERY_PIN 40 //A0
#define TFT_BL  -1
#define GPS_SERIAL Serial1
#define GPS_TX 0
#define GPS_RX 1
#define USB_DETECT 31
#define USERLED_PIN 34
#define SD_CS_PIN -1
#define RP_CLK_GPIO 2 // Set to CLK GPIO
#define RP_CMD_GPIO 3 // Set to CMD GPIO
#define RP_DAT0_GPIO 4 // Set to DAT0 GPIO. DAT1..3 must be consecutively connected. DAT1=5, DAT2=6, DAT3=7
#define SD_CS_SPI_PIN 7 // SPI フォールバック時の CS ピン（DAT3 = GPIO7）
#define SD_DETECT 8

#define BATTERY_MULTIPLYER(adr) (0.00238423334*adr) //VSYS 1/4098*3.3*(151/51)=0.00238423334
#define BAT_LOW_VOLTAGE 3.5
#define BAT_ZERO_VOLTAGE 3.4
#define PIN_PWMTONE 38
#define PIN_AMP_SD 39 //アンプシャットダウン(HIGHでON)
#define USERLED_PIN 34 //ユーザーLED（エラー表示用。エラー時 HIGH）
#define SIN_VOLUME 0.15f // WAVファイルの音量と合わせるために、Sin wave音の音量は下げられていますが、ここで調整可能です。0〜1.0f

// =====追加設定項目====
// TFTとの接続Pin設定は、TFT_eSPIも設定してください。設定サンプルは、CopySetupFile_TFT_eSPI.h にあります。



//======= Shared Global variables ======
//screen_mode
#define MODE_SETTING 1
#define MODE_MAP 2
#define MODE_GPSDETAIL 3
#define MODE_MAPLIST 4
#define MODE_SDDETAIL 5


#if !defined(TEMP)
  #define TEMP
  #if !defined(RELEASE) || !defined(RELEASE_GPS)
    #warning NOT RELEASE!
  #endif
#endif


//デバッグ用 print マクロ
#define PRINTREVERSEDATE_NUM 0
#ifndef RELEASE
  #define DEBUG_P(date,txt)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.print(txt);
  #define DEBUG_PN(date,txt,num)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.print(txt,num);
  #define DEBUGW_P(date,txt)  Serial.print(txt);
  #define DEBUG_PLN(date,txt)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.println(txt);
  #define DEBUG_PNLN(date,txt,num)  if(date >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.println(txt,num);
  #define DEBUGW_PLN(date,txt)  Serial.println(txt);
#else
  #define DEBUG_P(date,txt)  
  #define DEBUG_PN(date,txt,num)  
  #define DEBUGW_P(date,txt)  
  #define DEBUG_PLN(date,txt)
  #define DEBUG_PNLN(date,txt,num)   
  #define DEBUGW_PLN(date,txt)  
#endif

// ============================================================
// スタック残量計測マクロ
//
// Core0: リンカシンボル __StackLimit（スタック下限）と現在 SP の差 = 残りバイト数
// Core1: core1_separate_stack=true のため別ヒープから確保。
//        setup1() 先頭でキャプチャした _core1_base_sp との差 = 使用済みバイト数
//
// ※ 5 秒に 1 回だけ出力するのでシリアルが流れすぎない
// ============================================================
extern volatile uint32_t _core1_base_sp;  // GPS_TFT_map.ino で定義


#ifndef RELEASE
  #define DEBUG_STACK_C0(label) do { \
    static unsigned long _t0_; \
    if (millis() - _t0_ >= 5000) { \
      _t0_ = millis(); \
      Serial.print("[C0 stack free:" label "]="); \
      Serial.println((int)rp2040.getFreeStack()); \
    } \
  } while(0)

  #define DEBUG_STACK_C1(label) do { \
    static unsigned long _t1_; \
    if (millis() - _t1_ >= 5000) { \
      _t1_ = millis(); \
      uint32_t _sp_; \
      asm volatile ("mov %0, sp" : "=r" (_sp_)); \
      Serial.print("[C1 stack used:" label "]="); \
      Serial.println((int)(_core1_base_sp > _sp_ ? _core1_base_sp - _sp_ : 0)); \
    } \
  } while(0)
#else
  #define DEBUG_STACK_C0(label) 
  #define DEBUG_STACK_C1(label) 
#endif