//====== 設定画面 =======

// リリース時
#define RELEASE

#define BUILDDATE 20250508
#define BUILDVERSION "0.87"

#define PRINTREVERSEDATE_NUM 0
#define DEBUG_P(num,txt)  if(num >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.print(txt);
#define DEBUGW_P(num,txt)  Serial.print(txt);
#define DEBUG_PLN(num,txt)  if(num >= BUILDDATE-PRINTREVERSEDATE_NUM)Serial.println(txt);
#define DEBUGW_PLN(num,txt)  Serial.println(txt);


//----------GPS---------
//#define DEBUG_NMEA
#define QUECTEL_GPS
//#define MEADIATEK_GPS
//#define UBLOX_GPS

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

// TFTのBL調整があれば、BRIGHTNESS_SETTING_AVAIL。また、調整に使われているトランジスタ。NPN or PNP を指定する。（画面の明るさ設定のため）
//#define BRIGHTNESS_SETTING_AVAIL
//#define NPN_BL
//#define PNP_BL



// Hardware Ver1,2
/*
#define MEDIATEK_GPS
#define RP2040_ZERO
#define SW_PUSH   14
#define BATTERY_PIN 26 //A0
#define SW_UP     10
#define SW_DOWN   11
#define TFT_BL     7
#define GPS_SERIAL Serial2
#define GPS_TX 8
#define GPS_RX 9

#define SD_RX 0
#define SD_CS_PIN 1
#define SD_SCK 2
#define SD_TX 3
#define BATTERY_MULTIPLYER(adr) (adr/4096.0*3.3*2)
#define BAT_LOW_VOLTAGE 3.55
*/




// Hardware Ver3
/*
#define RP2040_PICO
#define SW_PUSH 5
#define BATTERY_PIN 26 //A0
#define SW_UP   -1
#define SW_DOWN -1
#define TFT_BL  -1
#define GPS_SERIAL Serial1
#define GPS_TX 0
#define GPS_RX 1
//#define DISABLE_SD
#define SD_RX 4    //MISO
#define SD_CS_PIN -1
#define SD_SCK 2
#define SD_TX 3    //CMD MOSI
#define BATTERY_MULTIPLYER(adr) (0.0088*adr - 0.5357)  //いろいろトラブルシュートしているうちに、Vref2.95, 分圧1/11 になった数値。
#define BAT_LOW_VOLTAGE 3.49
*/

// Hardware Ver5
#define SW_PUSH 26
#define BATTERY_PIN 29 //A0
#define TFT_BL  -1
#define GPS_SERIAL Serial1
#define GPS_TX 0
#define GPS_RX 1


#define USB_DETECT 24



//#define DISABLE_SD
//#define SD_RX 4    //MISO
#define SD_CS_PIN -1
//#define SD_SCK 2
//#define SD_TX 3    //CMD MOSI
// If you have all 4 DAT pins wired up to the Pico you can use SDIO mode
#define RP_CLK_GPIO 2 // Set to CLK GPIO
#define RP_CMD_GPIO 3 // Set to CMD GPIO
#define RP_DAT0_GPIO 4 // Set to DAT0 GPIO. DAT1..3 must be consecutively connected.


#define BATTERY_MULTIPLYER(adr) (0.002415812*adr) //VSYS 1/4098*3.3*3
#define BAT_LOW_VOLTAGE 3.4
#define PIN_TONE 9
#define PIN_AMP_SD 10 //アンプシャットダウン(HIGHでON)


#define SIN_VOLUME 0.15f // WAVファイルの音量と合わせるために、Sin wave音の音量は下げられていますが、ここで調整可能です。0〜1.0f

// =====追加設定項目====

// TFTとの接続Pin設定は、TFT_eSPIも設定してください。設定サンプルは、CopySetupFile_TFT_eSPI.h にあります。

// 秋月で売っているGPS など、Mediatek GPSを使う場合は、ublox_gps.h ではなく mediatek_gps.h に全て書き換えてください。





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