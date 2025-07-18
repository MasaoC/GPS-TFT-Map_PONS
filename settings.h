//====== 設定画面 =======

// リリース時 削除
#define RELEASE

#define BUILDDATE 20250718
#define BUILDVERSION "0.602"

#define DEBUG_P(num,txt)  if(num >= BUILDDATE-1)Serial.print(txt);
#define DEBUG_PLN(num,txt)  if(num >= BUILDDATE-1)Serial.println(txt);


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

//TFTスクリーン更新頻度 ms. (GPSと一致するように変更すればよい。1000ms>990ms for serial?)
#define SCREEN_INTERVAL 990

//強制画面リフレッシュ時間
#define SCREEN_FRESH_INTERVAL 60000


#define MAX_TRACK_CORDS 500



// =====Hardware Settings =====

// Hardware Ver4
#define SW_PUSH 26
#define BATTERY_PIN 29 //A0
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
#define BATTERY_MULTIPLYER(adr) (0.002415812*adr) //VSYS 1/4098*3.3*3
#define BAT_LOW_VOLTAGE 3.3
#define SINGLE_SWITCH //外部スイッチ（up,down,push)がない場合。SINGLE。３個ある場合は comment out。
#define PIN_TONE 5


// =====追加設定項目====

// TFTとの接続Pin設定は、TFT_eSPIも設定してください。設定サンプルは、CopySetupFile_TFT_eSPI.h にあります。

// 秋月で売っているGPS など、Mediatek GPSを使う場合は、ublox_gps.h ではなく mediatek_gps.h に全て書き換えてください。





//======= Shared Global variables ======
//screen_mode
#define MODE_SETTING 1
#define MODE_MAP 2
#define MODE_GPSDETAIL 3
#define MODE_MAPLIST 4


//destination mode
#define DMODE_FLYINTO 0
#define DMODE_FLYAWAY 1

#if !defined(TEMP)
  #define TEMP
  #if !defined(RELEASE) || !defined(RELEASE_GPS)
    #warning NOT RELEASE!
  #endif
#endif