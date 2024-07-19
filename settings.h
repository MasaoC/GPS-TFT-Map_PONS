//====== 設定画面 =======

//GPSのデバッグ用途。ひとつだけ選択。【リリース版は、RELEASE_GPSを選択】
  #define RELEASE_GPS
  //#define DEBUG_GPS_SIM_SHINURA         //新浦安固定座標
  //#define DEBUG_GPS_SIM_SHINURA2BIWA    //新浦安座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_OSAKA2BIWA      //阪大座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_SHINURA2OSAKA   //新浦安座標から阪大座標に置換

// TFTを選択
  #define TFT_USE_ST7789    //Tested well. Change User Setting at TFT_eSPI
  //#define TFT_USE_ILI9341   //Tested well.  Change User Setting at TFT_eSPI
  //#define TFT_USE_ST7735      //Not supported anymore.  Screen size too small.

//TFTスクリーン更新頻度 ms. (GPSと一致するように変更すればよい。1000ms>990ms for serial?)
#define SCREEN_INTERVAL 990

//強制画面リフレッシュ時間
#define SCREEN_FRESH_INTERVAL 30000



// =====Hardware Settings =====

//外部スイッチ（up,down,push)がない場合。SINGLE。３個ある場合は comment out。
#define SINGLE_SWITCH

// TFTのBL調整に使われているトランジスタ。NPN or PNP を指定する。（画面の明るさ設定のため）
#define NPN_BL
//#define PNP_BL

// Hardware Ver 0.1

#define SW_PUSH   14
#define RP2040_ZERO
#define SW_UP     10
#define SW_DOWN   11
#define TFT_BL     7


// Hardware Ver0.2 
/*#define RP2040_PICO
#define SW_PUSH 22 
#define SW_UP -1//10
#define SW_DOWN -1//11
#define TFT_BL        -1//7
*/

#define BATTERY_PIN 26 //A3

// TFTとの接続Pin設定は、TFT_eSPIも設定してください。設定サンプルは、CopySetupFile_TFT_eSPI.h にあります。



