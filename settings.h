//====== 設定画面 =======



//GPSのデモ用途。ひとつだけ選択。【リリース版は、RELEASE_GPSを選択】
  #define RELEASE_GPS
  //#define DEBUG_GPS_SIM_SHINURA2BIWA    //新浦安座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_OSAKA2BIWA      //阪大座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_SHINURA2OSAKA   //新浦安座標から阪大座標に置換


// TFTを選択
  #define TFT_USE_ST7789    //Tested well. Change User Setting at TFT_eSPI
  //#define TFT_USE_ILI9341   //Tested well.  Change User Setting at TFT_eSPI

  //#define TFT_USE_ST7735      //Not supported anymore.  Screen size too small.

//TFTスクリーン更新頻度 ms. (GPSと一致するように変更すればよい)
#define SCREEN_INTERVAL 500

