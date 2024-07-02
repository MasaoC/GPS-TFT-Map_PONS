//====== 設定画面 =======

//使用するマイコン種類を選択する。【どれか一つを選択】
  #define XIAO_RP2040              // RP2040が有効の場合、LEDを使用する。
  //#define XIAO_ESP32S3             // ESP32S3は、HardSerialのプログラムに変更が必要となるため、使用する場合は次の行を有効とする。
  //#define XIAO_SAMD21                // 無印のXIAO。クロックが遅く、メモリーが少ないため非推奨


//GPSのデモ用途。ひとつだけ選択。【リリース版は、RELEASE_GPSを選択】
  #define RELEASE_GPS
  //#define DEBUG_GPS_SIM_BIWAKO          //琵琶湖で移動していることをシミュレーションするモード
  //#define DEBUG_GPS_SIM_SHINURA2BIWA    //新浦安座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_OSAKA2BIWA      //阪大座標から琵琶湖座標に置換
  //#define DEBUG_GPS_SIM_SHINURA2OSAKA   //新浦安座標から阪大座標に置換


// TFTを選択
  //7789 is being under test.  Not debuged yet
  //#define TFT_USE_ST7789    //Not tested
  //#define TFT_USE_ST7735      //Tested well
  #define TFT_USE_ILI9341   //Tested some

//TFTスクリーン更新頻度 ms. (GPSと一致するように変更すればよい)
#define SCREEN_INTERVAL 500


//#define USE_OLED