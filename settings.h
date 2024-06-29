

//使用する XIAO のマイコン種類を選択する。(SAMD21は選択不要)

  // RP2040が有効の場合、LEDを使用する。
  #define RP2040
  // ESP32S3は、HardSerialのプログラムに変更が必要となるため、使用する場合は次の行を有効とする。
  //#define ESP32S3

//GPSのデモ用途。ひとつだけ選択。【リリース版は全てコメントアウト】
  //#define DEBUG_GPS_SIM_BIWAKO
  #define DEBUG_GPS_SIM_SHINURA2BIWA
  //#define DEBUG_GPS_SIM_OSAKA2BIWA
  //#define DEBUG_GPS_SIM_SHINURA2OSAKA


//TFTスクリーン更新頻度 ms. (GPSと一致するように変更すればよい)
#define SCREEN_INTERVAL 500

// TFTを選択
  //7789 is being under test.  Not debuged yet
  //#define TFT_USE_ST7789

  //Tested well
  //#define TFT_USE_ST7735

  #define TFT_USE_ILI9341
