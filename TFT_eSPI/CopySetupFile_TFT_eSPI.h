// TFT_eSPI ライブラリの設定を次のように変更してください。

//====================[Setting file FOR PONS v6  ST7789 16bit parallel]====================
#define USER_SETUP_ID 105
#define TFT_PARALLEL_16_BIT
#define ST7789_DRIVER
#define TFT_RGB_ORDER TFT_BGR  // Colour order Blue-Green-Red
//#define TFT_CS   -1  // Do not define, chip select control pin permanently connected to 0V
#define TFT_DC    11    // Data Command control pin
#define TFT_RST   29    // Reset pin
//#define TFT_RD   -	1  // Do not define, read pin must be permanently connected to 3V3
#define TFT_WR   12
// PIO requires these to be sequentially increasing GPIO with no gaps
#define TFT_D0   13
#define TFT_D1   14
#define TFT_D2  15
#define TFT_D3  16
#define TFT_D4  17
#define TFT_D5  18
#define TFT_D6  19
#define TFT_D7  20
#define TFT_D8  21
#define TFT_D9  22
#define TFT_D10  23
#define TFT_D11  24
#define TFT_D12  25
#define TFT_D13  26
#define TFT_D14  27
#define TFT_D15  28

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define SMOOTH_FONT
#define DISABLE_ALL_LIBRARY_WARNINGS

