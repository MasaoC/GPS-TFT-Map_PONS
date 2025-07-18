//====================[Setting file FOR ST7789 16bit parallel]====================

#define USER_SETUP_ID 105
#define TFT_PARALLEL_16_BIT
#define ST7789_DRIVER
//#define TFT_INVERSION_OFF
#define TFT_RGB_ORDER TFT_BGR  // Colour order Blue-Green-Red
//#define TFT_CS   -1  // Do not define, chip select control pin permanently connected to 0V
#define TFT_DC    27    // Data Command control pin
#define TFT_RST   22    // Reset pin
//#define TFT_RD   -1  // Do not define, read pin must be permanently connected to 3V3
#define TFT_WR   28
// PIO requires these to be sequentially increasing GPIO with no gaps
#define TFT_D0    6
#define TFT_D1    7
#define TFT_D2    8
#define TFT_D3    9
#define TFT_D4   10
#define TFT_D5   11
#define TFT_D6   12
#define TFT_D7   13
#define TFT_D8   14
#define TFT_D9   15
#define TFT_D10  16
#define TFT_D11  17
#define TFT_D12  18
#define TFT_D13  19
#define TFT_D14  20
#define TFT_D15  21

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define SMOOTH_FONT
#define DISABLE_ALL_LIBRARY_WARNINGS


