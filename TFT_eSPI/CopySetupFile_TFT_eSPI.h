
//====================[Setting file FOR PONS v5  ST7789 8bit parallel]====================
#define USER_SETUP_ID 105
#define TFT_PARALLEL_8_BIT
#define ST7789_DRIVER
#define TFT_RGB_ORDER TFT_BGR  // Colour order Blue-Green-Red
//#define TFT_CS   -1  // Do not define, chip select control pin permanently connected to 0V
#define TFT_DC    28    // Data Command control pin
#define TFT_RST   22    // Reset pin
//#define TFT_RD   -1  // Do not define, read pin must be permanently connected to 3V3
#define TFT_WR   27
// PIO requires these to be sequentially increasing GPIO with no gaps
#define TFT_D0   14
#define TFT_D1   15
#define TFT_D2  16
#define TFT_D3  17
#define TFT_D4  18
#define TFT_D5  19
#define TFT_D6  20
#define TFT_D7  21

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define SMOOTH_FONT
#define DISABLE_ALL_LIBRARY_WARNINGS
