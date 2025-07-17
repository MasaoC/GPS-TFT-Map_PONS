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

//====================[Setting file FOR ST7789 SPI]====================
// eSPI version must be [2.5.34]
// 2.5.43 version does not work!! due to bug.
// Change User_Setup_Select.h as well.

#define USER_SETUP_ID 18
#define ST7789_DRIVER     // Configure all registers
#define TFT_WIDTH  240
#define TFT_HEIGHT 320
#define TFT_INVERSION_OFF
#define TFT_RGB_ORDER TFT_BGR  // Colour order Blue-Green-Red


#define TFT_SCLK         2
#define TFT_MOSI         3
#define TFT_CS       	4
#define TFT_RST        5
#define TFT_DC         6

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts
#define SMOOTH_FONT

//SDカードの都合で20Mhzまで。いまのところ...
#define SPI_FREQUENCY       20000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000
#define DISABLE_ALL_LIBRARY_WARNINGS



//====================[Setting file FOR ILI9341]====================
//Copy the following lines into library setting file that you will use.
//Note:  Change User_Setup_Select.h as well to load the setting that you want to use.

#define USER_SETUP_ID 1
#define ILI9341_DRIVER
#define TFT_CS   5  // Chip select control pin D8
#define TFT_DC   6  // Data Command control pin
#define TFT_RST  4  // Reset pin (could connect to NodeMCU RST, see next line)
#define TFT_MOSI 3
#define TFT_SCLK 2
//#define TFT_RST  -1    // Set TFT_RST to -1 if the display RESET is connected to NodeMCU RST or 3.3V

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts
#define SMOOTH_FONT

#define SPI_FREQUENCY  20000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  250000
#define DISABLE_ALL_LIBRARY_WARNINGS
