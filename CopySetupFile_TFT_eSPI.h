
//====================[Setting file FOR ST7789]====================
// eSPI version [2.5.34]
// 2.5.43 version does not work!!!
//Copy the following lines into library setting file that you will use.
//Note:  Change User_Setup_Select.h as well to load the setting that you want to use.

#define USER_SETUP_ID 18
#define ST7789_DRIVER     // Configure all registers
#define TFT_WIDTH  240
#define TFT_HEIGHT 320
#define TFT_INVERSION_OFF
#define TFT_RGB_ORDER TFT_BGR  // Colour order Blue-Green-Red
/*
// For Pico (PR2040)
#define TFT_CS 17   // Chip Select pin
#define TFT_DC 16   // Data Command control pin 
#define TFT_RST -1  // No Reset pin
#define TFT_MOSI 19
#define TFT_SCLK 18
*/


//waveshare RP2040 zero
#define TFT_RST        4
#define TFT_CS        5
#define TFT_DC         6
#define TFT_MOSI         3
#define TFT_SCLK         2

#define LOAD_GLCD   // Font
#define SMOOTH_FONT
#define SPI_FREQUENCY       40000000
#define SPI_READ_FREQUENCY  20000000
//#define SPI_TOUCH_FREQUENCY  2500000







//====================[Setting file FOR ILI9341]====================
//Copy the following lines into library setting file that you will use.
//Note:  Change User_Setup_Select.h as well to load the setting that you want to use.

/*
#define USER_SETUP_ID 1
#define ILI9341_DRIVER
#define TFT_CS   5  // Chip select control pin D8
#define TFT_DC   6  // Data Command control pin
#define TFT_RST  4  // Reset pin (could connect to NodeMCU RST, see next line)
#define TFT_MOSI 3
#define TFT_SCLK 2
//#define TFT_RST  -1    // Set TFT_RST to -1 if the display RESET is connected to NodeMCU RST or 3.3V
#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
//#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
//#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:.
//#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
//#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts
#define SMOOTH_FONT
// #define SPI_FREQUENCY  27000000
#define SPI_FREQUENCY  40000000
// #define SPI_FREQUENCY  80000000

#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  250000
// #define SUPPORT_TRANSACTIONS
*/
