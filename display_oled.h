#include <Adafruit_SSD1306.h>

// Define the I2C address for the OLED display
#define SSD1306_I2C_ADDRESS 0x3C

// Create an instance of the SSD1306 display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1


void setup_oled();
void show_oled();
