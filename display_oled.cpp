#include "display_oled.h"
#include "gps_functions.h"

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

void setup_oled(){

  // Initialize the OLED display with the SSD1306_SWITCHCAPVCC parameter
  if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }else{
    display.display();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
  }
}

int lastshown = 0;
void show_oled(){
  if(millis()-lastshown < 1000){
    return;
  }
  lastshown = millis();
  bool fix = get_gps_fix();
  float spd = get_gps_speed();
  float mtrack = get_gps_magtrack();

   // Clear the display
    display.clearDisplay();

    // Set a larger text size
    display.setTextSize(2);

    // Display Ground Speed
    display.setCursor(0, 0);
    display.print("GS:");
  
    // Convert speed from knots to m/s (1 knot = 0.514444 m/s) and display with one decimal place
    float speed_m_s = spd * 0.514444;
    display.print(speed_m_s, 1);
    display.print("m/s");
    

    // Display Track Made Good
    display.setCursor(0, 40);
    display.print("MT");
    // Set a larger text size
    display.setTextSize(5);
    // Display Track Made Good
    display.setCursor(30, 24);
    char buf[4];
    sprintf( buf, "%03d", (int)mtrack );
    display.print(buf); // Display as an integer
  

    // Display the results
    display.display();
}