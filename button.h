#ifndef BUTTON_H
#define BUTTON_H
#include <Arduino.h>
#include <string>

  struct Setting {
      int id;
      std::string (*getLabel)(bool selected);
      void (*CallbackEnter)();
      void (*CallbackToggle)();
      void (*CallbackExit)();
      int (*iconColor)();
  };
  extern Setting menu_settings[];
  extern const int setting_size;
  // Button class definition
  class Button {
  public:
      Button(int p, void (*shortPressCb)() = NULL, void (*longPressCb)() = NULL);
      void read();
      int getPin(); // Method to get the pin number

  private:
      int pin;
      bool switchState;
      bool lastSwitchState;
      unsigned long pressTime;
      bool longPressHandled;
      void (*shortPressCallback)();
      void (*longPressCallback)();
  };

#endif
