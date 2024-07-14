#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

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
