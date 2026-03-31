#ifndef BOARD_LED_H
#define BOARD_LED_H

#include "Globals.h"
#include <FastLED.h>
#include <elapsedMillis.h>
#define BOARD_LED_DELAY 20

class BoardLed
{
public:
    BoardLed();

    ~BoardLed();

    void setup();

    void loop();

    void setColor(uint8_t r, uint8_t g, uint8_t b);

    void setColorBlack();

    void setColorWhite();

    void setColorRed();

    void setColorGreen();

    void setColorBlue();

    void setColorYellow();

    void setColorOrange();

    void setColorCyan();

private:
    CRGB pixel[LED_RGB_NUMBER];
    elapsedMillis delayTimer;
};

#endif