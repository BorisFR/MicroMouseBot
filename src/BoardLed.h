#ifndef BOARD_LED_H
#define BOARD_LED_H

#include "Globals.h"
#include <FastLED.h>
#include <elapsedMillis.h>
#define BOARD_LED_DELAY 20

class BoardLed
{
public:
    BoardLed()
    {
        // myTrace.println("💡 loaded");
    }

    ~BoardLed()
    {
        myTrace.println("💡 unloaded");
    }

    void setup()
    {
        myTrace.println("💡 setup");
        FastLED.addLeds<NEOPIXEL, LED_RGB_PIN>(pixel, LED_RGB_NUMBER);
        FastLED.setBrightness(LED_RGB_BRIGHTNESS);
        pixel[0] = CRGB::Red;
        FastLED.show();
    }

    void loop()
    {
        if (delayTimer > BOARD_LED_DELAY)
        {
            delayTimer = delayTimer - BOARD_LED_DELAY;
        }
    }

    void setColor(uint8_t r, uint8_t g, uint8_t b)
    {
        pixel[0] = CRGB(r, g, b);
        FastLED.show();
    }

    void setColorBlack()
    {
        pixel[0] = CRGB::Black;
        FastLED.show();
    }

    void setColorWhite()
    {
        pixel[0] = CRGB::White;
        FastLED.show();
    }

    void setColorRed()
    {
        pixel[0] = CRGB::Red;
        FastLED.show();
    }

    void setColorGreen()
    {
        pixel[0] = CRGB::Green;
        FastLED.show();
    }

    void setColorBlue()
    {
        pixel[0] = CRGB::Blue;
        FastLED.show();
    }

    void setColorYellow()
    {
        pixel[0] = CRGB::Yellow;
        FastLED.show();
    }

    void setColorOrange()
    {
        pixel[0] = CRGB::Orange;
        FastLED.show();
    }

    void setColorCyan()
    {
        pixel[0] = CRGB::Cyan;
        FastLED.show();
    }

private:
    CRGB pixel[LED_RGB_NUMBER];
    elapsedMillis delayTimer;
};

#endif