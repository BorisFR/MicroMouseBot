#include "BoardLed.h"

BoardLed::BoardLed() {}

BoardLed::~BoardLed() { myTrace.println("💡 unloaded"); }

void BoardLed::setup()
{
    myTrace.println("💡 setup");
    FastLED.addLeds<NEOPIXEL, LED_RGB_PIN>(pixel, LED_RGB_NUMBER);
    FastLED.setBrightness(LED_RGB_BRIGHTNESS);
    pixel[0] = CRGB::Red;
    FastLED.show();
}

void BoardLed::loop()
{
    if (delayTimer > BOARD_LED_DELAY)
    {
        delayTimer = delayTimer - BOARD_LED_DELAY;
    }
}

void BoardLed::setColor(uint8_t r, uint8_t g, uint8_t b)
{
    pixel[0] = CRGB(r, g, b);
    FastLED.show();
}

void BoardLed::setColorBlack()
{
    pixel[0] = CRGB::Black;
    FastLED.show();
}

void BoardLed::setColorWhite()
{
    pixel[0] = CRGB::White;
    FastLED.show();
}

void BoardLed::setColorRed()
{
    pixel[0] = CRGB::Red;
    FastLED.show();
}

void BoardLed::setColorGreen()
{
    pixel[0] = CRGB::Green;
    FastLED.show();
}

void BoardLed::setColorBlue()
{
    pixel[0] = CRGB::Blue;
    FastLED.show();
}

void BoardLed::setColorYellow()
{
    pixel[0] = CRGB::Yellow;
    FastLED.show();
}

void BoardLed::setColorOrange()
{
    pixel[0] = CRGB::Orange;
    FastLED.show();
}

void BoardLed::setColorCyan()
{
    pixel[0] = CRGB::Cyan;
    FastLED.show();
}
