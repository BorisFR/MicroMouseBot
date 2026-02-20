/*#ifndef THE_TOUCH_H
#define THE_TOUCH_H

#include "Globals.h"
#include <XPT2046_Touchscreen.h>

class TheTouch
{
public:
    TheTouch() : touch(XPT2046_CS, XPT2046_IRQ) {}

    ~TheTouch() { myTrace.println("TheTouch unloaded"); }

    void setup()
    {
        myTrace.println("TheTouch setup");
        // Initialize touch screen here
        SPI.begin(XPT2046_SCK, XPT2046_MISO, XPT2046_MOSI);
        touch.begin();
    }

    void loop()
    {
        if (touch.tirqTouched())
        {
            if (touch.touched())
            {
                TS_Point p = touch.getPoint();
                Serial.print("Pressure = ");
                Serial.print(p.z);
                Serial.print(", x = ");
                Serial.print(p.x);
                Serial.print(", y = ");
                Serial.print(p.y);
                delay(30);
                Serial.println();
            }
        }
    }

    bool isTouched()
    {
        // Return true if the screen is currently being touched
        return touch.touched();
    }

    TS_Point getPoint()
    {
        // Return the coordinates of the touch point
        return touch.getPoint();
    }

    void eventTouch(EVENT_ERROR callback)
    {
        theCallback = callback;
    }

private:
    XPT2046_Touchscreen touch;
    EVENT_ERROR theCallback;
};

#endif // THE_TOUCH_H
*/