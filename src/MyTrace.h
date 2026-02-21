#ifndef MY_TRACE_H
#define MY_TRACE_H

#include "Globals.h"

class MyTrace
{

public:
    MyTrace()
    {
        Serial.begin(115200);
        vTaskDelay(pdMS_TO_TICKS(500)); // Short delay to allow Serial to initialize
        // Serial.println("📰 loaded");
    }

    ~MyTrace()
    {
        Serial.println("📰 unloaded");
    }

    /*void setup() {
      Serial.println("📰 setup");
    }*/

    void print(const char *text)
    {
        Serial.print(text);
    }

    void print(String text)
    {
        Serial.print(text);
    }

    void println(const char *text)
    {
        delay(1);
        Serial.println(text);
    }

    void println(String text)
    {
        delay(1);
        Serial.println(text);
    }

    void println(int value)
    {
        delay(1);
        Serial.println(value);
    }

    void printlnHEX(byte value)
    {
        if (value < 16)
            Serial.print("0");
        Serial.println(value, HEX);
    }

    void printDEC(byte value)
    {
        Serial.print(value, DEC);
    }

private:
};

#endif