#include "WheelEncoder.h"

WheelEncoder *wheelInstance1 = nullptr;
WheelEncoder *wheelInstance2 = nullptr;

static void handleInterruptA1Static()
{
    if (wheelInstance1)
    {
        wheelInstance1->handleInterruptA();
    }
}

static void handleInterruptB1Static()
{
    if (wheelInstance1)
    {
        wheelInstance1->handleInterruptB();
    }
}

static void handleInterruptA2Static()
{
    if (wheelInstance2)
    {
        wheelInstance2->handleInterruptA();
    }
}

static void handleInterruptB2Static()
{
    if (wheelInstance2)
    {
        wheelInstance2->handleInterruptB();
    }
}

void WheelEncoder::setup(uint8_t index, uint8_t pinA, uint8_t pinB)
{
    myTrace.println("🔄 wheel encoder setup (" + String(index) + ")");
    this->pinA = pinA;
    this->pinB = pinB;

    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    ticks = 0;
    lastEncoded = (digitalRead(pinA) << 1) | digitalRead(pinB);
    lastResetTimeMs = millis();
    windowBaseTicks = 0;
    windowBaseTimeMs = lastResetTimeMs;
    switch (index)
    {
    case 0:
        wheelInstance1 = this;
        attachInterrupt(digitalPinToInterrupt(pinA), handleInterruptA1Static, CHANGE);
        attachInterrupt(digitalPinToInterrupt(pinB), handleInterruptB1Static, CHANGE);
        break;
    case 1:
        wheelInstance2 = this;
        attachInterrupt(digitalPinToInterrupt(pinA), handleInterruptA2Static, CHANGE);
        attachInterrupt(digitalPinToInterrupt(pinB), handleInterruptB2Static, CHANGE);
        break;
    default:
        myTrace.println("⚠️ wheel encoder setup: invalid index");
        break;
    }
}