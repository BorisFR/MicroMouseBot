#ifndef SENSOR_VL53L0X_H
#define SENSOR_VL53L0X_H

#include "../Globals.h"
// #include <Wire.h>
#include <VL53L0X.h>
#include "elapsedMillis.h"

#define VL53L0X_LONG_RANGE
#define VL53L0X_HIGH_ACCURACY
// #define VL53L0X_HIGH_SPEED

#define VL53L0X_DELAY_BETWEEN_READS 200 // ms

class SensorVL53L0X
{
public:
    SensorVL53L0X() {}

    ~SensorVL53L0X() { myTrace.println("SensorVL53L0X unloaded"); }

    void setup()
    {
        myTrace.println("SensorVL53L0X setup");
        sensor.setBus(&Wire);
        sensor.setTimeout(20);
        if (!sensor.init())
        {
            myTrace.println("Failed to detect and initialize VL53L0X sensor!");
            return;
        }
        isInitialized = true;
#if defined VL53L0X_LONG_RANGE
        // lower the return signal rate limit (default is 0.25 MCPS)
        sensor.setSignalRateLimit(0.1);
        // increase laser pulse periods (defaults are 14 and 10 PCLKs)
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined VL53L0X_HIGH_SPEED
        // reduce timing budget to 20 ms (default is about 33 ms)
        sensor.setMeasurementTimingBudget(20000);
#elif defined VL53L0X_HIGH_ACCURACY
        // increase timing budget to 200 ms
        sensor.setMeasurementTimingBudget(200000);
#endif
        // Start continuous readings at a rate of one measurement every 140 ms (the
        // inter-measurement period). This period should be at least as long as the
        // timing budget.
        sensor.startContinuous(VL53L0X_DELAY_BETWEEN_READS);
    }

    void loop()
    {
        if (!isInitialized)
            return;
        if (timeElapsed < VL53L0X_DELAY_BETWEEN_READS)
            return;
        timeElapsed = 0;
        uint16_t distance = sensor.readRangeContinuousMillimeters();
        if (sensor.timeoutOccurred())
        {
            myTrace.println("VL53L0X timeout");
        }
        else
        {
            if (distance != lastDistance)
            {
                lastDistance = distance;
                changed = true;
            }
        }
    }

    bool hasChanged()
    {
        if (changed)
        {
            changed = false;
            return true;
        }
        return false;
    }

    uint16_t getLastDistance()
    {
        return lastDistance;
    }

private:
    VL53L0X sensor;
    bool isInitialized = false;
    elapsedMillis timeElapsed;
    bool changed = false;
    uint16_t lastDistance = 0;
};

#endif // SENSOR_VL53L0X_H