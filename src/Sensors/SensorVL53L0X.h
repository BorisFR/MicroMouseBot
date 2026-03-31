#ifndef SENSOR_VL53L0X_H
#define SENSOR_VL53L0X_H

#include "../Globals.h"
#include <VL53L0X.h>
#include "elapsedMillis.h"

#define VL53L0X_LONG_RANGE
// one or the other of these can be defined, but not both at the same time
#define VL53L0X_HIGH_ACCURACY
// #define VL53L0X_HIGH_SPEED
#define VL53L0X_MAX_DISTANCE 200 // cm
#define VL53L0X_MIN_DISTANCE 3   // cm

#define VL53L0X_DELAY_BETWEEN_READS 200 // ms
#define VL53L0X_RECOVERY_VALID_READING_COUNT 3

class SensorVL53L0X
{
public:
    SensorVL53L0X() {}

    ~SensorVL53L0X() { myTrace.println("SensorVL53L0X unloaded"); }

    void setup()
    {
        myTrace.println("SensorVL53L0X setup");
        isInitialized = false;
        inErrorState = false;
        countError = 0;
        consecutiveValidReadings = 0;
        changed = false;
        lastDistance = 0;
        sensor.setBus(&Wire);
        sensor.setAddress(VL53L0x_ADDRESS);
        sensor.setTimeout(VL53L0x_TIMEOUT);
        uint8_t countAttempts = 0;
        while (!sensor.init())
        {
            countAttempts++;
            vTaskDelay(pdMS_TO_TICKS(100)); // Short delay before retrying
            if (countAttempts >= 5)
            {
                myTrace.println("Failed to initialize VL53L0X sensor after multiple attempts. Giving up.");
                return;
            }
        }
        myTrace.println("VL53L0X sensor initialized");
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
        uint16_t distance = sensor.readRangeContinuousMillimeters() / 10; // Convert to cm
        if (sensor.timeoutOccurred())
        {
            registerInvalidReading();
            return;
        }

        if (distance == 0)
        {
            registerInvalidReading();
            return;
        }

        if (distance < VL53L0X_MIN_DISTANCE)
            distance = VL53L0X_MIN_DISTANCE;
        if (distance > VL53L0X_MAX_DISTANCE)
            distance = VL53L0X_MAX_DISTANCE;

        registerValidReading(distance);
    }

    bool isInitializedSuccessfully()
    {
        return isInitialized;
    }

    bool isInErrorState()
    {
        return inErrorState;
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

    void eventError(EVENT_ERROR callback)
    {
        theCallbackError = callback;
    }

    void eventChangeValue(EVENT_CHANGE_WITH_UINT16 callback)
    {
        theCallbackValueChange = callback;
    }

private:
    void registerInvalidReading()
    {
        consecutiveValidReadings = 0;
        if (countError < 255)
            countError++;
        if (!inErrorState && countError >= VL53L0X_ERROR_READING_COUNT_THRESHOLD)
        {
            inErrorState = true;
            if (theCallbackError)
                theCallbackError();
        }
    }

    void registerValidReading(uint16_t distance)
    {
        if (inErrorState)
        {
            if (consecutiveValidReadings < 255)
                consecutiveValidReadings++;
            if (consecutiveValidReadings >= VL53L0X_RECOVERY_VALID_READING_COUNT)
            {
                inErrorState = false;
                countError = 0;
                consecutiveValidReadings = 0;
            }
        }
        else
        {
            countError = 0;
            consecutiveValidReadings = 0;
        }

        if (distance != lastDistance)
        {
            changed = false;
            lastDistance = distance;
            changed = true;
            if (theCallbackValueChange)
                theCallbackValueChange(distance);
        }
    }

    VL53L0X sensor;
    bool isInitialized = false;
    bool inErrorState = false;
    uint8_t countError = 0;
    uint8_t consecutiveValidReadings = 0;
    EVENT_ERROR theCallbackError;
    EVENT_CHANGE_WITH_UINT16 theCallbackValueChange;
    elapsedMillis timeElapsed;
    bool changed = false;
    uint16_t lastDistance = 0;
};

#endif // SENSOR_VL53L0X_H