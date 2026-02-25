#ifndef THE_COMPASS_H
#define THE_COMPASS_H

#include "Globals.h"

// LSM6DS3TR-C  + LIS3MDL
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>

class TheCompass
{
public:
    TheCompass() {}

    ~TheCompass() { myTrace.println("🧭 unloaded"); }

    void setup()
    {
        myTrace.println("🧭 setup");
        // Initialize compass hardware here
    }

    void begin()
    {
        if (lis3mdl.begin_I2C(LIS3MDL_ADDRESS))
        {
            myTrace.println("🧭 LIS3MDL initialized successfully");
            lis3mdlInitialized = true;
        }
        else
        {
            myTrace.println("🧭 LIS3MDL initialization failed");
        }

        if (lsm6ds.begin_I2C(LSM6DS3_ADDRESS))
        {
            myTrace.println("🧭 LSM6DS3 initialized successfully");
            lsm6dsInitialized = true;
        }
        else
        {
            myTrace.println("🧭 LSM6DS3 initialization failed");
        }

        isInitialized = lis3mdlInitialized && lsm6dsInitialized; // Set to true if initialization is successful
        currentHeading = 0.0f;                                   // Initialize heading to a default value
        changeDetected = true;
        if (theCallbackValueChange)
            theCallbackValueChange(currentHeading);
    }

    void loop()
    {
        if (!isInitialized)
            return; // Skip reading if not initialized
        // Read compass data and update heading
        sensors_event_t event;
        lis3mdl.getEvent(&event);
        float heading = atan2f(event.magnetic.y, event.magnetic.x);
        if (heading < 0)
            heading += 2 * PI;
        float newValue = heading;
        if (abs(newValue - currentHeading) > 0.1f) // Example threshold for change detection
        {
            currentHeading = newValue;
            changeDetected = true;
            if (theCallbackValueChange)
                theCallbackValueChange(newValue);
        }

        /*float newValue = 0.0f;                     // Placeholder value, replace with actual reading from the sensor
        newValue += random(0, 10) / 10.0f - 0.5f;  // Simulate random changes in the heading for testing purposes
        if (abs(newValue - currentHeading) > 0.1f) // Example threshold for change detection
        {
            currentHeading = newValue;
            changeDetected = true;
            if (theCallbackValueChange)
                theCallbackValueChange(newValue);
        }*/
    }

    bool isInitializedSuccessfully()
    {
        return isInitialized;
    }

    float getHeading()
    {
        // Return the current heading in radians (0 to 2*PI)
        // You can use a magnetometer sensor to get the heading
        return currentHeading; // Return the current heading value
    }

    bool isChangeDetected()
    {
        if (changeDetected)
        {
            changeDetected = false; // Reset the flag after reporting the change
            return true;
        }
        return false;
    }

    void eventError(EVENT_ERROR callback)
    {
        theCallbackError = callback;
    }

    void eventChangeValue(EVENT_CHANGE_WITH_FLOAT callback)
    {
        theCallbackValueChange = callback;
    }

    bool isLIS3MDLInitialized()
    {
        return lis3mdlInitialized;
    }

    bool isLSM6DSInitialized()
    {
        return lsm6dsInitialized;
    }

private:
    bool isInitialized = false;
    EVENT_ERROR theCallbackError;
    bool changeDetected = false;
    float currentHeading = 0.0f;
    EVENT_CHANGE_WITH_FLOAT theCallbackValueChange;

    Adafruit_LIS3MDL lis3mdl;
    Adafruit_LSM6DS3TRC lsm6ds;
    bool lis3mdlInitialized = false;
    bool lsm6dsInitialized = false;
};

#endif // THE_COMPASS_H