#ifndef THE_COMPASS_H
#define THE_COMPASS_H

#include "Globals.h"

// LSM6DS3TR-C  + LIS3MDL

class TheCompass
{
public:
    TheCompass() {}

    ~TheCompass() { myTrace.println("🧭 unloaded"); }

    void setup()
    {
        myTrace.println("🧭 setup");
        // Initialize compass hardware here
        isInitialized = true; // Set to true if initialization is successful
    }

    void begin()
    {
        changeDetected = true;
        if (theCallbackValueChange)
            theCallbackValueChange(0.0f); // Call with a default value of 0.0f to indicate that the compass is ready and has an initial heading value. This will allow the screen to update the compass status immediately after initialization.
    }

    void loop()
    {
        float newValue = 0.0f;                     // Placeholder value, replace with actual reading from the sensor
        newValue += random(0, 10) / 10.0f - 0.5f;         // Simulate random changes in the heading for testing purposes
        if (abs(newValue - currentHeading) > 0.1f) // Example threshold for change detection
        {
            currentHeading = newValue;
            changeDetected = true;
            if (theCallbackValueChange)
                theCallbackValueChange(newValue);
        }
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

private:
    bool isInitialized = false;
    EVENT_ERROR theCallbackError;
    bool changeDetected = false;
    float currentHeading = 0.0f;
    EVENT_CHANGE_WITH_FLOAT theCallbackValueChange;
};

#endif // THE_COMPASS_H