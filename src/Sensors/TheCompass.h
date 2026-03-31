#ifndef THE_COMPASS_H
#define THE_COMPASS_H

#include "Globals.h"

// LSM6DS3TR-C  + LIS3MDL
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <math.h>
// 100 or 104 Hz seems to be the sweet spot for the best performance/accuracy balance for the NXP sensor fusion filter on the LSM6DS3TR-C + LIS3MDL combo, which is what TheCompass is using. Higher rates can cause instability in the filter and worse accuracy, while lower rates can cause more lag and less responsiveness. The exact optimal rate may vary based on your specific use case and environment, so feel free to experiment with different rates around this range to see what works best for your application.
#define FILTER_UPDATE_RATE_HZ 104

class TheCompass
{
public:
    TheCompass();

    ~TheCompass();

    void setup();

    void begin();

    void loop();

    bool isCalibrationLoaded();

    bool isInitializedSuccessfully();

    float getRoll();

    float getPitch();

    float getHeading();

    bool isErrorDetected();

    bool isGyroErrorDetected();

    bool isMagnetometerErrorDetected();

    void eventError(EVENT_ERROR callback);

    void eventChangeValue(EVENT_CHANGE_WITH_FLOAT callback);

    bool isLIS3MDLInitialized();

    bool isLSM6DSInitialized();

private:
    static constexpr float HEADING_EPSILON_DEG = 0.2f;
    static constexpr uint32_t DATA_STALE_TIMEOUT_MS = 500;
    bool isInitialized = false;
    EVENT_ERROR theCallbackError;
    float currentHeading = 0.0f;
    float currentRoll = 0.0f;
    float currentPitch = 0.0f;
    EVENT_CHANGE_WITH_FLOAT theCallbackValueChange;

    Adafruit_LIS3MDL lis3mdl;
    Adafruit_LSM6DS3TRC lsm6ds;
    bool lis3mdlInitialized = false;
    bool lsm6dsInitialized = false;
    Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
    Adafruit_Sensor_Calibration_EEPROM calibration;
    bool calibrationLoaded = false;
    Adafruit_NXPSensorFusion filter; // slowest
    // Adafruit_Madgwick filter; // faster than NXP
    // Adafruit_Mahony filter;  // fastest/smalleset
    uint32_t lastUpdateMicros = 0;
    unsigned long lastGoodSampleMs = 0;
    bool imuErrorDetected = false;
    bool gyroErrorDetected = false;
    bool magnetometerErrorDetected = false;

    static bool isFiniteTriple(float x, float y, float z);

    bool isDataStale() const;

    void markRuntimeError();
};

#endif // THE_COMPASS_H