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
    SensorVL53L0X();

    ~SensorVL53L0X();

    void setup();

    void loop();

    bool isInitializedSuccessfully();

    bool isInErrorState();

    bool hasChanged();

    uint16_t getLastDistance();

    void eventError(EVENT_ERROR callback);

    void eventChangeValue(EVENT_CHANGE_WITH_UINT16 callback);

private:
    void registerInvalidReading();

    void registerValidReading(uint16_t distance);

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