#ifndef ALL_SENSORS_H
#define ALL_SENSORS_H

#include "Globals.h"
#include "HubPCA9548A.h"
#include "SensorVL53L0X.h"
#include "TheCompass.h"

// https://www.pololu.com/product/2490
// Time-of-Flight distance sensor VL53L0X
// 3 cm to 200 cm (2 m) range, accurate to 3mm, up to 50Hz (depending on timing budget)
// FOV 25°x25° (see https://www.st.com/resource/en/datasheet/vl53l0x.pdf)

class AllSensors
{
public:
    struct SensorFrame
    {
        uint16_t distances[VL53L0X_COUNT];
        float heading;
    };

    AllSensors();

    ~AllSensors();

    void setup();

    void begin();

    void loop();

    bool isHubReady();

    bool hasChanged();

    bool getLatestFrame(SensorFrame &frame);

    uint16_t getLastDistance(uint8_t sensorIndex);

    bool isSensorInitialized(uint8_t sensorIndex);

    bool isSensorErrorDetected(uint8_t sensorIndex);

    void eventHubChange(EVENT_CHANGE callback);

    void eventSensorWithIndexAndValue(EVENT_CHANGE_WITH_UINT8_UINT16 callback);

    void eventImuChange(EVENT_CHANGE_WITH_FLOAT callback);

    bool isIMUinitializedSuccessfully();

    bool isGyroReady();

    bool isGyroErrorDetected();

    float getGyroHeading();

    bool isMagnetometerReady();

    bool isMagnetometerErrorDetected();

    float getMagnetometerHeading();

    bool isIMUErrorDetected();

    float getHeading();

    bool isIMUCalibrated();

private:
    HubPCA9548A hubPCA9548A;
    SensorVL53L0X sensorVL53L0X[VL53L0X_COUNT]; // 1 sensor on each hub channel
    bool changedSensorVL53L0X = false;
    bool frameChanged = false;
    EVENT_CHANGE theCallbackHub;
    EVENT_CHANGE_WITH_UINT8_UINT16 theCallbackSensorWithIndexAndValue;
    TheCompass theCompass;
    float lastHeading = 0.0f;
    EVENT_CHANGE_WITH_FLOAT theCallbackIMU;

    void doFullScan();

    void doScan();
};

#endif // ALL_SENSORS_H