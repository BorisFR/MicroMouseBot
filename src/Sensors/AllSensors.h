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
        bool headingValid;
    };

    AllSensors() {}

    ~AllSensors() { myTrace.println("🕵️ unloaded"); }

    void setup()
    {
        myTrace.println("🕵️ setup");
        gpio_num_t sdaPin = static_cast<gpio_num_t>(SENSORS_SDA_PIN);
        gpio_num_t sclPin = static_cast<gpio_num_t>(SENSORS_SCL_PIN);
        gpio_set_pull_mode(sdaPin, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode(sclPin, GPIO_PULLUP_ONLY);
        vTaskDelay(pdMS_TO_TICKS(2)); // Short delay to allow pull-up resistors to stabilize
        Wire.begin(SENSORS_SDA_PIN, SENSORS_SCL_PIN);
        Wire.setClock(SENSORS_I2C_SPEED);
        vTaskDelay(pdMS_TO_TICKS(2)); // Short delay to allow I2C bus to stabilize
    }

    void begin()
    {
        hubPCA9548A.setup();
        if (theCallbackHub)
            theCallbackHub();
        if (!hubPCA9548A.isInitialized())
            return;
        for (uint8_t i = 0; i < VL53L0X_COUNT; i++)
        {
            hubPCA9548A.selectChannel(i);
            vTaskDelay(pdMS_TO_TICKS(100)); // Short delay to allow channel switching to stabilize
            sensorVL53L0X[i].setup();
            if (sensorVL53L0X[i].isInitializedSuccessfully())
            {
                myTrace.print("VL53L0X sensor on channel ");
                myTrace.printDEC(i);
                myTrace.println(" initialized successfully");
                sensorVL53L0X[i].eventChangeValue([i, this](uint16_t value)
                                                  {if (theCallbackSensorWithIndexAndValue)
                                                        theCallbackSensorWithIndexAndValue(i, value); });

                sensorVL53L0X[i].eventError([i, this]()
                                            {if (theCallbackSensorWithIndexAndValue)
                                                theCallbackSensorWithIndexAndValue(i, 0); });
            }
            else
            {
                myTrace.print("🚨 VL53L0X sensor on channel ");
                myTrace.printDEC(i);
                myTrace.println(" failed to initialize");
            }
            if (theCallbackSensorWithIndexAndValue)
                theCallbackSensorWithIndexAndValue(i, 0); // Call with a default value of 0 to indicate that the sensor is ready and has an initial distance value. This will allow the screen to update the sensor status immediately after initialization.
        }
        // doFullScan();

        theCompass.setup();
        theCompass.eventChangeValue([this](float value)
                                    { lastHeading = value;
                                    changeIMU = true;
                                    frameChanged = true;
                                    if (theCallbackIMU)
                                        theCallbackIMU(value); });
        theCompass.eventError([this]()
                              { myTrace.println("🚨 Compass error detected");
                                if (theCallbackIMU)
                                    theCallbackIMU(0); });
        theCompass.begin();
    }

    void loop()
    {
        for (uint8_t i = 0; i < VL53L0X_COUNT; i++)
        {
            if (!sensorVL53L0X[i].isInErrorState())
            {
                hubPCA9548A.selectChannel(i);
                sensorVL53L0X[i].loop();
                if (sensorVL53L0X[i].hasChanged())
                {
                    changedSensorVL53L0X = true;
                    frameChanged = true;
                }
            }
        }
        theCompass.loop();
    }

    bool isHubReady()
    {
        return hubPCA9548A.isInitialized();
    }

    bool hasChanged()
    {
        if (changedSensorVL53L0X)
        {
            changedSensorVL53L0X = false;
            return true;
        }
        return false;
    }

    bool getLatestFrame(SensorFrame &frame)
    {
        if (!frameChanged)
        {
            return false;
        }
        frameChanged = false;
        for (uint8_t i = 0; i < VL53L0X_COUNT; i++)
        {
            frame.distances[i] = sensorVL53L0X[i].getLastDistance();
        }
        frame.heading = lastHeading;
        frame.headingValid = isIMUinitializedSuccessfully();
        changeIMU = false;
        return true;
    }

    uint16_t getLastDistance(uint8_t sensorIndex)
    {
        if (sensorIndex < VL53L0X_COUNT)
            return sensorVL53L0X[sensorIndex].getLastDistance();
        return 0;
    }

    bool isSensorInitialized(uint8_t sensorIndex)
    {
        if (sensorIndex < VL53L0X_COUNT)
            return sensorVL53L0X[sensorIndex].isInitializedSuccessfully();
        return false;
    }

    bool isSensorErrorDetected(uint8_t sensorIndex)
    {
        if (sensorIndex < VL53L0X_COUNT)
            return sensorVL53L0X[sensorIndex].isInErrorState();
        return true;
    }

    void eventHubChange(EVENT_CHANGE callback)
    {
        theCallbackHub = callback;
    }

    void eventSensorWithIndexAndValue(EVENT_CHANGE_WITH_UINT8_UINT16 callback)
    {
        theCallbackSensorWithIndexAndValue = callback;
    }

    void eventImuChange(EVENT_CHANGE_WITH_FLOAT callback)
    {
        theCallbackIMU = callback;
    }

    bool isIMUinitializedSuccessfully()
    {
        return theCompass.isInitializedSuccessfully();
    }

    bool isIMUErrorDetected()
    {
        return false; // Placeholder, implement actual error detection logic for the IMU if available. For now, we are not simulating IMU errors, so we return false to indicate no error.
        // return theCompass.isErrorDetected(); // Using error detection as a proxy for error state for simplicity
    }

    bool isIMUChangeDetected()
    {
        if (changeIMU)
        {
            changeIMU = false;
            return true;
        }
        return false;
    }

    float getHeading()
    {
        return theCompass.getHeading();
    }

private:
    HubPCA9548A hubPCA9548A;
    SensorVL53L0X sensorVL53L0X[VL53L0X_COUNT]; // 1 sensor on each hub channel
    bool changedSensorVL53L0X = false;
    bool frameChanged = false;
    EVENT_CHANGE theCallbackHub;
    EVENT_CHANGE_WITH_UINT8_UINT16 theCallbackSensorWithIndexAndValue;
    TheCompass theCompass;
    bool changeIMU = false;
    float lastHeading = 0.0f;
    EVENT_CHANGE_WITH_FLOAT theCallbackIMU;

    void doFullScan()
    {
        myTrace.println("🕵️  Full Scanning...");
        for (uint8_t busNumber = 0; busNumber < BUS_TO_SCAN; busNumber++)
        {
            myTrace.print("🕵️  Scanning I2C bus ");
            myTrace.println(busNumber);
            hubPCA9548A.selectChannel(busNumber);
            doScan();
        }
    }

    void doScan()
    {
        byte error, address;
        int nDevices = 0;
        for (address = 1; address < 127; address++)
        {
            Wire.beginTransmission(address);
            error = Wire.endTransmission();
            if (error == 0)
            {
                myTrace.print("🕵️  I2C device found at address 0x");
                myTrace.printlnHEX(address);
                nDevices++;
            }
            else if (error == 4)
            {
                myTrace.print("🕵️  Unknown error at address 0x");
                myTrace.printlnHEX(address);
            }
            vTaskDelay(pdMS_TO_TICKS(2)); // Short delay to allow I2C bus to stabilize between address checks
        }
        if (nDevices == 0)
            myTrace.println("🕵️  No I2C devices found");
        else
            myTrace.println("🕵️  Scan complete");
    }
};

#endif // ALL_SENSORS_H