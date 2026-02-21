#ifndef ALL_SENSORS_H
#define ALL_SENSORS_H

#include "Globals.h"
#include "HubPCA9548A.h"
#include "SensorVL53L0X.h"

// https://www.pololu.com/product/2490
// Time-of-Flight distance sensor VL53L0X
// 3 cm to 200 cm (2 m) range, accurate to 3mm, up to 50Hz (depending on timing budget)
// FOV 25°x25° (see https://www.st.com/resource/en/datasheet/vl53l0x.pdf)

class AllSensors
{
public:
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
        if (theCallbackSensors)
            theCallbackSensors();
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
                sensorVL53L0X[i].eventError([i, this]()
                                            {
                    myTrace.print("🚨 VL53L0X sensor on channel ");
                    myTrace.printDEC(i);
                    myTrace.println(" has entered error state"); 
                    if (theCallbackSensors)
                        theCallbackSensors(); });
            }
            else
            {
                myTrace.print("🚨 VL53L0X sensor on channel ");
                myTrace.printDEC(i);
                myTrace.println(" failed to initialize");
            }
            if (theCallbackSensors)
                theCallbackSensors();
        }
        //doFullScan();
        if (theCallbackSensors)
            theCallbackSensors();
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
                    /*myTrace.print("VL53L0X [");
                    myTrace.printDEC(i);
                    myTrace.print("]: ");
                    myTrace.printDEC(sensorVL53L0X[i].getLastDistance());
                    myTrace.println(" mm");*/
                    changed = true;
                }
            }
        }
    }

    bool isHubReady()
    {
        return hubPCA9548A.isInitialized();
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

    void eventErrorHub(EVENT_ERROR callback)
    {
        theCallbackHub = callback;
    }

    void eventErrorSensor(EVENT_ERROR callback)
    {
        theCallbackSensors = callback;
    }

private:
    HubPCA9548A hubPCA9548A;
    SensorVL53L0X sensorVL53L0X[VL53L0X_COUNT]; // 1 sensor on each hub channel
    bool changed = false;
    EVENT_ERROR theCallbackHub;
    EVENT_ERROR theCallbackSensors;

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