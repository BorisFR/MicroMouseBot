#ifndef ALL_SENSORS_H
#define ALL_SENSORS_H

#include "Globals.h"
#include "HubPCA9548A.h"
#include "SensorVL53L0X.h"
#define VL53L0X_COUNT 1

// https://www.pololu.com/product/2490
// Time-of-Flight distance sensor VL53L0X
// 3 cm to 200 cm (2 m) range, accurate to 3mm, up to 50Hz (depending on timing budget)
// FOV 25°x25° (see https://www.st.com/resource/en/datasheet/vl53l0x.pdf)

#define BUS_TO_SCAN 1

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
        Wire.begin(SENSORS_SDA_PIN, SENSORS_SCL_PIN);
        Wire.setClock(SENSORS_I2C_SPEED);
        doScan();
        hubPCA9548A.setup();
        hubPCA9548A.selectChannel(0);
        for (uint8_t i = 0; i < VL53L0X_COUNT; i++)
            sensorVL53L0X[i].setup();
        doFullScan();
    }

    void loop()
    {
        for (uint8_t i = 0; i < VL53L0X_COUNT; i++)
        {
            sensorVL53L0X[i].loop();
            if (sensorVL53L0X[i].hasChanged())
            {
                uint16_t distance = sensorVL53L0X[i].getLastDistance();
                myTrace.print("VL53L0X [");
                myTrace.printDEC(i);
                myTrace.print("]: ");
                myTrace.printDEC(distance);
                myTrace.println(" mm");
            }
        }
    }

private:
    HubPCA9548A hubPCA9548A;
    SensorVL53L0X sensorVL53L0X[VL53L0X_COUNT]; // 1 sensor on channel 0

    void doFullScan()
    {
        myTrace.println("🕵️  Scanning...");
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
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (nDevices == 0)
            myTrace.println("🕵️  No I2C devices found");
        else
            myTrace.println("🕵️  Scan complete");
    }
};

#endif // ALL_SENSORS_H