#ifndef HUB_PCA9548A_H
#define HUB_PCA9548A_H

#include "../Globals.h"
#include <Wire.h>

// https://www.ti.com/product/PCA9548A
// I2C multiplexer PCA9548A
// 8 channels, 12-bit address (0x70-0x77)

class HubPCA9548A
{
public:
    HubPCA9548A() {}

    ~HubPCA9548A() { myTrace.println("HubPCA9548A unloaded"); }

    void setup()
    {
        myTrace.println("HubPCA9548A setup");
        pinMode(PCA9548A_RESET_PIN, OUTPUT);
        digitalWrite(PCA9548A_RESET_PIN, LOW);
        // wait 2 ms for reset
        vTaskDelay(pdMS_TO_TICKS(2));
        digitalWrite(PCA9548A_RESET_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(20)); // Short delay to allow hub to reset and be ready for communication
        Wire.beginTransmission(PCA9548A_ADDRESS);
        if (Wire.endTransmission() == 0)
        {
            myTrace.println("HubPCA9548A detected successfully");
            initialized = true;
            selectChannel(7); // Select last channel by default
        }
        else
        {
            myTrace.println("🚨 HubPCA9548A not detected");
        }
    }

    bool isInitialized()
    {
        return initialized;
    }

    void selectChannel(uint8_t channel)
    {
        if(!initialized) return;
        if (channel == currentChannel)
            return;
        if (channel > 7)
            return;
        Wire.beginTransmission(PCA9548A_ADDRESS);
        Wire.write(1 << channel);
        Wire.endTransmission();
        currentChannel = channel;
        // vTaskDelay(pdMS_TO_TICKS(2)); // Short delay to allow channel switching to stabilize
    }

private:
    int8_t currentChannel = -1;
    bool initialized = false;
};

#endif // HUB_PCA9548A_H