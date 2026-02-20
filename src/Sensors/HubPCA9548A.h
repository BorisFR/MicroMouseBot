#ifndef HUB_PCA9548A_H
#define HUB_PCA9548A_H

#include "../Globals.h"
#include <Wire.h>

// https://www.ti.com/product/PCA9548A
// I2C multiplexer PCA9548A
// 8 channels, 12-bit address (0x70-0x77)

#define PCA9548A_ADDRESS 0x70

class HubPCA9548A
{
public:
    HubPCA9548A() {}

    ~HubPCA9548A() { myTrace.println("HubPCA9548A unloaded"); }

    void setup()
    {
        myTrace.println("HubPCA9548A setup");
    }

    void selectChannel(uint8_t channel)
    {
        if(channel == currentChannel)
            return;
        if (channel > 7)
            return;
        Wire.beginTransmission(PCA9548A_ADDRESS);
        Wire.write(1 << channel);
        Wire.endTransmission();
        currentChannel = channel;
        vTaskDelay(1);
    }

private:
    int8_t currentChannel = -1;
};

#endif // HUB_PCA9548A_H