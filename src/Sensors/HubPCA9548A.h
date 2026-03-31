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
    HubPCA9548A();

    ~HubPCA9548A();

    void setup();

    bool isInitialized();

    void selectChannel(uint8_t channel);

private:
    int8_t currentChannel = -1;
    bool initialized = false;
};

#endif // HUB_PCA9548A_H