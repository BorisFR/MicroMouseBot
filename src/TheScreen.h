#ifndef THE_SCREEN_H
#define THE_SCREEN_H

/*
TFT SPI 2.8 240x320 v1.2
J1 soudé => 3.3V

PINs on device (vertical alignment)
* TOUCH
T_IRQ
T_DO
T_DIN
T_CS
T_CLK
* TFT       HSPI
SDO (MISO)  13              blanc
LED                 18      gris
SCK         12              mauve
SDI (MOSI)  11              bleu
DC                  9       vert
RESET               8       jaune
CS          10              orange
GND
VCC
*/

#include "Globals.h"
#include "TFT_eSPI.h"

#define TFT_ROTATE 2
#define TFT_GREY 0x7BEF

class TheScreen
{
public:
    TheScreen() {}

    ~TheScreen() { myTrace.println("🖥️ unloaded"); }

    void setup()
    {
        myTrace.println("🖥️ setup");
        theScreen.init();
        theScreen.setRotation(TFT_ROTATE);
        theScreen.fillScreen(TFT_BLACK);
        theScreen.fillRect(0, 0, currentWidth() - 1, 14, TFT_RED);
        theScreen.fillRect(0, currentHeight() - 14, currentWidth() - 1, 14, TFT_DARKGREEN);
        theScreen.setTextColor(TFT_WHITE, TFT_RED);
        theScreen.drawCentreString("* TFT_eSPI *", currentWidth() / 2, 4, 1);
        theScreen.setTextColor(TFT_CYAN, TFT_DARKGREEN);
        theScreen.drawCentreString("Adapted by Boris", currentWidth() / 2, currentHeight() - 14 + 4, 1);
        theScreen.drawRect(0, 14, currentWidth() - 1, currentHeight() - 28, TFT_BLUE);
    }

    void loop()
    {
        // Update the display with new information here
    }

private:
    TFT_eSPI theScreen; // Create an instance of the TFT_eSPI class

    int32_t currentWidth() { 
        switch(theScreen.getRotation()) {
            case 0:
            case 2:
                return TFT_WIDTH;
            case 1:
            case 3:
                return TFT_HEIGHT;
        }
    }

    int32_t currentHeight() { 
        switch(theScreen.getRotation()) {
            case 0:
            case 2:
                return TFT_HEIGHT;
            case 1:
            case 3:
                return TFT_WIDTH;
        }
    }
};

#endif // THE_SCREEN_H