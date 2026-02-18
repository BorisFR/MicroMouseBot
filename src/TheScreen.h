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
        theScreen.setRotation(1);
        theScreen.fillScreen(TFT_BLACK);

        theScreen.fillRect(0, 0, 319, 14, TFT_RED);

        theScreen.fillRect(0, 226, 319, 14, TFT_GREY);

        theScreen.setTextColor(TFT_BLACK, TFT_RED);
        theScreen.drawCentreString("* TFT_eSPI *", 160, 4, 1);
        theScreen.setTextColor(TFT_YELLOW, TFT_GREY);
        theScreen.drawCentreString("Adapted by Bodmer", 160, 228, 1);

        theScreen.drawRect(0, 14, 319, 211, TFT_BLUE);
    }

    void loop()
    {
        // Update the display with new information here
    }

private:
    TFT_eSPI theScreen; // Create an instance of the TFT_eSPI class
};

#endif // THE_SCREEN_H