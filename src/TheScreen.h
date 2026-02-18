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
        theScreen.drawCentreString("* MicroMouse *", currentWidth() / 2, 4, 1);
        theScreen.setTextColor(TFT_CYAN, TFT_DARKGREEN);
        theScreen.drawCentreString("By Boris", currentWidth() / 2, currentHeight() - 14 + 4, 1);
        theScreen.drawRect(0, 14, currentWidth() - 1, currentHeight() - 28, TFT_BLUE);
    }

    void loop()
    {
        // Update the display with new information here
    }

    void showMap()
    {
        // Display the map on the TFT screen in a simple way (for example, as a grid of colored squares)
        // You can iterate through the occupancy grid and draw rectangles for each cell based on its state
        uint16_t cellWidth = currentWidth() / MAP_WIDTH;
        uint16_t cellHeight = currentHeight() / MAP_HEIGHT;
        uint16_t cellSize = min(cellWidth, cellHeight);
        uint16_t offsetX = (currentWidth() - (cellSize * MAP_WIDTH)) / 2;
        uint16_t offsetY = (currentHeight() - (cellSize * MAP_HEIGHT)) / 2;
        for (int x = 0; x < MAP_WIDTH; x++)
        {
            for (int y = 0; y < MAP_HEIGHT; y++)
            {
                if (occupancyGrid[x][y] == oldGrid[x][y])
                {
                    continue; // No change, skip drawing
                }
                oldGrid[x][y] = occupancyGrid[x][y]; // Update oldGrid with new value
                uint16_t color;
                if (x == botPose.x && y == botPose.y)
                {
                    color = TFT_GREEN; // Robot's current position
                }
                else
                {
                    if (occupancyGrid[x][y] == CELL_UNKNOWN)
                        color = TFT_GREY;
                    else if (occupancyGrid[x][y] == CELL_FREE)
                        color = TFT_DARKGREEN;
                    else if (occupancyGrid[x][y] == CELL_OCCUPIED)
                        color = TFT_RED;
                    else if (occupancyGrid[x][y] == CELL_PERHAPS_OCCUPIED)
                        color = TFT_YELLOW;
                }
                theScreen.fillRect(offsetX + x * cellSize, offsetY + y * cellSize, cellSize, cellSize, color);
            }
        }
        if (cellSize >= 3)
        {
            // Optionally, you can also draw grid lines
            for (int x = 0; x <= MAP_WIDTH; x++)
            {
                theScreen.drawLine(offsetX + x * cellSize, offsetY, offsetX + x * cellSize, offsetY + MAP_HEIGHT * cellSize, TFT_BLACK);
            }
            for (int y = 0; y <= MAP_HEIGHT; y++)
            {
                theScreen.drawLine(offsetX, offsetY + y * cellSize, offsetX + MAP_WIDTH * cellSize, offsetY + y * cellSize, TFT_BLACK);
            }
            // Optionally, you can also draw grid lines
            for (int x = 0; x <= MAP_WIDTH; x++)
            {
                theScreen.drawLine(offsetX + x * cellSize, offsetY, offsetX + x * cellSize, offsetY + MAP_HEIGHT * cellSize, TFT_BLACK);
            }
            for (int y = 0; y <= MAP_HEIGHT; y++)
            {
                theScreen.drawLine(offsetX, offsetY + y * cellSize, offsetX + MAP_WIDTH * cellSize, offsetY + y * cellSize, TFT_BLACK);
            }
        }
    }

    unsigned int colorFromRGB(uint8_t r, uint8_t g, uint8_t b)
    {
        // colour = red << 11 | green << 5 | blue;
        return theScreen.color565(r, g, b);
    }

private:
    TFT_eSPI theScreen; // Create an instance of the TFT_eSPI class
    CellState oldGrid[MAP_WIDTH][MAP_HEIGHT];

    int32_t currentWidth()
    {
        switch (theScreen.getRotation())
        {
        case 0:
        case 2:
            return TFT_WIDTH;
        case 1:
        case 3:
            return TFT_HEIGHT;
        }
    }

    int32_t currentHeight()
    {
        switch (theScreen.getRotation())
        {
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