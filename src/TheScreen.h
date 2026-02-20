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
        display.init();
        display.setRotation(TFT_ROTATE);
        display.fillScreen(TFT_BLACK);
        display.fillRect(0, 0, currentWidth() - 1, 14, TFT_RED);
        display.fillRect(0, currentHeight() - 14, currentWidth() - 1, 14, TFT_DARKGREEN);
        display.setTextColor(TFT_WHITE, TFT_RED);
        display.drawCentreString("* MicroMouse *", currentWidth() / 2, 4, 1);
        display.setTextColor(TFT_CYAN, TFT_DARKGREEN);
        display.drawCentreString("By Boris", currentWidth() / 2, currentHeight() - 14 + 4, 1);
        display.drawRect(0, 14, currentWidth() - 1, currentHeight() - 28, TFT_BLUE);
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
                display.fillRect(offsetX + x * cellSize, offsetY + y * cellSize, cellSize, cellSize, color);
            }
        }
        if (cellSize >= 3)
        {
            // Optionally, you can also draw grid lines
            for (int x = 0; x <= MAP_WIDTH; x++)
            {
                display.drawLine(offsetX + x * cellSize, offsetY, offsetX + x * cellSize, offsetY + MAP_HEIGHT * cellSize, TFT_BLACK);
            }
            for (int y = 0; y <= MAP_HEIGHT; y++)
            {
                display.drawLine(offsetX, offsetY + y * cellSize, offsetX + MAP_WIDTH * cellSize, offsetY + y * cellSize, TFT_BLACK);
            }
            // Optionally, you can also draw grid lines
            for (int x = 0; x <= MAP_WIDTH; x++)
            {
                display.drawLine(offsetX + x * cellSize, offsetY, offsetX + x * cellSize, offsetY + MAP_HEIGHT * cellSize, TFT_BLACK);
            }
            for (int y = 0; y <= MAP_HEIGHT; y++)
            {
                display.drawLine(offsetX, offsetY + y * cellSize, offsetX + MAP_WIDTH * cellSize, offsetY + y * cellSize, TFT_BLACK);
            }
        }
    }

    unsigned int colorFromRGB(uint8_t r, uint8_t g, uint8_t b)
    {
        // colour = red << 11 | green << 5 | blue;
        return display.color565(r, g, b);
    }

    int32_t currentWidth()
    {
        switch (display.getRotation())
        {
        case 0:
        case 2:
            return TFT_WIDTH;
        case 1:
        case 3:
            return TFT_HEIGHT;
        }
        return 0; // Should never reach here
    }

    int32_t currentHeight()
    {
        switch (display.getRotation())
        {
        case 0:
        case 2:
            return TFT_HEIGHT;
        case 1:
        case 3:
            return TFT_WIDTH;
        }
        return 0; // Should never reach here
    }

    void fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
    {
        display.fillRect(x, y, w, h, color);
    }

    uint32_t blue() { return TFT_BLUE; }
    uint32_t red() { return TFT_RED; }
    uint32_t green() { return TFT_GREEN; }
    uint32_t yellow() { return TFT_YELLOW; }
    uint32_t orange() { return colorFromRGB(255, 165, 0); }
    
private:
    TFT_eSPI display; // Create an instance of the TFT_eSPI class
    CellState oldGrid[MAP_WIDTH][MAP_HEIGHT];

};

#endif // THE_SCREEN_H