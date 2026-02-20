#ifndef THE_MAP_H
#define THE_MAP_H

#include "Globals.h"
#include <vector>
#include <string>

struct LidarReading
{
    float angle;       // radians
    uint16_t distance; // centimeters
};

class TheMap
{
public:
    TheMap() {}

    ~TheMap() { myTrace.println("🗺️ unloaded"); }

    void setup()
    {
        myTrace.println("🗺️ setup");
        initializeGrid();
    }

    void loop()
    {
    }

    CellState getCellState(int x, int y)
    {
        if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
        {
            return static_cast<CellState>(occupancyGrid[x][y]);
        }
        return CELL_UNKNOWN; // Out of bounds
    }

    void updateCell(int x, int y, CellState state)
    {
        if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
        {
            if (state != occupancyGrid[x][y])
            {
                changedMap = true;
                occupancyGrid[x][y] = state;
            }
        }
    }

    void updateBotPose(uint16_t x, uint16_t y, float theta)
    {
        botPose.x = x;
        botPose.y = y;
        botPose.theta = theta;
    }

    void updateWithLidarReadings(const std::vector<LidarReading> &readings)
    {
        for (const auto &reading : readings)
        {
            // Convert polar coordinates to Cartesian
            float angle = botPose.theta + reading.angle;
            uint16_t distance = reading.distance;
            int cellX = static_cast<int>(botPose.x + distance * cos(angle));
            int cellY = static_cast<int>(botPose.y + distance * sin(angle));
            if(distance > CELL_DISTANCE_THRESHOLD) {
                updateCell(cellX, cellY, CELL_PERHAPS_OCCUPIED);
            }
            else {
                updateCell(cellX, cellY, CELL_OCCUPIED);
            }
        }
    }

    bool hasChanged()
    {
        if (changedMap)
        {
            changedMap = false;
            return true;
        }
        return false;
    }

    void printMap()
    {
        Serial.println("🗺️ Map:");
        for (int y = 0; y < MAP_HEIGHT; y++)
        {
            String row = "";
            for (int x = 0; x < MAP_WIDTH; x++)
            {
                if (x == botPose.x && y == botPose.y)
                {
                    row += "R"; // Robot's current position
                    continue;
                }
                CellState state = getCellState(x, y);
                if (state == CELL_UNKNOWN)
                    row += "?";
                else if (state == CELL_FREE)
                    row += ".";
                else if (state == CELL_OCCUPIED)
                    row += "#";
                else if (state == CELL_PERHAPS_OCCUPIED)
                    row += "o";
            }
            myTrace.println(row);
        }
    }

private:    
    bool changedMap = false;

    void initializeGrid()
    {
        for (int x = 0; x < MAP_WIDTH; x++)
        {
            for (int y = 0; y < MAP_HEIGHT; y++)
            {
                occupancyGrid[x][y] = CELL_UNKNOWN;
            }
        }
    }
};

#endif // THE_MAP_H