#ifndef THE_MAP_H
#define THE_MAP_H

#include "Globals.h"
#include <vector>
#include <string>

// unit is centimeter
#define CELL_SIZE 18
#define CELLS_BY_WIDTH 2
#define CELLS_BY_HEIGHT 1
#define MAP_WIDTH (CELL_SIZE * CELLS_BY_WIDTH) // 288 cm
#define MAP_HEIGHT (CELL_SIZE * CELLS_BY_HEIGHT) // 288 cm
// 288*288 cm map, divided into 16*16 cells of 18*18 cm each
// 82944 bytes for the occupancy grid

// The map is represented as a 2D grid of cells, where each cell can be in one of three states:
enum CellState
{
    CELL_UNKNOWN = -1,
    CELL_FREE = 0,
    CELL_OCCUPIED = 1
};

struct BotPose
{
    uint16_t x;  // centimeters
    uint16_t y;  // centimeters
    float theta; // radians
};

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
        botPose = {1, 1, 0.0f}; // Start at (1, 1) facing right (0 radians)
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
            updateCell(cellX, cellY, CELL_OCCUPIED);
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
            }
            myTrace.println(row);
        }
    }

private:
    int8_t occupancyGrid[MAP_WIDTH][MAP_HEIGHT];
    BotPose botPose; // Current pose of the robot
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