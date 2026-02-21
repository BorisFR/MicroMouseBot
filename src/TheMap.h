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

    void setPose(BotPose *poseRef)
    {
        pose = poseRef;
    }

    void loop()
    {
    }

    CellState getCellState(int x, int y) const
    {
        if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
        {
            return static_cast<CellState>(grid[x][y]);
        }
        return CELL_UNKNOWN; // Out of bounds
    }

    void updateCell(int x, int y, CellState state)
    {
        if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
        {
            if (state != grid[x][y])
            {
                changedMap = true;
                grid[x][y] = state;
            }
        }
    }

    void updateBotPose(uint16_t x, uint16_t y, float theta)
    {
        if (!pose)
        {
            return;
        }
        pose->x = x;
        pose->y = y;
        pose->theta = theta;
    }

    void updateWithLidarReadings(const std::vector<LidarReading> &readings)
    {
        for (const auto &reading : readings)
        {
            // Convert polar coordinates to Cartesian
            if (!pose)
            {
                continue;
            }
            float angle = pose->theta + reading.angle;
            uint16_t distance = reading.distance;
            int cellX = static_cast<int>(pose->x + distance * cos(angle));
            int cellY = static_cast<int>(pose->y + distance * sin(angle));
            markRayFree(pose->x, pose->y, cellX, cellY);
            if (distance > CELL_DISTANCE_THRESHOLD)
            {
                updateCell(cellX, cellY, CELL_PERHAPS_OCCUPIED);
            }
            else
            {
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
                if (pose && x == pose->x && y == pose->y)
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
    BotPose *pose = nullptr;

    void initializeGrid()
    {
        for (int x = 0; x < MAP_WIDTH; x++)
        {
            for (int y = 0; y < MAP_HEIGHT; y++)
            {
                grid[x][y] = CELL_UNKNOWN;
            }
        }
    }

    void markRayFree(int startX, int startY, int endX, int endY)
    {
        int x0 = startX;
        int y0 = startY;
        int x1 = endX;
        int y1 = endY;

        int dx = abs(x1 - x0);
        int sx = (x0 < x1) ? 1 : -1;
        int dy = -abs(y1 - y0);
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy;

        while (true)
        {
            if (x0 == x1 && y0 == y1)
            {
                break;
            }

            updateCell(x0, y0, CELL_FREE);

            int e2 = 2 * err;
            if (e2 >= dy)
            {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx)
            {
                err += dx;
                y0 += sy;
            }

            if (x0 < 0 || x0 >= MAP_WIDTH || y0 < 0 || y0 >= MAP_HEIGHT)
            {
                break;
            }
        }
    }

    CellState grid[MAP_WIDTH][MAP_HEIGHT];
};

#endif // THE_MAP_H