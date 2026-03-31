#ifndef THE_MAP_H
#define THE_MAP_H

#include "Globals.h"
#include <vector>
#include <string>
#include "PSRAM2DArray.h"

struct LidarReading
{
    float angle;       // degrees
    uint16_t distance; // centimeters
};

class TheMap
{
public:
    TheMap();

    ~TheMap();

    void setup();

    void setPose(BotPose *poseRef);

    void loop();

    CellState getCellState(int x, int y) const;

    void setCellState(int x, int y, CellState state);

    void updateBotPose(uint16_t x, uint16_t y, float theta);

    void updateWithLidarReadings(const std::vector<LidarReading> &readings);

    bool hasChanged();

    void printMap();

private:
    bool changedMap = false;
    BotPose *pose = nullptr;

    void initializeGrid();

    void markRayFree(int startX, int startY, int endX, int endY);

    // CellState grid[MAP_WIDTH][MAP_HEIGHT];
    PSRAM2DArray<CellState> grid; //(MAP_WIDTH, MAP_HEIGHT);
};

#endif // THE_MAP_H