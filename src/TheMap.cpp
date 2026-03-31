#include "TheMap.h"

TheMap::TheMap() : grid(MAP_WIDTH, MAP_HEIGHT) {}

TheMap::~TheMap() { myTrace.println("🗺️ unloaded"); }

void TheMap::setup()
{
    myTrace.println("🗺️ setup");
    if (grid.isValid())
    {
        myTrace.println("🗺️ PSRAM grid allocated successfully");
        String info = "🗺️ Grid dimensions: " + String(grid.numRows()) + " x " + String(grid.numCols());
        myTrace.println(info);
        info = "🗺️ Grid memory usage: " + String(grid.allocationSize() / 1024) + " KB";
        myTrace.println(info);
    }
    else
    {
        myTrace.println("🗺️ Failed to allocate PSRAM grid");
    }
    initializeGrid();
}

void TheMap::setPose(BotPose *poseRef)
{
    pose = poseRef;
}

void TheMap::loop()
{
}

CellState TheMap::getCellState(int x, int y) const
{
    if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
        return static_cast<CellState>(grid.at(x, y));
    return CELL_UNKNOWN;
}

void TheMap::setCellState(int x, int y, CellState state)
{
    if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
    {
        if (state != grid.at(x, y))
        {
            changedMap = true;
            grid.at(x, y) = state;
        }
    }
}

void TheMap::updateBotPose(uint16_t x, uint16_t y, float theta)
{
    if (!pose)
    {
        return;
    }
    pose->x = (x < MAP_WIDTH) ? x : static_cast<uint16_t>(MAP_WIDTH - 1);
    pose->y = (y < MAP_HEIGHT) ? y : static_cast<uint16_t>(MAP_HEIGHT - 1);
    pose->theta = theta;
}

void TheMap::updateWithLidarReadings(const std::vector<LidarReading> &readings)
{
    for (const auto &reading : readings)
    {
        if (!pose)
            continue;
        const float angleDeg = pose->theta + reading.angle;
        const float angleRad = angleDeg * PI / 180.0f;
        const uint16_t distance = reading.distance;
        const int cellX = static_cast<int>(pose->x + distance * cosf(angleRad));
        const int cellY = static_cast<int>(pose->y + distance * sinf(angleRad));
        markRayFree(pose->x, pose->y, cellX, cellY);
        if (distance > CELL_DISTANCE_THRESHOLD)
        {
            setCellState(cellX, cellY, CellState::CELL_PERHAPS_OCCUPIED);
        }
        else
        {
            setCellState(cellX, cellY, CellState::CELL_OCCUPIED);
        }
    }
}

bool TheMap::hasChanged()
{
    if (changedMap)
    {
        changedMap = false;
        return true;
    }
    return false;
}

void TheMap::printMap()
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
            if (state == CellState::CELL_UNKNOWN)
                row += "?";
            else if (state == CellState::CELL_FREE)
                row += ".";
            else if (state == CellState::CELL_OCCUPIED)
                row += "#";
            else if (state == CellState::CELL_PERHAPS_OCCUPIED)
                row += "o";
        }
        myTrace.println(row);
    }
}

void TheMap::initializeGrid()
{
    for (int x = 0; x < MAP_WIDTH; x++)
    {
        for (int y = 0; y < MAP_HEIGHT; y++)
        {
            grid.at(x, y) = CELL_UNKNOWN;
        }
    }
}

void TheMap::markRayFree(int startX, int startY, int endX, int endY)
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

        setCellState(x0, y0, CellState::CELL_FREE);

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
