#include "TheScreen.h"
#include "TheMap.h"
#include "Sensors/AllSensors.h"

TheScreen::TheScreen() {}

TheScreen::~TheScreen() { myTrace.println("🖥️ unloaded"); }

void TheScreen::setup()
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
    display.drawCentreString("(c) 2026 - by Boris", currentWidth() / 2, currentHeight() - 14 + 4, 1);
    display.drawRect(0, 14, currentWidth() - 1, currentHeight() - 28, TFT_BLUE);

    carOffsetX = (currentWidth() - CAR_WIDTH * CAR_SCALE_CM_TO_SCREEN) / 2;
    carOffsetY = (currentHeight() - CAR_LENGTH * CAR_SCALE_CM_TO_SCREEN) / 2;
    initialized = true;

    mapCellSize = MAP_CELL_PIXEL_SIZE;
    viewportLocked = true;
    viewportCenterX = MAP_WIDTH / 2;
    viewportCenterY = MAP_HEIGHT / 2;
    mapOffsetX = 0;
    mapOffsetY = VIEWPORT_MARGIN;

    myTrace.println("Map cell size: " + String(mapCellSize) + " pixels, viewport initialized at center ("
        + String(viewportCenterX) + ", " + String(viewportCenterY) + " cm)");
    forceMapRedraw();
}

void TheScreen::setPose(BotPose *poseRef)
{
    pose = poseRef;
}

void TheScreen::setMap(TheMap *mapRef)
{
    map = mapRef;
}

void TheScreen::loop()
{
    isTouched();
}

void TheScreen::showVL53L0Xstate(int32_t index, bool isInitialized, bool isError, uint16_t distance)
{
    if (currentScreen == SCREEN_STATE)
    {
        display.fillRect(4, 50 + index * 15, currentWidth() - 8, 15, TFT_BLACK);
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        int32_t y = 50 + index * 15;
        if (isInitialized)
        {
            if (isError)
            {
                showErrorIcon(4, y, 8);
                display.drawString("Sensor " + String(index) + " Error", 20, y, 1);
            }
            else
            {
                showOkIcon(4, y, 8);
                display.drawString("Sensor " + String(index) + " OK (" + String(distance) + " cm)", 20, y, 1);
            }
        }
        else
        {
            showErrorIcon(4, y, 8);
            display.drawString("Sensor " + String(index) + " Not Init", 20, y, 1);
        }
        return;
    }
    if (currentScreen == SCREEN_CAR)
    {
        showSensor(index);
        return;
    }
}

void TheScreen::showGyro(bool isInitialized, bool isError, float value)
{
    if (currentScreen == SCREEN_STATE)
    {
        display.fillRect(4, 50 + VL53L0X_COUNT * 15, currentWidth() - 8, 15, TFT_BLACK);
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        int32_t y = 50 + VL53L0X_COUNT * 15;
        if (isInitialized)
        {
            if (isError)
            {
                showErrorIcon(4, y, 8);
                display.drawString("Gyro Error", 20, y, 1);
            }
            else
            {
                showOkIcon(4, y, 8);
                display.drawString("Gyro OK (" + String(value, 2) + " deg)", 20, y, 1);
            }
        }
        else
        {
            showErrorIcon(4, y, 8);
            display.drawString("Gyro Not Init", 20, y, 1);
        }
    }
}

void TheScreen::showMagnetometer(bool isInitialized, bool isError, float value)
{
    if (currentScreen == SCREEN_STATE)
    {
        display.fillRect(4, 50 + (VL53L0X_COUNT + 1) * 15, currentWidth() - 8, 15, TFT_BLACK);
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        int32_t y = 50 + (VL53L0X_COUNT + 1) * 15;
        if (isInitialized)
        {
            if (isError)
            {
                showErrorIcon(4, y, 8);
                display.drawString("Magnetometer Error", 20, y, 1);
            }
            else
            {
                showOkIcon(4, y, 8);
                display.drawString("Magnetometer OK (" + String(value, 2) + " deg)", 20, y, 1);
            }
        }
        else
        {
            showErrorIcon(4, y, 8);
            display.drawString("Magnetometer Not Init", 20, y, 1);
        }
    }
}

void TheScreen::showIMUstate(bool isInitialized, bool isError, bool isCalibrated, float value)
{
    if (currentScreen == SCREEN_STATE)
    {
        display.fillRect(4, 50 + (VL53L0X_COUNT + 2) * 15, currentWidth() - 8, 15, TFT_BLACK);
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        int32_t y = 50 + (VL53L0X_COUNT + 2) * 15;
        if (isInitialized)
        {
            if (isError)
            {
                showErrorIcon(4, y, 8);
                display.drawString("IMU Error", 20, y, 1);
            }
            else
            {
                if (isCalibrated)
                {
                    showOkIcon(4, y, 8);
                    display.drawString("IMU OK & Calibrated (" + String(value, 2) + " deg)", 20, y, 1);
                } else {
                    showWarningIcon(4, y, 8);
                    display.drawString("IMU OK (Not Calibrated) (" + String(value, 2) + " deg)", 20, y, 1);
                }
            }
        }
        else
        {
            showErrorIcon(4, y, 8);
            display.drawString("IMU Not Init", 20, y, 1);
        }
    }
    if (currentScreen == SCREEN_CAR)
    {
        showIMU();
        return;
    }
}

void TheScreen::showStateScreen()
{
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.drawCentreString("State", currentWidth() / 2, 20, 2);
}

void TheScreen::showErrorIcon(int32_t x, int32_t y, int32_t size)
{
    display.fillRect(x, y, size, size, TFT_BLACK);
    display.drawLine(x, y, x + size, y + size, TFT_RED);
    display.drawLine(x + size, y, x, y + size, TFT_RED);
}

void TheScreen::showWarningIcon(int32_t x, int32_t y, int32_t size)
{
    display.fillRect(x, y, size, size, TFT_BLACK);
    display.drawLine(x + size / 2, y, x + size / 2, y + size * 0.75, TFT_YELLOW);
    display.fillCircle(x + size / 2, y + size * 0.875, size / 8, TFT_YELLOW);
}

void TheScreen::showOkIcon(int32_t x, int32_t y, int32_t size)
{
    display.fillRect(x, y, size, size, TFT_BLACK);
    display.drawLine(x, y + size / 2, x + size / 3, y + size, TFT_GREEN);
    display.drawLine(x + size / 3, y + size, x + size, y, TFT_GREEN);
}

void TheScreen::showHubState()
{
    display.fillRect(4, 35, currentWidth() - 8, 15, TFT_BLACK);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    if (allSensors.isHubReady())
    {
        showOkIcon(4, 35, 8);
        display.drawString("I2C Hub Ready", 20, 35, 1);
    }
    else
    {
        showErrorIcon(4, 35, 8);
        display.drawString("I2C Hub Error", 20, 35, 1);
    }
}

void TheScreen::showAllSensorsState()
{
    if (currentScreen != SCREEN_STATE)
        return;

    for (uint8_t i = 0; i < VL53L0X_COUNT; i++)
    {
        showVL53L0Xstate(i, allSensors.isSensorInitialized(i), allSensors.isSensorErrorDetected(i), allSensors.getLastDistance(i));
    }
    showGyro(allSensors.isGyroReady(), allSensors.isGyroErrorDetected(), allSensors.getGyroHeading());
    showMagnetometer(allSensors.isMagnetometerReady(), allSensors.isMagnetometerErrorDetected(), allSensors.getMagnetometerHeading());
    showIMUstate(allSensors.isIMUinitializedSuccessfully(), allSensors.isIMUErrorDetected(), allSensors.isIMUCalibrated(), allSensors.getHeading());
}

void TheScreen::refreshSensorsOnScreen()
{
    if (currentScreen != SCREEN_CAR)
        return;
    if (!initialized)
        return;
    showSensor(CAR_SENSOR_FRONT_INDEX);
    showSensor(CAR_SENSOR_LEFT_INDEX);
    showSensor(CAR_SENSOR_RIGHT_INDEX);
    showSensor(CAR_SENSOR_TOP_LEFT_INDEX);
    showSensor(CAR_SENSOR_TOP_RIGHT_INDEX);
}

void TheScreen::showSensor(uint8_t sensorIndex)
{
    uint32_t sensorColor = red();
    if (allSensors.isSensorInitialized(sensorIndex))
    {
        sensorColor = orange();
        if (!allSensors.isSensorErrorDetected(sensorIndex))
        {
            sensorColor = green();
        }
    }
    switch (sensorIndex)
    {
    case CAR_SENSOR_FRONT_INDEX:
        fillRectRotated(carOffsetX + CAR_SENSOR_FRONT_X_OFFSET * CAR_SCALE_CM_TO_SCREEN, carOffsetY + CAR_SENSOR_FRONT_Y_OFFSET * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_FRONT_ROTATE_OFFSET, sensorColor);
        break;
    case CAR_SENSOR_LEFT_INDEX:
        fillRectRotated(carOffsetX + CAR_SENSOR_LEFT_X_OFFSET * CAR_SCALE_CM_TO_SCREEN, carOffsetY + CAR_SENSOR_LEFT_Y_OFFSET * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LEFT_ROTATE_OFFSET, sensorColor);
        break;
    case CAR_SENSOR_RIGHT_INDEX:
        fillRectRotated(carOffsetX + CAR_SENSOR_RIGHT_X_OFFSET * CAR_SCALE_CM_TO_SCREEN, carOffsetY + CAR_SENSOR_RIGHT_Y_OFFSET * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_RIGHT_ROTATE_OFFSET, sensorColor);
        break;
    case CAR_SENSOR_TOP_LEFT_INDEX:
        fillRectRotated(carOffsetX + CAR_SENSOR_TOP_LEFT_X_OFFSET * CAR_SCALE_CM_TO_SCREEN, carOffsetY + CAR_SENSOR_TOP_LEFT_Y_OFFSET * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_TOP_LEFT_ROTATE_OFFSET, sensorColor);
        break;
    case CAR_SENSOR_TOP_RIGHT_INDEX:
        fillRectRotated(carOffsetX + CAR_SENSOR_TOP_RIGHT_X_OFFSET * CAR_SCALE_CM_TO_SCREEN, carOffsetY + CAR_SENSOR_TOP_RIGHT_Y_OFFSET * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_TOP_RIGHT_ROTATE_OFFSET, sensorColor);
        break;
    default:
        break;
    }
}

void TheScreen::showIMU()
{
    if (currentScreen != SCREEN_CAR)
        return;
    if (!initialized)
        return;
    if (!pose)
        return;
    int32_t arrowX = carOffsetX + CAR_WIDTH * CAR_SCALE_CM_TO_SCREEN / 2;
    int32_t arrowY = carOffsetY + CAR_LENGTH * CAR_SCALE_CM_TO_SCREEN / 2;
    int32_t arrowLength = 10;
    const float thetaRad = pose->theta * PI / 180.0f;
    const int32_t endX = arrowX + arrowLength * cosf(thetaRad);
    const int32_t endY = arrowY - arrowLength * sinf(thetaRad);
    display.fillCircle(arrowX, arrowY, arrowLength + 2, TFT_BLACK);
    display.drawLine(arrowX, arrowY, endX, endY, green());
}

void TheScreen::showCarOnScreen()
{
    fillRect(carOffsetX, carOffsetY, CAR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_LENGTH * CAR_SCALE_CM_TO_SCREEN, blue());
    refreshSensorsOnScreen();
    showIMU();
}

void TheScreen::forceMapRedraw()
{
    if (!map)
        return;

    for (int x = 0; x < CELLS_BY_WIDTH; x++)
    {
        for (int y = 0; y < CELLS_BY_HEIGHT; y++)
        {
            CellState current = map->getCellState(x * CELL_SIZE, y * CELL_SIZE);
            current = static_cast<CellState>(
                static_cast<uint8_t>(current) & ~static_cast<uint8_t>(CellState::CELL_NOT_CHANGED)
            );
            map->setCellState(x * CELL_SIZE, y * CELL_SIZE, current);
        }
    }
}

void TheScreen::updateViewport()
{
    if (mapCellSize <= 0)
        mapCellSize = MAP_CELL_PIXEL_SIZE;

    if (viewportLocked && pose)
    {
        viewportCenterX = pose->x;
        viewportCenterY = pose->y;
    }
    int16_t halfViewportWidthCm = (currentWidth() * CELL_SIZE) / (2 * mapCellSize);
    int16_t halfViewportHeightCm = ((currentHeight() - 2 * VIEWPORT_MARGIN) * CELL_SIZE) / (2 * mapCellSize);

    if (halfViewportWidthCm < 1)
        halfViewportWidthCm = 1;
    if (halfViewportHeightCm < 1)
        halfViewportHeightCm = 1;

    if (viewportCenterX < halfViewportWidthCm)
        viewportCenterX = halfViewportWidthCm;
    if (viewportCenterX > MAP_WIDTH - halfViewportWidthCm)
        viewportCenterX = MAP_WIDTH - halfViewportWidthCm;
    if (viewportCenterY < halfViewportHeightCm)
        viewportCenterY = halfViewportHeightCm;
    if (viewportCenterY > MAP_HEIGHT - halfViewportHeightCm)
        viewportCenterY = MAP_HEIGHT - halfViewportHeightCm;
}

void TheScreen::scrollViewport(int16_t deltaX, int16_t deltaY)
{
    viewportLocked = false;
    viewportCenterX += deltaX;
    viewportCenterY += deltaY;
    updateViewport();
    forceMapRedraw();
}

void TheScreen::setViewportLocked(bool locked)
{
    viewportLocked = locked;
    if (locked)
    {
        forceMapRedraw();
    }
}

bool TheScreen::isViewportLocked() const { return viewportLocked; }

void TheScreen::centerViewportOnRobot()
{
    viewportLocked = true;
    updateViewport();
    forceMapRedraw();
}

void TheScreen::drawScrollButtons()
{
    uint16_t btnColor = viewportLocked ? TFT_DARKGREY : colorFromRGB(0, 100, 200);
    uint16_t arrowColor = TFT_WHITE;
    int16_t centerX = currentWidth() / 2;
    int16_t centerY = (currentHeight() - 2 * VIEWPORT_MARGIN) / 2 + VIEWPORT_MARGIN;

    int16_t leftX = 5;
    int16_t leftY = centerY - SCROLL_BUTTON_SIZE / 2;
    display.fillRoundRect(leftX, leftY, SCROLL_BUTTON_SIZE, SCROLL_BUTTON_SIZE, 5, btnColor);
    display.fillTriangle(leftX + 30, leftY + 10, leftX + 30, leftY + 30, leftX + 10, leftY + 20, arrowColor);

    int16_t rightX = currentWidth() - SCROLL_BUTTON_SIZE - 5;
    int16_t rightY = centerY - SCROLL_BUTTON_SIZE / 2;
    display.fillRoundRect(rightX, rightY, SCROLL_BUTTON_SIZE, SCROLL_BUTTON_SIZE, 5, btnColor);
    display.fillTriangle(rightX + 10, rightY + 10, rightX + 10, rightY + 30, rightX + 30, rightY + 20, arrowColor);

    int16_t upX = centerX - SCROLL_BUTTON_SIZE / 2;
    int16_t upY = VIEWPORT_MARGIN + 5;
    display.fillRoundRect(upX, upY, SCROLL_BUTTON_SIZE, SCROLL_BUTTON_SIZE, 5, btnColor);
    display.fillTriangle(upX + 10, upY + 30, upX + 30, upY + 30, upX + 20, upY + 10, arrowColor);

    int16_t downX = centerX - SCROLL_BUTTON_SIZE / 2;
    int16_t downY = currentHeight() - VIEWPORT_MARGIN - SCROLL_BUTTON_SIZE - 5;
    display.fillRoundRect(downX, downY, SCROLL_BUTTON_SIZE, SCROLL_BUTTON_SIZE, 5, btnColor);
    display.fillTriangle(downX + 10, downY + 10, downX + 30, downY + 10, downX + 20, downY + 30, arrowColor);
}

void TheScreen::drawRobotOffscreenIndicator()
{
    if (!pose)
        return;

    int16_t viewportWidthPixels = currentWidth();
    int16_t viewportHeightPixels = currentHeight() - 2 * VIEWPORT_MARGIN;
    int16_t centerX = currentWidth() / 2;
    int16_t centerY = viewportHeightPixels / 2 + VIEWPORT_MARGIN;

    int32_t robotScreenX = mapOffsetX + (static_cast<int32_t>(pose->x) - viewportCenterX) * mapCellSize / CELL_SIZE + viewportWidthPixels / 2;
    int32_t robotScreenY = mapOffsetY + (static_cast<int32_t>(pose->y) - viewportCenterY) * mapCellSize / CELL_SIZE + viewportHeightPixels / 2;

    bool robotOnScreen = (robotScreenX >= 0 && robotScreenX < currentWidth() &&
                          robotScreenY >= VIEWPORT_MARGIN && robotScreenY < currentHeight() - VIEWPORT_MARGIN);
    if (robotOnScreen)
        return;

    float dx = static_cast<float>(pose->x) - static_cast<float>(viewportCenterX);
    float dy = static_cast<float>(pose->y) - static_cast<float>(viewportCenterY);
    float distanceCm = sqrtf(dx * dx + dy * dy);
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f)
        return;
    float ux = dx / len;
    float uy = dy / len;

    int16_t xMin = 2;
    int16_t xMax = currentWidth() - 3;
    int16_t yMin = VIEWPORT_MARGIN + 2;
    int16_t yMax = currentHeight() - VIEWPORT_MARGIN - 3;

    float tMin = 1e9f;
    float ix = static_cast<float>(centerX);
    float iy = static_cast<float>(centerY);

    if (fabsf(ux) > 0.0001f)
    {
        float t = (xMin - centerX) / ux;
        float y = centerY + t * uy;
        if (t > 0.0f && y >= yMin && y <= yMax && t < tMin)
        {
            tMin = t;
            ix = static_cast<float>(xMin);
            iy = y;
        }
        t = (xMax - centerX) / ux;
        y = centerY + t * uy;
        if (t > 0.0f && y >= yMin && y <= yMax && t < tMin)
        {
            tMin = t;
            ix = static_cast<float>(xMax);
            iy = y;
        }
    }

    if (fabsf(uy) > 0.0001f)
    {
        float t = (yMin - centerY) / uy;
        float x = centerX + t * ux;
        if (t > 0.0f && x >= xMin && x <= xMax && t < tMin)
        {
            tMin = t;
            ix = x;
            iy = static_cast<float>(yMin);
        }
        t = (yMax - centerY) / uy;
        x = centerX + t * ux;
        if (t > 0.0f && x >= xMin && x <= xMax && t < tMin)
        {
            tMin = t;
            ix = x;
            iy = static_cast<float>(yMax);
        }
    }

    const int16_t arrowLen = 12;
    const int16_t arrowHalf = 6;
    float baseX = ix - ux * arrowLen;
    float baseY = iy - uy * arrowLen;
    float px = -uy;
    float py = ux;

    int16_t x1 = static_cast<int16_t>(ix);
    int16_t y1 = static_cast<int16_t>(iy);
    int16_t x2 = static_cast<int16_t>(baseX + px * arrowHalf);
    int16_t y2 = static_cast<int16_t>(baseY + py * arrowHalf);
    int16_t x3 = static_cast<int16_t>(baseX - px * arrowHalf);
    int16_t y3 = static_cast<int16_t>(baseY - py * arrowHalf);

    display.fillTriangle(x1, y1, x2, y2, x3, y3, orange());

    String label = String(static_cast<uint16_t>(distanceCm)) + " cm";
    int16_t textX = static_cast<int16_t>(baseX - ux * 10);
    int16_t textY = static_cast<int16_t>(baseY - uy * 10);
    if (textY < VIEWPORT_MARGIN)
        textY = VIEWPORT_MARGIN;
    if (textY > currentHeight() - VIEWPORT_MARGIN - 8)
        textY = currentHeight() - VIEWPORT_MARGIN - 8;
    if (textX < 0)
        textX = 0;
    if (textX > currentWidth() - 30)
        textX = currentWidth() - 30;
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.drawString(label, textX - 12, textY - 6, 1);
}

uint8_t TheScreen::getTouchedScrollButton(uint16_t x, uint16_t y)
{
    int16_t centerX = currentWidth() / 2;
    int16_t centerY = (currentHeight() - 2 * VIEWPORT_MARGIN) / 2 + VIEWPORT_MARGIN;

    if (x >= 5 && x <= 5 + SCROLL_BUTTON_SIZE &&
        y >= centerY - SCROLL_BUTTON_SIZE / 2 && y <= centerY + SCROLL_BUTTON_SIZE / 2)
        return 1;

    if (x >= currentWidth() - SCROLL_BUTTON_SIZE - 5 && x <= currentWidth() - 5 &&
        y >= centerY - SCROLL_BUTTON_SIZE / 2 && y <= centerY + SCROLL_BUTTON_SIZE / 2)
        return 2;

    if (x >= centerX - SCROLL_BUTTON_SIZE / 2 && x <= centerX + SCROLL_BUTTON_SIZE / 2 &&
        y >= VIEWPORT_MARGIN + 5 && y <= VIEWPORT_MARGIN + 5 + SCROLL_BUTTON_SIZE)
        return 3;

    if (x >= centerX - SCROLL_BUTTON_SIZE / 2 && x <= centerX + SCROLL_BUTTON_SIZE / 2 &&
        y >= currentHeight() - VIEWPORT_MARGIN - SCROLL_BUTTON_SIZE - 5 &&
        y <= currentHeight() - VIEWPORT_MARGIN - 5)
        return 4;

    return 0;
}

void TheScreen::showMap()
{
    if (currentScreen != SCREEN_MAP)
        return;
    if (!map)
        return;
    if (mapCellSize <= 0)
        mapCellSize = MAP_CELL_PIXEL_SIZE;

    updateViewport();

    int16_t viewportWidthPixels = currentWidth();
    int16_t viewportHeightPixels = currentHeight() - 2 * VIEWPORT_MARGIN;

    int16_t visibleCellsX = (viewportWidthPixels / mapCellSize) + 1;
    int16_t visibleCellsY = (viewportHeightPixels / mapCellSize) + 1;
    if (visibleCellsX < 1)
        visibleCellsX = 1;
    if (visibleCellsY < 1)
        visibleCellsY = 1;

    int16_t centerCellX = viewportCenterX / CELL_SIZE;
    int16_t centerCellY = viewportCenterY / CELL_SIZE;

    int16_t startCellX = centerCellX - visibleCellsX / 2;
    int16_t endCellX = centerCellX + visibleCellsX / 2 + 1;
    int16_t startCellY = centerCellY - visibleCellsY / 2;
    int16_t endCellY = centerCellY + visibleCellsY / 2 + 1;

    if (startCellX < 0) startCellX = 0;
    if (endCellX > CELLS_BY_WIDTH) endCellX = CELLS_BY_WIDTH;
    if (startCellY < 0) startCellY = 0;
    if (endCellY > CELLS_BY_HEIGHT) endCellY = CELLS_BY_HEIGHT;

    for (int16_t cellX = startCellX; cellX < endCellX; cellX++)
    {
        for (int16_t cellY = startCellY; cellY < endCellY; cellY++)
        {
            int32_t cellCmX = cellX * CELL_SIZE;
            int32_t cellCmY = cellY * CELL_SIZE;

            CellState current = map->getCellState(cellCmX, cellCmY);

            if(cellHasFlag(current, CellState::CELL_NOT_CHANGED))
            {
                continue;
            }

            current = static_cast<CellState>(
                static_cast<uint8_t>(current) & ~static_cast<uint8_t>(CellState::CELL_NOT_CHANGED)
            );

            uint16_t color;
            if (pose && abs(cellCmX - static_cast<int32_t>(pose->x)) < CELL_SIZE && abs(cellCmY - static_cast<int32_t>(pose->y)) < CELL_SIZE)
            {
                color = TFT_GREEN;
            }
            else
            {
                if (current == CellState::CELL_UNKNOWN)
                    color = TFT_GREY;
                else if (current == CellState::CELL_FREE)
                    color = TFT_CYAN;
                else if (current == CellState::CELL_OCCUPIED)
                    color = TFT_RED;
                else if (current == CellState::CELL_PERHAPS_OCCUPIED)
                    color = TFT_YELLOW;
                else
                    color = TFT_GREY;
            }

            int32_t screenX = mapOffsetX + (cellCmX - viewportCenterX) * mapCellSize / CELL_SIZE + viewportWidthPixels / 2;
            int32_t screenY = mapOffsetY + (cellCmY - viewportCenterY) * mapCellSize / CELL_SIZE + viewportHeightPixels / 2;

            if (screenX + mapCellSize < 0 || screenX >= currentWidth() ||
                screenY + mapCellSize < VIEWPORT_MARGIN || screenY >= currentHeight() - VIEWPORT_MARGIN)
            {
                current |= CellState::CELL_NOT_CHANGED;
                map->setCellState(cellCmX, cellCmY, current);
                continue;
            }

            display.fillRect(screenX, screenY, mapCellSize, mapCellSize, color);
            display.drawRect(screenX, screenY, mapCellSize, mapCellSize, TFT_BLACK);

            current |= CellState::CELL_NOT_CHANGED;
            map->setCellState(cellCmX, cellCmY, current);
        }
    }

    drawScrollButtons();
    drawRobotOffscreenIndicator();
}

void TheScreen::clearCenter()
{
    display.fillRect(1, 15, currentWidth() - 3, currentHeight() - 30, TFT_BLACK);
}

unsigned int TheScreen::colorFromRGB(uint8_t r, uint8_t g, uint8_t b)
{
    return display.color565(r, g, b);
}

int32_t TheScreen::currentWidth()
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
    return 0;
}

int32_t TheScreen::currentHeight()
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
    return 0;
}

void TheScreen::fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
    display.fillRect(x, y, w, h, color);
}

void TheScreen::fillRectRotated(int32_t x, int32_t y, int32_t w, int32_t h, float theta, uint32_t color)
{
    float thetaRad = theta * PI / 180.0f;
    float centerX = x + w / 2.0f;
    float centerY = y + h / 2.0f;

    float corners[4][2] = {
        {(float)x, (float)y},
        {(float)(x + w), (float)y},
        {(float)(x + w), (float)(y + h)},
        {(float)x, (float)(y + h)}};

    for (int i = 0; i < 4; i++)
    {
        float dx = corners[i][0] - centerX;
        float dy = corners[i][1] - centerY;
        float rotatedX = dx * cos(thetaRad) - dy * sin(thetaRad);
        float rotatedY = dx * sin(thetaRad) + dy * cos(thetaRad);
        corners[i][0] = centerX + rotatedX;
        corners[i][1] = centerY + rotatedY;
    }

    display.fillTriangle(corners[0][0], corners[0][1], corners[1][0], corners[1][1], corners[2][0], corners[2][1], color);
    display.fillTriangle(corners[0][0], corners[0][1], corners[2][0], corners[2][1], corners[3][0], corners[3][1], color);
}

uint32_t TheScreen::blue() { return TFT_BLUE; }
uint32_t TheScreen::red() { return TFT_RED; }
uint32_t TheScreen::green() { return TFT_GREEN; }
uint32_t TheScreen::yellow() { return TFT_YELLOW; }
uint32_t TheScreen::orange() { return colorFromRGB(255, 165, 0); }

bool TheScreen::isTouched()
{
    uint16_t x, y;
    bool touched = display.getTouch(&x, &y);
    if (touched)
    {
        touchX = x;
        touchY = y;
    }
    if (touched && !lastTouchState)
    {
        if (currentScreen == SCREEN_MAP)
        {
            uint8_t button = getTouchedScrollButton(x, y);
            if (button > 0)
            {
                switch (button)
                {
                case 1:
                    scrollViewport(-SCROLL_STEP_CM, 0);
                    showMap();
                    break;
                case 2:
                    scrollViewport(SCROLL_STEP_CM, 0);
                    showMap();
                    break;
                case 3:
                    scrollViewport(0, -SCROLL_STEP_CM);
                    showMap();
                    break;
                case 4:
                    scrollViewport(0, SCROLL_STEP_CM);
                    showMap();
                    break;
                }
            }
            else
            {
                if (!viewportLocked)
                {
                    centerViewportOnRobot();
                    showMap();
                }
                else
                {
                    switchToNextScreen();
                    changeScreen(currentScreen);
                }
            }
        }
        else
        {
            switchToNextScreen();
            changeScreen(currentScreen);
        }
    }
    lastTouchState = touched;
    return touched;
}

void TheScreen::getTouchCoordinates(uint16_t &x, uint16_t &y)
{
    x = touchX;
    y = touchY;
}

void TheScreen::changeScreen(ScreenState newState)
{
    if (currentScreen == newState)
        return;
    currentScreen = newState;
    clearCenter();
    switch (currentScreen)
    {
    case SCREEN_STATE:
        showStateScreen();
        showHubState();
        showAllSensorsState();
        break;
    case SCREEN_MAP:
        forceMapRedraw();
        showMap();
        break;
    case SCREEN_CAR:
        showCarOnScreen();
        break;
    case SCREEN_ERROR:
        display.setTextColor(TFT_RED, TFT_BLACK);
        display.drawCentreString("Error Occurred", currentWidth() / 2, currentHeight() / 2, 2);
        break;
    }
}

void TheScreen::switchToNextScreen()
{
    if (currentScreen == SCREEN_STATE)
    {
        changeScreen(SCREEN_MAP);
    }
    else if (currentScreen == SCREEN_MAP)
    {
        changeScreen(SCREEN_CAR);
    }
    else if (currentScreen == SCREEN_CAR)
    {
        changeScreen(SCREEN_ERROR);
    }
    else
    {
        changeScreen(SCREEN_STATE);
    }
    myTrace.print("Switched to screen: ");
    switch (currentScreen)
    {
    case SCREEN_STATE:
        myTrace.println("State");
        break;
    case SCREEN_CAR:
        myTrace.println("Car");
        break;
    case SCREEN_MAP:
        myTrace.println("Map");
        break;
    case SCREEN_ERROR:
        myTrace.println("Error");
        break;
    }
}
