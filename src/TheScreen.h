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

enum ScreenState
{
    SCREEN_STATE,
    SCREEN_CAR,
    SCREEN_MAP,
    SCREEN_ERROR
};

#define CAR_SCALE_CM_TO_SCREEN 20                                          // 1 cm in the real world corresponds to 20 pixels on the screen (for visualization purposes)
#define CAR_WIDTH 8                                                        // cm
#define CAR_LENGTH 12                                                      // cm
#define CAR_SENSOR_WIDTH 1.5f                                              // cm, width of the front sensor (for visualization on the screen)
#define CAR_SENSOR_LENGTH 0.1f                                             // cm, length of the front sensor (for visualization on the screen)
#define CAR_SENSOR_FRONT_X_OFFSET (CAR_WIDTH / 2 - CAR_SENSOR_WIDTH / 2)   // cm, distance from the center of the car to the front edge where the front sensor is located
#define CAR_SENSOR_FRONT_Y_OFFSET 0                                        // cm, distance from the center of the car to the front edge where the front sensor is located
#define CAR_SENSOR_FRONT_ROTATE_OFFSET 0.0f                                // radians, rotation offset of the front sensor relative to the car's forward direction
#define CAR_SENSOR_LEFT_X_OFFSET -CAR_SENSOR_WIDTH / 2                     // cm, distance from the center of the car to the left edge where the left sensor is located
#define CAR_SENSOR_LEFT_Y_OFFSET (CAR_LENGTH / 2 - CAR_SENSOR_LENGTH / 2)  // cm, distance from the center of the car to the left edge where the left sensor is located
#define CAR_SENSOR_LEFT_ROTATE_OFFSET (PI / 2)                             // radians, rotation offset of the left sensor relative to the car's forward direction
#define CAR_SENSOR_RIGHT_X_OFFSET (CAR_WIDTH - CAR_SENSOR_WIDTH / 2)       // cm, distance from the center of the car to the right edge where the right sensor is located
#define CAR_SENSOR_RIGHT_Y_OFFSET (CAR_LENGTH / 2 - CAR_SENSOR_LENGTH / 2) // cm, distance from the center of the car to the right edge where the right sensor is located
#define CAR_SENSOR_RIGHT_ROTATE_OFFSET (-PI / 2)                           // radians, rotation offset of the right sensor relative to the car's forward direction
#define CAR_SENSOR_TOP_LEFT_X_OFFSET 0                                     // cm, distance from the center of the car to the top left edge where the top left sensor is located
#define CAR_SENSOR_TOP_LEFT_Y_OFFSET (CAR_SENSOR_WIDTH / 2)                // cm, distance from the center of the car to the top left edge where the top left sensor is located
#define CAR_SENSOR_TOP_LEFT_ROTATE_OFFSET (-PI / 4)                        // radians, rotation offset of the top left sensor relative to the car's forward direction
#define CAR_SENSOR_TOP_RIGHT_X_OFFSET (CAR_WIDTH - CAR_SENSOR_WIDTH)       // cm, distance from the center of the car to the top right edge where the top right sensor is located
#define CAR_SENSOR_TOP_RIGHT_Y_OFFSET (CAR_SENSOR_WIDTH / 2)               // cm, distance from the center of the car to the top right edge where the top right sensor is located
#define CAR_SENSOR_TOP_RIGHT_ROTATE_OFFSET (PI / 4)                        // radians, rotation offset of the top right sensor relative to the car's forward direction

class TheScreen
{
public:
    TheScreen() {}

    ~TheScreen() { myTrace.println("🖥️ unloaded"); }

    /// @brief Initializes the screen and displays a welcome message. Also calculates the offsets for drawing the car and the map on the screen.
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
        display.drawCentreString("(c) 2026 - by Boris", currentWidth() / 2, currentHeight() - 14 + 4, 1);
        display.drawRect(0, 14, currentWidth() - 1, currentHeight() - 28, TFT_BLUE);

        carOffsetX = (currentWidth() - CAR_WIDTH * CAR_SCALE_CM_TO_SCREEN) / 2;
        carOffsetY = (currentHeight() - CAR_LENGTH * CAR_SCALE_CM_TO_SCREEN) / 2;
        initialized = true;
        uint16_t mapCellWidth = currentWidth() / MAP_WIDTH;
        uint16_t mapCellHeight = currentHeight() / MAP_HEIGHT;
        mapCellSize = min(mapCellWidth, mapCellHeight);
        mapOffsetX = (currentWidth() - (mapCellSize * MAP_WIDTH)) / 2;
        mapOffsetY = (currentHeight() - (mapCellSize * MAP_HEIGHT)) / 2;
        forceMapRedraw();
    }

    void setPose(BotPose *poseRef)
    {
        pose = poseRef;
    }

    void setMap(const TheMap *mapRef)
    {
        map = mapRef;
    }

    /// @brief Main loop for the screen, currently only checks for touch input. In the future, this could be expanded to handle screen updates or animations.
    void loop()
    {
        isTouched();
    }

    // ************************************************************************

    // for all screens

    void showVL53L0Xstate(int32_t index, bool isInitialized, bool isError, uint16_t distance = 0)
    {
        if (currentScreen == SCREEN_STATE)
        {
            display.fillRect(4, 50 + index * 15, currentWidth() - 8, 15, TFT_BLACK); // Clear previous status
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

    void showIMUstate(bool isInitialized, bool isError, float value)
    {
        if (currentScreen == SCREEN_STATE)
        {
            display.fillRect(4, 50 + VL53L0X_COUNT * 15, currentWidth() - 8, 15, TFT_BLACK); // Clear previous status
            display.setTextColor(TFT_WHITE, TFT_BLACK);
            int32_t y = 50 + VL53L0X_COUNT * 15;
            if (isInitialized)
            {
                if (isError)
                {
                    showErrorIcon(4, y, 8);
                    display.drawString("IMU Error", 20, y, 1);
                }
                else
                {
                    showOkIcon(4, y, 8);
                    display.drawString("IMU OK (" + String(value, 2) + " rad)", 20, y, 1);
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

    // ************************************************************************

    // State of all components
    // The state screen will show the status of the I2C hub and all sensors, with a simple OK or Error icon next to each component

    void showStateScreen()
    {
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        display.drawCentreString("State", currentWidth() / 2, 20, 2);
    }

    void showErrorIcon(int32_t x, int32_t y, int32_t size)
    {
        // Draw a simple error icon (e.g., a red X) at the specified position
        display.fillRect(x, y, size, size, TFT_BLACK);
        display.drawLine(x, y, x + size, y + size, TFT_RED);
        display.drawLine(x + size, y, x, y + size, TFT_RED);
    }

    void showOkIcon(int32_t x, int32_t y, int32_t size)
    {
        // Draw a simple OK icon (e.g., a green checkmark) at the specified position
        display.fillRect(x, y, size, size, TFT_BLACK);
        display.drawLine(x, y + size / 2, x + size / 3, y + size, TFT_GREEN);
        display.drawLine(x + size / 3, y + size, x + size, y, TFT_GREEN);
    }

    void showHubState()
    {
        display.fillRect(4, 35, currentWidth() - 8, 15, TFT_BLACK); // Clear previous status
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

    void showAllSensorsState()
    {
        if (currentScreen != SCREEN_STATE)
            return; // Only show sensor status if we are in the STATE screen

        // display.fillRect(4, 50, currentWidth() - 8, VL53L0X_COUNT * 15, TFT_BLACK); // Clear previous sensor status
        // display.setTextColor(TFT_WHITE, TFT_BLACK);
        for (uint8_t i = 0; i < VL53L0X_COUNT; i++)
        {
            showVL53L0Xstate(i, allSensors.isSensorInitialized(i), allSensors.isSensorErrorDetected(i), allSensors.getLastDistance(i));
            /*int32_t y = 50 + i * 15;
            if (allSensors.isSensorInitialized(i))
            {
                if (allSensors.isSensorErrorDetected(i))
                {
                    showErrorIcon(4, y, 8);
                    display.drawString("Sensor " + String(i) + " Error", 20, y, 1);
                }
                else
                {
                    showOkIcon(4, y, 8);
                    display.drawString("Sensor " + String(i) + " OK", 20, y, 1);
                }
            }
            else
            {
                showErrorIcon(4, y, 8);
                display.drawString("Sensor " + String(i) + " Not Init", 20, y, 1);
            }*/
        }
        showIMUstate(allSensors.isIMUinitializedSuccessfully(), allSensors.isIMUErrorDetected(), allSensors.getHeading());
        /*if (allSensors.isIMUinitializedSuccessfully())
        {
            showOkIcon(4, 50 + VL53L0X_COUNT * 15, 8);
            display.drawString("IMU OK", 20, 50 + VL53L0X_COUNT * 15, 1);
        }
        else
        {
            showErrorIcon(4, 50 + VL53L0X_COUNT * 15, 8);
            display.drawString("IMU Not Init", 20, 50 + VL53L0X_COUNT * 15, 1);
        }*/
    }

    // ************************************************************************

    // Car visualization on the screen
    // The car screen will show a top-down view of the car in the center of the screen, with simple rectangles representing the sensors.
    // The color of each sensor will indicate its status (e.g., green for OK, red for error, grey for not initialized).
    // The screen will also display the current orientation of the car using a simple arrow.

    void refreshSensorsOnScreen()
    {
        if (currentScreen != SCREEN_CAR)
            return; // Only refresh sensors on screen if we are in the CAR screen state
        if (!initialized)
            return; // Don't show sensors on screen until the car is initialized
        showSensor(CAR_SENSOR_FRONT_INDEX);
        showSensor(CAR_SENSOR_LEFT_INDEX);
        showSensor(CAR_SENSOR_RIGHT_INDEX);
        showSensor(CAR_SENSOR_TOP_LEFT_INDEX);
        showSensor(CAR_SENSOR_TOP_RIGHT_INDEX);
    }

    void showSensor(uint8_t sensorIndex)
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
            fillRectRotated(carOffsetX + CAR_SENSOR_FRONT_X_OFFSET * CAR_SCALE_CM_TO_SCREEN, carOffsetY + CAR_SENSOR_FRONT_Y_OFFSET * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_FRONT_ROTATE_OFFSET, sensorColor); // Front sensor
            break;
        case CAR_SENSOR_LEFT_INDEX:
            fillRectRotated(carOffsetX + CAR_SENSOR_LEFT_X_OFFSET * CAR_SCALE_CM_TO_SCREEN, carOffsetY + CAR_SENSOR_LEFT_Y_OFFSET * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LEFT_ROTATE_OFFSET, sensorColor); // Left sensor
            break;
        case CAR_SENSOR_RIGHT_INDEX:
            fillRectRotated(carOffsetX + CAR_SENSOR_RIGHT_X_OFFSET * CAR_SCALE_CM_TO_SCREEN, carOffsetY + CAR_SENSOR_RIGHT_Y_OFFSET * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_RIGHT_ROTATE_OFFSET, sensorColor); // Right sensor
            break;
        case CAR_SENSOR_TOP_LEFT_INDEX:
            fillRectRotated(carOffsetX + CAR_SENSOR_TOP_LEFT_X_OFFSET * CAR_SCALE_CM_TO_SCREEN, carOffsetY + CAR_SENSOR_TOP_LEFT_Y_OFFSET * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_TOP_LEFT_ROTATE_OFFSET, sensorColor); // Top left sensor
            break;
        case CAR_SENSOR_TOP_RIGHT_INDEX:
            fillRectRotated(carOffsetX + CAR_SENSOR_TOP_RIGHT_X_OFFSET * CAR_SCALE_CM_TO_SCREEN, carOffsetY + CAR_SENSOR_TOP_RIGHT_Y_OFFSET * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * CAR_SCALE_CM_TO_SCREEN, CAR_SENSOR_TOP_RIGHT_ROTATE_OFFSET, sensorColor); // Top right sensor
            break;
        default:
            break;
        }
    }

    void showIMU()
    {
        if (currentScreen != SCREEN_CAR)
            return; // Only refresh sensors on screen if we are in the CAR screen state
        if (!initialized)
            return; // Don't show sensors on screen until the car is initialized
        if (!pose)
            return;
        // display the current orientation of the car using a simple arrow
        int arrowX = carOffsetX + CAR_WIDTH * CAR_SCALE_CM_TO_SCREEN / 2;
        int arrowY = carOffsetY + CAR_LENGTH * CAR_SCALE_CM_TO_SCREEN / 2;
        int arrowLength = 10;
        int arrowWidth = 3;
        int endX = arrowX + arrowLength * cos(pose->theta);
        int endY = arrowY - arrowLength * sin(pose->theta);
        display.fillCircle(arrowX, arrowY, arrowLength + 2, TFT_BLACK);
        // draw an arrow from (arrowX, arrowY) to (endX, endY) with the specified width and color
        display.drawLine(arrowX, arrowY, endX, endY, green());
    }

    void showCarOnScreen()
    {
        // Optionally, you can implement a method to show the car's position on the TFT screen
        fillRect(carOffsetX, carOffsetY, CAR_WIDTH * CAR_SCALE_CM_TO_SCREEN, CAR_LENGTH * CAR_SCALE_CM_TO_SCREEN, blue());
        refreshSensorsOnScreen();
        showIMU();
    }

    // ************************************************************************

    // Map visualization on the screen
    // The map screen will show a grid representing the occupancy grid, with different colors for free, occupied, and unknown cells.
    // The robot's current position will be highlighted on the map.
    // To optimize the drawing of the map, we can keep track of the previous state of the occupancy grid and only redraw cells that have changed since the last update.

    void forceMapRedraw()
    {
        for (int x = 0; x < MAP_WIDTH; x++)
        {
            for (int y = 0; y < MAP_HEIGHT; y++)
            {
                oldGrid[x][y] = CELL_REDRAW; // Force redraw of all cells on the map screen
            }
        }
    }

    void showMap()
    {
        if (currentScreen != SCREEN_MAP)
            return; // Only show map if we are in the MAP screen state
        if (!map)
            return;
        // Display the map on the TFT screen in a simple way (for example, as a grid of colored squares)
        // You can iterate through the occupancy grid and draw rectangles for each cell based on its state
        for (int x = 0; x < MAP_WIDTH; x++)
        {
            for (int y = 0; y < MAP_HEIGHT; y++)
            {
                CellState current = map->getCellState(x, y);
                if (current == oldGrid[x][y])
                {
                    continue; // No change, skip drawing
                }
                oldGrid[x][y] = current; // Update oldGrid with new value
                uint16_t color;
                if (pose && x == pose->x && y == pose->y)
                {
                    color = TFT_GREEN; // Robot's current position
                }
                else
                {
                    if (current == CELL_UNKNOWN)
                        color = TFT_GREY;
                    else if (current == CELL_FREE)
                        color = TFT_DARKGREEN;
                    else if (current == CELL_OCCUPIED)
                        color = TFT_RED;
                    else if (current == CELL_PERHAPS_OCCUPIED)
                        color = TFT_YELLOW;
                }
                display.fillRect(mapOffsetX + x * mapCellSize, mapOffsetY + y * mapCellSize, mapCellSize, mapCellSize, color);
            }
        }
        if (mapCellSize >= 3)
        {
            // Optionally, you can also draw grid lines
            for (int x = 0; x <= MAP_WIDTH; x++)
            {
                display.drawLine(mapOffsetX + x * mapCellSize, mapOffsetY, mapOffsetX + x * mapCellSize, mapOffsetY + MAP_HEIGHT * mapCellSize, TFT_BLACK);
            }
            for (int y = 0; y <= MAP_HEIGHT; y++)
            {
                display.drawLine(mapOffsetX, mapOffsetY + y * mapCellSize, mapOffsetX + MAP_WIDTH * mapCellSize, mapOffsetY + y * mapCellSize, TFT_BLACK);
            }
            // Optionally, you can also draw grid lines
            for (int x = 0; x <= MAP_WIDTH; x++)
            {
                display.drawLine(mapOffsetX + x * mapCellSize, mapOffsetY, mapOffsetX + x * mapCellSize, mapOffsetY + MAP_HEIGHT * mapCellSize, TFT_BLACK);
            }
            for (int y = 0; y <= MAP_HEIGHT; y++)
            {
                display.drawLine(mapOffsetX, mapOffsetY + y * mapCellSize, mapOffsetX + MAP_WIDTH * mapCellSize, mapOffsetY + y * mapCellSize, TFT_BLACK);
            }
        }
    }

    // ************************************************************************

    // Utility functions

    void clearCenter()
    {
        display.fillRect(1, 15, currentWidth() - 3, currentHeight() - 30, TFT_BLACK);
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

    void fillRectRotated(int32_t x, int32_t y, int32_t w, int32_t h, float theta, uint32_t color)
    {
        // Calculate the center of the rectangle
        float centerX = x + w / 2.0f;
        float centerY = y + h / 2.0f;

        // Calculate the corners of the rectangle before rotation
        float corners[4][2] = {
            {(float)x, (float)y},
            {(float)(x + w), (float)y},
            {(float)(x + w), (float)(y + h)},
            {(float)x, (float)(y + h)}};

        // Rotate each corner around the center
        for (int i = 0; i < 4; i++)
        {
            float dx = corners[i][0] - centerX;
            float dy = corners[i][1] - centerY;
            float rotatedX = dx * cos(theta) - dy * sin(theta);
            float rotatedY = dx * sin(theta) + dy * cos(theta);
            corners[i][0] = centerX + rotatedX;
            corners[i][1] = centerY + rotatedY;
        }

        // Draw the filled polygon using the rotated corners
        display.fillTriangle(corners[0][0], corners[0][1], corners[1][0], corners[1][1], corners[2][0], corners[2][1], color);
        display.fillTriangle(corners[0][0], corners[0][1], corners[2][0], corners[2][1], corners[3][0], corners[3][1], color);
    }

    uint32_t blue() { return TFT_BLUE; }
    uint32_t red() { return TFT_RED; }
    uint32_t green() { return TFT_GREEN; }
    uint32_t yellow() { return TFT_YELLOW; }
    uint32_t orange() { return colorFromRGB(255, 165, 0); }

    // ************************************************************************

    // Touch input handling

    bool isTouched()
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
            switchToNextScreen();
            changeScreen(currentScreen);
        }
        lastTouchState = touched;
        return touched;
    }

    void getTouchCoordinates(uint16_t &x, uint16_t &y)
    {
        x = touchX;
        y = touchY;
    }

    // ************************************************************************

    // Screen state management

    void changeScreen(ScreenState newState)
    {
        if (currentScreen == newState)
            return; // No change, skipping redraw
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

    void switchToNextScreen()
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

private:
    TFT_eSPI display; // Create an instance of the TFT_eSPI class
    uint16_t mapCellSize;
    uint16_t mapOffsetX;
    uint16_t mapOffsetY;
    CellState oldGrid[MAP_WIDTH][MAP_HEIGHT];

    uint16_t touchX = 0, touchY = 0;
    bool lastTouchState = false;
    ScreenState currentScreen = SCREEN_STATE;

    int32_t carOffsetX = 0;
    int32_t carOffsetY = 0;
    bool initialized = false;
    BotPose *pose = nullptr;
    const TheMap *map = nullptr;
};

#endif // THE_SCREEN_H