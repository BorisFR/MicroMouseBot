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
#define CAR_SENSOR_FRONT_ROTATE_OFFSET 0.0f                                // degrees, rotation offset of the front sensor relative to the car's forward direction
#define CAR_SENSOR_LEFT_X_OFFSET -CAR_SENSOR_WIDTH / 2                     // cm, distance from the center of the car to the left edge where the left sensor is located
#define CAR_SENSOR_LEFT_Y_OFFSET (CAR_LENGTH / 2 - CAR_SENSOR_LENGTH / 2)  // cm, distance from the center of the car to the left edge where the left sensor is located
#define CAR_SENSOR_LEFT_ROTATE_OFFSET (90.0f)                             // degrees, rotation offset of the left sensor relative to the car's forward direction
#define CAR_SENSOR_RIGHT_X_OFFSET (CAR_WIDTH - CAR_SENSOR_WIDTH / 2)       // cm, distance from the center of the car to the right edge where the right sensor is located
#define CAR_SENSOR_RIGHT_Y_OFFSET (CAR_LENGTH / 2 - CAR_SENSOR_LENGTH / 2) // cm, distance from the center of the car to the right edge where the right sensor is located
#define CAR_SENSOR_RIGHT_ROTATE_OFFSET (-90.0f)                           // degrees, rotation offset of the right sensor relative to the car's forward direction
#define CAR_SENSOR_TOP_LEFT_X_OFFSET 0                                     // cm, distance from the center of the car to the top left edge where the top left sensor is located
#define CAR_SENSOR_TOP_LEFT_Y_OFFSET (CAR_SENSOR_WIDTH / 2)                // cm, distance from the center of the car to the top left edge where the top left sensor is located
#define CAR_SENSOR_TOP_LEFT_ROTATE_OFFSET (-45.0f)                        // degrees, rotation offset of the top left sensor relative to the car's forward direction
#define CAR_SENSOR_TOP_RIGHT_X_OFFSET (CAR_WIDTH - CAR_SENSOR_WIDTH)       // cm, distance from the center of the car to the top right edge where the top right sensor is located
#define CAR_SENSOR_TOP_RIGHT_Y_OFFSET (CAR_SENSOR_WIDTH / 2)               // cm, distance from the center of the car to the top right edge where the top right sensor is located
#define CAR_SENSOR_TOP_RIGHT_ROTATE_OFFSET (45.0f)                        // degrees, rotation offset of the top right sensor relative to the car's forward direction

// Viewport constants for map scrolling
#define MAP_CELL_PIXEL_SIZE 8                                              // pixels, size of each map cell on the screen
#define VIEWPORT_MARGIN 14                                                 // pixels, top and bottom margins for header/footer
#define SCROLL_BUTTON_SIZE 40                                              // pixels, size of scroll arrow buttons
#define SCROLL_STEP_CM 36                                                  // cm, scroll step (2 cells)

class TheScreen
{
public:
    TheScreen();

    ~TheScreen();

    /// @brief Initializes the screen and displays a welcome message. Also calculates the offsets for drawing the car and the map on the screen.
    void setup();

    void setPose(BotPose *poseRef);

    void setMap(TheMap *mapRef);

    /// @brief Main loop for the screen, currently only checks for touch input. In the future, this could be expanded to handle screen updates or animations.
    void loop();

    // ************************************************************************

    // for all screens

    void showVL53L0Xstate(int32_t index, bool isInitialized, bool isError, uint16_t distance = 0);

    void showGyro(bool isInitialized, bool isError, float value);

    void showMagnetometer(bool isInitialized, bool isError, float value);

    void showIMUstate(bool isInitialized, bool isError, bool isCalibrated, float value);

    // ************************************************************************

    // State of all components
    // The state screen will show the status of the I2C hub and all sensors, with a simple OK or Error icon next to each component

    void showStateScreen();

    void showErrorIcon(int32_t x, int32_t y, int32_t size);
    void showWarningIcon(int32_t x, int32_t y, int32_t size);

    void showOkIcon(int32_t x, int32_t y, int32_t size);

    void showHubState();

    void showAllSensorsState();

    // ************************************************************************

    // Car visualization on the screen
    // The car screen will show a top-down view of the car in the center of the screen, with simple rectangles representing the sensors.
    // The color of each sensor will indicate its status (e.g., green for OK, red for error, grey for not initialized).
    // The screen will also display the current orientation of the car using a simple arrow.

    void refreshSensorsOnScreen();

    void showSensor(uint8_t sensorIndex);

    void showIMU();

    void showCarOnScreen();

    // ************************************************************************

    // Map visualization on the screen
    // The map screen will show a grid representing the occupancy grid, with different colors for free, occupied, and unknown cells.
    // The robot's current position will be highlighted on the map.
    // To optimize the drawing of the map, we can keep track of the previous state of the occupancy grid and only redraw cells that have changed since the last update.

    void forceMapRedraw();

    /// @brief Updates viewport position. If locked, centers on robot. Otherwise keeps manual position.
    void updateViewport();

    /// @brief Scrolls the viewport by a delta in centimeters. Unlocks viewport for manual control.
    /// @param deltaX Delta in centimeters (horizontal)
    /// @param deltaY Delta in centimeters (vertical)
    void scrollViewport(int16_t deltaX, int16_t deltaY);

    /// @brief Locks or unlocks viewport to robot position
    /// @param locked true to auto-center on robot, false for manual control
    void setViewportLocked(bool locked);

    /// @brief Returns whether viewport is locked to robot
    bool isViewportLocked() const;

    /// @brief Centers viewport on robot position and locks it
    void centerViewportOnRobot();

    /// @brief Draws arrow buttons for manual map scrolling
    void drawScrollButtons();

    /// @brief Draws an indicator on the nearest border when the robot is off-screen.
    void drawRobotOffscreenIndicator();

    /// @brief Checks if touch coordinates are within a button area
    /// @return 0=none, 1=left, 2=right, 3=up, 4=down
    uint8_t getTouchedScrollButton(uint16_t x, uint16_t y);

    void showMap();

    // ************************************************************************

    // Utility functions

    void clearCenter();

    unsigned int colorFromRGB(uint8_t r, uint8_t g, uint8_t b);

    int32_t currentWidth();

    int32_t currentHeight();

    void fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);

    void fillRectRotated(int32_t x, int32_t y, int32_t w, int32_t h, float theta, uint32_t color);

    uint32_t blue();
    uint32_t red();
    uint32_t green();
    uint32_t yellow();
    uint32_t orange();

    // ************************************************************************

    // Touch input handling

    bool isTouched();

    void getTouchCoordinates(uint16_t &x, uint16_t &y);

    // ************************************************************************

    // Screen state management

    void changeScreen(ScreenState newState);

    void switchToNextScreen();

private:
    TFT_eSPI display; // Create an instance of the TFT_eSPI class
    uint16_t mapCellSize;
    uint16_t mapOffsetX;
    uint16_t mapOffsetY;

    // Viewport scrolling variables
    int16_t viewportCenterX;  // Viewport center X position in cm
    int16_t viewportCenterY;  // Viewport center Y position in cm
    bool viewportLocked;      // true = auto-center on robot, false = manual control

    uint16_t touchX = 0, touchY = 0;
    bool lastTouchState = false;
    ScreenState currentScreen = SCREEN_STATE;

    int32_t carOffsetX = 0;
    int32_t carOffsetY = 0;
    bool initialized = false;
    BotPose *pose = nullptr;
    TheMap *map = nullptr;
};

#endif // THE_SCREEN_H