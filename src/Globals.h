#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

// onboard RGB LED
#define LED_RGB_PIN BOARD_LED_PIN
#define LED_RGB_NUMBER 1
#define LED_RGB_BRIGHTNESS 20 // Adjust brightness (0-255)

// Network Time Protocol
#define NTP_SERVER_EU "europe.pool.ntp.org"
#define NTP_REFRESH 3 * 60 * 60 // 3 hours
#define NTP_SYNC_TIMEOUT 60
#define NTP_TIMEZONE "Europe/Paris"

#define EVENT_ERROR std::function<void()>
#define EVENT_CHANGE std::function<void()>
#define EVENT_CHANGE_WITH_UINT8 std::function<void(uint8_t index)>
#define EVENT_CHANGE_WITH_UINT8_UINT16 std::function<void(uint8_t index, uint16_t value)>
#define EVENT_CHANGE_WITH_UINT16 std::function<void(uint16_t index)>
#define EVENT_CHANGE_WITH_FLOAT std::function<void(float value)>

#include "MyTrace.h"
extern MyTrace myTrace;

// Sensors
//#define PCA9548A_ADDRESS 0x70
//#define PCA9548A_RESET_PIN 42
#define BUS_TO_SCAN 6 // Number of I2C buses to scan for sensors (0 to 7 for PCA9548A)
#define VL53L0X_COUNT 5
#define VL53L0X_ERROR_READING_COUNT_THRESHOLD 5
#define COMPASS_CHANNEL 5
#define CAR_SENSOR_FRONT_INDEX 0
#define CAR_SENSOR_LEFT_INDEX 1
#define CAR_SENSOR_RIGHT_INDEX 2
#define CAR_SENSOR_TOP_LEFT_INDEX 3
#define CAR_SENSOR_TOP_RIGHT_INDEX 4
#define CAR_SENSOR_FRONT_DIRECTION 0.0f
#define CAR_SENSOR_LEFT_DIRECTION 90.0f
#define CAR_SENSOR_RIGHT_DIRECTION -90.0f
#define CAR_SENSOR_TOP_LEFT_DIRECTION 45.0f
#define CAR_SENSOR_TOP_RIGHT_DIRECTION -45.0f

#include "Sensors/AllSensors.h"
extern AllSensors allSensors;

// 288*288 cm map, divided into 16*16 cells of 18*18 cm each
// 82944 bytes for the occupancy grid (1 byte per cell), which can be stored in PSRAM.
// unit is centimeter
#define CELL_SIZE 1 // in centimeter. 
#define CELLS_BY_WIDTH 32 * 18
#define CELLS_BY_HEIGHT 32 * 18
#define MAP_WIDTH (CELL_SIZE * CELLS_BY_WIDTH) 
#define MAP_HEIGHT (CELL_SIZE * CELLS_BY_HEIGHT)
// The map is represented as a 2D grid of cells, where each cell can be in one of three states:
// With PSRAM, we can afford to store additional information for each cell, 
// such as the timestamp of the last update or a confidence level for the occupancy state. 
// This would allow us to implement more advanced features like decay of old information or probabilistic occupancy grids.

// The map is represented as a 2D grid of cells, where each cell can be in one of three states:
enum CellState : uint8_t
{
    // CELL_REDRAW = -2, // Special state to indicate that a cell needs to be redrawn on the screen (used for optimization)
    CELL_UNKNOWN = 0,
    CELL_FREE = 1 << 0,
    CELL_OCCUPIED = 1 << 1,
    CELL_PERHAPS_OCCUPIED = 1 << 2,
    CELL_NOT_CHANGED = 1 << 3 // Special flag to indicate that the cell state has not changed since the last time it was drawn on the screen (used for optimization)
};

inline CellState operator|(CellState lhs, CellState rhs) {
    using T = std::underlying_type_t<CellState>;
    return static_cast<CellState>(
        static_cast<T>(lhs) | static_cast<T>(rhs)
    );
}

inline CellState operator&(CellState lhs, CellState rhs) {
    using T = std::underlying_type_t<CellState>;
    return static_cast<CellState>(
        static_cast<T>(lhs) & static_cast<T>(rhs)
    );
}

inline CellState& operator|=(CellState& lhs, CellState rhs) {
    lhs = lhs | rhs;
    return lhs;
}

inline bool cellHasFlag(CellState value, CellState flag) {
    using T = std::underlying_type_t<CellState>;
    return (static_cast<T>(value) & static_cast<T>(flag)) != 0;
}

#define CELL_DISTANCE_THRESHOLD 100 // cm, if a reading is above this, we mark the cell as "perhaps occupied" instead of "occupied"

struct BotPose
{
    uint16_t x;  // centimeters
    uint16_t y;  // centimeters
    float theta; // degrees
};

#include "TheMap.h"
extern TheMap theMap;

#include "TheCar.h"
extern TheCar theCar;

#endif // GLOBALS_H