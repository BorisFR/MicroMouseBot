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

// Display


#include "MyTrace.h"
extern MyTrace myTrace;


// unit is centimeter
#define CELL_SIZE 18
#define CELLS_BY_WIDTH 5
#define CELLS_BY_HEIGHT 5
#define MAP_WIDTH (CELL_SIZE * CELLS_BY_WIDTH)   // 288 cm
#define MAP_HEIGHT (CELL_SIZE * CELLS_BY_HEIGHT) // 288 cm
// 288*288 cm map, divided into 16*16 cells of 18*18 cm each
// 82944 bytes for the occupancy grid

// The map is represented as a 2D grid of cells, where each cell can be in one of three states:
enum CellState
{
    CELL_UNKNOWN = -1,
    CELL_FREE = 0,
    CELL_OCCUPIED = 1,
    CELL_PERHAPS_OCCUPIED = 2
};

#define CELL_DISTANCE_THRESHOLD 100 // cm, if a reading is above this, we mark the cell as "perhaps occupied" instead of "occupied"

extern CellState occupancyGrid[MAP_WIDTH][MAP_HEIGHT];

struct BotPose
{
    uint16_t x;  // centimeters
    uint16_t y;  // centimeters
    float theta; // radians
};

extern BotPose botPose; // Current pose of the robot

#endif // GLOBALS_H