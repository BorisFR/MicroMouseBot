#ifndef ENUMS_H
#define ENUMS_H

#include <stdint.h>

enum class MotionSelfTestState : uint8_t
{
    STATE_DISABLED,
    STATE_FORWARD,
    STATE_STOP_AFTER_FORWARD,
    STATE_TURN_LEFT,
    STATE_STOP_AFTER_TURN,
    STATE_BACKWARD,
    STATE_COMPLETE
};

enum class SimulationScenario : uint8_t
{
    SCENARIO_SELFTEST,
    SCENARIO_STRAIGHT,
    SCENARIO_SQUARE,
    SCENARIO_SPIN
};

enum class RobotState : uint8_t
{
    STATE_INIT,
    STATE_IDLE,
    STATE_MAPPING,
    STATE_NAVIGATE,
    STATE_ERROR
};

enum class MotionCommand : uint8_t
{
    CMD_STOP,
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT
};

struct Waypoint
{
    uint16_t x = 0;
    uint16_t y = 0;
    bool valid = false;
};

#endif // ENUMS_H