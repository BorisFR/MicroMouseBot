#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <elapsedMillis.h>
#include "Enums.h"
#include "Globals.h"
#include "TheCar.h"
#include "Sensors/AllSensors.h"

/// Navigation controller orchestrating autonomous robot movement.
/// Encapsulates state machine (INIT → MAPPING → NAVIGATE → ERROR),
/// local waypoint planning, heading control, and obstacle avoidance.
/// Decoupled from App for testability and reusability.
class RobotController
{
public:
    /// Constructor injects references to car and pose components.
    /// @param carRef Reference to TheCar for motion command execution
    /// @param poseXRef Reference to global pose X coordinate (cm)
    /// @param poseYRef Reference to global pose Y coordinate (cm)
    /// @param poseThetaRef Reference to global pose heading (degrees)
    RobotController(TheCar &carRef, const float &poseXRef, const float &poseYRef, const float &poseThetaRef)
        : theCar(carRef), poseX(poseXRef), poseY(poseYRef), poseTheta(poseThetaRef) {}

    /// Initialize controller state on robot startup.
    void setup()
    {
        robotState = RobotState::STATE_INIT;
        robotStateTimer = 0;
        navControlTimer = 0;
        navWaypoint.valid = false;
        activeMotionCommand = MotionCommand::CMD_STOP;
    }

    /// Main control loop. Call from App::loop() with sensor frame.
    /// @param hasFrame True if fresh sensor data available
    /// @param frame Latest sensor readings (distances, IMU heading)
    void tick(bool hasFrame, const AllSensors::SensorFrame &frame)
    {
        if (!AUTONOMOUS_NAV_ENABLED)
            return;

        updateRobotState(hasFrame);

        if (robotState == RobotState::STATE_ERROR)
        {
            issueStop();
            return;
        }

        if (robotState != RobotState::STATE_NAVIGATE)
        {
            issueStop();
            return;
        }

        if (navControlTimer < NAV_CONTROL_INTERVAL_MS)
            return;
        navControlTimer = 0;

        runNavigationController(frame);
    }

    /// @return Current robot state (INIT, MAPPING, NAVIGATE, ERROR, IDLE)
    RobotState getState() const { return robotState; }

    /// @return Last issued motion command (used by App for simulation sync)
    MotionCommand getActiveCommand() const { return activeMotionCommand; }

private:
    // ==================== Navigation Configuration ====================
    static constexpr bool AUTONOMOUS_NAV_ENABLED = true;      // Master enable
    static constexpr uint32_t NAV_CONTROL_INTERVAL_MS = 80;   // navigation loop frequency
    static constexpr uint32_t NAV_WARMUP_MS = 1200;           // MAPPING state duration before NAVIGATE
    static constexpr uint8_t NAV_FORWARD_SPEED = 110;         // Normal forward PWM speed
    static constexpr uint8_t NAV_TURN_SPEED = 95;             // Turn PWM speed
    static constexpr uint16_t NAV_FRONT_STOP_DISTANCE_CM = 14; // Stop if obstacle within this distance
    static constexpr uint16_t NAV_FRONT_SLOW_DISTANCE_CM = 24; // Reduce speed if within this distance
    static constexpr uint16_t NAV_REPLAN_DISTANCE_CM = 28;    // Replan waypoint if approaching within this distance
    static constexpr uint16_t NAV_WAYPOINT_REACHED_CM = 8;    // Accept waypoint reached within this radius
    static constexpr float NAV_TURN_HEADING_ERR_DEG = 16.0f;  // Max heading error before turning toward waypoint

    // ==================== Dependencies (Injected) ====================
    TheCar &theCar;            // Motor command interface
    const float &poseX;        // Reference to global X position (cm, float precision)
    const float &poseY;        // Reference to global Y position (cm, float precision)
    const float &poseTheta;    // Reference to global heading (degrees)

    // ==================== State Tracking ====================
    RobotState robotState = RobotState::STATE_INIT;
    elapsedMillis robotStateTimer;        // Time in current state
    elapsedMillis navControlTimer;        // Throttle navigation updates
    Waypoint navWaypoint;                 // Target waypoint for local path planning
    MotionCommand activeMotionCommand = MotionCommand::CMD_STOP;

    // ==================== Motor Command Execution ====================

    void issueForward(uint8_t speed)
    {
        theCar.moveForwardSpeed(speed);
        activeMotionCommand = MotionCommand::CMD_FORWARD;
    }

    void issueBackward(uint8_t speed)
    {
        theCar.moveBackwardSpeed(speed);
        activeMotionCommand = MotionCommand::CMD_BACKWARD;
    }

    void issueTurnLeft(uint8_t speed)
    {
        theCar.turnLeftSpeed(speed);
        activeMotionCommand = MotionCommand::CMD_TURN_LEFT;
    }

    void issueTurnRight(uint8_t speed)
    {
        theCar.turnRightSpeed(speed);
        activeMotionCommand = MotionCommand::CMD_TURN_RIGHT;
    }

    void issueStop()
    {
        theCar.stop();
        activeMotionCommand = MotionCommand::CMD_STOP;
    }

    // ==================== State Machine ====================

    /// Update robot state based on sensor availability and timer thresholds.
    void updateRobotState(bool hasFrame)
    {
        switch (robotState)
        {
        case RobotState::STATE_INIT:
            // Transition once first sensor frame arrives
            if (hasFrame)
                transitionRobotState(RobotState::STATE_MAPPING);
            break;

        case RobotState::STATE_MAPPING:
            // Monitor sensor health during warmup period
            if (!hasFrame)
            {
                transitionRobotState(RobotState::STATE_ERROR);
                break;
            }
            // Transition to navigation after warmup timeout
            if (robotStateTimer >= NAV_WARMUP_MS)
                transitionRobotState(RobotState::STATE_NAVIGATE);
            break;

        case RobotState::STATE_IDLE:
            // Idle state (unused, but valid per state machine)
            break;

        case RobotState::STATE_NAVIGATE:
            // Monitor sensor health; transition to error if frame lost
            if (!hasFrame)
                transitionRobotState(RobotState::STATE_ERROR);
            break;

        case RobotState::STATE_ERROR:
            // Error is terminal; requires manual reset
            break;
        }
    }

    /// Perform state transition with reset of local timers.
    void transitionRobotState(RobotState nextState)
    {
        robotState = nextState;
        robotStateTimer = 0;
        if (nextState == RobotState::STATE_NAVIGATE)
            navWaypoint.valid = false;  // Force initial waypoint plan
    }

    // ==================== Navigation Controller ====================

    /// Main navigation loop: update target waypoint, heading control, obstacle avoidance.
    void runNavigationController(const AllSensors::SensorFrame &frame)
    {
        const uint16_t front = frame.distances[CAR_SENSOR_FRONT_INDEX];
        const uint16_t left = frame.distances[CAR_SENSOR_LEFT_INDEX];
        const uint16_t right = frame.distances[CAR_SENSOR_RIGHT_INDEX];

        // ---- Obstacle Stop Logic ----
        if (front <= NAV_FRONT_STOP_DISTANCE_CM)
        {
            navWaypoint.valid = false;
            // Turn toward side with more space
            if (left >= right)
                issueTurnLeft(NAV_TURN_SPEED);
            else
                issueTurnRight(NAV_TURN_SPEED);
            return;
        }

        // ---- Waypoint Planning ----
        if (!navWaypoint.valid || isWaypointReached() || front < NAV_REPLAN_DISTANCE_CM)
            planLocalWaypoint(frame);

        if (!navWaypoint.valid)
        {
            issueStop();
            return;
        }

        // ---- Heading Control ----
        const float headingToWaypoint = getHeadingToWaypointDeg(navWaypoint);
        const float headingError = normalizeDeg(headingToWaypoint - poseTheta);
        const float absHeadingError = fabsf(headingError);

        if (absHeadingError > NAV_TURN_HEADING_ERR_DEG)
        {
            // Turn toward desired heading
            if (headingError > 0.0f)
                issueTurnLeft(NAV_TURN_SPEED);
            else
                issueTurnRight(NAV_TURN_SPEED);
            return;
        }

        // ---- Forward Motion with Speed Modulation ----
        const uint8_t commandedSpeed = (front < NAV_FRONT_SLOW_DISTANCE_CM) 
            ? static_cast<uint8_t>(NAV_FORWARD_SPEED * 0.65f) 
            : NAV_FORWARD_SPEED;
        issueForward(commandedSpeed);
    }

    /// Plan next local waypoint based on sensor readings and current pose.
    /// Considers front distance, side clearances, and map boundaries.
    void planLocalWaypoint(const AllSensors::SensorFrame &frame)
    {
        const uint16_t front = frame.distances[CAR_SENSOR_FRONT_INDEX];
        const uint16_t left = frame.distances[CAR_SENSOR_LEFT_INDEX];
        const uint16_t right = frame.distances[CAR_SENSOR_RIGHT_INDEX];

        // Default: continue forward
        float targetHeadingDeg = poseTheta;
        uint16_t travelCm = front;

        // If front blocked, redirect toward side with more space
        if (front < NAV_REPLAN_DISTANCE_CM)
        {
            targetHeadingDeg = (left >= right) 
                ? normalizeDeg(poseTheta + 90.0f) 
                : normalizeDeg(poseTheta - 90.0f);
            travelCm = (left >= right) ? left : right;
        }

        // Cannot plan if no clear path
        if (travelCm <= NAV_FRONT_STOP_DISTANCE_CM)
        {
            navWaypoint.valid = false;
            return;
        }

        // Plan waypoint at ~60% of available distance (safety margin), clamped to [12, 60] cm
        const float boundedTravelCm = constrain(static_cast<float>(travelCm) * 0.6f, 12.0f, 60.0f);
        const float headingRad = degToRad(targetHeadingDeg);
        const float targetX = poseX + boundedTravelCm * cosf(headingRad);
        const float targetY = poseY + boundedTravelCm * sinf(headingRad);

        // Clamp to map boundaries
        navWaypoint.x = clampToMapAxis(targetX, MAP_WIDTH);
        navWaypoint.y = clampToMapAxis(targetY, MAP_HEIGHT);
        navWaypoint.valid = true;
    }

    /// Check if current position is within acceptance radius of target waypoint.
    bool isWaypointReached() const
    {
        if (!navWaypoint.valid)
            return true;
        const float dx = static_cast<float>(navWaypoint.x) - poseX;
        const float dy = static_cast<float>(navWaypoint.y) - poseY;
        const float distanceSq = dx * dx + dy * dy;
        return distanceSq <= (NAV_WAYPOINT_REACHED_CM * NAV_WAYPOINT_REACHED_CM);
    }

    /// Calculate absolute heading (degrees) from current pose to waypoint.
    float getHeadingToWaypointDeg(const Waypoint &waypoint) const
    {
        const float dx = static_cast<float>(waypoint.x) - poseX;
        const float dy = static_cast<float>(waypoint.y) - poseY;
        return normalizeDeg(radToDeg(atan2f(dy, dx)));
    }

    // ==================== Helper Functions ====================
    // (Duplicated from App.h for independence; not via shared header to avoid over-engineering)

    /// Normalize angle to [-180, 180) degrees.
    static float normalizeDeg(float angleDeg)
    {
        while (angleDeg <= -180.0f)
            angleDeg += 360.0f;
        while (angleDeg > 180.0f)
            angleDeg -= 360.0f;
        return angleDeg;
    }

    /// Convert degrees to radians.
    static float degToRad(float deg)
    {
        return deg * (PI / 180.0f);
    }

    /// Convert radians to degrees.
    static float radToDeg(float rad)
    {
        return rad * (180.0f / PI);
    }

    /// Clamp value to map axis, ensuring it stays within [0, maxCm).
    static uint16_t clampToMapAxis(float valueCm, uint16_t maxCm)
    {
        if (valueCm < 0.0f)
            return 0;
        const float maxAxis = static_cast<float>(maxCm - 1);
        if (valueCm > maxAxis)
            return static_cast<uint16_t>(maxAxis);
        return static_cast<uint16_t>(valueCm);
    }
};

#endif // ROBOT_CONTROLLER_H
