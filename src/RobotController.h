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
    void setup();

    /// Main control loop. Call from App::loop() with sensor frame.
    /// @param hasFrame True if fresh sensor data available
    /// @param frame Latest sensor readings (distances, IMU heading)
    void tick(bool hasFrame, const AllSensors::SensorFrame &frame);

    /// @return Current robot state (INIT, MAPPING, NAVIGATE, ERROR, IDLE)
    RobotState getState() const;

    /// @return Last issued motion command (used by App for simulation sync)
    MotionCommand getActiveCommand() const;

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

    void issueForward(uint8_t speed);

    void issueBackward(uint8_t speed);

    void issueTurnLeft(uint8_t speed);

    void issueTurnRight(uint8_t speed);

    void issueStop();

    // ==================== State Machine ====================

    /// Update robot state based on sensor availability and timer thresholds.
    void updateRobotState(bool hasFrame);

    /// Perform state transition with reset of local timers.
    void transitionRobotState(RobotState nextState);

    // ==================== Navigation Controller ====================

    /// Main navigation loop: update target waypoint, heading control, obstacle avoidance.
    void runNavigationController(const AllSensors::SensorFrame &frame);

    /// Plan next local waypoint based on sensor readings and current pose.
    /// Considers front distance, side clearances, and map boundaries.
    void planLocalWaypoint(const AllSensors::SensorFrame &frame);

    /// Check if current position is within acceptance radius of target waypoint.
    bool isWaypointReached() const;

    /// Calculate absolute heading (degrees) from current pose to waypoint.
    float getHeadingToWaypointDeg(const Waypoint &waypoint) const;

    // ==================== Helper Functions ====================
    // (Duplicated from App.h for independence; not via shared header to avoid over-engineering)

    /// Normalize angle to [-180, 180) degrees.
    static float normalizeDeg(float angleDeg);

    /// Convert degrees to radians.
    static float degToRad(float deg);

    /// Convert radians to degrees.
    static float radToDeg(float rad);

    /// Clamp value to map axis, ensuring it stays within [0, maxCm).
    static uint16_t clampToMapAxis(float valueCm, uint16_t maxCm);
};

#endif // ROBOT_CONTROLLER_H
