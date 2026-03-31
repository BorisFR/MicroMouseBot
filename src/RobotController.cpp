#include "RobotController.h"

void RobotController::setup()
{
    robotState = RobotState::STATE_INIT;
    robotStateTimer = 0;
    navControlTimer = 0;
    navWaypoint.valid = false;
    activeMotionCommand = MotionCommand::CMD_STOP;
}

void RobotController::tick(bool hasFrame, const AllSensors::SensorFrame &frame)
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

RobotState RobotController::getState() const { return robotState; }

MotionCommand RobotController::getActiveCommand() const { return activeMotionCommand; }

void RobotController::issueForward(uint8_t speed)
{
    theCar.moveForwardSpeed(speed);
    activeMotionCommand = MotionCommand::CMD_FORWARD;
}

void RobotController::issueBackward(uint8_t speed)
{
    theCar.moveBackwardSpeed(speed);
    activeMotionCommand = MotionCommand::CMD_BACKWARD;
}

void RobotController::issueTurnLeft(uint8_t speed)
{
    theCar.turnLeftSpeed(speed);
    activeMotionCommand = MotionCommand::CMD_TURN_LEFT;
}

void RobotController::issueTurnRight(uint8_t speed)
{
    theCar.turnRightSpeed(speed);
    activeMotionCommand = MotionCommand::CMD_TURN_RIGHT;
}

void RobotController::issueStop()
{
    theCar.stop();
    activeMotionCommand = MotionCommand::CMD_STOP;
}

void RobotController::updateRobotState(bool hasFrame)
{
    switch (robotState)
    {
    case RobotState::STATE_INIT:
        if (hasFrame)
            transitionRobotState(RobotState::STATE_MAPPING);
        break;

    case RobotState::STATE_MAPPING:
        if (!hasFrame)
        {
            transitionRobotState(RobotState::STATE_ERROR);
            break;
        }
        if (robotStateTimer >= NAV_WARMUP_MS)
            transitionRobotState(RobotState::STATE_NAVIGATE);
        break;

    case RobotState::STATE_IDLE:
        break;

    case RobotState::STATE_NAVIGATE:
        if (!hasFrame)
            transitionRobotState(RobotState::STATE_ERROR);
        break;

    case RobotState::STATE_ERROR:
        break;
    }
}

void RobotController::transitionRobotState(RobotState nextState)
{
    robotState = nextState;
    robotStateTimer = 0;
    if (nextState == RobotState::STATE_NAVIGATE)
        navWaypoint.valid = false;
}

void RobotController::runNavigationController(const AllSensors::SensorFrame &frame)
{
    const uint16_t front = frame.distances[CAR_SENSOR_FRONT_INDEX];
    const uint16_t left = frame.distances[CAR_SENSOR_LEFT_INDEX];
    const uint16_t right = frame.distances[CAR_SENSOR_RIGHT_INDEX];

    if (front <= NAV_FRONT_STOP_DISTANCE_CM)
    {
        navWaypoint.valid = false;
        if (left >= right)
            issueTurnLeft(NAV_TURN_SPEED);
        else
            issueTurnRight(NAV_TURN_SPEED);
        return;
    }

    if (!navWaypoint.valid || isWaypointReached() || front < NAV_REPLAN_DISTANCE_CM)
        planLocalWaypoint(frame);

    if (!navWaypoint.valid)
    {
        issueStop();
        return;
    }

    const float headingToWaypoint = getHeadingToWaypointDeg(navWaypoint);
    const float headingError = normalizeDeg(headingToWaypoint - poseTheta);
    const float absHeadingError = fabsf(headingError);

    if (absHeadingError > NAV_TURN_HEADING_ERR_DEG)
    {
        if (headingError > 0.0f)
            issueTurnLeft(NAV_TURN_SPEED);
        else
            issueTurnRight(NAV_TURN_SPEED);
        return;
    }

    const uint8_t commandedSpeed = (front < NAV_FRONT_SLOW_DISTANCE_CM)
        ? static_cast<uint8_t>(NAV_FORWARD_SPEED * 0.65f)
        : NAV_FORWARD_SPEED;
    issueForward(commandedSpeed);
}

void RobotController::planLocalWaypoint(const AllSensors::SensorFrame &frame)
{
    const uint16_t front = frame.distances[CAR_SENSOR_FRONT_INDEX];
    const uint16_t left = frame.distances[CAR_SENSOR_LEFT_INDEX];
    const uint16_t right = frame.distances[CAR_SENSOR_RIGHT_INDEX];

    float targetHeadingDeg = poseTheta;
    uint16_t travelCm = front;

    if (front < NAV_REPLAN_DISTANCE_CM)
    {
        targetHeadingDeg = (left >= right)
            ? normalizeDeg(poseTheta + 90.0f)
            : normalizeDeg(poseTheta - 90.0f);
        travelCm = (left >= right) ? left : right;
    }

    if (travelCm <= NAV_FRONT_STOP_DISTANCE_CM)
    {
        navWaypoint.valid = false;
        return;
    }

    const float boundedTravelCm = constrain(static_cast<float>(travelCm) * 0.6f, 12.0f, 60.0f);
    const float headingRad = degToRad(targetHeadingDeg);
    const float targetX = poseX + boundedTravelCm * cosf(headingRad);
    const float targetY = poseY + boundedTravelCm * sinf(headingRad);

    navWaypoint.x = clampToMapAxis(targetX, MAP_WIDTH);
    navWaypoint.y = clampToMapAxis(targetY, MAP_HEIGHT);
    navWaypoint.valid = true;
}

bool RobotController::isWaypointReached() const
{
    if (!navWaypoint.valid)
        return true;
    const float dx = static_cast<float>(navWaypoint.x) - poseX;
    const float dy = static_cast<float>(navWaypoint.y) - poseY;
    const float distanceSq = dx * dx + dy * dy;
    return distanceSq <= (NAV_WAYPOINT_REACHED_CM * NAV_WAYPOINT_REACHED_CM);
}

float RobotController::getHeadingToWaypointDeg(const Waypoint &waypoint) const
{
    const float dx = static_cast<float>(waypoint.x) - poseX;
    const float dy = static_cast<float>(waypoint.y) - poseY;
    return normalizeDeg(radToDeg(atan2f(dy, dx)));
}

float RobotController::normalizeDeg(float angleDeg)
{
    while (angleDeg <= -180.0f)
        angleDeg += 360.0f;
    while (angleDeg > 180.0f)
        angleDeg -= 360.0f;
    return angleDeg;
}

float RobotController::degToRad(float deg)
{
    return deg * (PI / 180.0f);
}

float RobotController::radToDeg(float rad)
{
    return rad * (180.0f / PI);
}

uint16_t RobotController::clampToMapAxis(float valueCm, uint16_t maxCm)
{
    if (valueCm < 0.0f)
        return 0;
    const float maxAxis = static_cast<float>(maxCm - 1);
    if (valueCm > maxAxis)
        return static_cast<uint16_t>(maxAxis);
    return static_cast<uint16_t>(valueCm);
}
