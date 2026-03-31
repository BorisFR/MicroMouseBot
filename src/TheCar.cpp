#include "TheCar.h"

TheCar::TheCar() {}

TheCar::~TheCar() { myTrace.println("🚗 unloaded"); }

void TheCar::setup()
{
    myTrace.println("🚗 setup");
    if (pose)
    {
        *pose = {9, 9, 0.0f};
    }
    stop();
}

void TheCar::setMotorCallbacks(std::function<void(uint8_t)> forwardCb,
                               std::function<void(uint8_t)> backwardCb,
                               std::function<void(uint8_t)> turnLeftCb,
                               std::function<void(uint8_t)> turnRightCb,
                               EVENT_CHANGE stopCb)
{
    motorForward = forwardCb;
    motorBackward = backwardCb;
    motorTurnLeft = turnLeftCb;
    motorTurnRight = turnRightCb;
    motorStop = stopCb;
}

void TheCar::setPose(BotPose *poseRef)
{
    pose = poseRef;
}

void TheCar::loop()
{
    // Fail-safe: if no command is refreshed in time, stop motors.
    if (isMoving && (millis() - lastCommandMs > commandTimeoutMs))
    {
        stop();
    }
}

void TheCar::moveForward(uint16_t distance)
{
    moveForwardSpeed(defaultCruiseSpeed);
    if (!pose)
        return;
    float thetaRad = pose->theta * PI / 180.0f;
    pose->x += distance * cos(thetaRad);
    pose->y += distance * sin(thetaRad);
}

void TheCar::moveForwardSpeed(uint8_t speed)
{
    if (motorForward)
    {
        motorForward(speed);
    }
    currentSpeed = speed;
    isMoving = true;
    lastCommandMs = millis();
}

void TheCar::moveBackwardSpeed(uint8_t speed)
{
    if (motorBackward)
    {
        motorBackward(speed);
    }
    currentSpeed = speed;
    isMoving = true;
    lastCommandMs = millis();
}

void TheCar::turnLeftSpeed(uint8_t speed)
{
    if (motorTurnLeft)
    {
        motorTurnLeft(speed);
    }
    currentSpeed = speed;
    isMoving = true;
    lastCommandMs = millis();
}

void TheCar::turnRightSpeed(uint8_t speed)
{
    if (motorTurnRight)
    {
        motorTurnRight(speed);
    }
    currentSpeed = speed;
    isMoving = true;
    lastCommandMs = millis();
}

void TheCar::turn(float angle)
{
    if (!pose)
        return;
    pose->theta += angle;
}

void TheCar::stop()
{
    if (motorStop)
    {
        motorStop();
    }
    currentSpeed = 0.0f;
    isMoving = false;
}

void TheCar::setCommandTimeoutMs(uint32_t timeoutMs)
{
    commandTimeoutMs = timeoutMs;
}

bool TheCar::isMotorLinked() const
{
    return static_cast<bool>(motorForward) && static_cast<bool>(motorBackward) &&
           static_cast<bool>(motorTurnLeft) && static_cast<bool>(motorTurnRight) &&
           static_cast<bool>(motorStop);
}

void TheCar::updatePose(uint16_t x, uint16_t y, float theta)
{
    if (!pose)
        return;
    pose->x = x;
    pose->y = y;
    pose->theta = theta;
}
